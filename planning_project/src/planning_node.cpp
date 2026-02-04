#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <rosgraph_msgs/Clock.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <loco_planning/Reference.h>
#include "geometry_utils.hpp"
#include "grid_map.hpp"
#include "pathfinding.hpp"
#include "orienteering.hpp"
#include "config.hpp"
#include "math_utils.hpp"
#include <memory>
#include <iomanip>
#include <fstream>
#include <algorithm>

class PathPlanner {
public:
    // Trajectory point structure (defined early for use in functions)
    struct TrajectoryPoint {
        double x, y, theta;
        double v, omega;
        double time;  // cumulative time from start
    };

    PathPlanner() {
        ros::NodeHandle nh;

        // Subscribers
        obs_sub_ = nh.subscribe("/obstacles", 1, &PathPlanner::obstacleCallback, this);
        border_sub_ = nh.subscribe("/map_borders", 1, &PathPlanner::borderCallback, this);
        victims_sub_ = nh.subscribe("/victims", 1, &PathPlanner::victimsCallback, this);
        gates_sub_ = nh.subscribe("/gates", 1, &PathPlanner::gatesCallback, this);
        odom_sub_ = nh.subscribe("/limo0/odom", 1, &PathPlanner::odomCallback, this);
        timeout_sub_ = nh.subscribe("/victims_timeout", 1, &PathPlanner::timeoutCallback, this);
        clock_sub_ = nh.subscribe("/clock", 1, &PathPlanner::clockCallback, this);

        // Publisher
        ref_pub_ = nh.advertise<loco_planning::Reference>("/limo0/ref", 10);

        ROS_INFO("Planning node initialized. Waiting for map data...");
    }

    void obstacleCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        obstacles_ = *msg;
        obstacles_received_ = true;
        tryBuildMap();
    }

    void borderCallback(const geometry_msgs::Polygon::ConstPtr& msg) {
        borders_ = *msg;
        borders_received_ = true;
        tryBuildMap();
    }

    void victimsCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        victims_.clear();
        for (const auto& obs : msg->obstacles) {
            if (!obs.polygon.points.empty()) {
                Victim v;
                v.x = obs.polygon.points[0].x;
                v.y = obs.polygon.points[0].y;
                v.value = obs.radius;  // Value stored as radius
                victims_.push_back(v);
            }
        }
        victims_received_ = true;
        tryComputeDistanceMatrix();
    }

    void gatesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        if (!msg->poses.empty()) {
            // Use first gate for scenario B (single gate)
            gate_x_ = msg->poses[0].position.x;
            gate_y_ = msg->poses[0].position.y;
            // Extract yaw from quaternion
            double qz = msg->poses[0].orientation.z;
            double qw = msg->poses[0].orientation.w;
            gate_theta_ = 2.0 * std::atan2(qz, qw);
        }
        gates_received_ = true;
        tryComputeDistanceMatrix();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        robot_theta_ = 2.0 * std::atan2(qz, qw);

        if (!odom_received_) {
            odom_received_ = true;
            tryComputeDistanceMatrix();
        }
    }

    void timeoutCallback(const std_msgs::Int32::ConstPtr& msg) {
        if (!timeout_received_) {
            timeout_seconds_ = msg->data;
            timeout_received_ = true;
            trySolveOrienteering();
        }
    }

    void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg) {
        if (!trajectory_ready_ || trajectory_.empty()) {
            return;
        }

        double current_time = msg->clock.toSec();

        // Record start time on first callback after trajectory is ready
        if (movement_start_time_ < 0) {
            movement_start_time_ = current_time;
        }

        double elapsed = current_time - movement_start_time_;

        // Get trajectory point at this time
        auto [pose, vel] = getTrajectoryAt(elapsed);

        // Publish reference
        loco_planning::Reference ref_msg;
        ref_msg.x_d = pose.x;
        ref_msg.y_d = pose.y;
        ref_msg.theta_d = pose.theta;
        ref_msg.v_d = vel.v;
        ref_msg.omega_d = vel.omega;
        ref_msg.plan_finished = (elapsed >= trajectory_.back().time);

        ref_pub_.publish(ref_msg);

        if (elapsed >= trajectory_.back().time && !execution_complete_) {
            execution_complete_ = true;
            ROS_INFO("Execution complete (%.2f sec)", elapsed);
        }
    }

    // Returns (pose, velocity) at given time by interpolating trajectory
    std::pair<TrajectoryPoint, TrajectoryPoint> getTrajectoryAt(double t) {
        if (trajectory_.empty()) {
            return {{0,0,0,0,0,0}, {0,0,0,0,0,0}};
        }

        // Clamp to trajectory bounds
        if (t <= 0) {
            return {trajectory_.front(), trajectory_.front()};
        }
        if (t >= trajectory_.back().time) {
            // At end, return final pose with zero velocity
            TrajectoryPoint final_pose = trajectory_.back();
            TrajectoryPoint final_vel = final_pose;
            final_vel.v = 0;
            final_vel.omega = 0;
            return {final_pose, final_vel};
        }

        // Binary search for the right segment
        size_t lo = 0, hi = trajectory_.size() - 1;
        while (lo < hi - 1) {
            size_t mid = (lo + hi) / 2;
            if (trajectory_[mid].time <= t) {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        // Interpolate between trajectory_[lo] and trajectory_[hi]
        const auto& p0 = trajectory_[lo];
        const auto& p1 = trajectory_[hi];
        double dt = p1.time - p0.time;
        double alpha = (dt > 1e-9) ? (t - p0.time) / dt : 0.0;

        TrajectoryPoint pose;
        pose.x = p0.x + alpha * (p1.x - p0.x);
        pose.y = p0.y + alpha * (p1.y - p0.y);
        // Interpolate angle carefully
        double dtheta = p1.theta - p0.theta;
        while (dtheta > M_PI) dtheta -= 2*M_PI;
        while (dtheta < -M_PI) dtheta += 2*M_PI;
        pose.theta = p0.theta + alpha * dtheta;
        pose.v = p0.v + alpha * (p1.v - p0.v);
        pose.omega = p0.omega + alpha * (p1.omega - p0.omega);
        pose.time = t;

        return {pose, pose};  // velocity is same as pose (v, omega fields)
    }

    void tryBuildMap() {
        if (!borders_received_ || !obstacles_received_ || map_built_) {
            return;
        }
        map_built_ = true;

        // Create grid from borders
        grid_map_ = std::make_unique<planning::GridMap>(borders_);

        // Inflate obstacles and mark them
        inflated_obstacles_ = planning::inflateObstacles(obstacles_);
        grid_map_->markObstacles(inflated_obstacles_);

        // Shrink borders and mark outside area
        auto shrunk_borders = planning::shrinkBorders(borders_);
        grid_map_->markOutsideBorders(shrunk_borders);

        // Refine MIXED cells by recursive subdivision
        const int refinement_depth = planning::getConfig().refinement_depth;
        grid_map_->refineMixedCells(inflated_obstacles_, shrunk_borders, refinement_depth);

        // Save map to file for visualization
        std::string pkg_path = ros::package::getPath("planning-project");
        std::string map_file = pkg_path + "/results/map.json";
        if (grid_map_->saveToFile(map_file)) {
            ROS_INFO("Grid map saved for visualization");
        }

        // Build pathfinding graph
        cell_graph_ = std::make_unique<planning::CellGraph>();
        cell_graph_->buildFromGridMap(*grid_map_);

        ROS_INFO("Grid map built (%d nodes)", (int)cell_graph_->numNodes());

        tryComputeDistanceMatrix();
    }

    void tryComputeDistanceMatrix() {
        if (!map_built_ || !victims_received_ || !gates_received_ || !odom_received_) {
            return;
        }
        if (distance_matrix_computed_) {
            return;
        }
        distance_matrix_computed_ = true;

        // Build points list: [start, victim0, victim1, ..., gate]
        std::vector<std::pair<double, double>> points;
        std::vector<double> headings;

        // Start: robot position and orientation
        points.push_back({robot_x_, robot_y_});
        headings.push_back(robot_theta_);

        // Victims: positions, headings unknown (NaN)
        for (const auto& v : victims_) {
            points.push_back({v.x, v.y});
            headings.push_back(std::nan(""));
        }

        // Gate: position and required orientation
        points.push_back({gate_x_, gate_y_});
        headings.push_back(gate_theta_);

        // Compute distance matrix using A* × heuristic factor
        distance_matrix_ = planning::computeDubinsDistanceMatrix(
            *cell_graph_, points, headings, inflated_obstacles_);

        ROS_INFO("Distance matrix computed (%lu victims)", victims_.size());
        trySolveOrienteering();
    }

    void trySolveOrienteering() {
        if (!distance_matrix_computed_ || !timeout_received_) {
            return;
        }
        if (orienteering_solved_) {
            return;
        }
        orienteering_solved_ = true;

        // Convert timeout to max distance (distance = velocity × time)
        double velocity = planning::getConfig().robot_velocity;
        double time_buffer = planning::getConfig().time_buffer_ratio;
        double max_distance;

        if (timeout_seconds_ <= 0) {
            max_distance = 1e9;
        } else {
            double effective_time = timeout_seconds_ * (1.0 - time_buffer);
            max_distance = effective_time * velocity;
        }

        // Extract victim values
        std::vector<double> victim_values;
        for (const auto& v : victims_) {
            victim_values.push_back(v.value);
        }

        // Solve orienteering problem
        auto result = planning::solveOrienteering(distance_matrix_, victim_values, max_distance);

        if (result.feasible) {
            chosen_route_ = result.route;
            ROS_INFO("Route: %lu victims, value=%.0f, dist=%.1fm (budget=%.1fm)",
                     chosen_route_.size(), result.total_value, result.total_distance, max_distance);

            // Generate Dubins path
            generatePath();
        } else {
            ROS_ERROR("No feasible route found! Cannot reach gate.");
        }
    }

    void generatePath() {
        double kmax = 1.0 / planning::getConfig().dubins_rho;

        // Build route: [start, victim0, victim1, ..., gate]
        std::vector<std::pair<double, double>> route;
        route.push_back({robot_x_, robot_y_});
        for (int v : chosen_route_) {
            route.push_back({victims_[v].x, victims_[v].y});
        }
        route.push_back({gate_x_, gate_y_});

        // Build safe waypoints (adds A* waypoints where direct Dubins collides)
        auto waypoints = planning::buildSafeWaypoints(
            *cell_graph_, route, robot_theta_, gate_theta_, kmax, inflated_obstacles_);

        // Generate multi-point Dubins path
        Pose start_pose = {robot_x_, robot_y_, robot_theta_};
        Pose end_pose = {gate_x_, gate_y_, gate_theta_};

        dubins_path_ = planning::generateDubinsPath(
            start_pose, end_pose, waypoints, kmax, 8, 2);

        // Sample trajectory and store for execution
        sampleTrajectory();

        // Execute the path
        executePath();
    }

    void sampleTrajectory() {
        trajectory_.clear();

        double velocity = planning::getConfig().robot_velocity;
        double dt = 0.01;  // 100 Hz sampling
        double ds = velocity * dt;  // distance per sample
        double cumulative_time = 0.0;

        for (const auto& curve : dubins_path_.curves) {
            for (int arc_idx = 0; arc_idx < 3; ++arc_idx) {
                const DubinsArc& arc = curve.arcs[arc_idx];
                if (arc.length < 1e-6) continue;

                int num_samples = std::max(1, static_cast<int>(std::ceil(arc.length / ds)));
                double step = arc.length / num_samples;
                double time_step = step / velocity;  // time = distance / velocity

                double x = arc.start.x;
                double y = arc.start.y;
                double theta = arc.start.theta;

                for (int i = 0; i <= num_samples; ++i) {
                    TrajectoryPoint pt;
                    pt.x = x;
                    pt.y = y;
                    pt.theta = theta;
                    pt.v = velocity;
                    pt.omega = velocity * arc.k;  // omega = v * curvature
                    pt.time = cumulative_time;
                    trajectory_.push_back(pt);

                    if (i < num_samples) {
                        cumulative_time += time_step;
                        // Integrate forward
                        if (std::abs(arc.k) < 1e-6) {
                            // Straight line
                            x += step * std::cos(theta);
                            y += step * std::sin(theta);
                        } else {
                            // Arc
                            double dtheta = arc.k * step;
                            x += (std::sin(theta + dtheta) - std::sin(theta)) / arc.k;
                            y += (-std::cos(theta + dtheta) + std::cos(theta)) / arc.k;
                            theta += dtheta;
                        }
                    }
                }
            }
        }

    }

    void executePath() {
        if (trajectory_.empty()) {
            ROS_ERROR("No trajectory to execute!");
            return;
        }

        ROS_INFO("Executing trajectory: %.2f sec, %.2f m",
                 trajectory_.back().time, dubins_path_.cost);

        // Save trajectory to file for visualization
        saveTrajectoryToFile();

        // Set flag - clockCallback will handle the actual publishing
        trajectory_ready_ = true;
    }

    void saveTrajectoryToFile() {
        std::string pkg_path = ros::package::getPath("planning-project");
        std::string traj_file = pkg_path + "/results/trajectory.json";

        std::ofstream file(traj_file);
        if (!file.is_open()) {
            ROS_WARN("Failed to open trajectory file for writing: %s", traj_file.c_str());
            return;
        }

        file << std::fixed << std::setprecision(6);
        file << "{\n";

        // Save start and goal poses
        file << "  \"start\": {\"x\": " << robot_x_ << ", \"y\": " << robot_y_
             << ", \"theta\": " << robot_theta_ << "},\n";
        file << "  \"goal\": {\"x\": " << gate_x_ << ", \"y\": " << gate_y_
             << ", \"theta\": " << gate_theta_ << "},\n";

        // Save victims (including which ones were visited)
        file << "  \"victims\": [\n";
        for (size_t i = 0; i < victims_.size(); ++i) {
            bool visited = (std::find(chosen_route_.begin(), chosen_route_.end(), i)
                          != chosen_route_.end());
            file << "    {\"x\": " << victims_[i].x << ", \"y\": " << victims_[i].y
                 << ", \"value\": " << victims_[i].value
                 << ", \"visited\": " << (visited ? "true" : "false") << "}";
            if (i < victims_.size() - 1) file << ",";
            file << "\n";
        }
        file << "  ],\n";

        // Save chosen route (victim indices)
        file << "  \"route\": [";
        for (size_t i = 0; i < chosen_route_.size(); ++i) {
            file << chosen_route_[i];
            if (i < chosen_route_.size() - 1) file << ", ";
        }
        file << "],\n";

        // Save Dubins path segments
        file << "  \"dubins_curves\": [\n";
        for (size_t c = 0; c < dubins_path_.curves.size(); ++c) {
            const auto& curve = dubins_path_.curves[c];
            file << "    {\n";
            file << "      \"arcs\": [\n";
            for (int a = 0; a < 3; ++a) {
                const DubinsArc& arc = curve.arcs[a];
                file << "        {\n";
                file << "          \"start\": {\"x\": " << arc.start.x << ", \"y\": " << arc.start.y
                     << ", \"theta\": " << arc.start.theta << "},\n";
                file << "          \"end\": {\"x\": " << arc.end.x << ", \"y\": " << arc.end.y
                     << ", \"theta\": " << arc.end.theta << "},\n";
                file << "          \"k\": " << arc.k << ",\n";
                file << "          \"length\": " << arc.length << "\n";
                file << "        }";
                if (a < 2) file << ",";
                file << "\n";
            }
            file << "      ]\n";
            file << "    }";
            if (c < dubins_path_.curves.size() - 1) file << ",";
            file << "\n";
        }
        file << "  ],\n";

        // Save sampled trajectory points (subsample for file size)
        int subsample = std::max(1, static_cast<int>(trajectory_.size() / 1000));
        file << "  \"trajectory\": [\n";
        for (size_t i = 0; i < trajectory_.size(); i += subsample) {
            const auto& pt = trajectory_[i];
            file << "    {\"x\": " << pt.x << ", \"y\": " << pt.y
                 << ", \"theta\": " << pt.theta << ", \"time\": " << pt.time << "}";
            if (i + subsample < trajectory_.size()) file << ",";
            file << "\n";
        }
        // Always include the last point
        if (subsample > 1 && !trajectory_.empty()) {
            const auto& pt = trajectory_.back();
            file << "    {\"x\": " << pt.x << ", \"y\": " << pt.y
                 << ", \"theta\": " << pt.theta << ", \"time\": " << pt.time << "}\n";
        }
        file << "  ],\n";

        // Save statistics
        file << "  \"stats\": {\n";
        file << "    \"total_distance\": " << dubins_path_.cost << ",\n";
        file << "    \"total_time\": " << (trajectory_.empty() ? 0 : trajectory_.back().time) << ",\n";
        file << "    \"num_trajectory_points\": " << trajectory_.size() << ",\n";
        file << "    \"num_victims_visited\": " << chosen_route_.size() << "\n";
        file << "  }\n";

        file << "}\n";
        file.close();

        ROS_INFO("Trajectory saved to: %s", traj_file.c_str());
    }

    void run() {
        ros::spin();
    }

private:
    // Subscribers
    ros::Subscriber obs_sub_;
    ros::Subscriber border_sub_;
    ros::Subscriber victims_sub_;
    ros::Subscriber gates_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber timeout_sub_;

    // Publisher
    ros::Publisher ref_pub_;

    // Map data
    geometry_msgs::Polygon borders_;
    obstacles_msgs::ObstacleArrayMsg obstacles_;

    // Victim data
    struct Victim {
        double x, y;
        double value;
    };
    std::vector<Victim> victims_;

    // Gate data
    double gate_x_ = 0, gate_y_ = 0, gate_theta_ = 0;

    // Robot pose
    double robot_x_ = 0, robot_y_ = 0, robot_theta_ = 0;

    // Timeout
    int timeout_seconds_ = 0;

    // Reception flags
    bool borders_received_ = false;
    bool obstacles_received_ = false;
    bool victims_received_ = false;
    bool gates_received_ = false;
    bool odom_received_ = false;
    bool timeout_received_ = false;
    bool map_built_ = false;
    bool distance_matrix_computed_ = false;
    bool orienteering_solved_ = false;

    // Planning data structures
    std::unique_ptr<planning::GridMap> grid_map_;
    std::unique_ptr<planning::CellGraph> cell_graph_;
    std::vector<geometry_msgs::Polygon> inflated_obstacles_;
    std::vector<std::vector<double>> distance_matrix_;

    // Chosen route (victim indices in order)
    std::vector<int> chosen_route_;

    // Generated Dubins path
    PathInfo dubins_path_;

    // Trajectory for execution
    std::vector<TrajectoryPoint> trajectory_;

    // Clock-based execution
    ros::Subscriber clock_sub_;
    double movement_start_time_ = -1.0;
    bool trajectory_ready_ = false;
    bool execution_complete_ = false;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "planning_node");
    PathPlanner planner;
    planner.run();
    return 0;
}
