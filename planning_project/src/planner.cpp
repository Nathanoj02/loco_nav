#include "planner.hpp"
#include "geometry_utils.hpp"
#include "rrt_star.hpp"
#include "orienteering.hpp"
#include "config.hpp"
#include <ros/package.h>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <cmath>

namespace planning {

// ============================================================================
// Constructor
// ============================================================================

Planner::Planner() {
    // Initialize members with default values
}

// ============================================================================
// Map Building
// ============================================================================

void Planner::setMapData(
    const geometry_msgs::Polygon& borders,
    const obstacles_msgs::ObstacleArrayMsg& obstacles) {

    borders_ = borders;
    obstacles_ = obstacles;
}

void Planner::addOtherRobots(const std::vector<obstacles_msgs::ObstacleMsg>& robots) {
    for (const auto& robot_obs : robots) {
        obstacles_.obstacles.push_back(robot_obs);
    }
}

void Planner::buildMapWithMargin(double safety_margin) {
    // Create grid from borders
    grid_map_ = std::make_unique<GridMap>(borders_);

    // Inflate obstacles and mark them
    inflated_obstacles_ = inflateObstacles(obstacles_);
    grid_map_->markObstacles(inflated_obstacles_);

    // Shrink borders and mark outside area
    shrunk_borders_ = shrinkBorders(borders_);
    grid_map_->markOutsideBorders(shrunk_borders_);

    // Refine MIXED cells by recursive subdivision
    const int refinement_depth = getConfig().refinement_depth;
    grid_map_->refineMixedCells(inflated_obstacles_, shrunk_borders_, refinement_depth);

    // Build pathfinding graph
    cell_graph_ = std::make_unique<CellGraph>();
    cell_graph_->buildFromGridMap(*grid_map_);
}

void Planner::prepareCollisionGeometry(double safety_margin) {
    // Inflate obstacles and shrink borders for collision checking only.
    // No grid map or cell graph is built.
    inflated_obstacles_ = inflateObstacles(obstacles_);
    shrunk_borders_ = shrinkBorders(borders_);
}

// ============================================================================
// Distance Matrix
// ============================================================================

void Planner::setGoalData(
    double robot_x, double robot_y, double robot_theta,
    const std::vector<Victim>& victims,
    double gate_x, double gate_y, double gate_theta) {

    robot_x_ = robot_x;
    robot_y_ = robot_y;
    robot_theta_ = robot_theta;
    victims_ = victims;
    gate_x_ = gate_x;
    gate_y_ = gate_y;
    gate_theta_ = gate_theta;
}

void Planner::computeDistanceMatrix() {
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
    distance_matrix_ = computeDubinsDistanceMatrix(
        *cell_graph_, points, headings, inflated_obstacles_);
}

void Planner::computeEuclideanDistanceMatrix() {
    // Build points list: [start, victim0, victim1, ..., gate]
    std::vector<std::pair<double, double>> points;
    points.push_back({robot_x_, robot_y_});
    for (const auto& v : victims_) {
        points.push_back({v.x, v.y});
    }
    points.push_back({gate_x_, gate_y_});

    int n = points.size();
    distance_matrix_.assign(n, std::vector<double>(n, 0.0));

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i != j) {
                double dx = points[j].first - points[i].first;
                double dy = points[j].second - points[i].second;
                distance_matrix_[i][j] = std::sqrt(dx*dx + dy*dy) * getConfig().euclidean_dubins_factor;
            }
        }
    }
}

bool Planner::checkVictimsReachable(double safety_margin) const {
    if (distance_matrix_.empty() || victims_.empty()) {
        return true;  // Nothing to check
    }

    size_t gate_idx = distance_matrix_.size() - 1;
    bool all_reachable = true;

    for (size_t i = 1; i <= victims_.size(); ++i) {
        bool unreachable = false;

        // Check if victim is reachable from start
        if (distance_matrix_[0][i] < 0) {
            unreachable = true;
        }

        // Check if gate is reachable from victim
        if (distance_matrix_[i][gate_idx] < 0) {
            unreachable = true;
        }

        if (unreachable) {
            all_reachable = false;
        }
    }

    return all_reachable;
}

// ============================================================================
// Path Planning
// ============================================================================

bool Planner::solveOrienteering(int timeout_seconds) {
    const auto& config = getConfig();
    double velocity = config.robot_velocity;
    double time_buffer = config.time_buffer_ratio;
    double max_distance;

    if (timeout_seconds <= 0) {
        max_distance = 1e9;
    } else {
        double effective_time = timeout_seconds * (1.0 - time_buffer);
        max_distance = effective_time * velocity;
    }

    std::vector<double> victim_values;
    for (const auto& v : victims_) {
        victim_values.push_back(v.value);
    }

    auto result = planning::solveOrienteering(distance_matrix_, victim_values, max_distance);

    if (!result.feasible) {
        return false;
    }

    chosen_route_ = result.route;
    return true;
}

bool Planner::generatePath() {
    const auto& config = getConfig();
    double kmax = 1.0 / config.dubins_rho;

    // Build route: [start, victim0, victim1, ..., gate]
    std::vector<std::pair<double, double>> route;
    route.push_back({robot_x_, robot_y_});
    for (int v : chosen_route_) {
        route.push_back({victims_[v].x, victims_[v].y});
    }
    route.push_back({gate_x_, gate_y_});

    // Build safe waypoints based on selected planner type
    std::vector<Point> waypoints;

    if (config.planner_type == PlannerType::SAMPLING) {
        // Use Informed RRT* (sampling-based approach)
        std::array<double, 4> bounds = computeWorldBounds();
        std::string pkg_path = ros::package::getPath("planning-project");
        std::string rrt_file = pkg_path + "/results/rrt_tree.json";
        waypoints = buildSafeWaypointsRRTWithSave(
            route, robot_theta_, gate_theta_, kmax, bounds, rrt_file);
    } else {
        // Use A* on grid (combinatorial approach - default)
        waypoints = buildSafeWaypoints(
            *cell_graph_, route, robot_theta_, gate_theta_, kmax, inflated_obstacles_, &shrunk_borders_);
    }

    // Generate multi-point Dubins path
    Pose start_pose = {robot_x_, robot_y_, robot_theta_};
    Pose end_pose = {gate_x_, gate_y_, gate_theta_};

    dubins_path_ = generateDubinsPath(
        start_pose, end_pose, waypoints, kmax, inflated_obstacles_, &shrunk_borders_, 8, 2);

    if (dubins_path_.curves.empty() || dubins_path_.cost >= 1e30) {
        return false;
    }

    return true;
}

// ============================================================================
// Trajectory
// ============================================================================

void Planner::sampleTrajectory() {
    trajectory_.clear();

    double velocity = getConfig().robot_velocity;
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

std::pair<TrajectoryPoint, TrajectoryPoint> Planner::getTrajectoryAt(double t) const {
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

// ============================================================================
// File I/O
// ============================================================================

bool Planner::saveMapToFile(const std::string& filepath) const {
    if (!grid_map_) {
        return false;
    }
    return grid_map_->saveToFile(filepath);
}

bool Planner::saveTrajectoryToFile(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        return false;
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

    // Save statistics
    file << "  \"stats\": {\n";
    file << "    \"total_distance\": " << dubins_path_.cost << ",\n";
    file << "    \"total_time\": " << (trajectory_.empty() ? 0 : trajectory_.back().time) << ",\n";
    file << "    \"num_trajectory_points\": " << trajectory_.size() << ",\n";
    file << "    \"num_victims_visited\": " << chosen_route_.size() << "\n";
    file << "  }\n";

    file << "}\n";
    file.close();

    return true;
}

std::vector<Point> Planner::buildSafeWaypointsRRTWithSave(
    const std::vector<std::pair<double, double>>& route,
    double start_theta,
    double end_theta,
    double kmax,
    const std::array<double, 4>& bounds,
    const std::string& rrt_filepath) {

    std::vector<Point> waypoints;
    if (route.size() < 2) return waypoints;

    const auto& config = getConfig();

    // Open file for combined RRT data
    std::ofstream file(rrt_filepath);
    if (!file.is_open()) {
        return buildSafeWaypointsRRT(route, start_theta, end_theta, kmax, inflated_obstacles_, bounds, &shrunk_borders_);
    }

    file << std::fixed << std::setprecision(6);
    file << "{\n";

    // Save bounds
    file << "  \"bounds\": {\"min_x\": " << bounds[0] << ", \"max_x\": " << bounds[1]
         << ", \"min_y\": " << bounds[2] << ", \"max_y\": " << bounds[3] << "},\n";

    // Save route points
    file << "  \"route\": [\n";
    for (size_t i = 0; i < route.size(); ++i) {
        file << "    [" << route[i].first << ", " << route[i].second << "]";
        if (i < route.size() - 1) file << ",";
        file << "\n";
    }
    file << "  ],\n";

    // Save obstacles
    file << "  \"obstacles\": [\n";
    for (size_t i = 0; i < inflated_obstacles_.size(); ++i) {
        file << "    [";
        for (size_t j = 0; j < inflated_obstacles_[i].points.size(); ++j) {
            file << "[" << inflated_obstacles_[i].points[j].x << ", " << inflated_obstacles_[i].points[j].y << "]";
            if (j < inflated_obstacles_[i].points.size() - 1) file << ", ";
        }
        file << "]";
        if (i < inflated_obstacles_.size() - 1) file << ",";
        file << "\n";
    }
    file << "  ],\n";

    // Process each segment and collect all trees
    file << "  \"segments\": [\n";

    for (size_t i = 0; i < route.size() - 1; ++i) {
        double x1 = route[i].first, y1 = route[i].second;
        double x2 = route[i + 1].first, y2 = route[i + 1].second;

        double theta1 = (i == 0) ? start_theta : std::atan2(y1 - route[i-1].second, x1 - route[i-1].first);
        double theta2 = (i == route.size() - 2) ? end_theta : std::atan2(y2 - y1, x2 - x1);

        Pose pose1 = {x1, y1, theta1};
        Pose pose2 = {x2, y2, theta2};

        bool collides = directDubinsCollides(pose1, pose2, kmax, inflated_obstacles_, &shrunk_borders_);

        file << "    {\n";
        file << "      \"start\": [" << x1 << ", " << y1 << "],\n";
        file << "      \"goal\": [" << x2 << ", " << y2 << "],\n";
        file << "      \"direct\": " << (collides ? "false" : "true") << ",\n";

        if (collides) {
            InformedRRTStar rrt(inflated_obstacles_, bounds);
            rrt.setStepSize(config.rrt_step_size);

            auto result = rrt.plan(x1, y1, x2, y2, config.rrt_max_iterations, config.rrt_goal_radius);

            // Save tree edges
            file << "      \"edges\": [\n";
            const auto& nodes = rrt.getNodes();
            bool first_edge = true;
            for (const auto& node : nodes) {
                if (node->parent) {
                    if (!first_edge) file << ",\n";
                    first_edge = false;
                    file << "        [[" << node->parent->x << ", " << node->parent->y
                         << "], [" << node->x << ", " << node->y << "]]";
                }
            }
            file << "\n      ],\n";

            // Save path
            file << "      \"path\": [";
            for (size_t j = 0; j < result.path.size(); ++j) {
                file << "[" << result.path[j].first << ", " << result.path[j].second << "]";
                if (j < result.path.size() - 1) file << ", ";
            }
            file << "],\n";

            if (result.found && result.path.size() > 2) {
                auto smoothed = smoothRRTPath(result.path, inflated_obstacles_, 50, 30);

                file << "      \"smoothed\": [";
                for (size_t j = 0; j < smoothed.size(); ++j) {
                    file << "[" << smoothed[j].first << ", " << smoothed[j].second << "]";
                    if (j < smoothed.size() - 1) file << ", ";
                }
                file << "],\n";

                for (size_t j = 1; j < smoothed.size() - 1; ++j) {
                    waypoints.push_back({static_cast<float>(smoothed[j].first),
                                        static_cast<float>(smoothed[j].second)});
                }
            } else {
                file << "      \"smoothed\": [],\n";
            }

            file << "      \"num_nodes\": " << nodes.size() << ",\n";
            file << "      \"cost\": " << result.cost << "\n";
        } else {
            file << "      \"edges\": [],\n";
            file << "      \"path\": [[" << x1 << ", " << y1 << "], [" << x2 << ", " << y2 << "]],\n";
            file << "      \"smoothed\": [],\n";
            file << "      \"num_nodes\": 0,\n";
            file << "      \"cost\": " << std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1)) << "\n";
        }

        file << "    }";
        if (i < route.size() - 2) file << ",";
        file << "\n";

        if (i < route.size() - 2) {
            waypoints.push_back({static_cast<float>(x2), static_cast<float>(y2)});
        }
    }

    file << "  ]\n";
    file << "}\n";
    file.close();

    return waypoints;
}

// ============================================================================
// Private Methods
// ============================================================================

std::array<double, 4> Planner::computeWorldBounds() const {
    // Extract min/max from borders polygon
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& pt : borders_.points) {
        min_x = std::min(min_x, static_cast<double>(pt.x));
        max_x = std::max(max_x, static_cast<double>(pt.x));
        min_y = std::min(min_y, static_cast<double>(pt.y));
        max_y = std::max(max_y, static_cast<double>(pt.y));
    }

    // Add small margin
    double margin = getConfig().totalInflation();
    return {min_x + margin, max_x - margin, min_y + margin, max_y - margin};
}

}  // namespace planning
