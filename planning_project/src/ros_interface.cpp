#include "ros_interface.hpp"
#include "config.hpp"
#include <ros/package.h>
#include <chrono>

namespace planning {

// ============================================================================
// Constructor
// ============================================================================

ROSInterface::ROSInterface() {
    ros::NodeHandle nh;

    // Create planner instance
    planner_ = std::make_unique<Planner>();

    // Subscribers
    obs_sub_ = nh.subscribe("/obstacles", 1, &ROSInterface::obstacleCallback, this);
    border_sub_ = nh.subscribe("/map_borders", 1, &ROSInterface::borderCallback, this);
    victims_sub_ = nh.subscribe("/victims", 1, &ROSInterface::victimsCallback, this);
    gates_sub_ = nh.subscribe("/gates", 1, &ROSInterface::gatesCallback, this);
    odom_sub_ = nh.subscribe("/limo0/odom", 1, &ROSInterface::odomCallback, this);
    other_robot_subs_.push_back(nh.subscribe("/limo1/odom", 1, &ROSInterface::otherRobotCallback1, this));
    other_robot_subs_.push_back(nh.subscribe("/limo2/odom", 1, &ROSInterface::otherRobotCallback2, this));
    timeout_sub_ = nh.subscribe("/victims_timeout", 1, &ROSInterface::timeoutCallback, this);
    clock_sub_ = nh.subscribe("/clock", 1, &ROSInterface::clockCallback, this);

    // Publisher
    ref_pub_ = nh.advertise<loco_planning::Reference>("/limo0/ref", 10);

    ROS_INFO("Planning node initialized. Waiting for map data...");
}

// ============================================================================
// Public Methods
// ============================================================================

void ROSInterface::setPlannerType(PlannerType type) {
    auto& config = getConfig();
    config.planner_type = type;
}

void ROSInterface::setDebugMode(bool debug) {
    debug_mode_ = debug;
    if (debug_mode_) {
        ROS_INFO("Debug mode enabled - trajectory execution disabled");
    }
}

void ROSInterface::run() {
    ros::spin();
}

// ============================================================================
// ROS Callbacks
// ============================================================================

void ROSInterface::obstacleCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
    obstacles_ = *msg;
    obstacles_received_ = true;
    tryBuildMap();
}

void ROSInterface::borderCallback(const geometry_msgs::Polygon::ConstPtr& msg) {
    borders_ = *msg;
    borders_received_ = true;
    tryBuildMap();
}

void ROSInterface::victimsCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
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

void ROSInterface::gatesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    if (!msg->poses.empty()) {
        // Gate
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

void ROSInterface::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
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

void ROSInterface::otherRobotCallback1(const nav_msgs::Odometry::ConstPtr& msg) {
    registerOtherRobot(msg, 1);
}

void ROSInterface::otherRobotCallback2(const nav_msgs::Odometry::ConstPtr& msg) {
    registerOtherRobot(msg, 2);
}

void ROSInterface::timeoutCallback(const std_msgs::Int32::ConstPtr& msg) {
    if (!timeout_received_) {
        timeout_seconds_ = msg->data;
        timeout_received_ = true;
        trySolveOrienteering();
    }
}

void ROSInterface::clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg) {
    if (debug_mode_ || !trajectory_ready_ || planner_->getTrajectory().empty()) {
        return;
    }

    double current_time = msg->clock.toSec();

    // Record start time on first callback after trajectory is ready
    if (movement_start_time_ < 0) {
        movement_start_time_ = current_time;
    }

    double elapsed = current_time - movement_start_time_;

    // Get trajectory point at this time
    auto [pose, vel] = planner_->getTrajectoryAt(elapsed);

    // Publish reference
    loco_planning::Reference ref_msg;
    ref_msg.x_d = pose.x;
    ref_msg.y_d = pose.y;
    ref_msg.theta_d = pose.theta;
    ref_msg.v_d = vel.v;
    ref_msg.omega_d = vel.omega;
    ref_msg.plan_finished = (elapsed >= planner_->getTotalTime());

    ref_pub_.publish(ref_msg);

    if (elapsed >= planner_->getTotalTime() && !execution_complete_) {
        execution_complete_ = true;
        ROS_INFO("Execution complete (%.2f sec)", elapsed);
        ros::shutdown();
    }
}

// ============================================================================
// Helper Methods
// ============================================================================

void ROSInterface::registerOtherRobot(const nav_msgs::Odometry::ConstPtr& msg, int robot_id) {
    if (other_robots_registered_[robot_id]) return;

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    obstacles_msgs::ObstacleMsg robot_obs;
    geometry_msgs::Point32 center;
    center.x = x;
    center.y = y;
    center.z = 0;
    robot_obs.polygon.points.push_back(center);
    robot_obs.radius = getConfig().robot_radius;  // physical size only -> inflation added later

    other_robot_obstacles_.push_back(robot_obs);
    other_robots_registered_[robot_id] = true;

    ROS_INFO("Registered limo%d at (%.2f, %.2f) as obstacle", robot_id, x, y);
    tryBuildMap();
}

bool ROSInterface::allOtherRobotsReady() {
    for (const auto& [id, registered] : other_robots_registered_) {
        if (!registered) return false;
    }
    return true;
}

void ROSInterface::detectOtherRobots() {
    if (other_robots_checked_) return;
    other_robots_checked_ = true;

    // Query ROS master for all advertised topics
    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);

    for (const auto& [id, _] : other_robots_registered_) {
        std::string topic = "/limo" + std::to_string(id) + "/odom";
        bool found = false;
        for (const auto& info : topic_list) {
            if (info.name == topic) {
                found = true;
                break;
            }
        }

        if (!found) {
            other_robots_registered_[id] = true;
        }
    }
}

// ============================================================================
// Planning State Machine
// ============================================================================

void ROSInterface::tryBuildMap() {
    if (!borders_received_ || !obstacles_received_ || map_built_) {
        return;
    }

    // First time: check which other robots exist
    detectOtherRobots();

    if (!allOtherRobotsReady()) return;

    map_built_ = true;

    // Pass map data to planner
    planner_->setMapData(borders_, obstacles_);
    planner_->addOtherRobots(other_robot_obstacles_);

    tryComputeDistanceMatrix();
}

void ROSInterface::tryComputeDistanceMatrix() {
    if (!map_built_ || !victims_received_ || !gates_received_ || !odom_received_) {
        return;
    }
    if (distance_matrix_computed_) {
        return;
    }
    distance_matrix_computed_ = true;

    trySolveOrienteering();
}

void ROSInterface::trySolveOrienteering() {
    if (!distance_matrix_computed_ || !timeout_received_) {
        return;
    }
    if (orienteering_solved_) {
        return;
    }
    orienteering_solved_ = true;

    runPlanningWithRetry();
}

void ROSInterface::runPlanningWithRetry() {
    auto& config = getConfig();
    auto t_total_start = std::chrono::high_resolution_clock::now();

    std::string pkg_path = ros::package::getPath("planning-project");

    // Set goal data (common to both approaches)
    planner_->setGoalData(robot_x_, robot_y_, robot_theta_, victims_, gate_x_, gate_y_, gate_theta_);

    for (double margin = config.max_safety_margin;
         margin >= config.min_safety_margin - 1e-9;
         margin -= config.margin_step)
    {
        config.safety_margin = std::max(margin, 0.0);
        ROS_INFO("=== Attempting with safety_margin=%.3f (inflation=%.3f) ===",
                 config.safety_margin, config.totalInflation());

        double map_time_ms = 0.0;
        double dist_time_ms = 0.0;

        if (config.planner_type == PlannerType::COMBINATORIAL) {
            // -- COMBINATORIAL: grid map + A* distances --
            auto t_map_start = std::chrono::high_resolution_clock::now();
            planner_->buildMapWithMargin(config.safety_margin);
            auto t_map_end = std::chrono::high_resolution_clock::now();
            map_time_ms = std::chrono::duration<double, std::milli>(t_map_end - t_map_start).count();

            std::string map_file = pkg_path + "/results/map.json";
            planner_->saveMapToFile(map_file);

            ROS_INFO("Grid map built (%lu nodes) in %.2f ms",
                     planner_->getCellGraph()->numNodes(), map_time_ms);

            auto t_dist_start = std::chrono::high_resolution_clock::now();
            planner_->computeDistanceMatrix();
            auto t_dist_end = std::chrono::high_resolution_clock::now();
            dist_time_ms = std::chrono::duration<double, std::milli>(t_dist_end - t_dist_start).count();

            ROS_INFO("Distance matrix (A*) computed (%lu victims) in %.2f ms",
                     victims_.size(), dist_time_ms);

            // Check if any victims are unreachable
            if (config.safety_margin > config.victim_margin) {
                if (!planner_->checkVictimsReachable(config.safety_margin)) {
                    ROS_WARN("Unreachable victims with safety_margin=%.3f, reducing...", config.safety_margin);
                    continue;
                }
            }
        } else {
            // -- SAMPLING: no grid map, Euclidean distances --
            auto t_geom_start = std::chrono::high_resolution_clock::now();
            planner_->prepareCollisionGeometry(config.safety_margin);
            auto t_geom_end = std::chrono::high_resolution_clock::now();
            map_time_ms = std::chrono::duration<double, std::milli>(t_geom_end - t_geom_start).count();

            ROS_INFO("Collision geometry prepared in %.2f ms", map_time_ms);

            auto t_dist_start = std::chrono::high_resolution_clock::now();
            planner_->computeEuclideanDistanceMatrix();
            auto t_dist_end = std::chrono::high_resolution_clock::now();
            dist_time_ms = std::chrono::duration<double, std::milli>(t_dist_end - t_dist_start).count();

            ROS_INFO("Distance matrix (Euclidean x%.1f) computed (%lu victims) in %.2f ms",
                     config.euclidean_dubins_factor, victims_.size(), dist_time_ms);
        }

        // Solve orienteering
        auto t_orient_start = std::chrono::high_resolution_clock::now();
        bool orienteering_ok = planner_->solveOrienteering(timeout_seconds_);
        auto t_orient_end = std::chrono::high_resolution_clock::now();
        double orient_time_ms = std::chrono::duration<double, std::milli>(t_orient_end - t_orient_start).count();

        if (!orienteering_ok) {
            ROS_WARN("No feasible route with safety_margin=%.3f, reducing...", config.safety_margin);
            continue;
        }
        ROS_INFO("Orienteering solved in %.2f ms", orient_time_ms);

        // Generate path
        if (config.planner_type == PlannerType::SAMPLING) {
            ROS_INFO("Using Informed RRT* for path planning...");
        } else {
            ROS_INFO("Using A* grid search for path planning...");
        }

        auto t_path_start = std::chrono::high_resolution_clock::now();
        bool path_ok = planner_->generatePath();
        auto t_path_end = std::chrono::high_resolution_clock::now();
        double path_time_ms = std::chrono::duration<double, std::milli>(t_path_end - t_path_start).count();

        if (!path_ok) {
            ROS_WARN("Path generation failed with safety_margin=%.3f, reducing...", config.safety_margin);
            continue;
        }

        ROS_INFO("Path generated in %.2f ms (%.2f m, %d victims, safety_margin=%.3f)",
                 path_time_ms, planner_->getTotalDistance(),
                 planner_->getNumVictimsVisited(), config.safety_margin);

        // Sample trajectory for execution (unless in debug mode)
        if (!debug_mode_) {
            planner_->sampleTrajectory();
            trajectory_ready_ = true;
            ROS_INFO("Trajectory sampled and ready for execution");
        }

        // Save results
        std::string traj_file = pkg_path + "/results/trajectory.json";
        if (planner_->saveTrajectoryToFile(traj_file)) {
            ROS_INFO("Trajectory saved to: %s", traj_file.c_str());
        }

        auto t_total_end = std::chrono::high_resolution_clock::now();
        double total_time_ms = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();

        const char* mode = (config.planner_type == PlannerType::SAMPLING) ? "SAMPLING" : "COMBINATORIAL";
        ROS_INFO("========================================");
        ROS_INFO("PLANNING COMPLETE [%s] - Timing:", mode);
        if (config.planner_type == PlannerType::COMBINATORIAL) {
            ROS_INFO("  Map building:      %8.2f ms", map_time_ms);
        } else {
            ROS_INFO("  Collision geom:    %8.2f ms", map_time_ms);
        }
        ROS_INFO("  Distance matrix:   %8.2f ms", dist_time_ms);
        ROS_INFO("  Orienteering:      %8.2f ms", orient_time_ms);
        ROS_INFO("  Path generation:   %8.2f ms", path_time_ms);
        ROS_INFO("  TOTAL:             %8.2f ms", total_time_ms);
        ROS_INFO("========================================");

        if (debug_mode_) {
            // Debug mode: shutdown immediately after planning
            ROS_INFO("Debug mode - shutting down");
            ros::shutdown();
        } else {
            // Normal mode: keep running to execute trajectory
            ROS_INFO("Starting trajectory execution...");
        }
        return;
    }

    ROS_ERROR("All safety margin attempts exhausted! No valid path found.");
    ros::shutdown();
}

}  // namespace planning
