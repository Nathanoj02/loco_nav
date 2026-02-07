#include "ros_interface.hpp"
#include "config.hpp"
#include <ros/package.h>

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
    // TODO: Re-enable for actual simulation
    return;

    if (!trajectory_ready_ || planner_->getTrajectory().empty()) {
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
    }
}

// ============================================================================
// Helper Methods
// ============================================================================

void ROSInterface::registerOtherRobot(const nav_msgs::Odometry::ConstPtr& msg, int robot_id) {
    if (other_robots_registered_[robot_id]) return;

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // Store separately (obstacles_ gets overwritten by obstacleCallback)
    obstacles_msgs::ObstacleMsg robot_obs;
    geometry_msgs::Point32 center;
    center.x = x;
    center.y = y;
    center.z = 0;
    robot_obs.polygon.points.push_back(center);
    robot_obs.radius = getConfig().robot_radius;  // physical size only; inflation added later

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

    // Don't build yet - the retry loop in runPlanningWithRetry() will
    // build with the appropriate safety margin
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

    // Don't build/compute yet - the retry loop handles everything
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

    for (double margin = config.max_safety_margin;
         margin >= config.min_safety_margin - 1e-9;
         margin -= config.margin_step)
    {
        config.safety_margin = std::max(margin, 0.0);
        ROS_INFO("=== Attempting with safety_margin=%.3f (inflation=%.3f) ===",
                 config.safety_margin, config.totalInflation());

        // Build map with new margin
        planner_->buildMapWithMargin(config.safety_margin);

        std::string pkg_path = ros::package::getPath("planning-project");
        std::string map_file = pkg_path + "/results/map.json";
        if (planner_->saveMapToFile(map_file)) {
            ROS_INFO("Grid map saved for visualization");
        }

        ROS_INFO("Grid map built (%lu nodes)", planner_->getCellGraph()->numNodes());

        // Set goal data and compute distance matrix
        planner_->setGoalData(robot_x_, robot_y_, robot_theta_, victims_, gate_x_, gate_y_, gate_theta_);
        planner_->computeDistanceMatrix();

        ROS_INFO("Distance matrix computed (%lu victims)", victims_.size());

        // Check if any victims are unreachable
        if (config.safety_margin > config.victim_margin) {
            if (!planner_->checkVictimsReachable(config.safety_margin)) {
                ROS_WARN("Unreachable victims with safety_margin=%.3f, reducing...", config.safety_margin);
                continue;
            }
        }

        // Solve orienteering
        if (!planner_->solveOrienteering(timeout_seconds_)) {
            ROS_WARN("No feasible route with safety_margin=%.3f, reducing...", config.safety_margin);
            continue;
        }

        double velocity = config.robot_velocity;
        double time_buffer = config.time_buffer_ratio;
        double max_distance;
        if (timeout_seconds_ <= 0) {
            max_distance = 1e9;
        } else {
            double effective_time = timeout_seconds_ * (1.0 - time_buffer);
            max_distance = effective_time * velocity;
        }

        ROS_INFO("Route: %d victims, dist=%.1fm (budget=%.1fm)",
                 planner_->getNumVictimsVisited(),
                 planner_->getTotalDistance(),
                 max_distance);

        // Try to generate path
        if (config.planner_type == PlannerType::SAMPLING) {
            ROS_INFO("Using Informed RRT* for path planning...");
        } else {
            ROS_INFO("Using A* grid search for path planning...");
        }

        if (!planner_->generatePath()) {
            ROS_WARN("Path generation failed with safety_margin=%.3f, reducing...", config.safety_margin);
            continue;
        }

        ROS_INFO("Planning succeeded with safety_margin=%.3f", config.safety_margin);

        // TODO: Re-enable for actual simulation
        // planner_->sampleTrajectory();
        // trajectory_ready_ = true;

        // For now, just save trajectory data for offline analysis
        std::string traj_file = pkg_path + "/results/trajectory.json";
        if (planner_->saveTrajectoryToFile(traj_file)) {
            ROS_INFO("Trajectory saved to: %s", traj_file.c_str());
        }

        ROS_INFO("Planning complete. Trajectory saved. Shutting down.");
        ros::shutdown();
        return;
    }

    ROS_ERROR("All safety margin attempts exhausted! No valid path found.");
    ros::shutdown();
}

}  // namespace planning
