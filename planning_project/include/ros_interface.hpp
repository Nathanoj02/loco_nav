#ifndef PLANNING_PROJECT_ROS_INTERFACE_HPP
#define PLANNING_PROJECT_ROS_INTERFACE_HPP

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <rosgraph_msgs/Clock.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <loco_planning/Reference.h>
#include "planner.hpp"
#include <memory>
#include <map>

namespace planning {

/**
 * @brief ROS communication interface for the planner.
 *
 * Handles all ROS subscriptions, publications, and callbacks.
 * Orchestrates the planning pipeline based on received data.
 */
class ROSInterface {
public:
    ROSInterface();

    /**
     * @brief Set the planner type (combinatorial or sampling).
     */
    void setPlannerType(PlannerType type);

    /**
     * @brief Start ROS event loop.
     */
    void run();

private:
    // ========================================================================
    // ROS Callbacks
    // ========================================================================

    void obstacleCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg);
    void borderCallback(const geometry_msgs::Polygon::ConstPtr& msg);
    void victimsCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg);
    void gatesCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void otherRobotCallback1(const nav_msgs::Odometry::ConstPtr& msg);
    void otherRobotCallback2(const nav_msgs::Odometry::ConstPtr& msg);
    void timeoutCallback(const std_msgs::Int32::ConstPtr& msg);
    void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg);

    // ========================================================================
    // Helper Methods
    // ========================================================================

    void registerOtherRobot(const nav_msgs::Odometry::ConstPtr& msg, int robot_id);
    bool allOtherRobotsReady();
    void detectOtherRobots();

    // ========================================================================
    // Planning State Machine
    // ========================================================================

    void tryBuildMap();
    void tryComputeDistanceMatrix();
    void trySolveOrienteering();
    void runPlanningWithRetry();

    // ========================================================================
    // ROS Communication
    // ========================================================================

    ros::Publisher ref_pub_;
    ros::Subscriber obs_sub_;
    ros::Subscriber border_sub_;
    ros::Subscriber victims_sub_;
    ros::Subscriber gates_sub_;
    ros::Subscriber odom_sub_;
    std::vector<ros::Subscriber> other_robot_subs_;
    ros::Subscriber timeout_sub_;
    ros::Subscriber clock_sub_;

    // ========================================================================
    // State Flags
    // ========================================================================

    bool borders_received_ = false;
    bool obstacles_received_ = false;
    bool victims_received_ = false;
    bool gates_received_ = false;
    bool odom_received_ = false;
    bool timeout_received_ = false;
    bool map_built_ = false;
    bool distance_matrix_computed_ = false;
    bool orienteering_solved_ = false;

    // ========================================================================
    // Other Robots
    // ========================================================================

    std::map<int, bool> other_robots_registered_ = {{1, false}, {2, false}};
    std::vector<obstacles_msgs::ObstacleMsg> other_robot_obstacles_;
    bool other_robots_checked_ = false;

    // ========================================================================
    // Data Storage (ROS message types)
    // ========================================================================

    geometry_msgs::Polygon borders_;
    obstacles_msgs::ObstacleArrayMsg obstacles_;
    std::vector<Victim> victims_;
    double gate_x_ = 0, gate_y_ = 0, gate_theta_ = 0;
    double robot_x_ = 0, robot_y_ = 0, robot_theta_ = 0;
    int timeout_seconds_ = 0;

    // ========================================================================
    // Execution
    // ========================================================================

    double movement_start_time_ = -1.0;
    bool trajectory_ready_ = false;
    bool execution_complete_ = false;

    // ========================================================================
    // Planner Instance
    // ========================================================================

    std::unique_ptr<Planner> planner_;
};

}  // namespace planning

#endif  // PLANNING_PROJECT_ROS_INTERFACE_HPP
