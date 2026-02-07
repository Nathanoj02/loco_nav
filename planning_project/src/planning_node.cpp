#include <ros/ros.h>
#include "ros_interface.hpp"
#include "config.hpp"
#include <string>

/**
 * @brief Main entry point for the planning node.
 *
 * Usage:
 *   rosrun planning_project planning_node --mode combinatorial
 *   rosrun planning_project planning_node --mode sampling
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "planning_node");

    // Parse --mode flag
    planning::PlannerType planner_type = planning::PlannerType::COMBINATORIAL;  // default

    for (int i = 1; i < argc - 1; ++i) {
        std::string arg(argv[i]);
        if (arg == "--mode") {
            std::string mode(argv[i + 1]);
            if (mode == "sampling") {
                planner_type = planning::PlannerType::SAMPLING;
                ROS_INFO("Using planner mode: SAMPLING (Informed RRT*)");
            } else if (mode == "combinatorial") {
                planner_type = planning::PlannerType::COMBINATORIAL;
                ROS_INFO("Using planner mode: COMBINATORIAL (A* grid search)");
            } else {
                ROS_WARN("Unknown mode '%s', using default: combinatorial", mode.c_str());
            }
            break;
        }
    }

    planning::ROSInterface interface;
    interface.setPlannerType(planner_type);
    interface.run();

    return 0;
}
