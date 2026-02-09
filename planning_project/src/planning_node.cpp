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
 *   rosrun planning_project planning_node --mode combinatorial --debug
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "planning_node");

    // Parse command-line flags
    planning::PlannerType planner_type = planning::PlannerType::COMBINATORIAL;  // default
    bool debug_mode = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--mode" && i + 1 < argc) {
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
            ++i;  // skip next argument
        } else if (arg == "--debug") {
            debug_mode = true;
        }
    }

    planning::ROSInterface interface;
    interface.setPlannerType(planner_type);
    interface.setDebugMode(debug_mode);
    interface.run();

    return 0;
}
