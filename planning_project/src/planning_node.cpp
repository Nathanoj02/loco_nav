#include <ros/ros.h>
#include "ros_interface.hpp"

/**
 * @brief Main entry point for the planning node.
 *
 * Creates ROSInterface which handles all ROS communication
 * and orchestrates the planning pipeline.
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "planning_node");

    planning::ROSInterface interface;
    interface.run();

    return 0;
}
