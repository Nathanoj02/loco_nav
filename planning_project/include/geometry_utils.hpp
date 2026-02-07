#ifndef PLANNING_PROJECT_GEOMETRY_UTILS_HPP
#define PLANNING_PROJECT_GEOMETRY_UTILS_HPP

#include <vector>
#include <geometry_msgs/Polygon.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include "config.hpp"

namespace planning {

/**
 * @brief Offset (expand/shrink) a polygon by a given distance.
 *
 * Positive offset expands the polygon outward, negative shrinks it inward.
 * Uses the edge-normal method: moves each edge along its outward normal,
 * then computes new vertices at edge intersections.
 *
 * @param polygon Input polygon vertices (assumed to be in CCW order)
 * @param offset Distance to offset (positive = expand, negative = shrink)
 * @return Offset polygon
 */
geometry_msgs::Polygon offsetPolygon(const geometry_msgs::Polygon& polygon, double offset);

/**
 * @brief Convert a circle to a polygon approximation.
 *
 * @param center_x X coordinate of circle center
 * @param center_y Y coordinate of circle center
 * @param radius Circle radius
 * @param num_vertices Number of vertices for approximation (default 16)
 * @return Polygon approximating the circle
 */
geometry_msgs::Polygon circleToPolygon(
    double center_x,
    double center_y,
    double radius,
    int num_vertices = 16);

/**
 * @brief Check if an obstacle is a circle (radius > 0 and single point).
 */
bool isCircleObstacle(const obstacles_msgs::ObstacleMsg& obstacle);

/**
 * @brief Process a single obstacle, inflating it by the configured amount.
 *
 * Handles both polygon and circle obstacles.
 * Circles are converted to polygons after inflation.
 *
 * @param obstacle The obstacle to process
 * @param inflation Amount to inflate (default: from config)
 * @return Inflated polygon representation
 */
geometry_msgs::Polygon inflateObstacle(
    const obstacles_msgs::ObstacleMsg& obstacle,
    double inflation);

/**
 * @brief Process a single obstacle using config's total inflation.
 */
geometry_msgs::Polygon inflateObstacle(const obstacles_msgs::ObstacleMsg& obstacle);

/**
 * @brief Process all obstacles from an ObstacleArrayMsg.
 *
 * @param msg The obstacle array message
 * @param inflation Amount to inflate each obstacle
 * @return Vector of inflated polygons
 */
std::vector<geometry_msgs::Polygon> inflateObstacles(
    const obstacles_msgs::ObstacleArrayMsg& msg,
    double inflation);

/**
 * @brief Process all obstacles using config's total inflation.
 */
std::vector<geometry_msgs::Polygon> inflateObstacles(
    const obstacles_msgs::ObstacleArrayMsg& msg);

/**
 * @brief Shrink map borders inward by the configured amount.
 *
 * This is equivalent to offsetPolygon with negative offset.
 * Borders are assumed to be in CCW order (interior on the left).
 *
 * @param borders The map border polygon
 * @param shrink_amount Amount to shrink inward (positive value)
 * @return Shrunk border polygon
 */
geometry_msgs::Polygon shrinkBorders(
    const geometry_msgs::Polygon& borders,
    double shrink_amount);

/**
 * @brief Shrink map borders using config's total inflation.
 */
geometry_msgs::Polygon shrinkBorders(const geometry_msgs::Polygon& borders);

}  // namespace planning

#endif  // PLANNING_PROJECT_GEOMETRY_UTILS_HPP
