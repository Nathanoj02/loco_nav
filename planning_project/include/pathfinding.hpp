#ifndef PLANNING_PROJECT_PATHFINDING_HPP
#define PLANNING_PROJECT_PATHFINDING_HPP

#include <vector>
#include <unordered_map>
#include <optional>
#include <array>
#include "grid_map.hpp"
#include "dubins.hpp"
#include "multipoint_dubins.hpp"
#include "config.hpp"

namespace planning {

/**
 * @brief Result of A* pathfinding.
 */
struct PathResult {
    bool found = false;
    double distance = 0.0;
    std::vector<size_t> cell_indices;  // indices into leaf_cells
    std::vector<std::pair<double, double>> waypoints;  // cell centers
};

/**
 * @brief Graph for pathfinding on subdivided cells.
 */
class CellGraph {
public:
    /**
     * @brief Build graph from GridMap's leaf cells.
     *
     * Creates adjacency list where cells sharing a boundary are connected.
     */
    void buildFromGridMap(const GridMap& grid_map);

    /**
     * @brief Find path from start position to goal position using A*.
     *
     * @param start_x, start_y Start world coordinates
     * @param goal_x, goal_y Goal world coordinates
     * @return PathResult with path info (empty if no path found)
     */
    PathResult findPath(double start_x, double start_y,
                        double goal_x, double goal_y) const;

    /**
     * @brief Find cell containing a world point.
     *
     * @return Index into cells_, or -1 if not found
     */
    int findCellContaining(double x, double y) const;

    /**
     * @brief Get number of nodes (FREE cells) in the graph.
     */
    size_t numNodes() const { return cells_.size(); }

    /**
     * @brief Get number of edges in the graph.
     */
    size_t numEdges() const;

    /**
     * @brief Get the cells (for external access).
     */
    const std::vector<Cell>& getCells() const { return cells_; }

    /**
     * @brief Get neighbors of a cell.
     */
    const std::vector<size_t>& getNeighbors(size_t cell_idx) const;

private:
    std::vector<Cell> cells_;  // FREE cells only
    std::vector<std::vector<size_t>> adjacency_;  // adjacency list

    // Check if two cells are adjacent (share boundary)
    bool cellsAdjacent(const Cell& a, const Cell& b) const;

    // Euclidean distance between cell centers
    double cellDistance(const Cell& a, const Cell& b) const;

    // Heuristic for A* (Euclidean distance to goal)
    double heuristic(const Cell& cell, double goal_x, double goal_y) const;
};

/**
 * @brief Compute distance matrix between a set of points using A* (Euclidean).
 *
 * @param graph The cell graph for pathfinding
 * @param points Vector of (x, y) coordinates
 * @return NxN matrix of distances (-1 if no path exists)
 */
std::vector<std::vector<double>> computeDistanceMatrix(
    const CellGraph& graph,
    const std::vector<std::pair<double, double>>& points);

/**
 * @brief Simplify a path by removing collinear waypoints.
 *
 * Keeps only waypoints where direction changes significantly.
 *
 * @param waypoints Original waypoints from A*
 * @param angle_threshold Minimum angle change to keep a waypoint (radians)
 * @return Simplified waypoint list
 */
std::vector<std::pair<double, double>> simplifyPath(
    const std::vector<std::pair<double, double>>& waypoints,
    double angle_threshold = 0.1);

/**
 * @brief Compute Dubins path length through a sequence of waypoints.
 *
 * @param waypoints Sequence of (x, y) waypoints
 * @param start_theta Starting orientation
 * @param end_theta Ending orientation (at last waypoint)
 * @param kmax Maximum curvature (1/rho)
 * @return Total Dubins path length, or -1 if no valid path
 */
double computeDubinsPathLength(
    const std::vector<std::pair<double, double>>& waypoints,
    double start_theta,
    double end_theta,
    double kmax);

/**
 * @brief Sample points along a Dubins curve for collision checking.
 *
 * @param curve The Dubins curve
 * @param step_size Distance between samples
 * @return Vector of (x, y) sample points
 */
std::vector<std::pair<double, double>> sampleDubinsCurve(
    const DubinsCurve& curve,
    double step_size = 0.05);

/**
 * @brief Check if a point is inside any of the obstacle polygons.
 */
bool pointCollidesWithObstacles(
    double x, double y,
    const std::vector<geometry_msgs::Polygon>& obstacles);

/**
 * @brief Check if a Dubins curve collides with obstacles.
 *
 * @param curve The Dubins curve to check
 * @param obstacles Inflated obstacle polygons
 * @param sample_step Distance between collision check samples
 * @return true if collision detected
 */
bool dubinsCurveCollides(
    const DubinsCurve& curve,
    const std::vector<geometry_msgs::Polygon>& obstacles,
    double sample_step = 0.05);

/**
 * @brief Compute distance matrix using Dubins paths through A* waypoints.
 *
 * For each pair:
 * 1. Find A* path (collision-free waypoints)
 * 2. Simplify path to key waypoints
 * 3. Compute Dubins path length through waypoints
 *
 * @param graph The cell graph for pathfinding
 * @param points Vector of (x, y) coordinates
 * @param headings Vector of orientations at each point (can be NaN for unknown)
 * @param obstacles Inflated obstacles for collision checking
 * @return NxN matrix of Dubins distances (-1 if no path exists)
 */
std::vector<std::vector<double>> computeDubinsDistanceMatrix(
    const CellGraph& graph,
    const std::vector<std::pair<double, double>>& points,
    const std::vector<double>& headings,
    const std::vector<geometry_msgs::Polygon>& obstacles);

/**
 * @brief Check if a direct Dubins path between two poses collides with obstacles.
 *
 * @param start Start pose (x, y, theta)
 * @param end End pose (x, y, theta)
 * @param kmax Maximum curvature
 * @param obstacles Inflated obstacles
 * @param sample_step Distance between collision check samples
 * @return true if collision detected, false if path is clear
 */
bool directDubinsCollides(
    const Pose& start,
    const Pose& end,
    double kmax,
    const std::vector<geometry_msgs::Polygon>& obstacles,
    double sample_step = 0.05);

/**
 * @brief Build safe waypoints list for multi-point Dubins solver.
 *
 * For each segment in the route:
 * - Try direct Dubins, check collision
 * - If collision, insert A* waypoints to go around obstacles
 *
 * @param graph Cell graph for A* pathfinding
 * @param route Ordered list of positions: [start, victim0, victim1, ..., gate]
 * @param start_theta Starting orientation
 * @param end_theta Required gate orientation
 * @param kmax Maximum curvature
 * @param obstacles Inflated obstacles
 * @return Vector of intermediate Points for multi_point_dubins (excludes start/end poses)
 */
std::vector<Point> buildSafeWaypoints(
    const CellGraph& graph,
    const std::vector<std::pair<double, double>>& route,
    double start_theta,
    double end_theta,
    double kmax,
    const std::vector<geometry_msgs::Polygon>& obstacles);

/**
 * @brief Generate complete Dubins path through route using multi-point solver.
 *
 * @param start Start pose
 * @param end End pose (gate)
 * @param intermediate_points Safe waypoints from buildSafeWaypoints
 * @param kmax Maximum curvature
 * @param num_angles Number of angle discretizations for DP (default 8)
 * @param refine_steps Refinement iterations (default 2)
 * @return PathInfo containing all Dubins curves
 */
PathInfo generateDubinsPath(
    const Pose& start,
    const Pose& end,
    std::vector<Point>& intermediate_points,
    double kmax,
    int num_angles = 8,
    int refine_steps = 2);

/**
 * @brief Build safe waypoints using Informed RRT* (sampling-based approach).
 *
 * Alternative to buildSafeWaypoints that uses RRT* instead of A* on grid.
 * For each segment, runs RRT* to find collision-free path, then smooths it.
 *
 * @param route Ordered list of positions: [start, victim0, victim1, ..., gate]
 * @param start_theta Starting orientation
 * @param end_theta Required gate orientation
 * @param kmax Maximum curvature
 * @param obstacles Inflated obstacles
 * @param bounds World boundaries [min_x, max_x, min_y, max_y]
 * @return Vector of intermediate Points for multi_point_dubins
 */
std::vector<Point> buildSafeWaypointsRRT(
    const std::vector<std::pair<double, double>>& route,
    double start_theta,
    double end_theta,
    double kmax,
    const std::vector<geometry_msgs::Polygon>& obstacles,
    const std::array<double, 4>& bounds);

}  // namespace planning

#endif  // PLANNING_PROJECT_PATHFINDING_HPP
