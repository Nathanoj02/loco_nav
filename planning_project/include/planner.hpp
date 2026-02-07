#ifndef PLANNING_PROJECT_PLANNER_HPP
#define PLANNING_PROJECT_PLANNER_HPP

#include <geometry_msgs/Polygon.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <memory>
#include <vector>
#include <array>
#include <string>
#include "grid_map.hpp"
#include "pathfinding.hpp"
#include "multipoint_dubins.hpp"

namespace planning {

struct Victim {
    double x, y;
    double value;
};

struct TrajectoryPoint {
    double x, y, theta;
    double v, omega;
    double time;
};

/**
 * @brief Core planning logic
 *
 * Handles map building, pathfinding, orienteering, and Dubins path generation.
 * Uses ROS message types for data but has no ROS communication dependencies.
 */
class Planner {
public:
    Planner();

    // ========================================================================
    // Map Building
    // ========================================================================

    /**
     * @brief Set map data (borders and obstacles).
     */
    void setMapData(
        const geometry_msgs::Polygon& borders,
        const obstacles_msgs::ObstacleArrayMsg& obstacles
    );

    /**
     * @brief Add other robots as obstacles.
     */
    void addOtherRobots(const std::vector<obstacles_msgs::ObstacleMsg>& robots);

    /**
     * @brief Build grid map with specified safety margin.
     */
    void buildMapWithMargin(double safety_margin);

    // ========================================================================
    // Distance Matrix
    // ========================================================================

    /**
     * @brief Set goal data (robot pose, victims, gate).
     */
    void setGoalData(
        double robot_x, double robot_y, double robot_theta,
        const std::vector<Victim>& victims,
        double gate_x, double gate_y, double gate_theta
    );

    /**
     * @brief Compute distance matrix between all points.
     */
    void computeDistanceMatrix();

    /**
     * @brief Check if all victims are reachable from start and to gate.
     * @param safety_margin Current safety margin (for logging)
     * @return true if all victims are reachable, false otherwise
     */
    bool checkVictimsReachable(double safety_margin) const;

    // ========================================================================
    // Path Planning
    // ========================================================================

    /**
     * @brief Solve orienteering problem and select victims to visit.
     * @return true if a feasible route was found
     */
    bool solveOrienteering(int timeout_seconds);

    /**
     * @brief Generate Dubins path through chosen route.
     * @return true if path was generated successfully
     */
    bool generatePath();

    // ========================================================================
    // Trajectory
    // ========================================================================

    /**
     * @brief Sample trajectory at high frequency for execution.
     */
    void sampleTrajectory();

    /**
     * @brief Get trajectory pose and velocity at given time.
     * @return (pose, velocity) pair
     */
    std::pair<TrajectoryPoint, TrajectoryPoint> getTrajectoryAt(double t) const;

    // ========================================================================
    // File I/O
    // ========================================================================

    /**
     * @brief Save grid map to JSON file.
     */
    bool saveMapToFile(const std::string& filepath) const;

    /**
     * @brief Save trajectory to JSON file.
     */
    bool saveTrajectoryToFile(const std::string& filepath) const;

    /**
     * @brief Build safe waypoints using RRT* and save tree to file.
     */
    std::vector<Point> buildSafeWaypointsRRTWithSave(
        const std::vector<std::pair<double, double>>& route,
        double start_theta,
        double end_theta,
        double kmax,
        const std::array<double, 4>& bounds,
        const std::string& rrt_filepath
    );

    // ========================================================================
    // Getters
    // ========================================================================

    const CellGraph* getCellGraph() const { return cell_graph_.get(); }
    const PathInfo& getDubinsPath() const { return dubins_path_; }
    const std::vector<TrajectoryPoint>& getTrajectory() const { return trajectory_; }
    const std::vector<int>& getChosenRoute() const { return chosen_route_; }
    const std::vector<Victim>& getVictims() const { return victims_; }
    const std::vector<geometry_msgs::Polygon>& getInflatedObstacles() const { return inflated_obstacles_; }
    const geometry_msgs::Polygon& getShrunkBorders() const { return shrunk_borders_; }
    const geometry_msgs::Polygon& getBorders() const { return borders_; }

    double getRobotX() const { return robot_x_; }
    double getRobotY() const { return robot_y_; }
    double getRobotTheta() const { return robot_theta_; }
    double getGateX() const { return gate_x_; }
    double getGateY() const { return gate_y_; }
    double getGateTheta() const { return gate_theta_; }

    double getTotalDistance() const { return dubins_path_.cost; }
    double getTotalTime() const { return trajectory_.empty() ? 0 : trajectory_.back().time; }
    int getNumVictimsVisited() const { return chosen_route_.size(); }

private:
    std::array<double, 4> computeWorldBounds() const;

    // Map data
    geometry_msgs::Polygon borders_;
    obstacles_msgs::ObstacleArrayMsg obstacles_;

    // Goal data
    double robot_x_ = 0, robot_y_ = 0, robot_theta_ = 0;
    std::vector<Victim> victims_;
    double gate_x_ = 0, gate_y_ = 0, gate_theta_ = 0;

    // Planning structures
    std::unique_ptr<GridMap> grid_map_;
    std::unique_ptr<CellGraph> cell_graph_;
    std::vector<geometry_msgs::Polygon> inflated_obstacles_;
    geometry_msgs::Polygon shrunk_borders_;
    std::vector<std::vector<double>> distance_matrix_;

    // Results
    std::vector<int> chosen_route_;
    PathInfo dubins_path_;
    std::vector<TrajectoryPoint> trajectory_;
};

}  // namespace planning

#endif  // PLANNING_PROJECT_PLANNER_HPP
