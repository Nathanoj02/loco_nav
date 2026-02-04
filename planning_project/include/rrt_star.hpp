#ifndef PLANNING_PROJECT_RRT_STAR_HPP
#define PLANNING_PROJECT_RRT_STAR_HPP

#include <vector>
#include <memory>
#include <random>
#include <cmath>
#include <limits>
#include <array>
#include <geometry_msgs/Polygon.h>

namespace planning {

/**
 * @brief A node in the RRT* tree
 */
struct RRTNode {
    double x, y;
    double cost;  // Cost from root to this node
    RRTNode* parent;
    std::vector<RRTNode*> children;

    RRTNode(double x_, double y_) : x(x_), y(y_), cost(0.0), parent(nullptr) {}
};

/**
 * @brief Result of RRT* path planning
 */
struct RRTResult {
    bool found = false;
    double cost = std::numeric_limits<double>::infinity();
    std::vector<std::pair<double, double>> path;  // waypoints from start to goal
};

/**
 * @brief Informed RRT* path planner
 *
 * Implements asymptotically optimal sampling-based path planning.
 * Uses informed sampling (ellipsoidal heuristic) after finding initial solution.
 */
class InformedRRTStar {
public:
    /**
     * @brief Constructor
     * @param obstacles Inflated obstacle polygons for collision checking
     * @param bounds World boundaries [min_x, max_x, min_y, max_y]
     */
    InformedRRTStar(
        const std::vector<geometry_msgs::Polygon>& obstacles,
        const std::array<double, 4>& bounds);

    ~InformedRRTStar();

    /**
     * @brief Plan a path from start to goal
     * @param start_x, start_y Start position
     * @param goal_x, goal_y Goal position
     * @param max_iterations Maximum sampling iterations
     * @param goal_radius Acceptance radius around goal
     * @return RRTResult with path and cost
     */
    RRTResult plan(
        double start_x, double start_y,
        double goal_x, double goal_y,
        int max_iterations = 5000,
        double goal_radius = 0.3);

    /**
     * @brief Set step size for tree extension
     */
    void setStepSize(double step) { step_size_ = step; }

    /**
     * @brief Set rewiring radius factor (radius = factor * (log(n)/n)^(1/d))
     */
    void setRewireRadiusFactor(double factor) { rewire_factor_ = factor; }

    /**
     * @brief Get all nodes (for visualization)
     */
    const std::vector<std::unique_ptr<RRTNode>>& getNodes() const { return nodes_; }

    /**
     * @brief Save tree to JSON file for visualization
     * @param filename Output file path
     * @param path The solution path (if found)
     * @return true if saved successfully
     */
    bool saveToFile(const std::string& filename,
                    const std::vector<std::pair<double, double>>& path = {}) const;

private:
    // Obstacle data
    const std::vector<geometry_msgs::Polygon>& obstacles_;
    std::array<double, 4> bounds_;  // [min_x, max_x, min_y, max_y]

    // Tree structure
    std::vector<std::unique_ptr<RRTNode>> nodes_;
    RRTNode* root_ = nullptr;

    // Parameters
    double step_size_ = 0.5;
    double rewire_factor_ = 1.5;
    double goal_bias_ = 0.05;  // Probability of sampling goal directly

    // Informed sampling state
    double c_best_ = std::numeric_limits<double>::infinity();  // Best path cost
    double c_min_ = 0.0;   // Minimum possible cost (straight line)
    double x_center_, y_center_;  // Ellipse center
    double cos_theta_, sin_theta_;  // Rotation to align ellipse

    // Random number generation
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_dist_{0.0, 1.0};

    // Core RRT* functions
    std::pair<double, double> sample();
    std::pair<double, double> sampleInformed();
    std::pair<double, double> sampleUnitBall();
    RRTNode* nearest(double x, double y);
    std::pair<double, double> steer(RRTNode* from, double to_x, double to_y);
    bool collisionFree(double x1, double y1, double x2, double y2);
    bool pointInFreeSpace(double x, double y);
    std::vector<RRTNode*> nearNodes(double x, double y, double radius);
    double distance(double x1, double y1, double x2, double y2);
    double computeRewireRadius();

    // Informed sampling helpers
    void updateInformedParams(double start_x, double start_y,
                              double goal_x, double goal_y);
    void updateBestCost(double cost);

    // Path extraction
    std::vector<std::pair<double, double>> extractPath(RRTNode* goal_node);
};

/**
 * @brief Smooth an RRT* path using shortcutting and gradient descent
 * @param path Original path waypoints
 * @param obstacles Obstacles for collision checking
 * @param shortcut_iterations Number of shortcutting passes
 * @param smooth_iterations Number of gradient smoothing iterations
 * @return Smoothed path
 */
std::vector<std::pair<double, double>> smoothRRTPath(
    const std::vector<std::pair<double, double>>& path,
    const std::vector<geometry_msgs::Polygon>& obstacles,
    int shortcut_iterations = 50,
    int smooth_iterations = 30);

/**
 * @brief Compute distance matrix using Informed RRT*
 *
 * For each pair of points, runs RRT* to find optimal path.
 *
 * @param obstacles Inflated obstacles
 * @param bounds World boundaries
 * @param points Vector of (x, y) coordinates
 * @param max_iterations_per_query Max RRT* iterations per path
 * @return NxN matrix of distances (-1 if no path)
 */
std::vector<std::vector<double>> computeRRTDistanceMatrix(
    const std::vector<geometry_msgs::Polygon>& obstacles,
    const std::array<double, 4>& bounds,
    const std::vector<std::pair<double, double>>& points,
    int max_iterations_per_query = 3000);

}  // namespace planning

#endif  // PLANNING_PROJECT_RRT_STAR_HPP
