#include "pathfinding.hpp"
#include "rrt_star.hpp"
#include "config.hpp"
#include <queue>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include <limits>
#include <iostream>

namespace planning {

namespace {
constexpr double EPSILON = 1e-6;

// Check if two 1D intervals overlap
bool intervalsOverlap(double a0, double a1, double b0, double b1) {
    return (a0 < b1 - EPSILON) && (b0 < a1 - EPSILON);
}
}  // namespace

bool CellGraph::cellsAdjacent(const Cell& a, const Cell& b) const {
    // Check if right edge of a touches left edge of b (or vice versa)
    bool x_touch_right = std::abs(a.x1 - b.x0) < EPSILON;
    bool x_touch_left = std::abs(a.x0 - b.x1) < EPSILON;
    bool y_overlap = intervalsOverlap(a.y0, a.y1, b.y0, b.y1);

    if ((x_touch_right || x_touch_left) && y_overlap) {
        return true;
    }

    // Check if top edge of a touches bottom edge of b (or vice versa)
    bool y_touch_top = std::abs(a.y1 - b.y0) < EPSILON;
    bool y_touch_bottom = std::abs(a.y0 - b.y1) < EPSILON;
    bool x_overlap = intervalsOverlap(a.x0, a.x1, b.x0, b.x1);

    if ((y_touch_top || y_touch_bottom) && x_overlap) {
        return true;
    }

    return false;
}

double CellGraph::cellDistance(const Cell& a, const Cell& b) const {
    double dx = a.centerX() - b.centerX();
    double dy = a.centerY() - b.centerY();
    return std::sqrt(dx * dx + dy * dy);
}

double CellGraph::heuristic(const Cell& cell, double goal_x, double goal_y) const {
    double dx = cell.centerX() - goal_x;
    double dy = cell.centerY() - goal_y;
    return std::sqrt(dx * dx + dy * dy);
}

void CellGraph::buildFromGridMap(const GridMap& grid_map) {
    cells_.clear();
    adjacency_.clear();

    // Collect only FREE cells
    const auto& leaf_cells = grid_map.getLeafCells();
    for (const auto& cell : leaf_cells) {
        if (cell.state == CellState::FREE) {
            cells_.push_back(cell);
        }
    }

    size_t n = cells_.size();
    adjacency_.resize(n);

    // Build adjacency list - O(n^2)
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            if (cellsAdjacent(cells_[i], cells_[j])) {
                adjacency_[i].push_back(j);
                adjacency_[j].push_back(i);
            }
        }
    }

    // Print statistics
    size_t edge_count = numEdges();
    std::cout << "CellGraph built: " << n << " nodes, " << edge_count << " edges\n";
    if (n > 0) {
        std::cout << "  Average degree: " << (2.0 * edge_count / n) << std::endl;
    }
}

size_t CellGraph::numEdges() const {
    size_t count = 0;
    for (const auto& neighbors : adjacency_) {
        count += neighbors.size();
    }
    return count / 2;  // Each edge counted twice
}

int CellGraph::findCellContaining(double x, double y) const {
    for (size_t i = 0; i < cells_.size(); ++i) {
        const Cell& c = cells_[i];
        if (x >= c.x0 && x <= c.x1 && y >= c.y0 && y <= c.y1) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

PathResult CellGraph::findPath(double start_x, double start_y,
                                double goal_x, double goal_y) const {
    PathResult result;

    int start_idx = findCellContaining(start_x, start_y);
    int goal_idx = findCellContaining(goal_x, goal_y);

    if (start_idx < 0 || goal_idx < 0) {
        // Start or goal not in free space
        return result;
    }

    if (start_idx == goal_idx) {
        // Already at goal
        result.found = true;
        result.distance = 0.0;
        result.cell_indices.push_back(start_idx);
        result.waypoints.push_back({cells_[start_idx].centerX(),
                                     cells_[start_idx].centerY()});
        return result;
    }

    size_t n = cells_.size();

    // A* data structures
    std::vector<double> g_score(n, std::numeric_limits<double>::infinity());
    std::vector<double> f_score(n, std::numeric_limits<double>::infinity());
    std::vector<int> came_from(n, -1);
    std::vector<bool> closed(n, false);

    // Priority queue: (f_score, node_index)
    using PQEntry = std::pair<double, size_t>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> open_set;

    g_score[start_idx] = 0.0;
    f_score[start_idx] = heuristic(cells_[start_idx], goal_x, goal_y);
    open_set.push({f_score[start_idx], static_cast<size_t>(start_idx)});

    while (!open_set.empty()) {
        size_t current = open_set.top().second;
        open_set.pop();

        if (closed[current]) continue;
        closed[current] = true;

        if (current == static_cast<size_t>(goal_idx)) {
            // Reconstruct path
            result.found = true;
            result.distance = g_score[goal_idx];

            std::vector<size_t> path;
            int node = goal_idx;
            while (node >= 0) {
                path.push_back(node);
                node = came_from[node];
            }
            std::reverse(path.begin(), path.end());

            result.cell_indices = path;
            for (size_t idx : path) {
                result.waypoints.push_back({cells_[idx].centerX(),
                                            cells_[idx].centerY()});
            }
            return result;
        }

        for (size_t neighbor : adjacency_[current]) {
            if (closed[neighbor]) continue;

            double tentative_g = g_score[current] + cellDistance(cells_[current], cells_[neighbor]);

            if (tentative_g < g_score[neighbor]) {
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                f_score[neighbor] = tentative_g + heuristic(cells_[neighbor], goal_x, goal_y);
                open_set.push({f_score[neighbor], neighbor});
            }
        }
    }

    // No path found
    return result;
}

bool segmentCollisionFree(
    double x1, double y1, double x2, double y2,
    const std::vector<geometry_msgs::Polygon>& obstacles,
    const geometry_msgs::Polygon* border,
    double sample_step) {

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dist = std::sqrt(dx * dx + dy * dy);
    int num_checks = std::max(2, static_cast<int>(std::ceil(dist / sample_step)));

    for (int i = 0; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;
        double x = x1 + t * dx;
        double y = y1 + t * dy;

        if (pointCollidesWithObstacles(x, y, obstacles, border)) {
            return false;
        }
    }
    return true;
}

std::vector<std::pair<double, double>> lineOfSightSimplify(
    const std::vector<std::pair<double, double>>& waypoints,
    const std::vector<geometry_msgs::Polygon>& obstacles,
    const geometry_msgs::Polygon* border) {

    if (waypoints.size() <= 2) {
        return waypoints;
    }

    std::vector<std::pair<double, double>> simplified;
    simplified.push_back(waypoints[0]);

    size_t current = 0;
    while (current < waypoints.size() - 1) {
        // Try to reach the farthest point directly (greedy skip)
        size_t farthest = current + 1;

        for (size_t j = waypoints.size() - 1; j > current + 1; --j) {
            if (segmentCollisionFree(
                    waypoints[current].first, waypoints[current].second,
                    waypoints[j].first, waypoints[j].second,
                    obstacles, border)) {
                farthest = j;
                break;
            }
        }

        simplified.push_back(waypoints[farthest]);
        current = farthest;
    }

    std::cout << "  Line-of-sight simplification: " << waypoints.size()
              << " -> " << simplified.size() << " waypoints" << std::endl;
    return simplified;
}

std::vector<std::pair<double, double>> sampleDubinsCurve(
    const DubinsCurve& curve,
    double step_size) {

    std::vector<std::pair<double, double>> samples;

    for (int arc_idx = 0; arc_idx < 3; ++arc_idx) {
        const DubinsArc& arc = curve.arcs[arc_idx];
        if (arc.length < 1e-6) continue;

        int num_samples = std::max(2, static_cast<int>(arc.length / step_size));
        double dt = arc.length / (num_samples - 1);

        double x = arc.start.x;
        double y = arc.start.y;
        double theta = arc.start.theta;

        for (int i = 0; i < num_samples; ++i) {
            samples.push_back({x, y});

            if (i < num_samples - 1) {
                // Integrate forward
                if (std::abs(arc.k) < 1e-6) {
                    // Straight line
                    x += dt * std::cos(theta);
                    y += dt * std::sin(theta);
                } else {
                    // Arc
                    double dtheta = arc.k * dt;
                    double radius = 1.0 / arc.k;
                    x += radius * (std::sin(theta + dtheta) - std::sin(theta));
                    y += radius * (-std::cos(theta + dtheta) + std::cos(theta));
                    theta += dtheta;
                }
            }
        }
    }

    return samples;
}

bool pointCollidesWithObstacles(
    double x, double y,
    const std::vector<geometry_msgs::Polygon>& obstacles,
    const geometry_msgs::Polygon* border) {

    // Outside border = collision
    if (border && !pointInPolygon(x, y, *border)) {
        return true;
    }

    for (const auto& obstacle : obstacles) {
        if (pointInPolygon(x, y, obstacle)) {
            return true;
        }
    }
    return false;
}

bool dubinsCurveCollides(
    const DubinsCurve& curve,
    const std::vector<geometry_msgs::Polygon>& obstacles,
    const geometry_msgs::Polygon* border,
    double sample_step) {

    auto samples = sampleDubinsCurve(curve, sample_step);

    for (const auto& pt : samples) {
        if (pointCollidesWithObstacles(pt.first, pt.second, obstacles, border)) {
            return true;
        }
    }

    return false;
}

std::vector<std::vector<double>> computeDubinsDistanceMatrix(
    const CellGraph& graph,
    const std::vector<std::pair<double, double>>& points,
    const std::vector<double>& headings,
    const std::vector<geometry_msgs::Polygon>& obstacles) {

    size_t n = points.size();
    std::vector<std::vector<double>> matrix(n, std::vector<double>(n, -1.0));

    for (size_t i = 0; i < n; ++i) {
        matrix[i][i] = 0.0;

        for (size_t j = i + 1; j < n; ++j) {
            auto path_result = graph.findPath(points[i].first, points[i].second,
                                               points[j].first, points[j].second);

            if (!path_result.found) {
                continue;  // No path exists
            }

            double estimated_dist = path_result.distance * planning::getConfig().dubins_factor;
            matrix[i][j] = estimated_dist;
            matrix[j][i] = estimated_dist;
        }
    }

    return matrix;
}

bool directDubinsCollides(
    const Pose& start,
    const Pose& end,
    double kmax,
    const std::vector<geometry_msgs::Polygon>& obstacles,
    const geometry_msgs::Polygon* border,
    double sample_step) {

    int pidx;
    DubinsCurve curve;
    Pose start_copy = start;
    Pose end_copy = end;

    float length = dubins_shortest_path(pidx, curve, start_copy, end_copy, static_cast<float>(kmax));

    if (pidx < 0 || length >= MAXFLOAT) {
        return true;  // No valid Dubins path exists, treat as collision
    }

    return dubinsCurveCollides(curve, obstacles, border, sample_step);
}

std::vector<Point> buildSafeWaypoints(
    const CellGraph& graph,
    const std::vector<std::pair<double, double>>& route,
    double start_theta,
    double end_theta,
    double kmax,
    const std::vector<geometry_msgs::Polygon>& obstacles,
    const geometry_msgs::Polygon* border) {

    std::vector<Point> waypoints;

    if (route.size() < 2) {
        return waypoints;
    }

    // For each segment in the route
    for (size_t i = 0; i < route.size() - 1; ++i) {
        double x1 = route[i].first, y1 = route[i].second;
        double x2 = route[i + 1].first, y2 = route[i + 1].second;

        // Determine headings for this segment
        double theta1, theta2;
        if (i == 0) {
            theta1 = start_theta;
        } else {
            // Use direction from previous point
            double dx = x1 - route[i - 1].first;
            double dy = y1 - route[i - 1].second;
            theta1 = std::atan2(dy, dx);
        }

        if (i == route.size() - 2) {
            // Last segment, heading to gate
            theta2 = end_theta;
        } else {
            // Use direction to next point
            double dx = x2 - x1;
            double dy = y2 - y1;
            theta2 = std::atan2(dy, dx);
        }

        Pose pose1 = {x1, y1, theta1};
        Pose pose2 = {x2, y2, theta2};

        // Check if direct Dubins collides
        bool collides = directDubinsCollides(pose1, pose2, kmax, obstacles, border);

        if (collides) {
            std::cout << "  Segment " << i << " (" << x1 << "," << y1 << ") -> ("
                      << x2 << "," << y2 << ") COLLIDES, adding A* waypoints" << std::endl;

            // Get A* path around obstacles
            auto path_result = graph.findPath(x1, y1, x2, y2);

            if (path_result.found && path_result.waypoints.size() > 2) {
                // Simplify using line-of-sight pruning
                auto simplified = lineOfSightSimplify(path_result.waypoints, obstacles, border);

                for (size_t j = 1; j < simplified.size() - 1; ++j) {
                    waypoints.push_back({simplified[j].first, simplified[j].second});
                }
            }
        } else {
            std::cout << "  Segment " << i << " (" << x1 << "," << y1 << ") -> ("
                      << x2 << "," << y2 << ") direct OK" << std::endl;
        }

        // Add the destination point (except for the last point which is the gate/end)
        if (i < route.size() - 2) {
            waypoints.push_back({x2, y2});
        }
    }

    return waypoints;
}

PathInfo generateDubinsPath(
    const Pose& start,
    const Pose& end,
    std::vector<Point>& intermediate_points,
    double kmax,
    const std::vector<geometry_msgs::Polygon>& obstacles,
    const geometry_msgs::Polygon* border,
    int num_angles,
    int refine_steps) {

    Pose start_copy = start;
    Pose end_copy = end;

    std::cout << "Generating multi-point Dubins path:" << std::endl;
    std::cout << "  Start: (" << start.x << ", " << start.y << ", " << start.theta << ")" << std::endl;
    std::cout << "  End: (" << end.x << ", " << end.y << ", " << end.theta << ")" << std::endl;
    std::cout << "  Intermediate points: " << intermediate_points.size() << std::endl;

    for (size_t i = 0; i < intermediate_points.size(); ++i) {
        std::cout << "    [" << i << "] (" << intermediate_points[i].x
                  << ", " << intermediate_points[i].y << ")" << std::endl;
    }

    // Create collision checker that tests Dubins curves against obstacles and border
    DubinsCollisionFn collision_checker = [&obstacles, border](const DubinsCurve& curve) -> bool {
        return dubinsCurveCollides(curve, obstacles, border, 0.05);
    };

    PathInfo result = multi_point_dubins_shortest_path(
        start_copy, end_copy, intermediate_points,
        num_angles, static_cast<float>(kmax), refine_steps,
        collision_checker);

    std::cout << "  Total path length: " << result.cost << " m" << std::endl;
    std::cout << "  Number of curves: " << result.curves.size() << std::endl;

    return result;
}

std::vector<Point> buildSafeWaypointsRRT(
    const std::vector<std::pair<double, double>>& route,
    double start_theta,
    double end_theta,
    double kmax,
    const std::vector<geometry_msgs::Polygon>& obstacles,
    const std::array<double, 4>& bounds,
    const geometry_msgs::Polygon* border) {

    std::vector<Point> waypoints;

    if (route.size() < 2) {
        return waypoints;
    }

    const auto& config = getConfig();

    std::cout << "Building safe waypoints using Informed RRT*..." << std::endl;

    // For each segment in the route
    for (size_t i = 0; i < route.size() - 1; ++i) {
        double x1 = route[i].first, y1 = route[i].second;
        double x2 = route[i + 1].first, y2 = route[i + 1].second;

        // Determine headings for this segment
        double theta1, theta2;
        if (i == 0) {
            theta1 = start_theta;
        } else {
            double dx = x1 - route[i - 1].first;
            double dy = y1 - route[i - 1].second;
            theta1 = std::atan2(dy, dx);
        }

        if (i == route.size() - 2) {
            theta2 = end_theta;
        } else {
            double dx = x2 - x1;
            double dy = y2 - y1;
            theta2 = std::atan2(dy, dx);
        }

        Pose pose1 = {x1, y1, theta1};
        Pose pose2 = {x2, y2, theta2};

        // Check if direct Dubins collides
        bool collides = directDubinsCollides(pose1, pose2, kmax, obstacles, border);

        if (collides) {
            std::cout << "  Segment " << i << " (" << x1 << "," << y1 << ") -> ("
                      << x2 << "," << y2 << ") COLLIDES, running RRT*" << std::endl;

            // Run Informed RRT*
            InformedRRTStar rrt(obstacles, bounds);
            rrt.setStepSize(config.rrt_step_size);

            auto result = rrt.plan(
                x1, y1, x2, y2,
                config.rrt_max_iterations,
                config.rrt_goal_radius);

            if (result.found && result.path.size() > 2) {
                // Smooth the RRT* path
                auto smoothed = smoothRRTPath(result.path, obstacles, 50, 30);

                std::cout << "    RRT* path: " << result.path.size() << " nodes"
                          << " -> smoothed: " << smoothed.size() << " waypoints" << std::endl;

                // Add intermediate waypoints (skip first and last)
                for (size_t j = 1; j < smoothed.size() - 1; ++j) {
                    waypoints.push_back({static_cast<float>(smoothed[j].first),
                                        static_cast<float>(smoothed[j].second)});
                }
            } else {
                std::cout << "    WARNING: RRT* failed to find path!" << std::endl;
            }
        } else {
            std::cout << "  Segment " << i << " (" << x1 << "," << y1 << ") -> ("
                      << x2 << "," << y2 << ") direct OK" << std::endl;
        }

        // Add the destination point (except for the last point which is the gate/end)
        if (i < route.size() - 2) {
            waypoints.push_back({static_cast<float>(x2), static_cast<float>(y2)});
        }
    }

    std::cout << "  Total intermediate waypoints: " << waypoints.size() << std::endl;
    return waypoints;
}

}  // namespace planning
