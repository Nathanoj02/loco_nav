#include "pathfinding.hpp"
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
    // Two cells are adjacent if they share a boundary segment
    // This happens when:
    // 1. One dimension's edges are touching (equal within epsilon)
    // 2. The other dimension's intervals overlap

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

    // Build adjacency list - O(n^2) but acceptable for reasonable cell counts
    // Could be optimized with spatial indexing for large maps
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

const std::vector<size_t>& CellGraph::getNeighbors(size_t cell_idx) const {
    static const std::vector<size_t> empty;
    if (cell_idx >= adjacency_.size()) return empty;
    return adjacency_[cell_idx];
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

std::vector<std::vector<double>> computeDistanceMatrix(
    const CellGraph& graph,
    const std::vector<std::pair<double, double>>& points) {

    size_t n = points.size();
    std::vector<std::vector<double>> matrix(n, std::vector<double>(n, -1.0));

    for (size_t i = 0; i < n; ++i) {
        matrix[i][i] = 0.0;  // Distance to self is 0

        for (size_t j = i + 1; j < n; ++j) {
            auto result = graph.findPath(points[i].first, points[i].second,
                                         points[j].first, points[j].second);
            if (result.found) {
                matrix[i][j] = result.distance;
                matrix[j][i] = result.distance;  // Symmetric
            }
        }
    }

    return matrix;
}

std::vector<std::pair<double, double>> simplifyPath(
    const std::vector<std::pair<double, double>>& waypoints,
    double angle_threshold) {

    if (waypoints.size() <= 2) {
        return waypoints;
    }

    std::vector<std::pair<double, double>> simplified;
    simplified.push_back(waypoints[0]);

    for (size_t i = 1; i < waypoints.size() - 1; ++i) {
        double dx1 = waypoints[i].first - waypoints[i-1].first;
        double dy1 = waypoints[i].second - waypoints[i-1].second;
        double dx2 = waypoints[i+1].first - waypoints[i].first;
        double dy2 = waypoints[i+1].second - waypoints[i].second;

        double angle1 = std::atan2(dy1, dx1);
        double angle2 = std::atan2(dy2, dx2);

        double angle_diff = std::abs(angle2 - angle1);
        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;

        // Keep waypoint if direction changes significantly
        if (angle_diff > angle_threshold) {
            simplified.push_back(waypoints[i]);
        }
    }

    simplified.push_back(waypoints.back());
    return simplified;
}

double computeDubinsPathLength(
    const std::vector<std::pair<double, double>>& waypoints,
    double start_theta,
    double end_theta,
    double kmax) {

    if (waypoints.size() < 2) {
        return 0.0;
    }

    double total_length = 0.0;

    // Compute heading at each waypoint based on direction to next waypoint
    std::vector<double> headings(waypoints.size());
    headings[0] = start_theta;
    headings.back() = end_theta;

    // Estimate intermediate headings as direction to next waypoint
    for (size_t i = 1; i < waypoints.size() - 1; ++i) {
        double dx = waypoints[i+1].first - waypoints[i].first;
        double dy = waypoints[i+1].second - waypoints[i].second;
        headings[i] = std::atan2(dy, dx);
    }

    // Compute Dubins path between consecutive waypoints
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        Pose start_pose = {waypoints[i].first, waypoints[i].second, headings[i]};
        Pose end_pose = {waypoints[i+1].first, waypoints[i+1].second, headings[i+1]};

        int pidx;
        DubinsCurve curve;
        float length = dubins_shortest_path(pidx, curve, start_pose, end_pose, kmax);

        if (pidx < 0 || length >= MAXFLOAT) {
            return -1.0;  // No valid Dubins path
        }

        total_length += length;
    }

    return total_length;
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
    const std::vector<geometry_msgs::Polygon>& obstacles) {

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
    double sample_step) {

    auto samples = sampleDubinsCurve(curve, sample_step);

    for (const auto& pt : samples) {
        if (pointCollidesWithObstacles(pt.first, pt.second, obstacles)) {
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

    // Use A* grid distances × heuristic factor to estimate Dubins travel cost.
    // Dubins curves add ~20-40% overhead over straight-line due to turning arcs.
    // Actual Dubins paths are computed later for the chosen route.
    constexpr double DUBINS_FACTOR = 1.3;

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

            double estimated_dist = path_result.distance * DUBINS_FACTOR;
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
    double sample_step) {

    int pidx;
    DubinsCurve curve;
    Pose start_copy = start;
    Pose end_copy = end;

    float length = dubins_shortest_path(pidx, curve, start_copy, end_copy, static_cast<float>(kmax));

    if (pidx < 0 || length >= MAXFLOAT) {
        return true;  // No valid Dubins path exists, treat as collision
    }

    return dubinsCurveCollides(curve, obstacles, sample_step);
}

std::vector<Point> buildSafeWaypoints(
    const CellGraph& graph,
    const std::vector<std::pair<double, double>>& route,
    double start_theta,
    double end_theta,
    double kmax,
    const std::vector<geometry_msgs::Polygon>& obstacles) {

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
        bool collides = directDubinsCollides(pose1, pose2, kmax, obstacles);

        if (collides) {
            std::cout << "  Segment " << i << " (" << x1 << "," << y1 << ") -> ("
                      << x2 << "," << y2 << ") COLLIDES, adding A* waypoints" << std::endl;

            // Get A* path around obstacles
            auto path_result = graph.findPath(x1, y1, x2, y2);

            if (path_result.found && path_result.waypoints.size() > 2) {
                // Simplify and add intermediate waypoints (skip first and last)
                auto simplified = simplifyPath(path_result.waypoints, 0.3);

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

    PathInfo result = multi_point_dubins_shortest_path(
        start_copy, end_copy, intermediate_points,
        num_angles, static_cast<float>(kmax), refine_steps);

    std::cout << "  Total path length: " << result.cost << " m" << std::endl;
    std::cout << "  Number of curves: " << result.curves.size() << std::endl;

    return result;
}

}  // namespace planning
