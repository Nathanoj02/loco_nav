#include "rrt_star.hpp"
#include "grid_map.hpp"  // For pointInPolygon
#include <algorithm>
#include <iostream>
#include <chrono>

namespace planning {

// ============================================================================
// InformedRRTStar Implementation
// ============================================================================

InformedRRTStar::InformedRRTStar(
    const std::vector<geometry_msgs::Polygon>& obstacles,
    const std::array<double, 4>& bounds)
    : obstacles_(obstacles), bounds_(bounds) {

    // Seed RNG with current time
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    rng_.seed(static_cast<unsigned int>(seed));
}

RRTResult InformedRRTStar::plan(
    double start_x, double start_y,
    double goal_x, double goal_y,
    int max_iterations,
    double goal_radius) {

    RRTResult result;

    // Clear previous tree
    nodes_.clear();

    // Initialize tree with start node
    auto start_node = std::make_unique<RRTNode>(start_x, start_y);
    root_ = start_node.get();
    nodes_.push_back(std::move(start_node));

    // Setup informed sampling parameters
    updateInformedParams(start_x, start_y, goal_x, goal_y);
    c_best_ = std::numeric_limits<double>::infinity();

    // Track best goal node
    RRTNode* best_goal_node = nullptr;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Sample point (uniform or informed based on c_best)
        auto [sample_x, sample_y] = (c_best_ < std::numeric_limits<double>::infinity())
            ? sampleInformed()
            : sample();

        // Goal biasing
        if (uniform_dist_(rng_) < goal_bias_) {
            sample_x = goal_x;
            sample_y = goal_y;
        }

        // Find nearest node
        RRTNode* nearest_node = nearest(sample_x, sample_y);
        if (!nearest_node) continue;

        // Steer toward sample
        auto [new_x, new_y] = steer(nearest_node, sample_x, sample_y);

        // Check collision
        if (!collisionFree(nearest_node->x, nearest_node->y, new_x, new_y)) {
            continue;
        }

        if (!pointInFreeSpace(new_x, new_y)) {
            continue;
        }

        // Create new node
        auto new_node = std::make_unique<RRTNode>(new_x, new_y);

        // Find near nodes for rewiring
        double rewire_radius = computeRewireRadius();
        auto near = nearNodes(new_x, new_y, rewire_radius);

        // Choose best parent from near nodes
        RRTNode* best_parent = nearest_node;
        double best_cost = nearest_node->cost + distance(nearest_node->x, nearest_node->y, new_x, new_y);

        for (RRTNode* near_node : near) {
            double tentative_cost = near_node->cost + distance(near_node->x, near_node->y, new_x, new_y);
            if (tentative_cost < best_cost &&
                collisionFree(near_node->x, near_node->y, new_x, new_y)) {
                best_parent = near_node;
                best_cost = tentative_cost;
            }
        }

        // Set parent and cost
        new_node->parent = best_parent;
        new_node->cost = best_cost;
        best_parent->children.push_back(new_node.get());

        // Rewire near nodes through new node if beneficial
        for (RRTNode* near_node : near) {
            if (near_node == best_parent) continue;

            double new_cost = new_node->cost + distance(new_x, new_y, near_node->x, near_node->y);
            if (new_cost < near_node->cost &&
                collisionFree(new_x, new_y, near_node->x, near_node->y)) {

                // Remove from old parent's children
                if (near_node->parent) {
                    auto& children = near_node->parent->children;
                    children.erase(std::remove(children.begin(), children.end(), near_node), children.end());
                }

                // Update parent and cost
                near_node->parent = new_node.get();
                near_node->cost = new_cost;
                new_node->children.push_back(near_node);
            }
        }

        RRTNode* added_node = new_node.get();
        nodes_.push_back(std::move(new_node));

        // Check if we reached the goal
        double dist_to_goal = distance(added_node->x, added_node->y, goal_x, goal_y);
        if (dist_to_goal <= goal_radius) {
            // Connect to exact goal position if possible
            if (collisionFree(added_node->x, added_node->y, goal_x, goal_y)) {
                double goal_cost = added_node->cost + dist_to_goal;

                if (goal_cost < c_best_) {
                    // Create goal node or update best
                    auto goal_node = std::make_unique<RRTNode>(goal_x, goal_y);
                    goal_node->parent = added_node;
                    goal_node->cost = goal_cost;
                    added_node->children.push_back(goal_node.get());

                    best_goal_node = goal_node.get();
                    updateBestCost(goal_cost);
                    result.found = true;
                    result.cost = goal_cost;

                    nodes_.push_back(std::move(goal_node));

                    std::cout << "  RRT* found path at iter " << iter
                              << ", cost: " << c_best_ << std::endl;
                }
            }
        }
    }

    // Extract best path
    if (best_goal_node) {
        result.path = extractPath(best_goal_node);
    }

    return result;
}

std::pair<double, double> InformedRRTStar::sample() {
    double x = bounds_[0] + uniform_dist_(rng_) * (bounds_[1] - bounds_[0]);
    double y = bounds_[2] + uniform_dist_(rng_) * (bounds_[3] - bounds_[2]);
    return {x, y};
}

std::pair<double, double> InformedRRTStar::sampleInformed() {
    if (c_best_ >= std::numeric_limits<double>::infinity()) {
        return sample();
    }

    // Sample from unit ball
    auto [ball_x, ball_y] = sampleUnitBall();

    // Scale to ellipse dimensions
    double a = c_best_ / 2.0;  // Semi-major axis
    double b = std::sqrt(c_best_ * c_best_ - c_min_ * c_min_) / 2.0;  // Semi-minor axis

    if (b < 1e-6) b = a * 0.1;  // Avoid degenerate ellipse

    double ellipse_x = a * ball_x;
    double ellipse_y = b * ball_y;

    // Rotate and translate to world frame
    double world_x = cos_theta_ * ellipse_x - sin_theta_ * ellipse_y + x_center_;
    double world_y = sin_theta_ * ellipse_x + cos_theta_ * ellipse_y + y_center_;

    // Clamp to bounds
    world_x = std::max(bounds_[0], std::min(bounds_[1], world_x));
    world_y = std::max(bounds_[2], std::min(bounds_[3], world_y));

    return {world_x, world_y};
}

std::pair<double, double> InformedRRTStar::sampleUnitBall() {
    // Rejection sampling for uniform distribution in unit disk
    double x, y;
    do {
        x = 2.0 * uniform_dist_(rng_) - 1.0;
        y = 2.0 * uniform_dist_(rng_) - 1.0;
    } while (x * x + y * y > 1.0);
    return {x, y};
}

RRTNode* InformedRRTStar::nearest(double x, double y) {
    // Linear search
    RRTNode* nearest_node = nullptr;
    double min_dist = std::numeric_limits<double>::infinity();

    for (const auto& node : nodes_) {
        double d = distance(node->x, node->y, x, y);
        if (d < min_dist) {
            min_dist = d;
            nearest_node = node.get();
        }
    }

    return nearest_node;
}

std::pair<double, double> InformedRRTStar::steer(RRTNode* from, double to_x, double to_y) {
    double dx = to_x - from->x;
    double dy = to_y - from->y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= step_size_) {
        return {to_x, to_y};
    }

    // Move step_size toward target
    double ratio = step_size_ / dist;
    return {from->x + dx * ratio, from->y + dy * ratio};
}

bool InformedRRTStar::collisionFree(double x1, double y1, double x2, double y2) {
    // Sample along segment and check each point
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dist = std::sqrt(dx * dx + dy * dy);

    int num_checks = std::max(2, static_cast<int>(std::ceil(dist / 0.05)));

    for (int i = 0; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;
        double x = x1 + t * dx;
        double y = y1 + t * dy;

        if (!pointInFreeSpace(x, y)) {
            return false;
        }
    }

    return true;
}

bool InformedRRTStar::pointInFreeSpace(double x, double y) {
    // Check bounds
    if (x < bounds_[0] || x > bounds_[1] || y < bounds_[2] || y > bounds_[3]) {
        return false;
    }

    // Check against all obstacles (using point-in-polygon)
    for (const auto& obstacle : obstacles_) {
        if (pointInPolygon(x, y, obstacle)) {
            return false;
        }
    }

    return true;
}

std::vector<RRTNode*> InformedRRTStar::nearNodes(double x, double y, double radius) {
    std::vector<RRTNode*> near;

    for (const auto& node : nodes_) {
        if (distance(node->x, node->y, x, y) <= radius) {
            near.push_back(node.get());
        }
    }

    return near;
}

double InformedRRTStar::distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double InformedRRTStar::computeRewireRadius() {
    // RRT* rewiring radius: gamma * (log(n) / n)^(1/d)
    // For 2D: d = 2
    double n = static_cast<double>(nodes_.size());
    if (n < 2) return step_size_ * 2;

    double gamma = rewire_factor_ * std::sqrt(2.0 * (bounds_[1] - bounds_[0]) * (bounds_[3] - bounds_[2]) / M_PI);
    double radius = gamma * std::pow(std::log(n) / n, 0.5);

    return std::min(radius, step_size_ * 3.0);
}

void InformedRRTStar::updateInformedParams(double start_x, double start_y,
                                            double goal_x, double goal_y) {
    // Ellipse center
    x_center_ = (start_x + goal_x) / 2.0;
    y_center_ = (start_y + goal_y) / 2.0;

    // Minimum cost (straight-line distance)
    c_min_ = distance(start_x, start_y, goal_x, goal_y);

    // Rotation to align ellipse major axis with start-goal line
    double dx = goal_x - start_x;
    double dy = goal_y - start_y;
    double theta = std::atan2(dy, dx);
    cos_theta_ = std::cos(theta);
    sin_theta_ = std::sin(theta);
}

void InformedRRTStar::updateBestCost(double cost) {
    c_best_ = cost;
}

std::vector<std::pair<double, double>> InformedRRTStar::extractPath(RRTNode* goal_node) {
    std::vector<std::pair<double, double>> path;

    RRTNode* current = goal_node;
    while (current != nullptr) {
        path.push_back({current->x, current->y});
        current = current->parent;
    }

    // Reverse to get start-to-goal order
    std::reverse(path.begin(), path.end());
    return path;
}

// ============================================================================
// Path Smoothing
// ============================================================================

std::vector<std::pair<double, double>> smoothRRTPath(
    const std::vector<std::pair<double, double>>& path,
    const std::vector<geometry_msgs::Polygon>& obstacles,
    int shortcut_iterations,
    int smooth_iterations) {

    if (path.size() <= 2) {
        return path;
    }

    std::vector<std::pair<double, double>> smoothed = path;

    // Helper lambda for collision checking along segment
    auto segmentCollisionFree = [&obstacles](double x1, double y1, double x2, double y2) -> bool {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dist = std::sqrt(dx * dx + dy * dy);
        int num_checks = std::max(2, static_cast<int>(std::ceil(dist / 0.05)));

        for (int i = 0; i <= num_checks; ++i) {
            double t = static_cast<double>(i) / num_checks;
            double x = x1 + t * dx;
            double y = y1 + t * dy;

            for (const auto& obstacle : obstacles) {
                if (pointInPolygon(x, y, obstacle)) {
                    return false;
                }
            }
        }
        return true;
    };

    // Phase 1: Shortcutting - try to connect non-adjacent waypoints
    std::mt19937 rng(42);
    for (int iter = 0; iter < shortcut_iterations && smoothed.size() > 2; ++iter) {
        // Pick two random indices
        std::uniform_int_distribution<size_t> dist(0, smoothed.size() - 1);
        size_t i = dist(rng);
        size_t j = dist(rng);

        if (i > j) std::swap(i, j);
        if (j - i <= 1) continue;  // Already adjacent

        // Try to shortcut
        if (segmentCollisionFree(smoothed[i].first, smoothed[i].second,
                                  smoothed[j].first, smoothed[j].second)) {
            // Remove intermediate waypoints
            smoothed.erase(smoothed.begin() + i + 1, smoothed.begin() + j);
        }
    }

    // Phase 2: Gradient smoothing - move waypoints toward line between neighbors
    double step = 0.1;
    for (int iter = 0; iter < smooth_iterations; ++iter) {
        for (size_t i = 1; i < smoothed.size() - 1; ++i) {
            double prev_x = smoothed[i - 1].first;
            double prev_y = smoothed[i - 1].second;
            double next_x = smoothed[i + 1].first;
            double next_y = smoothed[i + 1].second;

            // Target: midpoint of neighbors
            double target_x = (prev_x + next_x) / 2.0;
            double target_y = (prev_y + next_y) / 2.0;

            // Gradient direction
            double dx = target_x - smoothed[i].first;
            double dy = target_y - smoothed[i].second;

            // New position
            double new_x = smoothed[i].first + step * dx;
            double new_y = smoothed[i].second + step * dy;

            // Check if move is valid
            bool valid = true;
            for (const auto& obstacle : obstacles) {
                if (pointInPolygon(new_x, new_y, obstacle)) {
                    valid = false;
                    break;
                }
            }

            // Also check segments to neighbors
            if (valid) {
                valid = segmentCollisionFree(prev_x, prev_y, new_x, new_y) &&
                        segmentCollisionFree(new_x, new_y, next_x, next_y);
            }

            if (valid) {
                smoothed[i] = {new_x, new_y};
            }
        }

        step *= 0.95;  // Decay step size
    }

    return smoothed;
}

}  // namespace planning
