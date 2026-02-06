#include "multipoint_dubins.hpp"
#include "math_utils.hpp"

#include <cmath>
#include <cstdio>
#include <unordered_map>

/**
 * Constructs a graph matrix where each row corresponds to a point
 * and each column corresponds to a discrete orientation at that point.
 * @param points A vector of points (x, y)
 * @param num_angles The number of discrete orientations to consider at each point
 * @return A matrix of Pose structs representing the graph
 */
Matrix<Pose> construct_graph_matrix(std::vector<Point> &points, int num_angles) {
    Matrix<Pose> matrix(points.size(), std::vector<Pose>(num_angles));

    for (size_t i = 0; i < points.size(); i++) {
        for (int j = 0; j < num_angles; j++) {
            matrix[i][j] = {points[i].x, points[i].y, 2 * M_PI * j / num_angles};
        }
    }

    return matrix;
}

/**
 * Refines the graph matrix by narrowing the range of angles around
 * the best angle found in the previous iteration.
 * @param matrix The graph matrix to refine
 * @param path The current best path information
 * @param num_angles The number of discrete orientations at each point
 * @param refine_step The current refinement step (0-indexed)
 */
void refine_graph_matrix(Matrix<Pose> &matrix, PathInfo &path, int num_angles, int refine_step) {
    float h = 2 * M_PI / (pow(num_angles, refine_step + 1));
    
    for (size_t i = 0; i < matrix.size(); i++) {
        // Find best angle for point i
        float theta = path.curves[i+1].arcs[0].start.theta;
        
        float minAngle = theta - 1.5 * h;
        float maxAngle = theta + 1.5 * h;

        for (size_t j = 0; j < matrix[i].size(); j++) {
            matrix[i][j].theta = mod2pi(minAngle + (maxAngle - minAngle) * j / (num_angles - 1));
        }
    }
}


/**
 * Recursive function to compute the minimum cost path using dynamic programming
 * @param column The current column index in the graph matrix
 * @param graph_matrix The graph matrix of poses
 * @param end The ending pose
 * @param start_absolute The starting pose
 * @param kmax The maximum curvature
 * @return A PathInfo struct containing the minimum cost path information
 */
PathInfo multipoint_dubins_cost(
    int column, Matrix<Pose>& graph_matrix,
    Pose &end, Pose &start_absolute, float kmax,
    std::unordered_map<std::string, PathInfo> &memo,
    const DubinsCollisionFn& collision_checker
) {
    // Memoization
    std::string key = std::to_string(column) + "_" + std::to_string(end.x) + "_"
                        + std::to_string(end.y) + "_" + std::to_string(end.theta);

    if (memo.find(key) != memo.end()) {
        return memo[key];
    }

    PathInfo result;

    // Base case: direct path from start to first column of points
    if (column == -1) {
        result.pids.resize(1);
        result.curves.resize(1);
        if (collision_checker) {
            result.cost = dubins_shortest_collision_free_path(
                result.pids[0], result.curves[0], start_absolute, end, kmax, collision_checker);
        } else {
            result.cost = dubins_shortest_path(result.pids[0], result.curves[0],
                                              start_absolute, end, kmax);
        }

        memo[key] = result; // Store in memo

        return result;
    }

    // Try all angles at current column
    float min_cost = INFINITY;
    PathInfo best_path;
    int best_pidx = -1;
    DubinsCurve best_curve;

    for (size_t i = 0; i < graph_matrix[column].size(); i++) {
        Pose mid = graph_matrix[column][i];

        // Cost from mid to end
        int temp_pidx;
        DubinsCurve temp_curve;
        float cost_to_end;
        if (collision_checker) {
            cost_to_end = dubins_shortest_collision_free_path(
                temp_pidx, temp_curve, mid, end, kmax, collision_checker);
        } else {
            cost_to_end = dubins_shortest_path(temp_pidx, temp_curve, mid, end, kmax);
        }

        if (cost_to_end >= MAXFLOAT) continue; // No valid collision-free path for this angle

        // Recursive cost from start to mid
        PathInfo path_to_mid = multipoint_dubins_cost(column - 1, graph_matrix,
                                                      mid, start_absolute, kmax, memo,
                                                      collision_checker);

        float total_cost = path_to_mid.cost + cost_to_end;

        if (total_cost < min_cost) {
            min_cost = total_cost;
            best_path = path_to_mid;
            best_pidx = temp_pidx;
            best_curve = temp_curve;
        }
    }

    // Build result: previous path + new segment
    result.cost = min_cost;
    result.pids = best_path.pids;
    result.curves = best_path.curves;
    result.pids.push_back(best_pidx);
    result.curves.push_back(best_curve);

    // Store in memo
    memo[key] = result;

    return result;
}


PathInfo multi_point_dubins_shortest_path(
    Pose start, Pose end, std::vector<Point> &points, int num_angles, float kmax, int refine_steps,
    DubinsCollisionFn collision_checker
) {
    Matrix<Pose> graph_matrix = construct_graph_matrix(points, num_angles);
    std::unordered_map<std::string, PathInfo> memo;

    PathInfo result;

    for (int i = 0; i < refine_steps; i++) {
        memo.clear(); // Clear memoization for each refinement step

        result = multipoint_dubins_cost(graph_matrix.size() - 1, graph_matrix, end, start, kmax, memo,
                                        collision_checker);

        if (i == refine_steps - 1) {
            break; // No need to refine on last iteration
        }

        refine_graph_matrix(graph_matrix, result, num_angles, i);
    }

    return result;
}