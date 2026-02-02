#ifndef MULTIPOINT_DUBINS_HPP
#define MULTIPOINT_DUBINS_HPP

#include "dubins.hpp"
#include <vector>

template<typename T>
using Matrix = std::vector<std::vector<T>>;

typedef struct PathInfo {
    float cost;
    std::vector<int> pids;
    std::vector<DubinsCurve> curves;
} PathInfo;

/**
 * Solves the multi-point Dubins path problem using dynamic programming
 * @param start The starting pose (x, y, theta)
 * @param end The ending pose (x, y, theta)
 * @param points A vector of intermediate points (x, y) to visit
 * @param num_angles The number of discrete orientations to consider at each point
 * @param kmax The maximum curvature
 * @param refine_steps The number of refinement steps to perform
 * @return A struct containing the total cost, pids, and curves of the optimal path
 */
PathInfo multi_point_dubins_shortest_path(
    Pose start, Pose end, std::vector<Point> &points, int num_angles, float kmax, int refine_steps
);

#endif // MULTIPOINT_DUBINS_HPP