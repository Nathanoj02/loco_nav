#ifndef PLANNING_PROJECT_ORIENTEERING_HPP
#define PLANNING_PROJECT_ORIENTEERING_HPP

#include <vector>
#include <limits>

namespace planning {

/**
 * @brief Result of the Orienteering Problem solver.
 *
 * The Orienteering Problem: Given a start, end, set of locations with values,
 * and a time/distance budget, find the path that maximizes collected value
 * while staying within budget.
 */
struct OrienteeringResult {
    bool feasible = false;              // At least reaching the gate is possible
    double total_value = 0.0;           // Sum of collected victim values
    double total_distance = 0.0;        // Total path distance
    std::vector<int> route;             // Ordered victim indices to visit (0-indexed into victims array)
                                        // Does NOT include start or gate - those are implicit
};

/**
 * @brief Solve the Orienteering Problem using dynamic programming.
 *
 * Given a distance matrix and victim values, find the route that maximizes
 * total value while staying within the distance budget.
 *
 * Distance matrix structure:
 *   - Index 0: robot start position
 *   - Indices 1 to N-2: victims (victim i is at index i+1)
 *   - Index N-1: gate (must be reached)
 *
 * @param distance_matrix NxN matrix of distances (-1 means no path)
 * @param victim_values Values for each victim (size = num_victims)
 * @param max_distance Maximum total distance allowed
 * @return OrienteeringResult with optimal route
 */
OrienteeringResult solveOrienteeringDP(
    const std::vector<std::vector<double>>& distance_matrix,
    const std::vector<double>& victim_values,
    double max_distance);

/**
 * @brief Solve the Orienteering Problem using greedy heuristic.
 *
 * For large instances (>15 victims), uses a value/distance ratio greedy approach.
 * Faster but may not find optimal solution.
 *
 * @param distance_matrix NxN matrix of distances (-1 means no path)
 * @param victim_values Values for each victim (size = num_victims)
 * @param max_distance Maximum total distance allowed
 * @return OrienteeringResult with greedy route
 */
OrienteeringResult solveOrienteeringGreedy(
    const std::vector<std::vector<double>>& distance_matrix,
    const std::vector<double>& victim_values,
    double max_distance);

/**
 * @brief Choose appropriate solver based on problem size.
 *
 * Uses DP for ≤15 victims, greedy for larger instances.
 */
OrienteeringResult solveOrienteering(
    const std::vector<std::vector<double>>& distance_matrix,
    const std::vector<double>& victim_values,
    double max_distance);

}  // namespace planning

#endif  // PLANNING_PROJECT_ORIENTEERING_HPP
