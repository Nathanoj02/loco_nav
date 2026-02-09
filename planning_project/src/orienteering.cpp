#include "orienteering.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>

namespace planning {

namespace {
constexpr double INF = std::numeric_limits<double>::infinity();
constexpr int MAX_DP_VICTIMS = 15;

// For DP memoization: encode (current_node, visited_mask) -> (best_value, best_distance, best_route)
struct DPState {
    double value = -1.0;      // -1 means not computed
    double distance = INF;
    std::vector<int> route;
};

}  // namespace

OrienteeringResult solveOrienteeringDP(
    const std::vector<std::vector<double>>& dist,
    const std::vector<double>& values,
    double max_distance) {

    OrienteeringResult result;
    size_t n = dist.size();  // Total nodes: start + victims + gate
    size_t num_victims = values.size();

    if (n < 2) return result;

    const size_t START = 0;
    const size_t GATE = n - 1;

    // Check if direct path to gate exists
    if (dist[START][GATE] < 0) {
        std::cout << "  [OP] No path from start to gate!" << std::endl;
        return result;
    }

    // If no victims or direct path is only option
    if (num_victims == 0) {
        result.feasible = true;
        result.total_distance = dist[START][GATE];
        result.total_value = 0;
        return result;
    }

    // DP on subsets
    // State: dp[mask][last] = {min_distance to reach 'last' having visited 'mask'}
    // mask is a bitmask of visited victims (victim i = bit i)
    size_t num_states = 1 << num_victims;

    // dp[mask][last] = minimum distance to reach victim 'last' (0-indexed) having visited victims in 'mask'
    std::vector<std::vector<double>> dp_dist(num_states, std::vector<double>(num_victims, INF));
    std::vector<std::vector<int>> dp_prev(num_states, std::vector<int>(num_victims, -1));

    // Initialize: start -> each victim directly
    for (size_t v = 0; v < num_victims; ++v) {
        size_t node = v + 1;  // victim v is at dist index v+1
        if (dist[START][node] >= 0) {
            size_t mask = 1 << v;
            dp_dist[mask][v] = dist[START][node];
            dp_prev[mask][v] = -1;  // came from start
        }
    }

    // Fill DP table
    for (size_t mask = 1; mask < num_states; ++mask) {
        for (size_t last = 0; last < num_victims; ++last) {
            if (!(mask & (1 << last))) continue;  // last must be in mask
            if (dp_dist[mask][last] >= INF) continue;

            // Try extending to each unvisited victim
            for (size_t next = 0; next < num_victims; ++next) {
                if (mask & (1 << next)) continue;  // already visited

                size_t last_node = last + 1;
                size_t next_node = next + 1;
                if (dist[last_node][next_node] < 0) continue;

                size_t new_mask = mask | (1 << next);
                double new_dist = dp_dist[mask][last] + dist[last_node][next_node];

                if (new_dist < dp_dist[new_mask][next]) {
                    dp_dist[new_mask][next] = new_dist;
                    dp_prev[new_mask][next] = static_cast<int>(last);
                }
            }
        }
    }

    // Find best route: for each subset, check if we can reach gate within budget
    double best_value = 0;
    double best_total_dist = dist[START][GATE];  // direct to gate
    size_t best_mask = 0;
    int best_last = -1;

    // Always feasible if direct path exists
    result.feasible = true;

    for (size_t mask = 0; mask < num_states; ++mask) {
        // Calculate value of this subset
        double subset_value = 0;
        for (size_t v = 0; v < num_victims; ++v) {
            if (mask & (1 << v)) {
                subset_value += values[v];
            }
        }

        if (mask == 0) {
            // No victims - direct path
            continue;
        }

        // Find minimum distance to complete this subset and reach gate
        for (size_t last = 0; last < num_victims; ++last) {
            if (!(mask & (1 << last))) continue;
            if (dp_dist[mask][last] >= INF) continue;

            size_t last_node = last + 1;
            if (dist[last_node][GATE] < 0) continue;

            double total = dp_dist[mask][last] + dist[last_node][GATE];

            if (total <= max_distance && subset_value > best_value) {
                best_value = subset_value;
                best_total_dist = total;
                best_mask = mask;
                best_last = static_cast<int>(last);
            }
        }
    }

    result.total_value = best_value;
    result.total_distance = best_total_dist;

    // Reconstruct route
    if (best_last >= 0) {
        std::vector<int> route;
        size_t mask = best_mask;
        int curr = best_last;

        while (curr >= 0) {
            route.push_back(curr);
            int prev = dp_prev[mask][curr];
            mask ^= (1 << curr);
            curr = prev;
        }

        std::reverse(route.begin(), route.end());
        result.route = route;
    }

    return result;
}

OrienteeringResult solveOrienteeringGreedy(
    const std::vector<std::vector<double>>& dist,
    const std::vector<double>& values,
    double max_distance) {

    OrienteeringResult result;
    size_t n = dist.size();
    size_t num_victims = values.size();

    if (n < 2) return result;

    const size_t START = 0;
    const size_t GATE = n - 1;

    if (dist[START][GATE] < 0) {
        std::cout << "  [OP-Greedy] No path from start to gate!" << std::endl;
        return result;
    }

    result.feasible = true;

    if (num_victims == 0) {
        result.total_distance = dist[START][GATE];
        return result;
    }

    // Greedy: repeatedly add the victim with best value/detour ratio
    std::vector<bool> visited(num_victims, false);
    std::vector<int> route;
    size_t current = START;
    double current_dist = 0;

    while (true) {
        double best_ratio = -1;
        int best_victim = -1;
        double best_detour = INF;

        for (size_t v = 0; v < num_victims; ++v) {
            if (visited[v]) continue;

            size_t v_node = v + 1;

            // Can we reach this victim and still get to gate?
            if (dist[current][v_node] < 0 || dist[v_node][GATE] < 0) continue;

            double detour = dist[current][v_node] + dist[v_node][GATE];
            double remaining_after = max_distance - current_dist - detour;

            if (remaining_after < 0) continue;  // Can't afford this victim

            // Value per unit detour (compared to going directly to gate from current)
            double direct_to_gate = dist[current][GATE];
            double extra_dist = detour - direct_to_gate;
            if (extra_dist < 0) extra_dist = 0.01;  // Victim is on the way

            double ratio = values[v] / extra_dist;

            if (ratio > best_ratio) {
                best_ratio = ratio;
                best_victim = static_cast<int>(v);
                best_detour = dist[current][v + 1];
            }
        }

        if (best_victim < 0) break;  // No more affordable victims

        // Add this victim to route
        visited[best_victim] = true;
        route.push_back(best_victim);
        current_dist += best_detour;
        current = best_victim + 1;
        result.total_value += values[best_victim];
    }

    // Add distance to gate
    result.total_distance = current_dist + dist[current][GATE];
    result.route = route;

    return result;
}

OrienteeringResult solveOrienteering(
    const std::vector<std::vector<double>>& distance_matrix,
    const std::vector<double>& victim_values,
    double max_distance) {

    std::cout << "  [Orienteering] Solving with " << victim_values.size()
              << " victims, budget=" << max_distance << "m" << std::endl;

    OrienteeringResult result;

    if (victim_values.size() <= MAX_DP_VICTIMS) {
        std::cout << "  [Orienteering] Using DP (optimal)" << std::endl;
        result = solveOrienteeringDP(distance_matrix, victim_values, max_distance);
    } else {
        std::cout << "  [Orienteering] Using Greedy (victims > " << MAX_DP_VICTIMS << ")" << std::endl;
        result = solveOrienteeringGreedy(distance_matrix, victim_values, max_distance);
    }

    if (result.feasible) {
        std::cout << "  [Orienteering] Solution: value=" << result.total_value
                  << ", distance=" << result.total_distance << "m" << std::endl;
        std::cout << "  [Orienteering] Route: Start";
        for (int v : result.route) {
            std::cout << " -> V" << v;
        }
        std::cout << " -> Gate" << std::endl;
    } else {
        std::cout << "  [Orienteering] No feasible solution!" << std::endl;
    }

    return result;
}

}  // namespace planning
