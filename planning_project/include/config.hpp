#ifndef PLANNING_PROJECT_CONFIG_HPP
#define PLANNING_PROJECT_CONFIG_HPP

namespace planning {

// Planner type selection
enum class PlannerType {
    COMBINATORIAL,  // A* on grid (approximate cell decomposition)
    SAMPLING        // Informed RRT*
};

struct Config {
    // Planner selection
    PlannerType planner_type = PlannerType::SAMPLING;

    // Grid parameters (for combinatorial)
    double grid_resolution = 0.5;       // meters per cell
    int refinement_depth = 3;           // max subdivision depth for MIXED cells

    // RRT* parameters (for sampling)
    int rrt_max_iterations = 5000;      // max iterations per query
    double rrt_step_size = 0.4;         // extension step size
    double rrt_goal_radius = 0.3;       // goal acceptance radius

    // Robot geometry
    double robot_radius = 0.25;         // LIMO half-width approximately
    double safety_margin = 0.05;        // extra clearance from obstacles (current, may be adjusted by retry)
    double max_safety_margin = 0.25;    // start retry with this (100% of robot_radius)
    double min_safety_margin = 0.0;     // lowest we'll go
    double margin_step = 0.05;          // decrement per retry

    // Dubins parameters
    double dubins_rho = 1;            // minimum turning radius (v_max / omega_max)

    // Time budget
    double time_buffer_ratio = 0.15;    // reserve 15% of timeout for safety

    // Robot velocity (constant speed assumption)
    double robot_velocity = 1;        // m/s

    // Computed helper
    double totalInflation() const {
        return robot_radius + safety_margin;
    }
};

// Global config instance - modify this to change all parameters
inline Config& getConfig() {
    static Config config;
    return config;
}

}  // namespace planning

#endif  // PLANNING_PROJECT_CONFIG_HPP
