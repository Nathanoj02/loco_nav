#ifndef PLANNING_PROJECT_CONFIG_HPP
#define PLANNING_PROJECT_CONFIG_HPP

namespace planning {

struct Config {
    // Grid parameters
    double grid_resolution = 0.5;       // meters per cell
    int refinement_depth = 3;           // max subdivision depth for MIXED cells

    // Robot geometry
    double robot_radius = 0.25;         // LIMO half-width approximately
    double safety_margin = 0.05;        // extra clearance from obstacles

    // Dubins parameters
    double dubins_rho = 1;            // minimum turning radius (v_max / omega_max)

    // Time budget
    double time_buffer_ratio = 0.15;    // reserve 15% of timeout for safety

    // Robot velocity (constant speed assumption)
    double robot_velocity = 1.0;        // m/s

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
