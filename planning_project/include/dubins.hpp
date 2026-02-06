#ifndef DUBINS_HPP
#define DUBINS_HPP

#include <functional>

/**
 * Point in 2D space
 */
typedef struct Point {
    double x;
    double y;
} Point;

/**
 * Pose in 2D space (x, y, theta)
 */
typedef struct Pose {
    double x;
    double y;
    double theta; // orientation in radians
} Pose;

/**
 * Represents an arc of Dubins curve (straight or circular)
 */
typedef struct DubinsArc {
    Pose start;
    float k;
    float length;
    Pose end;
} DubinsArc;

/**
 * Represents a full Dubins curve, composed of 3 arcs
 */
typedef struct DubinsCurve {
    DubinsArc arcs[3];
    float total_length;
} DubinsCurve;


/**
 * Create a Dubins arc given a starting pose, ending pose and max curvature
 * @param[out] pidx Index of the selected primitive (0-5 corresponding to LSL, LSR, 
 *                  RSL, RSR, LRL, RLR), or -1 if no valid solution exists
 * @param[out] curve The Dubins curve to populate
 * @param[in] start The starting pose
 * @param[in] end The ending pose
 * @param[in] kmax The maximum curvature
 * @return The total length of the Dubins curve, or MAXFLOAT if no valid solution exists
 */
float dubins_shortest_path(int &pidx, DubinsCurve &curve, Pose &start, Pose &end, float kmax);

/**
 * Collision checker type: returns true if the curve collides with obstacles
 */
using DubinsCollisionFn = std::function<bool(const DubinsCurve&)>;

/**
 * Find the shortest collision-free Dubins path among all 6 types.
 * Tries all 6 primitives (LSL, RSR, LSR, RSL, RLR, LRL) and returns
 * the shortest one that doesn't collide.
 * @param[out] pidx Primitive index, or -1 if all collide
 * @param[out] curve The collision-free Dubins curve
 * @param[in] start Starting pose
 * @param[in] end Ending pose
 * @param[in] kmax Maximum curvature
 * @param[in] collides Collision checking function
 * @return Total length, or MAXFLOAT if no collision-free path
 */
float dubins_shortest_collision_free_path(
    int &pidx, DubinsCurve &curve, Pose &start, Pose &end, float kmax,
    const DubinsCollisionFn& collides);

#endif // DUBINS_HPP