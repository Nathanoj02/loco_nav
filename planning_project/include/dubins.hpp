#ifndef DUBINS_HPP
#define DUBINS_HPP

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

#endif // DUBINS_HPP