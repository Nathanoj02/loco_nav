#include "dubins.hpp"
#include "math_utils.hpp"
#include <cmath>

/**
 * Response structure for Dubins path primitives
 */
typedef struct DubinsResponse {
    bool ok;
    float scaled_s1;
    float scaled_s2;
    float scaled_s3;
} DubinsResponse;

/**
 * Function pointer type for Dubins path primitives
 */
typedef DubinsResponse (*DubinsFunc)(float, float, float);

// Evaluate an arc, given arc-length s
void circline(Pose* pose, float s, Pose* start_pose, float k) {
    pose->x = start_pose->x + s * sinc(k * s / 2.0) * cos(start_pose->theta + k * s / 2.0);
    pose->y = start_pose->y + s * sinc(k * s / 2.0) * sin(start_pose->theta + k * s / 2.0);
    pose->theta = mod2pi(start_pose->theta + k * s);
}

/**
 * Create a Dubins arc given a starting pose, curvature and length
 * @param start The starting pose
 * @param k The curvature
 * @param length The length of the arc
 * @return The created Dubins arc
 */
DubinsArc create_dubins_arc(Pose& start, float k, float length) {
    DubinsArc arc;
    arc.start = start;
    arc.k = k;
    arc.length = length;

    circline(&arc.end, length, &start, k);

    return arc;
}

/**
 * Create a Dubins curve given starting pose, lengths and curvatures of the 3 arcs
 * @param start The starting pose
 * @param s1 Length of the first arc
 * @param s2 Length of the second arc
 * @param s3 Length of the third arc
 * @param k0 Curvature of the first arc
 * @param k1 Curvature of the second arc
 * @param k2 Curvature of the third arc
 * @return The created Dubins curve
 */
DubinsCurve create_dubins_curve(Pose& start, float s1, float s2, float s3, float k0, float k1, float k2) {
    DubinsCurve curve;
    curve.arcs[0] = create_dubins_arc(start, k0, s1);
    curve.arcs[1] = create_dubins_arc(curve.arcs[0].end, k1, s2);
    curve.arcs[2] = create_dubins_arc(curve.arcs[1].end, k2, s3);
    
    curve.total_length = curve.arcs[0].length + curve.arcs[1].length + curve.arcs[2].length;

    return curve;
}

/**
 * Check the correctness of a Dubins curve solution
 * @param s1 Length of the first arc
 * @param s2 Length of the second arc
 * @param s3 Length of the third arc
 * @param k0 Curvature of the first arc
 * @param k1 Curvature of the second arc
 * @param k2 Curvature of the third arc
 * @param theta0 Starting orientation
 * @param thetaf Ending orientation
 * @return True if the solution is correct, false otherwise
 */
bool check(float s1, float s2, float s3, float k0, float k1, float k2, float theta0, float thetaf) {
    Pose start = {-1, 0, theta0};
    Pose end = {1, 0, thetaf};

    float eq1 = start.x + s1 * sinc(k0 * s1 / 2.0) * cos(start.theta + k0 * s1 / 2.0)
              + s2 * sinc(k1 * s2 / 2.0) * cos(start.theta + k0 * s1 + k1 * s2 / 2.0)
              + s3 * sinc(k2 * s3 / 2.0) * cos(start.theta + k0 * s1 + k1 * s2 + k2 * s3 / 2.0) - end.x;

    float eq2 = start.y + s1 * sinc(k0 * s1 / 2.0) * sin(start.theta + k0 * s1 / 2.0)
              + s2 * sinc(k1 * s2 / 2.0) * sin(start.theta + k0 * s1 + k1 * s2 / 2.0)
              + s3 * sinc(k2 * s3 / 2.0) * sin(start.theta + k0 * s1 + k1 * s2 + k2 * s3 / 2.0) - end.y;

    float eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + start.theta - end.theta);

    bool Lpos = (s1 >= 0) && (s2 >= 0) && (s3 >= 0);
    return Lpos && (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1e-5);
}

/**
 * Scale the problem to the standard form
 * @param[out] scaled_theta0 The scaled starting orientation
 * @param[out] scaled_thetaf The scaled ending orientation
 * @param[out] scaled_kmax The scaled maximum curvature
 * @param[out] lambda The scaling factor
 * @param[in] start The starting pose
 * @param[in] end The ending pose
 * @param[in] kmax The maximum curvature
 */
void scaleToStandard(float &scaled_theta0, float &scaled_thetaf, float &scaled_kmax, float &lambda, Pose &start, Pose &end, float kmax) {
    // Find transform parameters
    float dx = end.x - start.x;
    float dy = end.y - start.y;
    float phi = atan2(dy, dx);
    lambda = hypot(dx, dy)/2;

    // Scale and normalize angles and curvature
    scaled_theta0 = mod2pi(start.theta - phi);
    scaled_thetaf = mod2pi(end.theta - phi);
    scaled_kmax = kmax * lambda;
}

/**
 * Scale the solution from the standard form to the original problem
 * @param[out] s1 The length of the first arc in the original problem
 * @param[out] s2 The length of the second arc in the original problem
 * @param[out] s3 The length of the third arc in the original problem
 * @param[in] scaled_s1 The length of the first arc in the standard problem
 * @param[in] scaled_s2 The length of the second arc in the standard problem
 * @param[in] scaled_s3 The length of the third arc in the standard problem
 * @param[in] lambda The scaling factor
 */
void scaleFromStandard(float &s1, float &s2, float &s3, float scaled_s1, float scaled_s2, float scaled_s3, float lambda) {
    s1 = scaled_s1 * lambda;
    s2 = scaled_s2 * lambda;
    s3 = scaled_s3 * lambda;
}

/* ===== DUBINS PATH PRIMITIVES ===== */

/**
 * @param scaled_theta0 The scaled starting orientation
 * @param scaled_thetaf The scaled ending orientation
 * @param scaled_kmax The scaled maximum curvature
 * @return The response containing the lengths of the arcs and validity flag
 */
DubinsResponse LSL(float scaled_theta0, float scaled_thetaf, float scaled_kmax) {
    DubinsResponse response;

    float invK = 1 / scaled_kmax;

    float C = cos(scaled_thetaf) - cos(scaled_theta0);
    float S = 2 * scaled_kmax + sin(scaled_theta0) - sin(scaled_thetaf);
    
    float temp1 = atan2(C, S);
    response.scaled_s1 = invK * mod2pi(temp1 - scaled_theta0);

    float temp2 = 2 + 4 * scaled_kmax * scaled_kmax - 2 * cos(scaled_theta0 - scaled_thetaf) + 4 * scaled_kmax * (sin(scaled_theta0) - sin(scaled_thetaf));
    if (temp2 < 0) {
        response.ok = false;
        response.scaled_s1 = 0;
        response.scaled_s2 = 0;
        response.scaled_s3 = 0;
        return response;
    }

    response.scaled_s2 = invK * sqrtf(temp2);
    response.scaled_s3 = invK * mod2pi(scaled_thetaf - temp1);
    response.ok = true;

    return response;
}

DubinsResponse RSR(float scaled_theta0, float scaled_thetaf, float scaled_kmax) {
    DubinsResponse response;

    float invK = 1 / scaled_kmax;

    float C = cos(scaled_theta0) - cos(scaled_thetaf);
    float S = 2 * scaled_kmax - sin(scaled_theta0) + sin(scaled_thetaf);
    
    float temp1 = atan2(C, S);
    response.scaled_s1 = invK * mod2pi(scaled_theta0 - temp1);

    float temp2 = 2 + 4 * scaled_kmax * scaled_kmax - 2 * cos(scaled_theta0 - scaled_thetaf) - 4 * scaled_kmax * (sin(scaled_theta0) - sin(scaled_thetaf));
    if (temp2 < 0) {
        response.ok = false;
        response.scaled_s1 = 0;
        response.scaled_s2 = 0;
        response.scaled_s3 = 0;
        return response;
    }

    response.scaled_s2 = invK * sqrtf(temp2);
    response.scaled_s3 = invK * mod2pi(temp1 - scaled_thetaf);
    response.ok = true;

    return response;
}

DubinsResponse LSR(float scaled_theta0, float scaled_thetaf, float scaled_kmax) {
    DubinsResponse response;

    float invK = 1 / scaled_kmax;

    float C = cos(scaled_theta0) + cos(scaled_thetaf);
    float S = 2 * scaled_kmax + sin(scaled_theta0) + sin(scaled_thetaf);
    
    float temp1 = atan2(-C, S);
    
    float temp3 = 4 * scaled_kmax * scaled_kmax - 2 + 2 * cos(scaled_theta0 - scaled_thetaf) + 4 * scaled_kmax * (sin(scaled_theta0) + sin(scaled_thetaf));
    if (temp3 < 0) {
        response.ok = false;
        response.scaled_s1 = 0;
        response.scaled_s2 = 0;
        response.scaled_s3 = 0;
        return response;
    }

    response.scaled_s2 = invK * sqrtf(temp3);
    float temp2 = -atan2(-2, response.scaled_s2 * scaled_kmax);
    response.scaled_s1 = invK * mod2pi(temp1 + temp2 - scaled_theta0);
    response.scaled_s3 = invK * mod2pi(temp1 + temp2 - scaled_thetaf);
    response.ok = true;

    return response;
}

DubinsResponse RSL(float scaled_theta0, float scaled_thetaf, float scaled_kmax) {
    DubinsResponse response;

    float invK = 1 / scaled_kmax;

    float C = cos(scaled_theta0) + cos(scaled_thetaf);
    float S = 2 * scaled_kmax - sin(scaled_theta0) - sin(scaled_thetaf);
    
    float temp1 = atan2(C, S);
    
    float temp3 = 4 * scaled_kmax * scaled_kmax - 2 + 2 * cos(scaled_theta0 - scaled_thetaf) - 4 * scaled_kmax * (sin(scaled_theta0) + sin(scaled_thetaf));
    if (temp3 < 0) {
        response.ok = false;
        response.scaled_s1 = 0;
        response.scaled_s2 = 0;
        response.scaled_s3 = 0;
        return response;
    }

    response.scaled_s2 = invK * sqrtf(temp3);
    float temp2 = atan2(2, response.scaled_s2 * scaled_kmax);
    response.scaled_s1 = invK * mod2pi(scaled_theta0 - temp1 + temp2);
    response.scaled_s3 = invK * mod2pi(scaled_thetaf - temp1 + temp2);
    response.ok = true;

    return response;
}

DubinsResponse RLR(float scaled_theta0, float scaled_thetaf, float scaled_kmax) {
    DubinsResponse response;

    float invK = 1 / scaled_kmax;

    float C = cos(scaled_theta0) - cos(scaled_thetaf);
    float S = 2 * scaled_kmax - sin(scaled_theta0) + sin(scaled_thetaf);
    
    float temp1 = atan2(C, S);
    float temp2 = 0.125f * (6 - 4 * scaled_kmax * scaled_kmax + 2 * cos(scaled_theta0 - scaled_thetaf) + 4 * scaled_kmax * (sin(scaled_theta0) - sin(scaled_thetaf)));
    
    if (fabsf(temp2) > 1) {
        response.ok = false;
        response.scaled_s1 = 0;
        response.scaled_s2 = 0;
        response.scaled_s3 = 0;
        return response;
    }

    response.scaled_s2 = invK * mod2pi(2 * M_PI - acosf(temp2));
    response.scaled_s1 = invK * mod2pi(scaled_theta0 - temp1 + 0.5f * response.scaled_s2 * scaled_kmax);
    response.scaled_s3 = invK * mod2pi(scaled_theta0 - scaled_thetaf + scaled_kmax * (response.scaled_s2 - response.scaled_s1));
    response.ok = true;

    return response;
}

DubinsResponse LRL(float scaled_theta0, float scaled_thetaf, float scaled_kmax) {
    DubinsResponse response;

    float invK = 1 / scaled_kmax;

    float C = cos(scaled_thetaf) - cos(scaled_theta0);
    float S = 2 * scaled_kmax + sin(scaled_theta0) - sin(scaled_thetaf);
    
    float temp1 = atan2(C, S);
    float temp2 = 0.125f * (6 - 4 * scaled_kmax * scaled_kmax + 2 * cos(scaled_theta0 - scaled_thetaf) - 4 * scaled_kmax * (sin(scaled_theta0) - sin(scaled_thetaf)));
    
    if (fabsf(temp2) > 1) {
        response.ok = false;
        response.scaled_s1 = 0;
        response.scaled_s2 = 0;
        response.scaled_s3 = 0;
        return response;
    }

    response.scaled_s2 = invK * mod2pi(2 * M_PI - acosf(temp2));
    response.scaled_s1 = invK * mod2pi(temp1 - scaled_theta0 + 0.5f * response.scaled_s2 * scaled_kmax);
    response.scaled_s3 = invK * mod2pi(scaled_thetaf - scaled_theta0 + scaled_kmax * (response.scaled_s2 - response.scaled_s1));
    response.ok = true;

    return response;
}

// Common data shared by dubins_shortest_path and collision-free variant
static DubinsFunc dubins_funcs[6] = {LSL, RSR, LSR, RSL, RLR, LRL};
static float dubins_k_signs[6][3] = {
    {1, 0, 1},   // LSL
    {-1, 0, -1}, // RSR
    {1, 0, -1},  // LSR
    {-1, 0, 1},  // RSL
    {-1, 1, -1}, // RLR
    {1, -1, 1}   // LRL
};

float dubins_shortest_path(int &pidx, DubinsCurve &curve, Pose &start, Pose &end, float kmax) {
    // Compute params of standard scaled problem
    float scaled_theta0, scaled_thetaf, scaled_kmax, lambda;
    scaleToStandard(scaled_theta0, scaled_thetaf, scaled_kmax, lambda, start, end, kmax);

    // Try all the possible primitives, to find the optimal solution
    pidx = -1;
    float L = MAXFLOAT;
    float Lcur;
    float scaled_s1 = 0, scaled_s2 = 0, scaled_s3 = 0;
    float s1 = 0, s2 = 0, s3 = 0;

    for (int i = 0; i < 6; i++) {
        DubinsResponse res = dubins_funcs[i](scaled_theta0, scaled_thetaf, scaled_kmax);
        Lcur = res.scaled_s1 + res.scaled_s2 + res.scaled_s3;

        if (res.ok && Lcur < L) {
            L = Lcur;
            scaled_s1 = res.scaled_s1;
            scaled_s2 = res.scaled_s2;
            scaled_s3 = res.scaled_s3;
            pidx = i;
        }
    }

    if (pidx >= 0) {
        // Transform the solution to the problem in standard form to the
        // solution of the original problem (scale the lengths)
        scaleFromStandard(s1, s2, s3, scaled_s1, scaled_s2, scaled_s3, lambda);

        // Construct the Dubins curve with the computed optimal parameters
        curve = create_dubins_curve(start, s1, s2, s3,
                                    dubins_k_signs[pidx][0] * kmax,
                                    dubins_k_signs[pidx][1] * kmax,
                                    dubins_k_signs[pidx][2] * kmax);

        // Check the correctness of the algorithm
        if (!check(scaled_s1, scaled_s2, scaled_s3,
                   dubins_k_signs[pidx][0] * scaled_kmax,
                   dubins_k_signs[pidx][1] * scaled_kmax,
                   dubins_k_signs[pidx][2] * scaled_kmax,
                   scaled_theta0, scaled_thetaf)) {
            // Error
            pidx = -1;
            return MAXFLOAT;
        }

        return curve.total_length;  // Return actual length in meters
    }

    return MAXFLOAT;
}

float dubins_shortest_collision_free_path(
    int &pidx, DubinsCurve &curve, Pose &start, Pose &end, float kmax,
    const DubinsCollisionFn& collides) {

    // Compute params of standard scaled problem
    float scaled_theta0, scaled_thetaf, scaled_kmax, lambda;
    scaleToStandard(scaled_theta0, scaled_thetaf, scaled_kmax, lambda, start, end, kmax);

    // Collect all valid candidates sorted by length
    struct Candidate {
        int idx;
        float scaled_s1, scaled_s2, scaled_s3;
        float length;
    };
    Candidate candidates[6];
    int num_candidates = 0;

    for (int i = 0; i < 6; i++) {
        DubinsResponse res = dubins_funcs[i](scaled_theta0, scaled_thetaf, scaled_kmax);
        if (!res.ok) continue;

        float Lcur = res.scaled_s1 + res.scaled_s2 + res.scaled_s3;

        // Verify correctness
        if (!check(res.scaled_s1, res.scaled_s2, res.scaled_s3,
                   dubins_k_signs[i][0] * scaled_kmax,
                   dubins_k_signs[i][1] * scaled_kmax,
                   dubins_k_signs[i][2] * scaled_kmax,
                   scaled_theta0, scaled_thetaf)) {
            continue;
        }

        candidates[num_candidates++] = {i, res.scaled_s1, res.scaled_s2, res.scaled_s3, Lcur};
    }

    // Sort by length (insertion sort, max 6 elements)
    for (int i = 1; i < num_candidates; i++) {
        Candidate key = candidates[i];
        int j = i - 1;
        while (j >= 0 && candidates[j].length > key.length) {
            candidates[j + 1] = candidates[j];
            j--;
        }
        candidates[j + 1] = key;
    }

    // Try each candidate in order of length, return first collision-free one
    pidx = -1;
    for (int c = 0; c < num_candidates; c++) {
        auto& cand = candidates[c];
        float s1, s2, s3;
        scaleFromStandard(s1, s2, s3, cand.scaled_s1, cand.scaled_s2, cand.scaled_s3, lambda);

        DubinsCurve test_curve = create_dubins_curve(start, s1, s2, s3,
                                    dubins_k_signs[cand.idx][0] * kmax,
                                    dubins_k_signs[cand.idx][1] * kmax,
                                    dubins_k_signs[cand.idx][2] * kmax);

        if (!collides(test_curve)) {
            pidx = cand.idx;
            curve = test_curve;
            return test_curve.total_length;
        }
    }

    return MAXFLOAT;
}