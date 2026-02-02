#include "math_utils.hpp"
#include <cmath>

float sinc(float x) {
    if (fabs(x) < 2e-3) {
        return 1 - x*x / 6 + pow(x, 4) / 120;
    }

    return sin(x) / x;
}

float mod2pi(float angle) {
    while (angle < 0) {
        angle += 2 * M_PI;
    }
    while (angle >= 2 * M_PI) {
        angle -= 2 * M_PI;
    }

    return angle;
}

float rangeSymm(float angle) {
    while (angle <= -M_PI) {
        angle += 2 * M_PI;
    }
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }

    return angle;
}