#ifndef MATH_UTILS_CPP
#define MATH_UTILS_CPP

/**
 * Sinc function, defined as sin(x)/x, with a Taylor series expansion around 0
 * to avoid numerical instability for small x.
 * @param x The input value
 * @return The sinc of the input value
 */
float sinc(float x);

/**
 * Normalize an angle to the range [0, 2*pi)
 * @param angle The angle to normalize (in radians)
 * @return The normalized angle within the range [0, 2*pi)
 */
float mod2pi(float angle);

/**
 * Normalize an angular difference to the range [-pi, pi]
 * @param angle The angle to normalize (in radians)
 * @return The normalized angle within the range [-pi, pi]
 */
float rangeSymm(float angle);

#endif // MATH_UTILS_CPP