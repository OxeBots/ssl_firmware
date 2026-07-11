#ifndef HELPER_FUNC_H
#define HELPER_FUNC_H

#include <cmath>

/**
 * @brief Helper to normalize angle to [-PI, PI]
 * @param angle in radians
 * @return normalized angle in radians
 */
inline double normalize_angle(double angle)
{
    return angle - ((2.0f * M_PI) * std::floor((angle + M_PI) / (2.0f * M_PI)));
}

/**
 * @brief Helper to constrain a value between a min and max
 * @param val Value to constrain
 * @param min_val Minimum value
 * @param max_val Maximum value
 * @return Constrained value
 */
inline double constrain(const double val, const double min_val, const double max_val)
{
    return std::min(std::max(val, min_val), max_val);
}

#endif  // HELPER_FUNC_H
