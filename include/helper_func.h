#ifndef HELPER_FUNC_H
#define HELPER_FUNC_H

#include <cmath>

/**
 * @brief Helper to normalize angle to [-PI, PI]
 * @param angle in radians
 * @return normalized angle in radians
 */
inline float normalize_angle(float angle) { return angle - (2.0f * M_PI) * std::floor((angle + M_PI) / (2.0f * M_PI)); }

#endif  // HELPER_FUNC_H
