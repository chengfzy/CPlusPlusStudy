#pragma once
#include <ceres/ceres.h>

// normalize the angle in radius between [-pi, pi)
template <typename T>
inline T normalizeAngle(const T& angle) {
    // use ceres::floor because it is specialized for double and Jet types
    T twoPi{2.0 * M_PI};
    return angle - twoPi * ceres::floor((angle + T(M_PI)) / twoPi);
}
