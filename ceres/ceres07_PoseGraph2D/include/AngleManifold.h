#pragma once
#include <ceres/ceres.h>
#include "NormalizeAngle.h"

// define a local manifold for updating the angle to be constrained in [-pi, pi)
class AngleManifold {
  public:
    template <typename T>
    bool Plus(const T* theta, const T* deltaTheta, T* thetaPlusDelta) const {
        *thetaPlusDelta = normalizeAngle(*theta + *deltaTheta);
        return true;
    }

    template <typename T>
    bool Minus(const T* y, const T* x, T* yMinusX) const {
        *yMinusX = normalizeAngle(*y) - normalizeAngle(*x);
        return true;
    }

    static ceres::Manifold* create() { return new ceres::AutoDiffManifold<AngleManifold, 1, 1>; }
};