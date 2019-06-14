#pragma once
#include "NormalizeAngle.h"
#include "ceres/ceres.h"

// define a local parameterization for upadting the angle to be constrained in [-pi, pi)
class AngleLocalParameterization {
   public:
    template <typename T>
    bool operator()(const T* theta, const T* deltaTheta, T* thetaPlusDelta) const {
        *thetaPlusDelta = normalizeAngle(*theta + *deltaTheta);
        return true;
    }

    static ceres::LocalParameterization* create() {
        return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
    }
};