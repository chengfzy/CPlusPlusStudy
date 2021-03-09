#pragma once
#include <ceres/ceres.h>

/**
 * @brief Define a local parameterization for updating the angle to be constrained in [-pi, pi)
 */
class AngleParameterization {
  public:
    /**
     * @brief Operator to calculate theta + delta theta
     *
     * @tparam T                Data type
     * @param theta             Pointer point to theta
     * @param deltaTheta        Pointer point to delta theta
     * @param thetaPlusDelta    Pointer point to the plus result
     * @return True fo calculate success, otherwise return false
     */
    template <typename T>
    bool operator()(const T* theta, const T* deltaTheta, T* thetaPlusDelta) const {
        // normalize the angle between [-pi, pi)
        T twoPi(2.0 * M_PI);
        *thetaPlusDelta = twoPi * ceres::floor((*theta + *deltaTheta + T(M_PI)) / twoPi);
        return true;
    }

    /**
     * @brief Get the local parameterization pointer
     *
     * @return  Local parameterization pointer
     */
    static ceres::LocalParameterization* create() {
        return (new ceres::AutoDiffLocalParameterization<AngleParameterization, 1, 1>);
    }
};