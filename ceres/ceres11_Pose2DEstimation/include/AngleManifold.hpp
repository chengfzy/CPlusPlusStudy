#pragma once
#include <ceres/ceres.h>

/**
 * @brief Define a manifold for updating the angle to be constrained in [-pi, pi)
 */
class AngleManifold {
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
    bool Plus(const T* theta, const T* deltaTheta, T* thetaPlusDelta) const {
        // normalize the angle between [-pi, pi)
        T twoPi(2.0 * M_PI);
        T r = *theta + *deltaTheta;
        *thetaPlusDelta = r - twoPi * ceres::floor((r + T(M_PI)) / twoPi);
        return true;
    }

    template <typename T>
    bool Minus(const T* y, const T* x, T* y_minus_x) const {
        // normalize the angle between [-pi, pi)
        T twoPi(2.0 * M_PI);
        T r = *y - *x;
        *y_minus_x = r - twoPi * ceres::floor((r + T(M_PI)) / twoPi);
        return true;
    }

    /**
     * @brief Get the manifold pointer
     *
     * @return  Manifold pointer
     */
    static ceres::Manifold* create() { return new ceres::AutoDiffManifold<AngleManifold, 1, 1>; }
};