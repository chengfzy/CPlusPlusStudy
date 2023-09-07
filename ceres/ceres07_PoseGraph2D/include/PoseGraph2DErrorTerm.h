#pragma once
#include <Eigen/Core>
#include "NormalizeAngle.h"

// Compute the rotation matrix from yaw angle
template <typename T>
Eigen::Matrix<T, 2, 2> rotationMatrix2d(const T& yaw) {
    const T cosYaw = cos(yaw);
    const T sinYaw = sin(yaw);
    Eigen::Matrix<T, 2, 2> rot;
    rot << cosYaw, -sinYaw, sinYaw, cosYaw;
    return rot;
}

// Compute the error term for two poses that have a relative pose measurement between them.
// Let the hat variables be the measurement.
//
// residual = information^{1/2} * [ R_a^T * (pb - pa) - hat(p_ab)         ]
//                                [ Normalize(yaw_b - yaw_a - hat(yaw_ab) ]
//
// where R_a is the rotation matrix that rotates a vector represented in frame A into the global frame, and Normalize(*)
// ensures the angle are in the range [-pi, pi)
class PoseGraph2DErrorTerm {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PoseGraph2DErrorTerm(const double& x_ab, const double& y_ab, const double& yaw_ab,
                         const Eigen::Matrix3d& sqrtInformation)
        : p_ab_(x_ab, y_ab), yaw_ab_(yaw_ab), sqrtInfo_(sqrtInformation) {}

    template <typename T>
    bool operator()(const T* const x_a, const T* const y_a, const T* const yaw_a, const T* const x_b,
                    const T* const y_b, const T* const yaw_b, T* residual) const {
        const Eigen::Matrix<T, 2, 1> p_a(*x_a, *y_a);
        const Eigen::Matrix<T, 2, 1> p_b(*x_b, *y_b);

        Eigen::Map<Eigen::Matrix<T, 3, 1>> residualsMap(residual);
        residualsMap.template head<2>() = rotationMatrix2d(*yaw_a).transpose() * (p_b - p_a) - p_ab_.cast<T>();
        residualsMap(2) = normalizeAngle(*yaw_b - *yaw_a - static_cast<T>(yaw_ab_));

        // scale the residuals by the square root information matrix to account for the measurement uncertainty
        residualsMap = sqrtInfo_.cast<T>() * residualsMap;

        return true;
    }

    static ceres::CostFunction* create(const double& x_ab, const double& y_ab, const double& yaw_ab,
                                       const Eigen::Matrix3d& sqrtInformation) {
        return (new ceres::AutoDiffCostFunction<PoseGraph2DErrorTerm, 3, 1, 1, 1, 1, 1, 1>(
            new PoseGraph2DErrorTerm(x_ab, y_ab, yaw_ab, sqrtInformation)));
    }

  private:
    const Eigen::Vector2d p_ab_;      // the position of B relative to A in the frame A
    const double yaw_ab_;             // the orientation of frame B relative to frame A
    const Eigen::Matrix3d sqrtInfo_;  // the inverse square root of the measurement covariance matrix
};