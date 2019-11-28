#pragma once
#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

class PoseLocalParameterization : public ceres::LocalParameterization {
  public:
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const {
        // p1 = p0 + R0 * dp, R1 = R0 * Exp(dPhi)
        Eigen::Map<const Eigen::Vector3d> p0(x);
        Eigen::Map<const Sophus::SO3d> R0(x + 3);
        Eigen::Map<const Eigen::Vector3d> dp(delta);
        Eigen::Map<const Eigen::Vector3d> dPhi(delta + 3);
        Eigen::Map<Eigen::Vector3d> p1(x_plus_delta);
        Eigen::Map<Sophus::SO3d> R1(x_plus_delta + 3);

        p1 = p0 + R0 * dp;
        R1 = R0 * Sophus::SO3d::exp(dPhi);

        return true;
    }

    virtual bool ComputeJacobian(const double* x, double* jacobian) const {
        Eigen::Map<const Sophus::SE3d> T(x);
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobianMat(jacobian);
        jacobianMat = T.Dx_this_mul_exp_x_at_0();

        return true;
    }

    virtual int GlobalSize() const { return 7; }

    virtual int LocalSize() const { return 6; }
};
