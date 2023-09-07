#pragma once
#include <ceres/ceres.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

class PoseManifold : public ceres::Manifold {
  public:
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
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

    virtual bool PlusJacobian(const double* x, double* jacobian) const override {
        Eigen::Map<const Eigen::Vector3d> p(x);
        Eigen::Map<const Sophus::SO3d> R(x + 3);
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobianMat(jacobian);
        jacobianMat.setZero();
        jacobianMat.block<3, 3>(0, 0).setIdentity();
        jacobianMat.block<4, 3>(3, 3) = R.Dx_this_mul_exp_x_at_0();

        return true;
    }

    virtual bool Minus(const double* y, const double* x, double* y_minus_x) const override {
        // Eigen::Map<const Sophus::SE3d> Ty(y);
        // Eigen::Map<const Sophus::SE3d> Tx(x);
        // Eigen::Map<Eigen::Matrix<double, 6, 1>> delta(y_minus_x);
        // delta = (Tx.inverse() * Ty).log();

        return true;
    }

    virtual bool MinusJacobian(const double* x, double* jacobian) const override {
        // Eigen::Map<const Sophus::SE3d> Tx(x);
        // Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> jacobianMat(jacobian);
        // jacobianMat = Tx.Dx_log_this_inv_by_x_at_this();

        return true;
    }

    virtual int AmbientSize() const override { return 7; }
    virtual int TangentSize() const override { return 6; }
};
