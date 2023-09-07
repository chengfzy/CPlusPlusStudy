#pragma once
#include <ceres/ceres.h>
#include <sophus/so3.hpp>

/**
 * @brief Manifold for pose.
 *
 * Pose = [R, p], where R is SO3.
 *  R1 = R0 * Exp(phi)
 *  p1 = p0 + dP
 */
class PoseManifold : public ceres::Manifold {
  public:
    /**
     * @brief Generalization of the addition operation, x_plus_delta = Plus(x, delta)
     */
    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
        Eigen::Map<const Sophus::SO3d> r0(x);
        Eigen::Map<const Eigen::Vector3d> p0(x + 4);
        Eigen::Map<const Eigen::Vector3d> phi(delta);
        Eigen::Map<const Eigen::Vector3d> dP(delta + 3);
        Eigen::Map<Sophus::SO3d> r1(x_plus_delta);
        Eigen::Map<Eigen::Vector3d> p1(x_plus_delta + 4);

        r1 = r0 * Sophus::SO3d::exp(phi);
        p1 = p0 + dP;

        return true;
    }

    /**
     * @brief  The jacobian of Plus(x, delta) w.r.t delta at delta = 0
     * @param jacobian  A row-major GlobalSize() x LocalSize() matrix
     */
    bool PlusJacobian(const double* x, double* jacobian) const override {
        Eigen::Map<const Sophus::SO3d> r(x);
        Eigen::Map<const Eigen::Vector3d> p(x + 4);
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jacobian);

        // J = [D_(R*Exp(phi)) / D_Phi, 0; 0, I]
        J.setZero();
        J.block<4, 3>(0, 0) = r.Dx_this_mul_exp_x_at_0();
        J.block<3, 3>(4, 3).setIdentity();

        return true;
    }

    virtual bool Minus(const double* y, const double* x, double* y_minus_x) const override {
        Eigen::Map<const Sophus::SO3d> ry(y);
        Eigen::Map<const Eigen::Vector3d> py(y + 4);
        Eigen::Map<const Sophus::SO3d> rx(x);
        Eigen::Map<const Eigen::Vector3d> px(x + 4);
        Eigen::Map<Eigen::Vector3d> dR(y_minus_x);
        Eigen::Map<Eigen::Vector3d> dP(y_minus_x + 3);

        dR = (rx.inverse() * ry).log();
        dP = py - px;

        return true;
    };

    virtual bool MinusJacobian(const double* x, double* jacobian) const override {
        Eigen::Map<const Sophus::SO3d> r(x);
        Eigen::Map<const Eigen::Vector3d> p(x + 4);
        Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J(jacobian);

        J.setZero();
        J.block<3, 4>(0, 0) = r.Dx_log_this_inv_by_x_at_this();
        J.block<3, 3>(3, 4).setIdentity();

        return true;
    }

    /**
     * @brief Size of x.
     */
    int AmbientSize() const override { return 7; };

    /**
     * @brief Size of delta
     */
    int TangentSize() const override { return 6; };
};