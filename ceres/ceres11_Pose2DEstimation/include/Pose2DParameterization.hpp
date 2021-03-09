#pragma once
#include <ceres/ceres.h>
#include <sophus/so2.hpp>

/**
 * @brief Parameterization for 2D pose.
 *
 * Pose = [R, p], where R is SO2.
 *  R1 = R0 * Exp(phi)
 *  p1 = p0 + dP
 */
class Pose2DParameterization : public ceres::LocalParameterization {
  public:
    /**
     * @brief Generalization of the addition operation, x_plus_delta = Plus(x, delta)
     */
    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override {
        Eigen::Map<const Sophus::SO2d> r0(x);
        Eigen::Map<const Eigen::Vector2d> p0(x + 2);
        const double& phi = delta[0];
        Eigen::Map<const Eigen::Vector2d> dP(delta + 1);
        Eigen::Map<Sophus::SO2d> r1(x_plus_delta);
        Eigen::Map<Eigen::Vector2d> p1(x_plus_delta + 2);

        r1 = r0 * Sophus::SO2d::exp(phi);
        p1 = p0 + dP;

        return true;
    }

    /**
     * @brief  The jacobian of Plus(x, delta) w.r.t delta at delta = 0
     * @param jacobian  A row-major GlobalSize() x LocalSize() matrix
     */
    bool ComputeJacobian(const double* x, double* jacobian) const override {
        Eigen::Map<const Sophus::SO2d> r(x);
        Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> J(jacobian);

        // J = [D_(R*Exp(phi)) / D_Phi, 0; 0, I]
        J.setZero();
        J.block<2, 1>(0, 0) = r.Dx_this_mul_exp_x_at_0();
        J.block<2, 2>(2, 1).setIdentity();

        return true;
    }

    /**
     * @brief Size of x.
     */
    int GlobalSize() const override { return 4; };

    /**
     * @brief Size of delta
     */
    int LocalSize() const override { return 3; };
};
