#pragma once
#include <ceres/ceres.h>
#include <Eigen/Core>

/**
 * @brief 点到Catmull-Rom曲线的残差
 * 1. 优化变量: 控制点p0, p1, p2, p3, 4个R2向量
 * 2. 点到曲线的残差: R2
 */
class CatmullRomSplineFactor : public ceres::SizedCostFunction<2, 2, 2, 2, 2> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    CatmullRomSplineFactor(const Eigen::Vector2d& obs, const Eigen::Vector4d& coeff, const double& noise = 1.0)
        : obs_(obs), coeff_(coeff), sqrtInfo_(std::sqrt(noise)) {}

    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        double x = coeff_[0] * parameters[0][0] + coeff_[1] * parameters[1][0] + coeff_[2] * parameters[2][0] +
                   coeff_[3] * parameters[3][0];
        double y = coeff_[0] * parameters[0][1] + coeff_[1] * parameters[1][1] + coeff_[2] * parameters[2][1] +
                   coeff_[3] * parameters[3][1];
        residuals[0] = sqrtInfo_ * (x - obs_[0]);
        residuals[1] = sqrtInfo_ * (y - obs_[1]);

        if (jacobians) {
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> jac_p0(jacobians[0]);
                jac_p0.setZero();
                jac_p0(0, 0) = coeff_[0];  // dr[0]/dp[0]
                jac_p0(1, 1) = coeff_[0];  // dr[1]/dp[1]
                jac_p0 = sqrtInfo_ * jac_p0;
            }

            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> jac_p1(jacobians[1]);
                jac_p1.setZero();
                jac_p1(0, 0) = coeff_[1];  // dr[0]/dp[0]
                jac_p1(1, 1) = coeff_[1];  // dr[1]/dp[1]
                jac_p1 = sqrtInfo_ * jac_p1;
            }

            if (jacobians[2]) {
                Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> jac_p2(jacobians[2]);
                jac_p2.setZero();
                jac_p2(0, 0) = coeff_[2];  // dr[0]/dp[0]
                jac_p2(1, 1) = coeff_[2];  // dr[1]/dp[1]
                jac_p2 = sqrtInfo_ * jac_p2;
            }

            if (jacobians[3]) {
                Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor>> jac_p3(jacobians[3]);
                jac_p3.setZero();
                jac_p3(0, 0) = coeff_[3];  // dr[0]/dp[0]
                jac_p3(1, 1) = coeff_[3];  // dr[1]/dp[1]
                jac_p3 = sqrtInfo_ * jac_p3;
            }
        }

        return true;
    }

  private:
    Eigen::Vector2d obs_ = Eigen::Vector2d::Zero();    // observation
    Eigen::Vector4d coeff_ = Eigen::Vector4d::Zero();  // spline coeff, = U^T * M
    double sqrtInfo_{1.0};                             // sqrt information matrix
};