#pragma once
#include <ceres/ceres.h>
#include <Eigen/Core>

/**
 * @brief 2D transformation error.
 *
 * This error is used to calculate the 2D transformation (R, p) between two matched image points p1 = (x1, y1) and p2 =
 * (x2, y2), the residual will be write as r = R * p1 + p - p2.
 *
 * - Residuals size is 2
 * - Input parameters block are 2D transformation pose = (R, p), R4
 */
class TransformationError : public ceres::SizedCostFunction<2, 4> {
  public:
    /**
     * @brief Constructor
     *
     * @param p1            Point 1
     * @param p2            Point 2
     * @param covariance    Image point covariance, [sigma_x^2, 0; 0, sigma_y^2]
     */
    TransformationError(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Matrix2d& covariance);

    /**
     * @brief Calculate the residuals and Jacobians of transformation error
     *
     * @param parameters  An array of pointers point to arrays containing the various parameters block, i.e., the 2D
     * transformation pose = (R, p), R4
     * @param residuals   Residuals, R2
     * @param jacobians   Jacobian blocks corresponding to each parameter block
     * @return Indicate whether the computation of the residuals and/or jacobians was successful or not
     */
    bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override;

  private:
    Eigen::Vector2d p1_;        // p1 = (x1, y1)
    Eigen::Vector2d p2_;        // p2 = (x2, y2)
    Eigen::Matrix2d sqrtInfo_;  // sqrt information matrix
};
