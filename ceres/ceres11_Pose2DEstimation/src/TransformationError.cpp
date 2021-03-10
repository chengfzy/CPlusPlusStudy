#include "TransformationError.h"
#include <sophus/so2.hpp>

using namespace std;
using namespace Eigen;
using namespace Sophus;

// Constructor
TransformationError::TransformationError(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                                         const Eigen::Matrix2d& covariance)
    : p1_(p1), p2_(p2) {
    sqrtInfo_ = covariance.inverse().llt().matrixL();
}

// Calculate the residuals and Jacobians of transformation error
bool TransformationError::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    // get the 2D transformation pose = (R, p), R4
    Map<const SO2d> r(parameters[0]);
    Eigen::Map<const Vector2d> p(parameters[0] + 2);

    // residual
    Map<Vector2d> res(residuals);
    res = r * p1_ + p - p2_;
    // add information
    res = sqrtInfo_ * res;

    // Jacobians
    if (jacobians != nullptr) {
        if (jacobians[0] != nullptr) {
            // the Jacobians of res w.r.t. pose = [R, p], R2x4
            Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jacobians[0]);
            J.setZero();
            // J_res_R, unit_complex() = [cos(theta), sin(theta)]
            J.block<2, 2>(0, 0) = r.matrix() * Sophus::SO2d::hat(1) * p1_ *
                                  Eigen::Vector2d(-r.unit_complex()[1], r.unit_complex()[0]).transpose();
            // const auto& c = r.unit_complex()[0];
            // const auto& s = r.unit_complex()[1];
            // double a1 = s * p1_[0] + c * p1_[1];
            // double a2 = c * p1_[0] - s * p1_[1];
            // J(0, 0) = a1 * s;
            // J(0, 1) = -a1 * c;
            // J(1, 0) = -a2 * s;
            // J(1, 1) = a2 * c;

            // J_res_p
            J.block<2, 2>(0, 2).setIdentity();
        }
    }

    return true;
}
