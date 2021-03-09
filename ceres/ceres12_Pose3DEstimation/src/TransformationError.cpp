#include "TransformationError.h"
#include <sophus/so3.hpp>

using namespace std;
using namespace Eigen;
using namespace Sophus;

// Calculate the inverse Jacobian of q * Exp(phi) wrt. phi when phi->0, ie, D_phi / D(q * Exp(phi))
Eigen::Matrix<double, 3, 4> plusJacobianInv(const Eigen::Quaterniond& q) {
    Eigen::Matrix<double, 3, 4> J = Eigen::Matrix<double, 3, 4>::Zero();
    double w = 2 * q.w();
    double x = 2 * q.x();
    double y = 2 * q.y();
    double z = 2 * q.z();

    // clang-format off
    J(0, 0) =  w; J(0, 1) =  z; J(0, 2) = -y; J(0, 3) = -x;
    J(1, 0) = -z; J(1, 1) =  w; J(1, 2) =  x; J(1, 3) = -y;
    J(2, 0) =  y; J(2, 1) = -x; J(2, 2) =  w; J(2, 3) = -z;
    // clang-format on

    return J;
}

// Constructor
TransformationError::TransformationError(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                                         const Eigen::Matrix3d& covariance)
    : p1_(p1), p2_(p2) {
    sqrtInfo_ = covariance.inverse().llt().matrixL();
}

// Calculate the residuals and Jacobians of transformation error
bool TransformationError::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    // get the 3D transformation pose = (R, p), R7
    Map<const SO3d> r(parameters[0]);
    Eigen::Map<const Vector3d> p(parameters[0] + 4);

    // residual
    Map<Vector3d> res(residuals);
    res = r * p1_ + p - p2_;
    // add information
    res = sqrtInfo_ * res;

    // Jacobians
    if (jacobians != nullptr && jacobians[0] != nullptr) {
        // the Jacobians of res w.r.t. pose = [R, p], R3x7
        Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> J(jacobians[0]);
        J.setZero();

        J.block<3, 4>(0, 0) = -r.matrix() * SO3d::hat(p1_) * plusJacobianInv(r.unit_quaternion());  // J_res_R
        J.block<3, 3>(0, 4).setIdentity();
    }

    return true;
}
