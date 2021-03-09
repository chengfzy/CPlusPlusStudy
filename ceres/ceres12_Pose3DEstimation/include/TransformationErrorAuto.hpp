#pragma once
#include <ceres/ceres.h>
#include <Eigen/Core>

// Eigen's ostream operator is not compatible with ceres::Jet types.
// In particular, Eigen assumes that the scalar type (here Jet<T,N>) can be casted to an arithmetic type, which is not
// true for ceres::Jet. Unfortunatly, the ceres::Jet class does not define a conversion operator
// (http://en.cppreference.com/w/cpp/language/cast_operator).
//
// This workaround creates a template specilization for Eigen's cast_impl, when casting from a ceres::Jet type. It
// relies on Eigen's internal API and might break with future versions of Eigen.
namespace Eigen {
namespace internal {

template <class T, int N, typename NewType>
struct cast_impl<ceres::Jet<T, N>, NewType> {
    EIGEN_DEVICE_FUNC
    static inline NewType run(ceres::Jet<T, N> const& x) { return static_cast<NewType>(x.a); }
};

}  // namespace internal
}  // namespace Eigen

/**
 * @brief 3D transformation error using auto diff Jacobian
 *
 * This error is used to calculate the 3D transformation (R, p) between two matched image points p1 = (x1, y1, z1) and
 * p2 = (x2, y2, z2), the residual will be write as r = R * p1 + p - p2.
 *
 * - Residuals size is 3
 * - Input parameters block are 3D transformation pose = (R, p), R6, and where R in SO3, p in R3
 */
class TransformationErrorAuto {
  public:
    /**
     * @brief Constructor
     *
     * @param p1            Point 1
     * @param p2            Point 2
     * @param covariance    Image point covariance, [sigma_x^2, 0, 0; 0, sigma_y^2, 0, 0, 0, sigma_z^2]
     */
    TransformationErrorAuto(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Matrix3d& covariance)
        : p1_(p1), p2_(p2) {
        sqrtInfo_ = covariance.inverse().llt().matrixL();
    }

  public:
    /**
     * @brief Operator to evaluate the residual error based on parameters
     * @tparam T          Data type
     * @param pose        Data pointer point to 3D pose [R, p]
     * @param residuals   Data pointer point to the residual error
     * @return  True for calculation success, otherwise return false
     */
    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        // get the 3D transformation pose = (R, p), R7
        Eigen::Map<const Sophus::SO3<T>> r(pose);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p(pose + 4);

        // residual
        Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
        res = r * p1_ + p - p2_;
        // add information
        res = sqrtInfo_ * res;

        return true;
    }

    /**
     * @brief Create the cost funcdtion for transformation error
     *
     * @param p1            Point 1
     * @param p2            Point 2
     * @param covariance    Image point covariance, [sigma_x^2, 0, 0; 0, sigma_y^2, 0, 0, 0, sigma_z^2]
     * @return  Cost funcdtion for transformation error
     */
    static ceres::CostFunction* create(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
                                       const Eigen::Matrix3d& covariance) {
        return new ceres::AutoDiffCostFunction<TransformationErrorAuto, 3, 7>(
            new TransformationErrorAuto(p1, p2, covariance));
    }

  private:
    Eigen::Vector3d p1_;        // p1 = (x1, y1, z1)
    Eigen::Vector3d p2_;        // p2 = (x2, y2, z2)
    Eigen::Matrix3d sqrtInfo_;  // sqrt information matrix
};
