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
 * @brief 2D transformation error using auto diff Jacobians
 *
 * This error is used to calculate the 2D transformation (R, p) between two matched image points p1 = (x1, y1) and p2 =
 * (x2, y2), the residual will be write as r = R * p1 + p - p2.
 *
 * - Residuals size is 2
 * - Input parameters block are 2D transformation pose = (R, p), R4
 */
class TransformationErrorAuto {
  public:
    /**
     * @brief Constructor
     *
     * @param p1            Point 1
     * @param p2            Point 2
     * @param covariance    Image point covariance, [sigma_x^2, 0; 0, sigma_y^2]
     */
    TransformationErrorAuto(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Matrix2d& covariance)
        : p1_(p1), p2_(p2) {
        sqrtInfo_ = covariance.inverse().llt().matrixL();
    }

  public:
    /**
     * @brief Operator to evaluate the residual error based on parameters
     * @tparam T          Data type
     * @param pose        Data pointer point to 2D pose [R, p]
     * @param residuals   Data pointer point to the residual error
     * @return  True for calculation success, otherwise return false
     */
    template <typename T>
    bool operator()(const T* const pose, T* residuals) const {
        // get the 2D transformation pose = (R, p), R4
        Eigen::Map<const Sophus::SO2<T>> r(pose);
        Eigen::Map<const Eigen::Matrix<T, 2, 1>> p(pose + 2);

        // residual
        Eigen::Map<Eigen::Matrix<T, 2, 1>> res(residuals);
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
     * @param covariance    Image point covariance, [sigma_x^2, 0; 0, sigma_y^2]
     * @return  Cost funcdtion for transformation error
     */
    static ceres::CostFunction* create(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
                                       const Eigen::Matrix2d& covariance) {
        return new ceres::AutoDiffCostFunction<TransformationErrorAuto, 2, 4>(
            new TransformationErrorAuto(p1, p2, covariance));
    }

  private:
    Eigen::Vector2d p1_;        // p1 = (x1, y1, z1)
    Eigen::Vector2d p2_;        // p2 = (x2, y2, z2)
    Eigen::Matrix2d sqrtInfo_;  // sqrt information matrix
};
