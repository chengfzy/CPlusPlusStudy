#pragma once
#include "Eigen/Core"
#include "ceres/ceres.h"
#include "types.h"

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

// Compute the error term for two poses that have a relative pose measurement between them.
// Let the hat variables be the measurement, we have two poses x_a and x_b, and through sensor measurements we can
// measurement the measure the transformation of frame B wrt frame A denotes as hat(t_ab).
//
// We have chosen to the rigid transformation as a Hamiltonian quaternion q, and position p, the quaternion ordering is
// [x, y, z, w]. The estimated measurements is
//      t_ab = [ p_ab ] = [ R(q_a)^T * (p_b - p_a) ]
//             [ q_ab ] = [ q_a^{-1} * q_b         ]
//
// and
//
// residual = information^{1/2} * [ p_ab - hat(p_ab)                ]
//                                [ 2.0 * Vec(q_ab * hat(q_ab)^{-1} ]
//
// where Vec(*) returns the vector(imaginary) part of the quaternion
class PoseGraph3DErrorTerm {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PoseGraph3DErrorTerm(const Pose3d& t_ab, const Eigen::Matrix<double, 6, 6>& sqrtInformation)
        : t_ab_(t_ab), sqrtInfo_(sqrtInformation) {}

    template <typename T>
    bool operator()(const T* const p_a, const T* const p_b, T* residual) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pA(p_a);
        Eigen::Map<const Sophus::SO3<T>> rA(p_a + 3);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> pB(p_b);
        Eigen::Map<const Sophus::SO3<T>> rB(p_b + 3);

        using namespace std;
        cout << "pA = " << pA.transpose() << endl;
        std::cout << "rA = " << rA.params().transpose() << std::endl;

        // compute the relative transformation between the two frames
        Eigen::Matrix<T, 3, 1> pAB = rA.inverse() * (pB - pA);
        Sophus::SO3<T> rAB = rA.inverse() * rB;

        // compute the residuals
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residualMat(residual);
        residualMat.template block<3, 1>(0, 0) = pAB - t_ab_.p.template cast<T>();
        residualMat.template block<3, 1>(3, 0) = (rAB * t_ab_.r.template cast<T>().inverse()).log();

        // scale the residuals by the measurement uncertainty
        residualMat.applyOnTheLeft(sqrtInfo_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* create(const Pose3d& t_ab, const Eigen::Matrix<double, 6, 6>& sqrtInformation) {
        return (new ceres::AutoDiffCostFunction<PoseGraph3DErrorTerm, 6, 7, 7>(
            new PoseGraph3DErrorTerm(t_ab, sqrtInformation)));
    }

   private:
    const Pose3d t_ab_;                           // the measurement for the position of B relative to A in the frame A
    const Eigen::Matrix<double, 6, 6> sqrtInfo_;  // the square root of the measurement information matrix
};