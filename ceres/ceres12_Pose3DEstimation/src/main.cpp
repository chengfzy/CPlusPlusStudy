#include <ceres/ceres.h>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <Eigen/Core>
#include <common/common.hpp>
#include <iostream>
#include <sophus/se3.hpp>
#include <tuple>
#include "PoseManifold.hpp"
#include "TransformationError.h"
#include "TransformationErrorAuto.hpp"

using namespace std;
using namespace fmt;
using namespace Eigen;
using namespace common;

/**
 * @brief Generate simulated data
 *
 * @param pointsA   Points A
 * @param pointsB   Points B
 */
void generateData(MatrixX3d& pointsA, MatrixX3d& pointsB) {
    cout << Section("Generate Simulated Data");
    // set the rotation angle and translation position
    Vector3d theta(20., 30, 40);
    theta = theta / 180. * M_PI;
    Vector3d trans(5.123, 7.345, 8.363);
    Sophus::SO3d R = Sophus::SO3d::exp(theta);
    cout << format("theta = {} deg, translation = {}, rotation = \n{}", theta.transpose() / M_PI * 180,
                   trans.transpose(), R.matrix())
         << endl;

    // set the points A
    pointsA.resize(7, 3);
    pointsA << 0, 0, 0, 100., 100., 400, 100., 200., 500, 100., 300., 600, 200., 500., 700, 200., 700., 800, 200., 800.,
        900;

    // generate points B, pB = R * pA + p
    pointsB.resize(pointsA.rows(), 3);
    for (size_t i = 0; i < pointsA.rows(); ++i) {
        pointsB.row(i) = R * pointsA.row(i).transpose() + trans;
    }

    // print out points A and points B
    cout << format("pointsA = \n{}", pointsA) << endl;
    cout << format("pointsB = \n{}", pointsB) << endl;
}

/**
 * @brief Optimize with analytical Jacobians
 *
 * @param pointsA   Points A
 * @param pointsB   Points B
 */
void optimizePose(const MatrixX3d& pointsA, const MatrixX3d& pointsB) {
    cout << Section("Optimization using Analytical Jacobians");
    ceres::Problem problem;
    // pose
    Sophus::SE3d pose;
    auto poseManifold = new PoseManifold();
    problem.AddParameterBlock(pose.data(), 7, poseManifold);
    cout << format("before optimization, angle = [{}] deg, p = [{}]", pose.so3().log().transpose() / M_PI * 180.,
                   pose.translation().transpose())
         << endl;

    Matrix3d cov = Matrix3d::Identity();
    // add residuals
    vector<tuple<Vector3d, Vector3d, ceres::ResidualBlockId>> resBlocks;  // <lastPoint, point, blockID>
    for (size_t i = 0; i < pointsA.rows(); ++i) {
        auto id = problem.AddResidualBlock(new TransformationError(pointsA.row(i), pointsB.row(i), cov), nullptr,
                                           pose.data());
        resBlocks.emplace_back(make_tuple(pointsA.row(i), pointsB.row(i), id));
    }

    // solve options
    ceres::Solver::Options options;
    options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
    options.minimizer_progress_to_stdout = true;

    // solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;

    // print each residual of each matched point pair
    for (auto& v : resBlocks) {
        Vector3d lastPoint = get<0>(v);
        Vector3d point = get<1>(v);
        double cost{0};
        Vector3d res = Vector3d::Zero();
        problem.EvaluateResidualBlock(get<2>(v), true, &cost, res.data(), nullptr);
        cout << format("last point = [{}], point = [{}], cost = {:.5f}, res = [{}]", lastPoint.transpose(),
                       point.transpose(), cost, res.transpose())
             << endl;
    }

    // print final result
    cout << format("after optimization, angle = [{}] deg, p = [{}]", pose.so3().log().transpose() / M_PI * 180.,
                   pose.translation().transpose())
         << endl;
}

/**
 * @brief Optimize with auto Jacobians
 *
 * @param pointsA   Points A
 * @param pointsB   Points B
 */
void optimizeAuto(const MatrixX3d& pointsA, const MatrixX3d& pointsB) {
    cout << Section("Optimization using Auto Jacobians");
    ceres::Problem problem;
    // pose
    Sophus::SE3d pose;
    auto poseManifold = new PoseManifold();
    problem.AddParameterBlock(pose.data(), 7, poseManifold);
    cout << format("before optimization, angle = [{}] deg, p = [{}]", pose.so3().log().transpose() / M_PI * 180.,
                   pose.translation().transpose())
         << endl;

    Matrix3d cov = Matrix3d::Identity();
    // add residuals
    vector<tuple<Vector3d, Vector3d, ceres::ResidualBlockId>> resBlocks;  // <lastPoint, point, blockID>
    for (size_t i = 0; i < pointsA.rows(); ++i) {
        auto id = problem.AddResidualBlock(TransformationErrorAuto::create(pointsA.row(i), pointsB.row(i), cov),
                                           nullptr, pose.data());
        resBlocks.emplace_back(make_tuple(pointsA.row(i), pointsB.row(i), id));
    }

    // solve options
    ceres::Solver::Options options;
    options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
    options.minimizer_progress_to_stdout = true;

    // solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;

    // print each residual of each matched point pair
    for (auto& v : resBlocks) {
        Vector3d lastPoint = get<0>(v);
        Vector3d point = get<1>(v);
        double cost{0};
        Vector3d res = Vector3d::Zero();
        problem.EvaluateResidualBlock(get<2>(v), true, &cost, res.data(), nullptr);
        cout << format("last point = [{}], point = [{}], cost = {:.5f}, res = [{}]", lastPoint.transpose(),
                       point.transpose(), cost, res.transpose())
             << endl;
    }

    // print final result
    cout << format("after optimization, angle = [{}] deg, p = [{}]", pose.so3().log().transpose() / M_PI * 180.,
                   pose.translation().transpose())
         << endl;
}

int main(int argc, char* argv[]) {
    // generate the simulated data
    MatrixX3d pointsA, pointsB;
    generateData(pointsA, pointsB);

    // STEP 2, optimize
    optimizePose(pointsA, pointsB);
    optimizeAuto(pointsA, pointsB);

    return 0;
}