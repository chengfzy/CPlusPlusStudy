#include <ceres/ceres.h>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <Eigen/Core>
#include <common/common.hpp>
#include <iostream>
#include <sophus/se2.hpp>
#include <tuple>
#include "AngleManifold.hpp"
#include "Pose2DManifold.hpp"
#include "TransformationError.h"
#include "TransformationErrorAngle.hpp"
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
void generateData(MatrixX2d& pointsA, MatrixX2d& pointsB) {
    cout << Section("Generate Simulated Data");
    // set the rotation angle and translation position
    double theta{20. / 180. * M_PI};
    Vector2d trans(5.123, 7.345);
    Sophus::SO2d R(theta);
    cout << format("theta = {:.5f} deg, translation = {}, rotation = \n{}", theta / M_PI * 180, trans.transpose(),
                   R.matrix())
         << endl;

    // set the points A
    pointsA.resize(7, 2);
    pointsA << 0, 0, 100., 100., 100., 200., 100., 300., 200., 500., 200., 700., 200., 800.;

    // generate points B, pB = R * pA + p
    pointsB.resize(pointsA.rows(), 2);
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
void optimizePose(const MatrixX2d& pointsA, const MatrixX2d& pointsB) {
    cout << Section("Optimization using Analytical Jacobians");
    ceres::Problem problem;
    // pose
    Sophus::SE2d pose;
    auto pose2DManifold = new Pose2DManifold();
    problem.AddParameterBlock(pose.data(), 4, pose2DManifold);
    cout << format("before optimization, theta = {:.5f} deg, p = [{}]", pose.so2().log() / M_PI * 180.,
                   pose.translation().transpose())
         << endl;

    Matrix2d cov = Matrix2d::Identity();
    // add residuals
    vector<tuple<Vector2d, Vector2d, ceres::ResidualBlockId>> resBlocks;  // <lastPoint, point, blockID>
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
        Vector2d lastPoint = get<0>(v);
        Vector2d point = get<1>(v);
        double cost{0};
        Vector2d res = Vector2d::Zero();
        problem.EvaluateResidualBlock(get<2>(v), true, &cost, res.data(), nullptr);
        cout << format("last point = [{}], point = [{}], cost = {:.5f}, res = [{}]", lastPoint.transpose(),
                       point.transpose(), cost, res.transpose())
             << endl;
    }

    // print final result
    cout << format("after optimization, theta = {:.5f} deg, p = [{}]", pose.so2().log() / M_PI * 180.,
                   pose.translation().transpose())
         << endl;
}

/**
 * @brief Optimize with auto Jacobians
 * @param pointsA   Points A
 * @param pointsB   Points B
 */
void optimizeAuto(const MatrixX2d& pointsA, const MatrixX2d& pointsB) {
    cout << Section("Optimization using Auto Jacobians");
    ceres::Problem problem;
    // pose
    Sophus::SE2d pose;
    auto pose2DManifold = new Pose2DManifold();
    problem.AddParameterBlock(pose.data(), 4, pose2DManifold);
    cout << format("before optimization, theta = {:.5f} deg, p = [{}]", pose.so2().log() / M_PI * 180.,
                   pose.translation().transpose())
         << endl;

    Matrix2d cov = Matrix2d::Identity();
    // add residuals
    vector<tuple<Vector2d, Vector2d, ceres::ResidualBlockId>> resBlocks;  // <lastPoint, point, blockID>
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
        Vector2d lastPoint = get<0>(v);
        Vector2d point = get<1>(v);
        double cost{0};
        Vector2d res = Vector2d::Zero();
        problem.EvaluateResidualBlock(get<2>(v), true, &cost, res.data(), nullptr);
        cout << format("last point = [{}], point = [{}], cost = {:.5f}, res = [{}]", lastPoint.transpose(),
                       point.transpose(), cost, res.transpose())
             << endl;
    }

    // print final result
    cout << format("after optimization, theta = {:.5f} deg, p = [{}]", pose.so2().log() / M_PI * 180.,
                   pose.translation().transpose())
         << endl;
}

/**
 * @brief Optimize with auto Jacobians and angle manifold
 * @param pointsA   Points A
 * @param pointsB   Points B
 */
void optimizeAngle(const MatrixX2d& pointsA, const MatrixX2d& pointsB) {
    cout << Section("Optimization using Auto Jacobians and Angle Manifold");
    ceres::Problem problem;
    // pose
    double theta{0};
    Vector2d trans = Vector2d::Zero();
    auto angleManifold = AngleManifold::create();
    problem.AddParameterBlock(&theta, 1, angleManifold);
    cout << format("before optimization, theta = {:.5f} deg, p = [{}]", theta / M_PI * 180., trans.transpose()) << endl;

    Matrix2d cov = Matrix2d::Identity();
    // add residuals
    vector<tuple<Vector2d, Vector2d, ceres::ResidualBlockId>> resBlocks;  // <lastPoint, point, blockID>
    for (size_t i = 0; i < pointsA.rows(); ++i) {
        auto id = problem.AddResidualBlock(TransformationErrorAngle::create(pointsA.row(i), pointsB.row(i), cov),
                                           nullptr, &theta, trans.data());
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
        Vector2d lastPoint = get<0>(v);
        Vector2d point = get<1>(v);
        double cost{0};
        Vector2d res = Vector2d::Zero();
        problem.EvaluateResidualBlock(get<2>(v), true, &cost, res.data(), nullptr);
        cout << format("last point = [{}], point = [{}], cost = {:.5f}, res = [{}]", lastPoint.transpose(),
                       point.transpose(), cost, res.transpose())
             << endl;
    }

    // print final result
    cout << format("after optimization, theta = {:.5f} deg, p = [{}]", theta / M_PI * 180., trans.transpose()) << endl;
}

int main(int argc, char* argv[]) {
    // generate the simulated data
    MatrixX2d pointsA, pointsB;
    generateData(pointsA, pointsB);

    // optimize
    optimizePose(pointsA, pointsB);
    optimizeAuto(pointsA, pointsB);
    optimizeAngle(pointsA, pointsB);

    return 0;
}