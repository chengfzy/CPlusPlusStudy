#include "Optimizer.h"
#include <fmt/ranges.h>
#include <optional>
#include <tuple>
#include "CatmullRomSplineFactor.hpp"
#include "common//common.hpp"

const CatmullRomSpline& Optimizer::fit(const std::vector<Eigen::Vector2d>& observations) {
    // save observation
    observations_ = observations;

    // init control points
    initCtrlPoints();

    splineBeforeOpt_ = spline_;
    optimize();

    return spline_;
}

void Optimizer::initCtrlPoints() {
    // auto factors = calculateFactor01();
    auto factors = calculateFactor02();

    constexpr double kDistThresh{0.5};  // dist threshold to init control points
    LOG(INFO) << fmt::format("init control points, thresh: {:.5f}", kDistThresh);

    Eigen::Vector2d lastPoint = Eigen::Vector2d::Zero();
    for (std::size_t i = 0U; i < observations_.size(); ++i) {
        if (i == 0) {
            spline_.ctrlPoints_.emplace_back(observations_[i]);
            lastPoint = observations_[i];
        } else {
            double thresh = kDistThresh * factors[i];
            double dist = (lastPoint - observations_[i]).norm();
            if (dist > thresh) {
                spline_.ctrlPoints_.emplace_back(observations_[i]);
                lastPoint = observations_[i];
            } else if (spline_.ctrlPoints().size() == 1 || i == observations_.size() - 1) {
                if (dist > thresh * 0.5) {
                    // add second or last control points
                    spline_.ctrlPoints_.emplace_back(observations_[i]);
                    lastPoint = observations_[i];
                }
            }
        }
    }
    LOG(INFO) << fmt::format("init control points, size: {}", spline_.ctrlPoints().size());
}

std::vector<double> Optimizer::calculateFactor01() {
    // calculate factors to scale the distance thresh for initializing control points, using the angle between each
    // points to represent the curvature
    std::vector<double> factors(observations_.size(), 1.0);
    for (std::size_t i = 1U; i + 1 < observations_.size(); ++i) {
        auto& pA = observations_[i - 1];
        auto& pB = observations_[i];
        auto& pC = observations_[i + 1];
        Eigen::Vector2d BA = pA - pB;
        Eigen::Vector2d BC = pC - pB;
        double angle = std::abs(std::acos(BA.dot(BC) / (BA.norm() * BC.norm())));
        // [30 - 170] deg <=> [0.1 - 1.0], nan for 180.
        constexpr double kAngle0 = 60.0 / 180. * M_PI;
        constexpr double kAngle1 = 170.0 / 180. * M_PI;
        constexpr double kFactor0{0.1};
        constexpr double kFactor1{1.0};
        double factor{0.0};
        if (kAngle0 < angle && angle < kAngle1) {
            double ratio = (angle - kAngle0) / (kAngle1 - kAngle0);
            factor = kFactor0 * ratio + kFactor1 * (1 - ratio);
        } else if (angle <= kAngle0) {
            factor = kFactor0;
        } else {
            factor = kFactor1;
        }
        factors[i] = factor;
        LOG(INFO) << fmt::format("angle: {:.5f}, factor: {:.5f}, angle0/angle1 = [{:.5f}, {:.5f}]", angle, factor,
                                 kAngle0, kAngle1);
    }

    // first and last
    if (!factors.empty()) {
        factors[0] = factors[1];
        if (factors.size() > 2) {
            factors.back() = factors[factors.size() - 2];
        }
    }
    LOG(INFO) << fmt::format("control points factors: {}", factors);

    return factors;
}

std::vector<double> Optimizer::calculateFactor02() {
    // calculate factors to scale the distance thresh for initializing control points, based on the curvature(radius)
    std::vector<double> factors(observations_.size(), 1.0);
    for (std::size_t i = 1U; i + 1 < observations_.size(); ++i) {
        auto& pA = observations_[i - 1];
        auto& pB = observations_[i];
        auto& pC = observations_[i + 1];
        double AB = (pA - pB).norm();
        double BC = (pC - pB).norm();
        double CA = (pC - pA).norm();
        double s = 0.5 * (AB + BC + CA);
        double area = std::sqrt(s * (s - AB) * (s - BC) * (s - CA));
        double curvature = 4 * area / (AB * BC * CA);
        // R = [2.0 - 0.5]  <=> [1.0 - 0.25]
        constexpr double kCurvature0 = 1.0 / 2.0;
        constexpr double kCurvature1 = 1.0 / 0.5;
        constexpr double kFactor0{1.0};
        constexpr double kFactor1{0.25};
        double factor{0.0};
        if (kCurvature0 < curvature && curvature < kCurvature1) {
            double ratio = (curvature - kCurvature0) / (kCurvature1 - kCurvature0);
            factor = kFactor0 * (1 - ratio) + kFactor1 * ratio;
        } else if (curvature <= kCurvature0) {
            factor = kFactor0;
        } else {
            factor = kFactor1;
        }
        factors[i] = factor;

        LOG(INFO) << fmt::format("curvature: {:.5f}, factor: {:.5f}, curvature0/curvature1 = [{:.5f}, {:.5f}]",
                                 curvature, factor, kCurvature0, kCurvature1);
    }

    // first and last
    if (!factors.empty()) {
        factors[0] = factors[1];
        if (factors.size() > 2) {
            factors.back() = factors[factors.size() - 2];
        }
    }
    LOG(INFO) << fmt::format("control points factors: {}", factors);

    // average
    std::vector<double> avgFactors = factors;
    constexpr std::size_t kHalfWinSize{3};
    constexpr std::size_t kWindSize{2 * kHalfWinSize + 1};
    for (std::size_t i = kHalfWinSize; i + kWindSize < factors.size(); ++i) {
        auto itMin = std::min_element(factors.begin() + i, factors.begin() + i + kWindSize);
        avgFactors[i] = *itMin;
    }
    LOG(INFO) << fmt::format("after average, control points factors: {}", avgFactors);

    return avgFactors;
}

void Optimizer::optimize() {
    // init matrix tau
    Eigen::Matrix4d matTau = Eigen::Matrix4d::Zero();
    auto& tau = spline_.tau();
    matTau << 0, 1, 0, 0, -tau, 0, tau, 0, 2 * tau, tau - 3, 3 - 2 * tau, -tau, -tau, 2 - tau, tau - 2, tau;

    auto& ctrlPoints = spline_.ctrlPoints();
    // sample points before calculate U
    spline_.samplePointForFindU(20);

    // save temp data for each observation, <u, index of first control point(P0), residual block id>
    std::vector<std::tuple<std::optional<double>, std::size_t, ceres::ResidualBlockId>> params;

    // build problem
    ceres::Problem problem;
    for (auto& obs : observations_) {
        // find u, index of first control point(P0)
        std::size_t firstCtrlPointIndex{0U};
        auto u = spline_.findParamU(obs, firstCtrlPointIndex);
        ceres::ResidualBlockId blockId{nullptr};

        if (u) {
            Eigen::Vector2d est = spline_.evaluateAt(u.value(), firstCtrlPointIndex);
            Eigen::Vector2d error = est - obs;
            LOG(INFO) << fmt::format(
                "obs: {::.5f}, estimated point: {::.5f}, error: {::.5f}, |error|: {:.5f}, control point idx: {}, u: "
                "{:.5f}",
                obs, est, error, error.norm(), firstCtrlPointIndex, u.value());
            // add spline factor
            Eigen::Vector4d matU(1, u.value(), u.value() * u.value(), u.value() * u.value() * u.value());
            Eigen::Vector4d splineCoeff = matU.transpose() * matTau;
            auto* factor = new CatmullRomSplineFactor(obs, splineCoeff);
            blockId = problem.AddResidualBlock(
                factor, nullptr, ctrlPoints[firstCtrlPointIndex].data(), ctrlPoints[firstCtrlPointIndex + 1].data(),
                ctrlPoints[firstCtrlPointIndex + 2].data(), ctrlPoints[firstCtrlPointIndex + 3].data());
        } else {
            LOG(WARNING) << fmt::format("obs: {::.5f}, idx: {}", obs, firstCtrlPointIndex);
        }

        params.emplace_back(std::make_tuple(u, firstCtrlPointIndex, blockId));
    }

    // set first control point fixed
    problem.SetParameterBlockConstant(ctrlPoints.front().data());

    // calculate error before optimization
    double errorBeforeOpt = calculateError(spline_, params, estimatedPointsBeforeOpt_, true);
    LOG(INFO) << fmt::format("error before optimization: {:.5f}", errorBeforeOpt);

    // solve options
    ceres::Solver::Options options;
    options.minimizer_type = ceres::MinimizerType::TRUST_REGION;
    options.minimizer_progress_to_stdout = true;
    // solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    LOG(INFO) << summary.FullReport();

    // calculate error after optimization
    double errorAfterOpt = calculateError(spline_, params, estimatedPoints_);

    LOG(INFO) << fmt::format("error before/after optimization: {:.5f}/{:.5f}", errorBeforeOpt, errorAfterOpt);
}

double Optimizer::calculateError(
    const CatmullRomSpline& spline,
    const std::vector<std::tuple<std::optional<double>, std::size_t, ceres::ResidualBlockId>>& params,
    std::vector<Eigen::Vector2d>& estimatedPoints, bool print) {
    // calculate error for each block
    double sumError{0.0};
    estimatedPoints.resize(observations_.size(), Eigen::Vector2d::Zero());
    for (std::size_t i = 0U; i < params.size(); ++i) {
        auto u = std::get<0>(params[i]);
        if (u.has_value()) {
            auto index = std::get<1>(params[i]);
            ceres::ResidualBlockId blockId = std::get<2>(params[i]);
            Eigen::Vector2d est = spline.evaluateAt(u.value(), index);
            estimatedPoints[i] = est;
            Eigen::Vector2d error = est - observations_[i];
            sumError += error.squaredNorm();
            LOG_IF(INFO, print) << fmt::format("[{}] obs: {::.5f}, est: {::.5f}, error: {::.5f}, |error|: {:.5f}", i,
                                               observations_[i], est, error, error.norm());

            // ceres cost = 0.5 * error.squaredNorm();
            // double cost{0};
            // Eigen::Vector2d residual = Eigen::Vector2d::Zero();
            // problem.EvaluateResidualBlock(blockId, false, &cost, residual.data(), nullptr);
            // LOG(INFO) << fmt::format("\tresidual: {::.5f}, cost: {:.5f}", residual, cost);
        }
    }

    return std::sqrt(sumError);
}