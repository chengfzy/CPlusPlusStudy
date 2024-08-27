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

    // delete control point manually
    // for (auto& v : {17, 15, 4, 13, 12, 10}) {
    //     spline_.ctrlPoints_.erase(spline_.ctrlPoints_.begin() + v);
    // }

    splineBeforeOpt_ = spline_;
    optimize();

    return spline_;
}

void Optimizer::initCtrlPoints() {
    // auto factors = calculateFactor01();
    auto distances = calculateCtrlPointDist();

    constexpr double kEndPointThresh{3.0};  // 首尾最后一段的控制点最小间距 [m]

    int method{3};
    if (method == 0) {
        // 使用计算的阈值进行判断
        Eigen::Vector2d lastPoint = Eigen::Vector2d::Zero();
        for (std::size_t i = 0U; i < observations_.size(); ++i) {
            if (i == 0) {
                spline_.ctrlPoints_.emplace_back(observations_[i]);
                lastPoint = observations_[i];
            } else {
                const double& thresh = distances[i];
                double dist = (lastPoint - observations_[i]).norm();
                if (dist > thresh /* || (2 * distances[i - 1] < thresh && dist > distances[i - 1]) */) {
                    LOG(INFO) << fmt::format("[{}] add control point, distance: {:.3f}", i, distances[i]);
                    spline_.ctrlPoints_.emplace_back(observations_[i]);
                    lastPoint = observations_[i];
                } else if ((spline_.ctrlPoints().size() == 1 || i == observations_.size() - 1) &&
                           dist > kEndPointThresh) {
                    // add second or last control points
                    LOG(INFO) << fmt::format("[{}] add control point, distance: {:.3f}", i, distances[i]);
                    spline_.ctrlPoints_.emplace_back(observations_[i]);
                    lastPoint = observations_[i];
                }
            }
        }
    } else if (method == 1) {
        // 使用计算的阈值进行判断, 并且使用区域内阈值的最小值
        Eigen::Vector2d lastPoint = Eigen::Vector2d::Zero();
        double thresh = std::numeric_limits<double>::max();
        for (std::size_t i = 0U; i < observations_.size(); ++i) {
            if (i == 0) {
                spline_.ctrlPoints_.emplace_back(observations_[i]);
                lastPoint = observations_[i];
            } else {
                thresh = std::min(distances[i], thresh);
                double dist = (lastPoint - observations_[i]).norm();
                if (dist > thresh) {
                    LOG(INFO) << fmt::format("[{}] add control point, distance: {:.3f}", i, distances[i]);
                    spline_.ctrlPoints_.emplace_back(observations_[i]);
                    lastPoint = observations_[i];
                    thresh = std::numeric_limits<double>::max();
                } else if ((spline_.ctrlPoints().size() == 1 || i == observations_.size() - 1) &&
                           dist > kEndPointThresh) {
                    // add second or last control points
                    LOG(INFO) << fmt::format("[{}] add control point, distance: {:.3f}", i, distances[i]);
                    spline_.ctrlPoints_.emplace_back(observations_[i]);
                    lastPoint = observations_[i];
                    thresh = std::numeric_limits<double>::max();
                }
            }
        }
    } else if (method == 2) {
        // 使用前面两个控制点阈值的最小值来判断
        Eigen::Vector2d lastPoint = Eigen::Vector2d::Zero();
        double lastThresh1{std::numeric_limits<double>::max()};  // last thresh
        double lastThresh2{std::numeric_limits<double>::max()};  // last last thresh
        for (std::size_t i = 0U; i < observations_.size(); ++i) {
            if (i == 0) {
                spline_.ctrlPoints_.emplace_back(observations_[i]);
                lastPoint = observations_[i];
            } else {
                double thresh = std::min({distances[i], lastThresh1, lastThresh2});
                double dist = (lastPoint - observations_[i]).norm();
                if (dist > thresh) {
                    LOG(INFO) << fmt::format("[{}] add control point, distance: {:.3f}", i, distances[i]);
                    spline_.ctrlPoints_.emplace_back(observations_[i]);
                    lastPoint = observations_[i];
                    lastThresh2 = lastThresh1;
                    lastThresh1 = distances[i];
                } else if ((spline_.ctrlPoints().size() == 1 || i == observations_.size() - 1) &&
                           dist > kEndPointThresh) {
                    // add second or last control points
                    LOG(INFO) << fmt::format("[{}] add control point, distance: {:.3f}", i, distances[i]);
                    spline_.ctrlPoints_.emplace_back(observations_[i]);
                    lastPoint = observations_[i];
                    lastThresh2 = lastThresh1;
                    lastThresh1 = distances[i];
                }
            }
        }
    } else if (method == 3) {
        // 使用前面两个控制点阈值的最小值来判断, 并添加首尾控制点, 避免首尾段不能加入约束
        // 总体误差会变大, 但集中在首尾, 符合预期
        Eigen::Vector2d lastPoint = Eigen::Vector2d::Zero();
        double lastThresh1{std::numeric_limits<double>::max()};  // last thresh
        double lastThresh2{std::numeric_limits<double>::max()};  // last last thresh
        for (std::size_t i = 0U; i < observations_.size(); ++i) {
            if (i == 0) {
                spline_.ctrlPoints_.emplace_back(observations_[i]);
                lastPoint = observations_[i];
            } else {
                double thresh = std::min({distances[i], lastThresh1, lastThresh2});
                double dist = (lastPoint - observations_[i]).norm();
                if (dist > thresh) {
                    LOG(INFO) << fmt::format("[{}] add control point, distance: {:.3f}", i, distances[i]);
                    spline_.ctrlPoints_.emplace_back(observations_[i]);
                    lastPoint = observations_[i];
                    lastThresh2 = lastThresh1;
                    lastThresh1 = distances[i];
                } else if (i == observations_.size() - 1 && dist > kEndPointThresh) {
                    // add second or last control points
                    LOG(INFO) << fmt::format("[{}] add control point, distance: {:.3f}", i, distances[i]);
                    spline_.ctrlPoints_.emplace_back(observations_[i]);
                    lastPoint = observations_[i];
                    lastThresh2 = lastThresh1;
                    lastThresh1 = distances[i];
                }
            }
        }

        // 在首尾添加额外的控制点
        if (spline_.ctrlPoints().size() > 1) {
            spline_.ctrlPoints_.insert(spline_.ctrlPoints_.begin(),
                                       2 * spline_.ctrlPoints_[0] - spline_.ctrlPoints_[1]);
            spline_.ctrlPoints_.emplace_back(2 * spline_.ctrlPoints_.back() -
                                             spline_.ctrlPoints_[spline_.ctrlPoints().size() - 2]);
        }
    } else if (method == 4) {  // 根据p'(0) = tau * (p2 - p1), 即计算切向量才计算距离, 不行
        std::optional<Eigen::Vector2d> p1;
        std::optional<Eigen::Vector2d> p2;
        for (std::size_t i = 0U; i < observations_.size(); ++i) {
            if (i == 0) {
                spline_.ctrlPoints_.emplace_back(observations_[i]);
                p2 = observations_[i];
            } else {
                double thresh = 2 * distances[i];
                double dist{0};
                if (spline_.ctrlPoints_.size() <= 3) {
                    dist = 2 * (p2.value() - observations_[i]).norm();
                    LOG(INFO) << fmt::format("[{}] dist: {:.3f}, thresh: {:.3f}", i, dist, thresh);
                } else if (p1) {
                    dist = (p1.value() - observations_[i]).norm();
                    LOG(INFO) << fmt::format("[{}] dist: {:.3f}, thresh: {:.3f}", i, dist, thresh);
                } else {
                    dist = (p2.value() - observations_[i]).norm();
                    LOG(INFO) << fmt::format("[{}] dist: {:.3f}, thresh: {:.3f}", i, dist, thresh);
                }

                if (dist > thresh) {
                    spline_.ctrlPoints_.emplace_back(observations_[i]);
                    p1 = p2;
                    p2 = observations_[i];
                    LOG(INFO) << fmt::format("add control point, p1: {::.3f}, p2: {::.3f}", p1.value(), p2.value());
                } else if ((spline_.ctrlPoints().size() == 1 && dist > kEndPointThresh) ||
                           i == observations_.size() - 1) {
                    if (dist > kEndPointThresh) {
                        // add second or last control points
                        spline_.ctrlPoints_.emplace_back(observations_[i]);
                        p1 = p2;
                        p2 = observations_[i];
                        LOG(INFO) << fmt::format("add control point, p1: {::.3f}, p2: {::.3f}", p1.value(), p2.value());
                    }
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

std::vector<double> Optimizer::calculateCtrlPointDist() {
    // calculate the min distance between each ctrl points, based on the curvature(radius)
    std::vector<double> distances(observations_.size(), 1.0);
    for (std::size_t i = 1U; i + 1 < observations_.size(); ++i) {
        auto& pA = observations_[i - 1];
        auto& pB = observations_[i];
        auto& pC = observations_[i + 1];
        double AB = (pA - pB).norm();
        double BC = (pC - pB).norm();
        double CA = (pC - pA).norm();
        double s = 0.5 * (AB + BC + CA);
        double area = std::sqrt(s * (s - AB) * (s - BC) * (s - CA));
        double radius = AB * BC * CA / 4 / area;
        // 直线时比较远, 弯道较密, R <=> distance: [50.0 - 20.0 - 5.0]  <=> [50 - 10 - 1.0]
        // 距离适中, R <=> distance: [20.0 - 10.0 - 3.0]  <=> [30 - 10 - 1.0]
        // 直线不太远， 弯道适中, R <=> distance: [30.0 - 10.0 - 5.0]  <=> [25 - 10 - 1.0]
        constexpr double kRadius0 = 30.0;
        constexpr double kRadius1 = 20.0;
        constexpr double kRadius2 = 5.0;
        constexpr double kDistance0{25.0};
        constexpr double kDistance1{10.0};
        constexpr double kDistance2{1.0};
        double dist{0.0};

        if (radius >= kRadius0) {
            dist = kDistance0;
        } else if (kRadius1 < radius && radius <= kRadius0) {
            double ratio = (radius - kRadius0) / (kRadius1 - kRadius0);
            dist = kDistance0 * (1 - ratio) + kDistance1 * ratio;
        } else if (kRadius2 < radius && radius <= kRadius1) {
            double ratio = (radius - kRadius1) / (kRadius2 - kRadius1);
            dist = kDistance1 * (1 - ratio) + kDistance2 * ratio;
        } else {
            dist = kDistance2;
        }
        distances[i] = dist;

        LOG(INFO) << fmt::format("[{}] radius: {:.3f} m, dist: {:.3f}", i, radius, dist);
    }

    // first and last
    if (!distances.empty()) {
        distances[0] = distances[1];
        if (distances.size() > 2) {
            distances.back() = distances[distances.size() - 2];
        }
    }
    LOG(INFO) << fmt::format("min control points distance: {}", distances);

    // filter, using the min one, average
    std::vector<double> filteredDistance = distances;
    constexpr int kHalfWinSize{10};
    for (std::size_t i = 0U; i < distances.size(); ++i) {
        int i0 = std::max(0, static_cast<int>(i) - kHalfWinSize);
        int i1 = std::min(static_cast<int>(i) + kHalfWinSize + 1, static_cast<int>(distances.size()));
        auto itMin = std::min_element(distances.begin() + i0, distances.begin() + i1);
        filteredDistance[i] = *itMin;
    }

    LOG(INFO) << fmt::format("after average, min control points distance: {}", filteredDistance);

    return filteredDistance;
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
    for (std::size_t i = 0U; i < observations_.size(); ++i) {
        auto& obs = observations_[i];

        // find u, index of first control point(P0)
        std::size_t firstCtrlPointIndex{0U};
        auto u = spline_.findParamU(obs, firstCtrlPointIndex);
        ceres::ResidualBlockId blockId{nullptr};

        if (u) {
            Eigen::Vector2d est = spline_.evaluateAt(u.value(), firstCtrlPointIndex);
            Eigen::Vector2d error = est - obs;
            LOG(INFO) << fmt::format(
                "[{}/{}] obs: {::.5f}, estimated point: {::.5f}, error: {::.5f}, |error|: {:.5f}, control point idx: {}"
                ", u: {:.5f}",
                i, observations_.size(), obs, est, error, error.norm(), firstCtrlPointIndex, u.value());
            // add spline factor
            Eigen::Vector4d matU(1, u.value(), u.value() * u.value(), u.value() * u.value() * u.value());
            Eigen::Vector4d splineCoeff = matU.transpose() * matTau;
            auto* factor = new CatmullRomSplineFactor(obs, splineCoeff);
            blockId = problem.AddResidualBlock(
                factor, nullptr, ctrlPoints[firstCtrlPointIndex].data(), ctrlPoints[firstCtrlPointIndex + 1].data(),
                ctrlPoints[firstCtrlPointIndex + 2].data(), ctrlPoints[firstCtrlPointIndex + 3].data());
        } else {
            LOG(WARNING) << fmt::format("[{}/{}] obs: {::.5f}, idx: {}", i, observations_.size(), obs,
                                        firstCtrlPointIndex);
        }

        params.emplace_back(std::make_tuple(u, firstCtrlPointIndex, blockId));
    }

    // set first control point fixed, 加不加不影响最终结果
    // problem.SetParameterBlockConstant(ctrlPoints[1].data());

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
    std::vector<double> errors;
    for (std::size_t i = 0U; i < params.size(); ++i) {
        auto u = std::get<0>(params[i]);
        if (u.has_value()) {
            auto index = std::get<1>(params[i]);
            ceres::ResidualBlockId blockId = std::get<2>(params[i]);
            Eigen::Vector2d est = spline.evaluateAt(u.value(), index);
            estimatedPoints[i] = est;
            Eigen::Vector2d error = est - observations_[i];
            errors.emplace_back(error.norm());
            sumError += error.squaredNorm();
            LOG_IF(INFO, print) << fmt::format("[{}/{}] obs: {::.5f}, est: {::.5f}, error: {::.5f}, |error|: {:.5f}", i,
                                               params.size(), observations_[i], est, error, error.norm());

            // ceres cost = 0.5 * error.squaredNorm();
            // double cost{0};
            // Eigen::Vector2d residual = Eigen::Vector2d::Zero();
            // problem.EvaluateResidualBlock(blockId, false, &cost, residual.data(), nullptr);
            // LOG(INFO) << fmt::format("\tresidual: {::.5f}, cost: {:.5f}", residual, cost);
        }
    }
    LOG(INFO) << fmt::format("average error: {:.5f}",
                             std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size());

    return std::sqrt(sumError);
}