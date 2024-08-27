#include "CatmullRomSpline.h"
#include <fmt/format.h>
#include <common/common.hpp>
#include <numeric>

// using namespace std;
// using namespace Eigen;

CatmullRomSpline::CatmullRomSpline(const double& tau) : tau_(tau) {
    matTau_ << 0, 1, 0, 0, -tau_, 0, tau_, 0, 2 * tau_, tau_ - 3, 3 - 2 * tau_, -tau_, -tau_, 2 - tau_, tau_ - 2, tau_;
}

void CatmullRomSpline::setCtrlPoints(const std::vector<Eigen::Vector2d>& ctrlPoints) { ctrlPoints_ = ctrlPoints; }

Eigen::Vector2d CatmullRomSpline::evaluateAt(const double& u, const std::size_t& firstCtrlPointIndex) const {
    Eigen::Vector4d matU(1, u, u * u, u * u * u);
    Eigen::Matrix<double, 4, 2> matCtrlPoints = Eigen::Matrix<double, 4, 2>::Zero();
    for (std::size_t i = 0; i < 4; ++i) {
        matCtrlPoints.row(static_cast<Eigen::Index>(i)) = ctrlPoints_[firstCtrlPointIndex + i].transpose();
    }
    return matU.transpose() * matTau_ * matCtrlPoints;
}

std::vector<Eigen::Vector2d> CatmullRomSpline::samplePoint(int num) const {
    std::vector<Eigen::Vector2d> points;

    // sample u and calculate matrix U
    Eigen::VectorXd sampledU = Eigen::VectorXd::LinSpaced(num, 0.0, 1.0);
    std::vector<Eigen::Vector4d> allMatU;
    allMatU.reserve(sampledU.size() - 1);  // 不包括最后一个点, 避免多个segment时前后点重复
    for (int i = 0; i < sampledU.size() - 1; ++i) {
        allMatU.emplace_back(
            Eigen::Vector4d(1, sampledU[i], sampledU[i] * sampledU[i], sampledU[i] * sampledU[i] * sampledU[i]));
    }

    for (std::size_t m = 0U; m < ctrlPoints_.size(); ++m) {
        if (m + 3U >= ctrlPoints_.size()) {
            break;
        }

        Eigen::Matrix<double, 4, 2> matCtrlPoints = Eigen::Matrix<double, 4, 2>::Zero();
        for (std::size_t i = 0; i < 4; ++i) {
            matCtrlPoints.row(static_cast<Eigen::Index>(i)) = ctrlPoints_[m + i].transpose();
        }

        for (auto& v : allMatU) {
            points.emplace_back(v.transpose() * matTau_ * matCtrlPoints);
        }
    }

    return points;
}

void CatmullRomSpline::samplePointForFindU(int num) {
    std::vector<Eigen::Vector2d> points;
    sampledPoints_.clear();

    int numDuringSegment = num - 1;  // 中间片段(除开最后一个片段)的点数

    // sample u and calculate matrix U
    Eigen::VectorXd sampledU = Eigen::VectorXd::LinSpaced(num, 0.0, 1.0);
    std::vector<Eigen::Vector4d> allMatU;
    allMatU.reserve(num);  // 不包括最后一个点, 避免多个segment时前后点重复
    for (int i = 0; i < num; ++i) {
        allMatU.emplace_back(
            Eigen::Vector4d(1, sampledU[i], sampledU[i] * sampledU[i], sampledU[i] * sampledU[i] * sampledU[i]));
    }

    for (std::size_t m = 0U; m < ctrlPoints_.size(); ++m) {
        if (m + 3U >= ctrlPoints_.size()) {
            break;
        }

        Eigen::Matrix<double, 4, 2> matCtrlPoints = Eigen::Matrix<double, 4, 2>::Zero();
        for (std::size_t i = 0; i < 4; ++i) {
            matCtrlPoints.row(static_cast<Eigen::Index>(i)) = ctrlPoints_[m + i].transpose();
        }

        double calNum = m + 4U == ctrlPoints_.size() ? num : numDuringSegment;
        for (std::size_t n = 0U; n < calNum; ++n) {
            sampledPoints_.emplace_back(SampledPoint{
                .u = sampledU[n], .value = allMatU[n].transpose() * matTau_ * matCtrlPoints, .ctrlPointIndex = m});
        }
    }
}

std::optional<double> CatmullRomSpline::findParamU(const Eigen::Vector2d& pt, std::size_t& firstCtrlPointIndex) const {
    // find the top 2 nearest sampled points of pt
    double dist0 = std::numeric_limits<double>::max();  // nearest
    double dist1 = std::numeric_limits<double>::max();  // top2 nearest
    std::size_t tempIdx0{0U}, tempIdx1{0U};
    for (std::size_t i = 0U; i < sampledPoints_.size(); ++i) {
        double dist = (sampledPoints_[i].value - pt).norm();
        if (dist < dist0) {
            dist1 = dist0;
            dist0 = dist;
            tempIdx1 = tempIdx0;
            tempIdx0 = i;
        } else if (dist < dist1) {
            dist1 = dist;
            tempIdx1 = i;
        }
    }
    std::size_t idx1 = std::min(tempIdx0, tempIdx1);  // index of P1
    std::size_t idx2 = std::max(tempIdx0, tempIdx1);  // index of P2

    // check pt is in the middle of P1P2
    const Eigen::Vector2d& p1 = sampledPoints_[idx1].value;
    const Eigen::Vector2d& p2 = sampledPoints_[idx2].value;
    auto pp1 = p1 - pt;
    auto pp2 = p2 - pt;
    if (pp2.dot(pp1) / pp2.norm() / pp1.norm() > std::cos(20.0 / 180.0 * M_PI)) {
        // pt is not in the middle of P1P2
        double temp = pp2.dot(pp1) / pp2.norm() / pp1.norm();
        return std::nullopt;
    }

    // ensure idx1 and idx2 is in the same segment
    const double& u1 = sampledPoints_[idx1].u;
    double u2 = sampledPoints_[idx2].u;
    firstCtrlPointIndex = sampledPoints_[idx1].ctrlPointIndex;
    if (sampledPoints_[idx1].ctrlPointIndex != sampledPoints_[idx2].ctrlPointIndex) {
        u2 += 1.0;  // usual is 0.0 + 1.0 = 1.0
        // firstCtrlPointIndex = sampledPoints_[idx2].ctrlPointIndex;
        // --firstCtrlPointIndex;
    }

    Eigen::Vector2d p21 = p2 - p1;
    Eigen::Vector2d pA1 = pt - p1;
    double ratio = p21.dot(pA1) / p21.squaredNorm();
    double u = u1 * (1 - ratio) + u2 * ratio;
    return u;
}

std::vector<std::size_t> CatmullRomSpline::FindTwoNearestCtrlPoints(const Eigen::Vector2d& pt) const {
    auto idx = FindTwoNearestPoints(pt, ctrlPoints_);
    std::size_t idx0 = idx[0], idx1 = idx[1];

    // check pt is the middle of P1P2
    Eigen::Vector2d p1 = ctrlPoints_[idx0];
    Eigen::Vector2d p2 = ctrlPoints_[idx1];
    Eigen::Vector2d pp2 = p2 - pt;
    Eigen::Vector2d pp1 = p1 - pt;
    if (pp2.dot(pp1) < 0) {  // pt is in the middle of P1P2
        // bypass observation in first and last segment
        if (idx0 == 0 || idx1 == ctrlPoints_.size() - 1U) {
            return {};
        }
        return {idx0, idx1};
    }

    if (idx0 != 0) {
        // check pt is in the middle of P0P1
        Eigen::Vector2d p0 = ctrlPoints_[idx0 - 1];
        Eigen::Vector2d pp0 = p0 - pt;
        if (pp1.dot(pp0) < 0) {
            // pt is in the middle of P0P1
            idx1 = idx[0];
            if (idx1 == 0) {
                return {};
            } else {
                idx0 = idx1 - 1;
            }
        }
    }

    if (pp2.dot(pp1) > 0) {
        // pt is not in the middle to P1P2
        Eigen::Vector2d p0 = ctrlPoints_[idx0 - 1];
        Eigen::Vector2d p3 = ctrlPoints_[idx1 + 1];
        Eigen::Vector2d pp0 = p0 - pt;
        Eigen::Vector2d pp3 = p3 - pt;
        if (pp1.dot(pp0) < 0) {
            // pt is in the middle of P0P1
            idx1 = idx[0];
            if (idx1 == 0) {
                return {};
            } else {
                idx0 = idx1 - 1;
            }
        } else if (pp2.dot(pp3) < 0) {
            // pt is in the middle of P2P3
            idx0 = idx[1];
            if (idx0 == ctrlPoints_.size() - 1) {
                return {};
            } else {
                idx1 = idx0 + 1;
            }
        }
    }

    // bypass observation in first and last segment
    if (idx0 == 0 || idx1 == ctrlPoints_.size() - 1U) {
        return {};
    }

    return {idx0, idx1};
}

std::vector<std::size_t> CatmullRomSpline::FindTwoNearestPoints(const Eigen::Vector2d& pt,
                                                                const std::vector<Eigen::Vector2d>& points) const {
    double dist0 = std::numeric_limits<double>::max();  // nearest
    double dist1 = std::numeric_limits<double>::max();  // top2 nearest
    std::size_t tempIdx0{0U}, tempIdx1{0U};

    for (std::size_t i = 0U; i < points.size(); ++i) {
        double dist = (points[i] - pt).norm();
        if (dist < dist0) {
            dist1 = dist0;
            dist0 = dist;
            tempIdx1 = tempIdx0;
            tempIdx0 = i;
        } else if (dist < dist1) {
            dist1 = dist;
            tempIdx1 = i;
        }
    }

    std::size_t idx0 = std::min(tempIdx0, tempIdx1);
    std::size_t idx1 = std::max(tempIdx0, tempIdx1);
    return {idx0, idx1};
}
