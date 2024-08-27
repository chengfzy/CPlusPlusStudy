#pragma once
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <vector>
#include "CatmullRomSpline.h"

class Optimizer {
  public:
    // curve fit
    const CatmullRomSpline& fit(const std::vector<Eigen::Vector2d>& observations);

    [[nodiscard]] const CatmullRomSpline& splineBeforeOpt() const { return splineBeforeOpt_; }
    [[nodiscard]] const CatmullRomSpline& spline() const { return spline_; }
    [[nodiscard]] inline const std::vector<Eigen::Vector2d>& estimatedPointsBeforeOpt() const {
        return estimatedPointsBeforeOpt_;
    }
    [[nodiscard]] inline const std::vector<Eigen::Vector2d>& estimatedPoints() const { return estimatedPoints_; }

  private:
    void initCtrlPoints();

    // calculate factor to each points
    std::vector<double> calculateFactor01();

    // calculate the min distance between each control points
    std::vector<double> calculateCtrlPointDist();

    void optimize();

    double calculateError(
        const CatmullRomSpline& spline,
        const std::vector<std::tuple<std::optional<double>, std::size_t, ceres::ResidualBlockId>>& params,
        std::vector<Eigen::Vector2d>& estimatedPoints, bool print = false);

  private:
    std::vector<Eigen::Vector2d> observations_;  // observation points
    CatmullRomSpline splineBeforeOpt_;           // Catmull-Rom spline before opt
    CatmullRomSpline spline_;                    // Catmull-Rom spline

    std::vector<Eigen::Vector2d> estimatedPointsBeforeOpt_;  // estimated points before opt, same size as observations_
    std::vector<Eigen::Vector2d> estimatedPoints_;           // estimated points after opt, same size as observations_
};