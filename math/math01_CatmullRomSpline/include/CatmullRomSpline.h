#pragma once
#include <Eigen/Core>
#include <array>
#include <optional>
#include <vector>

/**
 * @brief Catmull-Rom Spline
 *
 * Ref:
 *  [1] Qiao et al., “Online Monocular Lane Mapping Using Catmull-Rom Spline.”
 *
 */
class CatmullRomSpline {
  private:
    struct SampledPoint {
        double u{0.0};                                    // parameter u
        Eigen::Vector2d value = Eigen::Vector2d::Zero();  // sampled point value
        std::size_t ctrlPointIndex{0};                    // first control point index
    };

  public:
    explicit CatmullRomSpline(const double& tau = 0.5);

  public:
    [[nodiscard]] inline const double& tau() const { return tau_; }
    [[nodiscard]] inline const std::vector<Eigen::Vector2d>& ctrlPoints() const { return ctrlPoints_; }
    [[nodiscard]] inline std::vector<Eigen::Vector2d>& ctrlPoints() { return ctrlPoints_; }

    void setCtrlPoints(const std::vector<Eigen::Vector2d>& ctrlPoints);

    [[nodiscard]] Eigen::Vector2d evaluateAt(const double& u, const std::size_t& firstCtrlPointIndex) const;

    [[nodiscard]] std::vector<Eigen::Vector2d> samplePoint(int num = 20) const;

    // sample points in place, only used for find u for input point
    void samplePointForFindU(int num = 20);

    /**
     * @brief Find the parameter u based the input point, and return the corresponding control points index(size=4)
     *
     * @param pt
     * @param ctrlPointIndex
     * @return
     */
    std::optional<double> findParamU(const Eigen::Vector2d& pt, std::size_t& firstCtrlPointIndex) const;

    [[nodiscard]] std::vector<std::size_t> FindTwoNearestCtrlPoints(const Eigen::Vector2d& pt) const;
    [[nodiscard]] std::vector<std::size_t> FindTwoNearestPoints(const Eigen::Vector2d& pt,
                                                                const std::vector<Eigen::Vector2d>& points) const;

  private:
  public:
    double tau_{0.5};                          // tau
    std::vector<Eigen::Vector2d> ctrlPoints_;  // control points

    Eigen::Matrix4d matTau_{Eigen::Matrix4d::Zero()};  // matrix M

    std::vector<SampledPoint> sampledPoints_;  // sampled points data

    // Eigen::VectorXd sampledU_;                                  // sampled u
    // std::vector<Eigen::Vector2d> sampledPoints_;                // sampled points, used for find u
    // std::vector<std::size_t> ctrlPointsIndexForSampledPoints_;  // control points index for sampled points
};