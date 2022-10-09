#pragma once
#include <Eigen/Core>
#include <vector>

class Dbscan {
  public:
    explicit Dbscan(int minPts = 4, const double& eps = 1.0);

  public:
    std::vector<int> fit(const std::vector<Eigen::Vector3d>& points);

  private:
    std::vector<std::size_t> regionQuery(const std::vector<Eigen::Vector3d>& points, int idx);

  private:
    // std::vector<Eigen::Vector3d> points_;  // points
    int minPts_;   // min points
    double eps2_;  // eps square
};