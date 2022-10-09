#include "Dbscan.h"

using namespace std;
using namespace Eigen;

Dbscan::Dbscan(int minPts, const double& eps) : minPts_(minPts), eps2_(eps * eps) {}

vector<int> Dbscan::fit(const vector<Vector3d>& points) {
    constexpr int kUndefined{-1};
    constexpr int kNoise{-2};
    const auto kSize = points.size();

    vector<int> labels(kSize, kUndefined);
    int label{0};  // current label
    for (size_t n = 0; n < kSize; ++n) {
        if (labels[n] == kUndefined) {
            auto seeds = regionQuery(points, n);
            if (seeds.size() < minPts_) {
                labels[n] = kNoise;
            } else {
                // set cluster id for seeds(core object)
                for (size_t i = 0; i < seeds.size(); ++i) {
                    labels[seeds[i]] = label;
                }

                // delete paint from seeds
                seeds.erase(remove(seeds.begin(), seeds.end(), n), seeds.end());

                // seeds => empty
                while (!seeds.empty()) {
                    auto m = seeds.front();
                    auto neighbors = regionQuery(points, m);
                    if (neighbors.size() >= minPts_) {
                        for (auto& idx : neighbors) {
                            if (labels[idx] < 0) {
                                if (labels[idx] == kUndefined) {
                                    seeds.emplace_back(idx);
                                }
                                labels[idx] = label;
                            }
                        }
                    }

                    // delete paint from seeds
                    seeds.erase(remove(seeds.begin(), seeds.end(), m), seeds.end());
                }

                ++label;
            }
        }
    }

    return labels;
}

vector<size_t> Dbscan::regionQuery(const vector<Vector3d>& points, int idx) {
    vector<size_t> neighbors;
    for (size_t i = 0; i < points.size(); ++i) {
        if ((points[i] - points[idx]).squaredNorm() < eps2_) {
            neighbors.emplace_back(i);
        }
    }

    return neighbors;
}