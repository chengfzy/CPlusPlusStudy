#include <fmt/format.h>
#include <fmt/ostream.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <iostream>

using namespace std;
using namespace fmt;
using namespace pcl;

int main(int argc, const char* argv[]) {
    PointCloud<PointXYZ>::Ptr cloudIn(new PointCloud<PointXYZ>(5, 1));
    PointCloud<PointXYZ>::Ptr cloudOut(new PointCloud<PointXYZ>);

    // fill in the input data
    for (auto& point : *cloudIn) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    // print
    cout << format("saved {} data points to input", cloudIn->size()) << endl;
    for (const auto& point : *cloudIn) {
        cout << format("\t{}", point) << endl;
    }

    // save output point
    *cloudOut = *cloudIn;
    cout << format("cloud out size: {}", cloudOut->size()) << endl;
    for (auto& point : *cloudOut) {
        point.x += 0.7;
    }
    for (const auto& point : *cloudOut) {
        cout << format("\t{}", point) << endl;
    }

    // ICP
    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    icp.setInputSource(cloudIn);
    icp.setInputTarget(cloudOut);
    PointCloud<PointXYZ> result;
    icp.align(result);
    cout << format("ICP converged: {}, score: {}", icp.hasConverged(), icp.getFitnessScore()) << endl;
    cout << format("transformation:\n{}", icp.getFinalTransformation()) << endl;

    return 0;
}