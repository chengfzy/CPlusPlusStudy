#include <ceres/ceres.h>
#include <fmt/format.h>
#include <Eigen/Core>
#include <boost/filesystem.hpp>
#include <common/common.hpp>
#include <fstream>
#include "CatmullRomSpline.h"
#include "Optimizer.h"

using namespace std;
using namespace fmt;
using namespace common;
using namespace Eigen;
namespace fs = boost::filesystem;

void savePoints(const std::vector<Eigen::Vector2d>& points, const fs::path& fileName) {
    if (!fs::exists(fileName.parent_path())) {
        fs::create_directories(fileName.parent_path());
    }
    LOG(INFO) << format("save {} points \"{}\"", points.size(), fileName.string());

    fstream fs(fileName.string(), ios::out);
    CHECK(fs.is_open()) << format("cannot open file to save points data \"{}\"", fileName.string());
    fs << "#X[m],Y[m]" << endl;
    for (auto& v : points) {
        fs << format("{:.10f},{:.10f}", v[0], v[1]) << endl;
    }
    fs.close();
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    // construct raw spline
    CatmullRomSpline rawSpline(0.5);
    std::vector<Eigen::Vector2d> rawCtrlPoints = {
        Eigen::Vector2d(0.0, 1.5), Eigen::Vector2d(0.4, 1.6), Eigen::Vector2d(2.0, 2.0),
        Eigen::Vector2d(3.0, 1.0), Eigen::Vector2d(4.0, 0.5), Eigen::Vector2d(5.0, 1.0),
        Eigen::Vector2d(6.0, 2.0), Eigen::Vector2d(6.5, 2.8), Eigen::Vector2d(7.0, 3.0)};
    rawSpline.setCtrlPoints(rawCtrlPoints);
    // generate observation points
    auto rawPoints = rawSpline.samplePoint(30);
    savePoints(rawPoints, "./data/CatmullRomSpline/RawPoints.csv");
    savePoints(rawSpline.ctrlPoints(), "./data/CatmullRomSpline/RawCtrlPoints.csv");

    // optimizer
    Optimizer optimizer;

    // optimize
    auto& spline = optimizer.fit(rawPoints);
    auto& splineBeforeOpt = optimizer.splineBeforeOpt();

    // generate optimization points
    auto optPointsBeforeOpt = splineBeforeOpt.samplePoint(20);
    auto optPoints = spline.samplePoint(20);
    savePoints(optPointsBeforeOpt, "./data/CatmullRomSpline/OptPointsBeforeOpt.csv");
    savePoints(optimizer.estimatedPointsBeforeOpt(), "./data/CatmullRomSpline/EstimatedPointsBeforeOpt.csv");
    savePoints(splineBeforeOpt.ctrlPoints(), "./data/CatmullRomSpline/OptCtrlPointsBeforeOpt.csv");
    savePoints(optPoints, "./data/CatmullRomSpline/OptPoints.csv");
    savePoints(optimizer.estimatedPoints(), "./data/CatmullRomSpline/EstimatedPoints.csv");
    savePoints(spline.ctrlPoints(), "./data/CatmullRomSpline/OptCtrlPoints.csv");

    closeLog();
    return 0;
}