#include <ceres/ceres.h>
#include <fmt/format.h>
#include <Eigen/Core>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <common/common.hpp>
#include <cxxopts.hpp>
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

std::vector<Eigen::Vector2d> loadPoints(const fs::path& file) {
    // open file
    std::fstream fs(file.string(), ios::in);
    CHECK(fs.is_open()) << format("cannot open file \"{}\"", file.string());

    std::vector<Eigen::Vector2d> points;
    string line;
    while (getline(fs, line)) {
        vector<string> tokens;
        split(tokens, line, boost::is_any_of(","));
        if (tokens.size() >= 2) {
            points.emplace_back(boost::lexical_cast<double>(boost::trim_copy(tokens[0])),
                                boost::lexical_cast<double>(boost::trim_copy(tokens[1])));
        }
    }
    fs.close();

    LOG(INFO) << format("load {} points from file \"{}\"", points.size(), file.string());
    return points;
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

#if 0
    // construct raw spline
    CatmullRomSpline rawSpline(0.5);
    std::vector<Eigen::Vector2d> rawCtrlPoints = {
        Eigen::Vector2d(0.0, 1.5), Eigen::Vector2d(0.4, 1.6), Eigen::Vector2d(2.0, 2.0),
        Eigen::Vector2d(3.0, 1.0), Eigen::Vector2d(4.0, 0.5), Eigen::Vector2d(5.0, 1.0),
        Eigen::Vector2d(6.0, 2.0), Eigen::Vector2d(6.5, 2.8), Eigen::Vector2d(7.0, 3.0)};
    rawSpline.setCtrlPoints(rawCtrlPoints);
    // generate observation points
    auto rawPoints = rawSpline.samplePoint(30);
    savePoints(rawPoints, "./temp/CatmullRomSpline/RawPoints.csv");
    savePoints(rawSpline.ctrlPoints(), "./temp/CatmullRomSpline/RawCtrlPoints.csv");
#else
    // argument parser
    cxxopts::Options options(argv[0], "Catmull-Rom Spline Curve Fitting");
    options.set_width(120);
    // clang-format off
    options.add_options()
        ("rawPointsFile", "raw points file", cxxopts::value<string>()->default_value("./data/CatmullRomSpline/LaneLine01.csv"))
        ("h,help", "help message");
    // clang-format on
    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        cout << options.help() << endl;
        return 0;
    }

    // 弯道入直, ./data/CatmullRomSpline/LaneLine01.csv.csv
    // parse
    fs::path rawPointsFile = fs::weakly_canonical(fs::path(result["rawPointsFile"].as<string>()));
    std::vector<Eigen::Vector2d> rawPoints = loadPoints(rawPointsFile);
    savePoints(rawPoints, "./temp/CatmullRomSpline/RawPoints.csv");
#endif

    // optimizer
    Optimizer optimizer;

    // optimize
    auto& spline = optimizer.fit(rawPoints);
    auto& splineBeforeOpt = optimizer.splineBeforeOpt();

    // generate optimization points
    auto optPointsBeforeOpt = splineBeforeOpt.samplePoint(20);
    auto optPoints = spline.samplePoint(20);
    savePoints(optPointsBeforeOpt, "./temp/CatmullRomSpline/OptPointsBeforeOpt.csv");
    savePoints(optimizer.estimatedPointsBeforeOpt(), "./temp/CatmullRomSpline/EstimatedPointsBeforeOpt.csv");
    savePoints(splineBeforeOpt.ctrlPoints(), "./temp/CatmullRomSpline/OptCtrlPointsBeforeOpt.csv");
    savePoints(optPoints, "./temp/CatmullRomSpline/OptPoints.csv");
    savePoints(optimizer.estimatedPoints(), "./temp/CatmullRomSpline/EstimatedPoints.csv");
    savePoints(spline.ctrlPoints(), "./temp/CatmullRomSpline/OptCtrlPoints.csv");

    closeLog();
    return 0;
}