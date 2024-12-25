#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/std.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <common/common.hpp>
#include <fstream>
#include <iostream>
#include "Dbscan.h"

using namespace std;
using namespace fmt;
using namespace Eigen;
using namespace common;
namespace fs = boost::filesystem;

vector<Vector3d> readPoints(const fs::path& file) {
    LOG(INFO) << format("read points from file {}", fmt::streamed(file));
    CHECK(fs::exists(file)) << format("data file {} don't exist", file);

    vector<Vector3d> points;
    std::fstream fs(file.string(), ios::in);
    CHECK(fs.is_open()) << format("cannot open file {}", file);
    string line;
    while (getline(fs, line)) {
        using namespace boost;
        vector<string> tokens;
        split(tokens, line, is_any_of(","));
        if (tokens.size() == 4) {
            points.emplace_back(lexical_cast<double>(trim_copy(tokens[0])), lexical_cast<double>(trim_copy(tokens[1])),
                                lexical_cast<double>(trim_copy(tokens[2])));
        }
    }
    fs.close();
    LOG(INFO) << format("read {} points", points.size());

    return points;
}

void saveResults(const vector<Vector3d>& points, const vector<int>& labels, const fs::path& file) {
    LOG(INFO) << format("save {} results to file {}", points.size(), file);

    std::ofstream fs(file.string());
    CHECK(fs.is_open()) << format("cannot create file \"{}\" to save result", file);
    fs << "#X,Y,Z,Label" << endl;
    for (size_t n = 0; n < points.size(); ++n) {
        fs << format("{},{},{},{}", points[n][0], points[n][1], points[n][2], labels[n]) << endl;
    }
    fs.close();
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    // read points from file
    fs::path inputFile{"./algorithm/alg01_DBSCAN/data/benchmark_hepta.dat"};
    auto points = readPoints(inputFile);

    // DBSCAN
    Dbscan dbscan(4, 0.75);
    auto labels = dbscan.fit(points);

    // save result
    fs::path saveFile{"./data/DbscanResult.csv"};
    saveResults(points, labels, saveFile);

    closeLog();
    return 0;
}
