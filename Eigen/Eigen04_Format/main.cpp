#include <fmt/ranges.h>
#include <Eigen/Core>
#include <common/common.hpp>
#include <iostream>
#include <numeric>
#include <vector>

using namespace std;
using namespace fmt;
using namespace Eigen;
using namespace common;

int main(int argc, char* argv[]) {
    // format std sequence
    cout << "format std sequence" << endl;
    vector<double> v1{1.123, 2.3545, 3.56898756};
    cout << fmt::format("v1: {}", v1) << endl;
    cout << fmt::format("v1: {::.3f}", v1) << endl;

    // format Eigen::Vector
    // fixed size at compile time
    cout << endl << "format Eigen::Vector" << endl;
    Vector3d v2(1.123, 2.3545, 3.56898756);
    cout << fmt::format("v2: {}", v2) << endl;
    cout << fmt::format("v2: {::.3f}", v2) << endl;
    // dynamic size
    VectorXd v3(3);
    v3 << 1.123, 2.3545, 3.56898756;
    cout << fmt::format("v3: {}", v3) << endl;
    cout << fmt::format("v3: {::.3f}", v3) << endl;

    // format Eigen::Matrix
    // fixed size at compile time
    cout << endl << "format Eigen::Matrix" << endl;
    Matrix3d m1 = Matrix3d::Random();
    cout << fmt::format("m1: {}", m1) << endl;
    cout << fmt::format("m1: {::.3f}", m1) << endl;
    // dynamic size
    MatrixXd m2(3, 4);
    m2.setRandom();
    cout << fmt::format("m2: {}", m2) << endl;
    cout << fmt::format("m2: {::.3f}", m2) << endl;
    MatrixXd m3(1, 4);
    m3.setRandom();
    cout << fmt::format("m3: {}", m3) << endl;
    cout << fmt::format("m3: {::.3f}", m3) << endl;

    return 0;
}
