#include <iostream>
#include "unsupported/Eigen/Splines"

using namespace std;
using namespace Eigen;

template <typename T>
T scaleX(const T& x, const T& min, const T& max) {
    return (x - min) / (max - min);
}

// center scale x data
VectorXd scaleXVec(const VectorXd& xVec) {
    const double min = xVec.minCoeff();
    const double max = xVec.maxCoeff();
    return xVec.unaryExpr([&](double x) { return scaleX(x, min, max); });
}

int main(int argc, char* argv[]) {
    VectorXd xVec(5);
    xVec << 1, 2, 3, 4, 6;
    const VectorXd yVec = xVec.array().square();

    Spline2d::ControlPointVectorType points(2, 5);
    points.row(0) = xVec;
    points.row(1) = yVec;
    cout << points << endl;

    // spline with scaled operation
    cout << "scaled..." << endl;
    Spline<double, 1> spline =
        SplineFitting<Spline<double, 1>>::Interpolate(yVec.transpose(), 3, scaleXVec(xVec).transpose());
    for (double x = 1.0; x < 6 + 0.5 * 0.1; x += 0.1) {
        cout << "(" << x << ", " << spline(scaleX(x, xVec.minCoeff(), xVec.maxCoeff())) << ")" << endl;
    }

    // spline without scaled operation, will be something error
    cout << "unscaled..." << endl;
    spline = SplineFitting<Spline<double, 1>>::Interpolate(yVec.transpose(), 3, xVec.transpose());
    for (double x = 1.0; x < 6 + 0.5 * 0.1; x += 0.1) {
        cout << "(" << x << ", " << spline(x)(0) << ")" << endl;
    }

    return 0;
}
