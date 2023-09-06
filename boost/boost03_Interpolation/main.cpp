#include <boost/math/interpolators/barycentric_rational.hpp>
#include <boost/math/interpolators/cubic_b_spline.hpp>
#include <boost/math/tools/roots.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/range/adaptors.hpp>
#include <iostream>
#include <random>
#include <common/common.hpp>

using namespace std;
using namespace boost;
using namespace common;

// simple rational interpolation.
// Ref: https://www.boost.org/doc/libs/1_67_0/libs/math/doc/html/math_toolkit/barycentric.html
void simpleRationalInterpolate() {
    cout << Section("Simple Rational Interpolation") << endl;
    // create data
    vector<double> r(45);
    vector<double> mrV(45);
    r[0] = 0.02;
    mrV[0] = 5.727;
    r[1] = 0.04;
    mrV[1] = 5.544;
    r[2] = 0.06;
    mrV[2] = 5.450;
    r[3] = 0.08;
    mrV[3] = 5.351;
    r[4] = 0.10;
    mrV[4] = 5.253;
    r[5] = 0.12;
    mrV[5] = 5.157;
    r[6] = 0.14;
    mrV[6] = 5.058;
    r[7] = 0.16;
    mrV[7] = 4.960;
    r[8] = 0.18;
    mrV[8] = 4.862;
    r[9] = 0.20;
    mrV[9] = 4.762;
    r[10] = 0.24;
    mrV[10] = 4.563;
    r[11] = 0.28;
    mrV[11] = 4.360;
    r[12] = 0.32;
    mrV[12] = 4.1584;
    r[13] = 0.36;
    mrV[13] = 3.9463;
    r[14] = 0.40;
    mrV[14] = 3.7360;
    r[15] = 0.44;
    mrV[15] = 3.5429;
    r[16] = 0.48;
    mrV[16] = 3.3797;
    r[17] = 0.52;
    mrV[17] = 3.2417;
    r[18] = 0.56;
    mrV[18] = 3.1209;
    r[19] = 0.60;
    mrV[19] = 3.0138;
    r[20] = 0.68;
    mrV[20] = 2.8342;
    r[21] = 0.76;
    mrV[21] = 2.6881;
    r[22] = 0.84;
    mrV[22] = 2.5662;
    r[23] = 0.92;
    mrV[23] = 2.4242;
    r[24] = 1.00;
    mrV[24] = 2.3766;
    r[25] = 1.08;
    mrV[25] = 2.3058;
    r[26] = 1.16;
    mrV[26] = 2.2458;
    r[27] = 1.24;
    mrV[27] = 2.2035;
    r[28] = 1.32;
    mrV[28] = 2.1661;
    r[29] = 1.40;
    mrV[29] = 2.1350;
    r[30] = 1.48;
    mrV[30] = 2.1090;
    r[31] = 1.64;
    mrV[31] = 2.0697;
    r[32] = 1.80;
    mrV[32] = 2.0466;
    r[33] = 1.96;
    mrV[33] = 2.0325;
    r[34] = 2.12;
    mrV[34] = 2.0288;
    r[35] = 2.28;
    mrV[35] = 2.0292;
    r[36] = 2.44;
    mrV[36] = 2.0228;
    r[37] = 2.60;
    mrV[37] = 2.0124;
    r[38] = 2.76;
    mrV[38] = 2.0065;
    r[39] = 2.92;
    mrV[39] = 2.0031;
    r[40] = 3.08;
    mrV[40] = 2.0015;
    r[41] = 3.24;
    mrV[41] = 2.0008;
    r[42] = 3.40;
    mrV[42] = 2.0004;
    r[43] = 3.56;
    mrV[43] = 2.0002;
    r[44] = 3.72;
    mrV[44] = 2.0001;

    // interpolate
    boost::math::barycentric_rational<double> b(r.data(), mrV.data(), r.size());

    // evaluate at other points
    for (size_t i = 1; i < 8; ++i) {
        double r = i * 0.5;
        cout << "(r, V) = (" << r << ", " << b(r) << ")" << endl;
    }
}

// more complex rational interpolation
// Ref: https://www.boost.org/doc/libs/1_67_0/libs/math/doc/html/math_toolkit/barycentric.html
void rationalInterpolate() {
    cout << Section("Complex Rational Interpolation") << endl;
    // could equally use and unordered map, a list of tuples or pairs, or 2-dimensional array
    map<double, double> r;
    r[0.02] = 5.727;
    r[0.04] = 5.544;
    r[0.06] = 5.450;
    r[0.08] = 5.351;
    r[0.10] = 5.253;
    r[0.12] = 5.157;
    r[0.14] = 5.058;
    r[0.16] = 4.960;
    r[0.18] = 4.862;
    r[0.20] = 4.762;
    r[0.24] = 4.563;
    r[0.28] = 4.360;
    r[0.32] = 4.1584;
    r[0.36] = 3.9463;
    r[0.40] = 3.7360;
    r[0.44] = 3.5429;
    r[0.48] = 3.3797;
    r[0.52] = 3.2417;
    r[0.56] = 3.1209;
    r[0.60] = 3.0138;
    r[0.68] = 2.8342;
    r[0.76] = 2.6881;
    r[0.84] = 2.5662;
    r[0.92] = 2.4242;
    r[1.00] = 2.3766;
    r[1.08] = 2.3058;
    r[1.16] = 2.2458;
    r[1.24] = 2.2035;
    r[1.32] = 2.1661;
    r[1.40] = 2.1350;
    r[1.48] = 2.1090;
    r[1.64] = 2.0697;
    r[1.80] = 2.0466;
    r[1.96] = 2.0325;
    r[2.12] = 2.0288;
    r[2.28] = 2.0292;
    r[2.44] = 2.0228;
    r[2.60] = 2.0124;
    r[2.76] = 2.0065;
    r[2.92] = 2.0031;
    r[3.08] = 2.0015;
    r[3.24] = 2.0008;
    r[3.40] = 2.0004;
    r[3.56] = 2.0002;
    r[3.72] = 2.0001;

    // creat 2 ranges for the x and y values
    auto xRange = boost::adaptors::keys(r);
    auto yRange = boost::adaptors::values(r);
    boost::math::barycentric_rational<double> b(xRange.begin(), xRange.end(), yRange.begin());

    // use simple bisection to find the root
    boost::uintmax_t iterations = std::numeric_limits<boost::uintmax_t>::max();
    double abscissa3 = boost::math::tools::bisect([=](double x) { return b(x) - 3; }, 0.44, 1.24,
                                                  boost::math::tools::eps_tolerance<double>(), iterations)
                           .first;
    cout << "Abscissa value that yields a potential of 3 = " << abscissa3 << endl;
    cout << "Root was found in " << iterations << " iterations." << endl;

    // however, we have a more efficient root finding algorithm than simple bisection
    iterations = std::numeric_limits<boost::uintmax_t>::max();
    abscissa3 = boost::math::tools::bracket_and_solve_root([=](double x) { return b(x) - 3; }, 0.6, 1.2, false,
                                                           boost::math::tools::eps_tolerance<double>(), iterations)
                    .first;
    cout << "Abscissa value that yields a potential of 3 = " << abscissa3 << endl;
    cout << "Root was found in " << iterations << " iterations." << endl;
}

// cubic B-spline interpolation.
// Ref: https://www.boost.org/doc/libs/1_67_0/libs/math/doc/html/math_toolkit/cubic_b.html
void cubicBSpline() {
    cout << Section("Cubic B-Spline Interpolation") << endl;

    // initialize the vector with a function we'd like to interpolate
    vector<double> v(500);
    double step = 0.01;
    for (size_t i = 0; i < v.size(); ++i) {
        v[i] = sin(i * step);
    }

    // create cubic B-spline interpolator
    boost::math::cubic_b_spline<double> spline(v.data(), v.size(), 0, step);

    // now we can evaluate the spline werever we please
    std::mt19937 gen;
    boost::random::uniform_real_distribution<double> absissa(0, v.size() * step);
    for (size_t i = 0; i < 10; ++i) {
        double x = absissa(gen);
        cout << "sin(" << x << ") = " << sin(x) << ", spline interpolation gives " << spline(x) << endl;
        cout << "cos(" << x << ") = " << cos(x) << ", spline derivative interpolation gives " << spline.prime(x)
             << endl;
    }
}

int main(int argc, char* argv[]) {
    simpleRationalInterpolate();
    rationalInterpolate();
    cubicBSpline();
    return 0;
}
