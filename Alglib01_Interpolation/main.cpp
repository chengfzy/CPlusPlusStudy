#include <iostream>
#include "alglib/src/interpolation.h"

using namespace std;
using namespace alglib;

// cubic interpolation
void cubicInterp() {
    real_1d_array x = "[-1.0, -0.5, 0.0, 0.5, 1.0]";
    real_1d_array y = "[1.0, 0.25, 0.0, 0.3, 1.0]";

    spline1dinterpolant s;
    spline1dbuildcubic(x, y, s);

    double t{0.25};
    double v = spline1dcalc(s, t);
    cout << "v(t = " << t << ") = " << v << endl;
}

int main(int argc, char* argv[]) {
    cubicInterp();

    return 0;
}
