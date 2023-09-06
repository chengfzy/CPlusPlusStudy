/*
 * using ceres to find the minimum of function 0.5 * (10 - x)^2
 */
#include <ceres/ceres.h>
#include <common/common.hpp>
#include <glog/logging.h>

using namespace std;
using namespace common;
using namespace ceres;

// A templated cost functor that will evaluate residual r = 10 - x
struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // the variable to solve for with its initial value
    double initX = 5.0;
    double x = initX;

    // build the problem
    Problem problem;
    CostFunction* costFunction = new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(costFunction, nullptr, &x);

    // solve
    Solver::Options options;
    options.linear_solver_type = DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    // print result
    cout << summary.FullReport() << endl;
    cout << "x : " << initX << " => " << x << endl << endl;

    // covariance estimation
    Covariance::Options covOptions;
    Covariance covariance(covOptions);
    vector<pair<const double*, const double*>> covarianceBlocks;
    covarianceBlocks.emplace_back(make_pair(&x, &x));
    covariance.Compute(covarianceBlocks, &problem);
    double cov{0};
    covariance.GetCovarianceBlock(&x, &x, &cov);
    cout << "covariance: = " << cov << endl;

    return 0;
}
