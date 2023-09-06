/*
 * using ceres to find the minimum of function 0.5 * (10 - x)^2
 */
#include <ceres/ceres.h>
#include <common/common.hpp>
#include <glog/logging.h>

using namespace std;
using namespace common;

// A templated cost functor that will evaluate residual r = 10 - x
struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

// solve using auto-differentiation
void solveUseAutoDiff() {
    cout << Section("Use Auto Diff") << endl;

    // the variable to solve for with its initial value
    double initialX = 5.0;
    double x = initialX;

    // build the problem
    ceres::Problem problem;

    // set up the only cost function (residual), this used auto-differentiation to obtain the derivative(Jacobian)
    ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(costFunction, nullptr, &x);

    // Run the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // print result
    cout << summary.BriefReport() << endl;
    cout << "x : " << initialX << " => " << x << endl << endl;
}

// numeric cost functor
struct NumericDiffCostFunctor {
    bool operator()(const double* const x, double* residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    }
};

// solve using numeric derivatives
void solveUseNumericDiff() {
    cout << Section("Use Numeric Diff") << endl;

    // the variable to solve for with its initial value
    double initialX = 5.0;
    double x = initialX;

    // build the problem
    ceres::Problem problem;

    // set up the only cost function (residual), this used numeric derivatives to obtain the derivative(jacobian)
    ceres::CostFunction* costFunction =
        new ceres::NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL, 1, 1>(new NumericDiffCostFunctor);
    problem.AddResidualBlock(costFunction, nullptr, &x);

    // Run the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // print result
    cout << summary.BriefReport() << endl;
    cout << "x : " << initialX << " => " << x << endl << endl;
}

// analytic derivatives
class QuadraticCostFunction : public ceres::SizedCostFunction<1, 1> {
  public:
    QuadraticCostFunction() = default;
    ~QuadraticCostFunction() override = default;

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override {
        const double x = parameters[0][0];
        residuals[0] = 10 - x;

        // compute the Jacobian if asked for
        if (jacobians != nullptr && jacobians[0] != nullptr) {
            jacobians[0][0] = -1;
        }

        return true;
    }
};

// solve using analytic derivatives
void solveUseAnalyticDiff() {
    cout << Section("Use Analytic Diff") << endl;

    // the variable to solve for with its initial value
    double initialX = 5.0;
    double x = initialX;

    // build the problem
    ceres::Problem problem;

    // set up the only cost function (residual), this used analytic derivatives to obtain the derivative(jacobian)
    ceres::CostFunction* costFunction = new QuadraticCostFunction;
    problem.AddResidualBlock(costFunction, nullptr, &x);

    // Run the solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // print result
    cout << summary.BriefReport() << endl;
    cout << "x : " << initialX << " => " << x << endl << endl;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    solveUseAutoDiff();
    solveUseNumericDiff();
    solveUseAnalyticDiff();

    return 0;
}
