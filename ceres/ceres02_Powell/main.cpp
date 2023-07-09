#include <ceres/ceres.h>

using namespace std;
using namespace ceres;

// f1 = x1 + 10 * x2
struct F1 {
    template <typename T>
    bool operator()(const T* const x1, const T* const x2, T* residual) const {
        residual[0] = x1[0] + 10.0 * x2[0];
        return true;
    }
};

// f2 = sqrt(5) * (x3 - x4)
struct F2 {
    template <typename T>
    bool operator()(const T* const x3, const T* const x4, T* residual) const {
        residual[0] = sqrt(5.0) * (x3[0] - x4[0]);
        return true;
    }
};

// f3 = (x2 - 2 * x3)^2
struct F3 {
    template <typename T>
    bool operator()(const T* const x2, const T* const x3, T* residual) const {
        residual[0] = (x2[0] - 2.0 * x3[0]) * (x2[0] - 2.0 * x3[0]);
        return true;
    }
};

// f4 = sqrt(10) * (x1 - x4)^2
struct F4 {
    template <typename T>
    bool operator()(const T* const x1, const T* const x4, T* residual) const {
        residual[0] = sqrt(10.0) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
        return true;
    }
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    double x1{3.0}, x2{-1.0}, x3{0.0}, x4{1.0};

    Problem problem;
    // add residual terms to the problem using the auto-diff wrapper
    problem.AddResidualBlock(new AutoDiffCostFunction<F1, 1, 1, 1>(new F1), nullptr, &x1, &x2);
    problem.AddResidualBlock(new AutoDiffCostFunction<F2, 1, 1, 1>(new F2), nullptr, &x3, &x4);
    problem.AddResidualBlock(new AutoDiffCostFunction<F3, 1, 1, 1>(new F3), nullptr, &x2, &x3);
    problem.AddResidualBlock(new AutoDiffCostFunction<F4, 1, 1, 1>(new F4), nullptr, &x1, &x4);

    // solver options
    Solver::Options options;
    options.minimizer_type = ceres::MinimizerType::TRUST_REGION;  // minimize type
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // run the solver
    cout << "Initial x = [" << x1 << ", " << x2 << ", " << x3 << ", " << x4 << "]" << endl;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;
    cout << "Final x = [" << x1 << ", " << x2 << ", " << x3 << ", " << x4 << "]" << endl;

    google::ShutdownGoogleLogging();
    return 0;
}
