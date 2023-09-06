/*
 * using g2o to fit curve y = a * exp(-lambda * x) + b
 */
#include <fmt/format.h>
#include <g2o/core/auto_differentiation.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/sampler.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace Eigen;
using namespace g2o;
using namespace common;

/**
 * @brief The params, a, b and lambda for "a * exp(-lambda * x) + b"
 *
 */
class VertexParam : public BaseVertex<3, Vector3d> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexParam() = default;
    virtual bool read(istream&) { return false; }
    virtual bool write(ostream&) const { return false; }
    virtual void setToOriginImpl() { cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl; }

    virtual void oplusImpl(const double* update) {
        Vector3d::ConstMapType v(update);
        _estimate += v;
    }
};

/**
 * @brief Measurement for a point on the curve
 *
 */
class EdgePointOnCurve : public BaseUnaryEdge<1, Vector2d, VertexParam> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    G2O_MAKE_AUTO_AD_FUNCTIONS  // use autodiff

    EdgePointOnCurve() = default;
    virtual bool read(istream&) { return false; }
    virtual bool write(ostream&) const { return false; }

    template <typename T>
    bool operator()(const T* params, T* error) const {
        const T& a = params[0];
        const T& b = params[1];
        const T& lambda = params[2];
        error[0] = T(measurement()[1]) - a * exp(-lambda * T(measurement()[0])) - b;
        return true;
    }
};

int main(int argc, char** argv) {
    // argument parser
    int numPoints;
    int maxIterations;
    bool verbose;
    string dumpFileName;
    CommandArgs arg;
    arg.param("numPoints", numPoints, 100, "number of points sampled from the curve");
    arg.param("i", maxIterations, 10, "perform n iterations");
    arg.param("v", verbose, false, "verbose output of the optimization process");
    arg.param("dump", dumpFileName, "", "dump the points into a file");
    arg.parseArgs(argc, argv);
    cout << format("number points = {}, max iterations = {}, verbose = {}", numPoints, maxIterations, verbose) << endl;
    cout << format("dump file: {}", dumpFileName) << endl;

    // generate random data, y = a * exp(-lambda * x) + b
    Sampler::seedRand();
    double a{2}, b{0.4}, lambda{0.2};
    vector<Vector2d> points(numPoints);
    for (size_t i = 0; i < numPoints; ++i) {
        double x = Sampler::uniformRand(0, 10);
        double y = a * exp(-lambda * x) + b + Sampler::gaussRand(0, 0.02);
        points[i] = Vector2d(x, y);
    }

    // dump to file
    if (!dumpFileName.empty()) {
        ofstream fs(dumpFileName);
        for (auto& p : points) {
            fs << p.transpose() << endl;
        }
    }

    // setup the solver
    SparseOptimizer optimizer;
    // allocate the solver
    auto solver = new OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverX>(g2o::make_unique<LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>()));
    optimizer.setAlgorithm(solver);

    // build the optimization problem given the points
    // 1. add the curve parameters vertex
    auto params = new VertexParam;
    params->setId(0);
    params->setEstimate(Vector3d(1, 1, 1));  // initial value for the curve
    optimizer.addVertex(params);
    // 2. add the points we measured
    for (size_t i = 0; i < numPoints; ++i) {
        auto e = new EdgePointOnCurve;
        e->setInformation(Matrix<double, 1, 1>::Identity());
        e->setVertex(0, params);
        e->setMeasurement(points[i]);
        optimizer.addEdge(e);
    }

    // perform the optimization
    optimizer.initializeOptimization();
    optimizer.setVerbose(verbose);
    optimizer.optimize(maxIterations);
    if (verbose) {
        cout << endl;
    }
    // print out the optimization result
    cout << format("a = {:.5f}, b = {:.5f}, lambda = {:.5f}", a, b, lambda) << endl;
    cout << format("optimization result, a = {:.5f}, b = {:.5f}, lambda = {:.5f}", params->estimate()[0],
                   params->estimate()[1], params->estimate()[2])
         << endl;

    return 0;
}
