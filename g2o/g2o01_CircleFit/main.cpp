/*
 * using g2o to fit circle
 */
#include <fmt/format.h>
#include <g2o/core/auto_differentiation.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/sparse_optimizer.h>
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

G2O_USE_OPTIMIZATION_LIBRARY(dense)

/**
 * @brief A circle located at x, y with radius r
 *
 */
class VertexCircle : public BaseVertex<3, Vector3d> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexCircle() = default;
    virtual bool read(istream&) { return false; }
    virtual bool write(ostream&) const { return false; }
    virtual void setToOriginImpl() { cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl; }

    virtual void oplusImpl(const double* update) {
        Vector3d::ConstMapType v(update);
        _estimate += v;
    }
};

/**
 * @brief Measurement for a point on the circle
 *
 * Here the measurement is the point which is on the circle, the error function compute the distance of the point to the
 * center minus the radius of the circle
 */
class EdgePointOnCircle : public BaseUnaryEdge<1, Vector2d, VertexCircle> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    G2O_MAKE_AUTO_AD_FUNCTIONS  // use autodiff

    EdgePointOnCircle() = default;
    virtual bool read(istream&) { return false; }
    virtual bool write(ostream&) const { return false; }

    template <typename T>
    bool operator()(const T* circle, T* error) const {
        typename g2o::VectorN<2, T>::ConstMapType center(circle);
        const T& radius = circle[2];
        error[0] = (measurement().cast<T>() - center).norm() - radius;
        return true;
    }
};

/**
 * @brief Calculate the sum error of all the point using estimated circle params
 *
 * @param points    Points
 * @param circle    Estimated circle parameters
 * @return  Sum of all errors
 */
double errorOfSolution(const vector<Vector2d>& points, const Vector3d& circle) {
    Vector2d center = circle.head<2>();
    const double& radius = circle[2];
    double error{0};
    for (auto& p : points) {
        double d = (p - center).norm() - radius;
        error += d * d;
    }
    return error;
}

int main(int argc, char** argv) {
    // argument parser
    int numPoints;
    int maxIterations;
    bool verbose;
    CommandArgs arg;
    arg.param("numPoints", numPoints, 100, "number of points sampled from the circle");
    arg.param("i", maxIterations, 10, "perform n iterations");
    arg.param("v", verbose, false, "verbose output of the optimization process");
    arg.parseArgs(argc, argv);
    cout << format("number points = {}, max iterations = {}, verbose = {}", numPoints, maxIterations, verbose) << endl;

    // generate random data
    Vector2d center(4.0, 2.0);
    double radius = 2.0;
    vector<Vector2d> points(numPoints);
    Sampler::seedRand();
    for (size_t i = 0; i < numPoints; ++i) {
        double r = Sampler::gaussRand(radius, 0.05);
        double angle = Sampler::uniformRand(0.0, 2.0 * M_PI);
        points[i].x() = center.x() + r * cos(angle);
        points[i].y() = center.y() + r * sin(angle);
    }

    // list all solvers
    cout << "all solvers:" << endl;
    OptimizationAlgorithmFactory::instance()->listSolvers(cout);
    cout << endl;

    // setup the solver
    SparseOptimizer optimizer;
    optimizer.setVerbose(true);
    // allocate the solver
    OptimizationAlgorithmProperty solverProperty;
    optimizer.setAlgorithm(OptimizationAlgorithmFactory::instance()->construct("lm_dense", solverProperty));

    // build the optimization problem given the points
    // 1. add the circle vertex
    auto circle = new VertexCircle;
    circle->setId(0);
    circle->setEstimate(Vector3d(3, 3, 3));  // initial value for the circle
    optimizer.addVertex(circle);
    // 2. add the points we measured
    for (size_t i = 0; i < numPoints; ++i) {
        auto e = new EdgePointOnCircle;
        e->setInformation(Matrix<double, 1, 1>::Identity());
        e->setVertex(0, circle);
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
    cout << format("optimization solution, center = ({:.5f}, {:.5f}), radius = {:.5f}, error = {:.5f}",
                   circle->estimate()[0], circle->estimate()[1], circle->estimate()[2],
                   errorOfSolution(points, circle->estimate()))
         << endl;

    // solve by linear least square
    // let (a, b) be center of the circle and r the radius of the circle, for a point (x, y) on the circle we have
    //      (x - a)^2 + (y - b)^2 = r^2
    // then
    //      (-2x -2y 1)^T * (a b c) = -x^2 - y^2
    // where c = a^2 + b^2 - r^2
    MatrixXd A(numPoints, 3);
    VectorXd b(numPoints);
    for (size_t i = 0; i < numPoints; ++i) {
        A(i, 0) = -2 * points[i].x();
        A(i, 1) = -2 * points[i].y();
        A(i, 2) = 1;
        b(i) = -pow(points[i].x(), 2) - pow(points[i].y(), 2);
    }
    Vector3d sol = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    sol[2] = sqrt(pow(sol[0], 2) + pow(sol[1], 2) - sol[2]);
    // print the linear square result
    cout << format("linear square solution, center = ({:.5f}, {:.5f}), radius = {:.5f}, error = {:.5f}", sol[0], sol[1],
                   sol[2], errorOfSolution(points, sol))
         << endl;

    return 0;
}
