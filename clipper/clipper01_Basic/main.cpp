#include <clipper/clipper.h>
#include <fmt/format.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <common/common.hpp>

using namespace std;
using namespace fmt;
using namespace common;
using namespace Eigen;
using namespace clipper;

void basic01() {
    invariants::EuclideanDistance::Params invariantParams;
    invariants::EuclideanDistancePtr invariant = make_shared<invariants::EuclideanDistance>(invariantParams);
    Params params;
    CLIPPER clipper(invariant, params);

    // create a target/model point cloud of data
    Matrix3Xd model(3, 4);
    model.col(0) << 0, 0, 0;
    model.col(1) << 2, 0, 0;
    model.col(2) << 0, 3, 0;
    model.col(3) << 2, 2, 0;

    // transform of data w.r.t model
    Affine3d Tmd;
    Tmd = AngleAxisd(M_PI / 8, Vector3d::UnitZ());
    Tmd.translation() << 5, 3, 0;

    // create source/data point cloud
    Eigen::Matrix3Xd data = Tmd.inverse() * model;

    // remove one point from the target(model) cloud, simulates a partial view
    data.conservativeResize(3, 3);

    // an empty association set will be assumed to be all-to-all
    clipper.scorePairwiseConsistency(model, data);

    // find the densest clique of the previously constructed consistency graph
    clipper.solve();

    // check that the select clique was correct
    clipper::Association inliers = clipper.getSelectedAssociations();
    LOG(INFO) << "inliers: " << endl << inliers;
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    basic01();

    closeLog();
    return 0;
}