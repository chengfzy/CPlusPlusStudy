#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "AngleManifold.h"
#include "G2OReader.h"
#include "PoseGraph2DErrorTerm.h"
#include "types.h"

using namespace ceres;
using namespace std;

DEFINE_string(inputFile, "./ceres/data/manhattan/originalDataset/g2o/manhattanOlson3500.g2o",
              "pose graph definition filename in g2o format");

// save poses to the file with format: ID x y yaw
bool savePose(const string& filename, const map<int, Pose2d>& poses) {
    fstream fs(filename, ios::out);
    if (!fs.is_open()) {
        LOG(ERROR) << "cannot create file \"" << filename << "\"";
        return false;
    }
    for (auto& p : poses) {
        fs << p.first << " " << p.second.x << " " << p.second.y << " " << p.second.yaw << endl;
    }
    fs.close();
    return true;
}

int main(int argc, char* argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);

    // check input
    CHECK(!FLAGS_inputFile.empty()) << "need to specify the input filename";

    // read data from file
    map<int, Pose2d> poses;
    vector<Constraint2d> constraints;
    if (!readG2OFile(FLAGS_inputFile, poses, constraints)) {
        LOG(FATAL) << "read data from file failed";
    }
    cout << "number of poses: " << poses.size() << endl;
    cout << "number of constraints: " << constraints.size() << endl;

    // save original poses
    savePose("./poses_2d_original.txt", poses);

    // build problem
    Problem problem;
    LossFunction* lossFunction = nullptr;
    auto angleManifold = AngleManifold::create();
    for (auto& c : constraints) {
        auto itPoseBegin = poses.find(c.idBegin);
        CHECK(itPoseBegin != poses.end()) << "Pose with ID = " << c.idBegin << " not found";
        auto itPoseEnd = poses.find(c.idEnd);
        CHECK(itPoseEnd != poses.end()) << "Pose with ID = " << c.idEnd << " not found";

        const Eigen::Matrix3d sqrtInformation = c.information.llt().matrixL();
        CostFunction* costFunction = PoseGraph2DErrorTerm::create(c.x, c.y, c.yaw, sqrtInformation);
        problem.AddResidualBlock(costFunction, lossFunction, &itPoseBegin->second.x, &itPoseBegin->second.y,
                                 &itPoseBegin->second.yaw, &itPoseEnd->second.x, &itPoseEnd->second.y,
                                 &itPoseEnd->second.yaw);

        problem.SetManifold(&itPoseBegin->second.yaw, angleManifold);
        problem.SetManifold(&itPoseEnd->second.yaw, angleManifold);
    }
    // constrain the gauge freedom by setting one of the poses as constant so the optimizer cannot change it
    auto itPoseStart = poses.begin();
    CHECK(itPoseStart != poses.end()) << "There are no poses";
    cout << "start poses: x = " << itPoseStart->second.x << ", y = " << itPoseStart->second.y
         << ", yaw = " << itPoseStart->second.yaw << endl;
    problem.SetParameterBlockConstant(&itPoseStart->second.x);
    problem.SetParameterBlockConstant(&itPoseStart->second.y);
    problem.SetParameterBlockConstant(&itPoseStart->second.yaw);

    // solve problem
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;

    // save optimized poses
    savePose("./poses_2d_optimized.txt", poses);

    google::ShutdownGoogleLogging();
    google::ShutDownCommandLineFlags();
    return 0;
}