/**
 * Simulate a robot traversing down a 1-dimension hallway with noise odometry readings and noise range readings of the
 * end of the hallway. By fusing the noisy odometry and sensor readings this example demonstrates how to compute the
 * maximum likelihood estimate (MLE) of the robot's pose at each timestamp.
 */

#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <common/common.hpp>
#include <iostream>
#include <random>

using namespace std;
using namespace ceres;
using namespace common;

DEFINE_double(corridorLength, 30.0, "length of the corridor that the robot is travelling down");
DEFINE_double(poseSeparation, 0.5, "the distance that the robot traverse between successive odometry update");
DEFINE_double(odometryStddev, 0.1, "the standard deviation of odometry error of the robot");
DEFINE_double(rangeStddev, 0.01, "the standard deviation of range readings of the robot");

// simulate robot
void simulateRobot(vector<double>& odometryValues, vector<double>& rangeReadings) {
    const int kNumSteps = static_cast<int>(std::ceil(FLAGS_corridorLength / FLAGS_poseSeparation));

    // random generator
    std::random_device randomDevice;
    std::mt19937 gen{randomDevice()};
    // random distribution
    std::normal_distribution<> d1(0, FLAGS_odometryStddev);
    std::normal_distribution<> d2(0, FLAGS_rangeStddev);

    // the robot starts out at the origin
    double robotLocation{0.0};
    for (int i = 0; i < kNumSteps; ++i) {
        double actualOdometryValue = min(FLAGS_poseSeparation, FLAGS_corridorLength - robotLocation);
        robotLocation += actualOdometryValue;
        double actualRange = FLAGS_corridorLength - robotLocation;

        // add noise
        double observedOdometry = actualOdometryValue + d1(gen);
        double observedRange = actualRange + d2(gen);

        // add to list
        odometryValues.emplace_back(observedOdometry);
        rangeReadings.emplace_back(observedRange);
    }
}

// print state
void printStates(const vector<double>& odometryValues, const vector<double>& rangeReadings) {
    CHECK_EQ(odometryValues.size(), rangeReadings.size());
    double robotLocation{0.0};
    cout << "pose:  location odometry  range    o.error  r.error" << endl;
    for (size_t i = 0; i < odometryValues.size(); ++i) {
        robotLocation += odometryValues[i];
        double odometryError = FLAGS_poseSeparation - odometryValues[i];
        double rangeError = robotLocation + rangeReadings[i] - FLAGS_corridorLength;
        printf("%4d: %8.3f %8.3f %8.3f %8.3f %8.3f\n", static_cast<int>(i), robotLocation, odometryValues[i],
               rangeReadings[i], odometryError, rangeError);
    }
}

// Odometry Constraints
struct OdometryConstraints {
    using OdometryCostFunction = ceres::AutoDiffCostFunction<OdometryConstraints, 1, 1>;

    OdometryConstraints(const double& odometryMean, const double& odometryStddev)
        : odometryMean(odometryMean), odometryStddev(odometryStddev) {}

    template <typename T>
    bool operator()(const T* const odometry, T* residual) const {
        *residual = (*odometry - odometryMean) / odometryStddev;
        return true;
    }

    static OdometryCostFunction* create(const double& odometryValue) {
        return new OdometryCostFunction(new OdometryConstraints(odometryValue, FLAGS_odometryStddev));
    }

    const double odometryMean;
    const double odometryStddev;
};

// Range Constraints
struct RangeConstraints {
    // the stride length of the dDynamicAutoDiffCostFunction evaluator
    // static const int kStride{10};
    using RangeCostFunction = DynamicAutoDiffCostFunction<RangeConstraints, 10>;

    RangeConstraints(int poseIndex, const double& rangeReading, const double& rangeStddev, const double corridorLength)
        : poseIdx(poseIndex), rangeReading(rangeReading), rangeStddev(rangeStddev), corridorLength(corridorLength) {}

    template <typename T>
    bool operator()(T const* const* relativePoses, T* residuals) const {
        T globalPose(0);
        for (int i = 0; i <= poseIdx; ++i) {
            globalPose += relativePoses[i][0];
        }
        residuals[0] = (globalPose + rangeReading - corridorLength) / rangeStddev;
        return true;
    }

    static RangeCostFunction* create(int poseIndex, const double& rangeReading, vector<double>* odometryValues,
                                     vector<double*>* parameterBlocks) {
        RangeConstraints* constraints =
            new RangeConstraints(poseIndex, rangeReading, FLAGS_rangeStddev, FLAGS_corridorLength);
        RangeCostFunction* costFunction = new RangeCostFunction(constraints);

        // add all the parameter blocks that affect this constraint
        parameterBlocks->clear();
        for (int i = 0; i <= poseIndex; ++i) {
            parameterBlocks->push_back(&((*odometryValues)[i]));
            costFunction->AddParameterBlock(1);
        }
        costFunction->SetNumResiduals(1);
        return costFunction;
    }

    const int poseIdx;
    const double rangeReading;
    const double rangeStddev;
    const double corridorLength;
};

int main(int argc, char* argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    // make sure that the arguments parsed are all positive
    CHECK_GT(FLAGS_corridorLength, 0.0);
    CHECK_GT(FLAGS_poseSeparation, 0.0);
    CHECK_GT(FLAGS_odometryStddev, 0.0);
    CHECK_GT(FLAGS_rangeStddev, 0.0);

    // simulate data
    vector<double> odometryValues;
    vector<double> rangeReadings;
    simulateRobot(odometryValues, rangeReadings);

    // print initial values
    cout << Section("Initial Values") << endl;
    printStates(odometryValues, rangeReadings);

    // build problem
    Problem problem;
    for (int i = 0; i < odometryValues.size(); ++i) {
        // create and add a DynamicAutoDiffCostFunction for the RangeConstraint from pose i
        vector<double*> parameterBlock;
        RangeConstraints::RangeCostFunction* rangeCostFunction =
            RangeConstraints::create(i, rangeReadings[i], &odometryValues, &parameterBlock);
        problem.AddResidualBlock(rangeCostFunction, nullptr, parameterBlock);

        // create and add an AutoDiffCostFunction for the OdometryConstraint for pose i
        problem.AddResidualBlock(OdometryConstraints::create(odometryValues[i]), nullptr, &(odometryValues[i]));
    }

    // solve
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    cout << summary.FullReport() << endl;
    cout << Section("Final Values") << endl;
    printStates(odometryValues, rangeReadings);

    google::ShutdownGoogleLogging();
    google::ShutDownCommandLineFlags();
    return 0;
}