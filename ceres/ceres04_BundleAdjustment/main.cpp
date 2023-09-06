#include <ceres/ceres.h>
#include <ceres/rotation.h>
using namespace std;

// Read a bundle adjustment in the large dataset
class BALProblem {
  public:
    BALProblem() = default;
    ~BALProblem() {
        delete[] pointIndex_;
        delete[] cameraIndex_;
        delete[] observations_;
        delete[] parameters_;
    }

    int numObservations() const { return numObservations_; }
    const double* observation() const { return observations_; }
    double* mutableCameras() { return parameters_; }
    double* mutablePoints() { return parameters_ + 9 * numCameras_; }

    double* mutableCameraForObservation(size_t i) { return mutableCameras() + cameraIndex_[i] * 9; }
    double* mutablePointForObservation(size_t i) { return mutablePoints() + pointIndex_[i] * 3; }

    bool loadFile(const char* filename) {
        FILE* fptr = fopen(filename, "r");
        if (fptr == nullptr) {
            return false;
        }

        fscanfOrDie(fptr, "%d", &numCameras_);
        fscanfOrDie(fptr, "%d", &numPoints_);
        fscanfOrDie(fptr, "%d", &numObservations_);

        cameraIndex_ = new int[static_cast<size_t>(numObservations_)];
        pointIndex_ = new int[static_cast<size_t>(numObservations_)];
        observations_ = new double[2 * static_cast<size_t>(numObservations_)];
        numParameters_ = 9 * numCameras_ + 3 * numPoints_;
        parameters_ = new double[static_cast<size_t>(numParameters_)];

        for (int i = 0; i < numObservations_; ++i) {
            fscanfOrDie(fptr, "%d", cameraIndex_ + i);
            fscanfOrDie(fptr, "%d", pointIndex_ + i);
            for (int j = 0; j < 2; ++j) {
                fscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
            }
        }

        for (int i = 0; i < numParameters_; ++i) {
            fscanfOrDie(fptr, "%lf", parameters_ + i);
        }

        return true;
    }

  private:
    template <typename T>
    void fscanfOrDie(FILE* fptr, const char* format, T* value) {
        int numScanned = fscanf(fptr, format, value);
        if (numScanned != 1) {
            LOG(FATAL) << "Invalid UW data file.";
        }
    }

  private:
    int numCameras_;
    int numPoints_;
    int numObservations_;
    int numParameters_;

    int* cameraIndex_;
    int* pointIndex_;
    double* observations_;
    double* parameters_;
};

// Templated pinhole camera model for used with Ceres. The camera is parameterized using 9 parameters: 3 for rotation, 3
// for translation, 1 for focal length and 2 for radical distortion. The principal point is not modeled (i.e. it is
// assumed be located at the image center)
struct SnavelyReprojectionError {
    SnavelyReprojectionError(const double& observedX, const double& observedY)
        : observedX_(observedX), observedY_(observedY) {}

    template <typename T>
    bool operator()(const T* const camera, const T* const point, T* residuals) const {
        // camera[0, 1, 2] are the angle-axis rotation
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);

        // camera[3, 4, 5] are the translation
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // compute the center of distortion, the sign change comes from the camera model that Noah Snavely's Bundler
        // assumes, whereby the camera coordinate systems has a negative z axis
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];

        // compute final projected point position with second and fourth order radical distortion
        // x = f * x * (1 + k2 * r^2 + k4 * r^4) + cx
        // y = f * y * (1 + k2 * r^2 + k4 * r^4) + cy
        const T& k2 = camera[7];
        const T& k4 = camera[8];
        T r2 = xp * xp + yp * yp;
        T distortion = 1.0 + r2 * (k2 + k4 * r2);
        const T& focal = camera[6];
        T predictedX = focal * distortion * xp;
        T predictedY = focal * distortion * yp;

        // the error is the difference between the predicted and observed position
        residuals[0] = predictedX - observedX_;
        residuals[1] = predictedY - observedY_;

        return true;
    }

    // Factory to hide the construction of the CostFunction object from the client code
    static ceres::CostFunction* Create(const double observedX, const double observedY) {
        return new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
            new SnavelyReprojectionError(observedX, observedY));
    }

    double observedX_;
    double observedY_;
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    BALProblem balProblem;
    if (!balProblem.loadFile("./ceres/data/problem-49-7776-pre.txt")) {
        cerr << "ERROR: unable to open file." << endl;
        return -1;
    }

    const double* observations = balProblem.observation();

    // create residuals for each observation in the bundle adjustment problem. The parameters for cameras and points are
    // added automatically.
    ceres::Problem problem;
    for (size_t i = 0; i < static_cast<size_t>(balProblem.numObservations()); ++i) {
        // Each residual block takes a point and a camera as input and outputs a dimensional residual. Internally, the
        // cost function stores the observed image location and compares the reprojection against the observation.
        ceres::CostFunction* costFunction =
            SnavelyReprojectionError::Create(observations[2 * i + 0], observations[2 * i + 1]);
        problem.AddResidualBlock(costFunction, nullptr, balProblem.mutableCameraForObservation(i),
                                 balProblem.mutablePointForObservation(i));
    }

    // Make Ceres automatically detect the bundle structure. Note that the standard solver, SPARSE_NORMAL_CHOLESKY,
    // also works fine but it is slower for standard bundle adjustment problems
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;

    google::ShutdownGoogleLogging();
    return 0;
}
