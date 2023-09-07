/**
 * Example Contains Bundle Adjustment used in libmv and Blender. It reads problems from files pass via the command line
 * and runs the bundle adjuster on the problem.
 */
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <common/common.hpp>
#include <fstream>

using namespace std;
using namespace ceres;
using namespace Eigen;
using namespace common;

DEFINE_string(fileName, "./ceres/data/libmv-ba-problems/problem_02.bin", "Input file name");
DEFINE_string(refineIntrinsics, "all", "camera intrinsics to be refined. Options are: '', 'none', 'radial', 'all'");

using Vector6d = Eigen::Matrix<double, 6, 1>;

/**
 * @brief The location and rotation of the camera viewing an image
 */
struct EuclideanCamera {
    EuclideanCamera() : imageIdx(-1) {}
    EuclideanCamera(const EuclideanCamera& c) = default;  //: imageIdx(c.imageIdx), R(c.R), t(c.t) {}

    int imageIdx;  // which image this camera represents
    Matrix3d R;    // 3x3 matrix representing the rotation of the camera
    Vector3d t;    // translation vector representing its positions
};

/**
 * @brief 3D location of a track(ie. features)
 */
struct EuclideanPoint {
    EuclideanPoint() : trackIdx(-1) {}
    EuclideanPoint(const EuclideanPoint& p) = default;  //: trackIdx(p.trackIdx), x(p.x) {}

    int trackIdx;  // which track this point corresponds to
    Vector3d x;    // 3D position of the track
};

/**
 * @brief 2D location of a tracked point in an image. All markers to the same target form a track identified by a
 * common track number
 */
struct Marker {
    int imageIdx;  // image index
    int trackIdx;  // track index
    double x;      // x position of the marker in pixels
    double y;      // y position of the marker in pixels
};

/**
 * @brief Camera intrinsics to be bundled
 */
enum BundleIntrinsics {
    BUNDLE_NO_INTRINSICS = 0b000000,
    BUNDLE_FOCAL_LENGTH = 0b000001,     // f
    BUNDLE_PRINCIPAL_POINT = 0b000010,  // cx, cy
    BUNDLE_RADIAL_K1 = 0b000100,        // k1
    BUNDLE_RADIAL_K2 = 0b001000,        // k2
    BUNDLE_RADIAL = 0b001100,           // k1 and k2
    BUNDLE_TANGENTIAL_P1 = 0b010000,    // p1
    BUNDLE_TANGENTIAL_P2 = 0b100000,    // p2
    BUNDLE_TANGENTIAL = 0b110000,       // p1 and p2
};

/**
 * @brief Which blocks to keep constant during bundling
 */
enum BundleConstraints {
    BUNDLE_NO_CONSTRAINTS = 0,
    BUNDLE_NO_TRANSLATION = 1,
};

/**
 * @brief The intrinsics need to get combined into a single parameter block, use these enums to index instead of
 * numeric constants
 */
enum {
    OFFSET_F,
    OFFSET_CX,
    OFFSET_CY,
    OFFSET_K1,
    OFFSET_K2,
    OFFSET_K3,
    OFFSET_P1,
    OFFSET_P2,
};

// return a pointer to the camera corresponding to a image
EuclideanCamera* cameraForImage(std::vector<EuclideanCamera>& cameras, int imageIdx) {
    if (imageIdx < 0 || imageIdx >= cameras.size()) {
        return nullptr;
    }

    EuclideanCamera* camera = &cameras[imageIdx];
    if (camera->imageIdx == -1) {
        return nullptr;
    }
    return camera;
}

const EuclideanCamera* cameraForImage(const std::vector<EuclideanCamera>& cameras, int imageIdx) {
    if (imageIdx < 0 || imageIdx >= cameras.size()) {
        return nullptr;
    }

    const EuclideanCamera* camera = &cameras[imageIdx];
    if (camera->imageIdx == -1) {
        return nullptr;
    }
    return camera;
}

// returns maximal image number at which marker exists
int maxImage(const vector<Marker>& markers) {
    if (markers.empty()) {
        return -1;
    }

    int ret = markers[0].imageIdx;
    for (auto& m : markers) {
        if (ret < m.imageIdx) {
            ret = m.imageIdx;
        }
    }
    return ret;
}

// return a pointer to the point corresponding to a track
EuclideanPoint* pointForTrack(vector<EuclideanPoint>& points, int trackIdx) {
    if (trackIdx < 0 || trackIdx >= points.size()) {
        return nullptr;
    }

    EuclideanPoint* point = &points[trackIdx];
    if (point->trackIdx == -1) {
        return nullptr;
    }
    return point;
}

// Reader of binary file which makes sure possibly needed endian conversion happens when loading values like floats and
// integers.
//
// File's endian type is reading from a first character of file, which could either be V for big endian or v for little
// endian. This means you need to design file format assuming first character denotes file endianness in this way.
class EndianAwareFileReader {
  public:
    EndianAwareFileReader() : file_(nullptr) {
        // Get an endian type of the host machine.
        union {
            unsigned char bytes[4];
            uint32_t value;
        } endian_test = {{0, 1, 2, 3}};
        host_endian_type_ = endian_test.value;
        file_endian_type_ = host_endian_type_;
    }

    ~EndianAwareFileReader() {
        if (file_ != nullptr) {
            fclose(file_);
        }
    }

    bool openFile(const std::string& file_name) {
        file_ = fopen(file_name.c_str(), "rb");
        if (file_ == nullptr) {
            return false;
        }
        // Get an endian type of data in the file.
        unsigned char file_endian_type_flag = read<unsigned char>();
        if (file_endian_type_flag == 'V') {
            file_endian_type_ = kBigEndian;
        } else if (file_endian_type_flag == 'v') {
            file_endian_type_ = kLittleEndian;
        } else {
            LOG(FATAL) << "Problem file is stored in unknown endian type.";
        }
        return true;
    }

    // read value from the file, will switch endian if needed.
    template <typename T>
    T read() const {
        T value;
        CHECK_GT(fread(&value, sizeof(value), 1, file_), 0);
        // Switch endian type if file contains data in different type
        // that current machine.
        if (file_endian_type_ != host_endian_type_) {
            value = SwitchEndian<T>(value);
        }
        return value;
    }

  private:
    static const long int kLittleEndian = 0x03020100ul;
    static const long int kBigEndian = 0x00010203ul;

    // Switch endian type between big to little.
    template <typename T>
    T SwitchEndian(const T value) const {
        if (sizeof(T) == 4) {
            unsigned int temp_value = static_cast<unsigned int>(value);
            return ((temp_value >> 24)) | ((temp_value << 8) & 0x00ff0000) | ((temp_value >> 8) & 0x0000ff00) |
                   ((temp_value << 24));
        } else if (sizeof(T) == 1) {
            return value;
        } else {
            LOG(FATAL) << "Entered non-implemented part of endian switching function.";
        }
    }

    FILE* file_;
    int host_endian_type_;
    int file_endian_type_;
};

// read 3x3 column-major matrix from the file
Matrix3d ReadMatrix3d(const EndianAwareFileReader& fileReader) {
    Matrix3d mat;
    for (int i = 0; i < 9; i++) {
        mat(i % 3, i / 3) = fileReader.read<float>();
    }
    return mat;
}

// read 3-vector from file
Vector3d ReadVector3d(const EndianAwareFileReader& fileReader) {
    Vector3d vec;
    for (int i = 0; i < 3; i++) {
        vec(i) = fileReader.read<float>();
    }
    return vec;
}

/**
 * @brief Reads a bundle adjustment problem from the file
 * @param fileName          Denotes from which file to read the problem
 * @param isImageSpace      Track point is image space(in pixel) or normalized image space
 * @param cameraIntrinsics  Contain initial camera intrinsics values
 * @param cameras           A vector of all reconstructed cameras to be optimized, vector element with number i will
 * contain camera for image i
 * @param points            A vector of all reconstructed 3D points to be optimized, vector element with number i will
 * contain point for track i
 * @param markers           A vector of all tracked markers existing in the problem. Only used for reprojection error
 * calculation, stay unchanged during optimization.
 * @return  False if any kind of error happened during reading.
 */
bool readProblemFromFile(const std::string& fileName, bool& isImageSpace, double cameraIntrinsics[8],
                         vector<EuclideanCamera>& cameras, vector<EuclideanPoint>& points, vector<Marker>& markers) {
    EndianAwareFileReader fileReader;
    if (!fileReader.openFile(fileName)) {
        return false;
    }

    // read markers' space flag.
    unsigned char isImageSpaceFlag = fileReader.read<unsigned char>();
    if (isImageSpaceFlag == 'P') {
        isImageSpace = true;
    } else if (isImageSpaceFlag == 'N') {
        isImageSpace = false;
    } else {
        LOG(FATAL) << "Problem file contains markers stored in unknown space.";
    }

    // read camera intrinsics.
    for (int i = 0; i < 8; i++) {
        cameraIntrinsics[i] = fileReader.read<float>();
    }

    // read all cameras.
    cameras.clear();
    int numberOfCameras = fileReader.read<int>();
    for (int i = 0; i < numberOfCameras; i++) {
        EuclideanCamera camera;
        camera.imageIdx = fileReader.read<int>();
        camera.R = ReadMatrix3d(fileReader);
        camera.t = ReadVector3d(fileReader);

        if (camera.imageIdx >= cameras.size()) {
            cameras.resize(static_cast<size_t>(camera.imageIdx + 1));
        }
        cameras[camera.imageIdx] = camera;
    }
    LOG(INFO) << "read " << numberOfCameras << " cameras.";

    // read all reconstructed 3D points
    points.clear();
    int numberOfPoints = fileReader.read<int>();
    for (int i = 0; i < numberOfPoints; i++) {
        EuclideanPoint point;
        point.trackIdx = fileReader.read<int>();
        point.x = ReadVector3d(fileReader);

        if (point.trackIdx >= points.size()) {
            points.resize(static_cast<size_t>(point.trackIdx + 1));
        }

        points[point.trackIdx] = point;
    }
    LOG(INFO) << "read " << numberOfPoints << " points.";

    // and finally read all markers
    markers.clear();
    int numberOfMarkers = fileReader.read<int>();
    for (int i = 0; i < numberOfMarkers; i++) {
        Marker marker;
        marker.imageIdx = fileReader.read<int>();
        marker.trackIdx = fileReader.read<int>();
        marker.x = fileReader.read<float>();
        marker.y = fileReader.read<float>();

        markers.emplace_back(marker);
    }
    LOG(INFO) << "read " << numberOfMarkers << " markers.";

    return true;
}

// apply camera intrinsics to the normalized point to get image coordinates
template <typename T>
void applyRadialDistortionCameraIntrinsics(const T& fx, const T& fy, const T& cx, const T& cy, const T& k1, const T& k2,
                                           const T& k3, const T& p1, const T& p2, const T& normX, const T& normY, T& x,
                                           T& y) {
    T r2 = normX * normX + normY * normY;
    T r4 = r2 * r2;
    T r6 = r4 * r2;
    T rCoeff = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    T xd = normX * rCoeff + 2.0 * p1 * normX * normY + p2 * (r2 + 2.0 * normX * normX);
    T yd = normY * rCoeff + 2.0 * p2 * normX * normY + p1 * (r2 + 2.0 * normY * normY);
    x = fx * xd + cx;
    y = fy * yd + cy;
}

// Cost functor which computes reprojection error of 3D point X on camera defined by angle-axis rotation and its
// translation
struct ReprojectionError {
    ReprojectionError(const double& x, const double& y) : x_(x), y_(y) {}

    // Rt: rotation denoted by angle axis followed with translation
    // X: point coordinates 3x1
    template <typename T>
    bool operator()(const T* const intrinsics, const T* const Rt, const T* const X, T* residuals) const {
        // unpack the intrinsics
        const T& f = intrinsics[OFFSET_F];
        const T& cx = intrinsics[OFFSET_CX];
        const T& cy = intrinsics[OFFSET_CY];
        const T& k1 = intrinsics[OFFSET_K1];
        const T& k2 = intrinsics[OFFSET_K2];
        const T& k3 = intrinsics[OFFSET_K3];
        const T& p1 = intrinsics[OFFSET_P1];
        const T& p2 = intrinsics[OFFSET_P2];

        // compute projective coordinates x = R * X + t
        T x[3];
        AngleAxisRotatePoint(Rt, X, x);  // x = R * X
        x[0] += Rt[3];                   // x = x + t
        x[1] += Rt[4];
        x[2] += Rt[5];

        // compute normalized coordinates: x = x / x.z
        T xn = x[0] / x[2];
        T yn = x[1] / x[2];

        // apply distortion to the normalized points to get (xd, yd)
        T predictedX, predictedY;
        applyRadialDistortionCameraIntrinsics(f, f, cx, cy, k1, k2, k3, p1, p2, xn, yn, predictedX, predictedY);

        // the error is the difference between the predicted and observed position
        residuals[0] = predictedX - x_;
        residuals[1] = predictedY - y_;

        return true;
    }

  private:
    const double x_;  // observation in x
    const double y_;  // observation in y
};

// print a log message containing which camera intrinsics are gonna to be optimized
void logBundleCameraIntrinsics(const int bundleIntrinsics) {
    if (bundleIntrinsics == BUNDLE_NO_INTRINSICS) {
        LOG(INFO) << "Bundling only camera positions";
    } else {
        string str;
#define APPEND_BUNDLING_INTRINSICS(name, flag) \
    if (bundleIntrinsics & flag) {             \
        if (!str.empty()) {                    \
            str += ", ";                       \
        }                                      \
        str += name;                           \
    }

        APPEND_BUNDLING_INTRINSICS("f", BUNDLE_FOCAL_LENGTH);
        APPEND_BUNDLING_INTRINSICS("px, py", BUNDLE_PRINCIPAL_POINT);
        APPEND_BUNDLING_INTRINSICS("k1", BUNDLE_RADIAL_K1);
        APPEND_BUNDLING_INTRINSICS("k2", BUNDLE_RADIAL_K2);
        APPEND_BUNDLING_INTRINSICS("p1", BUNDLE_TANGENTIAL_P1);
        APPEND_BUNDLING_INTRINSICS("p2", BUNDLE_TANGENTIAL_P2);
#undef APPEND_BUNDLING_INTRINSICS

        LOG(INFO) << "Bundling: " << str;
    }
}

// print a log message containing all the camera intrinsics values
void logCameraIntrinsics(const string& text, const double* cameraIntrinsics) {
    stringstream strStream;
    strStream << "f = " << cameraIntrinsics[OFFSET_F] << ", cx = " << cameraIntrinsics[OFFSET_CX]
              << ", cy = " << cameraIntrinsics[OFFSET_CY];

#define APPEND_DISTORTION_COEFF(name, offset)                     \
    if (cameraIntrinsics[offset] != 0.0) {                        \
        strStream << ", " name " = " << cameraIntrinsics[offset]; \
    }

    APPEND_DISTORTION_COEFF("k1", OFFSET_K1);
    APPEND_DISTORTION_COEFF("k2", OFFSET_K2);
    APPEND_DISTORTION_COEFF("k3", OFFSET_K3);
    APPEND_DISTORTION_COEFF("p1", OFFSET_P1);
    APPEND_DISTORTION_COEFF("p2", OFFSET_P2);

#undef APPEND_DISTORTION_COEFF
    LOG(INFO) << text << strStream.str();
}

// get a vector of camera's rotations denoted by angle axis conjuncted with translations into single block. Element with
// index i matches to a rotation+translation for camera at image i
vector<Vector6d> packCamerasRotationTranslation(const vector<Marker>& markers, const vector<EuclideanCamera>& cameras) {
    const int kMaxImage = maxImage(markers);
    vector<Vector6d> Rts(kMaxImage + 1);

    for (int i = 0; i <= kMaxImage; ++i) {
        const EuclideanCamera* camera = cameraForImage(cameras, i);
        if (camera == nullptr) continue;

        RotationMatrixToAngleAxis(&camera->R(0, 0), &Rts[i](0));  // R => so3
        Rts[i].tail<3>() = camera->t;                             // t
    }
    return Rts;
}

// convert camera rotations for angle axis back to rotation matrix
void unpackCamerasRotationTranslation(const vector<Marker>& markers, const vector<Vector6d>& cameraRts,
                                      vector<EuclideanCamera>& cameras) {
    const int kMaxImage = maxImage(markers);
    for (int i = 0; i <= kMaxImage; ++i) {
        EuclideanCamera* camera = cameraForImage(cameras, i);
        if (camera == nullptr) {
            continue;
        }

        AngleAxisToRotationMatrix(&cameraRts[i](0), &camera->R(0, 0));  // so3 => R
        camera->t = cameraRts[i].tail<3>();
    }
}

void euclideanBundleCommonIntrinsics(const vector<Marker>& markers, int bundleIntrinsics, int bundleConstraints,
                                     double* cameraIntrinsics, vector<EuclideanCamera>& cameras,
                                     vector<EuclideanPoint>& points) {
    logCameraIntrinsics("Origin Intrinsics: ", cameraIntrinsics);

    Problem::Options problemOptions;
    Problem problem(problemOptions);

    // convert cameras rotation to angle axis and merge with translation into single parameter block for maximal
    // minimization speed.
    // Block for minimization has got the following structure: <3 elements for angle-axis> <3 elements for translations>
    vector<Vector6d> cameraRts = packCamerasRotationTranslation(markers, cameras);

    // manifold used to restrict camera motion for modal solvers
    SubsetManifold* constantTransformManifold{nullptr};
    if (bundleConstraints & BUNDLE_NO_TRANSLATION) {
        // first 3 elements are rotation, last three are translation
        vector<int> constTranslations{3, 4, 5};
        constantTransformManifold = new SubsetManifold(6, constTranslations);
    }

    int residualsNum{0};
    bool haveLockedCamera{false};
    for (const auto& m : markers) {
        EuclideanCamera* camera = cameraForImage(cameras, m.imageIdx);
        EuclideanPoint* point = pointForTrack(points, m.trackIdx);
        if (camera == nullptr || point == nullptr) {
            continue;
        }

        // rotation of camera denoted in angle axis followed with camera translation
        double* currentCameraRt = &cameraRts[camera->imageIdx](0);

        problem.AddResidualBlock(
            new AutoDiffCostFunction<ReprojectionError, 2, 8, 6, 3>(new ReprojectionError(m.x, m.y)), nullptr,
            cameraIntrinsics, currentCameraRt, &point->x(0));

        // we lock the first camera to better deal with scene orientation ambiguity
        if (!haveLockedCamera) {
            problem.SetParameterBlockConstant(currentCameraRt);
            haveLockedCamera = true;
        }

        if (bundleConstraints & BUNDLE_NO_TRANSLATION) {
            problem.SetManifold(currentCameraRt, constantTransformManifold);
        }
        ++residualsNum;
    }
    LOG(INFO) << "Number of residuals: " << residualsNum;

    if (0 == residualsNum) {
        LOG(INFO) << "Skipping running minimizer with zero residuals";
        return;
    }

    logBundleCameraIntrinsics(bundleIntrinsics);

    if (bundleIntrinsics == BUNDLE_NO_CONSTRAINTS) {
        // no camera intrinsics are being refined, set the whole parameter block as constant for best performance
        problem.SetParameterBlockConstant(cameraIntrinsics);
    } else {
        // set the camera intrinsics that are not to be bundled as constant using some macro trickery
        vector<int> constantIntrinsics;
#define MAYBE_SET_CONSTANT(bundleEnum, offset)   \
    if (!(bundleIntrinsics & bundleEnum)) {      \
        constantIntrinsics.emplace_back(offset); \
    }

        MAYBE_SET_CONSTANT(BUNDLE_FOCAL_LENGTH, OFFSET_F);
        MAYBE_SET_CONSTANT(BUNDLE_PRINCIPAL_POINT, OFFSET_CX);
        MAYBE_SET_CONSTANT(BUNDLE_PRINCIPAL_POINT, OFFSET_CY);
        MAYBE_SET_CONSTANT(BUNDLE_RADIAL_K1, OFFSET_K1);
        MAYBE_SET_CONSTANT(BUNDLE_RADIAL_K2, OFFSET_K2);
        MAYBE_SET_CONSTANT(BUNDLE_TANGENTIAL_P1, OFFSET_P1);
        MAYBE_SET_CONSTANT(BUNDLE_TANGENTIAL_P2, OFFSET_P2);

#undef MAYBE_SET_CONSTANT

        // always set k3 constant, it's not used at the moment
        constantIntrinsics.emplace_back(OFFSET_K3);

        auto subsetManifold = new SubsetManifold(8, constantIntrinsics);
        problem.SetManifold(cameraIntrinsics, subsetManifold);
    }

    // configure the solver
    Solver::Options solverOptions;
    solverOptions.use_nonmonotonic_steps = true;
    solverOptions.preconditioner_type = SCHUR_JACOBI;
    solverOptions.linear_solver_type = ITERATIVE_SCHUR;
    solverOptions.use_inner_iterations = true;
    solverOptions.max_num_iterations = 100;
    solverOptions.minimizer_progress_to_stdout = true;

    // solve
    Solver::Summary summary;
    ceres::Solve(solverOptions, &problem, &summary);
    cout << "Final Report:" << endl << summary.FullReport();

    // copy rotations and translations back
    unpackCamerasRotationTranslation(markers, cameraRts, cameras);

    logCameraIntrinsics("Final Intrinsics: ", cameraIntrinsics);
}

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    if (FLAGS_fileName.empty()) {
        LOG(ERROR) << "input file name cannot be empty";
        return -1;
    }

    // read data
    double cameraIntrinsics[8];
    bool isImageSpace{false};
    vector<EuclideanCamera> cameras;
    vector<EuclideanPoint> points;
    vector<Marker> markers;
    if (!readProblemFromFile(FLAGS_fileName, isImageSpace, cameraIntrinsics, cameras, points, markers)) {
        LOG(ERROR) << "reading problem file fail";
        return -1;
    }

    // using command line argument refine intrinsics will explicitly declare with intrinsics need to be refined and
    // in this case refining flags does not depend on problem at all
    int bundleIntrinsics = BUNDLE_NO_INTRINSICS;
    if (FLAGS_refineIntrinsics.empty()) {
        if (isImageSpace) {
            bundleIntrinsics = BUNDLE_FOCAL_LENGTH | BUNDLE_RADIAL;
        }
    } else {
        if (FLAGS_refineIntrinsics == "none") {
            bundleIntrinsics = BUNDLE_NO_INTRINSICS;
        } else if (FLAGS_refineIntrinsics == "radial") {
            bundleIntrinsics = BUNDLE_FOCAL_LENGTH | BUNDLE_RADIAL;
        } else if (FLAGS_refineIntrinsics == "all") {
            bundleIntrinsics = BUNDLE_FOCAL_LENGTH | BUNDLE_PRINCIPAL_POINT | BUNDLE_RADIAL | BUNDLE_TANGENTIAL;
        } else {
            LOG(ERROR) << "unsupported value for refine intrinsics";
            return -1;
        }
    }

    // run the bundler
    euclideanBundleCommonIntrinsics(markers, bundleIntrinsics, BUNDLE_NO_CONSTRAINTS, cameraIntrinsics, cameras,
                                    points);

    google::ShutdownGoogleLogging();
    google::ShutDownCommandLineFlags();
    return 0;
}
