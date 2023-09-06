#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sophus/so3.hpp>
#include <common/common.hpp>
#include "opencv2/calib3d.hpp"
#include "unsupported/Eigen/EulerAngles"

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace common;

void basic01() {
    // euler angle
    double yaw{0.5}, pitch{0.8}, roll{1.0};
    Vector3d angle(roll, pitch, yaw);
    cout << "angle = [Rx, Ry, Rz] = [roll, pitch, yaw] = " << angle.transpose() << endl;

    // rotation sequence ZYX, R = Rz * Ry * Rx, Matlab's default
    cout << endl << Section("Rotation Sequence: ZYX, R = Rz * Ry * Rx") << endl;
    // Euler Angle => Rotation Matrix
    Matrix3d R1;
    R1 = AngleAxisd(angle[2], Vector3d::UnitZ()) * AngleAxisd(angle[1], Vector3d::UnitY()) *
         AngleAxisd(angle[0], Vector3d::UnitX());
    cout << "R1 = " << endl << R1 << endl;
    Vector3d euler1 = R1.eulerAngles(2, 1, 0);
    cout << "euler1 = " << euler1.transpose() << endl;

    // rotation sequence XYZ, R = Rx * Ry * Rz
    cout << endl << Section("Rotation Sequence: XYZ, R = Rx * Ry * Rz") << endl;
    // Euler Angle => Rotation Matrix
    Matrix3d R2;
    R2 = AngleAxisd(angle[0], Vector3d::UnitX()) * AngleAxisd(angle[1], Vector3d::UnitY()) *
         AngleAxisd(angle[2], Vector3d::UnitZ());
    cout << "R2 = " << endl << R2 << endl;
    // Rotation Matrix => Euler Angle
    Vector3d euler2 = R2.eulerAngles(0, 1, 2);
    cout << "euler2 = " << euler2.transpose() << endl;

    // SO3
    cout << endl << Section("SO3 and Rotation Vector(Rodrigues Formula)") << endl;
    // to SO3
    Sophus::SO3d SO3(R1);
    cout << "SO3 = " << endl << SO3.matrix() << endl;
    // Rotation Vector
    cout << "so3 = " << SO3.log().transpose() << endl;
}

void test02() {
    cout << endl << Section("Test") << endl;
    Matrix3d tempR = Matrix3d::Identity();
    cout << "tempR = " << endl << tempR << endl;
    Vector3d tempAngle = tempR.eulerAngles(2, 1, 0);
    cout << "tempAngle = " << tempAngle.transpose() << endl;

    Vector3d x1(-0.0104279, -0.0143366, -0.00132262);
    Quaterniond q1 = AngleAxisd(x1[2], Vector3d::UnitZ()) * AngleAxisd(x1[1], Vector3d::UnitY()) *
                     AngleAxisd(x1[0], Vector3d::UnitX());
    Matrix3d R1;
    R1 = AngleAxisd(x1[2], Vector3d::UnitZ()) * AngleAxisd(x1[1], Vector3d::UnitY()) *
         AngleAxisd(x1[0], Vector3d::UnitX());
    Vector3d x1b = R1.eulerAngles(2, 1, 0);
    Eigen::AngleAxisd x1c(R1);
    cout << "x1 = " << x1.transpose() << endl;
    cout << "R1 = " << endl << R1 << endl;
    cout << "x1b = " << x1b.transpose() << endl;
    cout << "x1c = " << x1c.angle() << ".\t" << x1c.axis().transpose() << endl;
    cout << "angle of q = " << Sophus::SO3d(q1).log().transpose() << endl;
    Matrix3d R2;
    R2 = AngleAxisd(x1[0], Vector3d::UnitX()) * AngleAxisd(x1[1], Vector3d::UnitY()) *
         AngleAxisd(x1[2], Vector3d::UnitZ());
    Vector3d x2b = R2.eulerAngles(2, 1, 0);
    cout << "R2 = " << endl << R2 << endl;
    cout << "x2b = " << x2b.transpose() << endl;
}

/**
 * @brief Calculate euler angle from rotation matrix. [Rx, Ry, Rz] = [roll, pitch, yaw]
 * @param R Rotation matrix
 * @return  Euler angle
 */
Vector3d eulerAngleFromRotationMatrix(const Matrix3d& R) {
    Vector3d angle;
    angle[2] = atan2(R(1, 0), R(0, 0));  // yaw
    angle[1] = asin(-R(2, 0));           // pitch
    angle[0] = atan2(R(2, 1), R(2, 2));  // roll
    return angle;
}

/**
 * @brief Conversion between rotation matrix and angle vector
 */
void RotMatAngleConversion() {
    // XYZ in extrinsic coordinates, R = Rz * Ry * Rx
    cout << Section("Geometry using Eigen::EulerAngles") << endl;

    // euler angle = [Rx, Ry, Rz] = [roll, pitch, yaw], below 2 angles result same rotation matrix
    Vector3d x0a(-0.0104279, -0.0143366, -0.00132262);
    Vector3d x0b(3.131164754, -3.127256054, 3.140270034);
    Vector3d x0 = x0a;
    cout << "raw angle x0a = " << x0a.transpose() << endl;
    cout << "raw angle x0b = " << x0b.transpose() << endl;

    // Rotation matrix
    cout << SubSection("Rotation Matrix") << endl;
    Matrix3d R1 = (AngleAxisd(x0[2], Vector3d::UnitZ()) * AngleAxisd(x0[1], Vector3d::UnitY()) *
                   AngleAxisd(x0[0], Vector3d::UnitX()))
                      .toRotationMatrix();
    Vector3d x1 = R1.eulerAngles(2, 1, 0);
    cout << "R1 = " << endl << R1 << endl;
    cout << "x1 = " << x1.transpose() << endl;

    // Euler angle A: constructed from angle
    cout << SubSection("Euler Angles") << endl;
    cout << Paragraph("02a") << endl;
    EulerAnglesZYXd x02a(x0[2], x0[1], x0[0]);
    cout << "R = " << x02a.matrix() << endl;
    cout << "x = " << x02a.angles().transpose() << endl;

    // Euler angle B: constructed from rotation matrix
    cout << Paragraph("02b") << endl;
    EulerAnglesZYXd x02b(R1);
    cout << "R = " << x02b.matrix() << endl;
    cout << "x = " << x02b.angles().transpose() << endl;

    // Euler angle C: constructed from rotation matrix
    cout << Paragraph("02c") << endl;
    // EulerAnglesZYXd x02c = EulerAnglesZYXd::FromRotation<false, false, false>(R1);
    EulerAnglesZYXd x02c(R1);
    cout << "R = " << x02c.toRotationMatrix() << endl;
    cout << "x = " << x02c.angles().transpose() << endl;

    // user-define function
    cout << SubSection("User-Defined Function") << endl;
    Vector3d x03a = eulerAngleFromRotationMatrix(R1);
    cout << "x03a = " << x03a.transpose() << endl;

    // lie groups and lie algebra
    cout << SubSection("SO3 and so3") << endl;
    Sophus::SO3d SO3(R1);
    cout << "SO3 = " << endl << SO3.matrix() << endl;
    cout << "so3 = " << SO3.log().transpose() << endl;

    // OpenCV:
    cout << SubSection("Rodrigues Formula(OpenCV)") << endl;
    Mat R04(3, 3, CV_64FC1);
    for (Index i = 0; i < 3; ++i) {
        for (Index j = 0; j < 3; ++j) {
            R04.at<double>(i, j) = R1(i, j);
        }
    }
    cout << "R04 = " << endl << R04 << endl;
    Mat x04;
    Rodrigues(R04, x04);
    cout << "x04 = " << x04.t() << endl;
}

// rotation between 2 vector
void rotationBetween2Vector() {
    cout << Section("Rotation between 2 Vector") << endl;
    Vector3d a0(1, 0, 0);
    Vector3d a1(1, 2, 0);

    // calculate using axis angle
    cout << SubSection("Axis Angle") << endl;
    double angle = acos(a0.dot(a1) / a0.norm() / a1.norm());
    Vector3d axis = a0.cross(a1).normalized();  // should normalized, or normalize a0 and a1 at first
    AngleAxisd angleAxisd(angle, axis);
    Matrix3d R1 = angleAxisd.toRotationMatrix();
    cout << "R1 = " << endl << R1 << endl;
    cout << "a1 = " << (R1 * a0).transpose() << endl;

    // calculate using quaternion
    cout << SubSection("Quaternion") << endl;
    Quaterniond q = Quaterniond::FromTwoVectors(a0, a1);
    Matrix3d R2 = q.toRotationMatrix();
    cout << "q = " << q.coeffs().transpose() << endl;
    cout << "R2 = " << endl << R2 << endl;
    cout << "theta = " << Sophus::SO3d(q).log().transpose() << endl;
}

int main(int argc, char* argv[]) {
    basic01();
    test02();
    RotMatAngleConversion();
    rotationBetween2Vector();

    return 0;
}
