#include <array>
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "sophus/so3.hpp"
#include "unsupported/Eigen/EulerAngles"

using namespace std;
using namespace Eigen;

void basic01() {
    // euler angle
    double yaw{0.5}, pitch{0.8}, roll{1.0};
    Vector3d angle(roll, pitch, yaw);
    cout << "angle = [Rx, Ry, Rz] = [roll, pitch, yaw] = " << angle.transpose() << endl;

    // rotation sequence ZYX, R = Rz * Ry * Rx, Matlab's default
    cout << endl << "================== Rotation Sequence: ZYX, R = Rz * Ry * Rx ==================" << endl;
    // Euler Angle => Rotation Matrix
    Matrix3d R1;
    R1 = AngleAxisd(angle[2], Vector3d::UnitZ()) * AngleAxisd(angle[1], Vector3d::UnitY()) *
         AngleAxisd(angle[0], Vector3d::UnitX());
    cout << "R1 = " << endl << R1 << endl;
    Vector3d euler1 = R1.eulerAngles(2, 1, 0);
    cout << "euler1 = " << euler1.transpose() << endl;

    // rotation sequence XYZ, R = Rx * Ry * Rz
    cout << endl << "================== Rotation Sequence: XYZ, R = Rx * Ry * Rz ==================" << endl;
    // Euler Angle => Rotation Matrix
    Matrix3d R2;
    R2 = AngleAxisd(angle[0], Vector3d::UnitX()) * AngleAxisd(angle[1], Vector3d::UnitY()) *
         AngleAxisd(angle[2], Vector3d::UnitZ());
    cout << "R2 = " << endl << R2 << endl;
    // Rotation Matrix => Euler Angle
    Vector3d euler2 = R2.eulerAngles(0, 1, 2);
    cout << "euler2 = " << euler2.transpose() << endl;

    // SO3
    cout << endl << "================== SO3 and Rotation Vector(Rodrigues Formula) ==================" << endl;
    // to SO3
    Sophus::SO3d SO3(R1);
    cout << "SO3 = " << endl << SO3.matrix() << endl;
    // Rotation Vector
    cout << "so3 = " << SO3.log().transpose() << endl;
}

void test02() {
    cout << endl << "================== Test ==================" << endl;
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
    cout << "angle of q = " <<

        cout << endl
         << endl;
    Matrix3d R2;
    R2 = AngleAxisd(x1[0], Vector3d::UnitX()) * AngleAxisd(x1[1], Vector3d::UnitY()) *
         AngleAxisd(x1[2], Vector3d::UnitZ());
    Vector3d x2b = R2.eulerAngles(2, 1, 0);
    cout << "R2 = " << endl << R2 << endl;
    cout << "x2b = " << x2b.transpose() << endl;
}

Vector3d eulerAngleFromRotationMatrix(const Matrix3d& R) {
    // euler angle = [Rx, Ry, Rz] = [roll, pitch, yaw]
    Vector3d angle;
    angle[2] = atan2(R(1, 0), R(0, 0));  // yaw
    angle[1] = asin(-R(2, 0));           // pitch
    angle[0] = atan2(R(2, 1), R(2, 2));  // roll
    return angle;
}

void test03() {
    // XYZ in extrinsic coordinates, R = Rz * Ry * Rx
    cout << endl << "======================= Geometry using Eigen::EulerAngles =======================" << endl;

    // euler angle = [Rx, Ry, Rz] = [roll, pitch, yaw]
    Vector3d x0(-0.0104279, -0.0143366, -0.00132262);
    cout << "x0 = " << x0.transpose() << endl;

    cout << endl << "---------------------- Rotation Matrix ----------------------" << endl;
    Matrix3d R1 = (AngleAxisd(x0[2], Vector3d::UnitZ()) * AngleAxisd(x0[1], Vector3d::UnitY()) *
                   AngleAxisd(x0[0], Vector3d::UnitX()))
                      .toRotationMatrix();
    Vector3d x1a = R1.eulerAngles(2, 1, 0);
    cout << "R1 = " << endl << R1 << endl;
    cout << "x1a = " << x1a.transpose() << endl;

    cout << endl << "---------------------- Euler Angles ----------------------" << endl;
    // contructed from angle
    EulerAnglesZYXd x02a(x0[2], x0[1], x0[0]);
    cout << "x02a.R = " << x02a.matrix() << endl;
    cout << "x02a.angle = " << x02a.angles().transpose() << endl;
    // contructed from rotation matrix
    cout << endl;
    EulerAnglesZYXd x02b(R1);
    cout << "x02b.R = " << x02b.matrix() << endl;
    cout << "x02b.angle = " << x02b.angles().transpose() << endl;
    // contructed from rotation matrix
    cout << endl;
    EulerAnglesZYXd x02c = EulerAnglesZYXd::FromRotation<false, false, false>(R1);
    cout << "x02c.R = " << x02c.toRotationMatrix() << endl;
    cout << "x02c.angle = " << x02c.angles().transpose() << endl;

    cout << endl << "---------------------- User-Defined Function ----------------------" << endl;
    Vector3d x03a = eulerAngleFromRotationMatrix(R1);
    cout << "x03a = " << x03a.transpose() << endl;

    //    cout << endl << endl;
    //    Matrix3d R2;
    //    R2 = AngleAxisd(x0[0], Vector3d::UnitX()) * AngleAxisd(x0[1], Vector3d::UnitY()) *
    //         AngleAxisd(x0[2], Vector3d::UnitZ());
    //    Vector3d x2b = R2.eulerAngles(2, 1, 0);
    //    cout << "R2 = " << endl << R2 << endl;
    //    cout << "x2b = " << x2b.transpose() << endl;
}

int main(int argc, char* argv[]) {
    test03();

    return 0;
}
