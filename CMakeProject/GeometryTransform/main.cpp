#include <array>
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "sophus/so3.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[]) {
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

    return 0;
}
