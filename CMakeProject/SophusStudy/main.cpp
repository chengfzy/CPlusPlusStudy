#include <iostream>
#include "Eigen/Core"
#include "sophus/so3.h"
#include "unsupported/Eigen/MatrixFunctions"

using namespace std;
using namespace Eigen;
using namespace Sophus;

int main(int argc, char* argv[]) {
    // rotation conversion
    cout << "--------------------- Rotation Conversion ---------------------" << endl;
    double angle = M_PI / 6;
    Vector3d axis(0.5, 1, 2);
    axis.normalize();
    cout << "Angle = " << angle << ", axis = " << axis.transpose() << endl;
    // Angle Axisd
    Matrix3d R = AngleAxisd(angle, axis).toRotationMatrix();
    AngleAxisd axisdAngle(R);
    cout << "Axisd Angle = " << axisdAngle.angle() * axisdAngle.axis().transpose() << endl;
    cout << "Angle = " << axisdAngle.angle() << ", Axis = " << axisdAngle.axis().transpose() << endl;
    cout << "R = " << R << endl;
    // SO3
    Sophus::SO3 R_SO3(R);
    cout << "R_SO3.matrix() = " << R_SO3.matrix() << endl;
    cout << "R_SO3 = " << R_SO3 << endl;
    cout << "Log(R_SO3) = " << R_SO3.log().transpose() << endl;
    // euler angle
    Vector3d eulerAngle = R.matrix().eulerAngles(0, 1, 2);
    cout << "Euler Angle = " << eulerAngle.transpose() << endl;
    // euler angle to rotation matrix
    AngleAxisd Rx(eulerAngle[0], Vector3d::UnitX());
    AngleAxisd Ry(eulerAngle[1], Vector3d::UnitY());
    AngleAxisd Rz(eulerAngle[2], Vector3d::UnitZ());
    Matrix3d RfromEuler = Rx.toRotationMatrix() * Ry.toRotationMatrix() * Rz.toRotationMatrix();
    cout << "Rx * Ry * Rz = " << RfromEuler << endl;
    // euler to quaternion and then to rotation matrix
    Quaterniond q = Rx * Ry * Rz;
    cout << "q.matrix = " << q.matrix() << endl;

    // perturbation
    cout << endl << "--------------------- Rotation Perturbation ---------------------" << endl;
    Vector3d deltaPhi(0.01 * M_PI, 0.15 * M_PI, 0.2 * M_PI);
    Matrix3d deltaPhiSkew;
    deltaPhiSkew << 0, -deltaPhi[2], deltaPhi[1], deltaPhi[2], 0, -deltaPhi[0], -deltaPhi[1], deltaPhi[0], 0;
    Matrix3d R1 = R * deltaPhiSkew.exp();  // unsupported functions
    cout << "perturb on R = " << endl << R1 << endl;
    Sophus::SO3 R1_SO3 = R_SO3 * Sophus::SO3::exp(deltaPhi);
    cout << "perturb on R_SO3 = " << endl << R1_SO3.matrix() << endl;
    // so(3)
    Vector3d so3 = R1_SO3.log();
    Matrix3d so3_hat = Sophus::SO3::hat(so3);
    cout << "so3 = " << so3.transpose() << endl;
    cout << "so3^ = " << endl << so3_hat << endl;

    return 0;
}
