#include <Eigen/Core>
#include <iostream>
#include <sophus/so3.hpp>
#include <common/common.hpp>
#include "unsupported/Eigen/MatrixFunctions"

using namespace std;
using namespace Eigen;
using namespace Sophus;
using namespace common;

int main(int argc, char* argv[]) {
    // rotation conversion
    cout << Section("Rotation Conversion") << endl;
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
    Sophus::SO3d R_SO3(R);
    cout << "R_SO3.matrix() = " << R_SO3.matrix() << endl;
    cout << "R_SO3 = " << R_SO3.log().transpose() << endl;
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
    cout << endl << Section("Rotation Perturbation") << endl;
    Vector3d deltaPhi(0.01 * M_PI, 0.15 * M_PI, 0.2 * M_PI);
    Matrix3d deltaPhiSkew;
    deltaPhiSkew << 0, -deltaPhi[2], deltaPhi[1], deltaPhi[2], 0, -deltaPhi[0], -deltaPhi[1], deltaPhi[0], 0;
    Matrix3d R1 = R * deltaPhiSkew.exp();  // unsupported functions
    cout << "perturb on R = " << endl << R1 << endl;
    Sophus::SO3d R1_SO3 = R_SO3 * Sophus::SO3d::exp(deltaPhi);
    cout << "perturb on R_SO3 = " << endl << R1_SO3.matrix() << endl;
    // perturbation product
    // plase note cannot construct with (v[0], v[1], v[2]), it's like Euler angle XYZ, not rotation angle
    SO3d deltaR = SO3d::exp(deltaPhi);
    SO3d R1_product = R_SO3 * deltaR;
    cout << "01: perturb product on R_SO3 = " << endl << R1_product.matrix() << endl;

    cout << "R = " << R_SO3.matrix() << endl;
    cout << "|R| = " << R_SO3.matrix().norm() << endl;
    cout << "R1 = " << R1_SO3.matrix() << endl;
    cout << "|R1| = " << R1_SO3.matrix().norm() << endl;

    Matrix3d R1Mat = R1_SO3.matrix().normalized();
    cout << "R1Mat = " << R1Mat << endl;
    cout << "|R1Mat| = " << R1Mat.norm() << endl;

    R1_SO3.normalize();
    cout << "R1 = " << R1_SO3.matrix() << endl;
    cout << "|R1| = " << R1_SO3.matrix().norm() << endl;

    // so(3)
    Vector3d so3 = R1_SO3.log();
    Matrix3d so3_hat = Sophus::SO3d::hat(so3);
    cout << "so3 = " << so3.transpose() << endl;
    cout << "so3^ = " << endl << so3_hat << endl;

    double x = 3;
    Matrix3d y = Matrix3d::Zero();
    y.diagonal() = Vector3d::Ones() * x;
    cout << "y = " << y << endl;

    // exp
    {
        Eigen::Vector3d x(10, 20, 30);
        double dt = 0.01;
        SO3d R0;
        SO3d R1 = R0 * SO3d::exp(x * dt);
        cout << "x^ = " << SO3d::hat(x) << endl;
        cout << "R0 = " << R0.matrix() << endl;
        cout << "R1 = " << R1.matrix() << endl;
        cout << "x1 = " << R1.log() << endl;
    }

    // test
    {
        double z0 = -158 * M_PI / 180;
        double z1 = 160 * M_PI / 180;
        Vector3d a0(0, 0, z0);
        Vector3d a1(0, 0, z1);
        SO3d R0 = SO3d::exp(a0);
        SO3d R1 = SO3d::exp(a1);
        SO3d dR0 = SO3d::exp(a0) * R1;
        SO3d dR1 = SO3d::exp(-a0) * R1;
        SO3d dR2 = R1 * SO3d::exp(a0);
        SO3d dR3 = R1 * SO3d::exp(-a0);
        Vector3d da0 = dR0.log();
        Vector3d da1 = dR1.log();
        Vector3d da2 = dR2.log();
        Vector3d da3 = dR3.log();

        cout << "da0 = " << da0.transpose() * 180 / M_PI << endl;
        cout << "da1 = " << da1.transpose() * 180 / M_PI << endl;
        cout << "da2 = " << da2.transpose() * 180 / M_PI << endl;
        cout << "da0 = " << da3.transpose() * 180 / M_PI << endl;
    }
    return 0;
}
