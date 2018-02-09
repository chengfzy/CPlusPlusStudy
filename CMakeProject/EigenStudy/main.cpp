#include <iostream>
#include "Eigen/Core"

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[]) {
    // matrix
    Matrix3d mat01;
    mat01 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    cout << "mat01 = " << mat01 << endl;
    mat01 *= -1;
    cout << "mat01 * -1 = " << mat01 << endl;
    // take sub-matrix
    Matrix2d subMat01 = mat01.block<2, 2>(1, 1);
    cout << "subMat01 = " << subMat01 << endl;

    // diagnal matrix
    Vector3d diagVec(1, 2, 3);
    Matrix3d diagMat = diagVec.asDiagonal();
    diagMat.diagonal() << 4, 5, 6;
    cout << "diagMat = " << diagMat << endl;
    DiagonalMatrix<double, 3> diagMat01(1, 2, 3);
    Matrix3d sumDiagonal = diagMat + diagMat01.toDenseMatrix();
    cout << "sum diag = " << sumDiagonal << endl;

    // random matrix
    Vector3d randVec = Vector3d::Random();
    Vector3d randCov(4, 5, 6);
    randVec = randVec.cwiseProduct(randCov);
    cout << "[1] randVec = " << randVec.transpose() << endl;
    randVec = Vector3d::Random();
    Vector3d randVec1 = randVec * 5;
    randVec = randVec.cwiseProduct(randCov);
    cout << "[2] randVec1 = " << randVec1.transpose() << endl;
    cout << "[2] randVec = " << randVec.transpose() << endl;

    return 0;
}
