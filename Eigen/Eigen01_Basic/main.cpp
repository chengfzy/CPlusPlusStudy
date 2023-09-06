#include <Eigen/Core>
#include <Eigen/SVD>
#include <array>
#include <iostream>
#include <numeric>
#include <common/common.hpp>

using namespace std;
using namespace Eigen;
using namespace common;

int main(int argc, char* argv[]) {
    // basic
    {
        cout << Section("Basic") << endl;
        // Eigen create variable with default initialization, e.g, Vector3d = 0
        Vector3d vec01;
        Vector3d vec02(0, 0, 0);
        cout << "vec with default init = " << vec01.transpose() << endl;
        cout << "vec with explicit init = " << vec02.transpose() << endl;

        cout << "x = " << Vector3d::Constant(3) << endl;

        // matrix
        Matrix3d mat01;
        mat01 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
        cout << "mat01 = " << mat01 << endl;
        mat01 *= -1;
        cout << "mat01 * -1 = " << mat01 << endl;
        // take sub-matrix
        Matrix2d subMat01 = mat01.block<2, 2>(1, 1);
        cout << "subMat01 = " << subMat01 << endl;
        // matrix + scalar
        Matrix3d mat01b = mat01.array() + 1;
        cout << "mat01 + 1 = " << endl << mat01b << endl;

        // set and cwise operation
        Vector3d vec3a(1, 2, 3);
        Matrix3d mat3a = Matrix3d::Zero();
        mat3a.diagonal() = vec3a;
        cout << "mat3a = \r\n" << mat3a << endl;
        // cwise product
        mat3a(1, 2) = 4;
        Matrix3d mat3b;
        mat3b << 1, 2, 3, 4, 5, 6, 7, 8, 9;
        cout << "Before mat3a = \r\n" << mat3a << endl;
        cout << "Before mat3b = \r\n" << mat3b << endl;
        mat3a = mat3a.cwiseProduct(mat3b);
        cout << "cwise product mat3a = \r\n" << mat3a << endl;
        // cwise max
        cout << "cwise max mat3a = \r\n" << mat3a.cwiseMax(10.0) << endl;

        // min and max operation
        Matrix3d mat4a = Matrix3d::Random();
        Matrix3d mat4b = Matrix3d::Random();
        Matrix3d mat4c = mat4a.cwiseMin(mat4b).matrix();
        cout << "mat4a = " << endl << mat4a << endl;
        cout << "mat4b = " << endl << mat4b << endl;
        cout << "min(mat4b, mat4b) = " << endl << mat4c << endl;
    }

    // diagonal matrix
    {
        cout << Section("Diagonal Matrix") << endl;
        Vector3d diagVec(1, 2, 3);
        Matrix3d diagMat = diagVec.asDiagonal();
        diagMat.diagonal() << 4, 5, 6;
        cout << "diagMat = " << diagMat << endl;
        DiagonalMatrix<double, 3> diagMat01(1, 2, 3);
        Matrix3d sumDiagonal = diagMat + diagMat01.toDenseMatrix();
        cout << "sum diag = " << sumDiagonal << endl;
    }

    // random matrix
    {
        cout << Section("Random Matrix") << endl;
        Vector3d randVec = Vector3d::Random();
        Vector3d randCov(4, 5, 6);
        randVec = randVec.cwiseProduct(randCov);
        cout << "[1] randVec = " << randVec.transpose() << endl;
        randVec = Vector3d::Random();
        Vector3d randVec1 = randVec * 5;
        randVec = randVec.cwiseProduct(randCov);
        cout << "[2] randVec1 = " << randVec1.transpose() << endl;
        cout << "[2] randVec = " << randVec.transpose() << endl;
    }

    // solve linear equations
    {
        cout << Section("Solve Linear Equations") << endl;
        Matrix3f A;
        Vector3f b;
        A << 1, 2, 3, 4, 5, 6, 7, 8, 10;
        b << 3, 3, 4;
        cout << " A = " << endl << A << endl;
        cout << "b = " << b.transpose() << endl;
        // Ax = b
        Vector3f x1 = A.colPivHouseholderQr().solve(b);
        cout << "Ax = b, x = " << x1.transpose() << endl;
        // xA = b => A^T * xT = b^T
        Vector3f x2 = A.transpose().colPivHouseholderQr().solve(b);
        cout << "xA = b, x= " << x2.transpose() << endl;
    }

    // matrix condition number and ill-conditioned matrix
    {
        cout << Section("Matrix Condition Number") << endl;
        Matrix2d matA;
        matA << 4.1, 2.8, 9.7, 6.1;
        //    Matrix2d invMatA = matA.inverse();
        //    cout << "matA^-1 = " << endl << invMatA << endl;
        JacobiSVD<MatrixXd> svd(matA);
        auto singVal = svd.singularValues();
        cout << "Singular Value = " << singVal.transpose() << endl;
        double cond = abs(singVal(0) / singVal(singVal.size() - 1));
        cout << "condition number = " << cond << endl;
    }

    // matrix mean and variance
    {
        cout << Section("Matrix Mean and Variance") << endl;
        const int N{10};

        // Case 01: calculate mean and variance for 1D vector data
        array<double, N> data01;
        for (size_t i = 0; i < N; ++i) {
            data01[i] = i;
        }
        // use map to operate data. but can only calculate mean
        Map<VectorXd> mat01a(data01.data(), data01.size());
        cout << "mat01a = " << mat01a.transpose() << endl;
        double mean01a = mat01a.mean();
        cout << "mean01a = " << mean01a << endl;

        // use constructor to calculate mean and variance
        Matrix<double, N, 1> mat01(data01.data());
        cout << "mat01 = " << mat01.transpose() << endl;
        Matrix<double, 1, 1> mean01 = mat01.colwise().mean();
        Matrix<double, 1, 1> var01 = (mat01.rowwise() - mean01).array().square().colwise().sum() / N;
        cout << "Mean01 = " << mean01 << ", Var01 = " << var01 << endl;

        // Case 02: calculate mean and variance for 2D vector data
        Matrix<double, N, 3> mat02;
        vector<Vector3d> data02(N, Vector3d::Zero());
        for (Index i = 0; i < mat02.rows(); ++i) {
            for (Index j = 0; j < mat02.cols(); ++j) {
                mat02(i, j) = 3 * i + j;
                data02[i][j] = 3 * i + j;
            }
        }
        cout << "mat02 = " << endl << mat02 << endl;
        Matrix<double, 1, 3> mean02 = mat02.colwise().mean();
        Matrix<double, 1, 3> var02 = (mat02.rowwise() - mean02).array().square().colwise().sum() / N;
        cout << "Mean02 = " << mean02 << ", Var02 = " << var02 << endl;
        // convert to Vector3d
        Vector3d meanVec(mean02), varVec(var02);
        cout << "Vector3d. Mean02 = " << meanVec.transpose() << ", Var02 = " << varVec.transpose() << endl;

        // calculate using STL function
        Vector3d mean03 = accumulate(data02.begin(), data02.end(), Vector3d::Zero().eval()) / data02.size();
        // Vector3d var03a = accumulate(data02.begin(), data02.end(), Vector3d::Zero().eval(),
        //                              [&](const Vector3d& s, const Vector3d& v) {
        //                                  // Vector3d temp1 = v - mean03;
        //                                  // Vector3d diff2 = (v - mean03).array().square();
        //                                  // // LOG(INFO) << format("temp1 = {}, diff2 = {}", temp1, diff2);
        //                                  // return s + diff2;
        //                                  return s + Vector3d((v - mean03).array().square());
        //                              });
        Vector3d var03a = Vector3d::Zero();
        var03a = accumulate(data02.begin(), data02.end(), var03a, [&](const Vector3d& s, const Vector3d& v) {
            // Vector3d temp1 = v - mean03;
            // Vector3d diff2 = (v - mean03).array().square();
            // // LOG(INFO) << format("temp1 = {}, diff2 = {}", temp1, diff2);
            // return s + diff2;
            return s + Vector3d((v - mean03).array().square());
        });
        var03a /= N;
        Vector3d var03 = Vector3d::Zero();
        for (auto& v : data02) {
            var03 += Vector3d((v - mean03).array().square());
        }
        var03 /= N;
        cout << "Vector3d. Mean03 = " << mean03.transpose() << ", Var03a = " << var03a.transpose() << endl;
        cout << "Vector3d. Mean03 = " << mean03.transpose() << ", Var03a = " << var03.transpose() << endl;
    }

    // matrix norm
    {
        cout << Section("Matrix Norm") << endl;
        Vector3d vec01 = Vector3d::Random();
        cout << "vec01 = " << vec01.transpose() << endl;
        cout << "norm-2 of vec01 = " << vec01.squaredNorm() << endl;

        Matrix<double, 5, 3> mat01a = Matrix<double, 5, 3>::Random();
        cout << "mat01a = " << endl << mat01a << endl;
        cout << "Norm(mat) = " << mat01a.rowwise().squaredNorm().transpose() << endl;
    }

    return 0;
}
