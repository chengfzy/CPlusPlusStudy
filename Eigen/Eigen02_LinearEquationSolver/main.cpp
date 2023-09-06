#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <array>
#include <iostream>
#include <common/common.hpp>

using namespace std;
using namespace Eigen;
using namespace common;

/*
 *  Solve Linear System Equation Ax = B
 *      (1) nonSquareEqn: When A is not square, data from Matlab pinv help
 *      (2) singularEqn: When A is singular, data from Matlab mldivide help
 *      (3) illCondEqn: When A is an ill-conditioned matrix, data from TrajectorEstimation smoother
 */

/*
 *  Calculate the pseudo inverse. Reference:
 *      (1)
 * http://eigen.tuxfamily.org/index.php?title=FAQ#Is_there_a_method_to_compute_the_.28Moore-Penrose.29_pseudo_inverse_.3F
 *      (2) https://blog.csdn.net/fengbingchun/article/details/72874623
 *      (3) https://gist.github.com/javidcf/25066cf85e71105d57b6
 */
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> pseudoinverse(
    const MatT& mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4})  // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance) {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        } else {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

// Solve non-square equation when A is non square
void nonSquareEqn() {
    cout << Section("When A is not Square") << endl;
    Matrix<double, 8, 6> A;
    Matrix<double, 8, 1> B = 260 * Matrix<double, 8, 1>::Ones();
    A << 64, 2, 3, 61, 60, 6, 9, 55, 54, 12, 13, 51, 17, 47, 46, 20, 21, 43, 40, 26, 27, 37, 36, 30, 32, 34, 35, 29, 28,
        38, 41, 23, 22, 44, 45, 19, 49, 15, 14, 52, 53, 11, 8, 58, 59, 5, 4, 62;
    cout << "A = " << endl << A << endl;
    cout << "B = " << B.transpose() << endl;

    // define error function
    auto errFunc = [&](const Matrix<double, 6, 1>& x) { return (A * x - B).norm(); };

    // solve equation with different method
    cout << "Solve Method:" << endl
         << "\t(1) colPivHouseholderQr()" << endl
         << "\t(2) fullPivHouseholderQr() " << endl
         << "\t(3) householderQr(ComputeThinU | ComputeThinV)" << endl
         << "\t(4) jacobiSvd(ComputeFullU | ComputeFullV)" << endl
         << "\t(5) jacobiSvd(ComputeThinU | ComputeThinV)" << endl
         << "\t(6) pinv(A) * B" << endl;
    vector<Matrix<double, 6, 1>> x;  // solve result
    x.emplace_back(A.colPivHouseholderQr().solve(B));
    x.emplace_back(A.fullPivHouseholderQr().solve(B));
    x.emplace_back(A.householderQr().solve(B));
    x.emplace_back(A.jacobiSvd(ComputeFullU | ComputeFullV).solve(B));
    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
    x.emplace_back(svd.solve(B));
    x.emplace_back(pseudoinverse(A) * B);

    for (size_t i = 0; i < x.size(); ++i) {
        cout << "x" << i + 1 << " = " << x[i].transpose() << "\t, |x| = " << x[i].norm()
             << ",\tError = " << errFunc(x[i]) << endl;
    }
}

// Solve singular equation when A is singular
void singularEqn() {
    cout << Section("When A is Singular") << endl;
    Matrix<double, 4, 4> A;
    Matrix<double, 4, 1> B = 34 * Matrix<double, 4, 1>::Ones();
    A << 16, 2, 3, 13, 5, 11, 10, 8, 9, 7, 6, 12, 4, 14, 15, 1;
    cout << "A = " << endl << A << endl;
    cout << "B = " << B.transpose() << endl;

    // define error function
    auto errFunc = [&](const Matrix<double, 4, 1>& x) { return (A * x - B).norm(); };

    // solve equation with different method
    cout << "Solve Method:" << endl
         << "\t(1) colPivHouseholderQr()" << endl
         << "\t(2) fullPivHouseholderQr() " << endl
         << "\t(3) householderQr(ComputeThinU | ComputeThinV)" << endl
         << "\t(4) jacobiSvd(ComputeFullU | ComputeFullV)" << endl
         << "\t(5) jacobiSvd(ComputeThinU | ComputeThinV)" << endl
         << "\t(6) pinv(A) * B" << endl
         << "\t(7) llt()" << endl
         << "\t(8) ldlt()" << endl
         << "\t(9) partialPivLu()" << endl
         << "\t(10) fullPivLu()" << endl;
    vector<Matrix<double, 4, 1>> x;  // solve result
    x.emplace_back(A.colPivHouseholderQr().solve(B));
    x.emplace_back(A.fullPivHouseholderQr().solve(B));
    x.emplace_back(A.householderQr().solve(B));
    x.emplace_back(A.jacobiSvd(ComputeFullU | ComputeFullV).solve(B));
    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
    x.emplace_back(svd.solve(B));
    x.emplace_back(pseudoinverse(A) * B);
    // x.emplace_back(A.inverse() * B);  //cannot calculated
    x.emplace_back(A.llt().solve(B));
    x.emplace_back(A.ldlt().solve(B));
    x.emplace_back(A.partialPivLu().solve(B));
    x.emplace_back(A.fullPivLu().solve(B));

    for (size_t i = 0; i < x.size(); ++i) {
        cout << "x" << i + 1 << " = " << x[i].transpose() << "\t, |x| = " << x[i].norm()
             << ",\tError = " << errFunc(x[i]) << endl;
    }
}

// Solve equation when A is ill-conditioned matrix
void illCondEqn() {
    cout << Section("When A is Ill-Conditioned") << endl;
    Matrix<double, 2, 2> A;
    Vector2d B;
    A << 2, 1, 2.01, 1.0;
    B << 3, 4;

    cout << "A = " << endl << A << endl;
    cout << "B = " << B.transpose() << endl;

    // define error function
    auto errFunc = [&](const Matrix<double, 2, 1>& x) { return (A * x - B).norm(); };

    // solve equation with different method
    cout << "Solve Method:" << endl
         << "\t(1) colPivHouseholderQr()" << endl
         << "\t(2) fullPivHouseholderQr() " << endl
         << "\t(3) householderQr(ComputeThinU | ComputeThinV)" << endl
         << "\t(4) jacobiSvd(ComputeFullU | ComputeFullV)" << endl
         << "\t(5) jacobiSvd(ComputeThinU | ComputeThinV)" << endl
         << "\t(6) pinv(A) * B" << endl
         << "\t(7) llt()" << endl
         << "\t(8) ldlt()" << endl
         << "\t(9) partialPivLu()" << endl
         << "\t(10) fullPivLu()" << endl
         << "\t(11) inv(A) * B" << endl;
    vector<Matrix<double, 2, 1>> x;  // solve result
    x.emplace_back(A.colPivHouseholderQr().solve(B));
    x.emplace_back(A.fullPivHouseholderQr().solve(B));
    x.emplace_back(A.householderQr().solve(B));
    x.emplace_back(A.jacobiSvd(ComputeFullU | ComputeFullV).solve(B));
    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
    x.emplace_back(svd.solve(B));
    x.emplace_back(pseudoinverse(A) * B);
    // x.emplace_back(A.inverse() * B);  //cannot calculated
    x.emplace_back(A.llt().solve(B));
    x.emplace_back(A.ldlt().solve(B));
    x.emplace_back(A.partialPivLu().solve(B));
    x.emplace_back(A.fullPivLu().solve(B));
    x.emplace_back(A.inverse() * B);

    for (size_t i = 0; i < x.size(); ++i) {
        cout << "x" << i + 1 << " = " << x[i].transpose() << "\t, |x| = " << x[i].norm()
             << ",\tError = " << errFunc(x[i]) << endl;
    }
}

int main(int argc, char* argv[]) {
    nonSquareEqn();
    singularEqn();
    illCondEqn();

    return 0;
}
