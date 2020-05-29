#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


// Solve equation Ax=b
MatrixXf LinearEquationSolve(const Ref<const MatrixXf>& A, const Ref<const VectorXf>& b, string type = "QR") {
    int MATRIX_SIZE = b.rows();
    Matrix<float, MATRIX_SIZE, 1> x;
    if (type == "Reverse") {
        x = A.inverse() * b;
    }
    else if (type == "Cholesky")
    {
        x = A.ldlt().solve(b);
    }
    else
    {
        x = A.colPivHouseholderQr().solve(b);
    }

    return x;
}