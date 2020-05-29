#include <iostream>
#include "linalg.h"

using namespace std;

#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#define MATRIX_SIZE 10


int main(int argc, char **argv) {
    Matrix<float, MATRIX_SIZE, MATRIX_SIZE> A
      = MatrixXf::Random(MATRIX_SIZE, MATRIX_SIZE);
    A = A * A.transpose();
    Matrix<float, MATRIX_SIZE, 1> b = MatrixXf::Random(MATRIX_SIZE, 1);

    MatrixXf x = LinearEquationSolve(A, b);
    cout << "Result is " << x.transpose() << endl;
    return 0;
}