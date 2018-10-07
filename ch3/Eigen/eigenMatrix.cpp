#include <iostream>
#include <ctime>
using namespace std;

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

#define MATRIX_SIZE 50

int main (int argc, char** argv)
{
    Eigen::Matrix<float, 2, 3> matrix_23;
    Eigen::Vector3d v_3d;
    Eigen::Matrix3d matrix_3x3 = Eigen::matrix3x3::Zero();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    Eigen::MatrixXd matrix_x;
    
    matrix_23 << 1, 2, 3, 4, 5, 6;
    cout << matrix_23 << endl;

   for (int i = 0; i < 1; i++)
        for (int j = 0; j < 2; j++)
            cout << marix_23(i, j) << endl;
    
    v_3d << 3, 2, 1;
    Eigen::Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
    Eigen::matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << result << endl;

    matrix_33 = Eigen::Matrix3d::Random();
    cout << matrix_33 << endl;

    Eigen::SelfAdjointEigenSolver<Eigen::matrix3d> eigen_solver (matrix_33.transpose() * matrix_33);
    cout << "Eigen values: " << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors: " << eigen_solver.eigenvectors() << endl;
    
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN;
}
