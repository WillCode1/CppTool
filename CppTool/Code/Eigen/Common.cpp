#include "Common.h"
#include "Eigen/Dense"
#include <iostream>
using namespace std;
using namespace Eigen;


void LeastSquareMethod()
{
    //对方程Ax=b
    MatrixXf A = MatrixXf::Random(3, 2);
    VectorXf b = VectorXf::Random(3);
    std::cout << "Here is the matrix A:\n" << A << std::endl;
    std::cout << "Here is the right hand side b:\n" << b << std::endl;
    
    /*
      总结
        当矩阵A为病态矩阵时，通过常规表达式求解时效果不好。
        SVD分解方法最准确，但是运算速度最慢；常规求解方法运算速度快，但准确度低；QR分解在两者之间。
     */
    cout << "********** normal equations ********************" << endl;
    // Ax = b is equivalent to solving the normal equation ATAx = ATb
    cout << "The solution using normal equations is:\n" << (A.transpose() * A).ldlt().solve(A.transpose() * b) << endl;

    cout << "********** SVD decomposition ********************" << endl;
    //jacobiSvd 方式:Slow (but fast for small matrices)
    cout << "The solution using the SVD decomposition is:\n" << A.bdcSvd(ComputeThinU | ComputeThinV).solve(b) << endl;

    cout << "********** QR decomposition ********************" << endl;
    //colPivHouseholderQr方法:fast
    cout << "The solution using the QR decomposition is:\n" << A.colPivHouseholderQr().solve(b) << endl;
}
