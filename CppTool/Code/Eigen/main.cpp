// Eigen.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include "Eigen/Dense"
#include "Common.h"
#include "MatrixAndVector.h"
#include "Pose.h"
using namespace std;
using namespace Eigen;


void test()
{
    MatrixXd m(2, 2);
    m(0, 0) = 3;
    m(1, 0) = 2.5;
    m(0, 1) = -1;
    m(1, 1) = m(1, 0) + m(0, 1);
    std::cout << m << std::endl;
}


int main()
{
    //test();
    //LeastSquareMethod();
    //TestMatrix();
    //TestVector();
    testPose3d();
    return 0;
}

