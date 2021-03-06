// Eigen.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include "Eigen/Dense"
#include "Common.h"
//#include "MatrixAndVector.h"
#include "Pose.h"
#include "EigenRotation.h"
using namespace std;
using namespace Eigen;


void test()
{
    auto v1 = Eigen::Vector3d::Zero();
    auto v2 = Eigen::Vector3d(2, 0, 0);
    auto v3 = Eigen::Vector3d(4, 0, 0);

    auto quat = Quaterniond::Identity();

    AngleAxisd rollAngle(AngleAxisd(0, Vector3d::UnitX()));
    AngleAxisd pitchAngle(AngleAxisd(0, Vector3d::UnitY()));
    AngleAxisd yawAngle(AngleAxisd(M_PI, Vector3d::UnitZ()));
    auto quat2 = yawAngle * pitchAngle * rollAngle;

    Rigid3d a(v1, quat);
    Rigid3d b(v2, quat);
    Rigid3d c(v3, quat2);
    auto d = c * b.inverse() * a;

    std::cout << d.translation() << endl;
    std::cout << d.eulerAngle() << endl;
}

/*
    左乘和右乘, 从几何角度可以这样理解：
    左乘结果是 向量旋转 之后相对于原坐标系的位置
    右乘是参考系旋转移动后，向量(并未移动)相对于新参考系的坐标。
 */
void testLeftRight()
{
    using namespace Eigen_Tool;
    // pose: x,y,yaw
    Eigen::Matrix3d org, inc, res;
    org << 1, 0, 1, 
        0, 1, 2, 
        0, 0, 1;
    //auto res = EigenRotation::RPY2RotationMatrix(Eigen::Vector3d(0, 0, M_PI / 6));
    //std::cout << res << std::endl;
    inc << 0.866025, -0.5, 1,
        0.5, 0.866025, 1,
        0, 0, 1;

    // org 左乘 inc, 原坐标系org位姿上的增量inc
    res = org * inc;
    std::cout << res << std::endl;

    // org 右乘 inc
    res = inc * org;
    std::cout << res << std::endl;
}


int main()
{
    //test();
    //LeastSquareMethod();
    //TestMatrix();
    //TestVector();
    //testPose3d();
    //RankOfMatrix();
    testLeftRight();
    return 0;
}

