#include "MatrixAndVector.h"
#include "Eigen/Dense"
#include <iostream>
using namespace std;
using namespace Eigen;


void MatrixInit()
{
    cout << __FUNCTION__ << endl;
    // 挨个赋值
    Matrix3f m1;
    m1 << 1, 2, 3,
            4, 5, 6,
            7, 8, 9;

    cout << m1 << endl;

    // 分块儿赋值，这对两个矩阵合并有用
    int rows = 5, cols = 5;
    MatrixXf m(rows, cols);
    m << (Matrix3f() << 1, 2, 3, 4, 5, 6, 7, 8, 9).finished(),
            MatrixXf::Zero(3, cols - 3),
            MatrixXf::Zero(rows - 3, 3),
            MatrixXf::Identity(rows - 3, cols - 3); // Zero()、Identity() 分别是0矩阵和单位矩阵。
    cout << m << endl;

    // 特殊矩阵
    MatrixXf A = MatrixXf::Random(3, 2);
    cout << A << endl;
    A = MatrixXf::Zero(3, 3);
    cout << A << endl;
    A = MatrixXf::Identity(3, 3);
    cout << A << endl;
    A = MatrixXf::Constant(rows, cols, 1); //常量矩阵
    cout << A << endl;
}

void MatrixOperations()
{
    cout << __FUNCTION__ << endl;
    MatrixXf m1;
    m1 = MatrixXf::Random(3, 3);
    cout << m1 << endl;
    cout << m1.transpose() << endl; //转置
    cout << m1.conjugate() << endl; //共轭
    cout << m1.adjoint() << endl; //共轭转置（伴随矩阵）


    MatrixXd m2(2, 2);
    m2 << 1, 2, 3, 4;
    cout << m2 << endl;
    cout << m2.sum() << endl;//所有元素求和
    cout << m2.prod() << endl;//所有元素乘积
    cout << m2.mean() << endl;//所有元素求平均
    cout << m2.minCoeff() << endl;//所有元素中最小值
    cout << m2.maxCoeff() << endl;//所有元素中最大值
    cout << m2.trace() << endl;//迹
}

void MatrixBlockOperactions()
{
    cout << __FUNCTION__ << endl;

    Eigen::MatrixXf matrix(4, 4);
    matrix << 1, 2, 3, 4,
            5, 6, 7, 8,
            9, 10, 11, 12,
            13, 14, 15, 16;

    //矩阵的块操作：
    //表示返回从矩阵的（i，j）开始，每行取P个元素，每列取q个元素，原矩阵不变
    cout << matrix.block(1, 1, 2, 2) << endl;
    //原矩阵中第(i, j)开始，获取一个p行q列的子矩阵，返回该子矩阵组成的临时 矩阵对象，原矩阵的元素不变
    cout << matrix.block<2, 2>(1, 1) << endl;

    //获取矩阵中的某行某列
    cout << matrix.row(3) << endl;
    cout << matrix.col(3) << endl;
}


void TestMatrix()
{
    MatrixInit();
    MatrixOperations();
    MatrixBlockOperactions();
}

void TestVector()
{
    Vector3d v1(1, 2, 3);
    Vector3d v2(4, 5, 6);
    Vector3d v;
    double dot_value, norm_value;
    dot_value = v1.dot(v2);//点乘，得到的是标量
    v = v1.cross(v2);//叉乘，得到的是向量
    cout << dot_value << endl;
    cout << v << endl;
    norm_value = v1.norm();//求模
    cout << norm_value << endl;

    Eigen::ArrayXf v3(6);
    v3 << 1, 2, 3, 4, 5, 6;
    v3.head(3);//获取向量的前n个元素
    v3.tail(3);//获取向量尾部的n个元素
    v3.segment(3, 3);//获取从向量的第i个元素开始的n个元素
    cout << v3.segment(3, 3) << endl;
}
