/*
 * Gauss-Newton iteration method
 * author:Davidwang
 * date  :2020.08.24
 */

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

const int N = 50; // 数据点数量

/// est：估计值，X：X值
void jacobi(const Mat &est, const Mat &x, cv::Mat &jacobi)
{
    for (int i = 0; i < x.rows; i++)
    {
        // de/da = a
        jacobi.at<float>(i, 0) = x.at<float>(i, 0);
        // de/dc = exp(bx + c)
        jacobi.at<float>(i, 2) = exp(est.at<float>(1, 0) * x.at<float>(i, 0) + est.at<float>(2, 0));
        // de/db = x * exp(bx + c)
        jacobi.at<float>(i, 1) = x.at<float>(i, 0) * jacobi.at<float>(i, 2);
    }
}

/// 计算 y = ax + exp(bx + c)
Mat reprojection(const Mat &est, const Mat &x)
{
    Mat_<float> Y(x.rows, x.cols);
    exp(est.at<float>(1) * x + est.at<float>(2), Y);
    Y = est.at<float>(0) * x + Y;
    return Y;
}

///高斯牛顿法
void GN(float *x, float *y, float *est0, int iterations)
{
    float cost = 0, lastCost = 0; // 本次迭代的cost和上一次迭代的cost
    // x矩阵，y矩阵，参数矩阵
    cv::Mat_<float> mat_X(N, 1, x);
    cv::Mat_<float> mat_Y(N, 1, y);
    cv::Mat_<float> mat_est(3, 1, est0);
    cv::Mat J(N, 3, CV_32F, cv::Scalar::all(0)); // 雅可比矩阵
    cv::Mat error, mat_b, mat_dx;                // 误差矩阵，b值矩阵，deltaX矩阵

    for (int iter = 0; iter < iterations; iter++)
    {
        error = mat_Y - reprojection(mat_est, mat_X);
        cost = error.dot(error);
        jacobi(mat_est, mat_X, J);
        mat_b = J.t() * error;
        Mat_<float> mat_H = J.t() * J;
        // J(x)J^T(x)△x = -J(x)f(x)
        // H△x = g
        if (solve(mat_H, mat_b, mat_dx))
        {
            if (isnan(mat_dx.at<float>(0)))
            {
                cout << "result is nan!" << endl;
                break;
            }
            if (iter > 0 && cost >= lastCost)
            {
                cout << "iteration: " << iter + 1 << ",cost: " << cost << ">= last cost: " << lastCost << ", break." << endl;
                cout << "THe Value, x: " << mat_est.at<float>(0) << ",y:" << mat_est.at<float>(1) << ",c:" << mat_est.at<float>(2) << endl;
                break;
            }
            mat_est += mat_dx;
            lastCost = cost;
        }
        else
        {
            cout << "can not solve the function ." << endl;
        }
    }
}

// https://davidwang.blog.csdn.net/article/details/108224354
int main(int argc, char **argv)
{
    float ar = 18.0, br = 2.0, cr = 1.0; // 真实参数值
    float est[] = {2.0, 4.0, 3.0};       // 估计参数值
    float w_sigma = 1.0;                 // 噪声Sigma值
    cv::RNG rng;                         // OpenCV随机数产生器

    float x_data[N], y_data[N]; // 生成真值数据
    for (int i = 0; i < N; i++)
    {
        float x = i / 100.0;
        x_data[i] = x;
        y_data[i] = ar * x + exp(br * x + cr) + rng.gaussian(w_sigma * w_sigma);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    GN(x_data, y_data, est, 50);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
    return 0;
}
