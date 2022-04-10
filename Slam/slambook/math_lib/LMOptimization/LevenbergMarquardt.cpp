/*
 * Levenberg-Marquardt iteration method
 * author:Davidwang
 * date  :2020.08.24
 */

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

const int N = 50;     // 数据点数量
const int Scale = 10; // lambd 缩放因子

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

///计算 y = ax + exp(bx + c)
Mat yEstimate(const Mat &est, const Mat &x)
{
    Mat_<float> Y(x.rows, x.cols);
    exp(est.at<float>(1) * x + est.at<float>(2), Y);
    Y = est.at<float>(0) * x + Y;
    return Y;
}

///列文伯格－马夸尔特法
void LMOptimization(float *x, float *y, float *est0, int iterations)
{
    float epsilon = 0.00001;     // ε ,足够小的数
    float cost = 0, estCost = 0; // 本次迭代的cost和评估的cost
    float lambd = 0.01;          // lambd值
    // x矩阵，y矩阵，参数矩阵
    Mat_<float> mat_X(N, 1, x);
    Mat_<float> mat_Y(N, 1, y);
    Mat_<float> mat_est(3, 1, est0);
    cv::Mat J(N, 3, CV_32F, cv::Scalar::all(0));                // 雅可比矩阵
    cv::Mat error, mat_H, mat_b, mat_dx, estxM, estY2M, error2; // 误差矩阵，b值矩阵，ΔX矩阵,评估的X矩阵,使用评估X矩阵得到的Y矩阵,评估X矩阵得到的误差矩阵

    for (int iter = 0; iter < iterations; iter++)
    {
        error = mat_Y - yEstimate(mat_est, mat_X);
        cost = error.dot(error);
        jacobi(mat_est, mat_X, J);
        mat_H = J.t() * J + lambd * Mat::eye(3, 3, CV_32F); // （H＋λI）,结果是3X3矩阵
        mat_b = J.t() * error;
        if (solve(mat_H, mat_b, mat_dx, cv::DECOMP_QR)) // 求解（H＋λI）Δx = b
        {
            if (isnan(mat_dx.at<float>(0)))
            {
                cout << "result is nan!" << endl;
                break;
            }
            estxM = mat_est + mat_dx;         // x + Δx
            estY2M = yEstimate(estxM, mat_X); // 得到新X值对应的Y值
            error2 = mat_Y - estY2M;          // y - y`
            estCost = error2.dot(error2);     // 再评估误差
            if (estCost < cost)               // 成功则更新向量与估计误差
            {
                if (mat_dx.dot(mat_dx) < epsilon) // Δx * Δx < ε
                {
                    cout << "iteration: " << iter + 1 << ",cost: " << cost << endl;
                    cout << "THe Value, x: " << mat_est.at<float>(0) << ",y:" << mat_est.at<float>(1) << ",c:" << mat_est.at<float>(2) << endl;
                    break;
                }
                else
                {
                    mat_est = estxM;
                    cost = estCost;
                    lambd = lambd / Scale;
                }
            }
            else
            {
                lambd = lambd * Scale;
            }
        }
        else
        {
            cout << "can not solve the function ." << endl;
        }
    }
}

// https://blog.csdn.net/yolon3000/article/details/109043895
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
    LMOptimization(x_data, y_data, est, 50);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<float> time_used = chrono::duration_cast<chrono::duration<float>>(t2 - t1);

    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
    return 0;
}
