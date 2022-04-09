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

/*  函数声明  */
void LM(double *, double *, double *);
Mat jacobi(const Mat &, const Mat &);
Mat yEstimate(const Mat &, const Mat &);

int main(int argc, char **argv)
{
    double ar = 18.0, br = 2.0, cr = 1.0; // 真实参数值
    double est[] = {2.0, 4.0, 3.0};       // 估计参数值
    double w_sigma = 1.0;                 // 噪声Sigma值
    cv::RNG rng;                          // OpenCV随机数产生器

    double x_data[N], y_data[N]; // 生成真值数据
    for (int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data[i] = x;
        y_data[i] = ar * x + exp(br * x + cr) + rng.gaussian(w_sigma * w_sigma);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    LM(x_data, y_data, est);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
    return 0;
}

///列文伯格－马夸尔特法
void LM(double *x, double *y, double *est0)
{
    int iterations = 50;                                                  // 迭代次数
    double epsilon = 0.00001;                                             // ε ,足够小的数
    double cost = 0, estCost = 0;                                         // 本次迭代的cost和评估的cost
    double lambd = 0.01;                                                  // lambd值
    Mat_<double> xM(N, 1, x), yM(N, 1, y), estM(3, 1, est0);              // x矩阵，y矩阵，参数矩阵，M表示 Matrix，
    Mat_<double> jacobiM, estYM, errorM, bM, dxM, estxM, estY2M, error2M; // 雅可比矩阵，评估值Y，误差矩阵，b值矩阵，ΔX矩阵,评估的X矩阵,使用评估X矩阵得到的Y矩阵,评估X矩阵得到的误差矩阵

    for (int iter = 0; iter < iterations; iter++)
    {
        jacobiM = jacobi(estM, xM);
        estYM = yEstimate(estM, xM);
        errorM = yM - estYM;                                                        // e
        cost = errorM.dot(errorM);                                                  // e^2
        bM = jacobiM.t() * errorM;                                                  // b
        Mat_<double> HM = jacobiM.t() * jacobiM + lambd * (Mat::eye(3, 3, CV_64F)); // （H＋λI）,结果是3X3矩阵
        if (solve(HM, bM, dxM))                                                     // 求解（H＋λI）Δx = b
        {
            if (isnan(dxM.at<double>(0)))
            {
                cout << "result is nan!" << endl;
                break;
            }
            estxM = estM + dxM;             // x + Δx
            estY2M = yEstimate(estxM, xM);  // 得到新X值对应的Y值
            error2M = yM - estY2M;          // y - y`
            estCost = error2M.dot(error2M); // 再评估误差
            if (estCost < cost)             // 成功则更新向量与估计误差
            {
                if (dxM.dot(dxM) < epsilon) // Δx * Δx < ε
                {
                    cout << "iteration: " << iter + 1 << ",cost: " << cost << endl;
                    cout << "THe Value, x: " << estM.at<double>(0) << ",y:" << estM.at<double>(1) << ",c:" << estM.at<double>(2) << endl;
                    break;
                }
                else
                {
                    estM = estxM;
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

/// est：估计值，X：X值
Mat jacobi(const Mat &est, const Mat &x)
{
    Mat_<double> J(x.rows, est.rows), da, db, dc; // a,b,c的导数
    da = x;
    exp(est.at<double>(1) * x + est.at<double>(2), dc);
    db = x.mul(dc);

    da.copyTo(J(Rect(0, 0, 1, J.rows)));
    db.copyTo(J(Rect(1, 0, 1, J.rows)));
    dc.copyTo(J(Rect(2, 0, 1, J.rows)));
    return J;
}
///计算 y = ax + exp(bx + c)
Mat yEstimate(const Mat &est, const Mat &x)
{
    Mat_<double> Y(x.rows, x.cols);
    exp(est.at<double>(1) * x + est.at<double>(2), Y);
    Y = est.at<double>(0) * x + Y;
    return Y;
}
