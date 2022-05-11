/********************************************************************
 * Created by 杨帮杰 on 11/10/18
 * Right to use this code in any way you want without
 * warranty, support or any guarantee of it working
 * E-mail: yangbangjie1998@qq.com
 * Association: SCAU 华南农业大学
 ********************************************************************/
#pragma once
#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d.hpp>

#define IMAGE1_PATH "/home/jacob/图片/1.png"
#define IMAGE2_PATH "/home/jacob/图片/2.png"
#define IMAGE3_PATH "/home/jacob/图片/3.png"

using namespace std;
using namespace cv;

class Histogram1D
{
private:
    int histSize[1];  // 项的数量
    float hranges[2]; // 统计像素的最大值和最小值
    const float *ranges[1];
    int channels[1]; // 仅计算一个通道

public:
    Histogram1D()
    {
        // 准备1D直方图的参数
        histSize[0] = 256;
        hranges[0] = 0.0f;
        hranges[1] = 255.0f;
        ranges[0] = hranges;
        channels[0] = 0;
    }

    Mat getHistogram(const Mat &image)
    {
        Mat hist;
        // 计算直方图
        calcHist(&image,   // 要计算图像的
                 1,        // 只计算一幅图像的直方图
                 channels, // 通道数量
                 Mat(),    // 不使用掩码
                 hist,     // 存放直方图
                 1,        // 1D直方图
                 histSize, // 统计的灰度的个数
                 ranges);  // 灰度值的范围
        return hist;
    }

    Mat getHistogramImage(const Mat &image)
    {
        Mat hist = getHistogram(image);

        //查找最大值用于归一化
        double maxVal = 0;

        // minMaxLoc(hist, NULL, &maxVal);

        //绘制直方图的图像
        Mat histImg(histSize[0], histSize[0], CV_8U, Scalar(255));

        // 设置最高点为最大值的90%
        // double hpt = 0.9 * histSize[0];
        //每个条目绘制一条垂直线
        for (int h = 0; h < histSize[0]; h++)
        {
            //直方图的元素类型为32位浮点数
            float binVal = hist.at<float>(h);
            // int intensity = static_cast<int>(binVal * hpt / maxVal);
            line(histImg, Point(h, histSize[0]), Point(h, histSize[0] - binVal), Scalar::all(0));
        }
        return histImg;
    }
};

/**
 * @brief EqualizeImage 对灰度图像进行直方图均衡化
 * @param src 输入图像
 * @param dst 均衡化后的图像
 */
void EqualizeImage(const Mat &src, Mat &dst)
{
    Histogram1D hist1D;
    Mat hist = hist1D.getHistogram(src);

    hist /= (src.rows * src.cols); // 对得到的灰度直方图进行归一化,得到密度（0～1）
    float cdf[256] = {0};          // 灰度的累积概率
    Mat lut(1, 256, CV_8U);        // 创建用于灰度变换的查找表
    for (int i = 0; i < 256; i++)
    {
        // 计算灰度级的累积概率
        if (i == 0)
            cdf[i] = hist.at<float>(i);
        else
            cdf[i] = cdf[i - 1] + hist.at<float>(i);

        lut.at<uchar>(i) = static_cast<uchar>(255 * cdf[i]); // 创建灰度的查找表
    }

    LUT(src, lut, dst); // 应用查找表，进行灰度变化，得到均衡化后的图像
}

/**
 * @brief HistSpecify 对灰度图像进行直方图规定化
 * @param src 输入图像
 * @param ref 参考图像，解析参考图像的直方图并用于规定化
 * @param result 直方图规定化后的图像
 * @note 手动设置一个直方图并用于规定化比较麻烦，这里使用一个参考图像来进行
 */
void HistSpecify(const Mat &src, const Mat &ref, Mat &result)
{
    Histogram1D hist1D;
    Mat src_hist = hist1D.getHistogram(src);
    Mat dst_hist = hist1D.getHistogram(ref);

    float src_cdf[256] = {0};
    float dst_cdf[256] = {0};

    // 直方图进行归一化处理
    src_hist /= (src.rows * src.cols);
    dst_hist /= (ref.rows * ref.cols);

    // 计算原始直方图和规定直方图的累积概率
    for (int i = 0; i < 256; i++)
    {
        if (i == 0)
        {
            src_cdf[i] = src_hist.at<float>(i);
            dst_cdf[i] = dst_hist.at<float>(i);
        }
        else
        {
            src_cdf[i] = src_cdf[i - 1] + src_hist.at<float>(i);
            dst_cdf[i] = dst_cdf[i - 1] + dst_hist.at<float>(i);
        }
    }

    // 累积概率的差值
    float diff_cdf[256][256];
    for (int i = 0; i < 256; i++)
        for (int j = 0; j < 256; j++)
            diff_cdf[i][j] = fabs(src_cdf[i] - dst_cdf[j]);

    // 构建灰度级映射表
    Mat lut(1, 256, CV_8U);
    for (int i = 0; i < 256; i++)
    {
        // 查找源灰度级为ｉ的映射灰度
        //　和ｉ的累积概率差值最小的规定化灰度
        float min = diff_cdf[i][0];
        int index = 0;
        for (int j = 1; j < 256; j++)
        {
            if (min > diff_cdf[i][j])
            {
                min = diff_cdf[i][j];
                index = j;
            }
        }
        lut.at<uchar>(i) = static_cast<uchar>(index);
    }

    // 应用查找表，做直方图规定化
    LUT(src, lut, result);
}

#if 0
int main()
{
    /****************显示图像的直方图******************/
    Histogram1D hist1;
    Mat img1 = imread(IMAGE1_PATH);
    Mat histImg1 = hist1.getHistogramImage(img1);

    imshow("Image1", img1);
    imshow("Histogram1", histImg1);

    /*****************直方图均衡*********************/
    Mat equImg = Mat::zeros(img1.rows, img1.cols, img1.type());
    EqualizeImage(img1, equImg);
    Histogram1D hist2;
    Mat histImg2 = hist2.getHistogramImage(equImg);

    imshow("Equalized Image1", equImg);
    imshow("Histogram2", histImg2);

    /*****************直方图规定化*******************/
    Mat img2 = imread(IMAGE2_PATH);
    Mat img3 = imread(IMAGE3_PATH);
    Mat specifyImg = Mat::zeros(img2.rows, img2.cols, img2.type());
    HistSpecify(img2, img3, specifyImg);

    Histogram1D hist3;
    Mat histImg3 = hist3.getHistogramImage(img2);
    Histogram1D hist4;
    Mat histImg4 = hist4.getHistogramImage(img3);
    Histogram1D hist5;
    Mat histImg5 = hist5.getHistogramImage(specifyImg);

    imshow("Image2", img2);
    imshow("Histogram3", histImg3);
    imshow("Image3", img3);
    imshow("Histogram4", histImg4);
    imshow("Specify Image", specifyImg);
    imshow("Histogram5", histImg5);

    waitKey();
    return 0;
}
#endif
