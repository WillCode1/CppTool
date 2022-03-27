#include "undistort.h"

bool CUndistort::readParams()
{
    ifstream in;
    in.open(calibResultPath+"calibResult.txt", ios::in);
    in>>K.at<float>(0, 0);
    in>>K.at<float>(1, 1);
    in>>K.at<float>(0, 2);
    in>>K.at<float>(1, 2);
#ifdef CV
    in >> discoeff.at<float>(0, 0);
    in >> discoeff.at<float>(1, 0);
    in >> discoeff.at<float>(2, 0);
    in >> discoeff.at<float>(3, 0);
    in >> discoeff.at<float>(4, 0);
#elif defined FISHEYE
    in >> discoeff.at<float>(0, 0);
	in >> discoeff.at<float>(1, 0);
	in >> discoeff.at<float>(2, 0);
	in >> discoeff.at<float>(3, 0);
#endif
    in.close();
    return true;
}

bool CUndistort::undistProcess()
{
    //***************У****************//
    R=Mat::eye(Size(3, 3),CV_32FC1);
    Mat mapx, mapy;
    Mat srcImg=imread(srcImgPath);
    Mat dstImg;
#ifdef CV
    cv::initUndistortRectifyMap(K, discoeff, R, K, srcImg.size(),CV_32FC1, mapx, mapy);
#elif defined FISHEYE
    cv::fisheye::initUndistortRectifyMap(K, discoeff,R, K, srcImg.size(), CV_32FC1, mapx, mapy);
#endif
    remap(srcImg, dstImg, mapx, mapy, cv::INTER_LINEAR);
    cv::resize(dstImg, dstImg, cv::Size(), 0.25, 0.25, cv::INTER_LINEAR);
    cv::namedWindow("show", 1);
    imshow("show", dstImg);
    waitKey(0);

    return true;
}

void CUndistort::run()
{
    bool readSuccess=readParams();
    if (!readSuccess)
    {
        cout << "read Params Failed!" << endl;
        getchar();
    }
    undistProcess();
}
