#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include "calibration.h"
#include "undistort.h"

using namespace std;
using namespace cv;

int main()
{
    string patternImgPath="data/pattern/";
    string calibResultPath="data/results/";
    string srcImgPath="data/srcImg/0.jpg";
    Size boardSize=Size(9, 6);
    CCalibration calibration(patternImgPath, calibResultPath, boardSize);
    calibration.run();
    CUndistort undistort(srcImgPath, calibResultPath);
    undistort.run();
    return 0;
}
