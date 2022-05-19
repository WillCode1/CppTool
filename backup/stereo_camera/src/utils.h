#pragma once

#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "json/json.h"
#include "log.h"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace std;
using namespace cv;

bool parse_calib_params(const std::string &config_path,
                        std::vector<std::string> &imagelist,
                        cv::Size &boardSize,float &squareSize);

static void
SaveStereoIntrinsic(const Mat &leftCameraMatrix, const Mat &rightCameraMatrix,
        const Mat &leftDistCoeffs, const Mat &rightDistCoeffs);
static void
SaveStereoExtrinsic(const Mat &R, const Mat &T,
        const Mat &R1, const Mat &R2,
        const Mat &P1, const Mat &P2, const Mat &Q);

static bool readStringList( const std::string& filename, std::vector<std::string>& l )
{
    l.resize(0);
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != FileNode::SEQ )
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        l.push_back((string)*it);
    return true;
};

// int print_help()
// {
//     cout <<
//             " Given a list of chessboard images, the number of corners (nx, ny)\n"
//             " on the chessboards, and a flag: useCalibrated for \n"
//             "   calibrated (0) or\n"
//             "   uncalibrated \n"
//             "     (1: use stereoCalibrate(), 2: compute fundamental\n"
//             "         matrix separately) stereo. \n"
//             " Calibrate the cameras and display the\n"
//             " rectified results along with the computed disparity images.   \n" << endl;
//     cout << "Usage:\n ./stereo_calib -w=<board_width default=9> -h=<board_height default=6> -s=<square_size default=1.0> <image list json file default=stereo_calib.json>\n" << endl;
//     return 0;
// };

bool
StereoCalib(const vector<string>& imagelist, Size boardSize, float squareSize,
            bool displayCorners, bool useCalibrated,
            bool showRectified, bool fix_intrinstic);

float judge_polar_metric(const std::vector<cv::Point2f>& l_pts,
                        const std::vector<cv::Point2f>& r_pts,
                        const cv::Size &boardSize);
