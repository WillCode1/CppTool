#ifndef INCLUDE_DETECT_LANE_H_
#define INCLUDE_DETECT_LANE_H_
// Author:slyang@nullmax.ai
// Time:2019-2-14
// step1:use ROI1 and ROI2 to select region of interest in img.
// step2:use canny operator to get edge.
// step3:use scharr operator to choose the lane.

#include "camera_calib_utils.h"
#include "parameter_struct.h"
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

cv::Mat DetectLane(const cv::Mat img_my,
                   const nullmax_perception::RoiData &roi_data,
                   std::vector<cv::Point> &point_l,
                   std::vector<cv::Point> &point_r,
                   const int &canny_threadhold);

#endif // INCLUDE_DETECT_LANE_H_
