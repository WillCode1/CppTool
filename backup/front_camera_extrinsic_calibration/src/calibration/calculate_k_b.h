#ifndef INCLUDE_CALCULATE_K_B_H_
#define INCLUDE_CALCULATE_K_B_H_
// This function calculate the slope and intercept of lane
// using ransac algorithm

#include "camera_calib_utils.h"
#include "detect_lane.h"
#include "ransac_lane.h"

void CalculateKB(const cv::Mat img_my,
                 const nullmax_perception::RoiData &roi_data, float *lane_kb,
                 const int &canny_threadhold) {
  // save lane information.
  // line_information[3] : y-average
  // line_information[2] : x-average
  // line_information[1] : refer to least square to calcualte k
  // line_information[0] : refer to least square to calcualte k
  float line_infomation[4] = {0.0};

  std::vector<cv::Point> point_l;
  std::vector<cv::Point> point_r;
  // detect edge
  cv::Mat lane_edge =
      DetectLane(img_my, roi_data, point_l, point_r, canny_threadhold);

  // point in left img
  int count_num_l = point_l.size();
  Point2D32f pointdata_l[count_num_l];
  for (size_t m = 0; m < count_num_l; m++) {
    pointdata_l[m].x = point_l[m].y;
    pointdata_l[m].y = point_l[m].x;
  }
  // point in right img
  int count_num_r = point_r.size();
  Point2D32f pointdata_r[count_num_r];
  for (size_t m = 0; m < count_num_r; m++) {
    pointdata_r[m].x = point_r[m].y;
    pointdata_r[m].y = point_r[m].x;
  }

  // parameters of ransac
  // the number of choosen points every time.
  // do not choose too big number.
  // because ransac will calculate many times
  int num_for_estimate = 3;
  float k = 0.0, b = 0.0;
  float success_probability = 0.999f;
  float max_outliers_percentage = 0.9f;

  // left img line
  RansacLane2d(pointdata_l, count_num_l, num_for_estimate, success_probability,
               max_outliers_percentage, line_infomation, true);
  k = line_infomation[1] / line_infomation[0];
  b = line_infomation[3] - k * line_infomation[2];
  lane_kb[0] = k;
  lane_kb[1] = b;

  // right img line
  RansacLane2d(pointdata_r, count_num_r, num_for_estimate, success_probability,
               max_outliers_percentage, line_infomation, false);
  k = line_infomation[1] / line_infomation[0];
  b = line_infomation[3] - k * line_infomation[2];
  lane_kb[2] = k;
  lane_kb[3] = b;
}

#endif // INCLUDE_CALCULATE_K_B_H_
