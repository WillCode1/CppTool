#ifndef _FUNCTION_WRITE_
#define _FUNCTION_WRITE_
#include "camera_calib_utils.h"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <parameter_struct.h>

cv::Mat FindLane(const cv::Mat &img, const cv::Mat &mask,
                 std::vector<double> &vx, std::vector<double> &vy, bool isLeft);

void CalculateLine(const std::vector<double> &vx, const std::vector<double> &vy,
                   double &k, double &b);

int CalculateRawPitch(const nullmax_perception::CameraIntrinsic &intrinsic,
                      std::vector<nullmax_perception::LaneCoef> &lane_data,
                      cv::Point2f &vanish_point, float &pitch_raw);

void OptimizePitchYaw(const std::vector<LaneVanishPoint> &lane_data,
                      const float &camera_height,
                      const nullmax_perception::CameraIntrinsic &intrinsic,
                      float &pitch_optimize, float &yaw_optimize);

void CalBearings(const nullmax_perception::CameraIntrinsic &intrinsic,
                 const double k, const double b, Eigen::Vector3d &b1,
                 Eigen::Vector3d &b2);

void OptimizePitchYawSingle(
    const LaneVanishPoint &lane_data, const float &camera_height,
    const nullmax_perception::CameraIntrinsic &intrinsic,
    float &pitch_optimize);

cv::Mat
CreateBirdView(const cv::Mat &img,
               const nullmax_perception::CameraRotationEuler &rotation_angle,
               const nullmax_perception::CameraIntrinsic &intrinsic,
               const float &camera_height);

#endif
