#ifndef EXTRINSIC_CALIBRATOR_H
#define EXTRINSIC_CALIBRATOR_H

#include "Eigen/Dense"
#include "camera_calib_utils.h"
//#include "eigen3/Eigen/Dense"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"

#define PI 3.14159265

class ExtrinsicCalibrator {
public:
  ExtrinsicCalibrator();

  ~ExtrinsicCalibrator();

  void init();

  cv::Mat Undistortion(float alpha, float maxrange, int mode);

  void LoadImage(std::string filename);
  void LoadImage(const cv::Mat &image_color);

  void LoadParameter(std::string str_camera_info);
  void LoadParameter(const float &camera_height,
                     nullmax_perception::CameraIntrinsic &intrinsic,
                     nullmax_perception::CameraDistortCoef &distort_coef);

private:
  Eigen::MatrixXf cc_;
  Eigen::MatrixXf fc_;
  Eigen::MatrixXf distcoeffs_;

  float scale_;

  int image_width_;
  int image_height_;

public:
  cv::Mat image_gray_;
  cv::Mat undistortion_image_;
};

#endif // EXTRINSIC_CALIBRATOR_H
