#ifndef INCLUDE_PARAMETER_STRUCT_H_
#define INCLUDE_PARAMETER_STRUCT_H_
#include "camera_calib_utils.h"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>

struct LaneVanishPoint {
  std::vector<nullmax_perception::LaneCoef> lane_coef;
  nullmax_perception::VanishPoint vanish_point;
  float pitch_raw;
};

struct Point2D32f {
  float x;
  float y;
};

struct LineFittingResidual {
  LineFittingResidual(double x, double y) : x_(x), y_(y) {}
  template <typename T>
  bool operator()(const T *const k, const T *const b, T *residual) const {
    residual[0] = y_ - (k[0] * x_ + b[0]);
    return true;
  }

private:
  const double x_;
  const double y_;
};

struct CostFatorPitchYaw {
  CostFatorPitchYaw(Eigen::Vector3d left_start, Eigen::Vector3d left_end,
                    Eigen::Vector3d right_start, Eigen::Vector3d right_end,
                    double height)
      : left_start_(left_start), left_end_(left_end), right_start_(right_start),
        right_end_(right_end), height_(height) {}
  bool operator()(const double *const pitch, const double *const yaw,
                  double *residual) const {

    cv::Mat Twc = cv::Mat::zeros(4, 4, CV_64F);
    Twc.at<double>(0, 3) = 0;
    Twc.at<double>(1, 3) = -height_;
    Twc.at<double>(2, 3) = 0;
    Twc.at<double>(3, 3) = 1.0;

    Eigen::Matrix3d Rx =
        (Eigen::AngleAxisd(pitch[0], Eigen::Vector3d::UnitX())).matrix();
    Eigen::Matrix3d Ry =
        (Eigen::AngleAxisd(-yaw[0], Eigen::Vector3d::UnitY())).matrix();
    Eigen::Matrix3d Rz =
        (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())).matrix();
    //         Eigen::Matrix3d R = Rz * Ry * Rx;
    Eigen::Matrix3d R = Rx * Rz * Ry;

    cv::Mat Rcw = (cv::Mat_<double>(3, 3) << R(0, 0), R(0, 1), R(0, 2), R(1, 0),
                   R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));
    cv::Mat Rwc = Rcw.inv();

    Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));

    cv::Mat p0_start = (cv::Mat_<double>(3, 1) << left_start_(0),
                        left_start_(1), left_start_(2));
    double k0_start = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p0_start)));
    cv::Mat pw0_start = Rwc * (k0_start * p0_start) + Twc.rowRange(0, 3).col(3);

    cv::Mat p0_end =
        (cv::Mat_<double>(3, 1) << left_end_(0), left_end_(1), left_end_(2));
    double k0_end = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p0_end)));
    cv::Mat pw0_end = Rwc * (k0_end * p0_end) + Twc.rowRange(0, 3).col(3);

    cv::Mat p1_start = (cv::Mat_<double>(3, 1) << right_start_(0),
                        right_start_(1), right_start_(2));
    double k1_start = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p1_start)));
    cv::Mat pw1_start = Rwc * (k1_start * p1_start) + Twc.rowRange(0, 3).col(3);

    cv::Mat p1_end =
        (cv::Mat_<double>(3, 1) << right_end_(0), right_end_(1), right_end_(2));
    double k1_end = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p1_end)));
    cv::Mat pw1_end = Rwc * (k1_end * p1_end) + Twc.rowRange(0, 3).col(3);

    double z0_start = pw0_start.at<double>(2, 0);
    double x0_start = pw0_start.at<double>(0, 0);

    double z0_end = pw0_end.at<double>(2, 0);
    double x0_end = pw0_end.at<double>(0, 0);

    double k = (z0_end - z0_start) / (x0_end - x0_start);
    double b = z0_end - k * x0_end;
    double r1 = (13.4 - b) / k;
    double r2 = (4 - b) / k;

    double z1_start = pw1_start.at<double>(2, 0);
    double x1_start = pw1_start.at<double>(0, 0);

    double z1_end = pw1_end.at<double>(2, 0);
    double x1_end = pw1_end.at<double>(0, 0);

    double k_ = (z1_end - z1_start) / (x1_end - x1_start);
    double b_ = z1_end - k_ * x1_end;
    double r3 = (13.4 - b_) / k_;
    double r4 = (4 - b_) / k_;
    residual[0] = fabs(r1 - r3) - fabs(r2 - r4) + fabs(r1 - r2);
    return true;
  }

private:
  const Eigen::Vector3d left_start_;
  const Eigen::Vector3d left_end_;
  const Eigen::Vector3d right_start_;
  const Eigen::Vector3d right_end_;
  const double height_;
};

struct CostFatorPitch {
  CostFatorPitch(Eigen::Vector3d left_start, Eigen::Vector3d left_end,
                 Eigen::Vector3d right_start, Eigen::Vector3d right_end,
                 double height)
      : left_start_(left_start), left_end_(left_end), right_start_(right_start),
        right_end_(right_end), height_(height) {}
  bool operator()(const double *const pitch, double *residual) const {

    // calcualte bearings intersetion with ground //
    cv::Mat Twc = cv::Mat::zeros(4, 4, CV_64F);
    Twc.at<double>(0, 3) = 0;
    Twc.at<double>(1, 3) = -height_;
    Twc.at<double>(2, 3) = 0;
    Twc.at<double>(3, 3) = 1.0;

    Eigen::Matrix3d Rx =
        (Eigen::AngleAxisd(pitch[0], Eigen::Vector3d::UnitX())).matrix();
    Eigen::Matrix3d Ry =
        (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())).matrix();
    Eigen::Matrix3d Rz =
        (Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())).matrix();
    Eigen::Matrix3d R = Rx * Rz * Ry;

    cv::Mat Rcw = (cv::Mat_<double>(3, 3) << R(0, 0), R(0, 1), R(0, 2), R(1, 0),
                   R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));

    cv::Mat Rwc = Rcw.inv();

    Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));

    cv::Mat p0_start = (cv::Mat_<double>(3, 1) << left_start_(0),
                        left_start_(1), left_start_(2));
    double k0_start = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p0_start)));
    cv::Mat pw0_start = Rwc * (k0_start * p0_start) + Twc.rowRange(0, 3).col(3);

    cv::Mat p0_end =
        (cv::Mat_<double>(3, 1) << left_end_(0), left_end_(1), left_end_(2));
    double k0_end = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p0_end)));
    cv::Mat pw0_end = Rwc * (k0_end * p0_end) + Twc.rowRange(0, 3).col(3);

    cv::Mat p1_start = (cv::Mat_<double>(3, 1) << right_start_(0),
                        right_start_(1), right_start_(2));
    double k1_start = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p1_start)));
    cv::Mat pw1_start = Rwc * (k1_start * p1_start) + Twc.rowRange(0, 3).col(3);

    cv::Mat p1_end =
        (cv::Mat_<double>(3, 1) << right_end_(0), right_end_(1), right_end_(2));
    double k1_end = (-Twc.at<double>(1, 3) / (Rwc.row(1).t().dot(p1_end)));
    cv::Mat pw1_end = Rwc * (k1_end * p1_end) + Twc.rowRange(0, 3).col(3);

    double z0_start = pw0_start.at<double>(2, 0);
    double x0_start = pw0_start.at<double>(0, 0);

    double z0_end = pw0_end.at<double>(2, 0);
    double x0_end = pw0_end.at<double>(0, 0);

    double k = (z0_end - z0_start) / (x0_end - x0_start);
    double b = z0_end - k * x0_end;
    double r1 = (13.4 - b) / k;
    double r2 = (4 - b) / k;

    double z1_start = pw1_start.at<double>(2, 0);
    double x1_start = pw1_start.at<double>(0, 0);

    double z1_end = pw1_end.at<double>(2, 0);
    double x1_end = pw1_end.at<double>(0, 0);

    double k_ = (z1_end - z1_start) / (x1_end - x1_start);
    double b_ = z1_end - k_ * x1_end;
    double r3 = (13.4 - b_) / k_;
    double r4 = (4 - b_) / k_;
    residual[0] = (r1 - r3) - (r2 - r4);
    return true;
  }

private:
  const Eigen::Vector3d left_start_;
  const Eigen::Vector3d left_end_;
  const Eigen::Vector3d right_start_;
  const Eigen::Vector3d right_end_;
  const double height_;
};

#endif // INCLUDE_PARAMETER_STRUCT_H_
