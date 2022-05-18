#pragma once
#include "opencv2/opencv.hpp"
#include <iostream>
#include <memory>
#include <time.h>

namespace nullmax_perception {
namespace calibration {
class ModelBase {
public:
  struct CameraConfig {
  public:
    std::string calib_type;
    cv::Size board_size;
    cv::Size img_size;
    float board_unit_length;
    bool is_show_undistorted;
    bool pinhole_fix_p;
    bool pinhole_fix_k3;
    std::string undistort_type;
    bool use_FOV;
    float target_FOV;
    bool use_specified_focal_length;
    double specified_fx;
    double specified_fy;
  };
  using Ptr = std::shared_ptr<ModelBase>;
  ModelBase(const CameraConfig &config) : config_(config){};
  virtual ~ModelBase(){};

  ModelBase() = delete;
  ModelBase &operator=(const ModelBase &) = delete;
  ModelBase(const ModelBase &) = delete;

  virtual bool
  RunCalibration(const std::vector<std::vector<cv::Point2f>> &image_points) = 0;
  virtual bool SaveResult() = 0;
  virtual cv::Mat Undistort(const cv::Mat &distorted_img) = 0;
  virtual void CalReprojectError() = 0;
  virtual bool ReadIntrinsics(const std::string &intrinsic_file) = 0;

  bool ReadRemapFile(const std::string &remap_file);
  void SetImages(const std::vector<cv::Mat> &images) { images_ = images; };

  std::string GetCalibType() const { return config_.calib_type; }
  cv::Size GetImageSize() const { return config_.img_size; }
  cv::Size GetBoardSize() const { return config_.board_size; }
  float GetBoardUnitLength() const { return config_.board_unit_length; }

protected:
  bool SavePixelError();
  void ShowPixelError();
  bool SaveRemapFile();

protected:
  CameraConfig config_;
  std::vector<cv::Mat> images_;
  std::vector<std::vector<cv::Point2f>> image_points_;
  std::vector<std::vector<cv::Point3f>> object_points_;
  std::vector<std::vector<cv::Point2f>> pixel_repro_errors_;
  std::pair<cv::Mat, cv::Mat> undistort_remap_;
};
} // calibration
} // nullmax_perception