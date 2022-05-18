#pragma once

#include "model_factory.h"
#include "mono_intrinsic_interface.h"
#include "opencv2/opencv.hpp"
#include <dirent.h>

namespace nullmax_perception {
namespace calibration {

class MonoIntrinsic : public MonoIntrinsicInterface {
public:
  struct CalibrationConfig {
  public:
    std::string image_path;
    bool is_show_undistorted;
    ModelBase::CameraConfig camera_cfg;

    friend std::ostream &
    operator<<(std::ostream &os, const MonoIntrinsic::CalibrationConfig &cfg);
  };

  struct CalibDataBase {
  public:
    std::vector<cv::Mat> images;
    std::vector<std::vector<cv::Point2f>> corners;
    // pose = {X, Y, size, skew}
    std::vector<cv::Vec4f> poses;
    cv::Vec4f progress;
    cv::Vec4f last_progress;
    std::vector<std::vector<bool>> progress_bins;
    std::vector<std::vector<bool>> bin_flags;
  };

  MonoIntrinsic() {}
  ~MonoIntrinsic() {}

  virtual bool Init(const std::string &config_file) override;
  virtual bool Calib() override;
  virtual bool ViewUndistort(const std::string &distorted_image_file,
                             const std::string &intrinsic_file) override;

private:
  std::vector<std::string> GetFileNames(const std::string &file_path);
  bool ReadConfigFile(const std::string &config_file);
  bool ConfigValidCheck();
  cv::Mat Undistort(const cv::Mat &distorted_img);
  void InputData();
  std::vector<cv::Point2f> DetectCorners(const cv::Mat &img);
  void UpdateDatabase(const cv::Mat &img,
                      const std::vector<cv::Point2f> &corners);
  cv::Vec4f GetBoardPose(const std::vector<cv::Point2f> &corners);
  bool IsGoodPose(const cv::Vec4f &pose);
  bool GetProgress(const cv::Vec4f &pose);
  bool ComputeProgress(const cv::Vec4f &pose);
  void InitDatabase();
  bool ComputeProgress_ROS();

private:
  CalibrationConfig config_;
  ModelBase::Ptr camera_model_;
  CalibDataBase database_;
};

} // calibration
} // nullmax_perception