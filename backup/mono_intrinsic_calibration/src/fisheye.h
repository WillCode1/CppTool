#pragma once
#include "model_base.h"

namespace nullmax_perception {
namespace calibration {
class Fisheye : public ModelBase {
public:
  using Ptr = std::shared_ptr<Fisheye>;
  Fisheye(const CameraConfig &config) : ModelBase(config) {}
  Fisheye() = delete;
  Fisheye &operator=(const Fisheye &) = delete;
  Fisheye(const Fisheye &) = delete;
  virtual ~Fisheye() = default;

  bool RunCalibration(
      const std::vector<std::vector<cv::Point2f>> &image_points) override;
  bool SaveResult() override;
  cv::Mat Undistort(const cv::Mat &distorted_img) override;
  void CalReprojectError() override;
  bool ReadIntrinsics(const std::string &intrinsic_file) override;

private:
  cv::Mat intrinsic_;
  cv::Mat dist_coeffs_;
  std::vector<cv::Mat> rvecs_, tvecs_;
};
} // calibration
} // nullmax_perception