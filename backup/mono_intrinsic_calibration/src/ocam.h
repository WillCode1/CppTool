#pragma once
#include "model_base.h"
#include "ocam_algrithom.h"

namespace nullmax_perception {
namespace calibration {
class Ocam : public ModelBase {
public:
  using Ptr = std::shared_ptr<Ocam>;
  Ocam(const CameraConfig &config) : ModelBase(config) {}
  Ocam() = delete;
  Ocam &operator=(const Ocam &) = delete;
  Ocam(const Ocam &) = delete;
  virtual ~Ocam() = default;

  bool RunCalibration(
      const std::vector<std::vector<cv::Point2f>> &image_points) override;
  bool SaveResult() override;
  cv::Mat Undistort(const cv::Mat &distorted_img) override;
  void CalReprojectError() override;
  bool ReadIntrinsics(const std::string &intrinsic_file) override;

private:
  cv::Point2f CameraToPixel(const cv::Point3f &point);
  bool CalVirtualf();

private:
  std::shared_ptr<OcamIntrinsics> intrinsic_;
  std::vector<double> repro_errors_;
  float left_max_;
  float right_min_;
  float up_max_;
  float down_min_;
};
} // calibration
} // nullmax_perception