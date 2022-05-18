#pragma once
#include "model_base.h"

namespace nullmax_perception {
namespace calibration {
class Pinhole : public ModelBase {
public:
  using Ptr = std::shared_ptr<Pinhole>;
  Pinhole(const CameraConfig &config) : ModelBase(config) {}
  Pinhole() = delete;
  Pinhole &operator=(const Pinhole &) = delete;
  Pinhole(const Pinhole &) = delete;
  virtual ~Pinhole() = default;

  bool RunCalibration(
      const std::vector<std::vector<cv::Point2f>> &image_points) override;
  bool SaveResult() override;
  cv::Mat Undistort(const cv::Mat &distorted_img) override;
  void CalReprojectError() override;
  bool ReadIntrinsics(const std::string &intrinsic_file) override;

private:
  double IterativeCalib(const double &init_rms, const int &calib_flag);
  std::vector<cv::Mat> CalParallalR();
  std::vector<cv::Point2f>
  ConvertCorners(const std::vector<cv::Point2f> &pixels1,
                 const cv::Mat &intrinsic1, const cv::Mat &r1,
                 const cv::Mat &t1, const std::vector<double> &dist_coeffs1,
                 const cv::Mat &intrinsic2, const cv::Mat &r2,
                 const cv::Mat &t2, const std::vector<double> &dist_coeffs2);

private:
  cv::Mat intrinsic_;
  cv::Mat dist_coeffs_;
  std::vector<cv::Mat> rvecs_, tvecs_;
};
} // calibration
} // nullmax_perception