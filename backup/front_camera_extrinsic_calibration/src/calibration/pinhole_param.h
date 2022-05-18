#ifndef PINHOLE_PARAMETER_H_
#define PINHOLE_PARAMETER_H_

#include "camera_param.h"

namespace nullmax_perception {

class PinholeParam : public CameraParam {
public:
  PinholeParam(CameraModelParam &model_param)
      : camera_model_param_(model_param){};
  PinholeParam(const std::string &calibration_file);
  ~PinholeParam() {}

  virtual void Load(const std::string &calbration_file);

  virtual cv::Point2f World2Camera(cv::Point3f pt3) override;
  virtual cv::Point3f Camera2World(cv::Point2f pt2) override;

  virtual int Undistortion(const cv::Mat &src, cv::Mat &dst) override;
  // TODO: depart fisheye from pinhole
  void OpencvPinholeUndistortion(const cv::Mat &src, cv::Mat &dst);
  void OpencvFisheyeUndistortion(const cv::Mat &src, cv::Mat &dst);

  virtual CameraModel getCameraModelType() override;
  virtual CameraModelParam getCameraModelParam() override;

private:
  CameraModelParam camera_model_param_;
  cv::Mat camera_matrix_in_;
  cv::Mat camera_matrix_out_;
  cv::Mat distortion_coeffs_;
};

inline CameraModel PinholeParam::getCameraModelType() {
  CameraModel camera_model;
  camera_model = PINHOLE_MODEL;
  return camera_model;
}

inline CameraModelParam PinholeParam::getCameraModelParam() {
  return camera_model_param_;
}
}
#endif // PINHOLE_PARAMETER_H_