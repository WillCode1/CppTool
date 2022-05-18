#ifndef FISHEYE_OCAM_PARAMETER_H_
#define FISHEYE_OCAM_PARAMETER_H_

#include "camera_param.h"

namespace nullmax_perception {

class FisheyeOcamParam : public CameraParam {

public:
  FisheyeOcamParam(CameraModelParam &model_param)
      : camera_model_param_(model_param){};
  FisheyeOcamParam(const std::string &calibration_file);
  ~FisheyeOcamParam() {}

  virtual void Load(const std::string &calbration_file) override;

  virtual cv::Point2f World2Camera(cv::Point3f pt2) override;
  virtual cv::Point3f Camera2World(cv::Point2f pt2) override;

  virtual int Undistortion(const cv::Mat &src, cv::Mat &dst) override;

  virtual CameraModel getCameraModelType() override;
  virtual CameraModelParam getCameraModelParam() override;

private:
  CameraModelParam camera_model_param_;
  cv::Mat mapx_persp_;  // 畸变表
  cv::Mat mapy_persp_;
};

inline CameraModel FisheyeOcamParam::getCameraModelType() {
  CameraModel camera_model;
  camera_model = FISHEYE_OCAM_MODEL;
  return camera_model;
}

inline CameraModelParam FisheyeOcamParam::getCameraModelParam() {
  return camera_model_param_;
}
} // namespace nullmax_perception
#endif // FISHEYE_OCAM_PARAMETER_H_
