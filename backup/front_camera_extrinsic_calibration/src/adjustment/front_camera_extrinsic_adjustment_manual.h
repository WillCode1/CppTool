#ifndef FRONT_CAMERA_EXTRINSIC_ADJUSTMENT_MANUAL_
#define FRONT_CAMERA_EXTRINSIC_ADJUSTMENT_MANUAL_
#include "camera_calib_utils.h"
#include "extrinsic_calibrator.h"
#include "front_camera_extrinsic_adjustment_base.h"
#include "verify_extrinsic.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace nullmax_perception {

// TODO: Find a better name for this class.
class FrontCameraExtrinsicAdjustmentManual
    : public FrontCameraExtrinsicAdjustmentBase {
public:
  FrontCameraExtrinsicAdjustmentManual()
      : is_camera_param_init_(false), is_verify_camera_init_(false){};
  virtual ~FrontCameraExtrinsicAdjustmentManual(){};

  virtual int InitCameraParam(const CameraIntrinsic &intrinsic,
                              const CameraDistortCoef &distort_coef,
                              const float &camera_height,
                              const int &image_width,
                              const int &image_height) override;
  virtual int UndistortImage(const cv::Mat &image_src,
                             cv::Mat &image_dst) override;
  virtual int
  VerifyCameraInit(const cv::Mat &image_data,  // image without distortion
                   const UVIPMRoi &uv_ipm_roi, // predefine lane roi
                   const CameraRotationEuler &rotation_angle) override;
  virtual int VerifyCameraFlush(const int &width, const int &height,
                                cv::Mat &image) override;
  virtual int VerifyCameraAdjustPitch(bool is_add) override;
  virtual int VerifyCameraAdjustYaw(bool is_add) override;
  virtual int VerifyCameraAdjustRoll(bool is_add) override;
  virtual CameraRotationEuler GetRotationAngle() override;

private:
  bool is_camera_param_init_;
  CameraIntrinsic intrinsic_;
  CameraDistortCoef distort_coef_;
  float camera_height_;
  int image_width_;
  int image_height_;
  ExtrinsicCalibrator extrinsic_calibrator_;

  bool is_verify_camera_init_;
  VerifyCalibration verify_camera_;
  cv::Mat image_uv_color_;
  EigenMat image_uv_eigen_;
  std::vector<float> dist_verticals_;
};
} // namespace nullmax_perception
#endif // FRONT_CAMERA_EXTRINSIC_ADJUSTMENT_MANUAL_
