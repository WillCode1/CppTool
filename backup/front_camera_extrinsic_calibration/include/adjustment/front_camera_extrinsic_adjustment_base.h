#ifndef FRONT_CAMERA_EXTRINSIC_ADJUSTMENT_BASE_
#define FRONT_CAMERA_EXTRINSIC_ADJUSTMENT_BASE_
#include "camera_calib_utils.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace nullmax_perception {

class FrontCameraExtrinsicAdjustmentBase {
public:
  virtual int InitCameraParam(const CameraIntrinsic &intrinsic,
                              const CameraDistortCoef &distort_coefconst,
                              const float &camera_height,
                              const int &image_width,
                              const int &image_height) = 0;
  virtual int UndistortImage(const cv::Mat &image_src, cv::Mat &image_dst) = 0;
  virtual int
  VerifyCameraInit(const cv::Mat &image_data,  // image without distortion
                   const UVIPMRoi &uv_ipm_roi, // predefine lane roi
                   const CameraRotationEuler &rotation_angle) = 0; // unit: deg
  virtual int VerifyCameraFlush(const int &width, const int &height,
                                cv::Mat &image) = 0;
  virtual int VerifyCameraAdjustPitch(bool is_add) = 0;
  virtual int VerifyCameraAdjustYaw(bool is_add) = 0;
  virtual int VerifyCameraAdjustRoll(bool is_add) = 0;
  virtual CameraRotationEuler GetRotationAngle() = 0; //  unit: deg

  virtual ~FrontCameraExtrinsicAdjustmentBase(){};
};

FrontCameraExtrinsicAdjustmentBase *
CreateFrontCameraExtrinsicAdjustmentHandle(const std::string &method);

void DestroyFrontCameraExtrinsicAdjustmentHandle(
    FrontCameraExtrinsicAdjustmentBase *handle);

} // namespace nullmax_perception
#endif // FRONT_CAMERA_EXTRINSIC_ADJUSTMENT_BASE_
