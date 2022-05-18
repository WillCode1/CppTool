#include "front_camera_extrinsic_adjustment_manual.h"
#include "cv_utils.h"
#include "eigen_cv_utils.h"
#include "verify_extrinsic.h"

namespace nullmax_perception {

int FrontCameraExtrinsicAdjustmentManual::InitCameraParam(
    const CameraIntrinsic &intrinsic, const CameraDistortCoef &distort_coef,
    const float &camera_height, const int &image_width,
    const int &image_height) {
  intrinsic_ = intrinsic;
  distort_coef_ = distort_coef;
  camera_height_ = camera_height;
  is_camera_param_init_ = true;
  image_width_ = image_width;
  image_height_ = image_height;
  is_verify_camera_init_ = false;
  return 0;
}

int FrontCameraExtrinsicAdjustmentManual::UndistortImage(
    const cv::Mat &image_src, cv::Mat &image_dst) {
  if (!is_camera_param_init_) {
    return 1;
  }

  extrinsic_calibrator_.LoadImage(image_src);
  extrinsic_calibrator_.LoadParameter(camera_height_, intrinsic_,
                                      distort_coef_);
  cv::Mat image_float = extrinsic_calibrator_.Undistortion(0.0f, 255.0f, 1);
  Convert8U3C(image_float, image_dst, image_float.rows, image_float.cols);
  return 0;
}

int FrontCameraExtrinsicAdjustmentManual::VerifyCameraInit(
    const cv::Mat &image_data,  // image without distortion
    const UVIPMRoi &uv_ipm_roi, // predefine lane roi
    const CameraRotationEuler &rotation_angle) {
  cv::Mat image_uv;
  if (image_data.channels() == 3) {
    cv::cvtColor(image_data, image_uv, cv::COLOR_BGR2GRAY);
    image_uv_color_ = image_data.clone();
  } else {
    image_uv = image_data.clone();
    cv::cvtColor(image_data, image_uv_color_, cv::COLOR_GRAY2BGR);
  }

  int rows = image_uv.rows;
  int cols = image_uv.cols;
  std::cout << "image size: #rows = " << rows << ", #cols = " << cols
            << std::endl;

  verify_camera_.Init(intrinsic_, distort_coef_, camera_height_, image_width_,
                      image_height_, rotation_angle, uv_ipm_roi);

  Mat image_uv_f;
  image_uv.convertTo(image_uv_f, CV_32F);

  CvMat2EigenMat(image_uv_f, image_uv_eigen_);

  dist_verticals_.resize(0);
  dist_verticals_.push_back(5.0);
  dist_verticals_.push_back(7.50);
  dist_verticals_.push_back(10.0);
  dist_verticals_.push_back(12.50);
  dist_verticals_.push_back(15.0);
  dist_verticals_.push_back(20.0);
  dist_verticals_.push_back(30.0);
  dist_verticals_.push_back(40.0);
  dist_verticals_.push_back(50.0);
  dist_verticals_.push_back(60.0);
  // dist_verticals_.push_back(25);

  is_verify_camera_init_ = true;

  return 0;
}

int FrontCameraExtrinsicAdjustmentManual::VerifyCameraFlush(const int &width,
                                                            const int &height,
                                                            cv::Mat &image) {
  if (!is_verify_camera_init_) {
    return 1;
  }
  verify_camera_.Verify(image_uv_color_, image_uv_eigen_, dist_verticals_,
                        width, height, image);

  return 0;
}

int FrontCameraExtrinsicAdjustmentManual::VerifyCameraAdjustPitch(bool is_add) {
  if (!is_verify_camera_init_) {
    return 1;
  }
  verify_camera_.AdjustPitch(is_add);
  return 0;
}
int FrontCameraExtrinsicAdjustmentManual::VerifyCameraAdjustYaw(bool is_add) {
  if (!is_verify_camera_init_) {
    return 1;
  }
  verify_camera_.AdjustYaw(is_add);
  return 0;
}
int FrontCameraExtrinsicAdjustmentManual::VerifyCameraAdjustRoll(bool is_add) {
  if (!is_verify_camera_init_) {
    return 1;
  }
  verify_camera_.AdjustRoll(is_add);
  return 0;
}

CameraRotationEuler FrontCameraExtrinsicAdjustmentManual::GetRotationAngle() {
  CameraRotationEuler rot;
  rot.pitch = verify_camera_.camera_info_.angle_pitch * 180.0 / CV_PI;
  rot.yaw = verify_camera_.camera_info_.angle_yaw * 180.0 / CV_PI;
  rot.roll = verify_camera_.camera_info_.angle_roll * 180.0 / CV_PI;
  return rot;
}

} // namespace nullmax_perception
