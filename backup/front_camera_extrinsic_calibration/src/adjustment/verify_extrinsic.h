#ifndef VERIFY_EXTRINSIC_H
#define VERIFY_EXTRINSIC_H

#include "transform_ipm.h"
#include "util_structure.h"

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "camera_calib_utils.h"

using namespace cv;
using namespace std;

namespace nullmax_calibration {

// lane detection feature
// put everything in class to support fast debuging and testing
class VerifyCalibration {

public:
  VerifyCalibration(){};
  virtual ~VerifyCalibration() {}

public:
  // interface of VerifyCalibration
  void Init(const std::string &str_camera_info,
            const std::string &str_ipm_info);
  void Init(const nullmax_perception::CameraIntrinsic &intrinsic,
            const nullmax_perception::CameraDistortCoef &distort_coef,
            const float &camera_height, const int &image_width,
            const int &image_height,
            const nullmax_perception::CameraRotationEuler &rotation_angle,
            const nullmax_perception::UVIPMRoi &uv_ipm_roi);

  void Verify(const Mat &image_uv_color, const EigenMat &image_uv,
              vector<float> dist_verticals, const int &window_width,
              const int &window_height, cv::Mat &image_in_out);

  void AdjustPitch(bool is_add);
  void AdjustYaw(bool is_add);
  void AdjustRoll(bool is_add);

protected:
  void ComputeIpm(const EigenMat &image_uv, EigenMat &image_ipm);

private:
  void loadCameraInfo(const std::string &str_camera_info);
  void
  loadCameraInfo(const float &camera_height, const int &image_width,
                 const int &image_height,
                 const nullmax_perception::CameraIntrinsic &intrinsic,
                 const nullmax_perception::CameraDistortCoef &distort_coef,
                 const nullmax_perception::CameraRotationEuler &rotation_angle);

  void loadIpmInfo(const std::string &str_ipm_info);
  void loadIpmInfo(const nullmax_perception::UVIPMRoi &uv_ipm_roi);

  void updateIpm();

  void prepareTf();
  void prepareTfGround2UV();
  void prepareTfUV2Ground();
  void prepareDistanceLimit();

  // efficient calling without init tf in the calling
  // tf is init only once
  // tf-(ground <-> ipm) is already efficient in tf_ipm
  void transformGround2Image(const EigenMat &mat_ground, EigenMat &mat_uv);
  void transformImage2Ground(const EigenMat &mat_uv, EigenMat &mat_ground);

  void transformImIPM2Im(const EigenMat &mat_ipm, EigenMat &mat_uv);
  void transformIm2ImIPM(const EigenMat &mat_uv, EigenMat &mat_ipm);

  void transformImIPM2Ground(const EigenMat &mat_ipm, EigenMat &mat_ground);
  void transformGround2ImIPM(const EigenMat &mat_ground, EigenMat &mat_ipm);

  Point2f getVanishingPoint();
  void printCameraAngle();

private:
  // protected:
public:
  IPMInfo ipm_info_;       // ipm info
  CameraInfo camera_info_; // camera info

  EigenMat image_uv_;
  Mat image_uv_color_;
  EigenMat image_ipm_;

private:
  EigenMat mat_tf_ground2uv_; // 3x3
  EigenMat mat_tf_uv2ground_; // 4x3

  EigenMat mat_grid_uv_; // for ipm

  int max_uv_y_;

private:
  EigenMat mat_pts_uv_;
  EigenMat mat_pts_ground_;

}; // end of VerifyCalibration

} // namespace nullmax_calibration

#endif // VERIFY_EXTRINSIC_H
