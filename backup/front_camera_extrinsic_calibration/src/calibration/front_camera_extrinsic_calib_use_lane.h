#ifndef FRONT_CAMERA_EXTRINSIC_CALIB_A_
#define FRONT_CAMERA_EXTRINSIC_CALIB_A_
#include "parameter_struct.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace nullmax_perception {

// TODO: Find a better name for this class.
class FrontCameraExtrinsicCalibUseLane : public FrontCameraExtrinsicCalibBase {
public:
  FrontCameraExtrinsicCalibUseLane() : is_camera_param_init_(false){};

  virtual ~FrontCameraExtrinsicCalibUseLane(){};

  virtual int InitCameraParam(CameraParam *camera_param,
                              const float &camera_height) override;
  virtual int UndistortImage(const cv::Mat &image_src,
                             cv::Mat &image_dst) override;
  virtual bool DetectLane(const cv::Mat &image_src, // image without distortion
                         const RoiData &roi_data,  // predefine lane roi
                         const int &edge_thread, // lane detect edge threadhold
                         std::vector<LaneCoef> &lane_coef,
                         VanishPoint &vanish_point, float &pitch_raw) override;
  virtual int
  CalcFrontCameraExtrinsic(const std::vector<std::vector<LaneCoef>>
                               &lane_coef, // each image has multi lane
                           const std::vector<VanishPoint>
                               &vanish_point, // each image has one vanish point
                           const std::vector<float> &
                               pitch_raw, // raw pitch calculate by DetectLane()
                           CameraRotationEuler &euler_angle) override;
  virtual int BirdviewGenerator(const CameraRotationEuler &euler_angle,
                                const cv::Mat &image_data,
                                cv::Mat &bird_view) override;

private:
  static bool LaneCompare_(const LaneVanishPoint &a, const LaneVanishPoint &b) {
    return a.vanish_point.y < b.vanish_point.y;
  }

  bool is_camera_param_init_;
  CameraParam *camera_param_;
  float camera_height_;

  int undistortPinholeModel(const cv::Mat &image_src, cv::Mat &image_dst);
};
} // namespace nullmax_perception
#endif // FRONT_CAMERA_EXTRINSIC_CALIB_A_
