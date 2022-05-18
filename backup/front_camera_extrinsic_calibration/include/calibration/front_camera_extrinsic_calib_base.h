#ifndef FRONT_CAMERA_EXTRINSIC_CALIB_BASE_
#define FRONT_CAMERA_EXTRINSIC_CALIB_BASE_
#include "camera_calib_utils.h"
#include "camera_param.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace nullmax_perception {

class FrontCameraExtrinsicCalibBase {
public:
  virtual int InitCameraParam(CameraParam *camera_param,
                              const float &camera_height) = 0;
  virtual int UndistortImage(const cv::Mat &image_src, cv::Mat &image_dst) = 0;
  virtual bool
  DetectLane(const cv::Mat &image_src, // image without distortion
             const RoiData
                 &roi_data, // predefine lane roi; ROI2.theta's units is degree
             const int &edge_thread, // lane detect edge threadhold
             std::vector<LaneCoef> &lane_coef, VanishPoint &vanish_point,
             float &pitch_raw // units: rad
             ) = 0;
  virtual int
  CalcFrontCameraExtrinsic(const std::vector<std::vector<LaneCoef>>
                               &lane_coef, // each image has multi lane
                           const std::vector<VanishPoint>
                               &vanish_point, // each image has one vanish point
                           const std::vector<float> &pitch_raw, // units: rad
                           CameraRotationEuler &euler_angle     // units: rad
                           ) = 0;
  virtual int
  BirdviewGenerator(const CameraRotationEuler &euler_angle, // units: rad
                    const cv::Mat &image_data, cv::Mat &bird_view) = 0;

  virtual ~FrontCameraExtrinsicCalibBase(){};
};

FrontCameraExtrinsicCalibBase *
CreateFrontCameraExtrinsicCalibHandle(const std::string &method);
void DestroyFrontCameraExtrinsicCalibHandle(
    FrontCameraExtrinsicCalibBase *handle);

} // namespace nullmax_perception
#endif // FRONT_CAMERA_EXTRINSIC_CALIB_BASE_
