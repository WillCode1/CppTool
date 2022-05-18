#include "front_camera_extrinsic_calib_base.h"

#include <string>
#include <vector>

#include "calculate_k_b.h"
#include "detect_lane.h"
#include "front_camera_extrinsic_calib_use_lane.h"
#include "parameter_struct.h"
#include <iostream>
#include <utils.h>

namespace nullmax_perception {

int FrontCameraExtrinsicCalibUseLane::InitCameraParam(
    CameraParam *camera_param, const float &camera_height) {
  if (camera_param == NULL) {
    return 1;
  }
  camera_param_ = camera_param;
  camera_height_ = camera_height;
  is_camera_param_init_ = true;
  return 0;
}

int FrontCameraExtrinsicCalibUseLane::UndistortImage(const cv::Mat &image_src,
                                                     cv::Mat &image_dst) {
  if (!is_camera_param_init_) {
    std::cout << "camera param not init \n";
    return 1;
  }

  return camera_param_->Undistortion(image_src, image_dst);
}

bool FrontCameraExtrinsicCalibUseLane::DetectLane(
    const cv::Mat &image_src, const RoiData &roi_data, const int &edge_thread,
    std::vector<LaneCoef> &lane_coef, VanishPoint &vanish_point,
    float &pitch_raw) {
  // lane_kb : save two lane marking k and b
  // lane_kb[0]: k of left lane
  // lane_kb[1]: b of left lane
  // lane_kb[2]: k of right lane
  // lane_kb[3]: b of right lane
  float lane_kb[4] = {0.0f};
  CalculateKB(image_src, roi_data, lane_kb, edge_thread);
  std::cout << "k1: " << lane_kb[0] << "  b1: " << lane_kb[1] << std::endl;
  std::cout << "k2: " << lane_kb[2] << "  b2: " << lane_kb[3] << std::endl;
  LaneCoef lane_coef_left;
  LaneCoef lane_coef_right;
  lane_coef_left.valid = true;
  lane_coef_left.coef_k = lane_kb[0];
  lane_coef_left.coef_b = lane_kb[1];
  lane_coef_right.valid = true;
  lane_coef_right.coef_k = lane_kb[2];
  lane_coef_right.coef_b = lane_kb[3];
  lane_coef.push_back(lane_coef_left);
  lane_coef.push_back(lane_coef_right);

  // return vanishing point-inter
  cv::Point2f inter;
  CameraIntrinsic intrinsic =
      camera_param_->getCameraModelParam().pinhole.intrinsic;
  CalculateRawPitch(intrinsic, lane_coef, inter, pitch_raw);
  // vanishing point-y
  // lane_data.inter_v = inter.y;
  vanish_point.valid = true;
  vanish_point.x = inter.x;
  vanish_point.y = inter.y;

  if(std::isnan(pitch_raw))
    return false;
  return true;
}

int FrontCameraExtrinsicCalibUseLane::CalcFrontCameraExtrinsic(
    const std::vector<std::vector<LaneCoef>>
        &lane_coef, // each image has multi lane
    const std::vector<VanishPoint>
        &vanish_point, // each image has one vanish point
    const std::vector<float> &pitch_raw, CameraRotationEuler &euler_angle) {
  if (!is_camera_param_init_) {
    std::cout << "camera param not init. \n";
    return 1;
  }

  if (lane_coef.size() != vanish_point.size() ||
      lane_coef.size() != pitch_raw.size()) {
    std::cout << "lane_coef ||vanish_point ||pitch_raw size not equal. \n";
    return 1;
  }

  // filter the lane coef
  std::vector<LaneVanishPoint> init_lane_data;
  std::vector<LaneVanishPoint> lane_datas;
  const int init_thredhold_count = 81;
  float avg_vp_y = 0.0f;
  const int threadhold_vp_y = 5;
  for (size_t i = 0; i < lane_coef.size(); i++) {
    // NOTE: only support 2 lane now
    if (lane_coef[i].size() != 2) {
      std::cout << "not found 2 lane in one image. \n";
      return 1;
    }

    // TODO: can be better here
    if (i + 1 < init_thredhold_count) {
      LaneVanishPoint lane_vanish_point;
      lane_vanish_point.lane_coef = lane_coef[i];
      lane_vanish_point.vanish_point = vanish_point[i];
      lane_vanish_point.pitch_raw = pitch_raw[i];
      init_lane_data.push_back(lane_vanish_point);
      continue;
    } else if (i + 1 == init_thredhold_count) {
      sort(init_lane_data.begin(), init_lane_data.end(), LaneCompare_);
      int i_begin = init_thredhold_count / 4;
      int i_end = init_thredhold_count * 3 / 4;
      for (int i = i_begin; i < i_end; ++i) {
        avg_vp_y += init_lane_data[i].vanish_point.y;
        lane_datas.push_back(init_lane_data[i]);
      }
      avg_vp_y /= (i_end - i_begin);
      init_lane_data.clear();
    }

    if (abs(vanish_point[i].y - avg_vp_y) < threadhold_vp_y) {
      LaneVanishPoint lane_vanish_point;
      lane_vanish_point.lane_coef = lane_coef[i];
      lane_vanish_point.vanish_point = vanish_point[i];
      lane_vanish_point.pitch_raw = pitch_raw[i];
      lane_datas.push_back(lane_vanish_point);
    }
  }

  CameraIntrinsic intrinsic =
      camera_param_->getCameraModelParam().pinhole.intrinsic;
  float pitch, yaw;
  OptimizePitchYaw(lane_datas, camera_height_, intrinsic, pitch, yaw);

  // TODO: why R = Rx * Rz * Ry ?
  Eigen::Matrix3f Rx =
      (Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX())).matrix();
  Eigen::Matrix3f Ry =
      (Eigen::AngleAxisf(-yaw, Eigen::Vector3f::UnitY())).matrix();
  Eigen::Matrix3f Rz =
      (Eigen::AngleAxisf(euler_angle.roll, Eigen::Vector3f::UnitZ())).matrix();
  // NOTE: roll is defind by input
  // Eigen::Matrix3f Rz =
  //     (Eigen::AngleAxisf(0.017, Eigen::Vector3f::UnitZ())).matrix();
  Eigen::Matrix3f R = Rx * Rz * Ry;

  cv::Mat Rcw = (cv::Mat_<float>(3, 3) << R(0, 0), R(0, 1), R(0, 2), R(1, 0),
                 R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));

  std::cout << "Rcw " << std::endl;
  std::cout << Rcw << std::endl;

  std::cout << " result " << std::endl;
  std::cout << " pitch " << pitch * 180 / CV_PI << std::endl;
  std::cout << " yaw " << yaw * 180 / CV_PI << std::endl;
  euler_angle.pitch = pitch;
  euler_angle.yaw = yaw;

  return 0;
}

int FrontCameraExtrinsicCalibUseLane::BirdviewGenerator(
    const CameraRotationEuler &euler_angle, const cv::Mat &image_data,
    cv::Mat &bird_view) {
  if (!is_camera_param_init_) {
    std::cout << "camera param not init \n";
    return 1;
  }

  CameraIntrinsic intrinsic =
      camera_param_->getCameraModelParam().pinhole.intrinsic;
  bird_view =
      CreateBirdView(image_data, euler_angle, intrinsic, camera_height_);
  return 0;
}

} // namespace nullmax_perception
