#pragma once

#include "camera_config.h"
#include <opencv2/opencv.hpp>

namespace SemanticSLAM {
class SystemConfig {
public:
  SystemConfig();
  ~SystemConfig();

  bool Initialization(cv::FileStorage &fsettings);

  static SystemConfig *GetSystemConfig();

  CameraExtrinsic &GetExtrinsicConfig();
  CameraIntrinsic &GetIntrinsicConfig();
  CameraConfig &GetCameraConfig();
  VehicleBodyMask &GetVehicleBodyMask();
  std::pair<int, int> GetImageSize();

public:
  unsigned int map_prob_min_slot_;
  unsigned int map_prob_min_dash_;
  unsigned int map_prob_min_arrow_;
  unsigned int map_prob_min_lane_;
  double map_max_length_;
  bool use_vision_opt_;
  double feature_map_length_;

private:
  VehicleBodyMask vehiclebodymask_;
  CameraConfig camera_config_;
  static SystemConfig *system_config_ptr_;
};
}
