// Have read
#include "system_config.h"
#include "colordef.h"

namespace SemanticSLAM
{
  SystemConfig *SystemConfig::system_config_ptr_ = nullptr;

  SystemConfig::SystemConfig()
      : map_max_length_(0), use_vision_opt_(true), feature_map_length_(30) {}
  SystemConfig::~SystemConfig() {}

  bool SystemConfig::Initialization(cv::FileStorage &fsettings)
  {
    map_prob_min_slot_ = fsettings["map_prob_min_slot"].real();
    map_prob_min_dash_ = fsettings["map_prob_min_dash"].real();
    map_prob_min_lane_ = fsettings["map_prob_min_lane"].real();
    map_prob_min_arrow_ = fsettings["map_prob_min_arrow"].real();

    map_max_length_ = fsettings["map_max_length"].real();

    // if there is no map_max_length is set or map_max_length is invalid, use default setting
    if (map_max_length_ < 1e-10)
    {
      map_max_length_ = 100;
    }

    if (!fsettings["use_vision_opt"].empty())
    {
      if (int(fsettings["use_vision_opt"].real()) == 0)
      {
        use_vision_opt_ = false;
      }
    }

    if (!fsettings["feature_map_length"].empty())
    {
      feature_map_length_ = fsettings["feature_map_length"];
      if (feature_map_length_ < 1e-10)
      {
        std::cout << kColorRed << " Feature map lenght invalid" << kColorReset << std::endl;
        feature_map_length_ = 30.0;
      }
    }

    std::cout << " use vision optimization " << kColorGreen << use_vision_opt_ << kColorReset << std::endl;

    std::cout << " map_prob_min_slot " << kColorGreen << map_prob_min_slot_ << kColorReset << std::endl;
    std::cout << " map_prob_min_dash " << kColorGreen << map_prob_min_dash_ << kColorReset << std::endl;
    std::cout << " map_prob_min_arrow " << kColorGreen << map_prob_min_arrow_ << kColorReset << std::endl;
    std::cout << " map_prob_min_lane " << kColorGreen << map_prob_min_lane_ << kColorReset << std::endl;
    std::cout << " map_max_length : " << kColorGreen << map_max_length_ << kColorReset << std::endl;

    std::cout << " feature_map_length : " << kColorGreen << feature_map_length_ << kColorReset << std::endl;

    // intialize camera config ;
    auto &cam_intrinsic = camera_config_.cam_intrinsic;
    cam_intrinsic.fx = fsettings["focal_length_x"].real();
    cam_intrinsic.fy = fsettings["focal_length_y"].real();
    cam_intrinsic.cx = fsettings["optical_center_x"].real();
    cam_intrinsic.cy = fsettings["optical_center_y"].real();

    cam_intrinsic.image_width = fsettings["image_width"].real();
    cam_intrinsic.image_height = fsettings["image_height"].real();

    auto &cam_extrinsic = camera_config_.cam_extrinsic;

    cam_extrinsic.baselink2cam = fsettings["baselink2center"].real();
    cam_extrinsic.scale = 15.0 / cam_intrinsic.image_width;
    cam_extrinsic.camera_height = 4.0;

    // intialize vehicle body mask
    vehiclebodymask_.top = 125;
    vehiclebodymask_.left = 176;
    vehiclebodymask_.width = (243 - 176);
    vehiclebodymask_.height = (280 - 125);

    return true;
  }

  SystemConfig *SystemConfig::GetSystemConfig()
  {
    if (!system_config_ptr_)
      system_config_ptr_ = new SystemConfig();
    return system_config_ptr_;
  }

  CameraExtrinsic &SystemConfig::GetExtrinsicConfig()
  {
    return camera_config_.cam_extrinsic;
  }

  CameraIntrinsic &SystemConfig::GetIntrinsicConfig()
  {
    return camera_config_.cam_intrinsic;
  }

  CameraConfig &SystemConfig::GetCameraConfig() { return camera_config_; }

  VehicleBodyMask &SystemConfig::GetVehicleBodyMask() { return vehiclebodymask_; }

  std::pair<int, int> SystemConfig::GetImageSize()
  {
    return std::make_pair(camera_config_.cam_intrinsic.image_width,
                          camera_config_.cam_intrinsic.image_height);
  }
}
