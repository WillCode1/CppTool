#include "keyframe.h"
#include "system_config.h"

namespace SemanticSLAM
{
  int KeyFrame::next_id_ = 0;
  KeyFrame::KeyFrame() : slot_point_num_(0) { id_ = -1; }

  KeyFrame::~KeyFrame() {}

  KeyFrame::KeyFrame(const Frame &f, const CameraConfig &camera_config)
      : camera_config_(camera_config), trans_world2base_(f.trans_world2base_)
  {
    id_ = next_id_++;

    // Generate semantic point for localization  ;
    GenerateSemanticPoints(f.image_slot_, PARKING_SLOT);
    GenerateSemanticPoints(f.image_dash_, DASH);
    //  GenerateSemanticPoints( f.image_arrow_,ARROW );
    GenerateSemanticPoints(f.image_lane_, LANE);
  }

  std::vector<KeyFrame *> KeyFrame::GetConnectedKeyframes()
  {
    std::lock_guard<std::mutex> lock(mutex_connected_keyframe_);
    std::vector<KeyFrame *> connected_keyframes(connected_keyframes_.begin(), connected_keyframes_.end());
    return connected_keyframes;
  }

  void KeyFrame::InsertConnectKeyframe(KeyFrame *keyframe)
  {
    std::lock_guard<std::mutex> lock(mutex_connected_keyframe_);

    if (keyframe == this)
      return;
    if (connected_keyframes_.count(keyframe) == 0)
      connected_keyframes_.insert(keyframe);
  }

  void KeyFrame::GenerateSemanticPoints(cv::InputArray image, SemanticFeature type)
  {
    const auto camera_config = SystemConfig::GetSystemConfig()->GetCameraConfig();

    int image_width = camera_config.cam_intrinsic.image_width;
    int image_height = camera_config.cam_intrinsic.image_height;

    cv::Mat prob_image = image.getMat();
    double scale = camera_config_.cam_extrinsic.scale;
    double baselink2cam = camera_config_.cam_extrinsic.baselink2cam;

    //  Too Sparse, need better strategy ????

    std::vector<SemanticPoint> smt_points;

    for (int i = 0; i < prob_image.rows; i = i + 3)
    {
      const unsigned char *pt = prob_image.ptr<uchar>(i);
      for (int j = 0; j < prob_image.cols; j = j + 3)
      {
        uchar measurement = pt[j];
        if (measurement == 0)
          continue;
        double x = (i - image_height / 2) * (-scale) + baselink2cam;
        double y = (j - image_width / 2) * (-scale);
        SemanticPoint sp(j, i, x, y, static_cast<float>(measurement));

        smt_points.push_back(sp);
      }
    }
    if (type == PARKING_SLOT)
    {
      slot_points_ = smt_points;
      slot_point_num_ = static_cast<int>(slot_points_.size());
    }
    else if (type == DASH)
    {
      dash_points_ = smt_points;
      dash_point_num_ = static_cast<int>(dash_points_.size());
    }
    else if (type == LANE)
    {
      lane_points_ = smt_points;
      lane_point_num_ = static_cast<int>(lane_points_.size());
    }
    else if (type == ARROW)
    {
      arrow_points_ = smt_points;
      arrow_point_num_ = static_cast<int>(arrow_points_.size());
    }
  }
}
