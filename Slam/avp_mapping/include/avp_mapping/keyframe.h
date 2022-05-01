#pragma once

#include "camera_config.h"
#include "frame.h"
#include "semantic_element_def.h"
#include "semantic_point.h"
#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
namespace SemanticSLAM {

class KeyFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KeyFrame();
  ~KeyFrame();

  KeyFrame(const Frame &f, const CameraConfig &camera_config_);
  std::vector<KeyFrame *> GetConnectedKeyframes();

  void InsertConnectKeyframe(KeyFrame *keyframe);

private:
  static int next_id_;
  void GenerateSemanticPoints(cv::InputArray image, SemanticFeature type);

public:
  CameraConfig camera_config_;
  int slot_point_num_;
  int dash_point_num_;
  int lane_point_num_;
  int arrow_point_num_;
  std::vector<SemanticPoint> slot_points_;
  std::vector<SemanticPoint> dash_points_;
  std::vector<SemanticPoint> arrow_points_;
  std::vector<SemanticPoint> lane_points_;

  std::mutex mutex_connected_keyframe_;
  std::set<KeyFrame *> connected_keyframes_;
  int id_;
  Mat33_t trans_world2base_;  // current keyframe pose
};
}
