#pragma once

#include "avp_mapping_interface.h"
#include "box.h"
#include "colordef.h"
#include "hpa_map.h"
#include "quadtree.h"
#include "semantic_element_def.h"
#include "simplecarmodel.h"
#include "utils.h"
#include <Eigen/Dense>
#include <mutex>
#include <pangolin/pangolin.h>
#include <thread>
#include <time.h>

class HpaViewer {
public:
  HpaViewer(int viewer_width, int viewer_height);
  HpaViewer() = delete;
  ~HpaViewer();

  HpaViewer(const HpaViewer &) = delete;
  HpaViewer(const HpaViewer &&) = delete;
  HpaViewer &operator=(const HpaViewer &) = delete;
  HpaViewer &operator=(const HpaViewer &&) = delete;

  void Run();
  static HpaViewer &GetInstance(int width = 416, int height = 416);
  void SetCurrentPose(const Eigen::Matrix3d &trans_world2base);
  void AddKeyframePose(const Eigen::Matrix3d &kf_world2base);
  void Reset();
  void UpdateVideo(cv::InputArray video_frame);
  void UpdateProjection(cv::InputArray projection_frame);
  void RequestFinish();
  bool IsFinish();
  void AddMarker();

  bool
  UpdateSemanticMap(const std::vector<SemanticSLAM::PointTyped> &semantic_map);

private:
  bool CheckFinish();
  void SetFinish();
  void DrawGrid();
  void DrawAxis();
  void DrawMap();
  void DrawQuadtree();
  void DrawKeyframes();
  void DrawMarkerPoints();
  pangolin::OpenGlMatrix GetCurrentCamMatrix();
  void DrawCurrentPose();

private:
  static std::mutex global_mutex_;
  //  2d visualization

  std::mutex mutex_texture_;
  bool video_img_changed_;
  bool projection_changed_;
  int video_width_;
  int video_height_;
  unsigned char *video_data_;
  unsigned char *projection_data_;
  std::mutex mutex_pose_;
  Eigen::Vector3d current_pose_;

  std::mutex mutex_keyframe_;
  std::vector<Eigen::Matrix3d> keyframe_poses_;
  std::shared_ptr<SimpleCarModel> simple_car_model_;
  std::mutex mutex_map_;
  std::vector<SemanticSLAM::PointTyped> semantic_map_;
  std::mutex mutex_marker_;
  std::vector<Eigen::Vector2d> marker_points_;
  std::mutex mutex_finishe_;
  bool finish_request_;
  bool is_finish_;
};
