#pragma once

#include "keyframe.h"
#include "map.h"
#include "mappoint.h"
#include "multicam_ext.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace FeatureSLAM {

class MapDrawer {
public:
  MapDrawer(Map *pMap, const string &strSettingPath);

  void DrawMapPoints();
  void DrawKeyFrames(const bool draw_keyfrm, const bool draw_graph);
  void DrawOdometry();
  void DrawAxis();
  void DrawCurrentCamera(cv::Mat trans_w2frm,
                         const std::vector<cv::Mat> &cam_ext);
  void SetCurrentCameraPose(const cv::Mat &Tcw);
  void SetReferenceKeyFrame(KeyFrame *pKF);
  void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &m, cv::Mat &Twc);
  pangolin::OpenGlMatrix ToGLMatrix(cv::Mat T);

private:
  Map *map_;
  float keyfrm_size_;
  float keyfrm_line_width_;
  float graph_line_width_;
  float point_size_;
  float cam_size_;
  float cam_line_width_;

  cv::Mat cam_pose_;

  std::mutex mutex_cam_;
};

} // namespace ORB_SLAM
