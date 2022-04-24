#pragma once

#include "framedrawer.h"
#include "mapdrawer.h"
#include "multicam_ext.h"
#include "system.h"
#include "tracking.h"
#include "vslam_types.h"
#include <mutex>

namespace FeatureSLAM {

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

class Viewer {
public:
  Viewer(System *system, FrameDrawer *frame_drawer, MapDrawer *map_drawer,
         Tracking *tracker, const string &setting_file);
  void Run();
  void RequestFinish();
  void RequestStop();
  bool isFinished();
  bool isStopped();
  void Release();

private:
  bool Stop();
  bool CheckFinish();
  void SetFinish();

private:
  // camera extrinsic parameters
  std::vector<cv::Mat> cam_ext_;
  System *system_;
  FrameDrawer *frame_drawer_;
  MapDrawer *map_drawer_;
  Tracking *tracker_;
  float image_width_;
  float img_height_;
  Vec3_t viewpoint_;
  float viewpoint_focal_length_;
  bool finish_requested_;
  bool is_finished_;
  std::mutex mutex_finish_;
  std::mutex mutex_multicam;
  bool is_stopped_;
  bool stop_requested_;
  std::mutex mutex_stop_;
};
}
