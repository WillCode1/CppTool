#pragma once

#include "map.h"
#include "mappoint.h"
#include "tracking.h"
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <time.h>

namespace FeatureSLAM {
class Tracking;
class Viewer;

class FrameDrawer {
public:
  FrameDrawer(Map *map);
  void Update(Tracking *tracker);
  cv::Mat DrawFrame();

protected:
  void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &im_text);
  cv::Mat image_;
  int num_feature_;
  int num_feature_left_, num_feature_front_, num_feature_right_,
      num_feature_back_;
  vector<cv::KeyPoint> cur_keypts_;
  vector<vector<cv::KeyPoint>> cur_mc_keypts_;
  vector<vector<bool>> map_point_;
  int mnTracked;
  vector<cv::KeyPoint> init_keypts_;
  vector<int> init_matches_;
  int state_;
  Map *map_;
  std::mutex mutex_;
  unsigned long int start_time_sec_;
};

} // namespace ORB_SLAM
