#ifndef MAPVIEWER_H_
#define MAPVIEWER_H_

#include "map.h"
#include "mappoint.h"
#include "multicamtracking.h"
#include "simplecarmodel.h"
#include <memory>
#include <pangolin/pangolin.h>

// Version 2 2018 9 3
namespace FeatureSLAM {

class MultiCamTracking;
class MapViewer {
public:
  MapViewer(Map *map, std::vector<cv::Mat> cam_extrinsic);

  void Run();
  void UpdateTrackPoint(Frame framefront, Frame frameback);
  void Update(MultiCamTracking *mtracker);

  void RequestFinish();
  bool IsFinish();

private:
  void DrawWholeMap();
  void DrawTrackedMapPoints();
  void DrawRays();
  void DrawCurrentCam();
  void DrawFrame();
  void DrawGrid();
  void DrawGraph();

  bool CheckFinish();
  void SetFinish();

private:
  Map *map_;
  std::mutex mutex_tracked_points_;
  int tracked_inliers_;
  std::vector<std::vector<MapPoint *>> tracked_mappoints_;
  cv::Mat Tcw_;
  std::vector<cv::Mat> cam_extrinsic_;
  cv::Mat front2wheel_;
  std::vector<cv::Mat> cam_center_;
  std::vector<std::vector<bool>> frame_mappoints_;
  std::vector<std::vector<cv::KeyPoint>> cur_mcfrm_keypts_;
  bool state_;

  std::vector<Eigen::Vector3d> map_points_;
  std::shared_ptr<SimpleCarModel> simple_car_model_;

  Eigen::MatrixXf ray_color_;
  cv::Mat im;
  bool show_frame_;

  std::mutex mutex_finishe_;
  bool finish_request_;
  bool is_finish_;
};
}

#endif
