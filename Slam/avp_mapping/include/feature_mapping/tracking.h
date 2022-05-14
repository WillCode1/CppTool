#pragma once

#include "colordef.h"
#include "frame.h"
#include "viewerconfig.h"

#ifdef ENABLE_VIEWER
#include "viewer/framedrawer.h"
#include "viewer/mapdrawer.h"
#include "viewer/viewer.h"
#endif

#include "initializer.h"
#include "keyframedatabase.h"
#include "local_mapping.h"
#include "map.h"
#include "mcframe.h"
#include "mckeyframe.h"
#include "multicam_ext.h"
#include "orbextractor.h"
#include "orbvocabulary.h"
#include "system.h"
#include "vslam_types.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace FeatureSLAM {

const int kCamNum = 4;
class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class System;
class Tracking {

public:
  Tracking(System *sys, ORBVocabulary *voc, Map *map,
           KeyFrameDatabase *keyfrm_database, const string &setting_file);
  cv::Mat GrabImageMultiCam(const cv::Mat &imleft, const cv::Mat &imfront,
                            const cv::Mat &imright, const cv::Mat &imback,
                            double &timestamp,
                            const SemanticSLAM::WheelOdometry &odometry);

#ifdef ENABLE_VIEWER
  void SetViewer(FrameDrawer *frame_drawer, MapDrawer *map_drawer) {
    frame_drawer_ = frame_drawer;
    map_drawer_ = map_drawer;
  }
  void SetViewer(Viewer *viewer) { viewer_ = viewer; }

#endif

public:
  // Tracking states
  enum eTrackingState {
    SYSTEM_NOT_READY = -1,
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    LOST = 3
  };

  eTrackingState state_;
  eTrackingState last_process_state_;
  Frame *current_frame_;
  Eigen::Vector3d first_odom_;
  MCFrame current_mc_frame_;
  cv::Mat img_gray_;
  cv::Mat img_gray_left_, img_gray_front_, img_gray_right_, img_gray_back_;
  std::vector<cv::Mat>
      cam_extrinsic_; // Extrinsic parameters  (camera extrinsic pose
                      // relative to front camera )

  std::vector<std::vector<int>> init_matches_;  // question: 这三个什么?

  std::vector<std::vector<cv::Point2f>> prematched_points_;
  std::vector<std::vector<cv::Point3f>> init_mappoints_;
  Frame initial_frame_;
  MCFrame initial_mc_frame_;

  list<cv::Mat> mlRelativeFramePoses;
  list<KeyFrame *> mlpReferences;
  list<double> mlFrameTimes;
  list<bool> mlbLost;
  void Reset();
  void SetSLAMMode(const bool &flag);

protected:
  void Track();
  void LoadExtrinsicParam(const cv::FileStorage &fSettings);

  // Map initialization for monocular
  void Sysinitialization();
  void CreateInitialMap();

  void CheckReplacedInLastMCFrame();
  bool TrackReferenceKeyFrame();
  void UpdateLastFrame();
  bool TrackWithMotionModel();

  void UpdateLocalMap();
  void UpdateLocalPoints();
  void UpdateLocalKeyFrames();

  bool TrackLocalMap();
  void SearchLocalPoints();
  bool TrackUntrackedPointsSameCam(MapPoint *mp);
  bool TrackUntrackedPoints(MapPoint *pMP);

  bool NeedNewKeyFrame();
  void CreateNewKeyFrame();

  // Other Thread Pointers
  LocalMapping *local_mapper_;

  std::set<double> mPreviousKFTimestamp;

  // ORB
  ORBextractor *orb_extractor_left_, *orb_extractor_front_,
      *orb_extractor_right_, *orb_extractor_back_;
  ORBextractor *init_orb_extrator_;   // question: 初始化成功前, 先多提取特征点
  std::vector<ORBextractor *> extractors_;

  // BoW
  ORBVocabulary *orb_voc_;
  KeyFrameDatabase *mpKeyFrameDB;

  Initializer *initialier_;

  KeyFrame *mpReferenceKF;
  MCKeyFrame *reference_mc_keyframe_ptr_;
  std::vector<KeyFrame *> local_keyframes_;
  std::vector<MapPoint *> local_mappoints_;

  // System
  System *system_;

#ifdef ENABLE_VIEWER
  // Drawers
  Viewer *viewer_;
  FrameDrawer *frame_drawer_;
  MapDrawer *map_drawer_;
#endif
  // Map
  Map *map_;
  int max_frame_thr_;       // camera hz
  int num_matched_inliers_;
  int num_matched_inliers_total_;
  KeyFrame *last_keyfrm_;
  MCFrame last_mc_frame_;
  unsigned int last_keyfrm_id_;
  unsigned int last_reloc_frm_id_;
  cv::Mat velocity_;
  bool slam_mode_;    // question: 定位/建图
  MultiCam *multi_cam_;
  list<MapPoint *> mlpTemporalPoints;
};
}
