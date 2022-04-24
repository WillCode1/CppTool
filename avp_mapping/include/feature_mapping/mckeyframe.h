#pragma once

#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"
#include "frame.h"
#include "keyframe.h"
#include "keyframedatabase.h"
#include "mappoint.h"
#include "mcframe.h"
#include "multicam_ext.h"
#include "orbextractor.h"
#include "orbvocabulary.h"
#include <mutex>

namespace FeatureSLAM {

class Map;
class MapPoint;
class Frame;
class KeyFrame;
class KeyFrameDatabase;
class MCFrame;

class MCKeyFrame {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MCKeyFrame(KeyFrame *pKFLeft, KeyFrame *pKFFront, KeyFrame *pKFRight,
             KeyFrame *pKFBack, MultiCam *pMulticam);

  MCKeyFrame(MCFrame *mc_frame, Map *map, KeyFrameDatabase *keyframe_database,
             bool scale_confirmed);
  void SetPose(const cv::Mat &Tcw);
  void SetKeyFrame(KeyFrame *pKF, int Camera_index);
  bool GetState();
  void SetState(bool state);
  void UpdateScale(const float scale);
  cv::Mat GetPose();
  int TrackedMapPoints(const int &minObs, std::vector<int> &vnTrackedMapPoints);
  Eigen::Vector3d GetOdometry();
  void SetOdometry(const Eigen::Vector3d &odometry_vector);

  std::vector<MCKeyFrame *> GetVectorCovisibleKeyFrames();

  void ComputeBoW();

  static bool lId(MCKeyFrame *mc_keyfrm1, MCKeyFrame *mc_keyfrm2) {
    return mc_keyfrm1->id_ < mc_keyfrm2->id_;
  }

public:
  static long unsigned int next_id_;
  long unsigned int id_;
  long unsigned int frame_id_;
  double timestamp_;

  long unsigned int local_ba_for_keyfrm_;
  long unsigned int ba_fixed_for_keyfrm_;
  std::vector<KeyFrame *> keyframes_;
  std::vector<cv::Mat> tcws_;
  bool mc_ready_;
  std::vector<cv::Mat> cam_exts_;
  MultiCam *multicam_;
  Eigen::Vector3d odometry_vector_;

protected:
  std::mutex mutex_scale_state_;
  cv::Mat tcw_;
  std::mutex mutex_state_;
  std::mutex mutex_pose_;
};
}
