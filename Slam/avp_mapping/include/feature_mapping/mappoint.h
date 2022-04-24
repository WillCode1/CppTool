#pragma once

#include "frame.h"
#include "keyframe.h"
#include "map.h"
#include "mcframe.h"
#include <mutex>
#include <opencv2/core/core.hpp>

namespace FeatureSLAM {

class KeyFrame;
class Map;
class Frame;
class MCFrame;

class MapPoint {
public:
  MapPoint(const cv::Mat &pos, int first_mc_keyfrm_id, Map *map);
  MapPoint(const cv::Mat &pos, KeyFrame *ref_keyfrm, Map *map);
  MapPoint(const cv::Mat &pos, Map *map, Frame *frame,
           const int &index_in_frame);

  void SetWorldPos(const cv::Mat &pos);
  void UpdateWorldPos(const cv::Mat &pos);
  cv::Mat GetWorldPos();

  cv::Mat GetNormal();
  KeyFrame *GetReferenceKeyFrame();

  std::map<KeyFrame *, size_t> GetObservations();
  int Observations();

  void AddObservation(KeyFrame *keyfrm, size_t idx);
  void EraseObservation(KeyFrame *pKF);

  int GetIndexInKeyFrame(KeyFrame *pKF);
  bool IsInKeyFrame(KeyFrame *pKF);

  void SetBadFlag();
  bool isBad();

  void Replace(MapPoint *pMP);
  MapPoint *GetReplaced();

  void IncreaseVisible(int n = 1);
  void IncreaseFound(int n = 1);
  float GetFoundRatio();
  inline int GetFound() { return num_found_; }
  void SetReferenceKeyFrame(KeyFrame *ref_keyfrm);
  void ComputeDistinctiveDescriptors();

  cv::Mat GetDescriptor();

  void UpdateNormalAndDepth();

  float GetMinDistanceInvariance();
  float GetMaxDistanceInvariance();
  int PredictScale(const float &currentDist, KeyFrame *pKF);
  int PredictScale(const float &currentDist, Frame *frame);
  void SetDescriptor(const cv::Mat &des);

public:
  long unsigned int id_;
  static long unsigned int next_id_;
  bool is_mature_;
  const long int first_mc_keyfrm_id_;
  int num_obs_;

  // Variables used by the tracking
  float track_proj_x1_;
  float rrack_proj_x2_;
  float track_proj_y1_;
  float track_proj_y2_;
  int track_scale_level1_;
  int track_scale_level2_;
  float track_view_cos1_;
  float track_view_cos2_;

  bool is_tracked_in_view_;
  long unsigned int track_ref_mcfrm_id_;
  long int last_frame_seen_;

  // Variables used by local mapping
  long unsigned int local_ba_keyfrm_id_;
  long unsigned int fuse_candidate_keyfrm_id_;

  int track_cam_index1_;
  int track_cam_index2_;

  //    Seed* seed_;

protected:
  // Position in absolute coordinates
  cv::Mat world_pos_;

  // Keyframes observing the point and associated index in keyframe
  std::map<KeyFrame *, size_t> observations_;
  cv::Mat normal_vector_;
  cv::Mat descriptor_;

  // Reference KeyFrame
  KeyFrame *ref_keyfrm_;

  // Tracking counters
  int num_visible_;
  int num_found_;

  // Bad flag (we do not currently erase MapPoint from memory)
  bool is_bad_;
  MapPoint *replaced_point_;

  // Scale invariance distances
  float min_distance_;
  float max_distance_;

  Map *map_;

  std::mutex mutex_pos_;
  std::mutex mutex_features_;
};
}
