#pragma once

#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"
#include "frame.h"
#include "keyframedatabase.h"
#include "mappoint.h"
#include "mckeyframe.h"
#include "orbextractor.h"
#include "orbvocabulary.h"
#include <mutex>

namespace FeatureSLAM {

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame {
public:
  KeyFrame(Frame &F, Map *map, KeyFrameDatabase *keyfrm_database);
  void SetPose(const cv::Mat &Tcw);
  cv::Mat GetPose();
  cv::Mat GetPoseInverse();
  cv::Mat GetCameraCenter();
  cv::Mat GetRotation();
  cv::Mat GetTranslation();

  void SetMCConnection(MCKeyFrame *mc_keyfrm);
  MCKeyFrame *GetMCKeyFrame();
  void SetCameraIndex(int cam_index);

  // Bag of Words Representation
  void ComputeBoW();

  // Covisibility graph functions
  void AddConnection(KeyFrame *keyfrm, const int &weight);
  void AddConnectionForce(KeyFrame *keyfrm, const int &weight);
  void EraseConnection(KeyFrame *keyfrm);
  void UpdateConnections();
  void UpdateBestCovisibles();
  std::set<KeyFrame *> GetConnectedKeyFrames();
  std::vector<KeyFrame *> GetVectorCovisibleKeyFrames();
  std::vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int &num_feature_);
  std::vector<KeyFrame *> GetCovisiblesByWeight(const int &w);
  int GetWeight(KeyFrame *keyfrm);

  // Spanning tree functions
  void AddChild(KeyFrame *keyfrm);
  void EraseChild(KeyFrame *keyfrm);
  void ChangeParent(KeyFrame *keyfrm);
  std::set<KeyFrame *> GetChilds();
  KeyFrame *GetParent();
  bool hasChild(KeyFrame *keyfrm);

  // MapPoint observation functions
  void AddMapPoint(MapPoint *mp, const size_t &idx);
  void EraseMapPointMatch(const size_t &idx);
  void EraseMapPointMatch(MapPoint *mp);
  void ReplaceMapPointMatch(const size_t &idx, MapPoint *mp);
  std::set<MapPoint *> GetMapPoints();
  std::vector<MapPoint *> GetMapPointMatches();
  int TrackedMapPoints(const int &min_obs);
  MapPoint *GetMapPoint(const size_t &idx);

  // KeyPoint functions
  std::vector<size_t> GetFeaturesInArea(const float &x, const float &y,
                                        const float &r) const;
  // Image
  bool IsInImage(const float &x, const float &y) const;

  // Enable/Disable bad flag changes
  void SetNotErase();
  void SetErase();

  // Set/check bad flag
  void SetBadFlag();
  bool isBad();

  // Compute Scene Depth (q=2 median).
  float ComputeSceneMedianDepth(const int q);

  static bool weightComp(int a, int b) { return a > b; }

  static bool lId(KeyFrame *keyfrm1, KeyFrame *keyfrm2) {
    return keyfrm1->id_ < keyfrm2->id_;
  }

public:
  int camera_index_;

  static long unsigned int next_id_;
  long unsigned int id_;
  const double timestamp_;
  const int num_cell_cols_;
  const int num_cell_rows_;
  const float cell_element_width_inv_;
  const float cell_element_height_inv_;

  long unsigned int track_reffrm_id_;
  long unsigned int fuse_target_frm_id_;

  long unsigned int mnLoopQuery;
  int mnLoopWords;
  float mLoopScore;
  long unsigned int mnRelocQuery;
  int mnRelocWords;
  float mRelocScore;

  const int num_feature_;
  const std::vector<cv::KeyPoint> keypts_;
  const cv::Mat desc_;

  // BoW
  DBoW2::BowVector bow_vec_;
  DBoW2::FeatureVector featd_vec_;
  // Pose relative to parent (this is computed when bad flag is activated)

  const int scale_levels_;
  const float scale_factor_;
  const float log_scale_factor_;
  const std::vector<float>
      scale_factors_; // 尺度因子，scale^n，scale=1.2，n为层数
  const std::vector<float> level_sigma2_; // 尺度因子的平方
  const std::vector<float> inv_level_sigma2_;
  // Image bounds and calibration
  const int min_x_;
  const int min_y_;
  const int max_x_;
  const int max_y_;
  std::vector<MapPoint *> mappoints_;

protected:
  // SE3 Pose and camera center
  cv::Mat Tcw_;
  cv::Mat Twc;
  cv::Mat Ow;

  KeyFrameDatabase *keyfrm_database_;
  ORBVocabulary *orb_voc_;

  std::vector<std::vector<std::vector<size_t>>> cells_;

  std::map<KeyFrame *, int> connected_keyfrm_weights_;
  std::vector<KeyFrame *> ordered_connected_keyfrms_;
  std::vector<int> ordered_weights_;

  bool first_connection_;
  KeyFrame *parent_keyfrm_;
  std::set<KeyFrame *> children_keyfrms_;

  bool not_erase_;
  bool to_be_erased_;
  bool is_bad_;

  Map *map_;

  MCKeyFrame *mc_keyfrm_;

  std::mutex mutex_pose_;
  std::mutex mutex_connections_;
  std::mutex mutex_features_;
  std::mutex mutex_mc_connections_;
};
}
