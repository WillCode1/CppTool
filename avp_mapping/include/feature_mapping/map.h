#pragma once

#include "keyframe.h"
#include "mappoint.h"
#include "multicam_ext.h"
#include "orbvocabulary.h"

#include <mutex>
#include <set>
#include <string>

namespace FeatureSLAM {

class MapPoint;
class KeyFrame;
class MCKeyFrame;
class ORBextractor;

class Map {
public:
  Map();

  void AddKeyFrame(KeyFrame *keyfrm);
  void AddMapPoint(MapPoint *mp);
  void EraseMapPoint(MapPoint *mp);
  void EraseKeyFrame(KeyFrame *keyfrm);
  void SetReferenceMapPoints(const std::vector<MapPoint *> &mps);

  std::vector<KeyFrame *> GetAllKeyFrames();
  std::vector<MCKeyFrame *> GetAllMCKeyFrames();
  std::vector<MapPoint *> GetAllMapPoints();
  std::vector<MapPoint *> GetReferenceMapPoints();

  long unsigned int MapPointsInMap();
  long unsigned int SematicFeaturesInMap();
  long unsigned KeyFramesInMap();

  void clear();
  bool Save(const std::string &filename, const bool save_bow = false);
  bool Load(const std::string &filename, ORBVocabulary &voc,
            const std::string mask_path, std::vector<cv::Mat> cam_extrinsics);

  void MoveMap(cv::Mat T);

  void AlignToOdometry();

  void SaveOdometry(const string &filename);

  void SaveKeyFrameOdom(const std::string &filename);

  void SetBaKeyFrame(const std::vector<MCKeyFrame *> &local_mc_keyfrms,
                     const std::vector<MCKeyFrame *> &fixed_mc_keyfrms);

  std::vector<KeyFrame *> GetBAKeyFramesFixed();
  std::vector<KeyFrame *> GetBAKeyFrames();

public:
  vector<KeyFrame *> keyfrm_origins_;
  std::mutex mutex_map_update_;
  std::mutex mutex_point_creation_;

protected:
  void WriteMapPoint(ofstream &f, MapPoint *mp);
  void WriteKeyFrame(ofstream &f, KeyFrame *kf,
                     map<MapPoint *, unsigned long int> &idx_of_mp);

  void WriteKeyFrameBow(ofstream &f, KeyFrame *kf,
                        map<MapPoint *, unsigned long int> &idx_of_mp);

  MapPoint *ReadMapPoint(ifstream &f);
  KeyFrame *ReadKeyFrame(ifstream &f, ORBVocabulary &voc,
                         map<unsigned long int, MapPoint *> mp_of_idx,
                         ORBextractor *ex);

protected:
  std::set<MapPoint *> map_points_; ///< MapPoints
  std::set<KeyFrame *> keyframes_;  ///< Keyframs
  std::vector<MCKeyFrame *> ba_local_mckeyfrms_;
  std::vector<MCKeyFrame *> ba_fixed_mc_keyfrms_;
  // only for visualization
  std::vector<MapPoint *> reference_mappoints_;
  long unsigned int max_keyfrm_id_;
  std::mutex mutex_map_;
  std::mutex mutex_ba_keyfrm_;
};
}
