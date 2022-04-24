#pragma once

#include "frame.h"
#include "initializer.h"
#include "keyframedatabase.h"
#include "map.h"
#include "mapviewer/mapviewer.h"
#include "mcframe.h"
#include "mckeyframe.h"
#include "multicam_ext.h"
#include "orbextractor.h"
#include "orbvocabulary.h"
#include "timer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <mutex>

namespace FeatureSLAM {

class Map;
class MapViewer;
class MultiCamTracking {

public:
  MultiCamTracking(ORBVocabulary *voc, Map *map,
                   KeyFrameDatabase *keyfrm_database, const string &config_file,
                   MapViewer *pMapviewer);
  cv::Mat GrabImageMultiCam(const cv::Mat &imleft, const cv::Mat &imfront,
                            const cv::Mat &imright, const cv::Mat &imback,
                            double &timestamp);

public:
  enum eTrackingState {
    LOST = 0,
    OK = 1,
  };

  eTrackingState state_;
  eTrackingState last_processed_state_;

  // Current Frame
  Frame current_frame_;
  MCFrame mCurrentMcFrame;
  cv::Mat img_gray_;
  cv::Mat img_gray_left_, img_gray_front_, img_gray_right_, img_gray_back_;
  std::vector<cv::Mat> cam_extrinsic_;

  MultiCam *multicam_;

  std::list<cv::Mat> relative_frm_poses_;
  std::list<KeyFrame *> references_list_;
  int num_matches_inliers_;

protected:
  void Track();
  void SetLost();
  void LoadExtrinsicParam(const cv::FileStorage &settings);
  void CheckReplacedInLastMCFrame();
  bool TrackReferenceKeyFrameMC();
  void UpdateLastMCFrame();
  bool TrackWithMotionModelMC();
  bool Relocalization_panoramic();
  bool RelocalizationMultiCam();
  void UpdateLocalMap();
  void UpdateLocalPoints();
  void UpdateLocalKeyFramesMC();

  void UpdateMCFrame(Frame &f);
  bool TrackLocalMapMC();
  void SearchLocalPointsMC();
  void SearchUnTrackedPoints(std::vector<MapPoint *> &mappoints);
  bool TrackUntrackedPointsSameCam(MapPoint *mp);
  bool TrackUntrackedPoints(MapPoint *mp);

  ORBextractor *extractor_left_, *extractor_front_, *extractor_right_,
      *extractor_back_;
  std::vector<ORBextractor *> extractors_;
  ORBVocabulary *orb_vocabulary_;
  KeyFrameDatabase *keyfrm_database_;
  // Local Map
  KeyFrame *ref_keyfrm_;
  MCKeyFrame *mpReferenceMCKF;
  std::vector<KeyFrame *> local_keyfrms_;
  std::vector<MapPoint *> local_mappoints_;

  MapViewer *map_viewer_;

  // Map
  Map *map_;

  float keyfrm_dist_thr_;
  float mappoint_dist_thr_;

  KeyFrame *last_keyfrm_;
  Frame last_frame_;
  MCFrame last_mc_frame_;
  unsigned int last_keyfrm_id_;
  unsigned int last_reloc_frm_id_;
  cv::Mat velocity_;
};
}
