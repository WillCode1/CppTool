#pragma once

#include "colordef.h"
#include "keyframe.h"
#include "keyframedatabase.h"
#include "map.h"
#include "mckeyframe.h"
#include "tracking.h"
#include "triangulator.h"
#include <mutex>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

namespace FeatureSLAM {

class Map;
class MCKeyFrame;

class LocalMapping {
public:
  LocalMapping(Map *map, KeyFrameDatabase *keyfrm_databse);
  // Main function
  void Run();
  void InsertMCKeyFrame(MCKeyFrame *mc_keyfrm);
  // Thread Synch
  void RequestStop();
  void RequestReset();
  bool Stop();
  void Release();
  bool isStopped();
  bool stopRequested();
  bool AcceptKeyFrames();
  void SetKeyframeAcceptability(bool flag);
  bool SetNotStop(bool flag);
  void InterruptBA();
  void RequestFinish();
  bool isFinished();
  int McKeyframesInQueue();
  bool CheckNewKeyFrames();

  void SpinOnce();

  void EnableStructBa();

protected:
  void ProcessNewKeyFrame();
  void CreateNewMapPoints();
  int CreateNewMapPoints(KeyFrame *cur_keyfrm);
  void MapPointCulling();

  void SearchInNeighbors();
  void SearchFrameInNeighbors(KeyFrame *cur_keyfrm);
  void KeyFrameCulling();
  cv::Mat CreateE12(KeyFrame *&pKF1, KeyFrame *&pKF2);
  cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

  void ResetIfRequested();
  bool reset_requested_;
  std::mutex mutex_reset_;

  bool CheckTerminate();
  void SetTerminate();
  bool terminate_requested_;
  bool is_terminated_;
  std::mutex mutex_finish_;
  Map *map_;
  bool use_struct_ba_;

  KeyFrameDatabase *keyframe_database_;
  // Keyframes need to be processed
  std::list<MCKeyFrame *> keyfrms_queue_;

  KeyFrame *cur_keyfrm_;

  std::list<MapPoint *> recent_added_mappoints_;
  std::mutex mutex_new_keyfrm_;
  bool abort_ba_;
  bool stopped_;
  bool stop_requested_;
  bool not_stop_;
  std::mutex mutex_stop;
  bool accept_keyfrms_;
  std::mutex mutex_accept_;
};
}
