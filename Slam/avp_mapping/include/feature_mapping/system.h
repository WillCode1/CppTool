#pragma once

#include <opencv2/core/core.hpp>
#include <string>
#include <thread>

#include "basictool.h"
#include "viewerconfig.h"

#ifdef ENABLE_VIEWER
#include "viewer/framedrawer.h"
#include "viewer/mapdrawer.h"
#include "viewer/viewer.h"
#include <pangolin/pangolin.h>
#endif

#include "avp_mapping_interface.h"
#include "keyframedatabase.h"
#include "local_mapping.h"
#include "map.h"
#include "message_filter/message_filter.h"
#include "orbvocabulary.h"
#include "tracking.h"
#include "vslam_types.h"

namespace FeatureSLAM {

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;

class System final : public FeatureMapper {
public:
public:
  System(const string &setting_file);
  System(const std::string &map_file, const string &voc_file,
         const string &setting_file, const bool use_viewer = true);
  virtual cv::Mat TrackMultiCam(const cv::Mat &imleft, const cv::Mat &imfront,
                                const cv::Mat &imright, const cv::Mat &imback,
                                uint64_t timestamp) override;

  virtual cv::Mat
  TrackMultiCam(const cv::Mat &imleft, const cv::Mat &imfront,
                const cv::Mat &imright, const cv::Mat &imback,
                const SemanticSLAM::WheelOdometry &odometry) override;

  virtual void
  InsertOdometry(uint64_t timestamp,
                 const SemanticSLAM::WheelOdometry &odometry) override;

  virtual void Shutdown() override;
  virtual bool SaveMap(const std::string &filename,
                       bool with_bow = false) override;

  virtual void AlignMaptoTrajectory(const std::string &file_name) override;

private:
  ORBVocabulary *vocabulary_;
  KeyFrameDatabase *keyfrm_database_;
  Map *map_;
  Tracking *tracker_;
  std::shared_ptr<MessageFilter> message_filter_;

#ifdef ENABLE_VIEWER
  Viewer *viewer_;
  FrameDrawer *frame_drawer_;
  MapDrawer *map_drawer_;
  std::thread *viewer_thread_;
#endif
};
}
