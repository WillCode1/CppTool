#pragma once
#include "camera_config.h"
#include "colordef.h"
#include "feature/avp_distance_transform.h"
#include "frame.h"
#include "hpa_map.h"
#include "interface/avp_mapping_interface.h"
#include "keyframe.h"
#include "log.h"
#include "optimizer.h"
#include "system_config.h"
#include "trajectory_log.h"
#include "trajectory_smoother.h"
#include "utils.h"
#include "viewerconfig.h"
#include <iostream>
#include <string>
#include <time.h>
#ifdef ENABLE_VIEWER
#include "hpa_viewer.h"
#include "plot_viewer.h"
#endif

namespace SemanticSLAM
{
  class BevTracker
  {
  public:
    BevTracker() = delete;
    BevTracker(const std::string &config, const std::shared_ptr<HpaMap> &hpa_map);

    bool GrabSegImages(uint64_t microsecond, std::vector<unsigned char *> seg_imgs,
                       const WheelOdometry& odom, AVPPose &vehicle_pose);

    void GenerateEnergyMat(std::vector<unsigned char *> imgs);

    ~BevTracker();

    void Track();

    void SetRawImage(cv::InputArray image);

    void PredictPoseByOdom();

    bool NeedKeyFrame();

    void CreateNewKeyFrame();

    void SaveTrajectory(const std::string &filename);

    void SaveOdometry(const std::string &filename);

    void Reset();

    void CreateNewMappoints();

    bool RemapImageRequired();

  public:
    enum class TrackingState
    {
      NOT_INITIALIZED = -1,
      OK = 0,
      LOST = 1,
    };

  private:
    void LoadConfiguration(const std::string &config);
    void Initialize();

  private:
    TrackingState state_;
    std::shared_ptr<TrajectorySmoother> trajectory_smoother_;
    std::shared_ptr<HpaMap> map_;
    double trajectory_length_;
    Frame current_frame_;
    Frame last_frame_;
    KeyFrame *ref_keyfm_;

    // for demo visualization
    cv::Mat image_raw_;
    std::shared_ptr<AvpDistanceTransformer> avp_distance_transformer_;

    std::shared_ptr<TrajectoryLog> odometry_log_;
    std::shared_ptr<TrajectoryLog> trajectory_log_;
    cv::Mat mapping_mask_;  // ./mask/mask.bmp, 车身mask
  };
}
