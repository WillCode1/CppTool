// Have read
#include "bevtracker.h"
#include "timer.h"

namespace SemanticSLAM
{
  BevTracker::BevTracker(const std::string &config, const std::shared_ptr<HpaMap> &hpa_map)
      : state_(TrackingState::NOT_INITIALIZED), map_(hpa_map), trajectory_length_(0), ref_keyfm_(nullptr)
  {
    LoadConfiguration(config);

    // image_resolution   width x height
    auto image_resolution = SystemConfig::GetSystemConfig()->GetImageSize();
    auto image_width = image_resolution.first;
    auto image_height = image_resolution.second;
    avp_distance_transformer_ = std::make_shared<AvpDistanceTransformer>(image_height, image_width);

#ifdef ENABLE_VIEWER
    std::shared_ptr<std::thread> viewer_thread = std::make_shared<std::thread>(&HpaViewer::Run, &HpaViewer::GetInstance(image_resolution.first, image_resolution.second));
    viewer_thread->detach();
    PlotViewer::GetInstance().SetLabel("distance-transform");
    PlotViewer::GetInstance().SetLabel("optimization");
    PlotViewer::GetInstance().SetLabel("create-map-points");
    PlotViewer::GetInstance().SetLabel("update semantic points");
    PlotViewer::GetInstance().SetLabel("total");
    PlotViewer::GetInstance().SetTitle("Time consuming (ms) ");

    std::shared_ptr<std::thread> plot_viewer_thread = std::make_shared<std::thread>(&PlotViewer::Run, &PlotViewer::GetInstance());
    plot_viewer_thread->detach();
#endif

    trajectory_smoother_ = std::make_shared<TrajectorySmoother>(1, 1, 0.1);

    // track log
    odometry_log_ = std::make_shared<TrajectoryLog>("", TrajectoryLog::Format::NULLMAX);
    trajectory_log_ = std::make_shared<TrajectoryLog>("", TrajectoryLog::Format::NULLMAX);
  }

  bool BevTracker::GrabSegImages(uint64_t microsecond, std::vector<unsigned char *> seg_imgs, const WheelOdometry& odom, AVPPose &vehicle_pose)
  {
    // generate energy mat
    current_frame_ = Frame(odom, current_frame_);

    GenerateEnergyMat(seg_imgs);
    Track();

    last_frame_ = Frame(current_frame_);
    vehicle_pose = Utils::Matrix2AvpPose(current_frame_.trans_world2base_);

    odometry_log_->ExportPose(microsecond, Utils::WheelOdom2Matrix(odom));
    trajectory_log_->ExportPose(microsecond, Utils::Matrix3d2Matrix4d(current_frame_.trans_world2base_));

#ifdef ENABLE_VIEWER
    HpaViewer::GetInstance().UpdateVideo(image_raw_);
#endif

    // TO DO, auto check mapping state
    return true;
  }

  BevTracker::~BevTracker()
  {
#ifdef ENABLE_VIEWER
    //  Shutdown viewer
    PlotViewer::GetInstance().RequestFinish();
    while (!PlotViewer::GetInstance().IsFinish())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    HpaViewer::GetInstance().RequestFinish();
    while (!HpaViewer::GetInstance().IsFinish())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
#endif
  }

  void BevTracker::Track()
  {
    if (state_ == TrackingState::NOT_INITIALIZED)
    {
      Initialize();
    }
    else
    {
      // predict current  pose by odometry
      auto camera_config = SystemConfig::GetSystemConfig()->GetCameraConfig();
      PredictPoseByOdom();

      // Use visual data to optimize current pose
      if (SystemConfig::GetSystemConfig()->use_vision_opt_)
      {
        Optimizer::GetInstance().OptimizeFramePose(ref_keyfm_, current_frame_, camera_config);
      }
    }

    trajectory_smoother_->Smoother(&current_frame_);

    if (NeedKeyFrame())
    {
      CreateNewKeyFrame();
      CreateNewMappoints();
    }

#ifdef ENABLE_VIEWER
    HpaViewer::GetInstance().SetCurrentPose(current_frame_.trans_world2base_);
#endif
  }

  void BevTracker::SetRawImage(cv::InputArray image)
  {
    image_raw_ = image.getMat();
  }

  void BevTracker::PredictPoseByOdom()
  {
    Mat33_t last_odom = Utils::Se2Vector2Matrix(last_frame_.odometry_);
    Mat33_t current_odom = Utils::Se2Vector2Matrix(current_frame_.odometry_);

    Mat33_t odom_increment = last_odom.inverse() * current_odom;
    current_frame_.trans_world2base_ = last_frame_.trans_world2base_ * odom_increment;

    double trajectory_increment = hypot(odom_increment(0, 2), odom_increment(1, 2));
    trajectory_length_ += trajectory_increment;
  }

  bool BevTracker::NeedKeyFrame()
  {
    if (trajectory_length_ > SystemConfig::GetSystemConfig()->map_max_length_)
    {
      std::cout << kColorGreen;
      NM_INFO("EXCEED MAX TRAJECTORY LENGTH, Save Map Automatically");
      std::cout << kColorReset << std::endl;
      // Exceed Max Map Length

      const std::string kMapName = Utils::GetCurrentTime() + "-map.bin";
      map_->SaveMap(kMapName);
      return false;
    }

    // condition 1
    // if there is no reference keyframe
    if (ref_keyfm_ == nullptr)
      return true;

    Mat33_t trans_world2base = current_frame_.trans_world2base_;
    std::vector<KeyFrame *> connected_keyframes = ref_keyfm_->GetConnectedKeyframes();

    // find closest keyframe
    KeyFrame *closest_keyframe = nullptr;
    double min_distance = 1e10;
    for (auto kf : connected_keyframes)
    {
      auto keyframe_pose = kf->trans_world2base_;
      double distance = hypot(keyframe_pose(0, 2) - trans_world2base(0, 2),
                              keyframe_pose(1, 2) - trans_world2base(1, 2));
      if (distance < min_distance)
      {
        min_distance = distance;
        closest_keyframe = kf;
      }
    }

    // condition 2
    // the the distance of closet keyframe is larger than the threshold
    if (min_distance < 3.5 && ref_keyfm_ != closest_keyframe)
    {
      ref_keyfm_ = closest_keyframe;
      return false;
    }

    // condition 3
    // the output of perception is not enough
    if (current_frame_.edge_size_ < 100)
      return true;

    double dist = hypot(current_frame_.trans_world2base_(0, 2) - ref_keyfm_->trans_world2base_(0, 2),
                        current_frame_.trans_world2base_(1, 2) - ref_keyfm_->trans_world2base_(1, 2));

    const double kDistanceThreshold = 2.0;
    if (dist > kDistanceThreshold)
    {
      return true;
    }

    return false;
  }

  std::vector<Vec3_t> GetSemanticPoints(const CameraConfig &camera_config, cv::InputArray image, cv::InputArray mapping_mask,
                                        const Mat33_t &trans_world2base, unsigned int measurement_threshold)
  {
    cv::Mat mask = mapping_mask.getMat();

    double cx = camera_config.cam_intrinsic.cx;
    double cy = camera_config.cam_intrinsic.cy;

    double baselink2cam = camera_config.cam_extrinsic.baselink2cam;
    double scale = camera_config.cam_extrinsic.scale;

    std::vector<Vec3_t> world_points;
    cv::Mat img = image.getMat();

    for (int i = 0; i < img.rows; i++)
    {
      const unsigned char *pt = img.ptr<uchar>(i);

      for (int j = 0; j < img.cols; j++)
      {
        unsigned char measurement = pt[j];
        if (measurement == 0)
          continue;
        // for visualization
        if (measurement < measurement_threshold)
          continue;
        double x = (i - cy) * (-scale) + baselink2cam;
        double y = (j - cx) * (-scale);

        // to do more automated
        if (mask.empty())
        {
          if (i < 300 && i > 100)
          {
            Vec3_t p(x, y, 1);
            Vec3_t pw = trans_world2base * p;
            world_points.push_back(Vec3_t(pw.x(), pw.y(), measurement));
          }
        }
        else
        {
          if (mask.ptr(i)[j] > 254)
          {
            Vec3_t p(x, y, 1);
            Vec3_t pw = trans_world2base * p;
            world_points.push_back(Vec3_t(pw.x(), pw.y(), measurement));
          }
        }
      }
    }
    return world_points;
  }

  void BevTracker::CreateNewKeyFrame()
  {
    auto camera_config = SystemConfig::GetSystemConfig()->GetCameraConfig();
    KeyFrame *previous_keyframe = ref_keyfm_;

    Timer timer("create new keyfrm");
    ref_keyfm_ = new KeyFrame(current_frame_, camera_config);
    timer.Print();

    map_->InsertKeyframe(ref_keyfm_);

    // update keyframe connections
    if (previous_keyframe != nullptr)
    {
      previous_keyframe->InsertConnectKeyframe(ref_keyfm_);
      ref_keyfm_->InsertConnectKeyframe(previous_keyframe);
    }
  }

  void BevTracker::SaveTrajectory(const std::string &filename)
  {
    trajectory_log_->SaveLog(filename);
  }

  void BevTracker::SaveOdometry(const std::string &filename)
  {
    odometry_log_->SaveLog(filename);
  }

  void BevTracker::Reset()
  {
    ref_keyfm_ = nullptr;
    current_frame_.Reset();
    last_frame_.Reset();
    map_->Reset();
#ifdef ENABLE_VIEWER
    HpaViewer::GetInstance().Reset();
#endif
    state_ = TrackingState::NOT_INITIALIZED;
    trajectory_length_ = 0.0;
    trajectory_smoother_->Reset();
  }

  void BevTracker::CreateNewMappoints()
  {
    Timer timer("create-map-points");
    auto camera_config = SystemConfig::GetSystemConfig()->GetCameraConfig();
    auto trans_world2base = current_frame_.trans_world2base_;

    const unsigned int map_prob_min_slot = SystemConfig::GetSystemConfig()->map_prob_min_slot_;
    const unsigned int map_prob_min_dash = SystemConfig::GetSystemConfig()->map_prob_min_dash_;
    const unsigned int map_prob_min_arrow = SystemConfig::GetSystemConfig()->map_prob_min_arrow_;
    const unsigned int map_prob_min_lane = SystemConfig::GetSystemConfig()->map_prob_min_lane_;

    std::vector<Vec3_t> slot_points = GetSemanticPoints(camera_config, current_frame_.image_slot_, mapping_mask_, trans_world2base, map_prob_min_slot);
    std::vector<Vec3_t> dash_points = GetSemanticPoints(camera_config, current_frame_.image_dash_, mapping_mask_, trans_world2base, map_prob_min_dash);
    std::vector<Vec3_t> arrow_points = GetSemanticPoints(camera_config, current_frame_.image_arrow_, mapping_mask_, trans_world2base, map_prob_min_arrow);
    std::vector<Vec3_t> lane_points = GetSemanticPoints(camera_config, current_frame_.image_lane_, mapping_mask_, trans_world2base, map_prob_min_lane);

    map_->AddSlotPoints(slot_points);
    map_->AddDashPoints(dash_points);
    map_->AddArrowPoint(arrow_points);
    map_->AddLanePoint(lane_points);

#ifdef ENABLE_VIEWER
    HpaViewer::GetInstance().AddKeyframePose(current_frame_.trans_world2base_);
    timer.Print();
    PlotViewer::GetInstance().UpdateData("create-map-points", timer.GetTimeConsuming());
#endif
  }

  bool BevTracker::RemapImageRequired()
  {
    return trajectory_length_ < SystemConfig::GetSystemConfig()->feature_map_length_;
  }

  void BevTracker::LoadConfiguration(const std::string &config)
  {
    cv::FileStorage fsettings(config, cv::FileStorage::READ);
    if (!fsettings.isOpened())
    {
      std::cout << " open config file " << config << " failed  " << std::endl;
      exit(0);
    }

    const size_t bar_index = config.find_last_of('/');
    std::string str_data_path = config.substr(0, bar_index + 1);

    const std::string str_mask = fsettings["mask"];
    mapping_mask_ = cv::imread(str_data_path + str_mask, cv::IMREAD_GRAYSCALE);

    if (mapping_mask_.empty())
    {
      std::cout << " fatal error mask is empty " << std::endl;
      exit(0);
    }
    // load sysytem config
    SystemConfig::GetSystemConfig()->Initialization(fsettings);
    fsettings.release();
  }

  void BevTracker::Initialize()
  {
    // Fix the first frame to the origin;
    current_frame_.trans_world2base_ = Mat33_t::Identity();
    CreateNewKeyFrame();
    CreateNewMappoints();
    state_ = TrackingState::OK;
  }

  void BevTracker::GenerateEnergyMat(std::vector<unsigned char *> imgs)
  {
    // segmentation images : parking slot, dash, arrow, lane.
    if (imgs.size() != 4)
    {
      std::cout << kColorRed << " Input Segementation Image Size != 4" << kColorReset << std::endl;
      NM_ERROR(" Input Segementation Image Size != 4 ");
      return;
    }

    // 类似梯度矩阵
    Timer timer("distance_tranform");
    avp_distance_transformer_->DistranceTransform(imgs[0], current_frame_.image_slot_.data);
    avp_distance_transformer_->DistranceTransform(imgs[1], current_frame_.image_dash_.data);
    avp_distance_transformer_->DistranceTransform(imgs[2], current_frame_.image_arrow_.data);
    avp_distance_transformer_->DistranceTransform(imgs[3], current_frame_.image_lane_.data);

    timer.Print();

#ifdef ENABLE_VIEWER
    PlotViewer::GetInstance().UpdateData("distance-transform", timer.GetTimeConsuming());
#endif
  }
}
