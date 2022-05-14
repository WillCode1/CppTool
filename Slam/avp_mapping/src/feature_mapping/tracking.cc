#include "tracking.h"
#include "converter.h"
#include "optimizer/global_bundle_adjuster.h"
#include "optimizer/pose_optimizer.h"
#include "optimizer/struct_bundle_adjuster.h"
#include "orbmatcher.h"

namespace FeatureSLAM {

Tracking::Tracking(System *sys, ORBVocabulary *voc, Map *map,
                   KeyFrameDatabase *keyfrm_database,
                   const string &setting_file)
    : state_(NO_IMAGES_YET), orb_voc_(voc), mpKeyFrameDB(keyfrm_database),
      initialier_(nullptr), system_(sys), map_(map), last_reloc_frm_id_(0),
      slam_mode_(true), multi_cam_(nullptr) {
  cv::FileStorage fsettings(setting_file, cv::FileStorage::READ);
  // Useless parameters
  const size_t bar_index = setting_file.find_last_of('/');
  std::string data_path = setting_file.substr(0, bar_index + 1);

  int fps = int(fsettings["Camera.fps"].real());
  if (fps == 0)
    fps = 30;

  max_frame_thr_ = fps;

  int feautre_num = fsettings["ORBextractor.nFeatures"];
  float scale_fator = fsettings["ORBextractor.scaleFactor"];
  int level_num = fsettings["ORBextractor.nLevels"];
  int init_th_fast = fsettings["ORBextractor.iniThFAST"];
  int min_th_fast = fsettings["ORBextractor.minThFAST"];

  std::string mask_left = fsettings["MaskLeft"];
  std::string mask_front = fsettings["MaskFront"];
  std::string mask_right = fsettings["MaskRight"];
  std::string mask_back = fsettings["MaskBack"];

  std::cout << " Mask  path " << std::endl;
  std::cout << "  " << data_path + mask_left << std::endl;
  std::cout << "  " << data_path + mask_front << std::endl;
  std::cout << "  " << data_path + mask_right << std::endl;
  std::cout << "  " << data_path + mask_back << std::endl;

  orb_extractor_left_ =
      new ORBextractor(feautre_num, scale_fator, level_num, init_th_fast,
                       min_th_fast, data_path + mask_left);
  orb_extractor_front_ =
      new ORBextractor(feautre_num, scale_fator, level_num, init_th_fast,
                       min_th_fast, data_path + mask_front);
  orb_extractor_right_ =
      new ORBextractor(feautre_num, scale_fator, level_num, init_th_fast,
                       min_th_fast, data_path + mask_right);
  orb_extractor_back_ =
      new ORBextractor(feautre_num, scale_fator, level_num, init_th_fast,
                       min_th_fast, data_path + mask_back);

  extractors_ =
      std::vector<ORBextractor *>{orb_extractor_left_, orb_extractor_front_,
                                  orb_extractor_right_, orb_extractor_back_};

  init_orb_extrator_ =
      new ORBextractor(int(4.0 * feautre_num), scale_fator, level_num,
                       init_th_fast, min_th_fast, data_path + mask_front);
  local_mapper_ = new LocalMapping(map_, mpKeyFrameDB);

  LoadExtrinsicParam(fsettings);

  if (fsettings["StructBA"].string().find("on") != std::string::npos) {
    std::cout << std::endl
              << kColorGreen << "🅂 🅃 🅁 🅄 🄲 🅃  🄱 🄰" << kColorReset << std::endl;
    local_mapper_->EnableStructBa();
  } else {
    std::cout << std::endl
              << kColorGreen << "🄻 🄾 🄲 🄰 🄻  🄱 🄰" << kColorReset << std::endl;
  }

  fsettings.release();

  std::cout << std::endl << "ORB Extractor Parameters: " << std::endl;
  std::cout << "- Number of Features: " << feautre_num << std::endl;
  std::cout << "- Scale Levels: " << level_num << std::endl;
  std::cout << "- Scale Factor: " << scale_fator << std::endl;
  std::cout << "- Initial Fast Threshold: " << init_th_fast << std::endl;
  std::cout << "- Minimum Fast Threshold: " << min_th_fast << std::endl;

  prematched_points_.resize(kCamNum);
  init_matches_.resize(kCamNum);
  init_mappoints_.resize(kCamNum);
}

void Tracking::LoadExtrinsicParam(const cv::FileStorage &fSettings) {

  cv::Mat tcam_01 = fSettings["Tcam_01"].mat();
  cv::Mat tcam_11 = fSettings["Tcam_11"].mat();
  cv::Mat tcam_21 = fSettings["Tcam_21"].mat();
  cv::Mat tcam_31 = fSettings["Tcam_31"].mat();
  cv::Mat front2wheel = fSettings["toWheelCoordinate"].mat();

  cam_extrinsic_ = std::vector<cv::Mat>{tcam_01, tcam_11, tcam_21, tcam_31};
  MultiCamExt::GetInstance().Initialize(cam_extrinsic_, front2wheel);
  multi_cam_ = new MultiCam(cam_extrinsic_);
}

cv::Mat
Tracking::GrabImageMultiCam(const cv::Mat &imleft, const cv::Mat &imfront,
                            const cv::Mat &imright, const cv::Mat &imback,
                            double &timestamp,
                            const SemanticSLAM::WheelOdometry &odometry) {

  //    UpdateLastFrameMapPoints();
  auto odom_vector = Converter::toOdomVector(odometry);

  if (state_ == NO_IMAGES_YET) {
    std::cout << kColorGreen << " First odometry is set " << kColorReset
              << std::endl;
    first_odom_ = odom_vector;
    MCFrame::next_id_ = 0;
  }

  Eigen::Vector3d odom_inc = Converter::GetOdomInc(odom_vector, first_odom_);

  img_gray_left_ = imleft;
  img_gray_front_ = imfront;
  img_gray_right_ = imright;
  img_gray_back_ = imback;
  img_gray_ = cv::Mat::zeros(img_gray_front_.rows, img_gray_front_.cols * 4,
                             img_gray_left_.type());

  img_gray_left_.copyTo(img_gray_.colRange(0, img_gray_left_.cols));
  img_gray_front_.copyTo(img_gray_.colRange(img_gray_left_.cols, 2 * img_gray_left_.cols));
  img_gray_right_.copyTo(img_gray_.colRange(2 * img_gray_left_.cols, 3 * img_gray_left_.cols));
  img_gray_back_.copyTo(img_gray_.colRange(3 * img_gray_left_.cols, 4 * img_gray_left_.cols));

  std::vector<cv::Mat> images{img_gray_left_, img_gray_front_, img_gray_right_, img_gray_back_};

  if (state_ == NOT_INITIALIZED || state_ == NO_IMAGES_YET) {
    extractors_[FRONT_CAMERA] = init_orb_extrator_;
  } else if (state_ == OK) {
    extractors_[FRONT_CAMERA] = orb_extractor_front_;
  }

  current_mc_frame_ =
      MCFrame(images, odom_inc, timestamp, extractors_, orb_voc_, multi_cam_);

  Track();
  return current_mc_frame_.tcw_.clone();
}

void Tracking::SetSLAMMode(const bool &flag) { slam_mode_ = flag; }

void Tracking::Track() {
  if (state_ == NO_IMAGES_YET) {
    state_ = NOT_INITIALIZED;
  }
  last_process_state_ = state_;
  unique_lock<mutex> lock(map_->mutex_map_update_);
  if (state_ == NOT_INITIALIZED) {
    Sysinitialization();
#ifdef ENABLE_VIEWER
    frame_drawer_->Update(this);
#endif
    return;
  } else {

    bool bOK = false;

    if (state_ == OK) // IMPORTANT
    {
      CheckReplacedInLastMCFrame();
      if (velocity_.empty()) {
        bOK = TrackReferenceKeyFrame();
      } else {
        bOK = TrackWithMotionModel();
        if (!bOK)
          bOK = TrackReferenceKeyFrame();
      }
    } else // Track Lost
    {
      // TO DO
      // Force to run mapping
    }

    if (slam_mode_)
      current_mc_frame_.reference_mckeyframe_ = reference_mc_keyframe_ptr_;

    if (bOK)
      bOK = TrackLocalMap();
    if (bOK) {
      state_ = OK;
    } else {
      state_ = LOST;
      std::cout << kColorRed << " Tracking Lost " << kColorReset << std::endl;
    }
#ifdef ENABLE_VIEWER
    frame_drawer_->Update(this);
#endif
    if (bOK && state_ == OK) {
      if (!last_mc_frame_.tcw_.empty()) {
        cv::Mat poseLastFrame = last_mc_frame_.tcw_.clone();
        cv::Mat last_Twc = poseLastFrame.inv();
        velocity_ = current_mc_frame_.tcw_ * last_Twc; // Tcl
      } else
        velocity_ = cv::Mat();
#ifdef ENABLE_VIEWER
      map_drawer_->SetCurrentCameraPose(current_mc_frame_.tcw_);
#endif
      if (NeedNewKeyFrame()) {
        CreateNewKeyFrame();
      }

      for (int camId = 0; camId < 4; camId++) {
        for (int i = 0; i < current_mc_frame_.frames_[camId].num_feature_;
             i++) {
          if (current_mc_frame_.frames_[camId].mappoints_[i] &&
              current_mc_frame_.frames_[camId].outliers_[i])
            current_mc_frame_.frames_[camId].mappoints_[i] = nullptr;
        }
      }
    }

    if (state_ == LOST) {
      std::cout << "Track lost soon after initialisation,  reinitializing ..."
                << std::endl;
      if (initialier_)
        delete initialier_;
      initialier_ = nullptr;
      state_ = NOT_INITIALIZED;
      velocity_ = cv::Mat();
      return;
    }

    if (state_ == OK) {
      last_mc_frame_ = MCFrame(current_mc_frame_);
    }
  }

  if (state_ == LOST)
    return;
  if (state_ == OK) {
    if (!current_mc_frame_.tcw_.empty()) {
      cv::Mat Tcr = current_mc_frame_.tcw_ *
                    current_mc_frame_.reference_mckeyframe_->GetPose().inv();
      mlRelativeFramePoses.push_back(Tcr);
    }
  }
}

void Tracking::Sysinitialization() {

  static const int kMinInitMatches = 150;

  // Do Not Have initial(ref) frame
  if (!initialier_) {
    if (current_mc_frame_.frames_[FRONT_CAMERA].keypts_.size() > 100) {
      // copy the front frame
      last_mc_frame_ = MCFrame(current_mc_frame_);
      initial_mc_frame_ = MCFrame(current_mc_frame_);

      // fill prematched point coordinates
      for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
        prematched_points_[cam_index].resize(
            current_mc_frame_.frames_[cam_index].keypts_.size());
      }

      for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
        for (size_t i = 0;
             i < current_mc_frame_.frames_[cam_index].keypts_.size(); i++) {
          prematched_points_[cam_index][i] =
              current_mc_frame_.frames_[cam_index].keypts_[i].pt;
        }
      }

      if (initialier_)
        delete initialier_;
      float reproj_err_thr = 2.0;
      float parallax_thr = 1.0; // degree
      initialier_ =
          new Initializer(current_mc_frame_, reproj_err_thr, parallax_thr);

      for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
        fill(init_matches_[cam_index].begin(), init_matches_[cam_index].end(),
             -1);
      }
      return;
    }
  } else {

    // only check front frame features num
    if ((int)current_mc_frame_.frames_[FRONT_CAMERA].keypts_.size() <= 150) {
      delete initialier_;
      initialier_ = nullptr;
      for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
        fill(init_matches_[cam_index].begin(), init_matches_[cam_index].end(),
             -1);
      }
      return;
    }

    ORBmatcher matcher(0.8, true);
    int window_size = 80;

    int nmatches = 0;

    for (int cam_index = 0; cam_index < kCamNum; cam_index++) {

      int num_matches = matcher.SearchForInitialization(
          initial_mc_frame_.frames_[cam_index],
          current_mc_frame_.frames_[cam_index], prematched_points_[cam_index],
          init_matches_[cam_index], window_size);

      if (cam_index == FRONT_CAMERA) {
        nmatches = num_matches;
      }
    }

    if (nmatches < kMinInitMatches) {

      std::cout << kColorRed << " intializer has too few matches "
                << kColorReset << std::endl;
      delete initialier_;
      initialier_ = nullptr;
      return;
    }
    std::cout << "intitalize matches : " << kColorGreen << nmatches
              << kColorReset << std::endl;
    cv::Mat trans_21;

    std::vector<vector<bool>> triangulated_list(kCamNum);

    if (initialier_->Initialize(current_mc_frame_, init_matches_, trans_21,
                                init_mappoints_, triangulated_list)) {

      std::cout
          << kColorGreen
          << " ############        success initialization ####################"
          << kColorReset << std::endl;

      // refresh the matches

      for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
        for (size_t i = 0, iend = init_matches_[cam_index].size(); i < iend;
             i++) {
          if (init_matches_[cam_index][i] >= 0 &&
              !triangulated_list[cam_index][i]) {
            init_matches_[cam_index][i] = -1;
          }
        }
      }

      // set coordinate orgin to first odom
      cv::Mat front2wheel = MultiCamExt::GetInstance().GetFront2Wheel();

      cv::Mat init2odom = MultiCamExt::GetInstance().BaselinkPose2Camera(
          Converter::toOdometryMatrix(initial_mc_frame_.odom_vector_),
          FRONT_CAMERA);
      initial_mc_frame_.SetPose(init2odom);
      initial_mc_frame_.UpdatePose();
      current_mc_frame_.SetPose(trans_21 * init2odom);
      current_mc_frame_.UpdatePose();

      CreateInitialMap();
    }
  }
}

void Tracking::CreateInitialMap() {

  MCKeyFrame *init_mc_keyframe =
      new MCKeyFrame(&initial_mc_frame_, map_, mpKeyFrameDB, true);
  MCKeyFrame *current_mc_keyframe =
      new MCKeyFrame(&current_mc_frame_, map_, mpKeyFrameDB, true);

  KeyFrame *init_keyframe = init_mc_keyframe->keyframes_[FRONT_CAMERA];
  KeyFrame *current_keyframe = current_mc_keyframe->keyframes_[FRONT_CAMERA];

  init_mc_keyframe->ComputeBoW();
  current_mc_keyframe->ComputeBoW();

  for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
    map_->AddKeyFrame(init_mc_keyframe->keyframes_[cam_index]);
    map_->AddKeyFrame(current_mc_keyframe->keyframes_[cam_index]);

    // create new map points

    for (size_t i = 0; i < init_matches_[cam_index].size(); i++) {
      if (init_matches_[cam_index][i] < 0)
        continue;

      auto cur_keyfrm = current_mc_keyframe->keyframes_[cam_index];
      auto init_keyfrm = init_mc_keyframe->keyframes_[cam_index];

      // Create MapPoint.
      cv::Mat pos_in_cam(init_mappoints_[cam_index][i]);
      cv::Mat Twc = init_keyfrm->GetPoseInverse();
      cv::Mat rwc = Twc.rowRange(0, 3).colRange(0, 3);
      cv::Mat twc = Twc.rowRange(0, 3).col(3);
      cv::Mat pw = rwc * pos_in_cam + twc;

      MapPoint *map_point = new MapPoint(pw, cur_keyfrm, map_);
      init_keyfrm->AddMapPoint(map_point, i);
      cur_keyfrm->AddMapPoint(map_point, init_matches_[cam_index][i]);
      map_point->AddObservation(init_keyfrm, i);
      map_point->AddObservation(cur_keyfrm, init_matches_[cam_index][i]);
      map_point->ComputeDistinctiveDescriptors();
      map_point->UpdateNormalAndDepth();

      // Fill Current Frame structure
      current_mc_frame_.frames_[cam_index]
          .mappoints_[init_matches_[cam_index][i]] = map_point;
      current_mc_frame_.frames_[cam_index]
          .outliers_[init_matches_[cam_index][i]] = false;
      map_->AddMapPoint(map_point);

      cur_keyfrm->UpdateConnections();
      init_keyfrm->UpdateConnections();
    }
  }

  std::cout << "New Map created with " << kColorGreen << map_->MapPointsInMap()
            << kColorReset << " points" << std::endl;

  //  // use struct bundle adjustment to prevent scale diff at the beginning
  //  const auto struct_bundle_adjuster = StructBundleAdjuster(5, 5);
  //  struct_bundle_adjuster.Optimize(current_mc_keyframe, map_);

  float median_depth = init_keyframe->ComputeSceneMedianDepth(2);

  if (median_depth < 0 || current_keyframe->TrackedMapPoints(1) < 70) {
    std::cout << "Wrong initialization, reseting..." << std::endl;
    Reset();
    return;
  }

  local_mapper_->InsertMCKeyFrame(init_mc_keyframe);
  local_mapper_->SpinOnce();
  local_mapper_->InsertMCKeyFrame(current_mc_keyframe);
  local_mapper_->SpinOnce();

  last_keyfrm_id_ = current_mc_frame_.id_;
  last_keyfrm_ = current_keyframe;

  for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
    local_keyframes_.push_back(current_mc_keyframe->keyframes_[cam_index]);
    local_keyframes_.push_back(init_mc_keyframe->keyframes_[cam_index]);
  }

  local_mappoints_ = map_->GetAllMapPoints();
  mpReferenceKF = current_keyframe;

  last_mc_frame_ = MCFrame(current_mc_frame_);
  map_->SetReferenceMapPoints(local_mappoints_);
#ifdef ENABLE_VIEWER
  map_drawer_->SetCurrentCameraPose(current_keyframe->GetPose());
#endif
  map_->keyfrm_origins_.push_back(init_keyframe);
  reference_mc_keyframe_ptr_ = current_mc_keyframe;
  state_ = OK;
  return;
}

void Tracking::CheckReplacedInLastMCFrame() {
  for (int camId = 0; camId < 4; camId++) {
    for (int i = 0; i < last_mc_frame_.frames_[camId].num_feature_; i++) {
      MapPoint *pMP = last_mc_frame_.frames_[camId].mappoints_[i];
      if (pMP) {
        MapPoint *pRep = pMP->GetReplaced();
        if (pRep) {
          last_mc_frame_.frames_[camId].mappoints_[i] = pRep;
        }
      }
    }
  }
}

bool Tracking::TrackReferenceKeyFrame() {

  ORBmatcher matcher(0.7, true);
  int nmatches = 0;

  current_mc_frame_.ComputeBoW();

  for (int i = 0; i < 4; i++) {

    vector<MapPoint *> vpMapPointMatches;
    nmatches +=
        matcher.SearchByBoW(reference_mc_keyframe_ptr_->keyframes_[i],
                            current_mc_frame_.frames_[i], vpMapPointMatches);
    current_mc_frame_.frames_[i].mappoints_ = vpMapPointMatches;
  }
  if (nmatches < 30)
    return false;

  //  auto predict_pose =
  //  MultiCamExt::GetInstance().BaselinkPose2Camera(
  //  Converter::toOdometryMatrix( current_mc_frame_.odom_vector_ ) ,
  //  FRONT_CAMERA );
  //  current_mc_frame_.SetPose(  predict_pose );

  current_mc_frame_.SetPose(last_mc_frame_.tcw_);
  current_mc_frame_.UpdatePose();

  const auto pose_optimizer = PoseOptimizer(4, 10);
  pose_optimizer.PoseOptimizationMC(&current_mc_frame_);

  int num_matches = 0;
  for (int cam_index = 0; cam_index < 4; cam_index++) {
    for (int i = 0; i < current_mc_frame_.frames_[cam_index].num_feature_;
         i++) {
      if (current_mc_frame_.frames_[cam_index].mappoints_[i]) {
        if (current_mc_frame_.frames_[cam_index].outliers_[i]) {
          MapPoint *mp = current_mc_frame_.frames_[cam_index].mappoints_[i];

          current_mc_frame_.frames_[cam_index].mappoints_[i] = nullptr;
          current_mc_frame_.frames_[cam_index].outliers_[i] = false;
          mp->is_tracked_in_view_ = false;
          mp->last_frame_seen_ = current_mc_frame_.frames_[cam_index].id_;
          nmatches--;
        } else if (current_mc_frame_.frames_[cam_index]
                       .mappoints_[i]
                       ->Observations() > 0)
          num_matches++;
      }
    }
  }

  return num_matches >= 10;
}

void Tracking::UpdateLastFrame() {

  MCKeyFrame *ref_keyfrm = last_mc_frame_.reference_mckeyframe_;
  cv::Mat Tlr = mlRelativeFramePoses.back();
  last_mc_frame_.SetPose(Tlr * ref_keyfrm->GetPose());
  last_mc_frame_.UpdatePose();
  return;
}

bool Tracking::TrackWithMotionModel() {

  ORBmatcher matcher(0.9, true);
  UpdateLastFrame();
  current_mc_frame_.SetPose(velocity_ * last_mc_frame_.tcw_);
  current_mc_frame_.UpdatePose();

  int th = 15;
  last_mc_frame_.UpdatePose();

  int num_max_matches = 0;
  std::vector<int> num_matches_list(4, 0);
  for (int camId = 0; camId < 4; camId++) {
    fill(current_mc_frame_.frames_[camId].mappoints_.begin(),
         current_mc_frame_.frames_[camId].mappoints_.end(), nullptr);

    auto &current_frame = current_mc_frame_.frames_[camId];
    auto &last_frame = last_mc_frame_.frames_[camId];

    int num_matches =
        matcher.SearchByProjection(current_frame, last_frame, th, true);
    num_matches_list[camId] = num_matches;
  }

  auto max_matches =
      max_element(num_matches_list.begin(), num_matches_list.end());
  num_max_matches = *max_matches;

  for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
    if (num_matches_list[cam_index] < 20) {
      fill(current_mc_frame_.frames_[cam_index].mappoints_.begin(),
           current_mc_frame_.frames_[cam_index].mappoints_.end(), nullptr);
      auto &current_frame = current_mc_frame_.frames_[cam_index];
      auto &last_frame = last_mc_frame_.frames_[cam_index];

      int num_matches =
          matcher.SearchByProjection(current_frame, last_frame, 2 * th, true);
      num_matches_list[cam_index] = num_matches;
    }
  }

  auto max_matches2 =
      max_element(num_matches_list.begin(), num_matches_list.end());
  num_max_matches = *max_matches2;

  if (num_max_matches < 20)
    return false;
  const auto pose_optimizer = PoseOptimizer(4, 10);
  pose_optimizer.PoseOptimizationMC(&current_mc_frame_);
  int num_mp_matches = 0;
  for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
    for (int i = 0; i < current_mc_frame_.frames_[cam_index].num_feature_;
         i++) {
      if (current_mc_frame_.frames_[cam_index].mappoints_[i]) {
        if (current_mc_frame_.frames_[cam_index].outliers_[i]) {
          MapPoint *pMP = current_mc_frame_.frames_[cam_index].mappoints_[i];

          current_mc_frame_.frames_[cam_index].mappoints_[i] = nullptr;
          current_mc_frame_.frames_[cam_index].outliers_[i] = false;
          pMP->is_tracked_in_view_ = false;
          pMP->last_frame_seen_ = current_mc_frame_.frames_[cam_index].id_;
        } else if (current_mc_frame_.frames_[cam_index]
                       .mappoints_[i]
                       ->Observations() > 0)
          num_mp_matches++;
      }
    }
  }
  return num_mp_matches >= 20;
}

bool Tracking::TrackLocalMap() {

  UpdateLocalMap();
  SearchLocalPoints();
  const auto pose_optimizer = PoseOptimizer(4, 10);
  pose_optimizer.PoseOptimizationMC(&current_mc_frame_);

  num_matched_inliers_ = 0;
  num_matched_inliers_total_ = 0;

  for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
    for (int i = 0; i < current_mc_frame_.frames_[cam_index].num_feature_;
         i++) {
      if (current_mc_frame_.frames_[cam_index].mappoints_[i]) {
        if (!current_mc_frame_.frames_[cam_index].outliers_[i]) {
          current_mc_frame_.frames_[cam_index].mappoints_[i]->IncreaseFound();
          if (current_mc_frame_.frames_[cam_index]
                  .mappoints_[i]
                  ->Observations() > 0) {
            if (cam_index == FRONT_CAMERA)
              num_matched_inliers_++;
            num_matched_inliers_total_++;
          }
        }
      }
    }
  }

  if (num_matched_inliers_ < 30)
    return false;
  return true;
}

bool Tracking::NeedNewKeyFrame() {
  if (state_ == LOST)
    return false;

  if (!slam_mode_)
    return false;

  int id_diff = current_mc_frame_.id_ - last_keyfrm_id_;

  if (num_matched_inliers_total_ > 800 && id_diff < 10)
    return false;

  if (reference_mc_keyframe_ptr_) {
    KeyFrame *pKF = reference_mc_keyframe_ptr_->keyframes_[FRONT_CAMERA];

    cv::Mat mCurrent2LastKF = current_mc_frame_.tcw_ * pKF->GetPoseInverse();

    if (cv::norm(mCurrent2LastKF.rowRange(0, 3).col(3)) < 0.35) {
      return false;
    }
  }

  //  bool bLocalMappingIdle = local_mapper_->AcceptKeyFrames();
  bool bLocalMappingIdle = true;

  // there is no keyframe for a long time
  const bool c1a =
      current_mc_frame_.id_ >= last_keyfrm_id_ + 0.2 * max_frame_thr_;
  if (num_matched_inliers_ < 100 || c1a) {
    if (bLocalMappingIdle)
      return true;
    else {
      //  local mapper is always idle
      //  local_mapper_->InterruptBA();
    }
  } else
    return false;

  return false;
}

void Tracking::CreateNewKeyFrame() {

  //  if (!local_mapper_->SetNotStop(true))
  //    return;

  MCKeyFrame *mc_keyfrm =
      new MCKeyFrame(&current_mc_frame_, map_, mpKeyFrameDB, true);

  std::cout << kColorGreen << " current mc keyframe id " << mc_keyfrm->id_
            << kColorReset << std::endl;

  mc_keyfrm->SetState(true);
  reference_mc_keyframe_ptr_ = mc_keyfrm;
  current_mc_frame_.reference_mckeyframe_ = mc_keyfrm;
  local_mapper_->InsertMCKeyFrame(mc_keyfrm);
  local_mapper_->SpinOnce();
  //  local_mapper_->SetNotStop(false);
  last_keyfrm_id_ = current_mc_frame_.id_;
}

void Tracking::SearchLocalPoints() {

  for (auto mp : local_mappoints_) {
    if (mp->isBad())
      continue;
    mp->is_tracked_in_view_ = false;
    mp->last_frame_seen_ = NONE_CAMERA;
    mp->track_cam_index1_ = NONE_CAMERA;
    mp->track_cam_index2_ = NONE_CAMERA;
  }
  for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
    for (vector<MapPoint *>::iterator
             vit = current_mc_frame_.frames_[cam_index].mappoints_.begin(),
             vend = current_mc_frame_.frames_[cam_index].mappoints_.end();
         vit != vend; vit++) {
      MapPoint *mp = *vit;
      if (mp) {
        mp->last_frame_seen_ = NONE_CAMERA;
      }
    }
  }
  for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
    for (vector<MapPoint *>::iterator
             vit = current_mc_frame_.frames_[cam_index].mappoints_.begin(),
             vend = current_mc_frame_.frames_[cam_index].mappoints_.end();
         vit != vend; vit++) {
      MapPoint *mp = *vit;
      if (mp) {
        if (mp->isBad()) {
          *vit = nullptr;
        } else {
          if (mp->last_frame_seen_ == MULTI_CAMERA) {
            continue;
          } else if (mp->last_frame_seen_ != NONE_CAMERA &&
                     mp->last_frame_seen_ != cam_index) {
            mp->IncreaseVisible();
            mp->last_frame_seen_ = MULTI_CAMERA;
            mp->is_tracked_in_view_ = false;
            continue;
          }

          mp->IncreaseVisible();
          mp->is_tracked_in_view_ = false;
          mp->last_frame_seen_ = cam_index;
        }
      }
    }
  }

  int matches = 0;

  auto frame_left_ = &(current_mc_frame_.frames_[LEFT_CAMERA]);
  auto frame_front_ = &(current_mc_frame_.frames_[FRONT_CAMERA]);
  auto frame_right_ = &(current_mc_frame_.frames_[RIGHT_CAMERA]);
  auto frame_back_ = &(current_mc_frame_.frames_[BACK_CAMERA]);

  for (auto mp : local_mappoints_) {

    if (mp->isBad())
      continue;
    if (mp->last_frame_seen_ == MULTI_CAMERA)
      continue;
    switch (mp->last_frame_seen_) {
    case LEFT_CAMERA:
      if (frame_back_->isInFrustum(mp, 0.5)) {
        mp->IncreaseVisible();
        matches++;
        mp->track_cam_index1_ = BACK_CAMERA;
      } else if (frame_front_->isInFrustum(mp, 0.5)) {
        mp->IncreaseVisible();
        matches++;
        mp->track_cam_index1_ = FRONT_CAMERA;
      }
      break;
    case FRONT_CAMERA:
      if (frame_left_->isInFrustum(mp, 0.5)) {
        mp->IncreaseVisible();
        matches++;
        mp->track_cam_index1_ = LEFT_CAMERA;
      } else if (frame_right_->isInFrustum(mp, 0.5)) {
        mp->IncreaseVisible();
        matches++;
        mp->track_cam_index1_ = RIGHT_CAMERA;
      }
      break;
    case RIGHT_CAMERA:
      if (frame_back_->isInFrustum(mp, 0.5)) {
        mp->IncreaseVisible();
        matches++;
        mp->track_cam_index1_ = BACK_CAMERA;
      } else if (frame_front_->isInFrustum(mp, 0.5)) {
        mp->IncreaseVisible();
        matches++;
        mp->track_cam_index1_ = FRONT_CAMERA;
      }
      break;
    case BACK_CAMERA:
      if (frame_left_->isInFrustum(mp, 0.5)) {
        mp->IncreaseVisible();
        matches++;
        mp->track_cam_index1_ = LEFT_CAMERA;
      } else if (frame_right_->isInFrustum(mp, 0.5)) {
        mp->IncreaseVisible();
        matches++;
        mp->track_cam_index1_ = RIGHT_CAMERA;
      }
      break;
    case NONE_CAMERA:
      const bool bTrackedSameCam = false;
      if (bTrackedSameCam) {
        if (TrackUntrackedPointsSameCam(mp))
          matches++;
      } else {
        if (TrackUntrackedPoints(mp))
          matches++;
      }
      break;
    }
  }

  // find matches
  if (matches > 0) {
    ORBmatcher matcher(0.8);
    int th = 1;
    int nmatches = 0;
    for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
      nmatches +=
          matcher.SearchByProjection(current_mc_frame_.frames_[cam_index],
                                     local_mappoints_, cam_index, th);
    }
  }
}

bool Tracking::TrackUntrackedPointsSameCam(MapPoint *mp) {

  auto frame_left_ = &current_mc_frame_.frames_[LEFT_CAMERA];
  auto frame_front_ = &current_mc_frame_.frames_[FRONT_CAMERA];
  auto frame_right_ = &current_mc_frame_.frames_[RIGHT_CAMERA];
  auto frame_back_ = &current_mc_frame_.frames_[BACK_CAMERA];

  std::map<KeyFrame *, size_t> obs = mp->GetObservations();
  std::vector<int> vNCamCounter(4, 0);
  for (std::map<KeyFrame *, size_t>::iterator sit = obs.begin(),
                                              send = obs.end();
       sit != send; sit++) {
    vNCamCounter[sit->first->camera_index_] += 1;
  }
  int predict_cam = 0;
  int nMaxobsCamIndex = 0;
  for (int camId = 0; camId < 4; camId++) {
    if (vNCamCounter[camId] > nMaxobsCamIndex) {
      nMaxobsCamIndex = vNCamCounter[camId];
      predict_cam = camId;
    }
  }
  switch (predict_cam) {
  case LEFT_CAMERA:
    if (frame_left_->isInFrustum(mp, 0.5)) {
      mp->IncreaseVisible();
      mp->track_cam_index1_ = LEFT_CAMERA;
      return true;
    }
    break;
  case RIGHT_CAMERA:
    if (frame_right_->isInFrustum(mp, 0.5)) {
      mp->IncreaseVisible();
      mp->track_cam_index1_ = RIGHT_CAMERA;
      return true;
    }
    break;
  case FRONT_CAMERA:
    if (frame_front_->isInFrustum(mp, 0.5)) {
      mp->IncreaseVisible();
      mp->track_cam_index1_ = FRONT_CAMERA;
      return true;
    }
    break;
  case BACK_CAMERA:
    if (frame_back_->isInFrustum(mp, 0.5)) {
      mp->IncreaseVisible();
      mp->track_cam_index1_ = BACK_CAMERA;
      return true;
    }
    break;
  }
  return false;
}
bool Tracking::TrackUntrackedPoints(MapPoint *mp) {
  bool is_tracked = false;

  float view_cos_limit = 0.5f;

  if (current_mc_frame_.frames_[BACK_CAMERA].isInFrustum(mp, view_cos_limit)) {
    mp->IncreaseVisible();
    if (!is_tracked) {
      mp->track_cam_index1_ = BACK_CAMERA;
      is_tracked = true;
    } else {
      mp->track_cam_index2_ = BACK_CAMERA;
      return true;
    }
  }
  if (current_mc_frame_.frames_[FRONT_CAMERA].isInFrustum(mp, view_cos_limit)) {
    mp->IncreaseVisible();
    if (!is_tracked) {
      mp->track_cam_index1_ = FRONT_CAMERA;
      is_tracked = true;
    } else {
      mp->track_cam_index2_ = FRONT_CAMERA;
      return true;
    }
  }
  if (current_mc_frame_.frames_[LEFT_CAMERA].isInFrustum(mp, view_cos_limit)) {
    mp->IncreaseVisible();
    if (!is_tracked) {
      mp->track_cam_index1_ = LEFT_CAMERA;
      is_tracked = true;
    } else {
      mp->track_cam_index2_ = LEFT_CAMERA;
      return true;
    }
  }
  if (current_mc_frame_.frames_[RIGHT_CAMERA].isInFrustum(mp, view_cos_limit)) {
    mp->IncreaseVisible();
    if (!is_tracked) {
      mp->track_cam_index1_ = RIGHT_CAMERA;
      is_tracked = true;
    } else {
      mp->track_cam_index2_ = RIGHT_CAMERA;
      return true;
    }
  }
  if (is_tracked) {
    mp->is_tracked_in_view_ = true;
  }
  return is_tracked;
}

void Tracking::UpdateLocalMap() {
  UpdateLocalKeyFrames();
  UpdateLocalPoints();
  map_->SetReferenceMapPoints(local_mappoints_);
}

void Tracking::UpdateLocalPoints() {
  local_mappoints_.clear();

  for (auto keyfrm : local_keyframes_) {

    const std::vector<MapPoint *> mappoints_matched =
        keyfrm->GetMapPointMatches();

    for (auto mp : mappoints_matched)

    {
      if (!mp)
        continue;
      if (mp->track_ref_mcfrm_id_ == current_mc_frame_.id_)
        continue;
      if (!mp->isBad()) {
        local_mappoints_.push_back(mp);
        mp->track_ref_mcfrm_id_ = current_mc_frame_.id_;
      }
    }
  }
}

void Tracking::UpdateLocalKeyFrames() {
  std::map<KeyFrame *, int> keyframe_counter;

  for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
    for (int i = 0; i < current_mc_frame_.frames_[cam_index].num_feature_;
         i++) {
      if (current_mc_frame_.frames_[cam_index].mappoints_[i]) {
        MapPoint *mp = current_mc_frame_.frames_[cam_index].mappoints_[i];
        if (!mp->isBad()) {
          const map<KeyFrame *, size_t> observations = mp->GetObservations();
          for (map<KeyFrame *, size_t>::const_iterator
                   it = observations.begin(),
                   itend = observations.end();
               it != itend; it++)
            keyframe_counter[it->first]++;
        } else {
          current_mc_frame_.frames_[cam_index].mappoints_[i] = nullptr;
        }
      }
    }
  }

  if (keyframe_counter.empty())
    return;

  int max = 0;
  KeyFrame *keyfram_max = nullptr;
  local_keyframes_.clear();
  local_keyframes_.reserve(3 * keyframe_counter.size());

  for (map<KeyFrame *, int>::const_iterator it = keyframe_counter.begin(),
                                            itEnd = keyframe_counter.end();
       it != itEnd; it++) {
    KeyFrame *keyfrm = it->first;
    if (it->second < 4)
      continue;
    if (keyfrm->isBad())
      continue;

    if (it->second > max && keyfrm->camera_index_ == FRONT_CAMERA) {
      max = it->second;
      keyfram_max = keyfrm;
    }

    local_keyframes_.push_back(it->first);
    keyfrm->track_reffrm_id_ = current_mc_frame_.id_;
  }

  for (auto keyfrm : local_keyframes_) {
    if (local_keyframes_.size() > 120)
      break;

    const vector<KeyFrame *> neigh_keyfrms =
        keyfrm->GetBestCovisibilityKeyFrames(10);
    for (auto ngh_keyfrm : neigh_keyfrms) {
      if (!ngh_keyfrm->isBad()) {
        if (ngh_keyfrm->track_reffrm_id_ != current_mc_frame_.id_) {
          local_keyframes_.push_back(ngh_keyfrm);
          ngh_keyfrm->track_reffrm_id_ = current_mc_frame_.id_;
          break;
        }
      }
    }
    const set<KeyFrame *> child_keyfrms = keyfrm->GetChilds();

    for (auto child_keyfrm : child_keyfrms) {
      if (!child_keyfrm->isBad()) {
        if (child_keyfrm->track_reffrm_id_ != current_mc_frame_.id_) {
          local_keyframes_.push_back(child_keyfrm);
          child_keyfrm->track_reffrm_id_ = current_mc_frame_.id_;
          break;
        }
      }
    }
    KeyFrame *parent_keyfrm = keyfrm->GetParent();
    if (parent_keyfrm) {
      if (parent_keyfrm->track_reffrm_id_ != current_mc_frame_.id_) {
        local_keyframes_.push_back(parent_keyfrm);
        parent_keyfrm->track_reffrm_id_ = current_mc_frame_.id_;
        break;
      }
    }
  }
  if (keyfram_max) {
    {

      reference_mc_keyframe_ptr_ = keyfram_max->GetMCKeyFrame();
      current_mc_frame_.reference_mckeyframe_ = reference_mc_keyframe_ptr_;
    }
  }
}

void Tracking::Reset() {

#ifdef ENABLE_VIEWER

  if (viewer_) {
    viewer_->RequestStop();
    while (!viewer_->isStopped())
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
  }
#endif
  std::cout << "System Reseting" << std::endl;

  // Reset Local Mapping
  std::cout << "Reseting Local Mapper...";
  //  local_mapper_->RequestReset();
  // local_mapper->Reset();
  std::cout << " done" << std::endl;

  std::cout << " done" << std::endl;

  // Clear BoW Database
  std::cout << "Reseting Database...";
  mpKeyFrameDB->clear();
  std::cout << " done" << std::endl;

  // Clear Map (this erase MapPoints and KeyFrames)
  map_->clear();

  KeyFrame::next_id_ = 0;
  Frame::next_id_ = 0;
  MCFrame::next_id_ = 0;
  MCKeyFrame::next_id_ = 0;

  state_ = NO_IMAGES_YET;

  if (initialier_) {
    delete initialier_;
    initialier_ = nullptr;
  }

  mlRelativeFramePoses.clear();
  mlpReferences.clear();
  mlFrameTimes.clear();
  mlbLost.clear();
#ifdef ENABLE_VIEWER

  if (viewer_)
    viewer_->Release();
#endif
}
}
