#include "local_mapping.h"
#include "converter.h"
#include "optimizer/local_bundle_adjuster.h"
#include "optimizer/struct_bundle_adjuster.h"
#include "orbmatcher.h"
#include <iostream>
#include <mutex>

namespace FeatureSLAM
{

  LocalMapping::LocalMapping(Map *map, KeyFrameDatabase *keyfrm_databse)
      : reset_requested_(false), terminate_requested_(false),
        is_terminated_(true), map_(map), use_struct_ba_(false),
        keyframe_database_(keyfrm_databse), abort_ba_(false), stopped_(false),
        stop_requested_(false), not_stop_(false), accept_keyfrms_(true) {}

  void LocalMapping::Run()
  {

    std::cout << kColorGreen << " start mapping thread " << kColorReset
              << std::endl;

    is_terminated_ = false;
    while (true)
    {

      std::this_thread::sleep_for(std::chrono::milliseconds(5));

      // Lock
      SetKeyframeAcceptability(false);

      if (CheckNewKeyFrames())
      {

        // Compute Keyframe Bow, update mappoints observations and descriptor
        ProcessNewKeyFrame();

        // Remove unstable mapppoints and store good map points
        MapPointCulling();

        // Match and triangulate features
        CreateNewMapPoints();

        // Fuse new map points
        SearchInNeighbors();

        abort_ba_ = false;

        if (map_->KeyFramesInMap() > 4)
        {

          const auto local_ba = LocalBundleAdjuster(5, 5);
          local_ba.Optimize(cur_keyfrm_->GetMCKeyFrame(), map_);
        }

        // there is no need for keyframe culling
        // KeyFrameCulling();
      }
      else if (Stop())
      {
        while (isStopped() && !CheckTerminate())
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
        if (CheckTerminate())
          break;
      }
      ResetIfRequested();
      SetKeyframeAcceptability(true);

      // if terminate is required, end loop
      if (CheckTerminate())
        break;
    }
    SetTerminate();
  }

  void LocalMapping::InsertMCKeyFrame(MCKeyFrame *mc_keyfrm)
  {

    {
      unique_lock<mutex> lock(mutex_new_keyfrm_);
      keyfrms_queue_.push_back(mc_keyfrm);
    }
  }

  bool LocalMapping::CheckNewKeyFrames()
  {
    unique_lock<mutex> lock(mutex_new_keyfrm_);
    return (!keyfrms_queue_.empty());
  }

  void LocalMapping::SpinOnce()
  {

    ProcessNewKeyFrame();

    // Remove unstable mapppoints and store good map points
    MapPointCulling();
    // Match and triangulate features
    CreateNewMapPoints();
    // Fuse new map points
    SearchInNeighbors();
    abort_ba_ = false;
    // Local Bundle Adjustment
    if (map_->GetAllMCKeyFrames().size() > 2)
    {

      if (cur_keyfrm_->GetMCKeyFrame()->id_ % 4 != 0 ||
          cur_keyfrm_->GetMCKeyFrame()->id_ == 0)
        return;

      if (use_struct_ba_)
      {

        const auto struct_ba = StructBundleAdjuster(3, 5);
        struct_ba.Optimize(cur_keyfrm_->GetMCKeyFrame(), map_);
      }
      else
      {
        const auto local_ba = LocalBundleAdjuster();
        local_ba.Optimize(cur_keyfrm_->GetMCKeyFrame(), map_);
      }
    }
  }

  // question: EnableStructBa?
  void LocalMapping::EnableStructBa() { use_struct_ba_ = true; }

  void LocalMapping::ProcessNewKeyFrame()
  {

    std::vector<KeyFrame *> keyframes;

    {
      unique_lock<mutex> lock(mutex_new_keyfrm_);
      auto cur_mc_keyfrm = keyfrms_queue_.front();
      cur_keyfrm_ = cur_mc_keyfrm->keyframes_[FRONT_CAMERA];
      keyfrms_queue_.pop_front();

      for (auto keyfrm : cur_mc_keyfrm->keyframes_)
      {
        keyframes.push_back(keyfrm);
      }
    }

    // Process each keyframe
    for (auto kf : keyframes)
    {
      kf->ComputeBoW();
      const vector<MapPoint *> mappoint_matches = kf->GetMapPointMatches();

      for (size_t keypoint_id = 0; keypoint_id < mappoint_matches.size();
           keypoint_id++)
      {
        MapPoint *mp = mappoint_matches[keypoint_id];
        if (mp)
        {
          if (!mp->isBad())
          {

            if (!mp->IsInKeyFrame(kf))
            {
              mp->AddObservation(kf, keypoint_id);
              mp->UpdateNormalAndDepth();
              mp->ComputeDistinctiveDescriptors();
            }
            else
            {
              recent_added_mappoints_.push_back(mp);
            }
          }
        }
      }
      kf->UpdateConnections();
      map_->AddKeyFrame(kf);
    }
  }

  void LocalMapping::SearchInNeighbors()
  {

    auto cur_mc_keyfrm = cur_keyfrm_->GetMCKeyFrame();
    if (cur_mc_keyfrm)
    {
      for (auto keyfrm : cur_mc_keyfrm->keyframes_)
        SearchFrameInNeighbors(keyfrm);
    }
  }

  void LocalMapping::MapPointCulling()
  {

    list<MapPoint *>::iterator lit = recent_added_mappoints_.begin();
    const unsigned long int cur_mc_keyfrm_id = cur_keyfrm_->GetMCKeyFrame()->id_;
    float observed_ratio_thr = 0.25f; // 0.3f
    int num_obs_thr = 2;
    while (lit != recent_added_mappoints_.end())
    {
      MapPoint *mp = *lit;
      if (mp->isBad())
      {
        lit = recent_added_mappoints_.erase(lit);
      }
      else if (mp->GetFoundRatio() < observed_ratio_thr)
      {
        mp->SetBadFlag();
        lit = recent_added_mappoints_.erase(lit);
      }
      else if (((int)cur_mc_keyfrm_id - (int)mp->first_mc_keyfrm_id_) >= 2 &&
               mp->Observations() <= num_obs_thr)
      {
        mp->SetBadFlag();
        lit = recent_added_mappoints_.erase(lit);
      }
      else if (((int)cur_mc_keyfrm_id - (int)mp->first_mc_keyfrm_id_) >= 3)
        lit = recent_added_mappoints_.erase(lit);
      else
        lit++;
    }
  }

  void LocalMapping::CreateNewMapPoints()
  {

    auto cur_mc_keyfrm = cur_keyfrm_->GetMCKeyFrame();
    for (auto keyfrm : cur_mc_keyfrm->keyframes_)
    {
      CreateNewMapPoints(keyfrm);
    }
  }

  int LocalMapping::CreateNewMapPoints(KeyFrame *cur_keyfrm)
  {

    int num_covisibilities = 20;
    constexpr float chi_sq_2D = 5.99146;

    // Retrieve neighbor keyframes in covisibility graph
    const vector<KeyFrame *> cur_covisibility_keyframes =
        map_->KeyFramesInMap() < 20
            ? map_->GetAllKeyFrames()
            : cur_keyfrm->GetBestCovisibilityKeyFrames(num_covisibilities);

    int new_point_count = 0;

    //  ORBmatcher matcher(0.6, false);
    ORBmatcher matcher(0.0, false);

    Mat33_t cur_rcw = Converter::toMatrix3d(cur_keyfrm->GetRotation());
    Mat33_t cur_rwc = cur_rcw.transpose();

    auto cam_pose_1w_ = Converter::toMatrix4d(cur_keyfrm->GetPose());
    //  Vec3_t
    Vec3_t cur_cam_center = Converter::toVector3d(cur_keyfrm->GetCameraCenter());

    const float ratio_factor_thr = 2.0f * cur_keyfrm->scale_factor_;

    float rays_parallax_deg_thr = 1.0f;
    float cos_rays_parallax_thr = std::cos(rays_parallax_deg_thr * M_PI / 180.0);

    for (auto ngh_keyfrm : cur_covisibility_keyframes)
    {

      if (ngh_keyfrm->id_ == cur_keyfrm->id_)
        continue;

      auto cam_pose_2w_ = Converter::toMatrix4d(ngh_keyfrm->GetPose());

      Vec3_t ngh_cam_center =
          Converter::toVector3d(ngh_keyfrm->GetCameraCenter());
      Vec3_t baseline_vec = ngh_cam_center - cur_cam_center;
      const float baseline_dist = baseline_vec.norm();

      const float median_depth_in_ngh = ngh_keyfrm->ComputeSceneMedianDepth(2);
      const float ratio_baseline_depth = baseline_dist / median_depth_in_ngh;

      // origin 0.1 is used
      if (ratio_baseline_depth < 0.02)
        continue;

      // estimate matches between the current and neighbor keyframes,
      // then reject outliers using Essential matrix computed from the two camera
      // poses

      // (cur bearing) * E_ngh_to_cur * (ngh bearing) = 0
      // const Mat33_t E_ngh_to_cur =
      // solve::essential_solver::create_E_21(ngh_keyfrm, cur_keyfrm);

      cv::Mat E12 = CreateE12(cur_keyfrm, ngh_keyfrm);

      // Search matches that fullfil epipolar constraint
      vector<pair<size_t, size_t>> matched_pair;
      matcher.SearchForTriangulation(cur_keyfrm, ngh_keyfrm, E12, matched_pair);

      Mat33_t ngh_rcw = Converter::toMatrix3d(ngh_keyfrm->GetRotation());
      Mat33_t ngh_rwc = ngh_rcw.transpose();

      // Triangulate each match
      const int num_of_matches = matched_pair.size();

      for (int ikp = 0; ikp < num_of_matches; ikp++)
      {
        const int &idx_cur = matched_pair[ikp].first;
        const int &idx_ngh = matched_pair[ikp].second;

        const cv::KeyPoint &keypt1 = cur_keyfrm->keypts_[idx_cur];
        const cv::KeyPoint &keypt2 = ngh_keyfrm->keypts_[idx_ngh];

        // rays with reference of each camera
        auto ray_c_1 = Converter::KeypointToBearing(keypt1);
        auto ray_c_2 = Converter::KeypointToBearing(keypt2);

        // rays with the world reference
        Vec3_t ray_w_1 = cur_rwc * ray_c_1;
        Vec3_t ray_w_2 = ngh_rwc * ray_c_2;

        const float cos_rays_parallax = ray_w_1.dot(ray_w_2);

        Vec3_t pos_w;

        if (cos_rays_parallax > 0 &&
            (cos_rays_parallax < cos_rays_parallax_thr))
        {

          pos_w = Triangulator::Triangulate(ray_c_1, ray_c_2, cam_pose_1w_, cam_pose_2w_);
        }
        else
          continue;

        // check the triangulated point is located in front of the two cameras

        Vec4_t point_in_cur =
            cam_pose_1w_ * Vec4_t(pos_w.x(), pos_w.y(), pos_w.z(), 1);
        Vec4_t point_in_ngh =
            cam_pose_2w_ * Vec4_t(pos_w.x(), pos_w.y(), pos_w.z(), 1);

        if (point_in_cur.z() < 0 || point_in_ngh.z() < 0)
          continue;

        // reject the point if reprojection errors are larger than reasonable
        // threshold
        auto reproj_in_cur = Converter::ReprojectToImage(
            cv::Point3f(point_in_cur.x(), point_in_cur.y(), point_in_cur.z()));
        auto reproj_in_ngh = Converter::ReprojectToImage(
            cv::Point3f(point_in_ngh.x(), point_in_ngh.y(), point_in_ngh.z()));

        float reproj_err_norm_cur = hypot(reproj_in_cur.x - keypt1.pt.x, reproj_in_cur.y - keypt1.pt.y);
        float reproj_err_norm_ngh = hypot(reproj_in_ngh.x - keypt2.pt.x, reproj_in_ngh.y - keypt2.pt.y);

        // checkout reprojection  error

        if (reproj_err_norm_cur * reproj_err_norm_cur > chi_sq_2D * cur_keyfrm->level_sigma2_[keypt1.octave])
          continue;

        if (reproj_err_norm_ngh * reproj_err_norm_ngh >
            chi_sq_2D * ngh_keyfrm->level_sigma2_[keypt2.octave])
          continue;

        // reject the point if the real scale factor and the predicted one are
        // much different

        Vec3_t cam_1_to_lm_vec = pos_w - cur_cam_center;
        float cam_1_to_lm_dist = cam_1_to_lm_vec.norm();

        Vec3_t cam_2_to_lm_vec = pos_w - ngh_cam_center;
        float cam_2_to_lm_dist = cam_2_to_lm_vec.norm();

        if (cam_1_to_lm_dist == 0 || cam_2_to_lm_dist == 0)
          continue;
        const float ratio_dist = cam_2_to_lm_dist / cam_1_to_lm_dist;
        const float ratio_octave = cur_keyfrm->scale_factors_[keypt1.octave] /
                                   ngh_keyfrm->scale_factors_[keypt2.octave];

        if (ratio_dist * ratio_factor_thr < ratio_octave ||
            ratio_dist > ratio_octave * ratio_factor_thr)
          continue;

        // Triangulation is succesfull

        cv::Mat pos_w3d = (cv::Mat_<float>(3, 1) << pos_w.x(), pos_w.y(), pos_w.z());
        MapPoint *mp = new MapPoint(pos_w3d, cur_keyfrm, map_);
        mp->AddObservation(cur_keyfrm, idx_cur);
        mp->AddObservation(ngh_keyfrm, idx_ngh);

        cur_keyfrm->AddMapPoint(mp, idx_cur);
        ngh_keyfrm->AddMapPoint(mp, idx_ngh);
        mp->ComputeDistinctiveDescriptors();
        mp->UpdateNormalAndDepth();
        map_->AddMapPoint(mp);
        recent_added_mappoints_.push_back(mp);
        new_point_count++;
      }
    }

    return new_point_count;
  }

  void LocalMapping::SearchFrameInNeighbors(KeyFrame *cur_keyfrm)
  {

    // fuse mappoints
    int num_covisibilities = 20;
    const vector<KeyFrame *> cur_covisibility_keyframes = cur_keyfrm->GetBestCovisibilityKeyFrames(num_covisibilities);

    vector<KeyFrame *> fuse_tgt_keyfrms;

    for (auto ngh_keyfrm : cur_covisibility_keyframes)
    {
      if (ngh_keyfrm->isBad() || ngh_keyfrm->fuse_target_frm_id_ == cur_keyfrm->id_)
        continue;
      fuse_tgt_keyfrms.push_back(ngh_keyfrm);
      ngh_keyfrm->fuse_target_frm_id_ = cur_keyfrm->id_;
      //   Thers is no need to extend the second neighbors
    }

    ORBmatcher matcher;
    vector<MapPoint *> mappoint_matches = cur_keyfrm->GetMapPointMatches();
    for (auto fuse_tgt_keyfrm : fuse_tgt_keyfrms)
    {
      matcher.Fuse(fuse_tgt_keyfrm, mappoint_matches);
    }
    // Search matches by projection from target keyframe to current keyframe
    vector<MapPoint *> fuse_condidate_mappoints;
    fuse_condidate_mappoints.reserve(fuse_tgt_keyfrms.size() * mappoint_matches.size());

    for (auto fuse_tgt_keyfrm : fuse_tgt_keyfrms)
    {

      vector<MapPoint *> mappoint_matches_tgt = fuse_tgt_keyfrm->GetMapPointMatches();

      for (auto tgt_mp : mappoint_matches_tgt)
      {
        if (!tgt_mp)
          continue;
        if (tgt_mp->isBad() || tgt_mp->fuse_candidate_keyfrm_id_ == cur_keyfrm->id_)
          continue;
        tgt_mp->fuse_candidate_keyfrm_id_ = cur_keyfrm->id_;
        fuse_condidate_mappoints.push_back(tgt_mp);
      }
    }

    matcher.Fuse(cur_keyfrm, fuse_condidate_mappoints);

    // Update points
    mappoint_matches = cur_keyfrm->GetMapPointMatches();
    for (auto mp : mappoint_matches)
    {
      if (mp)
      {
        if (!mp->isBad())
        {
          mp->ComputeDistinctiveDescriptors();
          mp->UpdateNormalAndDepth();
        }
      }
    }
    cur_keyfrm->UpdateConnections();
  }

  cv::Mat LocalMapping::CreateE12(KeyFrame *&pKF1, KeyFrame *&pKF2)
  {
    // Essential Matrix: t12 cross R12
    // Fundamental Matrix: inv(K1)*E*inv(K2)

    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w * R2w.t();
    cv::Mat t12 = -R1w * R2w.t() * t2w + t1w;
    cv::Mat t12x = SkewSymmetricMatrix(t12);
    return t12x * R12;
  }

  bool LocalMapping::Stop()
  {
    unique_lock<mutex> lock(mutex_stop);
    if (stop_requested_ && !not_stop_)
    {
      stopped_ = true;
      cout << "Local Mapping STOP" << endl;
      return true;
    }

    return false;
  }

  bool LocalMapping::isStopped()
  {
    unique_lock<mutex> lock(mutex_stop);
    return stopped_;
  }

  bool LocalMapping::stopRequested()
  {
    unique_lock<mutex> lock(mutex_stop);
    return stop_requested_;
  }

  void LocalMapping::Release()
  {
    unique_lock<mutex> lock(mutex_stop);
    unique_lock<mutex> lock2(mutex_finish_);
    if (is_terminated_)
      return;
    stopped_ = false;
    stop_requested_ = false;

    for (auto mc_keyfrm : keyfrms_queue_)
    {
      delete mc_keyfrm;
    }

    keyfrms_queue_.clear();
    cout << "Local Mapping RELEASE" << endl;
  }

  bool LocalMapping::AcceptKeyFrames()
  {
    unique_lock<mutex> lock(mutex_accept_);
    return accept_keyfrms_;
  }

  void LocalMapping::SetKeyframeAcceptability(bool flag)
  {
    unique_lock<mutex> lock(mutex_accept_);
    accept_keyfrms_ = flag;
  }

  bool LocalMapping::SetNotStop(bool flag)
  {
    unique_lock<mutex> lock(mutex_stop);

    if (flag && stopped_)
      return false;

    not_stop_ = flag;

    return true;
  }

  void LocalMapping::InterruptBA()
  {

    std::cout << kColorRed << " can not interrupt BA " << kColorReset
              << std::endl;
    //    abort_ba_ = true;
  }

  void LocalMapping::KeyFrameCulling()
  {
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are
    // seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points

    vector<KeyFrame *> local_keyfrms = cur_keyfrm_->GetVectorCovisibleKeyFrames();

    for (auto keyfrm : local_keyfrms)
    {
      if (keyfrm->id_ == 0)
        continue;
      const vector<MapPoint *> mappoints = keyfrm->GetMapPointMatches();

      const int kObsThr = 3;
      int redundant_obs_num = 0;
      int mps_num = 0;

      for (size_t idx = 0; idx < mappoints.size(); idx++)
      {
        MapPoint *mp = mappoints[idx];
        if (mp)
        {
          if (!mp->isBad())
          {
            mps_num++;
            if (mp->Observations() > kObsThr)
            {
              const int &scaleLevel = keyfrm->keypts_[idx].octave;
              const map<KeyFrame *, size_t> observations = mp->GetObservations();
              int obs_num = 0;

              for (auto mit : observations)
              {
                KeyFrame *keyfrm2 = mit.first;
                if (keyfrm2 == keyfrm)
                  continue;
                const int &scaleLeveli = keyfrm2->keypts_[mit.second].octave;

                // Scale Condition
                if (scaleLeveli <= scaleLevel + 1)
                {
                  obs_num++;
                  if (obs_num >= kObsThr)
                    break;
                }
              }
              if (obs_num >= kObsThr)
              {
                redundant_obs_num++;
              }
            }
          }
        }
      }
      if (redundant_obs_num > 0.9 * mps_num)
        keyfrm->SetBadFlag();
    }
  }

  cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
  {
    return (cv::Mat_<float>(3, 3) << 0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2), 0, -v.at<float>(0), -v.at<float>(1), v.at<float>(0),
            0);
  }

  void LocalMapping::RequestReset()
  {
    {
      unique_lock<mutex> lock(mutex_reset_);
      reset_requested_ = true;
    }

    while (1)
    {
      {
        unique_lock<mutex> lock2(mutex_reset_);
        if (!reset_requested_)
          break;
      }
      // usleep(3000);
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
  }

  void LocalMapping::ResetIfRequested()
  {
    unique_lock<mutex> lock(mutex_reset_);
    if (reset_requested_)
    {
      keyfrms_queue_.clear();
      recent_added_mappoints_.clear();
      reset_requested_ = false;
    }
  }

  void LocalMapping::RequestFinish()
  {
    unique_lock<mutex> lock(mutex_finish_);
    terminate_requested_ = true;
  }

  bool LocalMapping::CheckTerminate()
  {
    unique_lock<mutex> lock(mutex_finish_);
    return terminate_requested_;
  }

  void LocalMapping::SetTerminate()
  {
    unique_lock<mutex> lock(mutex_finish_);
    is_terminated_ = true;
    unique_lock<mutex> lock2(mutex_stop);
    stopped_ = true;
  }

  bool LocalMapping::isFinished()
  {
    unique_lock<mutex> lock(mutex_finish_);
    return is_terminated_;
  }

  int LocalMapping::McKeyframesInQueue()
  {
    unique_lock<std::mutex> lock(mutex_new_keyfrm_);
    return keyfrms_queue_.size();
  }
}
