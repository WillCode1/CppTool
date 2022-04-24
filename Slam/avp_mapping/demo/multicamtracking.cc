#include "multicamtracking.h"
#include "converter.h"
#include "map.h"
#include "optimizer/pose_optimizer.h"
#include "orbmatcher.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
// Use opengv to sovle central camera pnp problem ,
#include <colordef.h>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

using namespace opengv;
namespace FeatureSLAM {

MultiCamTracking::MultiCamTracking(ORBVocabulary *voc, Map *map,
                                   KeyFrameDatabase *keyfrm_database,
                                   const string &config_file,
                                   MapViewer *mapviewer)
    : state_(LOST), orb_vocabulary_(voc), keyfrm_database_(keyfrm_database),
      map_viewer_(mapviewer), map_(map), keyfrm_dist_thr_(10.0),
      mappoint_dist_thr_(100.0), last_reloc_frm_id_(0) {

  cv::FileStorage settings(config_file, cv::FileStorage::READ);

  const size_t bar_index = config_file.find_last_of('/');
  std::string data_path = config_file.substr(0, bar_index + 1);

  int num_features = settings["ORBextractor.nFeatures"];
  float sale_factor = settings["ORBextractor.scaleFactor"];
  int num_levels = settings["ORBextractor.nLevels"];
  int ini_thr_fast = settings["ORBextractor.iniThFAST"];
  int min_thr_fast = settings["ORBextractor.minThFAST"];

  std::string mask_left = settings["MaskLeft"].string();
  std::string mask_front = settings["MaskFront"].string();
  std::string mask_right = settings["MaskRight"].string();
  std::string mask_back = settings["MaskBack"].string();

  std::cout << " Mask  path " << std::endl;
  std::cout << "   " << data_path + mask_left << std::endl;
  std::cout << "   " << data_path + mask_front << std::endl;
  std::cout << "   " << data_path + mask_right << std::endl;
  std::cout << "   " << data_path + mask_back << std::endl;

  extractor_left_ =
      new ORBextractor(num_features, sale_factor, num_levels, ini_thr_fast,
                       min_thr_fast, data_path + mask_left);
  extractor_front_ =
      new ORBextractor(num_features, sale_factor, num_levels, ini_thr_fast,
                       min_thr_fast, data_path + mask_front);
  extractor_right_ =
      new ORBextractor(num_features, sale_factor, num_levels, ini_thr_fast,
                       min_thr_fast, data_path + mask_right);
  extractor_back_ =
      new ORBextractor(num_features, sale_factor, num_levels, ini_thr_fast,
                       min_thr_fast, data_path + mask_back);

  extractors_ = std::vector<ORBextractor *>{extractor_left_, extractor_front_,
                                            extractor_right_, extractor_back_};
  LoadExtrinsicParam(settings);

  settings.release();

  std::cout << std::endl << "ORB Extractor Parameters: " << std::endl;
  std::cout << "- Number of Features: " << num_features << std::endl;
  std::cout << "- Scale Levels: " << num_levels << std::endl;
  std::cout << "- Scale Factor: " << sale_factor << std::endl;
  std::cout << "- Initial Fast Threshold: " << ini_thr_fast << std::endl;
  std::cout << "- Minimum Fast Threshold: " << min_thr_fast << std::endl
            << std::endl;
}

void MultiCamTracking::LoadExtrinsicParam(const cv::FileStorage &settings) {

  cam_extrinsic_.resize(4);

  cam_extrinsic_[LEFT_CAMERA] = settings["Tcam_01"].mat();
  cam_extrinsic_[FRONT_CAMERA] = settings["Tcam_11"].mat();
  cam_extrinsic_[RIGHT_CAMERA] = settings["Tcam_21"].mat();
  cam_extrinsic_[BACK_CAMERA] = settings["Tcam_31"].mat();
  multicam_ = new MultiCam(cam_extrinsic_);
}

cv::Mat MultiCamTracking::GrabImageMultiCam(const cv::Mat &imleft,
                                            const cv::Mat &imfront,
                                            const cv::Mat &imright,
                                            const cv::Mat &imback,
                                            double &timestamp) {
  img_gray_left_ = imleft;
  img_gray_front_ = imfront;
  img_gray_right_ = imright;
  img_gray_back_ = imback;

  img_gray_ = cv::Mat::zeros(img_gray_front_.rows * 2, img_gray_front_.cols * 2,
                             img_gray_left_.type());

  img_gray_front_.copyTo(img_gray_.rowRange(0, img_gray_left_.rows)
                             .colRange(0, img_gray_left_.cols));
  img_gray_left_.copyTo(
      img_gray_.rowRange(0, img_gray_left_.rows)
          .colRange(img_gray_left_.cols, img_gray_left_.cols * 2));
  img_gray_back_.copyTo(
      img_gray_.rowRange(img_gray_left_.rows, img_gray_left_.rows * 2)
          .colRange(0, img_gray_left_.cols));
  img_gray_right_.copyTo(
      img_gray_.rowRange(img_gray_left_.rows, img_gray_left_.rows * 2)
          .colRange(img_gray_left_.cols, img_gray_left_.cols * 2));

  std::vector<cv::Mat> images;
  images.push_back(img_gray_left_);
  images.push_back(img_gray_front_);
  images.push_back(img_gray_right_);
  images.push_back(img_gray_back_);

  Eigen::Vector3d fake_odom(0, 0, 0);
  mCurrentMcFrame = MCFrame(images, fake_odom, timestamp, extractors_,
                            orb_vocabulary_, multicam_);
  current_frame_ = mCurrentMcFrame.frames_[FRONT_CAMERA];
  Track();
  if (map_viewer_)
    map_viewer_->Update(this);
  return mCurrentMcFrame.frames_[FRONT_CAMERA].Tcw_.clone();
}

void MultiCamTracking::Track() {

  bool bOK;
  if (state_ == OK) {
    CheckReplacedInLastMCFrame();
    if (velocity_.empty())
      bOK = TrackReferenceKeyFrameMC();
    else {
      bOK = TrackWithMotionModelMC();
      if (!bOK)
        bOK = TrackReferenceKeyFrameMC();
    }
  } else {
    bOK = RelocalizationMultiCam();
    if (!bOK)
      cout << " Relocalization Failed " << endl;
  }

  mCurrentMcFrame.reference_mckeyframe_ = mpReferenceMCKF;

  if (bOK)
    bOK = TrackLocalMapMC();
  if (bOK)
    state_ = OK;
  else
    state_ = LOST;

  if (state_ == OK) {
    if (!last_mc_frame_.tcw_.empty()) {
      cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
      cv::Mat poseLastFrame = last_mc_frame_.tcw_.clone();
      LastTwc = poseLastFrame.inv();
      velocity_ = mCurrentMcFrame.tcw_ * LastTwc; // Tcl
    } else
      velocity_ = cv::Mat();
    for (int camId = 0; camId < 4; camId++) {
      for (int i = 0; i < mCurrentMcFrame.frames_[camId].num_feature_; i++) {
        if (mCurrentMcFrame.frames_[camId].mappoints_[i] &&
            mCurrentMcFrame.frames_[camId].outliers_[i])
          mCurrentMcFrame.frames_[camId].mappoints_[i] = nullptr;
      }
    }
    last_mc_frame_ = MCFrame(mCurrentMcFrame);
  }
  if (state_ == LOST)
    return;
  if (state_ == OK) {
    if (!mCurrentMcFrame.tcw_.empty()) {
      cv::Mat Tcr = mCurrentMcFrame.tcw_ *
                    mCurrentMcFrame.reference_mckeyframe_->GetPose().inv();
      relative_frm_poses_.push_back(Tcr);
    }
  }
}

void MultiCamTracking::CheckReplacedInLastMCFrame() {
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

bool MultiCamTracking::TrackReferenceKeyFrameMC() {
  std::cout << " TrackReference MCKeyFrame" << std::endl;
  ORBmatcher matcher(0.7, true);
  int nmatches = 0;
  mCurrentMcFrame.ComputeBoW();

  for (int i = 0; i < 4; i++) {
    vector<MapPoint *> vpMapPointMatches;
    nmatches +=
        matcher.SearchByBoW(mpReferenceMCKF->keyframes_[i],
                            mCurrentMcFrame.frames_[i], vpMapPointMatches);
    mCurrentMcFrame.frames_[i].mappoints_ = vpMapPointMatches;
  }
  if (nmatches < 30)
    return false;

  mCurrentMcFrame.SetPose(last_mc_frame_.tcw_);
  mCurrentMcFrame.UpdatePose();
  const auto pose_optimizer = PoseOptimizer(4, 10);
  pose_optimizer.PoseOptimizationMC(&mCurrentMcFrame);
  int nmatchesMap = 0;
  for (int camId = 0; camId < 4; camId++) {
    for (int i = 0; i < mCurrentMcFrame.frames_[camId].num_feature_; i++) {
      if (mCurrentMcFrame.frames_[camId].mappoints_[i]) {
        if (mCurrentMcFrame.frames_[camId].outliers_[i]) {
          MapPoint *pMP = mCurrentMcFrame.frames_[camId].mappoints_[i];

          mCurrentMcFrame.frames_[camId].mappoints_[i] = nullptr;
          mCurrentMcFrame.frames_[camId].outliers_[i] = false;
          pMP->is_tracked_in_view_ = false;
          pMP->last_frame_seen_ = mCurrentMcFrame.frames_[camId].id_;
          nmatches--;
        } else if (mCurrentMcFrame.frames_[camId]
                       .mappoints_[i]
                       ->Observations() > 0)
          nmatchesMap++;
      }
    }
  }

  return nmatchesMap >= 10;
}

void MultiCamTracking::UpdateLastMCFrame() {

  MCKeyFrame *pRef = last_mc_frame_.reference_mckeyframe_;
  cv::Mat Tlr = relative_frm_poses_.back();
  last_mc_frame_.SetPose(Tlr * pRef->GetPose());
  last_mc_frame_.UpdatePose();
  return;
}

bool MultiCamTracking::TrackWithMotionModelMC() {

  ORBmatcher matcher(0.9, true);
  UpdateLastMCFrame();
  mCurrentMcFrame.SetPose(velocity_ * last_mc_frame_.tcw_);
  mCurrentMcFrame.UpdatePose();
  int th = 15;
  last_mc_frame_.UpdatePose();

  //    int nMinMatches = INT_MAX;
  int nMaxMatches = 0;
  std::vector<int> vnmatches(4, 0);
  for (int camId = 0; camId < 4; camId++) {
    fill(mCurrentMcFrame.frames_[camId].mappoints_.begin(),
         mCurrentMcFrame.frames_[camId].mappoints_.end(), nullptr);

    auto &current_frame = mCurrentMcFrame.frames_[camId];
    auto &last_frame = last_mc_frame_.frames_[camId];

    int nMatches =
        matcher.SearchByProjection(current_frame, last_frame, th, true);
    vnmatches[camId] = nMatches;
  }

  auto _max_matches = max_element(vnmatches.begin(), vnmatches.end());
  nMaxMatches = *_max_matches;

  for (int camId = 0; camId < 4; camId++) {
    if (vnmatches[camId] < 20) {
      fill(mCurrentMcFrame.frames_[camId].mappoints_.begin(),
           mCurrentMcFrame.frames_[camId].mappoints_.end(), nullptr);
      auto &current_frame = mCurrentMcFrame.frames_[camId];
      auto &last_frame = last_mc_frame_.frames_[camId];

      int nMatches =
          matcher.SearchByProjection(current_frame, last_frame, 2 * th, true);
      vnmatches[camId] = nMatches;
    }
  }

  auto _max_matches2 = max_element(vnmatches.begin(), vnmatches.end());
  nMaxMatches = *_max_matches2;

  if (nMaxMatches < 20)
    return false;
  const auto pose_optimizer = PoseOptimizer(4, 10);
  pose_optimizer.PoseOptimizationMC(&mCurrentMcFrame);
  int nmatchesMap = 0;
  for (int camId = 0; camId < 4; camId++) {
    for (int i = 0; i < mCurrentMcFrame.frames_[camId].num_feature_; i++) {
      if (mCurrentMcFrame.frames_[camId].mappoints_[i]) {
        if (mCurrentMcFrame.frames_[camId].outliers_[i]) {
          MapPoint *pMP = mCurrentMcFrame.frames_[camId].mappoints_[i];

          mCurrentMcFrame.frames_[camId].mappoints_[i] = nullptr;
          mCurrentMcFrame.frames_[camId].outliers_[i] = false;
          pMP->is_tracked_in_view_ = false;
          pMP->last_frame_seen_ = mCurrentMcFrame.frames_[camId].id_;
        } else if (mCurrentMcFrame.frames_[camId]
                       .mappoints_[i]
                       ->Observations() > 0)
          nmatchesMap++;
      }
    }
  }

  return nmatchesMap >= 20;
}

bool MultiCamTracking::TrackLocalMapMC() {
  current_frame_ = Frame(mCurrentMcFrame.frames_[FRONT_CAMERA]);
  UpdateLocalMap();
  SearchLocalPointsMC();
  const auto pose_optimizer = PoseOptimizer(4, 10);
  pose_optimizer.PoseOptimizationMC(&mCurrentMcFrame);
  num_matches_inliers_ = 0;

  for (int camId = 0; camId < 4; camId++) {
    for (int i = 0; i < mCurrentMcFrame.frames_[camId].num_feature_; i++) {
      if (mCurrentMcFrame.frames_[camId].mappoints_[i]) {
        if (!mCurrentMcFrame.frames_[camId].outliers_[i]) {
          mCurrentMcFrame.frames_[camId].mappoints_[i]->IncreaseFound();
          if (mCurrentMcFrame.frames_[camId].mappoints_[i]->Observations() > 0)
            num_matches_inliers_++;
        }
      }
    }
  }

  for (int cam_index = 0; cam_index < 4; cam_index++) {
    for (int i = 0; i < mCurrentMcFrame.frames_[cam_index].num_feature_; i++) {
      if (mCurrentMcFrame.frames_[cam_index].mappoints_[i]) {
      }
    }
  }

  if (num_matches_inliers_ < 20)
    return false;
  else {
    return true;
  }
}

void MultiCamTracking::SearchLocalPointsMC() {

  for (auto mp : local_mappoints_) {
    if (mp->isBad())
      continue;
    mp->is_tracked_in_view_ = false;
    mp->last_frame_seen_ = NONE_CAMERA;
    mp->track_cam_index1_ = NONE_CAMERA;
    mp->track_cam_index2_ = NONE_CAMERA;
  }
  for (int cam_index = 0; cam_index < 4; cam_index++) {
    for (vector<MapPoint *>::iterator
             vit = mCurrentMcFrame.frames_[cam_index].mappoints_.begin(),
             vend = mCurrentMcFrame.frames_[cam_index].mappoints_.end();
         vit != vend; vit++) {
      MapPoint *mp = *vit;
      if (mp) {
        mp->last_frame_seen_ = NONE_CAMERA;
      }
    }
  }
  for (int cam_index = 0; cam_index < 4; cam_index++) {
    for (vector<MapPoint *>::iterator
             vit = mCurrentMcFrame.frames_[cam_index].mappoints_.begin(),
             vend = mCurrentMcFrame.frames_[cam_index].mappoints_.end();
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

  auto frame_left_ = &(mCurrentMcFrame.frames_[LEFT_CAMERA]);
  auto frame_front_ = &(mCurrentMcFrame.frames_[FRONT_CAMERA]);
  auto frame_right_ = &(mCurrentMcFrame.frames_[RIGHT_CAMERA]);
  auto frame_back_ = &(mCurrentMcFrame.frames_[BACK_CAMERA]);

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
    for (int cam_index = 0; cam_index < 4; cam_index++) {
      nmatches += matcher.SearchByProjection(mCurrentMcFrame.frames_[cam_index],
                                             local_mappoints_, cam_index, th);
    }
  }
}

bool MultiCamTracking::TrackUntrackedPoints(MapPoint *mp) {
  bool is_tracked = false;

  float view_cos_limit = 0.5f;
  if (mCurrentMcFrame.frames_[BACK_CAMERA].isInFrustum(mp, view_cos_limit)) {
    mp->IncreaseVisible();
    if (!is_tracked) {
      mp->track_cam_index1_ = BACK_CAMERA;
      is_tracked = true;
    } else {
      mp->track_cam_index2_ = BACK_CAMERA;
      return true;
    }
  }
  if (mCurrentMcFrame.frames_[FRONT_CAMERA].isInFrustum(mp, view_cos_limit)) {
    mp->IncreaseVisible();
    if (!is_tracked) {
      mp->track_cam_index1_ = FRONT_CAMERA;
      is_tracked = true;
    } else {
      mp->track_cam_index2_ = FRONT_CAMERA;
      return true;
    }
  }
  if (mCurrentMcFrame.frames_[LEFT_CAMERA].isInFrustum(mp, view_cos_limit)) {
    mp->IncreaseVisible();
    if (!is_tracked) {
      mp->track_cam_index1_ = LEFT_CAMERA;
      is_tracked = true;
    } else {
      mp->track_cam_index2_ = LEFT_CAMERA;
      return true;
    }
  }
  if (mCurrentMcFrame.frames_[RIGHT_CAMERA].isInFrustum(mp, view_cos_limit)) {
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

bool MultiCamTracking::TrackUntrackedPointsSameCam(MapPoint *mp) {

  auto frame_left_ = &mCurrentMcFrame.frames_[LEFT_CAMERA];
  auto frame_front_ = &mCurrentMcFrame.frames_[FRONT_CAMERA];
  auto frame_right_ = &mCurrentMcFrame.frames_[RIGHT_CAMERA];
  auto frame_back_ = &mCurrentMcFrame.frames_[BACK_CAMERA];

  std::map<KeyFrame *, size_t> obs = mp->GetObservations();
  std::vector<int> cam_coutner(4, 0);
  for (std::map<KeyFrame *, size_t>::iterator sit = obs.begin(),
                                              send = obs.end();
       sit != send; sit++) {
    cam_coutner[sit->first->camera_index_] += 1;
  }
  int nPreCam = 0;
  int nMaxobsCamIndex = 0;
  for (int camId = 0; camId < 4; camId++) {
    if (cam_coutner[camId] > nMaxobsCamIndex) {
      nMaxobsCamIndex = cam_coutner[camId];
      nPreCam = camId;
    }
  }
  switch (nPreCam) {
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

void MultiCamTracking::SearchUnTrackedPoints(vector<MapPoint *> &mappoints) {

  for (auto mp : mappoints) {
    if (mp->isBad())
      continue;
    if (mp->last_frame_seen_ == NONE_CAMERA) {
      cv::Mat pos = mp->GetWorldPos();
      for (size_t cam_index = 0; cam_index < 4; cam_index++) {
        mp->is_tracked_in_view_ = false;
        cv::Mat Tcw = mCurrentMcFrame.frames_[cam_index].Tcw_;
        cv::Mat posC =
            Tcw.rowRange(0, 3).colRange(0, 3) * pos + Tcw.rowRange(0, 3).col(3);
        const float &pc_x = posC.at<float>(0);
        const float &pc_y = posC.at<float>(1);
        const float &pc_z = posC.at<float>(2);
        if (pc_z < 0.0f)
          continue;
        cv::Point2f reproj_point =
            Converter::ReprojectToImage(cv::Point3f(pc_x, pc_y, pc_z));

        if (reproj_point.x < 0.0f || reproj_point.x > 640.0f)
          continue;
        if (reproj_point.y < 0 || reproj_point.y > 370.0f)
          continue;
        const float max_distance = mp->GetMaxDistanceInvariance();
        const float min_distance = mp->GetMinDistanceInvariance();
        const cv::Mat po =
            pos - mCurrentMcFrame.frames_[cam_index].GetCameraCenter();
        const float dist = cv::norm(po);
        if (dist < min_distance || dist > max_distance)
          continue;
        cv::Mat Pn = mp->GetNormal();
        const float view_cos = po.dot(Pn) / dist;
        if (view_cos < 0.5f)
          continue;
        const int rredicted_level =
            mp->PredictScale(dist, &mCurrentMcFrame.frames_[cam_index]);
        mp->is_tracked_in_view_ = true;
        mp->track_proj_x1_ = reproj_point.x;
        mp->track_proj_y1_ = reproj_point.y;
        mp->track_scale_level1_ = rredicted_level;
        mp->track_view_cos1_ = view_cos;
        mp->IncreaseVisible();
        mp->track_cam_index1_ = cam_index;
        break;
      }
    }
  }
}

void MultiCamTracking::UpdateLocalMap() {

  UpdateLocalKeyFramesMC();
  UpdateLocalPoints();
  map_->SetReferenceMapPoints(local_mappoints_);
}

void MultiCamTracking::UpdateLocalPoints() {
  local_mappoints_.clear();
  for (auto keyfrm : local_keyfrms_) {

    for (auto mp : keyfrm->GetMapPointMatches()) {
      if (!mp)
        continue;
      if (mp->track_ref_mcfrm_id_ == current_frame_.id_)
        continue;
      if (!mp->isBad()) {
        local_mappoints_.push_back(mp);
        mp->track_ref_mcfrm_id_ = current_frame_.id_;
      }
    }
  }
}

void MultiCamTracking::UpdateLocalKeyFramesMC() {

  map<KeyFrame *, int> keyframeCounter;

  for (int cam_index = 0; cam_index < 4; cam_index++) {
    for (int i = 0; i < mCurrentMcFrame.frames_[cam_index].num_feature_; i++) {
      if (mCurrentMcFrame.frames_[cam_index].mappoints_[i]) {
        MapPoint *mp = mCurrentMcFrame.frames_[cam_index].mappoints_[i];
        if (!mp->isBad()) {
          const std::map<KeyFrame *, size_t> observations =
              mp->GetObservations();
          for (std::map<KeyFrame *, size_t>::const_iterator
                   it = observations.begin(),
                   itend = observations.end();
               it != itend; it++) {
            keyframeCounter[it->first]++;
          }
        } else {
          mCurrentMcFrame.frames_[cam_index].mappoints_[i] = nullptr;
        }
      }
    }
  }

  if (keyframeCounter.empty())
    return;

  int max = 0;
  KeyFrame *keyfrm_max = nullptr;
  local_keyfrms_.clear();
  local_keyfrms_.reserve(3 * keyframeCounter.size());

  cv::Mat front_cam_center =
      mCurrentMcFrame.frames_[FRONT_CAMERA].GetCameraCenter();
  cv::Mat back_cam_center =
      mCurrentMcFrame.frames_[BACK_CAMERA].GetCameraCenter();
  cv::Mat vehicle_center = (front_cam_center + back_cam_center) / 2;

  for (std::map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(),
                                                 itEnd = keyframeCounter.end();
       it != itEnd; it++) {
    KeyFrame *keyfrm = it->first;

    if (keyfrm->isBad())
      continue;

    cv::Mat cam_center = keyfrm->GetCameraCenter();
    float dis = cv::norm(cam_center - vehicle_center);
    if (dis > keyfrm_dist_thr_)
      continue;

    if (it->second > max) {
      max = it->second;
      keyfrm_max = keyfrm;
    }

    local_keyfrms_.push_back(it->first);
    keyfrm->track_reffrm_id_ = current_frame_.id_;
  }

  for (vector<KeyFrame *>::const_iterator keyfrm_it = local_keyfrms_.begin(),
                                          keyfrm_it_end = local_keyfrms_.end();
       keyfrm_it != keyfrm_it_end; keyfrm_it++) {
    if (local_keyfrms_.size() > 150)
      break;

    KeyFrame *keyfrm = *keyfrm_it;

    const std::vector<KeyFrame *> ngh_keyfrms =
        keyfrm->GetBestCovisibilityKeyFrames(10);
    for (std::vector<KeyFrame *>::const_iterator
             itNeighKF = ngh_keyfrms.begin(),
             itEndNeighKF = ngh_keyfrms.end();
         itNeighKF != itEndNeighKF; itNeighKF++) {
      KeyFrame *ngh_keyfrm = *itNeighKF;

      if (!ngh_keyfrm->isBad()) {
        if (ngh_keyfrm->track_reffrm_id_ != current_frame_.id_) {
          local_keyfrms_.push_back(ngh_keyfrm);
          ngh_keyfrm->track_reffrm_id_ = current_frame_.id_;
          break;
        }
      }
    }
    const std::set<KeyFrame *> child_keyfrms = keyfrm->GetChilds();
    for (std::set<KeyFrame *>::const_iterator sit = child_keyfrms.begin(),
                                              send = child_keyfrms.end();
         sit != send; sit++) {
      KeyFrame *child_keyfrm = *sit;

      if (!child_keyfrm->isBad()) {
        if (child_keyfrm->track_reffrm_id_ != current_frame_.id_) {
          local_keyfrms_.push_back(child_keyfrm);
          child_keyfrm->track_reffrm_id_ = current_frame_.id_;
          break;
        }
      }
    }
    KeyFrame *parent_keyfrm = keyfrm->GetParent();
    if (parent_keyfrm) {

      if (parent_keyfrm->track_reffrm_id_ != current_frame_.id_) {
        local_keyfrms_.push_back(parent_keyfrm);
        parent_keyfrm->track_reffrm_id_ = current_frame_.id_;
        break;
      }
    }
  }

  if (keyfrm_max) {

    MCKeyFrame *mc_keyfrm = keyfrm_max->GetMCKeyFrame();
    mpReferenceMCKF = mc_keyfrm;
  }
}
bool MultiCamTracking::Relocalization_panoramic() {
  Frame *frame = &current_frame_;
  frame->ComputeBoW();
  vector<KeyFrame *> camdidate_keyfrms =
      keyfrm_database_->DetectRelocalizationCandidates(frame);
  if (camdidate_keyfrms.empty())
    return false;
  const int nKFs = camdidate_keyfrms.size();
  ORBmatcher matcher(0.8, true);
  vector<opengv::bearingVectors_t> matchedBearingVecs(nKFs);
  vector<opengv::points_t> points3D(nKFs);
  opengv::translations_t camOffsets;
  opengv::rotations_t camRotations;
  camOffsets.push_back(Eigen::Vector3d::Zero());
  camRotations.push_back(Eigen::Matrix3d::Identity());

  vector<vector<int>> mvKeyPointIndices(nKFs, vector<int>());
  vector<vector<MapPoint *>> vvpMapPointMatches;
  vvpMapPointMatches.resize(nKFs);

  vector<bool> vbDiscarded;
  vbDiscarded.resize(nKFs);
  int nCandidates = 0;

  for (size_t i = 0; i < camdidate_keyfrms.size(); ++i) {
    KeyFrame *pKF = camdidate_keyfrms[i];
    if (pKF->isBad())
      vbDiscarded[i] = true;
    else if (pKF->camera_index_ != FRONT_CAMERA) {
      vbDiscarded[i] = true;
    } else {
      int nmatches = matcher.SearchByBoW(pKF, *frame, vvpMapPointMatches[i]);
      if (nmatches < 15) {
        vbDiscarded[i] = true;
        continue;
      } else {
        opengv::bearingVectors_t mvP2D;
        opengv::points_t mvP3Dw;
        int idx = 0;
        for (size_t j = 0, iend = vvpMapPointMatches[i].size(); j < iend; ++j) {
          MapPoint *pMP = vvpMapPointMatches[i][j];

          if (pMP) {
            if (!pMP->isBad()) {
              auto bearing = Converter::KeypointToBearing(frame->keypts_[j]);
              mvP2D.push_back(bearing);
              cv::Vec3d Pos = pMP->GetWorldPos();
              mvP3Dw.push_back(opengv::point_t(Pos(0), Pos(1), Pos(2)));
              mvKeyPointIndices[i].push_back(j);
              ++idx;
            }
          }
        }
        matchedBearingVecs[i] = mvP2D;
        points3D[i] = mvP3Dw;
        ++nCandidates;
      }
    }
  }

  bool bMatch = false;
  for (size_t i = 0; i < camdidate_keyfrms.size(); i++) {
    if (vbDiscarded[i])
      continue;
    vector<int> inliers;
    opengv::absolute_pose::CentralAbsoluteAdapter adapter(matchedBearingVecs[i],
                                                          points3D[i]);
    sac::Ransac<sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
    std::shared_ptr<sac_problems::absolute_pose::AbsolutePoseSacProblem>
        absposeproblem_ptr(
            new sac_problems::absolute_pose::AbsolutePoseSacProblem(
                adapter,
                sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP));

    ransac.sac_model_ = absposeproblem_ptr;
    ransac.threshold_ = 0.02;
    ransac.max_iterations_ = 200;
    ransac.computeModel();
    inliers = ransac.inliers_;
    opengv::transformation_t trafo = ransac.model_coefficients_;

    // If Ransac reaches max. iterations discard keyframe
    if (ransac.iterations_ >= ransac.max_iterations_) {
      vbDiscarded[i] = true;
      --nCandidates;
    }

    else {
      cv::Mat trafoOut = Converter::ogv2ocv(trafo);
      frame->SetPose(trafoOut);
      set<MapPoint *> sFound;
      for (size_t ii = 0; ii < frame->mappoints_.size(); ++ii)
        frame->mappoints_[ii] = nullptr;
      for (size_t j = 0; j < inliers.size(); ++j) {
        frame->mappoints_[mvKeyPointIndices[i][inliers[j]]] =
            vvpMapPointMatches[i][mvKeyPointIndices[i][inliers[j]]];
        sFound.insert(vvpMapPointMatches[i][mvKeyPointIndices[i][inliers[j]]]);
      }

      auto pose_optimizer = PoseOptimizer(4, 10);
      int nGood = pose_optimizer.PoseOptimization(frame);
      if (nGood < 10)
        continue;
      for (size_t io = 0, ioend = frame->outliers_.size(); io < ioend; ++io)
        if (frame->outliers_[io])
          frame->mappoints_[io] = nullptr;
      if (nGood >= 10) {
        bMatch = true;
        mpReferenceMCKF = camdidate_keyfrms[i]->GetMCKeyFrame();
        break;
      }
    }
  }

  if (!bMatch) {
    return false;

  } else {
    last_reloc_frm_id_ = current_frame_.id_;
    std::cout << kColorGreen << "  ######  Relocalization success ##### "
              << kColorReset << std::endl;
    UpdateMCFrame(*frame);
    return true;
  }
}

bool MultiCamTracking::RelocalizationMultiCam() {
  Frame *frame_left = &mCurrentMcFrame.frames_[LEFT_CAMERA];
  Frame *frame_front = &mCurrentMcFrame.frames_[FRONT_CAMERA];
  Frame *frame_right = &mCurrentMcFrame.frames_[RIGHT_CAMERA];
  Frame *frame_back = &mCurrentMcFrame.frames_[BACK_CAMERA];

  frame_left->ComputeBoW();
  frame_front->ComputeBoW();
  frame_right->ComputeBoW();
  frame_back->ComputeBoW();

  std::vector<KeyFrame *> candidate_keyfrms =
      keyfrm_database_->DetectRelocalizationCandidates(
          frame_front); // use front camera to detect relocalization candidates

  if (candidate_keyfrms.size() == 0)
    return false;
  const unsigned long num_keyfrms = candidate_keyfrms.size();
  ORBmatcher matcher(0.8f, true);

  vector<opengv::bearingVectors_t> matchedBearingVecs(num_keyfrms);
  vector<opengv::points_t> points3D(num_keyfrms);
  opengv::translations_t camOffsets;
  opengv::rotations_t camRotations;

  for (int cam_index = 0; cam_index < 4; cam_index++) {
    cv::Mat cam_ext_inv = multicam_->cam_extrinsic_[cam_index].inv();
    Eigen::Matrix3d rotTemp;
    opengv::translation_t t;
    cv::cv2eigen<double>(cam_ext_inv.rowRange(0, 3).colRange(0, 3), rotTemp);
    cv::cv2eigen(cam_ext_inv.rowRange(0, 3).col(3), t);
    camOffsets.push_back(t);
    camRotations.push_back(rotTemp);
  }

  std::vector<std::vector<int>> camCorrespondences(num_keyfrms);
  vector<vector<int>> mvKeyPointIndices0(num_keyfrms, std::vector<int>());
  vector<vector<int>> mvKeyPointIndices1(num_keyfrms, std::vector<int>());
  vector<vector<int>> mvKeyPointIndices2(num_keyfrms, std::vector<int>());
  vector<vector<int>> mvKeyPointIndices3(num_keyfrms, std::vector<int>());
  vector<int> n1(num_keyfrms, 0);
  vector<int> n2(num_keyfrms, 0);
  vector<int> n3(num_keyfrms, 0);

  vector<vector<MapPoint *>> vvpMapPointMatches0(num_keyfrms,
                                                 std::vector<MapPoint *>());
  vector<vector<MapPoint *>> vvpMapPointMatches1(num_keyfrms,
                                                 std::vector<MapPoint *>());
  vector<vector<MapPoint *>> vvpMapPointMatches2(num_keyfrms,
                                                 std::vector<MapPoint *>());
  vector<vector<MapPoint *>> vvpMapPointMatches3(num_keyfrms,
                                                 std::vector<MapPoint *>());

  vector<bool> discard_list;
  discard_list.resize(num_keyfrms);

  for (size_t i = 0; i < candidate_keyfrms.size(); ++i) {
    KeyFrame *front_keyfrm = candidate_keyfrms[i];
    if (!front_keyfrm->GetMCKeyFrame() || front_keyfrm->isBad() ||
        front_keyfrm->camera_index_ != FRONT_CAMERA) {
      discard_list[i] = true;
      continue;
    }

    // tag
    KeyFrame *left_keyfrm =
        front_keyfrm->GetMCKeyFrame()->keyframes_[LEFT_CAMERA];
    KeyFrame *right_keyfrm =
        front_keyfrm->GetMCKeyFrame()->keyframes_[RIGHT_CAMERA];
    KeyFrame *back_keyfrm =
        front_keyfrm->GetMCKeyFrame()->keyframes_[BACK_CAMERA];

    if (left_keyfrm->isBad() || right_keyfrm->isBad() || back_keyfrm->isBad()) {
      discard_list[i] = true;
      continue;
    }
    int num_matches =
        matcher.SearchByBoW(left_keyfrm, *frame_left, vvpMapPointMatches0[i]);
    num_matches +=
        matcher.SearchByBoW(front_keyfrm, *frame_front, vvpMapPointMatches1[i]);
    num_matches +=
        matcher.SearchByBoW(right_keyfrm, *frame_right, vvpMapPointMatches2[i]);
    num_matches +=
        matcher.SearchByBoW(back_keyfrm, *frame_back, vvpMapPointMatches3[i]);

    if (num_matches < 30) {
      discard_list[i] = true;
      continue;
    }
    opengv::bearingVectors_t mvP2D;
    opengv::points_t mvP3Dw;
    //  setup 2D-3D features cooresponds
    for (size_t j = 0; j < vvpMapPointMatches0[i].size(); j++) {
      MapPoint *mp = vvpMapPointMatches0[i][j];
      if (mp) {
        if (!mp->isBad()) {

          auto bearing = Converter::KeypointToBearing(frame_left->keypts_[j]);
          mvP2D.push_back(bearing);
          cv::Vec3d Pos = mp->GetWorldPos();
          mvP3Dw.push_back(opengv::point_t(Pos(0), Pos(1), Pos(2)));
          mvKeyPointIndices0[i].push_back(j);
          camCorrespondences[i].push_back(LEFT_CAMERA);
        }
      }
    }
    n1[i] = mvKeyPointIndices0[i].size();

    for (size_t j = 0; j < vvpMapPointMatches1[i].size(); j++) {
      MapPoint *pMP = vvpMapPointMatches1[i][j];
      if (pMP) {
        if (!pMP->isBad()) {
          auto bearing = Converter::KeypointToBearing(frame_front->keypts_[j]);
          mvP2D.push_back(bearing);
          cv::Vec3d Pos = pMP->GetWorldPos();
          mvP3Dw.push_back(opengv::point_t(Pos(0), Pos(1), Pos(2)));
          mvKeyPointIndices1[i].push_back(j);
          camCorrespondences[i].push_back(FRONT_CAMERA);
        }
      }
    }

    n2[i] = mvKeyPointIndices1[i].size() + n1[i];

    for (size_t j = 0; j < vvpMapPointMatches2[i].size(); j++) {
      MapPoint *pMP = vvpMapPointMatches2[i][j];
      if (pMP) {
        if (!pMP->isBad()) {
          auto bearing = Converter::KeypointToBearing(frame_right->keypts_[j]);
          mvP2D.push_back(bearing);
          cv::Vec3d Pos = pMP->GetWorldPos();
          mvP3Dw.push_back(opengv::point_t(Pos(0), Pos(1), Pos(2)));
          mvKeyPointIndices2[i].push_back(j);
          camCorrespondences[i].push_back(RIGHT_CAMERA);
        }
      }
    }

    n3[i] = mvKeyPointIndices2[i].size() + n2[i];
    for (size_t j = 0; j < vvpMapPointMatches3[i].size(); j++) {
      MapPoint *pMP = vvpMapPointMatches3[i][j];
      if (pMP) {
        if (!pMP->isBad()) {
          auto bearing = Converter::KeypointToBearing(frame_back->keypts_[j]);
          mvP2D.push_back(bearing);
          cv::Vec3d Pos = pMP->GetWorldPos();
          mvP3Dw.push_back(opengv::point_t(Pos(0), Pos(1), Pos(2)));
          mvKeyPointIndices3[i].push_back(j);
          camCorrespondences[i].push_back(BACK_CAMERA);
        }
      }
    }
    matchedBearingVecs[i] = mvP2D;
    points3D[i] = mvP3Dw;
  }
  bool is_match = false;
  for (size_t i = 0; i < candidate_keyfrms.size(); i++) {
    if (discard_list[i])
      continue;
    vector<int> inliers;
    opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(
        matchedBearingVecs[i], camCorrespondences[i], points3D[i], camOffsets,
        camRotations);
    opengv::sac::Ransac<
        opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
        ransac;
    std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
        absposeproblem_ptr(
            new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
                adapter, opengv::sac_problems::absolute_pose::
                             AbsolutePoseSacProblem::GP3P));
    ransac.sac_model_ = absposeproblem_ptr;
    ransac.threshold_ = 0.02;
    ransac.max_iterations_ = 200;

    ransac.computeModel();
    inliers = ransac.inliers_;
    opengv::transformation_t trafo = ransac.model_coefficients_;
    // If Ransac reaches max. iterations discard keyframe
    if (ransac.iterations_ >= ransac.max_iterations_) {
      discard_list[i] = true;
    } else {

      //      trafo = opengv::absolute_pose::gpnp(adapter, ransac.inliers_);
      cv::Mat trafoOut = Converter::ogv2ocv(trafo);
      frame_front->SetPose(trafoOut);
      mCurrentMcFrame.SetPose(trafoOut);

      for (size_t ii = 0; ii < frame_left->mappoints_.size(); ++ii)
        frame_left->mappoints_[ii] = nullptr;
      for (size_t ii = 0; ii < frame_front->mappoints_.size(); ++ii)
        frame_front->mappoints_[ii] = nullptr;
      for (size_t ii = 0; ii < frame_right->mappoints_.size(); ++ii)
        frame_right->mappoints_[ii] = nullptr;
      for (size_t ii = 0; ii < frame_back->mappoints_.size(); ++ii)
        frame_back->mappoints_[ii] = nullptr;

      for (size_t j = 0; j < inliers.size(); ++j) {
        if (inliers[j] < n1[i]) {
          const int idx = mvKeyPointIndices0[i][inliers[j]];
          frame_left->mappoints_[idx] = vvpMapPointMatches0[i][idx];
        } else if (inliers[j] >= n1[i] && inliers[j] < n2[i]) {
          const int idx = mvKeyPointIndices1[i][inliers[j] - n1[i]];
          frame_front->mappoints_[idx] = vvpMapPointMatches1[i][idx];

        } else if (inliers[j] >= n2[i] && inliers[j] < n3[i]) {
          const int idx = mvKeyPointIndices2[i][inliers[j] - n2[i]];
          frame_right->mappoints_[idx] = vvpMapPointMatches2[i][idx];
        } else if (inliers[j] >= n3[i]) {
          const int idx = mvKeyPointIndices3[i][inliers[j] - n3[i]];
          frame_back->mappoints_[idx] = vvpMapPointMatches3[i][idx];
        }
      }

      const auto pose_optimizer = PoseOptimizer(4, 10);
      int num_inliers = pose_optimizer.PoseOptimizationMC(&mCurrentMcFrame);
      if (num_inliers < 30)
        continue;

      for (int cam_index = 0; cam_index < 4; cam_index++) {
        for (size_t io = 0,
                    ioend = mCurrentMcFrame.frames_[cam_index].outliers_.size();
             io < ioend; ++io)
          if (mCurrentMcFrame.frames_[cam_index].outliers_[io])
            mCurrentMcFrame.frames_[cam_index].mappoints_[io] = nullptr;
      }
      if (num_inliers >= 30) {
        is_match = true;
        mpReferenceMCKF = candidate_keyfrms[i]->GetMCKeyFrame();

        std::cout << kColorGreen << " Found matches :" << num_inliers
                  << kColorReset << std::endl;
        break;
      }
    }
  }
  if (!is_match) {
    return false;

  } else {
    last_reloc_frm_id_ = frame_front->id_;
    std::cout << kColorGreen << " Relocalization success" << kColorReset
              << std::endl;
    mCurrentMcFrame.tcw_ = frame_front->Tcw_;
    mCurrentMcFrame.UpdatePose();
    return true;
  }
}

void MultiCamTracking::UpdateMCFrame(Frame &f) {
  mCurrentMcFrame.frames_[FRONT_CAMERA] = Frame(f);
  mCurrentMcFrame.tcw_ = f.Tcw_;
  mCurrentMcFrame.UpdatePose();
}

void MultiCamTracking::SetLost() { state_ = LOST; }
}
