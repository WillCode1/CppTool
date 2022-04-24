#include "mckeyframe.h"
#include "converter.h"
#include "keyframe.h"
#include "orbmatcher.h"
#include <mutex>

namespace FeatureSLAM {

long unsigned int MCKeyFrame::next_id_ = 0;

MCKeyFrame::MCKeyFrame(KeyFrame *pKFLeft, KeyFrame *pKFFront,
                       KeyFrame *pKFRight, KeyFrame *pKFBack,
                       MultiCam *pMulticam)
    : frame_id_(0), local_ba_for_keyfrm_(-1), ba_fixed_for_keyfrm_(-1),
      mc_ready_(true), multicam_(pMulticam) {

  id_ = next_id_++;

  if (keyframes_.size() != 4) {
    keyframes_.clear();
    keyframes_.resize(4);
  }
  fill(keyframes_.begin(), keyframes_.end(), static_cast<KeyFrame *>(NULL));
  cam_exts_.clear();
  cam_exts_.resize(4);
  for (int i = 0; i < 4; i++) {
    cam_exts_[i] = multicam_->cam_extrinsic_[i];
  }
  pKFLeft->SetMCConnection(this);
  pKFFront->SetMCConnection(this);
  pKFRight->SetMCConnection(this);
  pKFBack->SetMCConnection(this);
  pKFLeft->SetCameraIndex(LEFT_CAMERA);
  pKFFront->SetCameraIndex(FRONT_CAMERA);
  pKFRight->SetCameraIndex(RIGHT_CAMERA);
  pKFBack->SetCameraIndex(BACK_CAMERA);

  keyframes_[LEFT_CAMERA] = pKFLeft;
  keyframes_[FRONT_CAMERA] = pKFFront;
  keyframes_[RIGHT_CAMERA] = pKFRight;
  keyframes_[BACK_CAMERA] = pKFBack;
}

MCKeyFrame::MCKeyFrame(MCFrame *mc_frame, Map *map,
                       KeyFrameDatabase *keyframe_database,
                       bool scale_confirmed)
    : frame_id_(mc_frame->id_), timestamp_(mc_frame->timestamp_),
      local_ba_for_keyfrm_(0), ba_fixed_for_keyfrm_(0),
      mc_ready_(scale_confirmed), multicam_(mc_frame->multicam_),
      odometry_vector_(mc_frame->odom_vector_) {

  id_ = next_id_++;

  if (keyframes_.size() != 4)
    keyframes_.resize(4);
  fill(keyframes_.begin(), keyframes_.end(), nullptr);

  if (!mc_ready_) {
    cv::Mat Tc1w = mc_frame->frames_[FRONT_CAMERA].Tcw_;
    cv::Mat Tc2w = mc_frame->frames_[RIGHT_CAMERA].Tcw_;

    if (Tc1w.empty() || Tc2w.empty()) {
      cout << " fatal error  in MCKeyframe consturctor " << endl;
    }

    cam_exts_.resize(4);
    cam_exts_[LEFT_CAMERA] = mc_frame->cam_ext_[LEFT_CAMERA];
    cam_exts_[FRONT_CAMERA] = mc_frame->cam_ext_[FRONT_CAMERA];
    cam_exts_[RIGHT_CAMERA] = mc_frame->cam_ext_[RIGHT_CAMERA];
    cam_exts_[BACK_CAMERA] = mc_frame->cam_ext_[BACK_CAMERA];

    mc_frame->frames_[LEFT_CAMERA].SetPose(mc_frame->cam_ext_[LEFT_CAMERA]);
    mc_frame->frames_[BACK_CAMERA].SetPose(mc_frame->cam_ext_[BACK_CAMERA]);

    keyframes_[LEFT_CAMERA] =
        new KeyFrame(mc_frame->frames_[LEFT_CAMERA], map, keyframe_database);
    keyframes_[FRONT_CAMERA] =
        new KeyFrame(mc_frame->frames_[FRONT_CAMERA], map, keyframe_database);
    keyframes_[RIGHT_CAMERA] =
        new KeyFrame(mc_frame->frames_[RIGHT_CAMERA], map, keyframe_database);
    keyframes_[BACK_CAMERA] =
        new KeyFrame(mc_frame->frames_[BACK_CAMERA], map, keyframe_database);
    tcw_ = mc_frame->frames_[FRONT_CAMERA].Tcw_.clone();
  } else {
    cv::Mat FramePose = mc_frame->tcw_;
    if (FramePose.empty()) {
      cout << " fatal error in MCKeyframe consturctor " << endl;
    }
    cam_exts_.resize(4);
    cam_exts_[LEFT_CAMERA] = mc_frame->cam_ext_[LEFT_CAMERA];
    cam_exts_[FRONT_CAMERA] = mc_frame->cam_ext_[FRONT_CAMERA];
    cam_exts_[RIGHT_CAMERA] = mc_frame->cam_ext_[RIGHT_CAMERA];
    cam_exts_[BACK_CAMERA] = mc_frame->cam_ext_[BACK_CAMERA];

    mc_frame->frames_[LEFT_CAMERA].SetPose(mc_frame->cam_ext_[LEFT_CAMERA] *
                                           FramePose);
    mc_frame->frames_[FRONT_CAMERA].SetPose(mc_frame->cam_ext_[FRONT_CAMERA] *
                                            FramePose);
    mc_frame->frames_[RIGHT_CAMERA].SetPose(mc_frame->cam_ext_[RIGHT_CAMERA] *
                                            FramePose);
    mc_frame->frames_[BACK_CAMERA].SetPose(mc_frame->cam_ext_[BACK_CAMERA] *
                                           FramePose);
    keyframes_[LEFT_CAMERA] =
        new KeyFrame(mc_frame->frames_[LEFT_CAMERA], map, keyframe_database);
    keyframes_[FRONT_CAMERA] =
        new KeyFrame(mc_frame->frames_[FRONT_CAMERA], map, keyframe_database);
    keyframes_[RIGHT_CAMERA] =
        new KeyFrame(mc_frame->frames_[RIGHT_CAMERA], map, keyframe_database);
    keyframes_[BACK_CAMERA] =
        new KeyFrame(mc_frame->frames_[BACK_CAMERA], map, keyframe_database);
    tcw_ = mc_frame->frames_[FRONT_CAMERA].Tcw_.clone();
  }

  for (int cam_index = 0; cam_index < 4; cam_index++) {
    keyframes_[cam_index]->SetMCConnection(this);
    keyframes_[cam_index]->SetCameraIndex(cam_index);
  }
}
void MCKeyFrame::SetPose(const cv::Mat &Tcw_) {

  for (int i = 0; i < 4; i++) {
    if (!keyframes_[i])
      std::cerr << "fatal error " << std::endl;
    keyframes_[i]->SetPose(cam_exts_[i] * Tcw_);
  }
}

void MCKeyFrame::SetKeyFrame(KeyFrame *pKF, int Camera_index) {
  if (keyframes_.size() != 4) {
    cout << " !!!!!!!!!!!!!!! warning !!  you should Init MCKeyframe before "
            "set  keyframe"
         << endl;
    keyframes_.resize(4);
  }
  keyframes_[Camera_index] = pKF;
}

void MCKeyFrame::UpdateScale(const float scale) {
  if (keyframes_.size() != 4) {
    cout << " fatal error  , MCKfs size !=4 " << endl;
    return;
  }

  cout << " MCKeyframe update scale and calculate the left and back camera "
          "poses "
       << endl;
  KeyFrame *pkf = keyframes_[RIGHT_CAMERA];
  cv::Mat pose = pkf->GetPose();
  Converter::UpdatePoseScale(pose, scale);
  pkf->SetPose(pose);

  cv::Mat poseFront = keyframes_[FRONT_CAMERA]->GetPose();
  keyframes_[LEFT_CAMERA]->SetPose(cam_exts_[LEFT_CAMERA] * poseFront);
  keyframes_[RIGHT_CAMERA]->SetPose(cam_exts_[RIGHT_CAMERA] * poseFront);
  unique_lock<mutex> lock(mutex_pose_);
  tcw_ = poseFront;
}

void MCKeyFrame::SetState(bool state) {
  std::unique_lock<mutex> lock(mutex_state_);
  mc_ready_ = state;
}

int MCKeyFrame::TrackedMapPoints(const int &minObs,
                                 std::vector<int> &vnTrackedMapPoints) {

  vnTrackedMapPoints = vector<int>(4, 0);
  int nTrackedMapPoints = 0;
  int nTrackedMapPoints_ = 0;
  for (int i = 0; i < 4; i++) {
    if (!keyframes_[i]) {
      cout << " fatal error  ,KeyFrame pointer is Null" << endl;
      return 0;
    }
    nTrackedMapPoints_ = keyframes_[i]->TrackedMapPoints(minObs);
    vnTrackedMapPoints[i] = nTrackedMapPoints;
    nTrackedMapPoints += nTrackedMapPoints_;
  }

  return nTrackedMapPoints;
}

Eigen::Vector3d MCKeyFrame::GetOdometry() { return odometry_vector_; }

void MCKeyFrame::SetOdometry(const Eigen::Vector3d &odometry_vector) {
  odometry_vector_ = odometry_vector;
}

std::vector<MCKeyFrame *> MCKeyFrame::GetVectorCovisibleKeyFrames() {
  std::set<MCKeyFrame *> covis_mc_keyfrms;

  for (int cam_index = 0; cam_index < 4; cam_index++) {
    const std::vector<KeyFrame *> covis_keyfrms =
        keyframes_[cam_index]->GetVectorCovisibleKeyFrames();

    for (auto keyfrm : covis_keyfrms) {
      auto mckeyfrm = keyfrm->GetMCKeyFrame();
      if (mckeyfrm && covis_mc_keyfrms.count(mckeyfrm) == 0) {
        covis_mc_keyfrms.insert(mckeyfrm);
      }
    }
  }
  return std::vector<MCKeyFrame *>(covis_mc_keyfrms.begin(),
                                   covis_mc_keyfrms.end());
}

void MCKeyFrame::ComputeBoW() {
  for (auto keyframe : keyframes_) {
    if (keyframe) {
      keyframe->ComputeBoW();
    }
  }
}

cv::Mat MCKeyFrame::GetPose() {
  unique_lock<mutex> lock(mutex_pose_);
  tcw_ = keyframes_[FRONT_CAMERA]->GetPose();
  return tcw_;
}
bool MCKeyFrame::GetState() {
  std::unique_lock<mutex> lock(mutex_state_);
  return mc_ready_;
}
}
