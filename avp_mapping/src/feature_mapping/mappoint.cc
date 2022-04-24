#include "mappoint.h"
#include "orbmatcher.h"
#include <mutex>

namespace FeatureSLAM {

long unsigned int MapPoint::next_id_ = 0;

MapPoint::MapPoint(const cv::Mat &pos, int first_mc_keyfrm_id, Map *map)
    : is_mature_(false), first_mc_keyfrm_id_(first_mc_keyfrm_id), num_obs_(0),
      track_ref_mcfrm_id_(0), last_frame_seen_(0), local_ba_keyfrm_id_(0),
      fuse_candidate_keyfrm_id_(0), track_cam_index1_(-1),
      track_cam_index2_(-1), ref_keyfrm_(nullptr), num_visible_(1),
      num_found_(1), is_bad_(false), replaced_point_(nullptr), min_distance_(0),
      max_distance_(0), map_(map) {
  pos.copyTo(world_pos_);
  normal_vector_ = cv::Mat::zeros(3, 1, CV_32F);
  unique_lock<mutex> lock(map_->mutex_point_creation_);
  id_ = next_id_++;
}

MapPoint::MapPoint(const cv::Mat &pos, KeyFrame *ref_keyfrm, Map *map)
    : is_mature_(false), first_mc_keyfrm_id_(ref_keyfrm->GetMCKeyFrame()->id_),
      num_obs_(0), track_ref_mcfrm_id_(0), last_frame_seen_(0),
      local_ba_keyfrm_id_(0), fuse_candidate_keyfrm_id_(0),
      track_cam_index1_(-1), track_cam_index2_(-1), ref_keyfrm_(ref_keyfrm),
      num_visible_(1), num_found_(1), is_bad_(false), replaced_point_(nullptr),
      min_distance_(0), max_distance_(0), map_(map) {
  pos.copyTo(world_pos_);
  normal_vector_ = cv::Mat::zeros(3, 1, CV_32F);

  // MapPoints can be created from Tracking and Local Mapping. This mutex avoid
  // conflicts with id.
  unique_lock<mutex> lock(map_->mutex_point_creation_);
  id_ = next_id_++;
}

MapPoint::MapPoint(const cv::Mat &pos, Map *map, Frame *frame,
                   const int &index_in_frame)
    : is_mature_(false), first_mc_keyfrm_id_(-1), num_obs_(0),
      track_ref_mcfrm_id_(0), last_frame_seen_(0), local_ba_keyfrm_id_(0),
      fuse_candidate_keyfrm_id_(0), track_cam_index1_(-1),
      track_cam_index2_(-1), ref_keyfrm_(nullptr), num_visible_(1),
      num_found_(1), is_bad_(false), replaced_point_(nullptr), map_(map)

{
  pos.copyTo(world_pos_);
  cv::Mat camera_center = frame->GetCameraCenter();
  normal_vector_ = world_pos_ - camera_center;
  normal_vector_ = normal_vector_ / cv::norm(normal_vector_);

  cv::Mat pc_vector = pos - camera_center;
  const float dist = cv::norm(pc_vector);
  const int level = frame->keypts_[index_in_frame].octave;
  const float level_scale_factor = frame->scale_factors_[level];
  const int num_levels = frame->scale_levels_;

  max_distance_ = dist * level_scale_factor;
  min_distance_ = max_distance_ / frame->scale_factors_[num_levels - 1];

  frame->descriptors_.row(index_in_frame).copyTo(descriptor_);

  unique_lock<mutex> lock(map_->mutex_point_creation_);
  id_ = next_id_++;
}

void MapPoint::SetWorldPos(const cv::Mat &pos) {
  unique_lock<mutex> lock(mutex_pos_);
  pos.copyTo(world_pos_);
}

cv::Mat MapPoint::GetWorldPos() {
  unique_lock<mutex> lock(mutex_pos_);
  return world_pos_.clone();
}
void MapPoint::SetReferenceKeyFrame(KeyFrame *ref_keyfrm) {
  ref_keyfrm_ = ref_keyfrm;
}

cv::Mat MapPoint::GetNormal() {
  unique_lock<mutex> lock(mutex_pos_);
  return normal_vector_.clone();
}

KeyFrame *MapPoint::GetReferenceKeyFrame() {
  unique_lock<mutex> lock(mutex_features_);
  return ref_keyfrm_;
}

void MapPoint::AddObservation(KeyFrame *keyfrm, size_t idx) {
  unique_lock<mutex> lock(mutex_features_);
  if (observations_.count(keyfrm))
    return;
  observations_[keyfrm] = idx;
  num_obs_++;
}

void MapPoint::EraseObservation(KeyFrame *keyfrm) {
  bool is_bad = false;
  {
    unique_lock<mutex> lock(mutex_features_);
    if (observations_.count(keyfrm)) {
      num_obs_--;
      observations_.erase(keyfrm);

      if (ref_keyfrm_ == keyfrm)
        ref_keyfrm_ = observations_.begin()->first;

      if (num_obs_ <= 2)
        is_bad = true;
    }
  }

  if (is_bad)
    SetBadFlag();
}

map<KeyFrame *, size_t> MapPoint::GetObservations() {
  unique_lock<mutex> lock(mutex_features_);
  return observations_;
}

int MapPoint::Observations() {
  unique_lock<mutex> lock(mutex_features_);
  return num_obs_;
}

void MapPoint::SetBadFlag() {
  map<KeyFrame *, size_t> obs;
  {
    unique_lock<mutex> lock1(mutex_features_);
    unique_lock<mutex> lock2(mutex_pos_);
    is_bad_ = true;
    obs = observations_;
    observations_.clear();
  }
  for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end();
       mit != mend; mit++) {
    KeyFrame *keyfrm = mit->first;
    keyfrm->EraseMapPointMatch(mit->second);
  }

  map_->EraseMapPoint(this);
}

MapPoint *MapPoint::GetReplaced() {
  unique_lock<mutex> lock1(mutex_features_);
  unique_lock<mutex> lock2(mutex_pos_);
  return replaced_point_;
}

void MapPoint::Replace(MapPoint *mp) {
  if (mp->id_ == this->id_)
    return;

  int num_visible, num_found;
  map<KeyFrame *, size_t> obs;
  {
    unique_lock<mutex> lock1(mutex_features_);
    unique_lock<mutex> lock2(mutex_pos_);
    obs = observations_;
    observations_.clear();
    is_bad_ = true;
    num_visible = num_visible_;
    num_found = num_found_;
    replaced_point_ = mp;
  }

  for (map<KeyFrame *, size_t>::iterator mit = obs.begin(), mend = obs.end();
       mit != mend; mit++) {
    // Replace measurement in keyframe
    KeyFrame *keyfrm = mit->first;

    if (!mp->IsInKeyFrame(keyfrm)) {
      keyfrm->ReplaceMapPointMatch(mit->second, mp);
      mp->AddObservation(keyfrm, mit->second);
    } else {

      keyfrm->EraseMapPointMatch(mit->second);
    }
  }
  mp->IncreaseFound(num_found);
  mp->IncreaseVisible(num_visible);
  mp->ComputeDistinctiveDescriptors();

  map_->EraseMapPoint(this);
}

bool MapPoint::isBad() {
  unique_lock<mutex> lock(mutex_features_);
  unique_lock<mutex> lock2(mutex_pos_);
  return is_bad_;
}

void MapPoint::IncreaseVisible(int n) {
  unique_lock<mutex> lock(mutex_features_);
  num_visible_ += n;
}

void MapPoint::IncreaseFound(int n) {
  unique_lock<mutex> lock(mutex_features_);
  num_found_ += n;
}

float MapPoint::GetFoundRatio() {
  unique_lock<mutex> lock(mutex_features_);
  return static_cast<float>(num_found_) / num_visible_;
}

void MapPoint::ComputeDistinctiveDescriptors() {
  // Retrieve all observed descriptors
  std::vector<cv::Mat> desc_vectors;

  std::map<KeyFrame *, size_t> observations;

  {
    unique_lock<mutex> lock1(mutex_features_);
    if (is_bad_)
      return;
    observations = observations_;
  }

  if (observations.empty())
    return;

  desc_vectors.reserve(observations.size());

  for (map<KeyFrame *, size_t>::iterator mit = observations.begin(),
                                         mend = observations.end();
       mit != mend; mit++) {
    KeyFrame *keyfrm = mit->first;

    if (!keyfrm->isBad())
      desc_vectors.push_back(keyfrm->desc_.row(mit->second));
  }

  if (desc_vectors.empty())
    return;

  const size_t N = desc_vectors.size();

  // float Distances[N][N];
  std::vector<std::vector<float>> distances;
  distances.resize(N, vector<float>(N, 0));
  for (size_t i = 0; i < N; i++) {
    distances[i][i] = 0;
    for (size_t j = i + 1; j < N; j++) {
      int distij =
          ORBmatcher::DescriptorDistance(desc_vectors[i], desc_vectors[j]);
      distances[i][j] = distij;
      distances[j][i] = distij;
    }
  }

  // Take the descriptor with least median distance to the rest
  int best_median = INT_MAX;
  int best_index = 0;
  for (size_t i = 0; i < N; i++) {
    // vector<int> vDists(Distances[i],Distances[i]+N);
    vector<int> vDists(distances[i].begin(), distances[i].end());
    sort(vDists.begin(), vDists.end());

    int median = vDists[0.5 * (N - 1)];

    if (median < best_median) {
      best_median = median;
      best_index = i;
    }
  }

  {
    unique_lock<mutex> lock(mutex_features_);

    descriptor_ = desc_vectors[best_index].clone();
  }
}

cv::Mat MapPoint::GetDescriptor() {
  unique_lock<mutex> lock(mutex_features_);
  return descriptor_.clone();
}

void MapPoint::SetDescriptor(const cv::Mat &des) {
  unique_lock<mutex> lock(mutex_features_);
  descriptor_ = des.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF) {
  unique_lock<mutex> lock(mutex_features_);
  if (observations_.count(pKF))
    return observations_[pKF];
  else
    return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF) {
  unique_lock<mutex> lock(mutex_features_);
  return (observations_.count(pKF));
}

void MapPoint::UpdateNormalAndDepth() {
  map<KeyFrame *, size_t> observations;
  KeyFrame *ref_keyfrm;
  cv::Mat pos;
  {
    unique_lock<mutex> lock1(mutex_features_);
    unique_lock<mutex> lock2(mutex_pos_);
    if (is_bad_)
      return;

    observations = observations_;
    ref_keyfrm = ref_keyfrm_;
    pos = world_pos_.clone();
  }

  if (observations.empty())
    return;

  cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
  int n = 0;
  for (map<KeyFrame *, size_t>::iterator mit = observations.begin(),
                                         mend = observations.end();
       mit != mend; mit++) {
    KeyFrame *keyfrm = mit->first;
    cv::Mat cam_center = keyfrm->GetCameraCenter();
    cv::Mat normali = world_pos_ - cam_center;
    normal = normal + normali / cv::norm(normali);
    n++;
  }

  cv::Mat pc_vector = pos - ref_keyfrm->GetCameraCenter();
  const float dist = cv::norm(pc_vector);
  const int level = ref_keyfrm->keypts_[observations[ref_keyfrm]].octave;
  const float levelScaleFactor = ref_keyfrm->scale_factors_[level];
  const int num_levels = ref_keyfrm->scale_levels_;

  {
    unique_lock<mutex> lock3(mutex_pos_);
    max_distance_ = dist * levelScaleFactor;
    min_distance_ = max_distance_ / ref_keyfrm->scale_factors_[num_levels - 1];
    normal_vector_ = normal / n;
  }
}

float MapPoint::GetMinDistanceInvariance() {
  unique_lock<mutex> lock(mutex_pos_);
  return 0.8f * min_distance_;
}

float MapPoint::GetMaxDistanceInvariance() {
  unique_lock<mutex> lock(mutex_pos_);
  return 1.2f * max_distance_;
}

//              ____
// Nearer      /____\     level:n-1 --> dmin
//            /______\                       d/dmin = 1.2^(n-1-m)
//           /________\   level:m   --> d
//          /__________\                     dmax/d = 1.2^m
// Farther /____________\ level:0   --> dmax
//
//           log(dmax/d)
// m = ceil(------------)
//            log(1.2)
int MapPoint::PredictScale(const float &currentDist, KeyFrame *keyfrm) {
  float ratio;
  {
    unique_lock<mutex> lock(mutex_pos_);

    ratio = max_distance_ / currentDist;
  }

  int int_scale = ceil(log(ratio) / keyfrm->log_scale_factor_);
  if (int_scale < 0)
    int_scale = 0;
  else if (int_scale >= keyfrm->scale_levels_)
    int_scale = keyfrm->scale_levels_ - 1;

  return int_scale;
}

int MapPoint::PredictScale(const float &currentDist, Frame *frame) {
  float ratio;
  {
    unique_lock<mutex> lock(mutex_pos_);
    ratio = max_distance_ / currentDist;
  }

  int num_scale = ceil(log(ratio) / frame->log_scale_factor_);
  if (num_scale < 0)
    num_scale = 0;
  else if (num_scale >= frame->scale_levels_)
    num_scale = frame->scale_levels_ - 1;

  return num_scale;
}
}
