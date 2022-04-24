#include "keyframe.h"
#include "converter.h"
#include "orbmatcher.h"
#include <mutex>

namespace FeatureSLAM {

long unsigned int KeyFrame::next_id_ = 0;

KeyFrame::KeyFrame(Frame &F, Map *map, KeyFrameDatabase *keyfrm_database)
    : camera_index_(FRONT_CAMERA), timestamp_(F.timestamp_),
      num_cell_cols_(FRAME_GRID_COLS), num_cell_rows_(FRAME_GRID_ROWS),
      cell_element_width_inv_(F.mfGridElementWidthInv),
      cell_element_height_inv_(F.mfGridElementHeightInv), track_reffrm_id_(0),
      fuse_target_frm_id_(0), mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0),
      mnRelocWords(0), num_feature_(F.num_feature_), keypts_(F.keypts_),
      desc_(F.descriptors_.clone()), bow_vec_(F.bow_vec_),
      featd_vec_(F.feat_vec_), scale_levels_(F.scale_levels_),
      scale_factor_(F.scale_factor_), log_scale_factor_(F.log_scale_factor_),
      scale_factors_(F.scale_factors_), level_sigma2_(F.level_sigma2_),
      inv_level_sigma2_(F.inv_level_sigma2_), min_x_(F.mnMinX),
      min_y_(F.mnMinY), max_x_(F.mnMaxX), max_y_(F.mnMaxY),
      mappoints_(F.mappoints_), keyfrm_database_(keyfrm_database),
      orb_voc_(F.orb_voc_), first_connection_(true), parent_keyfrm_(nullptr),
      not_erase_(false), to_be_erased_(false), is_bad_(false), map_(map) {
  id_ = next_id_++;

  cells_.resize(num_cell_cols_);
  for (int i = 0; i < num_cell_cols_; i++) {
    cells_[i].resize(num_cell_rows_);
    for (int j = 0; j < num_cell_rows_; j++)
      cells_[i][j] = F.keypt_indices_in_cells_[i][j];
  }
  mc_keyfrm_ = nullptr;
  SetPose(F.Tcw_);
}

void KeyFrame::ComputeBoW() {
  if (bow_vec_.empty() || featd_vec_.empty()) {
    std::vector<cv::Mat> current_descs = Converter::toDescriptorVector(desc_);
    // Feature vector associate features with nodes in the 4th level (from
    // leaves up)
    // We assume the vocabulary tree has 6 levels, change the 4 otherwise
    orb_voc_->transform(current_descs, bow_vec_, featd_vec_, 4);
  }
}

void KeyFrame::SetPose(const cv::Mat &Tcw) {
  unique_lock<mutex> lock(mutex_pose_);
  Tcw.copyTo(Tcw_);
  cv::Mat Rcw = Tcw_.rowRange(0, 3).colRange(0, 3);
  cv::Mat tcw = Tcw_.rowRange(0, 3).col(3);
  cv::Mat Rwc = Rcw.t();
  Ow = -Rwc * tcw;

  Twc = cv::Mat::eye(4, 4, Tcw_.type());
  Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
  Ow.copyTo(Twc.rowRange(0, 3).col(3));
}

cv::Mat KeyFrame::GetPose() {
  unique_lock<mutex> lock(mutex_pose_);
  return Tcw_.clone();
}

cv::Mat KeyFrame::GetPoseInverse() {
  unique_lock<mutex> lock(mutex_pose_);
  return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter() {
  unique_lock<mutex> lock(mutex_pose_);
  return Ow.clone();
}

cv::Mat KeyFrame::GetRotation() {
  unique_lock<mutex> lock(mutex_pose_);
  return Tcw_.rowRange(0, 3).colRange(0, 3).clone();
}

cv::Mat KeyFrame::GetTranslation() {
  unique_lock<mutex> lock(mutex_pose_);
  return Tcw_.rowRange(0, 3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *keyfrm, const int &weight) {
  {
    unique_lock<mutex> lock(mutex_connections_);
    if (!connected_keyfrm_weights_.count(keyfrm))
      connected_keyfrm_weights_[keyfrm] = weight;
    else if (connected_keyfrm_weights_[keyfrm] != weight)
      connected_keyfrm_weights_[keyfrm] = weight;
    else
      return;
  }

  UpdateBestCovisibles();
}

void KeyFrame::AddConnectionForce(KeyFrame *keyfrm, const int &weight) {
  {
    unique_lock<mutex> lock(mutex_connections_);
    connected_keyfrm_weights_[keyfrm] = weight;
  }
}

void KeyFrame::UpdateBestCovisibles() {
  unique_lock<mutex> lock(mutex_connections_);
  // http://stackoverflow.com/questions/3389648/difference-between-stdliststdpair-and-stdmap-in-c-stl
  vector<pair<int, KeyFrame *>> pairs;
  pairs.reserve(connected_keyfrm_weights_.size());

  for (auto kw_iter : connected_keyfrm_weights_) {
    pairs.push_back(std::make_pair(kw_iter.second, kw_iter.first));
  }

  std::sort(pairs.begin(), pairs.end());
  std::list<KeyFrame *> lKFs; // keyframe
  std::list<int> lWs;         // weight
  for (size_t i = 0, iend = pairs.size(); i < iend; i++) {
    lKFs.push_front(pairs[i].second);
    lWs.push_front(pairs[i].first);
  }

  ordered_connected_keyfrms_ =
      std::vector<KeyFrame *>(lKFs.begin(), lKFs.end());
  ordered_weights_ = std::vector<int>(lWs.begin(), lWs.end());
}

std::set<KeyFrame *> KeyFrame::GetConnectedKeyFrames() {
  unique_lock<mutex> lock(mutex_connections_);
  std::set<KeyFrame *> cnt_keyfrms;

  for (auto cnt_key_wgt : connected_keyfrm_weights_)
    cnt_keyfrms.insert(cnt_key_wgt.first);

  return cnt_keyfrms;
}

vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames() {
  unique_lock<mutex> lock(mutex_connections_);
  return ordered_connected_keyfrms_;
}

vector<KeyFrame *> KeyFrame::GetBestCovisibilityKeyFrames(const int &N) {
  unique_lock<mutex> lock(mutex_connections_);
  if ((int)ordered_connected_keyfrms_.size() < N)
    return ordered_connected_keyfrms_;
  else
    return vector<KeyFrame *>(ordered_connected_keyfrms_.begin(),
                              ordered_connected_keyfrms_.begin() + N);
}

vector<KeyFrame *> KeyFrame::GetCovisiblesByWeight(const int &w) {
  unique_lock<mutex> lock(mutex_connections_);

  if (ordered_connected_keyfrms_.empty())
    return vector<KeyFrame *>();

  // http://www.cplusplus.com/reference/algorithm/upper_bound/

  vector<int>::iterator it =
      upper_bound(ordered_weights_.begin(), ordered_weights_.end(), w,
                  KeyFrame::weightComp);
  if (it == ordered_weights_.end() && *ordered_weights_.rbegin() < w)
    return vector<KeyFrame *>();
  else {
    int n = it - ordered_weights_.begin();
    return vector<KeyFrame *>(ordered_connected_keyfrms_.begin(),
                              ordered_connected_keyfrms_.begin() + n);
  }
}

int KeyFrame::GetWeight(KeyFrame *keyfrm) {
  unique_lock<mutex> lock(mutex_connections_);
  if (connected_keyfrm_weights_.count(keyfrm))
    return connected_keyfrm_weights_[keyfrm];
  else
    return 0;
}

void KeyFrame::AddMapPoint(MapPoint *mp, const size_t &idx) {
  unique_lock<mutex> lock(mutex_features_);
  mappoints_[idx] = mp;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx) {
  unique_lock<mutex> lock(mutex_features_);
  mappoints_[idx] = nullptr;
}

void KeyFrame::EraseMapPointMatch(MapPoint *mp) {
  int idx = mp->GetIndexInKeyFrame(this);
  if (idx >= 0)
    mappoints_[idx] = nullptr;
}

void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint *mp) {
  mappoints_[idx] = mp;
}

std::set<MapPoint *> KeyFrame::GetMapPoints() {
  unique_lock<mutex> lock(mutex_features_);
  std::set<MapPoint *> mappoints;

  for (auto mp : mappoints) {
    if (!mp)
      continue;
    if (!mp->isBad())
      mappoints.insert(mp);
  }
  return mappoints;
}

int KeyFrame::TrackedMapPoints(const int &min_obs) {
  unique_lock<mutex> lock(mutex_features_);

  int num_points = 0;
  const bool check_obs = min_obs > 0;
  for (int i = 0; i < num_feature_; i++) {
    MapPoint *mp = mappoints_[i];
    if (mp) {
      if (!mp->isBad()) {
        if (check_obs) {
          if (mappoints_[i]->Observations() >= min_obs)
            num_points++;
        } else
          num_points++;
      }
    }
  }

  return num_points;
}

vector<MapPoint *> KeyFrame::GetMapPointMatches() {
  unique_lock<mutex> lock(mutex_features_);
  return mappoints_;
}

MapPoint *KeyFrame::GetMapPoint(const size_t &idx) {
  unique_lock<mutex> lock(mutex_features_);
  return mappoints_[idx];
}

void KeyFrame::UpdateConnections() {

  std::map<KeyFrame *, int> keyfrm_counter;

  std::vector<MapPoint *> mps;

  {

    unique_lock<mutex> lockMPs(mutex_features_);
    mps = mappoints_;
  }

  for (auto mp : mps) {

    if (!mp)
      continue;

    if (mp->isBad())
      continue;

    map<KeyFrame *, size_t> observations = mp->GetObservations();

    for (map<KeyFrame *, size_t>::iterator mit = observations.begin(),
                                           mend = observations.end();
         mit != mend; mit++) {
      if (mit->first->id_ == id_)
        continue;
      keyfrm_counter[mit->first]++;
    }
  }

  if (keyfrm_counter.empty())
    return;

  int nmax = 0;
  KeyFrame *keyfrm_max = nullptr;
  int th = 15;

  vector<pair<int, KeyFrame *>> pairs;
  pairs.reserve(keyfrm_counter.size());
  for (map<KeyFrame *, int>::iterator mit = keyfrm_counter.begin(),
                                      mend = keyfrm_counter.end();
       mit != mend; mit++) {
    if (mit->second > nmax) {
      nmax = mit->second;
      keyfrm_max = mit->first;
    }
    if (mit->second >= th) {
      pairs.push_back(make_pair(mit->second, mit->first));
      (mit->first)->AddConnection(this, mit->second);
    }
  }

  if (pairs.empty()) {

    pairs.push_back(make_pair(nmax, keyfrm_max));
    keyfrm_max->AddConnection(this, nmax);
  }

  sort(pairs.begin(), pairs.end());
  list<KeyFrame *> lKFs;
  list<int> lWs;
  for (size_t i = 0; i < pairs.size(); i++) {
    lKFs.push_front(pairs[i].second);
    lWs.push_front(pairs[i].first);
  }

  {
    unique_lock<mutex> lockCon(mutex_connections_);
    connected_keyfrm_weights_ = keyfrm_counter;
    ordered_connected_keyfrms_ = vector<KeyFrame *>(lKFs.begin(), lKFs.end());
    ordered_weights_ = vector<int>(lWs.begin(), lWs.end());

    if (first_connection_ && id_ != 0) {
      parent_keyfrm_ = ordered_connected_keyfrms_.front();
      parent_keyfrm_->AddChild(this);
      first_connection_ = false;
    }
  }
}

void KeyFrame::AddChild(KeyFrame *keyfrm) {
  unique_lock<mutex> lockCon(mutex_connections_);
  children_keyfrms_.insert(keyfrm);
}

void KeyFrame::EraseChild(KeyFrame *keyfrm) {
  unique_lock<mutex> lockCon(mutex_connections_);
  children_keyfrms_.erase(keyfrm);
}

void KeyFrame::ChangeParent(KeyFrame *keyfrm) {
  unique_lock<mutex> lockCon(mutex_connections_);
  parent_keyfrm_ = keyfrm;
  keyfrm->AddChild(this);
}

set<KeyFrame *> KeyFrame::GetChilds() {
  unique_lock<mutex> lockCon(mutex_connections_);
  return children_keyfrms_;
}

KeyFrame *KeyFrame::GetParent() {
  unique_lock<mutex> lockCon(mutex_connections_);
  return parent_keyfrm_;
}

MCKeyFrame *KeyFrame::GetMCKeyFrame() {
  unique_lock<mutex> lock(mutex_mc_connections_);
  return mc_keyfrm_;
}

void KeyFrame::SetMCConnection(MCKeyFrame *mc_keyfrm) {
  unique_lock<mutex> lock(mutex_mc_connections_);
  mc_keyfrm_ = mc_keyfrm;
}

void KeyFrame::SetCameraIndex(int cam_index) { camera_index_ = cam_index; }

bool KeyFrame::hasChild(KeyFrame *keyfrm) {
  unique_lock<mutex> lockCon(mutex_connections_);
  return children_keyfrms_.count(keyfrm);
}

void KeyFrame::SetNotErase() {
  unique_lock<mutex> lock(mutex_connections_);
  not_erase_ = true;
}

void KeyFrame::SetErase() {
  {
    unique_lock<mutex> lock(mutex_connections_);
    not_erase_ = false;
  }

  if (to_be_erased_) {
    SetBadFlag();
  }
}

void KeyFrame::SetBadFlag() {

  for (map<KeyFrame *, int>::iterator mit = connected_keyfrm_weights_.begin(),
                                      mend = connected_keyfrm_weights_.end();
       mit != mend; mit++)
    mit->first->EraseConnection(this);

  for (size_t i = 0; i < mappoints_.size(); i++)
    if (mappoints_[i])
      mappoints_[i]->EraseObservation(this);

  map_->EraseKeyFrame(this);
  is_bad_ = true;
  parent_keyfrm_ = nullptr;
}

bool KeyFrame::isBad() {
  unique_lock<mutex> lock(mutex_connections_);
  return is_bad_;
}

void KeyFrame::EraseConnection(KeyFrame *keyfrm) {
  bool update = false;
  {
    unique_lock<mutex> lock(mutex_connections_);
    if (connected_keyfrm_weights_.count(keyfrm)) {
      connected_keyfrm_weights_.erase(keyfrm);
      update = true;
    }
  }

  if (update)
    UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y,
                                           const float &r) const {
  vector<size_t> indices;
  indices.reserve(num_feature_);

  const int kMinCellX =
      max(0, (int)floor((x - min_x_ - r) * cell_element_width_inv_));
  if (kMinCellX >= num_cell_cols_)
    return indices;

  const int kMaxCellX =
      min((int)num_cell_cols_ - 1,
          (int)ceil((x - min_x_ + r) * cell_element_width_inv_));
  if (kMaxCellX < 0)
    return indices;

  const int kMinCellY =
      max(0, (int)floor((y - min_y_ - r) * cell_element_height_inv_));
  if (kMinCellY >= num_cell_rows_)
    return indices;

  const int kMaxCellY =
      min((int)num_cell_rows_ - 1,
          (int)ceil((y - min_y_ + r) * cell_element_height_inv_));
  if (kMaxCellY < 0)
    return indices;

  for (int ix = kMinCellX; ix <= kMaxCellX; ix++) {
    for (int iy = kMinCellY; iy <= kMaxCellY; iy++) {
      const vector<size_t> vCell = cells_[ix][iy];
      for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
        const cv::KeyPoint &keypt = keypts_[vCell[j]];
        const float distx = keypt.pt.x - x;
        const float disty = keypt.pt.y - y;

        if (fabs(distx) < r && fabs(disty) < r)
          indices.push_back(vCell[j]);
      }
    }
  }

  return indices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const {
  return (x >= min_x_ && x < max_x_ && y >= min_y_ && y < max_y_);
}

float KeyFrame::ComputeSceneMedianDepth(const int q) {
  vector<MapPoint *> mappoints;
  cv::Mat Tcw;
  {
    unique_lock<mutex> lock(mutex_features_);
    unique_lock<mutex> lock2(mutex_pose_);
    mappoints = mappoints_;
    Tcw = Tcw_.clone();
  }

  std::vector<float> depths;
  depths.reserve(num_feature_);
  cv::Mat Rcw2 = Tcw.row(2).colRange(0, 3);
  Rcw2 = Rcw2.t();
  float zcw = Tcw.at<float>(2, 3);
  for (int i = 0; i < num_feature_; i++) {
    if (mappoints_[i]) {
      MapPoint *mp = mappoints_[i];
      cv::Mat pos = mp->GetWorldPos();
      float z = Rcw2.dot(pos) + zcw;
      depths.push_back(z);
    }
  }

  sort(depths.begin(), depths.end());

  return depths[(depths.size() - 1) / q];
}
}
