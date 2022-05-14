#include "map.h"
#include "converter.h"
#include "orbextractor.h"
#include <climits>
#include <mutex>
#include <sys/stat.h>

namespace FeatureSLAM {

Map::Map() {}

/**
 * @brief Insert KeyFrame in the map
 * @param pKF KeyFrame
 */
void Map::AddKeyFrame(KeyFrame *keyfrm) {
  unique_lock<mutex> lock(mutex_map_);
  keyframes_.insert(keyfrm);
}

/**
 * @brief Insert MapPoint in the map
 * @param pMP MapPoint
 */
void Map::AddMapPoint(MapPoint *mp) {
  unique_lock<mutex> lock(mutex_map_);
  map_points_.insert(mp);
}

/**
 * @brief Erase MapPoint from the map
 * @param pMP MapPoint
 */
void Map::EraseMapPoint(MapPoint *mp) {
  unique_lock<mutex> lock(mutex_map_);
  map_points_.erase(mp);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

/**
 * @brief Erase KeyFrame from the map
 * @param pKF KeyFrame
 */
void Map::EraseKeyFrame(KeyFrame *keyfrm) {
  unique_lock<mutex> lock(mutex_map_);
  keyframes_.erase(keyfrm);

  // TODO: This only erase the pointer.
  // Delete the MapPoint
}

/**
 * @brief 设置参考MapPoints，将用于DrawMapPoints函数画图
 * @param vpMPs Local MapPoints
 */
void Map::SetReferenceMapPoints(const vector<MapPoint *> &mps) {
  unique_lock<mutex> lock(mutex_map_);
  reference_mappoints_ = mps;
}

vector<KeyFrame *> Map::GetAllKeyFrames() {
  unique_lock<mutex> lock(mutex_map_);
  return vector<KeyFrame *>(keyframes_.begin(), keyframes_.end());
}

std::vector<MCKeyFrame *> Map::GetAllMCKeyFrames() {
  std::unique_lock<std::mutex> lock(mutex_map_);

  std::set<MCKeyFrame *> mc_keyfrms;

  for (auto keyfrm : keyframes_) {
    auto mc_keyfrm = keyfrm->GetMCKeyFrame();

    if (mc_keyfrm && mc_keyfrms.count(mc_keyfrm) == 0) {
      mc_keyfrms.insert(mc_keyfrm);
    }
  }
  return std::vector<MCKeyFrame *>(mc_keyfrms.begin(), mc_keyfrms.end());
}

vector<MapPoint *> Map::GetAllMapPoints() {
  unique_lock<mutex> lock(mutex_map_);
  return vector<MapPoint *>(map_points_.begin(), map_points_.end());
}

long unsigned int Map::MapPointsInMap() {
  unique_lock<mutex> lock(mutex_map_);
  return map_points_.size();
}

long unsigned int Map::KeyFramesInMap() {
  unique_lock<mutex> lock(mutex_map_);
  return keyframes_.size();
}

vector<MapPoint *> Map::GetReferenceMapPoints() {
  unique_lock<mutex> lock(mutex_map_);
  return reference_mappoints_;
}

void Map::clear() {
  for (set<MapPoint *>::iterator sit = map_points_.begin(),
                                 send = map_points_.end();
       sit != send; sit++)
    delete *sit;

  for (set<KeyFrame *>::iterator sit = keyframes_.begin(),
                                 send = keyframes_.end();
       sit != send; sit++)
    delete *sit;

  map_points_.clear();
  keyframes_.clear();
  reference_mappoints_.clear();
  keyfrm_origins_.clear();
}

KeyFrame *Map::ReadKeyFrame(ifstream &f, ORBVocabulary &voc,
                            map<unsigned long, MapPoint *> mp_of_idx,
                            ORBextractor *orb_ext) {
  Frame fr;
  fr.orb_voc_ = &voc;
  f.read((char *)&fr.id_, sizeof(fr.id_)); // ID
  //  cerr << " reading keyfrane id " << fr.mnId << endl;
  f.read((char *)&fr.timestamp_, sizeof(fr.timestamp_)); // timestamp
  cv::Mat Tcw = cv::Mat::zeros(4, 4, CV_32F);            // position
  f.read((char *)&Tcw.at<float>(0, 3), sizeof(float));
  f.read((char *)&Tcw.at<float>(1, 3), sizeof(float));
  f.read((char *)&Tcw.at<float>(2, 3), sizeof(float));
  Tcw.at<float>(3, 3) = 1.;
  cv::Mat Qcw(1, 4, CV_32F); // orientation
  f.read((char *)&Qcw.at<float>(0, 0), sizeof(float));
  f.read((char *)&Qcw.at<float>(0, 1), sizeof(float));
  f.read((char *)&Qcw.at<float>(0, 2), sizeof(float));
  f.read((char *)&Qcw.at<float>(0, 3), sizeof(float));

  Converter::RmatOfQuat(Tcw, Qcw);
  fr.SetPose(Tcw);
  f.read((char *)&fr.num_feature_, sizeof(fr.num_feature_)); // nb keypoints
  fr.keypts_.reserve(fr.num_feature_);
  fr.descriptors_.create(fr.num_feature_, 32, CV_8UC1);
  fr.mappoints_ =
      vector<MapPoint *>(fr.num_feature_, static_cast<MapPoint *>(NULL));

  for (size_t i = 0; i < fr.num_feature_; i++) {
    cv::KeyPoint kp;
    f.read((char *)&kp.pt.x, sizeof(kp.pt.x));
    f.read((char *)&kp.pt.y, sizeof(kp.pt.y));
    f.read((char *)&kp.size, sizeof(kp.size));
    f.read((char *)&kp.angle, sizeof(kp.angle));
    f.read((char *)&kp.response, sizeof(kp.response));
    f.read((char *)&kp.octave, sizeof(kp.octave));
    fr.keypts_.push_back(kp);
    for (int j = 0; j < 32; j++)
      f.read((char *)&fr.descriptors_.at<unsigned char>(i, j), sizeof(char));
    unsigned long int mpidx;
    f.read((char *)&mpidx, sizeof(mpidx));
    if (mpidx == ULONG_MAX)
      fr.mappoints_[i] = nullptr;
    else
      fr.mappoints_[i] = mp_of_idx[mpidx];
  }

  // mono only for now
  fr.orb_extractor_ = orb_ext;

  fr.InitializeScaleLevels();
  fr.AssignFeaturesToGrid();
  fr.ComputeBoW();

  KeyFrame *kf = new KeyFrame(fr, this, nullptr);
  kf->id_ = fr.id_; // bleeee why? do I store that?

  for (int i = 0; i < fr.num_feature_; i++) {
    if (fr.mappoints_[i]) {
      fr.mappoints_[i]->AddObservation(kf, i);
      if (!fr.mappoints_[i]->GetReferenceKeyFrame())
        fr.mappoints_[i]->SetReferenceKeyFrame(kf);
    }
  }

  return kf;
}

MapPoint *Map::ReadMapPoint(ifstream &f) {
  long unsigned int id;
  f.read((char *)&id, sizeof(id)); // ID
  cv::Mat wp(3, 1, CV_32F);
  f.read((char *)&wp.at<float>(0), sizeof(float));
  f.read((char *)&wp.at<float>(1), sizeof(float));
  f.read((char *)&wp.at<float>(2), sizeof(float));
  long int first_keyfrm_id = 0;
  MapPoint *mp = new MapPoint(wp, first_keyfrm_id, this);
  mp->id_ = id;
  return mp;
}

// question: front_camera_mask??
bool Map::Load(const std::string &filename, ORBVocabulary &voc,
               const std::string mask_path,
               std::vector<cv::Mat> cam_extrinsics) {
  int num_feature = 1000;
  float scale_factor = 1.2;
  int num_level = 8, ini_thr_fast = 20, min_thr_fast = 7;
  ORBextractor orb_ext = ORBextractor(num_feature, scale_factor, num_level,
                                      ini_thr_fast, min_thr_fast, mask_path);

  std::cout << kColorGreen << "Map: reading from " << filename << kColorReset
            << std::endl;
  ifstream fin;
  fin.open(filename.c_str());

  long unsigned int num_mappoints, max_id = 0;
  fin.read((char *)&num_mappoints, sizeof(num_mappoints));
  std::cout << "reading " << num_mappoints << " mappoints" << std::endl;
  for (unsigned int i = 0; i < num_mappoints; i++) {
    MapPoint *mp = ReadMapPoint(fin);
    if (mp->id_ >= max_id)
      max_id = mp->id_;
    AddMapPoint(mp);
  }
  MapPoint::next_id_ =
      max_id + 1; // that is probably wrong if last mappoint is not here :(

  std::map<unsigned long int, MapPoint *> mp_of_idx;

  for (auto mp : map_points_) {
    mp_of_idx[mp->id_] = mp;
  }

  long unsigned int num_keyframes;
  fin.read((char *)&num_keyframes, sizeof(num_keyframes));
  std::cout << "reading " << num_keyframes << " keyframe" << std::endl;
  vector<KeyFrame *> kf_by_order;
  unsigned long int max_keyfrm_id = 0;
  for (unsigned int i = 0; i < num_keyframes; i++) {
    KeyFrame *kf = ReadKeyFrame(fin, voc, mp_of_idx, &orb_ext);
    AddKeyFrame(kf);
    if (kf->id_ > max_keyfrm_id)
      max_keyfrm_id = kf->id_;
    kf_by_order.push_back(kf);
  }

  KeyFrame::next_id_ = max_keyfrm_id;

  std::set<unsigned long int> no_exist_keyfrms;
  std::vector<bool> found(max_keyfrm_id + 1, false);

  // Load Spanning tree
  std::map<unsigned long int, KeyFrame *> kf_by_id;

  for (std::set<KeyFrame *>::iterator sit = keyframes_.begin(),
                                      send = keyframes_.end();
       sit != send; sit++) {
    KeyFrame *pKF = *sit;
    kf_by_id[pKF->id_] = pKF;
    found[pKF->id_] = true;
  }
  for (unsigned long int i = 0; i < found.size(); i++) {
    if (!found[i])
      no_exist_keyfrms.insert(i);
  }

  for (auto kf : kf_by_order) {
    unsigned long int parent_id;
    fin.read((char *)&parent_id, sizeof(parent_id)); // parent id
    if (parent_id != ULONG_MAX) {
      if (no_exist_keyfrms.count(parent_id)) {
        cout << kf->id_ << " Parent Id not found " << endl;
      } else {
        kf->ChangeParent(kf_by_id[parent_id]);
      }
    }
    unsigned long int num_connections; // number connected keyframe
    fin.read((char *)&num_connections, sizeof(num_connections));
    for (unsigned long int i = 0; i < num_connections; i++) {
      unsigned long int id;
      int weight;
      fin.read((char *)&id, sizeof(id));         // connected keyframe
      fin.read((char *)&weight, sizeof(weight)); // connection weight

      if (no_exist_keyfrms.count(id)) {
        cout << kf->id_ << " Children KeyFrame not Found : " << id << endl;
      } else {
        kf->AddConnectionForce(kf_by_id[id], weight);
      }
    }
  }

  for (auto kf : kf_by_order) {
    kf->UpdateBestCovisibles();
  }

  for (auto mp : map_points_) {
    mp->ComputeDistinctiveDescriptors();
    mp->UpdateNormalAndDepth();
  }

  long unsigned int num_mckeyfrms = 0;
  fin.read((char *)&num_mckeyfrms, sizeof(num_mckeyfrms));

  std::cout << "reading " << num_mckeyfrms << " MultiCam Keyframes ! "
            << std::endl;

  MultiCam *multi_cam = new MultiCam(cam_extrinsics);

  for (size_t i = 0; i < num_mckeyfrms; i++) {
    unsigned long int left_cam_id = 0;
    unsigned long int front_cam_id = 0;
    unsigned long int right_cam_id = 0;
    unsigned long int back_cam_id = 0;

    fin.read((char *)&left_cam_id, sizeof(left_cam_id));
    fin.read((char *)&front_cam_id, sizeof(front_cam_id));
    fin.read((char *)&right_cam_id, sizeof(right_cam_id));
    fin.read((char *)&back_cam_id, sizeof(back_cam_id));
    MCKeyFrame *mc_keyfrm = new MCKeyFrame(
        kf_by_id[left_cam_id], kf_by_id[front_cam_id], kf_by_id[right_cam_id],
        kf_by_id[back_cam_id], multi_cam);
  }

  std::cout << kColorGreen << " Finishing reading map file" << kColorReset
            << std::endl;
  return true;
}

void Map::WriteMapPoint(ofstream &f, MapPoint *mp) {
  f.write((char *)&mp->id_, sizeof(mp->id_)); // id: long unsigned int
  cv::Mat wp = mp->GetWorldPos();
  f.write((char *)&wp.at<float>(0), sizeof(float)); // pos x: float
  f.write((char *)&wp.at<float>(1), sizeof(float)); // pos y: float
  f.write((char *)&wp.at<float>(2), sizeof(float)); // pos z: float
}

void Map::WriteKeyFrame(ofstream &f, KeyFrame *kf,
                        map<MapPoint *, unsigned long int> &idx_of_mp) {
  f.write((char *)&kf->id_, sizeof(kf->id_)); // id: long unsigned int
  f.write((char *)&kf->timestamp_, sizeof(kf->timestamp_)); // ts: double

  cv::Mat Tcw = kf->GetPose();
  f.write((char *)&Tcw.at<float>(0, 3), sizeof(float)); // px: float
  f.write((char *)&Tcw.at<float>(1, 3), sizeof(float)); // py: float
  f.write((char *)&Tcw.at<float>(2, 3), sizeof(float)); // pz: float
  vector<float> Qcw =
      Converter::toQuaternion(Tcw.rowRange(0, 3).colRange(0, 3));
  f.write((char *)&Qcw[0], sizeof(float)); // qx: float
  f.write((char *)&Qcw[1], sizeof(float)); // qy: float
  f.write((char *)&Qcw[2], sizeof(float)); // qz: float
  f.write((char *)&Qcw[3], sizeof(float)); // qw: float
  f.write((char *)&kf->num_feature_,
          sizeof(kf->num_feature_)); // nb_features: int
  for (int i = 0; i < kf->num_feature_; i++) {
    cv::KeyPoint kp = kf->keypts_[i];
    f.write((char *)&kp.pt.x, sizeof(kp.pt.x));         // float
    f.write((char *)&kp.pt.y, sizeof(kp.pt.y));         // float
    f.write((char *)&kp.size, sizeof(kp.size));         // float
    f.write((char *)&kp.angle, sizeof(kp.angle));       // float
    f.write((char *)&kp.response, sizeof(kp.response)); // float
    f.write((char *)&kp.octave, sizeof(kp.octave));     // int
    for (int j = 0; j < 32; j++)
      f.write((char *)&kf->desc_.at<unsigned char>(i, j), sizeof(char));

    unsigned long int mpidx;
    MapPoint *mp = kf->GetMapPoint(i);
    if (mp == nullptr)
      mpidx = ULONG_MAX;
    else
      mpidx = idx_of_mp[mp];
    f.write((char *)&mpidx, sizeof(mpidx)); // long int
  }
}

void Map::WriteKeyFrameBow(ofstream &f, KeyFrame *kf,
                           map<MapPoint *, unsigned long> &idx_of_mp) {
  f.write((char *)&kf->id_, sizeof(kf->id_)); // id: long unsigned int
  f.write((char *)&kf->timestamp_, sizeof(kf->timestamp_)); // ts: double

  cv::Mat Tcw = kf->GetPose();
  f.write((char *)&Tcw.at<float>(0, 3), sizeof(float)); // px: float
  f.write((char *)&Tcw.at<float>(1, 3), sizeof(float)); // py: float
  f.write((char *)&Tcw.at<float>(2, 3), sizeof(float)); // pz: float
  vector<float> Qcw =
      Converter::toQuaternion(Tcw.rowRange(0, 3).colRange(0, 3));
  f.write((char *)&Qcw[0], sizeof(float)); // qx: float
  f.write((char *)&Qcw[1], sizeof(float)); // qy: float
  f.write((char *)&Qcw[2], sizeof(float)); // qz: float
  f.write((char *)&Qcw[3], sizeof(float)); // qw: float
  f.write((char *)&kf->num_feature_,
          sizeof(kf->num_feature_)); // nb_features: int
  for (int i = 0; i < kf->num_feature_; i++) {
    cv::KeyPoint kp = kf->keypts_[i];
    f.write((char *)&kp.pt.x, sizeof(kp.pt.x));         // float
    f.write((char *)&kp.pt.y, sizeof(kp.pt.y));         // float
    f.write((char *)&kp.size, sizeof(kp.size));         // float
    f.write((char *)&kp.angle, sizeof(kp.angle));       // float
    f.write((char *)&kp.response, sizeof(kp.response)); // float
    f.write((char *)&kp.octave, sizeof(kp.octave));     // int
    for (int j = 0; j < 32; j++)
      f.write((char *)&kf->desc_.at<unsigned char>(i, j), sizeof(char));

    unsigned long int mpidx;
    MapPoint *mp = kf->GetMapPoint(i);
    if (mp == NULL)
      mpidx = ULONG_MAX;
    else
      mpidx = idx_of_mp[mp];
    f.write((char *)&mpidx, sizeof(mpidx)); // long int
  }

  // write bow
  unsigned int nWord = kf->bow_vec_.size();
  f.write((char *)&nWord, sizeof(nWord));
  for (DBoW2::BowVector::iterator sit = kf->bow_vec_.begin(),
                                  send = kf->bow_vec_.end();
       sit != send; sit++) {
    DBoW2::WordId wid = sit->first;
    DBoW2::WordValue wvalue = sit->second;
    f.write((char *)&wid, sizeof(wid));
    f.write((char *)&wvalue, sizeof(wvalue));
  }

  // write feature

  unsigned int nNode = kf->featd_vec_.size();
  f.write((char *)&nNode, sizeof(nNode));

  for (DBoW2::FeatureVector::iterator sit = kf->featd_vec_.begin(),
                                      send = kf->featd_vec_.end();
       sit != send; sit++) {
    DBoW2::NodeId nid = sit->first;
    f.write((char *)&nid, sizeof(nid));

    std::vector<unsigned int> node_value = sit->second;
    unsigned int nNodeValueSize = node_value.size();

    f.write((char *)&nNodeValueSize, sizeof(nNodeValueSize));

    for (size_t i = 0; i < node_value.size(); i++) {
      unsigned int node_valuei = node_value[i];
      f.write((char *)&node_valuei, sizeof(node_valuei));
    }
  }
}

bool Map::Save(const std::string &filename, bool save_bow) {
  cerr << "Map: Saving to " << filename << endl;
  ofstream fout;
  fout.open(filename.c_str(), ios_base::out | ios::binary);
  std::cout << "  writing " << map_points_.size() << " mappoints" << std::endl;
  unsigned long int num_mappoints = map_points_.size();
  fout.write((char *)&num_mappoints, sizeof(num_mappoints));
  for (auto mp : map_points_)
    WriteMapPoint(fout, mp);

  std::map<MapPoint *, unsigned long int> idx_of_mp;
  for (auto mp : map_points_) {
    idx_of_mp[mp] = mp->id_;
  }

  std::cout << "  writing " << keyframes_.size() << " keyframes" << std::endl;
  unsigned long int num_keyfrms = keyframes_.size();
  fout.write((char *)&num_keyfrms, sizeof(num_keyfrms));
  for (auto kf : keyframes_) {
    if (save_bow)
      WriteKeyFrameBow(fout, kf, idx_of_mp);
    else
      WriteKeyFrame(fout, kf, idx_of_mp);
  }

  // store tree and graph
  for (auto kf : keyframes_) {
    KeyFrame *parent_keyfrm = kf->GetParent();
    unsigned long int parent_id = ULONG_MAX;
    if (parent_keyfrm)
      parent_id = parent_keyfrm->id_;
    fout.write((char *)&parent_id, sizeof(parent_id));
    unsigned long int num_connected_keyfrms =
        kf->GetConnectedKeyFrames().size();
    fout.write((char *)&num_connected_keyfrms, sizeof(num_connected_keyfrms));
    for (auto connected_keyfrm : kf->GetConnectedKeyFrames()) {
      int weight = kf->GetWeight(connected_keyfrm);
      fout.write((char *)&connected_keyfrm->id_, sizeof(connected_keyfrm->id_));
      fout.write((char *)&weight, sizeof(weight));
    }
  }

  // save

  unsigned long int num_mc_keyfrm = 0;
  for (auto kf : keyframes_) {
    if (kf->camera_index_ == LEFT_CAMERA)
      num_mc_keyfrm++;
  }

  cout << " MCkeyframes : " << num_mc_keyfrm << endl;
  fout.write((char *)&num_mc_keyfrm, sizeof(num_mc_keyfrm));
  for (auto kf : keyframes_) {
    if (kf->camera_index_ == LEFT_CAMERA) {
      MCKeyFrame *mc_keyfrm = kf->GetMCKeyFrame();
      fout.write((char *)&mc_keyfrm->keyframes_[LEFT_CAMERA]->id_,
                 sizeof(mc_keyfrm->keyframes_[LEFT_CAMERA]->id_));
      fout.write((char *)&mc_keyfrm->keyframes_[FRONT_CAMERA]->id_,
                 sizeof(mc_keyfrm->keyframes_[FRONT_CAMERA]->id_));
      fout.write((char *)&mc_keyfrm->keyframes_[RIGHT_CAMERA]->id_,
                 sizeof(mc_keyfrm->keyframes_[RIGHT_CAMERA]->id_));
      fout.write((char *)&mc_keyfrm->keyframes_[BACK_CAMERA]->id_,
                 sizeof(mc_keyfrm->keyframes_[BACK_CAMERA]->id_));
    }
  }

  fout.close();
  std::cout << kColorGreen << "Map: finished saving" << kColorReset
            << std::endl;
  struct stat st;
  stat(filename.c_str(), &st);
  std::cout << "Map: saved " << kColorGreen << st.st_size << kColorReset
            << " bytes" << endl;

  return true;
}

void Map::MoveMap(cv::Mat transform) {

  vector<MapPoint *> all_mps = GetAllMapPoints();
  for (size_t i = 0; i < all_mps.size(); i++) {
    MapPoint *mp = all_mps[i];
    cv::Mat pos = cv::Mat::ones(4, 1, CV_32F);
    mp->GetWorldPos().copyTo(pos.rowRange(0, 3));
    cv::Mat new_pos = transform * pos;
    mp->SetWorldPos(new_pos.rowRange(0, 3));
  }

  vector<KeyFrame *> all_keyfrms = GetAllKeyFrames();
  for (size_t i = 0; i < all_keyfrms.size(); i++) {
    KeyFrame *keyfrm = all_keyfrms[i];
    keyfrm->SetPose(keyfrm->GetPose() * transform.inv());
  }

  std::cout << " the map is transformed to new coordinate " << std::endl;
}

void Map::AlignToOdometry() {

  std::vector<MCKeyFrame *> all_mc_keyfrm = GetAllMCKeyFrames();

  std::map<int, cv::Mat> mc_keyfrm_tcw;
  std::map<int, cv::Mat> correct_mc_keyfrm_twc;

  for (auto mc_keyfrm : all_mc_keyfrm) {
    mc_keyfrm_tcw[mc_keyfrm->id_] = mc_keyfrm->GetPose();
    cv::Mat tcw = MultiCamExt::GetInstance().BaselinkPose2Camera(
        Converter::toOdometryMatrix(mc_keyfrm->odometry_vector_), FRONT_CAMERA);
    mc_keyfrm->SetPose(tcw);
    correct_mc_keyfrm_twc[mc_keyfrm->id_] = tcw.inv();
  }

  std::vector<MapPoint *> all_mps = GetAllMapPoints();
  for (auto mp : all_mps) {

    if (mp->isBad())
      continue;
    auto mc_keyfrm = mp->GetReferenceKeyFrame()->GetMCKeyFrame();
    int mc_keyfrm_id = mc_keyfrm->id_;

    if (mc_keyfrm_tcw.count(mc_keyfrm_id) == 0 ||
        correct_mc_keyfrm_twc.count(mc_keyfrm_id) == 0) {
      std::cout << kColorRed
                << " impossible : can not find corresponding mc_keyfrm "
                << kColorReset << std::endl;
      continue;
    }

    cv::Mat tcw = mc_keyfrm_tcw[mc_keyfrm_id];
    cv::Mat correct_twc = correct_mc_keyfrm_twc[mc_keyfrm_id];

    cv::Mat pos = mp->GetWorldPos();
    cv::Mat pos4d = (cv::Mat_<float>(4, 1) << pos.at<float>(0),
                     pos.at<float>(1), pos.at<float>(2), 1);

    cv::Mat new_pos = correct_twc * tcw * pos4d;
    cv::Mat new_pos3d = new_pos.rowRange(0, 3);
    mp->SetWorldPos(new_pos3d);
    mp->UpdateNormalAndDepth();
  }
}

void Map::SaveOdometry(const string &filename) {
  std::vector<MCKeyFrame *> all_mc_keyfrms = GetAllMCKeyFrames();
  std::sort(all_mc_keyfrms.begin(), all_mc_keyfrms.end(), MCKeyFrame::lId);

  std::ofstream fout;
  fout.open(filename);
  fout.precision(20);
  for (auto mc_keyfrm : all_mc_keyfrms) {

    auto odom_matrix = Converter::toOdometryMatrix(mc_keyfrm->odometry_vector_);

    Mat33_t rotation = odom_matrix.block(0, 0, 3, 3);
    Quat_t quat(rotation);

    fout << mc_keyfrm->timestamp_ << " " << odom_matrix(0, 3) << " "
         << odom_matrix(1, 3) << " " << odom_matrix(2, 3) << " " << quat.x()
         << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
  }
  fout.close();
}

void Map::SaveKeyFrameOdom(const string &filename) {
  std::vector<MCKeyFrame *> all_mc_keyfrms = GetAllMCKeyFrames();
  std::sort(all_mc_keyfrms.begin(), all_mc_keyfrms.end(), MCKeyFrame::lId);

  std::ofstream fout;
  fout.open(filename);
  fout.precision(20);
  for (auto mc_keyfrm : all_mc_keyfrms) {

    cv::Mat tcw = mc_keyfrm->GetPose();
    auto odom_matrix = Converter::toMatrix4d(
        MultiCamExt::GetInstance().CameraPose2Baselink(tcw, FRONT_CAMERA));

    Mat33_t rotation = odom_matrix.block(0, 0, 3, 3);
    Quat_t quat(rotation);

    fout << mc_keyfrm->timestamp_ << " " << odom_matrix(0, 3) << " "
         << odom_matrix(1, 3) << " " << odom_matrix(2, 3) << " " << quat.x()
         << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
  }
  fout.close();
}

void Map::SetBaKeyFrame(const std::vector<MCKeyFrame *> &local_mc_keyfrms,
                        const std::vector<MCKeyFrame *> &fixed_mc_keyfrms) {
  std::unique_lock<std::mutex> lock(mutex_ba_keyfrm_);
  ba_local_mckeyfrms_ = local_mc_keyfrms;
  ba_fixed_mc_keyfrms_ = fixed_mc_keyfrms;
}

std::vector<KeyFrame *> Map::GetBAKeyFramesFixed() {
  std::unique_lock<std::mutex> lock(mutex_ba_keyfrm_);

  std::vector<KeyFrame *> ba_fiexed_keyfrms;
  for (auto mc_keyfrm : ba_fixed_mc_keyfrms_) {
    for (auto keyfrm : mc_keyfrm->keyframes_) {
      ba_fiexed_keyfrms.push_back(keyfrm);
    }
  }
  return ba_fiexed_keyfrms;
}

std::vector<KeyFrame *> Map::GetBAKeyFrames() {
  std::unique_lock<std::mutex> lock(mutex_ba_keyfrm_);

  std::vector<KeyFrame *> ba_local_keyfrms;
  for (auto mc_keyfrm : ba_local_mckeyfrms_) {
    for (auto keyfrm : mc_keyfrm->keyframes_) {
      ba_local_keyfrms.push_back(keyfrm);
    }
  }
  return ba_local_keyfrms;
}
}
