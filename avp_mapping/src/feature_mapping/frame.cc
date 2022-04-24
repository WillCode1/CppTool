#include "frame.h"
#include "converter.h"
#include "orbmatcher.h"
#include <thread>

namespace FeatureSLAM {

long unsigned int Frame::next_id_ = 0;
bool Frame::mbInitialComputations = true;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame() { id_ = next_id_++; }

Frame::Frame(const double &timeStamp, ORBextractor *extractor,
             ORBVocabulary *voc)
    : orb_voc_(voc), orb_extractor_(extractor),

      timestamp_(timeStamp) {
  id_ = next_id_++;
  scale_levels_ = orb_extractor_->GetLevels();
  scale_factor_ = orb_extractor_->GetScaleFactor();
  log_scale_factor_ = log(scale_factor_);
  scale_factors_ = orb_extractor_->GetScaleFactors();
  inv_scale_factors_ = orb_extractor_->GetInverseScaleFactors();
  level_sigma2_ = orb_extractor_->GetScaleSigmaSquares();
  inv_level_sigma2_ = orb_extractor_->GetInverseScaleSigmaSquares();
}

Frame::Frame(const Frame &frame)
    : orb_voc_(frame.orb_voc_), orb_extractor_(frame.orb_extractor_),
      timestamp_(frame.timestamp_), num_feature_(frame.num_feature_),
      keypts_(frame.keypts_), bow_vec_(frame.bow_vec_),
      feat_vec_(frame.feat_vec_), descriptors_(frame.descriptors_.clone()),
      mappoints_(frame.mappoints_), outliers_(frame.outliers_), id_(frame.id_),
      scale_levels_(frame.scale_levels_), scale_factor_(frame.scale_factor_),
      log_scale_factor_(frame.log_scale_factor_),
      scale_factors_(frame.scale_factors_),
      inv_scale_factors_(frame.inv_scale_factors_),
      level_sigma2_(frame.level_sigma2_),
      inv_level_sigma2_(frame.inv_level_sigma2_) {
  for (int i = 0; i < FRAME_GRID_COLS; i++)
    for (int j = 0; j < FRAME_GRID_ROWS; j++)
      keypt_indices_in_cells_[i][j] = frame.keypt_indices_in_cells_[i][j];

  if (!frame.Tcw_.empty())
    SetPose(frame.Tcw_);
}

void Frame::Initialization() {

  num_feature_ = keypts_.size();
  if (keypts_.empty())
    return;
  mappoints_ = std::vector<MapPoint *>(num_feature_, nullptr);
  outliers_ = std::vector<bool>(num_feature_, false);
  if (mbInitialComputations) {
    ComputeImageBounds(kImgWidth, kImgUpBound - kImgLowBound);
    mfGridElementWidthInv =
        static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
    mfGridElementHeightInv =
        static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

    mbInitialComputations = false;
  }
  AssignFeaturesToGrid();
}
Frame::Frame(const cv::Mat &im_gray, const double &timestamp,
             ORBextractor *extractor, ORBVocabulary *voc)
    : orb_voc_(voc), orb_extractor_(extractor), timestamp_(timestamp) {
  // Frame ID
  id_ = next_id_++;

  // Scale Level Info
  scale_levels_ = orb_extractor_->GetLevels();
  scale_factor_ = orb_extractor_->GetScaleFactor();
  log_scale_factor_ = log(scale_factor_);
  scale_factors_ = orb_extractor_->GetScaleFactors();
  inv_scale_factors_ = orb_extractor_->GetInverseScaleFactors();
  level_sigma2_ = orb_extractor_->GetScaleSigmaSquares();
  inv_level_sigma2_ = orb_extractor_->GetInverseScaleSigmaSquares();

  // ORB extraction
  ExtractORB(im_gray);

  num_feature_ = keypts_.size();
  if (keypts_.empty())
    return;

  mappoints_ = std::vector<MapPoint *>(num_feature_, nullptr);
  outliers_ = std::vector<bool>(num_feature_, false);
  // This is done only for the first Frame (or after a change in the
  // calibration)
  if (mbInitialComputations) {
    ComputeImageBounds(im_gray.cols, im_gray.rows);

    mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) /
                            static_cast<float>(mnMaxX - mnMinX);
    mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) /
                             static_cast<float>(mnMaxY - mnMinY);
    mbInitialComputations = false;
  }

  AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid() {
  unsigned int num_reserve = static_cast<unsigned int>(
      0.5f * num_feature_ / (FRAME_GRID_COLS * FRAME_GRID_ROWS));
  for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
    for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
      keypt_indices_in_cells_[i][j].reserve(num_reserve);

  for (int i = 0; i < num_feature_; i++) {
    const cv::KeyPoint &kp = keypts_[i];
    int n_cell_pos_x, n_cell_pos_y;
    if (PosInCell(kp, n_cell_pos_x, n_cell_pos_y))
      keypt_indices_in_cells_[n_cell_pos_x][n_cell_pos_y].push_back(i);
  }
}

void Frame::ExtractORB(const cv::Mat &im) {
  (*orb_extractor_)(im, cv::Mat(), keypts_, descriptors_);
}

void Frame::SetPose(cv::Mat Tcw) {
  Tcw_ = Tcw.clone();
  UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices() {
  mRcw = Tcw_.rowRange(0, 3).colRange(0, 3);
  mRwc = mRcw.t();
  mtcw = Tcw_.rowRange(0, 3).col(3);
  mOw = -mRcw.t() * mtcw;
}

bool Frame::isInFrustum(MapPoint *mp, float viewing_cos_limit) {
  mp->is_tracked_in_view_ = false;
  // 3D in absolute coordinates
  cv::Mat pw = mp->GetWorldPos();
  // 3D in camera coordinates
  const cv::Mat pc = mRcw * pw + mtcw;
  const float &pc_x = pc.at<float>(0);
  const float &pc_y = pc.at<float>(1);
  const float &pc_z = pc.at<float>(2);
  // Check positive depth
  if (pc_z < 0.0f)
    return false;
  // Project in image and check it is not outside
  cv::Point2f image_point =
      Converter::ReprojectToImage(cv::Point3f(pc_x, pc_y, pc_z));
  const float u = image_point.x;
  const float v = image_point.y;

  if (u < mnMinX || u > mnMaxX)
    return false;
  if (v < mnMinY || v > mnMaxY)
    return false;

  // Check distance is in the scale invariance region of the MapPoint
  const float kMaxDistance = mp->GetMaxDistanceInvariance();
  const float minDistance = mp->GetMinDistanceInvariance();
  const cv::Mat po = pw - mOw;
  const float kDist = cv::norm(po);
  if (kDist < minDistance || kDist > kMaxDistance)
    return false;
  // Check viewing angle
  cv::Mat Pn = mp->GetNormal();
  const float viewCos = po.dot(Pn) / kDist;
  if (viewCos < viewing_cos_limit)
    return false;
  const int nPredictedLevel = mp->PredictScale(kDist, this);
  // Data used by the tracking
  mp->is_tracked_in_view_ = true;

  if (mp->track_cam_index1_ < 0) {
    mp->track_proj_x1_ = u;
    mp->track_proj_y1_ = v;
    mp->track_scale_level1_ = nPredictedLevel;
    mp->track_view_cos1_ = viewCos;

  } else {
    mp->rrack_proj_x2_ = u;
    mp->track_proj_y2_ = v;
    mp->track_scale_level2_ = nPredictedLevel;
    mp->track_view_cos2_ = viewCos;
  }

  return true;
}

std::vector<size_t> Frame::GetFeaturesInArea(const float &x, const float &y,
                                             const float &r,
                                             const int min_level,
                                             const int max_level) const {
  std::vector<size_t> indices;
  indices.reserve(num_feature_);

  const int nMinCellX =
      std::max(0, (int)floor((x - mnMinX - r) * mfGridElementWidthInv));
  if (nMinCellX >= FRAME_GRID_COLS)
    return indices;

  const int nMaxCellX =
      std::min((int)FRAME_GRID_COLS - 1,
               (int)ceil((x - mnMinX + r) * mfGridElementWidthInv));
  if (nMaxCellX < 0)
    return indices;

  const int nMinCellY =
      std::max(0, (int)floor((y - mnMinY - r) * mfGridElementHeightInv));
  if (nMinCellY >= FRAME_GRID_ROWS)
    return indices;

  const int nMaxCellY =
      std::min((int)FRAME_GRID_ROWS - 1,
               (int)ceil((y - mnMinY + r) * mfGridElementHeightInv));
  if (nMaxCellY < 0)
    return indices;

  const bool check_levels = (min_level > 0) || (max_level >= 0);

  for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
    for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
      const std::vector<size_t> vCell = keypt_indices_in_cells_[ix][iy];
      if (vCell.empty())
        continue;

      for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
        const cv::KeyPoint &keypt = keypts_[vCell[j]];
        if (check_levels) {
          if (keypt.octave < min_level)
            continue;
          if (max_level >= 0)
            if (keypt.octave > max_level)
              continue;
        }

        const float distx = keypt.pt.x - x;
        const float disty = keypt.pt.y - y;

        if (fabs(distx) < r && fabs(disty) < r)
          indices.push_back(vCell[j]);
      }
    }
  }

  return indices;
}

bool Frame::PosInCell(const cv::KeyPoint &kp, int &pos_x, int &pos_y) {
  pos_x = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
  pos_y = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

  // Keypoint's coordinates are undistorted, which could cause to go out of the
  // image
  if (pos_x < 0 || pos_x >= FRAME_GRID_COLS || pos_y < 0 ||
      pos_y >= FRAME_GRID_ROWS)
    return false;

  return true;
}

void Frame::ComputeBoW() {
  if (bow_vec_.empty()) {
    std::vector<cv::Mat> cur_desc = Converter::toDescriptorVector(descriptors_);
    orb_voc_->transform(cur_desc, bow_vec_, feat_vec_, 4);
  }
}

void Frame::ComputeImageBounds(const float &kImageWidth,
                               const float &kImageHeight) {
  mnMinX = 0.0f;
  mnMaxX = kImageWidth;
  mnMinY = 0.0f;
  mnMaxY = kImageHeight;
}
}
