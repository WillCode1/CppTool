#pragma once

#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"
#include "keyframe.h"
#include "mappoint.h"
#include "orbextractor.h"
#include "orbvocabulary.h"
#include <opencv2/opencv.hpp>
#include <vector>

namespace FeatureSLAM {
#define FRAME_GRID_ROWS 37
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame {
public:
  Frame();
  Frame(const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc);
  void Initialization();
  inline void InitializeScaleLevels() {
    scale_levels_ = orb_extractor_->GetLevels();
    scale_factor_ = orb_extractor_->GetScaleFactor();
    log_scale_factor_ = log(scale_factor_);
    scale_factors_ = orb_extractor_->GetScaleFactors();
    inv_scale_factors_ = orb_extractor_->GetInverseScaleFactors();
    level_sigma2_ = orb_extractor_->GetScaleSigmaSquares();
    inv_level_sigma2_ = orb_extractor_->GetInverseScaleSigmaSquares();
  }
  // Copy constructor.
  Frame(const Frame &frame);
  Frame(const cv::Mat &im_gray, const double &timestamp,
        ORBextractor *extractor, ORBVocabulary *voc);

  void ExtractORB(const cv::Mat &im);
  void ComputeBoW();
  void SetPose(cv::Mat Tcw);
  void UpdatePoseMatrices();

  inline cv::Mat GetCameraCenter() { return mOw.clone(); }

  // Returns inverse of rotation
  inline cv::Mat GetRotationInverse() { return mRwc.clone(); }
  bool isInFrustum(MapPoint *mp, float viewing_cos_limit);

  bool PosInCell(const cv::KeyPoint &kp, int &pos_x, int &pos_y);
  std::vector<size_t> GetFeaturesInArea(const float &x, const float &y,
                                        const float &r,
                                        const int min_level = -1,
                                        const int max_level = -1) const;

public:
  ORBVocabulary *orb_voc_;
  ORBextractor *orb_extractor_;

  // Frame timestamp.
  double timestamp_;
  int num_feature_;

  std::vector<cv::KeyPoint> keypts_;

  DBoW2::BowVector bow_vec_;
  DBoW2::FeatureVector feat_vec_;
  cv::Mat descriptors_;
  std::vector<MapPoint *> mappoints_;
  std::vector<bool> outliers_;

  static float mfGridElementWidthInv;
  static float mfGridElementHeightInv;
  std::vector<std::size_t> keypt_indices_in_cells_[FRAME_GRID_COLS]
                                                  [FRAME_GRID_ROWS];

  // Camera pose.
  cv::Mat Tcw_;

  static long unsigned int next_id_; ///< Next Frame id.
  long unsigned int id_;             ///< Current Frame id.

  int scale_levels_;
  float scale_factor_;
  float log_scale_factor_; //
  std::vector<float> scale_factors_;
  std::vector<float> inv_scale_factors_;
  std::vector<float> level_sigma2_;
  std::vector<float> inv_level_sigma2_;

  static float mnMinX;
  static float mnMaxX;
  static float mnMinY;
  static float mnMaxY;

  static bool mbInitialComputations;
  void AssignFeaturesToGrid();

private:
  void ComputeImageBounds(const float &kImageWidth, const float &kImageHeight);
  // Rotation, translation and camera center
  cv::Mat mRcw;
  cv::Mat mtcw;
  cv::Mat mRwc;
  cv::Mat mOw;
};
}
