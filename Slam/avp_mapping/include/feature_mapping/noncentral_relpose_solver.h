#pragma once

#include "keyframe.h"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opengv/absolute_pose/NoncentralAbsoluteMultiAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/sac/MultiRansac.hpp>
#include <opengv/sac_problems/absolute_pose/MultiNoncentralAbsolutePoseSacProblem.hpp>

namespace FeatureSLAM {

class NonCentralRelPoseSolver {
public:
  NonCentralRelPoseSolver(KeyFrame *pKF1, KeyFrame *pKF2,
                          const std::vector<MapPoint *> &vpMatched12,
                          KeyFrame *pKF1_, KeyFrame *pKF2_,
                          const std::vector<MapPoint *> &vpMatched12_);
  ~NonCentralRelPoseSolver();

  cv::Mat GetEstimatedRotation();
  cv::Mat GetEstimatedTranslation();
  float GetEstimatedScale();

  cv::Mat iterate(std::vector<bool> &inliers, std::vector<bool> &inliers_,
                  bool &bNoMore);

private:
  int numberCameras_;
  opengv::translation_t position_;
  opengv::rotation_t rotation_;
  cv::Mat best_rotation_;
  cv::Mat best_traslation_;
  opengv::translations_t camOffsets_;
  opengv::rotations_t camRotations_;
  cv::Mat matched_pose_;
  std::vector<unsigned int> vbMatched;
  std::vector<unsigned int> vbMatched_;

  std::vector<std::shared_ptr<opengv::points_t>> multiPoints;
  std::vector<std::shared_ptr<opengv::bearingVectors_t>> multiBearingVectors;

  unsigned int n_;
  unsigned int iterations_;
  unsigned int mInliers_;
};
}
