#pragma once

#include "frame.h"
#include "keyframe.h"
#include "mappoint.h"
#include "mcframe.h"
#include "vslam_types.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

namespace FeatureSLAM {
class MapPoint;
class Frame;
class KeyFrame;
class ORBmatcher {
public:
  ORBmatcher(float nnratio = 0.6, bool checkOri = true);
  static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);
  int SearchByProjection(Frame &F, const std::vector<MapPoint *> &vpMapPoints,
                         const int nCamIndex, const float th);
  int SearchByProjection(Frame &F, const std::vector<MapPoint *> &vpMapPoints,
                         const float th);

  int SearchByProjection(KeyFrame *pKF,
                         const std::vector<MapPoint *> &vpMapPoints,
                         const float th = 3);
  int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame,
                         const float th, const bool bMono);
  int SearchByProjectionMultiThread(Frame &CurrentFrame, const Frame &LastFrame,
                                    const float th, int &nMatches);
  int SearchByProjection(KeyFrame *pKF, cv::Mat Scw,
                         const std::vector<MapPoint *> &vpPoints,
                         std::vector<MapPoint *> &vpMatched, int th);
  int SearchByBoW(KeyFrame *pKF, Frame &F,
                  std::vector<MapPoint *> &vpMapPointMatches);
  int SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2,
                  std::vector<MapPoint *> &vpMatches12);
  int SearchForInitialization(Frame &frame1, Frame &frame2,
                              std::vector<cv::Point2f> &prevmatched,
                              std::vector<int> &matches12,
                              int window_size = 10);
  int SearchForTriangulation(
      KeyFrame *keyfrm_1, KeyFrame *keyfrm_2, cv::Mat E12,
      std::vector<std::pair<size_t, size_t>> &vMatchedPairs);
  int SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2,
                   std::vector<MapPoint *> &vpMatches12, const float &s12,
                   const cv::Mat &R12, const cv::Mat &t12, const float th);
  int Fuse(KeyFrame *keyfrm, const std::vector<MapPoint *> &mappoints_to_check,
           const float th = 3.0);
  int Fuse(KeyFrame *pKF, cv::Mat Scw, const std::vector<MapPoint *> &vpPoints,
           float th, std::vector<MapPoint *> &vpReplacePoint);

public:
  static const int TH_LOW;
  static const int TH_HIGH;
  static const int HISTO_LENGTH;
  static const int MAX_HAMMING_DIST;

protected:
  bool CheckEpipolarConstraint(const Eigen::Vector3d &bearing_1,
                               const Eigen::Vector3d &bearing_2,
                               Eigen::Matrix3d &E_12,
                               const float bearing_1_scale_factor);

  float RadiusByViewingCos(const float &viewCos);

  void ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1,
                          int &ind2, int &ind3);

  float mfNNratio;
  bool mbCheckOrientation;
};
}
