#include "orbmatcher.h"
#include "DBoW2/FeatureVector.h"
#include "colordef.h"
#include "converter.h"
#include <limits.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <stdint.h>

using namespace std;

namespace FeatureSLAM {

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;
const int ORBmatcher::MAX_HAMMING_DIST = 256;

/**
 * Constructor
 * @param nnratio  ratio of the best and the second score
 * @param checkOri check orientation
 */
ORBmatcher::ORBmatcher(float nnratio, bool checkOri)
    : mfNNratio(nnratio), mbCheckOrientation(checkOri) {}

int ORBmatcher::SearchByProjection(KeyFrame *pKF,
                                   const std::vector<MapPoint *> &vpMapPoints,
                                   const float th) {

  int nmatches = 0;
  const bool bFactor = th != 1.0;

  for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++) {
    MapPoint *pMP = vpMapPoints[iMP];
    if (!pMP->is_tracked_in_view_)
      continue;
    if (pMP->isBad())
      continue;
    const int &nPredictedLevel = pMP->track_scale_level1_;
    float r = RadiusByViewingCos(pMP->track_view_cos1_);
    if (bFactor)
      r *= th;

    cv::Mat camPoint =
        pKF->GetRotation() * pMP->GetWorldPos() + pKF->GetTranslation();
    if (camPoint.at<float>(2, 0) < 0)
      continue;

    cv::Point2f imgPoint = Converter::ReprojectToImage(
        cv::Point3f(camPoint.at<float>(0, 0), camPoint.at<float>(1, 0),
                    camPoint.at<float>(2, 0)));
    const vector<size_t> vIndices = pKF->GetFeaturesInArea(
        imgPoint.x, imgPoint.y, r * pKF->scale_factors_[nPredictedLevel]);
    if (vIndices.empty())
      continue;

    const cv::Mat MPdescriptor = pMP->GetDescriptor();

    int bestDist = 256;
    int bestLevel = -1;
    int bestDist2 = 256;
    int bestLevel2 = -1;
    int bestIdx = -1;

    // Get best and second matches with near keypoints
    for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                        vend = vIndices.end();
         vit != vend; vit++) {
      const size_t idx = *vit;

      if (pKF->mappoints_[idx])
        if (pKF->mappoints_[idx]->Observations() > 0)
          continue;

      const cv::Mat &d = pKF->desc_.row(idx);
      const int dist = DescriptorDistance(MPdescriptor, d);
      if (dist < bestDist) {
        bestDist2 = bestDist;
        bestDist = dist;
        bestLevel2 = bestLevel;
        bestLevel = pKF->keypts_[idx].octave;
        bestIdx = idx;
      } else if (dist < bestDist2) {
        bestLevel2 = pKF->keypts_[idx].octave;
        bestDist2 = dist;
      }
    }

    // Apply ratio to second match (only if best and second are in the same
    // scale level)
    if (bestDist <= TH_HIGH) {
      if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
        continue;

      pKF->mappoints_[bestIdx] = pMP; // ???Frame??????????????????????????????MapPoint
      pMP->AddObservation(pKF, bestIdx);
      pMP->ComputeDistinctiveDescriptors();
      pMP->UpdateNormalAndDepth();

      nmatches++;
    }
  }

  return nmatches;
}

int ORBmatcher::SearchByProjection(Frame &F,
                                   const vector<MapPoint *> &vpMapPoints,
                                   const int nCamIndex, const float th) {

  int nmatches = 0;

  const bool bFactor = th != 1.0;

  for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++) {
    MapPoint *pMP = vpMapPoints[iMP];
    if (!pMP->is_tracked_in_view_)
      continue;
    if (pMP->track_cam_index1_ < 0 && pMP->track_cam_index2_ < 0)
      continue;
    if (pMP->track_cam_index1_ != nCamIndex &&
        pMP->track_cam_index2_ != nCamIndex)
      continue;
    if (pMP->isBad())
      continue;

    bool bPrimary = (pMP->track_cam_index1_ == nCamIndex);
    //        const int &nPredictedLevel = pMP->mnTrackScaleLevel;
    //        float r = RadiusByViewingCos(pMP->mTrackViewCos);

    //        if(bFactor)
    //            r*=th;
    //        const vector<size_t> vIndices =
    //                F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel);

    int nPredictedLevel;
    float r;
    float ProjX, ProjY;
    if (bPrimary) {
      nPredictedLevel = pMP->track_scale_level1_;
      r = RadiusByViewingCos(pMP->track_view_cos1_);
      ProjX = pMP->track_proj_x1_;
      ProjY = pMP->track_proj_y1_;
    } else {
      nPredictedLevel = pMP->track_scale_level2_;
      r = RadiusByViewingCos(pMP->track_view_cos2_);
      ProjX = pMP->rrack_proj_x2_;
      ProjY = pMP->track_proj_y2_;
    }

    if (bFactor)
      r *= th;

    const vector<size_t> vIndices =
        F.GetFeaturesInArea(ProjX, ProjY, r * F.scale_factors_[nPredictedLevel],
                            nPredictedLevel - 1, nPredictedLevel);

    if (vIndices.empty())
      continue;

    const cv::Mat MPdescriptor = pMP->GetDescriptor();

    int bestDist = 256;
    int bestLevel = -1;
    int bestDist2 = 256;
    int bestLevel2 = -1;
    int bestIdx = -1;
    for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                        vend = vIndices.end();
         vit != vend; vit++) {
      const size_t idx = *vit;
      if (F.mappoints_[idx])
        if (F.mappoints_[idx]->Observations() > 0)
          continue;

      const cv::Mat &d = F.descriptors_.row(idx);
      const int dist = DescriptorDistance(MPdescriptor, d);
      if (dist < bestDist) {
        bestDist2 = bestDist;
        bestDist = dist;
        bestLevel2 = bestLevel;
        bestLevel = F.keypts_[idx].octave;
        bestIdx = idx;
      } else if (dist < bestDist2) {
        bestLevel2 = F.keypts_[idx].octave;
        bestDist2 = dist;
      }
    }
    if (bestDist <= TH_HIGH) {
      if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
        continue;
      F.mappoints_[bestIdx] = pMP; // ???Frame??????????????????????????????MapPoint
      nmatches++;
    }
  }
  return nmatches;
}

int ORBmatcher::SearchByProjection(Frame &F,
                                   const std::vector<MapPoint *> &vpMapPoints,
                                   const float th) {

  int nmatches = 0;
  const bool bFactor = th != 1.0;

  for (size_t i = 0; i < vpMapPoints.size(); i++) {
    MapPoint *pMP = vpMapPoints[i];
    pMP->is_tracked_in_view_ = false;
    pMP->track_cam_index1_ = NONE_CAMERA;
    if (F.isInFrustum(pMP, 0.5)) {
      pMP->IncreaseVisible();
    }
  }
  for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++) {
    MapPoint *pMP = vpMapPoints[iMP];
    if (pMP->isBad())
      continue;
    if (!pMP->is_tracked_in_view_)
      continue;
    const int &nPredictedLevel = pMP->track_scale_level1_;
    float r = RadiusByViewingCos(pMP->track_view_cos1_);

    if (bFactor)
      r *= th;
    const vector<size_t> vIndices =
        F.GetFeaturesInArea(pMP->track_proj_x1_, pMP->track_proj_y1_,
                            r * F.scale_factors_[nPredictedLevel],
                            nPredictedLevel - 1, nPredictedLevel);

    if (vIndices.empty())
      continue;
    const cv::Mat MPdescriptor = pMP->GetDescriptor();
    int bestDist = 256;
    int bestLevel = -1;
    int bestDist2 = 256;
    int bestLevel2 = -1;
    int bestIdx = -1;
    for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                        vend = vIndices.end();
         vit != vend; vit++) {
      const size_t idx = *vit;
      if (F.mappoints_[idx])
        if (F.mappoints_[idx]->Observations() > 0)
          continue;

      const cv::Mat &d = F.descriptors_.row(idx);
      const int dist = DescriptorDistance(MPdescriptor, d);
      if (dist < bestDist) {
        bestDist2 = bestDist;
        bestDist = dist;
        bestLevel2 = bestLevel;
        bestLevel = F.keypts_[idx].octave;
        bestIdx = idx;
      } else if (dist < bestDist2) {
        bestLevel2 = F.keypts_[idx].octave;
        bestDist2 = dist;
      }
    }
    if (bestDist <= TH_HIGH) {
      if (bestLevel == bestLevel2 && bestDist > mfNNratio * bestDist2)
        continue;

      F.mappoints_[bestIdx] = pMP; // ???Frame??????????????????????????????MapPoint
      nmatches++;
    }
  }
  return nmatches;
}

float ORBmatcher::RadiusByViewingCos(const float &viewCos) {
  if (viewCos > 0.998)
    return 2.5;
  else
    return 4.0;
}

bool ORBmatcher::CheckEpipolarConstraint(const Eigen::Vector3d &bearing_1,
                                         const Eigen::Vector3d &bearing_2,
                                         Eigen::Matrix3d &E_12,
                                         const float bearing_1_scale_factor) {
  const Eigen::Vector3d epiplane_in_1 = E_12 * bearing_2;

  const auto cos_residual = epiplane_in_1.dot(bearing_1) / epiplane_in_1.norm();
  const auto residual_rad = M_PI / 2.0 - std::abs(std::acos(cos_residual));

  constexpr double residual_deg_thr = 0.2;
  constexpr double residual_rad_thr = residual_deg_thr * M_PI / 180.0;

  return residual_rad < residual_rad_thr * bearing_1_scale_factor;
}

int ORBmatcher::SearchByBoW(KeyFrame *pKF, Frame &F,
                            vector<MapPoint *> &vpMapPointMatches) {
  const vector<MapPoint *> vpMapPointsKF = pKF->GetMapPointMatches();

  vpMapPointMatches =
      vector<MapPoint *>(F.num_feature_, static_cast<MapPoint *>(NULL));

  const DBoW2::FeatureVector &vFeatVecKF = pKF->featd_vec_;

  int nmatches = 0;

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++)
    rotHist[i].reserve(500);
  const float factor = HISTO_LENGTH / 360.0f;

  DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
  DBoW2::FeatureVector::const_iterator Fit = F.feat_vec_.begin();
  DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
  DBoW2::FeatureVector::const_iterator Fend = F.feat_vec_.end();

  while (KFit != KFend && Fit != Fend) {
    if (KFit->first ==
        Fit->first) //??????1???????????????????????????node???ORB?????????(??????????????????node???????????????????????????)
    {
      const vector<unsigned int> vIndicesKF = KFit->second;
      const vector<unsigned int> vIndicesF = Fit->second;

      // ??????2?????????KF????????????node????????????
      for (size_t iKF = 0; iKF < vIndicesKF.size(); iKF++) {
        const unsigned int realIdxKF = vIndicesKF[iKF];

        MapPoint *pMP =
            vpMapPointsKF[realIdxKF]; // ??????KF?????????????????????MapPoint

        if (!pMP)
          continue;

        if (pMP->isBad())
          continue;

        const cv::Mat &dKF =
            pKF->desc_.row(realIdxKF); // ??????KF??????????????????????????????

        int bestDist1 = 256; // ?????????????????????????????????
        int bestIdxF = -1;
        int bestDist2 = 256; // ????????????????????????????????????????????????

        // ??????3?????????F????????????node???????????????????????????????????????
        for (size_t iF = 0; iF < vIndicesF.size(); iF++) {
          const unsigned int realIdxF = vIndicesF[iF];

          if (vpMapPointMatches
                  [realIdxF]) // ??????????????????????????????????????????????????????????????????
            continue;

          const cv::Mat &dF =
              F.descriptors_.row(realIdxF); // ??????F??????????????????????????????

          const int dist = DescriptorDistance(dKF, dF); // ?????????????????????

          if (dist < bestDist1) // dist < bestDist1 < bestDist2?????????bestDist1
                                // bestDist2
          {
            bestDist2 = bestDist1;
            bestDist1 = dist;
            bestIdxF = realIdxF;
          } else if (dist <
                     bestDist2) // bestDist1 < dist < bestDist2?????????bestDist2
          {
            bestDist2 = dist;
          }
        }

        // ??????4??????????????? ??? ???????????????????????????
        if (bestDist1 <= TH_LOW) // ????????????????????????????????????
        {
          // trick!
          // ???????????????????????????????????????????????????????????????????????????
          if (static_cast<float>(bestDist1) <
              mfNNratio * static_cast<float>(bestDist2)) {
            //                        cv::Point2f ptKF =
            //                        pKF->mvKeysUn[realIdxKF].pt;
            //                        cv::Point2f ptF = F.mvKeysUn[bestIdxF].pt;
            //                        if(fabs(ptKF.y-ptF.y)<0.1*IMAGEWIDTH)
            {
              vpMapPointMatches[bestIdxF] = pMP;
              const cv::KeyPoint &kp = pKF->keypts_[realIdxKF];
              if (mbCheckOrientation) {
                // trick!
                // angle?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
                // ????????????????????????????????????????????????????????????????????????????????????????????????????????????
                float rot = kp.angle -
                            F.keypts_[bestIdxF].angle; // ??????????????????????????????
                if (rot < 0.0)
                  rot += 360.0f;
                int bin = round(rot * factor); // ???rot?????????bin???
                if (bin == HISTO_LENGTH)
                  bin = 0;
                assert(bin >= 0 && bin < HISTO_LENGTH);
                rotHist[bin].push_back(bestIdxF);
              }
              nmatches++;
            }
          }
        }
      }

      KFit++;
      Fit++;
    } else if (KFit->first < Fit->first) {
      KFit = vFeatVecKF.lower_bound(Fit->first);
    } else {
      Fit = F.feat_vec_.lower_bound(KFit->first);
    }
  }

  // ?????????????????????????????????
  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    // ??????rotHist?????????????????????index
    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      // ?????????????????????????????????????????????????????????????????????
      if (i == ind1 || i == ind2 || i == ind3)
        continue;

      // ?????????ind1 ind2 ind3????????????????????????
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        vpMapPointMatches[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
        nmatches--;
      }
    }
  }

  return nmatches;
}

int ORBmatcher::SearchByProjection(KeyFrame *pKF, cv::Mat Scw,
                                   const vector<MapPoint *> &vpPoints,
                                   vector<MapPoint *> &vpMatched, int th) {
  // Decompose Scw
  cv::Mat sRcw = Scw.rowRange(0, 3).colRange(0, 3);
  const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0))); // ??????????????????s
  cv::Mat Rcw = sRcw / scw;
  cv::Mat tcw =
      Scw.rowRange(0, 3).col(3) /
      scw; // pKF?????????????????????????????????pKF??????????????????????????????????????????pKF
  cv::Mat Ow =
      -Rcw.t() *
      tcw; // ?????????????????????pKF?????????????????????????????????????????????????????????pKF????????????????????????pKF?????????????????????
  set<MapPoint *> spAlreadyFound(vpMatched.begin(), vpMatched.end());
  spAlreadyFound.erase(static_cast<MapPoint *>(NULL));

  int nmatches = 0;
  for (int iMP = 0, iendMP = vpPoints.size(); iMP < iendMP; iMP++) {
    MapPoint *pMP = vpPoints[iMP];
    // Discard Bad MapPoints and already found
    if (pMP->isBad() || spAlreadyFound.count(pMP))
      continue;

    // Get 3D Coords.
    cv::Mat p3Dw = pMP->GetWorldPos();
    // Transform into Camera Coords.
    cv::Mat p3Dc = Rcw * p3Dw + tcw;
    // Depth must be positive
    if (p3Dc.at<float>(2) < 0.0)
      continue;

    const float PX = p3Dc.at<float>(0);
    const float PY = p3Dc.at<float>(1);
    const float PZ = p3Dc.at<float>(2);
    cv::Point2f imagePoint =
        Converter::ReprojectToImage(cv::Point3f(PX, PY, PZ));
    const float u = imagePoint.x;
    const float v = imagePoint.y;

    // Point must be inside the image
    if (!pKF->IsInImage(u, v))
      continue;

    // Depth must be inside the scale invariance region of the point
    // ?????????????????????????????????????????????
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    cv::Mat PO = p3Dw - Ow;
    const float dist = cv::norm(PO);

    if (dist < minDistance || dist > maxDistance)
      continue;

    // Viewing angle must be less than 60 deg
    //  No Need for viewding angle judgement
    //        cv::Mat Pn = pMP->GetNormal();

    //        if(PO.dot(Pn)<0.5*dist)
    //            continue;

    int nPredictedLevel = pMP->PredictScale(dist, pKF);

    // Search in a radius
    // ??????????????????????????????
    const float radius = th * pKF->scale_factors_[nPredictedLevel];

    const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);

    if (vIndices.empty())
      continue;

    // Match to the most similar keypoint in the radius
    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = 256;
    int bestIdx = -1;
    // ?????????????????????????????????????????????MapPoint????????????????????????
    for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                        vend = vIndices.end();
         vit != vend; vit++) {
      const size_t idx = *vit;
      if (vpMatched[idx])
        continue;

      const int &kpLevel = pKF->keypts_[idx].octave;

      if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
        continue;

      const cv::Mat &dKF = pKF->desc_.row(idx);

      const int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    // ???MapPoint???bestIdx??????????????????????????????
    if (bestDist <= TH_LOW) {
      vpMatched[bestIdx] = pMP;
      nmatches++;
    }
  }

  return nmatches;
}

int ORBmatcher::SearchForInitialization(Frame &frame1, Frame &frame2,
                                        vector<cv::Point2f> &prevmatched,
                                        vector<int> &matches12,
                                        int window_size) {
  int nmatches = 0;
  matches12 = vector<int>(frame1.keypts_.size(), -1);

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++)
    rotHist[i].reserve(500);
  const float factor = HISTO_LENGTH / 360.0f;

  vector<int> vMatchedDistance(frame2.keypts_.size(), INT_MAX);
  vector<int> vnMatches21(frame2.keypts_.size(), -1);

  for (size_t i1 = 0, iend1 = frame1.keypts_.size(); i1 < iend1; i1++) {
    cv::KeyPoint kp1 = frame1.keypts_[i1];
    int level1 = kp1.octave;
    if (level1 > 0)
      continue;

    vector<size_t> vIndices2 = frame2.GetFeaturesInArea(
        prevmatched[i1].x, prevmatched[i1].y, window_size, level1, level1);

    if (vIndices2.empty())
      continue;

    cv::Mat d1 = frame1.descriptors_.row(i1);

    int bestDist = INT_MAX;
    int bestDist2 = INT_MAX;
    int bestIdx2 = -1;

    for (vector<size_t>::iterator vit = vIndices2.begin();
         vit != vIndices2.end(); vit++) {
      size_t i2 = *vit;

      cv::Mat d2 = frame2.descriptors_.row(i2);

      int dist = DescriptorDistance(d1, d2);

      if (vMatchedDistance[i2] <= dist)
        continue;

      if (dist < bestDist) {
        bestDist2 = bestDist;
        bestDist = dist;
        bestIdx2 = i2;
      } else if (dist < bestDist2) {
        bestDist2 = dist;
      }
    }

    // ??????SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*>
    // &vpMapPointMatches)????????????4
    if (bestDist <= TH_LOW) {
      if (bestDist < (float)bestDist2 * mfNNratio) {
        if (vnMatches21[bestIdx2] >= 0) {
          matches12[vnMatches21[bestIdx2]] = -1;
          nmatches--;
        }
        matches12[i1] = bestIdx2;
        vnMatches21[bestIdx2] = i1;
        vMatchedDistance[bestIdx2] = bestDist;
        nmatches++;

        if (mbCheckOrientation) {
          float rot = frame1.keypts_[i1].angle - frame2.keypts_[bestIdx2].angle;
          if (rot < 0.0)
            rot += 360.0f;
          int bin = round(rot * factor);
          if (bin == HISTO_LENGTH)
            bin = 0;
          assert(bin >= 0 && bin < HISTO_LENGTH);
          rotHist[bin].push_back(i1);
        }
      }
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3)
        continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        int idx1 = rotHist[i][j];
        if (matches12[idx1] >= 0) {
          matches12[idx1] = -1;
          nmatches--;
        }
      }
    }
  }

  // Update prev matched
  for (size_t i1 = 0, iend1 = matches12.size(); i1 < iend1; i1++)
    if (matches12[i1] >= 0)
      prevmatched[i1] = frame2.keypts_[matches12[i1]].pt;

  return nmatches;
}

/**
 * @brief
 * ????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
 *
 * ??????bow???pKF???F???????????????????????????????????????????????????node?????????????????????????????????
 * \n
 * ???????????????node????????????????????????????????????????????? \n
 * ?????????????????????vpMatches12 \n
 * ?????????????????????????????????????????????????????????????????????
 * @param  pKF1               KeyFrame1
 * @param  pKF2               KeyFrame2
 * @param  vpMatches12        pKF2??????pKF1?????????MapPoint???null??????????????????
 * @return                    ?????????????????????
 */
int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2,
                            vector<MapPoint *> &vpMatches12) {
  // ????????????????????????SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*>
  // &vpMapPointMatches)

  const vector<cv::KeyPoint> &vKeysUn1 = pKF1->keypts_;
  const DBoW2::FeatureVector &vFeatVec1 = pKF1->featd_vec_;
  const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
  const cv::Mat &Descriptors1 = pKF1->desc_;

  const vector<cv::KeyPoint> &vKeysUn2 = pKF2->keypts_;
  const DBoW2::FeatureVector &vFeatVec2 = pKF2->featd_vec_;
  const vector<MapPoint *> vpMapPoints2 = pKF2->GetMapPointMatches();
  const cv::Mat &Descriptors2 = pKF2->desc_;

  vpMatches12 =
      vector<MapPoint *>(vpMapPoints1.size(), static_cast<MapPoint *>(NULL));
  vector<bool> vbMatched2(vpMapPoints2.size(), false);

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++)
    rotHist[i].reserve(500);

  const float factor = HISTO_LENGTH / 360.0f;

  int nmatches = 0;

  DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
  DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
  DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
  DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

  while (f1it != f1end && f2it != f2end) {
    if (f1it->first ==
        f2it->first) //??????1???????????????????????????node???ORB?????????(??????????????????node???????????????????????????)
    {
      // ??????2?????????KF????????????node????????????
      for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
        const size_t idx1 = f1it->second[i1];

        MapPoint *pMP1 = vpMapPoints1[idx1];
        if (!pMP1)
          continue;
        if (pMP1->isBad())
          continue;

        const cv::Mat &d1 = Descriptors1.row(idx1);

        int bestDist1 = 256;
        int bestIdx2 = -1;
        int bestDist2 = 256;

        // ??????3?????????F????????????node???????????????????????????????????????
        for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
          const size_t idx2 = f2it->second[i2];

          MapPoint *pMP2 = vpMapPoints2[idx2];

          if (vbMatched2[idx2] || !pMP2)
            continue;

          if (pMP2->isBad())
            continue;

          const cv::Mat &d2 = Descriptors2.row(idx2);

          int dist = DescriptorDistance(d1, d2);

          if (dist < bestDist1) {
            bestDist2 = bestDist1;
            bestDist1 = dist;
            bestIdx2 = idx2;
          } else if (dist < bestDist2) {
            bestDist2 = dist;
          }
        }

        // ??????4??????????????? ??? ???????????????????????????
        // ??????SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*>
        // &vpMapPointMatches)????????????4
        if (bestDist1 < TH_LOW) {
          if (static_cast<float>(bestDist1) <
              mfNNratio * static_cast<float>(bestDist2)) {
            vpMatches12[idx1] = vpMapPoints2[bestIdx2];
            vbMatched2[bestIdx2] = true;

            if (mbCheckOrientation) {
              float rot = vKeysUn1[idx1].angle - vKeysUn2[bestIdx2].angle;
              if (rot < 0.0)
                rot += 360.0f;
              int bin = round(rot * factor);
              if (bin == HISTO_LENGTH)
                bin = 0;
              assert(bin >= 0 && bin < HISTO_LENGTH);
              rotHist[bin].push_back(idx1);
            }
            nmatches++;
          }
        }
      }

      f1it++;
      f2it++;
    } else if (f1it->first < f2it->first) {
      f1it = vFeatVec1.lower_bound(f2it->first);
    } else {
      f2it = vFeatVec2.lower_bound(f1it->first);
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3)
        continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        vpMatches12[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
        nmatches--;
      }
    }
  }

  return nmatches;
}

int ORBmatcher::SearchForTriangulation(
    KeyFrame *keyfrm_1, KeyFrame *keyfrm_2, cv::Mat E12,
    std::vector<std::pair<size_t, size_t>> &matched_pairs) {

  int num_matches = 0;

  Eigen::Matrix3d E12_matrix = Converter::toMatrix3d(E12);

  const DBoW2::FeatureVector &feat_vec1 = keyfrm_1->featd_vec_;
  const DBoW2::FeatureVector &feat_vec2 = keyfrm_2->featd_vec_;

  // Compute epipole in second image

  cv::Mat cam_center_1 = keyfrm_1->GetCameraCenter(); // twc1
  cv::Mat rot_2w = keyfrm_2->GetRotation();           // Rc2w
  cv::Mat trans_2w = keyfrm_2->GetTranslation();      // tc2w
  cv::Mat epiplane_in_keyfrm_2 = rot_2w * cam_center_1 + trans_2w;
  epiplane_in_keyfrm_2 /= cv::norm(epiplane_in_keyfrm_2);

  const float epiplane_in_keyfrm_2_x = epiplane_in_keyfrm_2.at<float>(0);
  const float epiplane_in_keyfrm_2_y = epiplane_in_keyfrm_2.at<float>(1);
  const float epiplane_in_keyfrm_2_z = epiplane_in_keyfrm_2.at<float>(2);

  vector<bool> is_already_matched_in_keyfrm_2(keyfrm_2->num_feature_, false);
  vector<int> matched_indices_2_in_keyfrm_1(keyfrm_1->num_feature_, -1);

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++)
    rotHist[i].reserve(500);

  const float factor = HISTO_LENGTH / 360.0f;

  DBoW2::FeatureVector::const_iterator f1it = feat_vec1.begin();
  DBoW2::FeatureVector::const_iterator f2it = feat_vec2.begin();
  DBoW2::FeatureVector::const_iterator f1end = feat_vec1.end();
  DBoW2::FeatureVector::const_iterator f2end = feat_vec2.end();

  while (f1it != f1end && f2it != f2end) {
    // ??????f1it???f2it???????????????node??????
    if (f1it->first == f2it->first) {
      // ??????2????????????node?????????(f1it->first)??????????????????
      for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
        const size_t idx1 = f1it->second[i1];
        MapPoint *mp1 = keyfrm_1->GetMapPoint(idx1);

        // Already got matched, skip
        if (mp1)
          continue;

        const cv::KeyPoint &keypt_1 = keyfrm_1->keypts_[idx1];
        auto bearing_1 = Converter::KeypointToBearing(keypt_1);
        const cv::Mat &desc_1 = keyfrm_1->desc_.row(idx1);

        int best_hamm_dist = TH_LOW;
        int best_idx_2 = -1;

        for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
          size_t idx2 = f2it->second[i2];

          MapPoint *mp2 = keyfrm_2->GetMapPoint(idx2);
          if (is_already_matched_in_keyfrm_2[idx2] || mp2)
            continue;

          // ??????3.2????????????????????????idx2???pKF2???????????????????????????????????????
          const cv::Mat &desc_2 = keyfrm_2->desc_.row(idx2);
          const int hamm_dist = DescriptorDistance(desc_1, desc_2);

          if (hamm_dist > TH_LOW || hamm_dist > best_hamm_dist)
            continue;

          // ??????3.3????????????????????????idx2???pKF2???????????????????????????
          const cv::KeyPoint &keypt_2 = keyfrm_2->keypts_[idx2];
          auto bearing_2 = Converter::KeypointToBearing(keypt_2);

          // monocular
          {

            float cos_dist = std::sqrt(epiplane_in_keyfrm_2_x * bearing_2.x() +
                                       epiplane_in_keyfrm_2_y * bearing_2.y() +
                                       epiplane_in_keyfrm_2_z * bearing_2.z());

            constexpr double cos_dist_thr = 0.99862953475;

            if (cos_dist_thr < cos_dist) {
              continue;
            }
          }

          if (CheckEpipolarConstraint(
                  bearing_1, bearing_2, E12_matrix,
                  keyfrm_1->scale_factors_[keypt_1.octave])) {
            best_idx_2 = idx2;
            best_hamm_dist = hamm_dist;
          }
        }

        if (best_idx_2 >= 0) {
          const cv::KeyPoint &kp2 = keyfrm_2->keypts_[best_idx_2];
          matched_indices_2_in_keyfrm_1[idx1] = best_idx_2;
          is_already_matched_in_keyfrm_2[best_idx_2] = true;
          num_matches++;

          if (mbCheckOrientation) {
            float rot = keypt_1.angle - kp2.angle;
            if (rot < 0.0)
              rot += 360.0f;
            int bin = round(rot * factor);
            if (bin == HISTO_LENGTH)
              bin = 0;
            assert(bin >= 0 && bin < HISTO_LENGTH);
            rotHist[bin].push_back(idx1);
          }
        }
      }

      f1it++;
      f2it++;
    } else if (f1it->first < f2it->first) {
      f1it = feat_vec1.lower_bound(f2it->first);
    } else {
      f2it = feat_vec2.lower_bound(f1it->first);
    }
  }

  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i == ind1 || i == ind2 || i == ind3)
        continue;
      for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
        matched_indices_2_in_keyfrm_1[rotHist[i][j]] = -1;
        num_matches--;
      }
    }
  }

  matched_pairs.clear();
  matched_pairs.reserve(num_matches);

  for (size_t i = 0, iend = matched_indices_2_in_keyfrm_1.size(); i < iend;
       i++) {
    if (matched_indices_2_in_keyfrm_1[i] < 0)
      continue;
    matched_pairs.push_back(make_pair(i, matched_indices_2_in_keyfrm_1[i]));
  }

  return num_matches;
}

int ORBmatcher::Fuse(KeyFrame *keyfrm,
                     const std::vector<MapPoint *> &mappoints_to_check,
                     const float th) {
  const Vec3_t cam_center = Converter::toVector3d(keyfrm->GetCameraCenter());
  const Mat33_t rot_cw = Converter::toMatrix3d(keyfrm->GetRotation());
  const Vec3_t trans_cw = Converter::toVector3d(keyfrm->GetTranslation());

  int num_fused = 0;

  for (auto mp : mappoints_to_check) {
    if (!mp)
      continue;

    if (mp->isBad() || mp->IsInKeyFrame(keyfrm))
      continue;

    //    Converter::repro

    const Vec3_t pos_w = Converter::toVector3d(mp->GetWorldPos());

    const Vec3_t pos_c = rot_cw * pos_w + trans_cw;

    if (pos_c.z() < 0.0f)
      continue;

    Vec2_t reproj;

    if (!Converter::ReprojectToImage(rot_cw, trans_cw, pos_w, reproj))
      continue;

    const float u = reproj.x();
    const float v = reproj.y();

    // Point must be inside the image
    if (!keyfrm->IsInImage(u, v))
      continue;
    const float kMaxValidDistance = mp->GetMaxDistanceInvariance();
    const float kMinValidDistance = mp->GetMinDistanceInvariance();
    Vec3_t cam_to_mp_vec = pos_w - cam_center;
    const float kCamToMpDist = cam_to_mp_vec.norm();

    // Depth must be inside the scale pyramid of the image
    if (kCamToMpDist < kMinValidDistance || kCamToMpDist > kMaxValidDistance)
      continue;

    // Viewing angle must be less than 60 deg
    Vec3_t obs_mean_normal = Converter::toVector3d(mp->GetNormal());

    if (cam_to_mp_vec.dot(obs_mean_normal) < 0.5 * kCamToMpDist)
      continue;

    int pred_scale_level = mp->PredictScale(kCamToMpDist, keyfrm);

    // Search in a radius
    const float radius = th * keyfrm->scale_factors_[pred_scale_level];

    const vector<size_t> indices = keyfrm->GetFeaturesInArea(u, v, radius);

    if (indices.empty())
      continue;

    // Match to the most similar keypoint in the radius

    const cv::Mat mp_desc = mp->GetDescriptor();

    int best_dist = MAX_HAMMING_DIST;
    int best_idx = -1;

    for (auto idx : indices) {
      const cv::KeyPoint &kp = keyfrm->keypts_[idx];
      const int &scale_level = kp.octave;

      // check scale
      if (scale_level < pred_scale_level - 1 || scale_level > pred_scale_level)
        continue;

      const float ex = u - kp.pt.x;
      const float ey = v - kp.pt.y;
      const float reproj_error_sq = ex * ex + ey * ey;
      constexpr float chi_sq_2D = 5.99146;

      if (reproj_error_sq * keyfrm->inv_level_sigma2_[scale_level] > chi_sq_2D)
        continue;

      const cv::Mat &desc = keyfrm->desc_.row(idx);
      const int hamm_dist = DescriptorDistance(mp_desc, desc);

      if (hamm_dist < best_dist) {
        best_dist = hamm_dist;
        best_idx = idx;
      }
    }

    // found matched keypoint
    if (best_dist <= TH_LOW) {
      MapPoint *mp_in_keyfrm = keyfrm->GetMapPoint(best_idx);
      if (mp_in_keyfrm) {
        if (!mp_in_keyfrm->isBad()) // ????????????MapPoint??????bad????????????????????????
        {
          if (mp_in_keyfrm->Observations() > mp->Observations())
            mp->Replace(mp_in_keyfrm);
          else
            mp_in_keyfrm->Replace(mp);
        }
      } else {
        mp->AddObservation(keyfrm, best_idx);
        keyfrm->AddMapPoint(mp, best_idx);
      }
      num_fused++;
    }
  }
  return num_fused;
}

int ORBmatcher::Fuse(KeyFrame *pKF, cv::Mat Scw,
                     const std::vector<MapPoint *> &vpPoints, float th,
                     std::vector<MapPoint *> &vpReplacePoint) {
  cv::Mat sRcw = Scw.rowRange(0, 3).colRange(0, 3);
  const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0))); // ??????????????????s
  cv::Mat Rcw = sRcw / scw;                             // ??????s
  cv::Mat tcw = Scw.rowRange(0, 3).col(3) / scw;        // ??????s
  cv::Mat Ow = -Rcw.t() * tcw;

  // Set of MapPoints already found in the KeyFrame
  const set<MapPoint *> spAlreadyFound = pKF->GetMapPoints();

  int nFused = 0;

  const int nPoints = vpPoints.size();

  // For each candidate MapPoint project and match
  // ???????????????MapPoints
  for (int iMP = 0; iMP < nPoints; iMP++) {
    MapPoint *pMP = vpPoints[iMP];

    // Discard Bad MapPoints and already found
    if (pMP->isBad() || spAlreadyFound.count(pMP))
      continue;

    // Get 3D Coords.
    cv::Mat p3Dw = pMP->GetWorldPos();

    // Transform into Camera Coords.
    cv::Mat p3Dc = Rcw * p3Dw + tcw;
    //         Depth must be positive
    if (p3Dc.at<float>(2) < 0.0f)
      continue;

    const float PX = p3Dc.at<float>(0);
    const float PY = p3Dc.at<float>(1);
    const float PZ = p3Dc.at<float>(2);
    cv::Point2f imagePoint =
        Converter::ReprojectToImage(cv::Point3f(PX, PY, PZ));
    const float u = imagePoint.x;
    const float v = imagePoint.y;

    // Point must be inside the image
    if (!pKF->IsInImage(u, v))
      continue;

    // Depth must be inside the scale pyramid of the image
    // ????????????????????????????????????????????????????????????????????????????????????60????????????MapPoint????????????
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    cv::Mat PO = p3Dw - Ow;
    const float dist3D = cv::norm(PO);

    if (dist3D < minDistance || dist3D > maxDistance)
      continue;

    // Viewing angle must be less than 60 deg
    // No need for viewing angule judgement
    cv::Mat Pn = pMP->GetNormal();
    if (PO.dot(Pn) < 0.5 * dist3D)
      continue;

    // Compute predicted scale level
    const int nPredictedLevel = pMP->PredictScale(dist3D, pKF);

    // Search in a radius
    // ??????????????????
    const float radius = th * pKF->scale_factors_[nPredictedLevel];

    // ??????pKF???????????????????????????
    const vector<size_t> vIndices = pKF->GetFeaturesInArea(u, v, radius);

    if (vIndices.empty())
      continue;

    // Match to the most similar keypoint in the radius

    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = INT_MAX;
    int bestIdx = -1;
    for (vector<size_t>::const_iterator vit = vIndices.begin();
         vit != vIndices.end(); vit++) {
      const size_t idx = *vit;
      const int &kpLevel = pKF->keypts_[idx].octave;

      if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
        continue;

      const cv::Mat &dKF = pKF->desc_.row(idx);

      int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    // If there is already a MapPoint replace otherwise add new measurement
    if (bestDist <= TH_LOW) {
      MapPoint *pMPinKF = pKF->GetMapPoint(bestIdx);
      // ????????????MapPoint???????????????????????????
      // ?????????????????????????????????MapPoint?????????????????????????????????????????????????????????????????????????????????Replace???????????????
      if (pMPinKF) {
        if (!pMPinKF->isBad())
          vpReplacePoint[iMP] = pMPinKF; // ???????????????pMPinKF??????????????????
      } else // ????????????MapPoint????????????????????????
      {
        pMP->AddObservation(pKF, bestIdx);
        pKF->AddMapPoint(pMP, bestIdx);
      }
      nFused++;
    }
  }

  return nFused;
}

// ??????Sim3???????????????pKF1???????????????pKF2????????????????????????????????????pKF2???????????????pKF1??????????????????
// ????????????????????????????????????????????????pKF1???pKF2????????????????????????????????????vpMatches12???????????????SearchByBoW??????????????????????????????????????????
int ORBmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2,
                             vector<MapPoint *> &vpMatches12, const float &s12,
                             const cv::Mat &R12, const cv::Mat &t12,
                             const float th) {
  // ??????1??????????????????
  // Camera 1 from world
  // ???world???camera?????????
  cv::Mat R1w = pKF1->GetRotation();
  cv::Mat t1w = pKF1->GetTranslation();

  // Camera 2 from world
  cv::Mat R2w = pKF2->GetRotation();
  cv::Mat t2w = pKF2->GetTranslation();

  // Transformation between cameras
  cv::Mat sR12 = s12 * R12;
  cv::Mat sR21 = (1.0 / s12) * R12.t();
  cv::Mat t21 = -sR21 * t12;

  const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
  const int N1 = vpMapPoints1.size();

  const vector<MapPoint *> vpMapPoints2 = pKF2->GetMapPointMatches();
  const int N2 = vpMapPoints2.size();

  vector<bool> vbAlreadyMatched1(N1, false); // ??????????????????????????????????????????
  vector<bool> vbAlreadyMatched2(N2,
                                 false); // ?????????????????????????????????pKF1????????????

  // ??????2??????vpMatches12??????vbAlreadyMatched1???vbAlreadyMatched2
  for (int i = 0; i < N1; i++) {
    MapPoint *pMP = vpMatches12[i];
    if (pMP) {
      vbAlreadyMatched1[i] = true; // ???????????????????????????
      int idx2 = pMP->GetIndexInKeyFrame(pKF2);
      if (idx2 >= 0 && idx2 < N2)
        vbAlreadyMatched2[idx2] = true; // ???????????????pKF1????????????
    }
  }

  vector<int> vnMatch1(N1, -1);
  vector<int> vnMatch2(N2, -1);

  // Transform from KF1 to KF2 and search
  // ??????3.1?????????Sim???????????????pKF1???????????????pKF2?????????????????????
  //         ????????????????????????????????????????????????pKF1???pKF2????????????????????????????????????vpMatches12
  //         ???????????????SearchByBoW??????????????????????????????????????????
  for (int i1 = 0; i1 < N1; i1++) {
    MapPoint *pMP = vpMapPoints1[i1];

    if (!pMP || vbAlreadyMatched1[i1]) // ????????????????????????????????????????????????
      continue;

    if (pMP->isBad())
      continue;

    cv::Mat p3Dw = pMP->GetWorldPos();
    cv::Mat p3Dc1 = R1w * p3Dw +
                    t1w; // ???pKF1?????????MapPoint???world??????????????????camera1?????????
    cv::Mat p3Dc2 = sR21 * p3Dc1 +
                    t21; // ?????????Sim3??????MapPoint???camera1?????????camera2?????????

    // Depth must be positive
    //        if(p3Dc2.at<float>(2)<0.0)
    //            continue;

    // ?????????camera2????????????
    //        const float invz = 1.0/p3Dc2.at<float>(2);
    //        const float x = p3Dc2.at<float>(0)*invz;
    //        const float y = p3Dc2.at<float>(1)*invz;

    //        const float u = fx*x+cx;
    //        const float v = fy*y+cy;

    const float PX = p3Dc2.at<float>(0);
    const float PY = p3Dc2.at<float>(1);
    const float PZ = p3Dc2.at<float>(2);
    cv::Point2f imagePoint =
        Converter::ReprojectToImage(cv::Point3f(PX, PY, PZ));
    const float u = imagePoint.x;
    const float v = imagePoint.y;

    // Point must be inside the image
    if (!pKF2->IsInImage(u, v))
      continue;

    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const float dist3D = cv::norm(p3Dc2);

    // Depth must be inside the scale invariance region
    if (dist3D < minDistance || dist3D > maxDistance)
      continue;

    // Compute predicted octave
    // ?????????MapPoint?????????????????????????????????????????????
    const int nPredictedLevel = pMP->PredictScale(dist3D, pKF2);

    // Search in a radius
    // ???????????????????????????
    const float radius = th * pKF2->scale_factors_[nPredictedLevel];

    // ????????????????????????????????????
    const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u, v, radius);

    if (vIndices.empty())
      continue;

    // Match to the most similar keypoint in the radius
    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = INT_MAX;
    int bestIdx = -1;
    // ?????????????????????????????????????????????pMP?????????????????????
    for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                        vend = vIndices.end();
         vit != vend; vit++) {
      const size_t idx = *vit;

      const cv::KeyPoint &kp = pKF2->keypts_[idx];

      if (kp.octave < nPredictedLevel - 1 || kp.octave > nPredictedLevel)
        continue;

      const cv::Mat &dKF = pKF2->desc_.row(idx);

      const int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    if (bestDist <= TH_HIGH) {
      vnMatch1[i1] = bestIdx;
    }
  }

  // Transform from KF2 to KF1 and search
  // ??????3.2?????????Sim???????????????pKF2???????????????pKF1?????????????????????
  //         ????????????????????????????????????????????????pKF1???pKF2????????????????????????????????????vpMatches12
  //         ???????????????SearchByBoW??????????????????????????????????????????
  for (int i2 = 0; i2 < N2; i2++) {
    MapPoint *pMP = vpMapPoints2[i2];

    if (!pMP || vbAlreadyMatched2[i2])
      continue;

    if (pMP->isBad())
      continue;

    cv::Mat p3Dw = pMP->GetWorldPos();
    cv::Mat p3Dc2 = R2w * p3Dw + t2w;
    cv::Mat p3Dc1 = sR12 * p3Dc2 + t12;

    // Depth must be positive
    //        if(p3Dc1.at<float>(2)<0.0)
    //            continue;

    //        const float invz = 1.0/p3Dc1.at<float>(2);
    //        const float x = p3Dc1.at<float>(0)*invz;
    //        const float y = p3Dc1.at<float>(1)*invz;

    //        const float u = fx*x+cx;
    //        const float v = fy*y+cy;

    const float PX = p3Dc1.at<float>(0);
    const float PY = p3Dc1.at<float>(1);
    const float PZ = p3Dc1.at<float>(2);
    cv::Point2f imagePoint =
        Converter::ReprojectToImage(cv::Point3f(PX, PY, PZ));
    const float u = imagePoint.x;
    const float v = imagePoint.y;

    // Point must be inside the image
    if (!pKF1->IsInImage(u, v))
      continue;

    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const float dist3D = cv::norm(p3Dc1);

    // Depth must be inside the scale pyramid of the image
    if (dist3D < minDistance || dist3D > maxDistance)
      continue;

    // Compute predicted octave
    const int nPredictedLevel = pMP->PredictScale(dist3D, pKF1);

    // Search in a radius of 2.5*sigma(ScaleLevel)
    const float radius = th * pKF1->scale_factors_[nPredictedLevel];

    const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u, v, radius);

    if (vIndices.empty())
      continue;

    // Match to the most similar keypoint in the radius
    const cv::Mat dMP = pMP->GetDescriptor();

    int bestDist = INT_MAX;
    int bestIdx = -1;
    for (vector<size_t>::const_iterator vit = vIndices.begin(),
                                        vend = vIndices.end();
         vit != vend; vit++) {
      const size_t idx = *vit;

      const cv::KeyPoint &kp = pKF1->keypts_[idx];

      if (kp.octave < nPredictedLevel - 1 || kp.octave > nPredictedLevel)
        continue;

      const cv::Mat &dKF = pKF1->desc_.row(idx);

      const int dist = DescriptorDistance(dMP, dKF);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }

    if (bestDist <= TH_HIGH) {
      vnMatch2[i2] = bestIdx;
    }
  }

  // Check agreement
  int nFound = 0;

  for (int i1 = 0; i1 < N1; i1++) {
    int idx2 = vnMatch1[i1];

    if (idx2 >= 0) {
      int idx1 = vnMatch2[idx2];
      if (idx1 == i1) {
        vpMatches12[i1] = vpMapPoints2[idx2];
        nFound++;
      }
    }
  }
  return nFound;
}

int ORBmatcher::SearchByProjectionMultiThread(Frame &CurrentFrame,
                                              const Frame &LastFrame,
                                              const float th, int &nMatches) {
  nMatches = 0;

  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++)
    rotHist[i].reserve(500);
  const float factor = HISTO_LENGTH / 360.0f;

  const cv::Mat Rcw = CurrentFrame.Tcw_.rowRange(0, 3).colRange(0, 3);
  const cv::Mat tcw = CurrentFrame.Tcw_.rowRange(0, 3).col(3);

  const cv::Mat twc = -Rcw.t() * tcw; // twc(w)

  const cv::Mat Rlw = LastFrame.Tcw_.rowRange(0, 3).colRange(0, 3);
  const cv::Mat tlw = LastFrame.Tcw_.rowRange(0, 3).col(3); // tlw(l)

  const cv::Mat tlc =
      Rlw * twc + tlw; // Rlw*twc(w) = twc(l), twc(l) + tlw(l) = tlc(l)

  for (int i = 0; i < LastFrame.num_feature_; i++) {
    MapPoint *pMP = LastFrame.mappoints_[i];

    if (pMP) {
      if (!LastFrame.outliers_[i]) {
        cv::Mat x3Dw = pMP->GetWorldPos();
        cv::Mat x3Dc = Rcw * x3Dw + tcw;

        const float xc = x3Dc.at<float>(0);
        const float yc = x3Dc.at<float>(1);
        const float zc = x3Dc.at<float>(2);

        if (zc < 0)
          continue;

        cv::Point2f imagePoint =
            Converter::ReprojectToImage(cv::Point3f(xc, yc, zc));
        const float u = imagePoint.x;
        const float v = imagePoint.y;

        if (u < CurrentFrame.mnMinX || u > CurrentFrame.mnMaxX)
          continue;
        if (v < CurrentFrame.mnMinY || v > CurrentFrame.mnMaxY)
          continue;

        int nLastOctave = LastFrame.keypts_[i].octave;

        float radius = th * CurrentFrame.scale_factors_[nLastOctave];
        vector<size_t> vIndices2;
        vIndices2 = CurrentFrame.GetFeaturesInArea(
            u, v, radius, nLastOctave - 1, nLastOctave + 1);

        if (vIndices2.empty())
          continue;
        const cv::Mat dMP = pMP->GetDescriptor();
        int bestDist = 256;
        int bestIdx2 = -1;

        for (vector<size_t>::const_iterator vit = vIndices2.begin(),
                                            vend = vIndices2.end();
             vit != vend; vit++) {
          const size_t i2 = *vit;
          if (CurrentFrame.mappoints_[i2])
            if (CurrentFrame.mappoints_[i2]->Observations() > 0)
              continue;

          const cv::Mat &d = CurrentFrame.descriptors_.row(i2);
          const int dist = DescriptorDistance(dMP, d);
          if (dist < bestDist) {
            bestDist = dist;
            bestIdx2 = i2;
          }
        }
        if (bestDist <= TH_HIGH) {
          CurrentFrame.mappoints_[bestIdx2] = pMP; // ??????????????????MapPoint
          nMatches++;
          if (mbCheckOrientation) {
            float rot = LastFrame.keypts_[i].angle -
                        CurrentFrame.keypts_[bestIdx2].angle;
            if (rot < 0.0)
              rot += 360.0f;
            int bin = round(rot * factor);
            if (bin == HISTO_LENGTH)
              bin = 0;
            assert(bin >= 0 && bin < HISTO_LENGTH);
            rotHist[bin].push_back(bestIdx2);
          }
        }
      }
    }
  }
  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;
    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i != ind1 && i != ind2 && i != ind3) {
        for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
          CurrentFrame.mappoints_[rotHist[i][j]] =
              static_cast<MapPoint *>(NULL);
          nMatches--;
        }
      }
    }
  }
}

int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame,
                                   const float th, const bool bMono) {
  int nmatches = 0;

  // Rotation Histogram (to check rotation consistency)
  vector<int> rotHist[HISTO_LENGTH];
  for (int i = 0; i < HISTO_LENGTH; i++)
    rotHist[i].reserve(500);
  const float factor = HISTO_LENGTH / 360.0f;

  const cv::Mat Rcw = CurrentFrame.Tcw_.rowRange(0, 3).colRange(0, 3);
  const cv::Mat tcw = CurrentFrame.Tcw_.rowRange(0, 3).col(3);

  const cv::Mat twc = -Rcw.t() * tcw; // twc(w)

  const cv::Mat Rlw = LastFrame.Tcw_.rowRange(0, 3).colRange(0, 3);
  const cv::Mat tlw = LastFrame.Tcw_.rowRange(0, 3).col(3); // tlw(l)

  // vector from LastFrame to CurrentFrame expressed in LastFrame
  const cv::Mat tlc =
      Rlw * twc + tlw; // Rlw*twc(w) = twc(l), twc(l) + tlw(l) = tlc(l)

  for (int i = 0; i < LastFrame.num_feature_; i++) {
    MapPoint *pMP = LastFrame.mappoints_[i];

    if (pMP) {
      if (!LastFrame.outliers_[i]) {
        // Project
        cv::Mat x3Dw = pMP->GetWorldPos();
        cv::Mat x3Dc = Rcw * x3Dw + tcw;

        const float xc = x3Dc.at<float>(0);
        const float yc = x3Dc.at<float>(1);
        const float zc = x3Dc.at<float>(2);

        if (zc < 0)
          continue;

        cv::Point2f imagePoint =
            Converter::ReprojectToImage(cv::Point3f(xc, yc, zc));
        const float u = imagePoint.x;
        const float v = imagePoint.y;

        if (u < CurrentFrame.mnMinX || u > CurrentFrame.mnMaxX)
          continue;
        if (v < CurrentFrame.mnMinY || v > CurrentFrame.mnMaxY)
          continue;

        int nLastOctave = LastFrame.keypts_[i].octave;

        // Search in a window. Size depends on scale
        float radius = th * CurrentFrame.scale_factors_[nLastOctave];
        vector<size_t> vIndices2;
        vIndices2 = CurrentFrame.GetFeaturesInArea(
            u, v, radius, nLastOctave - 1, nLastOctave + 1);

        if (vIndices2.empty())
          continue;
        const cv::Mat dMP = pMP->GetDescriptor();
        int bestDist = 256;
        int bestIdx2 = -1;

        // ??????????????????????????????
        for (vector<size_t>::const_iterator vit = vIndices2.begin(),
                                            vend = vIndices2.end();
             vit != vend; vit++) {
          // ????????????????????????????????????MapPoint???,?????????????????????
          const size_t i2 = *vit;
          if (CurrentFrame.mappoints_[i2])
            if (CurrentFrame.mappoints_[i2]->Observations() > 0)
              continue;

          const cv::Mat &d = CurrentFrame.descriptors_.row(i2);

          const int dist = DescriptorDistance(dMP, d);

          if (dist < bestDist) {
            bestDist = dist;
            bestIdx2 = i2;
          }
        }

        // ??????SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*>
        // &vpMapPointMatches)????????????4
        if (bestDist <= TH_HIGH) {
          CurrentFrame.mappoints_[bestIdx2] = pMP; // ??????????????????MapPoint
          nmatches++;

          if (mbCheckOrientation) {
            float rot = LastFrame.keypts_[i].angle -
                        CurrentFrame.keypts_[bestIdx2].angle;
            if (rot < 0.0)
              rot += 360.0f;
            int bin = round(rot * factor);
            if (bin == HISTO_LENGTH)
              bin = 0;
            assert(bin >= 0 && bin < HISTO_LENGTH);
            rotHist[bin].push_back(bestIdx2);
          }
        }
      }
    }
  }

  // Apply rotation consistency
  if (mbCheckOrientation) {
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++) {
      if (i != ind1 && i != ind2 && i != ind3) {
        for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
          CurrentFrame.mappoints_[rotHist[i][j]] =
              static_cast<MapPoint *>(NULL);
          nmatches--;
        }
      }
    }
  }
  return nmatches;
}

// ????????????????????????????????????index
void ORBmatcher::ComputeThreeMaxima(vector<int> *histo, const int L, int &ind1,
                                    int &ind2, int &ind3) {
  int max1 = 0;
  int max2 = 0;
  int max3 = 0;

  for (int i = 0; i < L; i++) {
    const int s = histo[i].size();
    if (s > max1) {
      max3 = max2;
      max2 = max1;
      max1 = s;
      ind3 = ind2;
      ind2 = ind1;
      ind1 = i;
    } else if (s > max2) {
      max3 = max2;
      max2 = s;
      ind3 = ind2;
      ind2 = i;
    } else if (s > max3) {
      max3 = s;
      ind3 = i;
    }
  }

  if (max2 < 0.1f * (float)max1) {
    ind2 = -1;
    ind3 = -1;
  } else if (max3 < 0.1f * (float)max1) {
    ind3 = -1;
  }
}

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b) {
  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();

  int dist = 0;

  for (int i = 0; i < 8; i++, pa++, pb++) {
    unsigned int v = *pa ^ *pb;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}
}
