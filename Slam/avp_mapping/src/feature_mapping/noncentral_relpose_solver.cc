#include "noncentral_relpose_solver.h"

namespace FeatureSLAM {

// question: 什么作用
NonCentralRelPoseSolver::NonCentralRelPoseSolver(
    KeyFrame *pKF1, KeyFrame *pKF2, const std::vector<MapPoint *> &vpMatched12,
    KeyFrame *pKF1_, KeyFrame *pKF2_,
    const std::vector<MapPoint *> &vpMatched12_)
    : numberCameras_(2), n_(0), iterations_(50), mInliers_(100) {
  //
  MCKeyFrame *pMCKF = pKF1->GetMCKeyFrame();
  MultiCam *pMultiCam = pMCKF->multicam_;

  matched_pose_ = pKF2->GetPose();

  //  camOffsets_;
  //  camRotations_;

  //  multiPoints;
  //  multiBearingVectors;

  {
    // set camOffsets_ and camRotations_
    opengv::translation_t position1 = Eigen::Vector3d::Zero();
    opengv::rotation_t rotation1 = Eigen::Matrix3d::Identity();

    camOffsets_.push_back(position1);
    camRotations_.push_back(rotation1);

    if (pKF1->camera_index_ == LEFT_CAMERA) {
      cv::Mat transform2 = pMultiCam->cam_extrinsic_[LEFT_CAMERA] *
                           pMultiCam->cam_extrinsic_[RIGHT_CAMERA].inv();
      cv::Mat R = transform2.rowRange(0, 3).colRange(0, 3);
      cv::Mat t = transform2.rowRange(0, 3).col(3);

      opengv::translation_t position2(t.at<float>(0), t.at<float>(1),
                                      t.at<float>(2));
      opengv::rotation_t rotation2;
      rotation2 << R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
          R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
          R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2);

      camOffsets_.push_back(position2);
      camRotations_.push_back(rotation2);
    } else if (pKF1->camera_index_ == RIGHT_CAMERA) {
      cv::Mat transform2 = pMultiCam->cam_extrinsic_[RIGHT_CAMERA] *
                           pMultiCam->cam_extrinsic_[LEFT_CAMERA].inv();
      cv::Mat R = transform2.rowRange(0, 3).colRange(0, 3);
      cv::Mat t = transform2.rowRange(0, 3).col(3);

      opengv::translation_t position2(t.at<float>(0), t.at<float>(1),
                                      t.at<float>(2));
      opengv::rotation_t rotation2;
      rotation2 << R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
          R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
          R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2);

      camOffsets_.push_back(position2);
      camRotations_.push_back(rotation2);
    } else {
      std::cout << " Fatal error " << std::endl;
      std::cout << " Fatal error " << std::endl;
      std::cout << " Fatal error " << std::endl;
      std::cout << " Fatal error " << std::endl;
      std::cout << " Fatal error " << std::endl;
      return;
    }
  }

  {

    std::shared_ptr<opengv::points_t> points(new opengv::points_t());
    std::shared_ptr<opengv::bearingVectors_t> bearingVectors(
        new opengv::bearingVectors_t());

    for (size_t i = 0; i < vpMatched12.size(); i++) {
      MapPoint *pMP2 = vpMatched12[i];
      MapPoint *pMP1 = pKF1->mappoints_[i];

      if (!pMP1 || !pMP2)
        continue;
      if (pMP1->isBad() || pMP2->isBad())
        continue;

      auto bearing1 = Converter::KeypointToBearing(pKF1->keypts_[i]);
      bearingVectors->push_back(bearing1);

      cv::Vec3d Pos = pMP2->GetWorldPos();
      points->push_back(opengv::point_t(Pos(0), Pos(1), Pos(2)));
      vbMatched.push_back(i);
    }

    multiBearingVectors.push_back(bearingVectors);
    multiPoints.push_back(points);
  }

  {

    std::shared_ptr<opengv::points_t> points(new opengv::points_t());
    std::shared_ptr<opengv::bearingVectors_t> bearingVectors(
        new opengv::bearingVectors_t());

    for (size_t i = 0; i < vpMatched12_.size(); i++) {
      MapPoint *pMP2 = vpMatched12_[i];
      MapPoint *pMP1 = pKF1_->mappoints_[i];

      if (!pMP1 || !pMP2)
        continue;
      if (pMP1->isBad() || pMP2->isBad())
        continue;

      auto bearing1 = Converter::KeypointToBearing(pKF1_->keypts_[i]);
      bearingVectors->push_back(bearing1);

      cv::Vec3d Pos = pMP2->GetWorldPos();
      points->push_back(opengv::point_t(Pos(0), Pos(1), Pos(2)));
      vbMatched_.push_back(i);
    }

    multiBearingVectors.push_back(bearingVectors);
    multiPoints.push_back(points);
  }
}

NonCentralRelPoseSolver::~NonCentralRelPoseSolver() {}

cv::Mat NonCentralRelPoseSolver::GetEstimatedRotation() {
  return best_rotation_.clone();
}

cv::Mat NonCentralRelPoseSolver::GetEstimatedTranslation() {
  return best_traslation_.clone();
}

float NonCentralRelPoseSolver::GetEstimatedScale() { return 1.0; }

cv::Mat NonCentralRelPoseSolver::iterate(std::vector<bool> &inliers,
                                         std::vector<bool> &inliers_,
                                         bool &bNoMore) {

  bNoMore = false;
  if (n_ >= iterations_ || mInliers_ < 40) {
    bNoMore = true;
    return cv::Mat();
  }

  cv::Mat result;
  opengv::absolute_pose::NoncentralAbsoluteMultiAdapter adapter(
      multiBearingVectors, multiPoints, camOffsets_, camRotations_);

  // Create a AbsolutePoseSacProblem and Ransac
  // The method is set to GP3P
  opengv::sac::MultiRansac<opengv::sac_problems::absolute_pose::
                               MultiNoncentralAbsolutePoseSacProblem>
      ransac;
  std::shared_ptr<opengv::sac_problems::absolute_pose::
                      MultiNoncentralAbsolutePoseSacProblem>
      absposeproblem_ptr(new opengv::sac_problems::absolute_pose::
                             MultiNoncentralAbsolutePoseSacProblem(adapter));

  ransac.sac_model_ = absposeproblem_ptr;
  //  ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/800.0));
  ransac.threshold_ = 0.002;
  ransac.max_iterations_ = 50;
  ransac.computeModel();

  opengv::transformation_t trafo = ransac.model_coefficients_;
  n_ = ransac.iterations_;
  mInliers_ = ransac.inliers_[0].size() + ransac.inliers_[1].size();

  // If Ransac reaches max. iterations discard keyframe
  if (ransac.iterations_ >= ransac.max_iterations_) {
    std::cout << " nothing" << std::endl;
    return cv::Mat();

  } else if ((ransac.inliers_[0].size() + ransac.inliers_[1].size()) < 40) {

    std::cout << " no enouph inliers " << std::endl;
    return cv::Mat();

  }

  else {
    std::cout << " multi cams " << std::endl;
    std::cout << multiPoints[0]->size() << std::endl;
    std::cout << multiPoints[1]->size() << std::endl;

    for (size_t j = 0; j < ransac.inliers_[0].size(); j++) {
      unsigned int inlier_index = vbMatched[ransac.inliers_[0][j]];
      inliers[inlier_index] = true;
    }

    for (size_t j = 0; j < ransac.inliers_[1].size(); j++) {
      unsigned int inlier_index = vbMatched_[ransac.inliers_[1][j]];
      inliers_[inlier_index] = true;
    }

    std::cout << " inliers : " << ransac.inliers_[0].size() << " "
              << ransac.inliers_[1].size() << std::endl;

    for (size_t i = 0; i < ransac.inliers_.size(); i++) {
      for (size_t j = 0; j < ransac.inliers_[i].size(); j++)
        std::cout << ransac.inliers_[i][j] << " ";
    }

    cv::Mat trafoOut = Converter::ogv2ocv(trafo); // trafoOut is
    result = trafoOut.inv() * matched_pose_.inv();
    best_rotation_ = result.rowRange(0, 3).colRange(0, 3);
    best_traslation_ = result.rowRange(0, 3).col(3);

    std::cout << " minjie " << std::endl;
    std::cout << result << std::endl;
  }

  return result;
}
}
