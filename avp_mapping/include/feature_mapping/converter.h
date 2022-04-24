#pragma once

#include <opencv2/core/core.hpp>

#include "g2o/types/types_seven_dof_expmap.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "vslam_types.h"
#include <Eigen/Dense>

namespace FeatureSLAM {

const float kPixel2Rad = M_PI / kImgWidth;
const float kRad2Pixel = kImgWidth / M_PI;

class Converter {
public:
  static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);
  static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
  static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);
  static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
  static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
  static cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &m);
  static cv::Mat toCvMat(const Eigen::Matrix3d &m);
  static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &m);
  static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R,
                         const Eigen::Matrix<double, 3, 1> &t);

  static cv::Mat toOdomCvMat(const Eigen::Vector3d &odom);
  ///@}

  /**
   * @name toEigen
   */
  ///@{
  static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &cvVector);
  static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Point3f &cvPoint);
  static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cvMat3);
  static Eigen::Matrix<double, 4, 4> toMatrix4d(const cv::Mat &cvMat4);
  static Eigen::Matrix<double, 3, 3> toMatrix3d(const Eigen::Vector3d &v3d);
  static std::vector<float> toQuaternion(const cv::Mat &M);

  static Eigen::Vector3d toOdomVector(const SemanticSLAM::WheelOdometry &odom);
  static Eigen::Vector3d GetOdomInc(const Eigen::Vector3d &cur_odom,
                                    const Eigen::Vector3d &first_odom);
  static Eigen::Matrix4d toOdometryMatrix(const Eigen::Vector3d &odom);

  ///@}
  ///

  static void RmatOfQuat(cv::Mat &M, const cv::Mat &q);
  static cv::Mat ogv2ocv(const Eigen::Matrix<double, 3, 4> &ogv_mat);
  static void UpdatePoseScale(cv::Mat &M, const float scale);

  // Projection and Re-projection
  static cv::Point2f ReprojectToImage(cv::Point3f point);
  static bool ReprojectToImage(const Eigen::Matrix3d &rcw,
                               const Eigen::Vector3d &tcw,
                               const Eigen::Vector3d &pw,
                               Eigen::Vector2d &projection);

  static Eigen::Vector3d KeypointToBearing(const cv::KeyPoint &kp);
};
}
