#include "converter.h"
#include <iostream>
using namespace std;
namespace FeatureSLAM {

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors) {
  std::vector<cv::Mat> vDesc;
  vDesc.reserve(Descriptors.rows);
  for (int j = 0; j < Descriptors.rows; j++)
    vDesc.push_back(Descriptors.row(j));

  return vDesc;
}
void Converter::RmatOfQuat(cv::Mat &M, const cv::Mat &q) {
  Eigen::Quaterniond _q(q.at<float>(0, 3), q.at<float>(0, 0), q.at<float>(0, 1),
                        q.at<float>(0, 2));
  Eigen::Matrix<double, 3, 3> _m = _q.toRotationMatrix();
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      M.at<float>(i, j) = _m(i, j);
}
g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT) {
  Eigen::Matrix<double, 3, 3> R;
  R << cvT.at<float>(0, 0), cvT.at<float>(0, 1), cvT.at<float>(0, 2),
      cvT.at<float>(1, 0), cvT.at<float>(1, 1), cvT.at<float>(1, 2),
      cvT.at<float>(2, 0), cvT.at<float>(2, 1), cvT.at<float>(2, 2);

  Eigen::Matrix<double, 3, 1> t(cvT.at<float>(0, 3), cvT.at<float>(1, 3),
                                cvT.at<float>(2, 3));

  return g2o::SE3Quat(R, t);
}

cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3) {
  Eigen::Matrix<double, 4, 4> eigMat = SE3.to_homogeneous_matrix();
  return toCvMat(eigMat);
}

cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3) {
  Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
  Eigen::Vector3d eigt = Sim3.translation();
  double s = Sim3.scale();
  return toCvSE3(s * eigR, eigt);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 4, 4> &m) {
  cv::Mat cvMat(4, 4, CV_32F);
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m) {
  cv::Mat cvMat(3, 3, CV_32F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 3, 1> &m) {
  cv::Mat cvMat(3, 1, CV_32F);
  for (int i = 0; i < 3; i++)
    cvMat.at<float>(i) = m(i);

  return cvMat.clone();
}

cv::Mat Converter::toCvSE3(const Eigen::Matrix<double, 3, 3> &R,
                           const Eigen::Matrix<double, 3, 1> &t) {
  cv::Mat cvMat = cv::Mat::eye(4, 4, CV_32F);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      cvMat.at<float>(i, j) = R(i, j);
    }
  }
  for (int i = 0; i < 3; i++) {
    cvMat.at<float>(i, 3) = t(i);
  }

  return cvMat.clone();
}

cv::Mat Converter::toOdomCvMat(const Eigen::Vector3d &odom) {
  cv::Mat m = cv::Mat::eye(4, 4, CV_32FC1);
  double theta = odom.z();
  double x = odom.x();
  double y = odom.y();
  m.at<float>(0, 0) = cos(theta);
  m.at<float>(0, 1) = -sin(theta);
  m.at<float>(0, 3) = x;
  m.at<float>(1, 0) = sin(theta);
  m.at<float>(1, 1) = cos(theta);
  m.at<float>(1, 3) = y;
  return m;
}

Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Mat &cvVector) {
  Eigen::Matrix<double, 3, 1> v;
  v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

  return v;
}

Eigen::Matrix<double, 3, 1> Converter::toVector3d(const cv::Point3f &cvPoint) {
  Eigen::Matrix<double, 3, 1> v;
  v << cvPoint.x, cvPoint.y, cvPoint.z;

  return v;
}

Eigen::Matrix<double, 3, 3> Converter::toMatrix3d(const cv::Mat &cvMat3) {
  Eigen::Matrix<double, 3, 3> M;

  M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
      cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
      cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

  return M;
}

Eigen::Matrix<double, 4, 4> Converter::toMatrix4d(const cv::Mat &cvMat4) {
  Eigen::Matrix<double, 4, 4> M;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      M(i, j) = cvMat4.at<float>(i, j);
    }
  }
  return M;
}

Eigen::Matrix<double, 3, 3> Converter::toMatrix3d(const Eigen::Vector3d &v3d) {
  Eigen::Matrix3d m = Eigen::Matrix3d::Identity();

  m(0, 0) = cos(v3d.z());
  m(0, 1) = -sin(v3d.z());
  m(1, 0) = sin(v3d.z());
  m(1, 1) = cos(v3d.z());

  m(0, 2) = v3d.x();
  m(1, 2) = v3d.y();
  return m;
}

std::vector<float> Converter::toQuaternion(const cv::Mat &M) {
  Eigen::Matrix<double, 3, 3> eigMat = toMatrix3d(M);
  Eigen::Quaterniond q(eigMat);

  std::vector<float> v(4);
  v[0] = q.x();
  v[1] = q.y();
  v[2] = q.z();
  v[3] = q.w();

  return v;
}

Eigen::Vector3d
Converter::toOdomVector(const SemanticSLAM::WheelOdometry &odom) {

  Eigen::Quaterniond quat(odom.qw, odom.qx, odom.qy, odom.qz);
  auto rotation = quat.toRotationMatrix();
  Eigen::Vector3d v;
  v.x() = odom.x;
  v.y() = odom.y;
  v.z() = std::atan2(rotation(1, 0), rotation(0, 0));
  return v;
}

Eigen::Vector3d Converter::GetOdomInc(const Eigen::Vector3d &cur_odom,
                                      const Eigen::Vector3d &first_odom) {
  auto cur_odom_matrix = toMatrix3d(cur_odom);
  auto first_odom_matrix = toMatrix3d(first_odom);

  auto odom_inc_matrix = first_odom_matrix.inverse() * cur_odom_matrix;

  Eigen::Vector3d odom_inc(odom_inc_matrix(0, 2), odom_inc_matrix(1, 2),
                           atan2(odom_inc_matrix(1, 0), odom_inc_matrix(0, 0)));
  return odom_inc;
}

Eigen::Matrix4d Converter::toOdometryMatrix(const Eigen::Vector3d &odom) {
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();

  double theta = odom.z();
  double x = odom.x();
  double y = odom.y();

  m(0, 0) = cos(theta);
  m(0, 1) = -sin(theta);
  m(0, 3) = x;
  m(1, 0) = sin(theta);
  m(1, 1) = cos(theta);
  m(1, 3) = y;

  return m;
}

cv::Point2f Converter::ReprojectToImage(cv::Point3f point) {

  cv::Point2f reproj;
  float norm = cv::norm(point);
  cv::Point3f p = point / norm;
  const auto latitude = -std::asin(p.y);
  auto longitude = std::atan2(p.z, p.x);

  if (longitude < -M_PI / 2.0f)
    longitude += 2.0f * M_PI;

  reproj.x = kRad2Pixel * (-longitude + M_PI);
  reproj.y = kRad2Pixel * (-latitude + M_PI / 2.0f) - kImgLowBound;
  return reproj;
}

cv::Mat Converter::ogv2ocv(const Eigen::Matrix<double, 3, 4> &ogv_mat) {
  cv::Mat ocv_mat = cv::Mat::eye(4, 4, CV_32F);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      ocv_mat.at<float>(i, j) = (float)ogv_mat(i, j);
    }
  }

  return ocv_mat;
}

void Converter::UpdatePoseScale(cv::Mat &M, const float scale) {
  if (M.rows != 4 || M.cols != 4) {
    std::cout << " Mat dimension error " << endl;
    return;
  }

  M.at<float>(0, 3) *= scale;
  M.at<float>(1, 3) *= scale;
  M.at<float>(2, 3) *= scale;
}

bool Converter::ReprojectToImage(const Eigen::Matrix3d &rcw,
                                 const Eigen::Vector3d &tcw,
                                 const Eigen::Vector3d &pw,
                                 Eigen::Vector2d &projection) {

  Eigen::Vector3d pc = rcw * pw + tcw;
  const float coef = kImgWidth / M_PI;
  float theta, phi;
  theta = atan2(pc.z(), pc.x());
  if (theta < -M_PI / 2.0f)
    theta = 2.0f * M_PI + theta;

  phi = atan(-cos(theta) * pc.y() / pc.x());
  projection.x() = coef * (-theta + M_PI);
  projection.y() = coef * (-phi + M_PI / 2.0) - kImgLowBound;

  if (projection.x() < 0 || projection.x() > 640)
    return false;
  if (projection.y() < 0 || projection.y() > 370)
    return false;
  return true;
}

Eigen::Vector3d Converter::KeypointToBearing(const cv::KeyPoint &kp) {
  float lon = -kp.pt.x * kPixel2Rad + M_PI;
  float lat = -(kp.pt.y + kImgLowBound) * kPixel2Rad + M_PI / 2.0f;

  Eigen::Vector3d bearing;
  bearing.x() = cos(lat) * cos(lon);
  bearing.y() = -sin(lat);
  bearing.z() = cos(lat) * sin(lon);
  return bearing;
}
}
