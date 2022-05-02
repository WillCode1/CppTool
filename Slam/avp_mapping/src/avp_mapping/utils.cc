#include "utils.h"
SemanticSLAM::AVPPose Utils::Matrix2AvpPose(const Eigen::Matrix3d &m) {
  SemanticSLAM::AVPPose pose;
  pose.x = m(0, 2);
  pose.y = m(1, 2);
  pose.z = 0;

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  rotation.block(0, 0, 2, 2) = m.block(0, 0, 2, 2);
  Eigen::Quaterniond quat(rotation);
  pose.qw = quat.w();
  pose.qx = quat.x();
  pose.qy = quat.y();
  pose.qz = quat.z();
  return pose;
}

Eigen::Matrix3d Utils::Se2Vector2Matrix(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m = Eigen::Matrix3d::Identity();

  m(0, 2) = v.x();
  m(1, 2) = v.y();

  m(0, 0) = cos(v.z());
  m(0, 1) = -sin(v.z());
  m(1, 0) = sin(v.z());
  m(1, 1) = cos(v.z());
  return m;
}

Eigen::Vector3d Utils::Se2Matrix2Vector(const Eigen::Matrix3d &m) {
  return Eigen::Vector3d(m(0, 2), m(1, 2), atan2(m(1, 0), m(0, 0)));
}

Eigen::Matrix4d Utils::WheelOdom2Matrix(const SemanticSLAM::WheelOdometry &odom) {
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m(0, 3) = odom.x;
  m(1, 3) = odom.y;
  m(2, 3) = odom.z;
  Eigen::Quaterniond quat(odom.qw, odom.qx, odom.qy, odom.qz);
  m.block(0, 0, 3, 3) = quat.toRotationMatrix();
  return m;
}

Eigen::Matrix4d Utils::Matrix3d2Matrix4d(const Eigen::Matrix3d &mat) {
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m(0, 3) = mat(0, 2);
  m(1, 3) = mat(1, 2);
  m.block(0, 0, 2, 2) = mat.block(0, 0, 2, 2);
  return m;
}

std::string Utils::GetCurrentTime() {
  std::time_t nowtime;
  struct tm *p;
  time(&nowtime);
  p = localtime(&nowtime);
  char name_prefix[100];
  sprintf(name_prefix, "%04d-%02d-%02d-%02d-%02d-%02d", p->tm_year + 1900,
          p->tm_mon + 1, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);
  return std::string(name_prefix);
}
