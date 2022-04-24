#pragma once

#include "interface/avp_mapping_interface.h"
#include <Eigen/Dense>
#include <time.h>

class Utils {
public:
  Utils();
  ~Utils();

  static SemanticSLAM::AVPPose Matrix2AvpPose(const Eigen::Matrix3d &m);
  static Eigen::Matrix3d Se2Vector2Matrix(const Eigen::Vector3d &v);
  static Eigen::Vector3d Se2Matrix2Vector(const Eigen::Matrix3d &m);
  static Eigen::Matrix4d
  WheelOdom2Matrix(const SemanticSLAM::WheelOdometry &odom);
  static Eigen::Matrix4d Matrix3d2Matrix4d(const Eigen::Matrix3d &mat);
  static std::string GetCurrentTime();
};
