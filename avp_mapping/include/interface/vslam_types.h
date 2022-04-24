#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace SemanticSLAM {
struct WheelOdometry {
  double x; // meter
  double y;
  double z;

  double qw;
  double qx;
  double qy;
  double qz;

  WheelOdometry(double _x, double _y, double _z, double _qw, double _qx,
                double _qy, double _qz)
      : x(_x), y(_y), z(_z), qw(_qw), qx(_qx), qy(_qy), qz(_qz) {}
  WheelOdometry() : x(0), y(0), z(0), qw(0), qx(0), qy(0), qz(0) {}
};

typedef struct {
  double x;
  double y;
  double z;
  double qx;
  double qy;
  double qz;
  double qw;
  int layer;
} AVPPose;

// Eigen matrix types

template <size_t R, size_t C> using MatRC_t = Eigen::Matrix<double, R, C>;

using Mat22_t = Eigen::Matrix2d;

using Mat33_t = Eigen::Matrix3d;

using Mat44_t = Eigen::Matrix4d;

using Mat55_t = MatRC_t<5, 5>;

using Mat66_t = MatRC_t<6, 6>;

using Mat77_t = MatRC_t<7, 7>;

using Mat34_t = MatRC_t<3, 4>;

using MatX_t = Eigen::MatrixXd;

// Eigen vector types

template <size_t R> using VecR_t = Eigen::Matrix<double, R, 1>;

using Vec2_t = Eigen::Vector2d;

using Vec3_t = Eigen::Vector3d;

using Vec4_t = Eigen::Vector4d;

using Vec5_t = VecR_t<5>;

using Vec6_t = VecR_t<6>;

using Vec7_t = VecR_t<7>;

using VecX_t = Eigen::VectorXd;

// Eigen Quaternion type
using Quat_t = Eigen::Quaterniond;
}
