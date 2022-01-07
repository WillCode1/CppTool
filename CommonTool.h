/* Author: Will Di */
#ifndef __COMMON_TOOL__
#define __COMMON_TOOL__

#include <cmath>
#include <math.h>

double RadianToAngle(const double &dRadian)
{
  return (dRadian * 180 / M_PI);
}

double AngleToRadian(const double &dAngle)
{
  return (dAngle * M_PI / 180);
}

double &NormalizeAngle(double &theta, double min, double max)
{
  while (theta >= max)
    theta -= 2 * M_PI;
  while (theta < min)
    theta += 2 * M_PI;
  return theta;
}

//=====================================
struct Point
{
  Point(const double &x_ = 0, const double &y_ = 0, const double &z_ = 0)
      : x(x_), y(y_), z(z_) {}

  double distance2d(const Point &target) const
  {
    return std::sqrt(std::pow(x - target.x, 2) + std::pow(y - target.y, 2));
  }

  double distance3d(const Point &target) const
  {
    return std::sqrt(std::pow(x - target.x, 2) + std::pow(y - target.y, 2) + std::pow(z - target.z, 2));
  }

  double x;
  double y;
  double z;
};

struct EulerAngle
{
  EulerAngle(const double &x_ = 0, const double &y_ = 0, const double &z_ = 0)
      : roll(x_), pitch(y_), yaw(z_) {}

  void Normalize()
  {
    NormalizeAngle(roll, roll_min, roll_max);
    NormalizeAngle(pitch, pitch_min, pitch_max);
    NormalizeAngle(yaw, yaw_min, yaw_max);
  }

  EulerAngle deltaAngle(const EulerAngle &target) const
  {
    return EulerAngle(target.roll - roll, target.pitch - pitch, target.yaw - yaw);
  }

  EulerAngle absAngle(const EulerAngle &target) const
  {
    return EulerAngle(std::abs(target.roll - roll), std::abs(target.pitch - pitch), std::abs(target.yaw - yaw));
  }

  double roll;
  double pitch;
  double yaw;

  // set min max
  static double roll_min;
  static double roll_max;
  static double pitch_min;
  static double pitch_max;
  static double yaw_min;
  static double yaw_max;
};
double EulerAngle::roll_min = -M_PI;
double EulerAngle::roll_max = M_PI;
double EulerAngle::pitch_min = -M_PI;
double EulerAngle::pitch_max = M_PI;
double EulerAngle::yaw_min = -M_PI;
double EulerAngle::yaw_max = M_PI;

struct Quaternion
{
  Quaternion(const double &x_ = 0, const double &y_ = 0, const double &z_ = 0, const double &w_ = 1)
      : x(x_), y(y_), z(z_), w(w_) {}
  /*
    w = cos(theta/2)
    x = ax * sin(theta/2)
    y = ay * sin(theta/2)
    z = az * sin(theta/2)
   */
  static EulerAngle Quat2Euler(const Quaternion &q)
  {
    EulerAngle angles;
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
      angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
      angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
  }

  static Quaternion Euler2Quat(const EulerAngle &euler)
  {
    double cy = cos(euler.yaw * 0.5);
    double sy = sin(euler.yaw * 0.5);
    double cp = cos(euler.pitch * 0.5);
    double sp = sin(euler.pitch * 0.5);
    double cr = cos(euler.roll * 0.5);
    double sr = sin(euler.roll * 0.5);

    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
  }

  double x;
  double y;
  double z;
  double w;
};

struct Pose3d
{
  Pose3d() = default;

  Pose3d(const Point &pos_, const EulerAngle &euler_)
      : position(pos_), euler(euler_)
  {
    quat = Quaternion::Euler2Quat(euler_);
  }

  Pose3d(const Point &pos_, const Quaternion &quat_)
      : position(pos_), quat(quat_)
  {
    euler = Quaternion::Quat2Euler(quat_);
  }

  void setEulerAngle(const EulerAngle &euler_)
  {
    euler = euler_;
    quat = Quaternion::Euler2Quat(euler_);
  }

  void setQuaternion(const Quaternion &quat_)
  {
    quat = quat_;
    euler = Quaternion::Quat2Euler(quat_);
  }

  Point position;
  EulerAngle euler;
  Quaternion quat;
};

#endif

//=============test==============
#if 0
  Quaternion q; 
  EulerAngle euler(0,0,M_PI/2);
  ROS_INFO("EulerAngle: roll=%f, pitch=%f, yaw=%f", euler.roll, euler.pitch, euler.yaw);
  q = Quaternion::Euler2Quat(euler);
  ROS_INFO("Quaternion: x=%f, y=%f, z=%f, w=%f", q.x, q.y, q.z, q.w);
  euler = Quaternion::Quat2Euler(q);
  ROS_INFO("EulerAngle: roll=%f, pitch=%f, yaw=%f", euler.roll, euler.pitch, euler.yaw);
  exit(0);
#endif
