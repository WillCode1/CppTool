#ifndef __IMU_TRACKER_H__
#define __IMU_TRACKER_H__

#include <chrono>
#include "Eigen/Geometry"

namespace estimation
{
  using TimeSec = std::chrono::duration<double, std::ratio<1>>;
  // using TimeSec = std::chrono::steady_clock::time_point;

  // Returns a quaternion representing the same rotation as the given 'angle_axis' vector.
  template <typename T>
  Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(const Eigen::Matrix<T, 3, 1> &angle_axis)
  {
    T scale = T(0.5);
    T w = T(1.);
    constexpr double kCutoffAngle = 1e-8; // We linearize below this angle.
    if (angle_axis.squaredNorm() > kCutoffAngle)
    {
      const T norm = angle_axis.norm();
      scale = sin(norm / 2.) / norm;
      w = cos(norm / 2.);
    }
    const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
    return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(), quaternion_xyz.z());
  }

  static Eigen::Vector3d RotationMatrix2YPR(const Eigen::Matrix3d &rotation)
  {
    Eigen::Vector3d n = rotation.col(0);
    Eigen::Vector3d o = rotation.col(1);
    Eigen::Vector3d a = rotation.col(2);

    Eigen::Vector3d ypr(3);
    const double &y = atan2(n(1), n(0));
    const double &p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    const double &r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;
    return ypr;
  }

  /*
      维护robot状态
      静止时,更新imu零漂情况；运动时,补偿
   */
  class ImuZeroDriftCompensation
  {
  public:
    ImuZeroDriftCompensation(const TimeSec &time, bool imu_debug = false);
    void updateStateMachine(bool is_static_now, const TimeSec& time);
    void calculateAndAddZeroDriftCompensation(Eigen::Quaterniond& orientation, const TimeSec& time);
    void setFilterOutlierThreshold(const Eigen::Vector3d& threshold);
    void setFilterRandomErrorThreshold(const Eigen::Vector3d& threshold);
    const Eigen::Vector3d &getTotalCompensationRadianAbs() const { return total_compensation_radian_abs_; }
    void setStaticTimeout(const double &static_timeout) { static_timeout_ = static_timeout; }

  private:
    bool filterIfOutlier(const Eigen::Quaterniond &orientation, const TimeSec& time);
    void filterRandomError(Eigen::Vector3d &cur_angular_velocity_compensation);
    void prepareForCalculateAngularVelocityCompensation(const Eigen::Quaterniond& orientation, const TimeSec& time);
    void updateZeroDriftAngularVelocityCompensation(const Eigen::Quaterniond &orientation, const TimeSec& time);
    void addZeroDriftCompensation(Eigen::Quaterniond& orientation, const TimeSec& time);
    Eigen::Quaterniond calculateZeroDriftCompensation(const Eigen::Vector3d& angular_velocity_compensation, const double& delta_time);

  private:
    // for test
    bool debug_{false};
    bool first_time_{true};
    Eigen::Vector3d total_compensation_radian_abs_;

    // state machine
    bool last_is_static_{false};
    bool record_for_calculate_angular_velocity_{false};       // record quat_start_statistic_
    bool need_update_zero_drift_compensation_{false};         // need update zero drift compensation
    double static_timeout_;
    TimeSec time_for_start_static_;

    TimeSec time_start_statistic_;                            // 开始统计时间
    Eigen::Quaterniond quat_start_statistic_;                 // 静止时，开始角度
    Eigen::Quaterniond last_compensatoin_end_;                // 上次补偿终点
    Eigen::Vector3d filter_outlier_threshold_;                // 过滤异常值阈值
    Eigen::Vector3d filter_random_error_threshold_;           // 排除只是随机误差的阈值

    TimeSec last_time_update_compensatoin_;                   // 上次补偿时间
    Eigen::Vector3d cur_angular_velocity_compensation_;       // 当前补偿的角速度
    Eigen::Quaterniond previous_zero_drift_compensation_;     // 累计的零漂补偿角度
  };

  // Keeps track of the orientation using angular velocities and linear
  // accelerations from an IMU. Because averaged linear acceleration (assuming
  // slow movement) is a direct measurement of gravity, roll/pitch does not drift,
  // though yaw does.
  /*
      使用IMU的角速度和线性加速度来跟踪方向。
      因为平均线性加速度(假设运动缓慢)是重力的直接测量，所以横滚/俯仰不会漂移，但偏航会。
   */
  class ImuGravityCorrection
  {
  public:
    ImuGravityCorrection(const TimeSec& time, double imu_gravity_time_constant = 0.3f);

    // Advances to the given 'time' and updates the orientation to reflect this.
    void Advance(const TimeSec &time, const Eigen::Quaterniond &orientation, const Eigen::Vector3d &imu_linear_acceleration, bool keep_original_yaw = false);

    // Updates from an IMU reading (in the IMU frame).
    // 根据传感器读数更新传感器的最新状态，得到经过重力校正的线加速度、角速度等。
    Eigen::Quaterniond AddImuAngularVelocityObservation(const Eigen::Quaterniond &orientation);
    Eigen::Quaterniond AddImuLinearAccelerationObservation(const TimeSec &time, const Eigen::Vector3d &imu_linear_acceleration);

    // keep original yaw
    void KeepOriginalYaw(const Eigen::Quaterniond &original);

    // Query the current time.
    TimeSec time() const { return last_advance_time_; }

    // Query the current orientation estimate.
    Eigen::Quaterniond orientation() const { return orientation_; }

  private:
    const double imu_gravity_time_constant_; // align重力的时间间隔
    TimeSec last_advance_time_;
    Eigen::Quaterniond last_gyroscope_orientation_; // 上次陀螺仪姿态
    Eigen::Quaterniond orientation_;       // 当前姿态
    Eigen::Vector3d gravity_vector_;       // 当前重力方向
  };
}

#endif
