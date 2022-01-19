#include "robot_pose_ekf/imu_tracker.h"
#include <limits>
#include <iostream>


namespace estimation
{
  ImuZeroDriftCompensation::ImuZeroDriftCompensation(Ctime time, const Eigen::Vector3d &filter_outlier_threshold)
      : last_time_update_compensatoin_(time),
        filter_outlier_threshold_(filter_outlier_threshold),
        cur_angular_velocity_compensation_(Eigen::Vector3d::Zero()),
        previous_zero_drift_compensation_(Eigen::Quaterniond::Identity())
  {
  }

  void ImuZeroDriftCompensation::updateStateMachine(bool is_static_now)
  {
    if (is_static_now)
    {
      // start static
      if (!last_is_static_)
      {
        record_for_calculate_angular_velocity_ = true;
        time_for_start_static_ = std::chrono::steady_clock::now();
      }
      // static and timeout(1 second)
      else if (std::chrono::duration<double>(std::chrono::steady_clock::now() - time_for_start_static_).count() > 1.)
      {
        need_update_zero_drift_compensation_ = true;
        time_for_start_static_ = std::chrono::steady_clock::now();
      }

      last_is_static_ = true;
    }
    // move
    else if (last_is_static_)
    {
      last_is_static_ = false;
      record_for_calculate_angular_velocity_ = false;
      need_update_zero_drift_compensation_ = false;
      // std::cout << "move!" << std::endl;
    }
  }

  void ImuZeroDriftCompensation::calculateAndAddZeroDriftCompensation(Eigen::Quaterniond &orientation)
  {
    if (record_for_calculate_angular_velocity_)
    {
      prepareForCalculateAngularVelocityCompensation(orientation);
      record_for_calculate_angular_velocity_ = false;
    }

    if (need_update_zero_drift_compensation_)
    {
      updateZeroDriftAngularVelocityCompensation(orientation);
      need_update_zero_drift_compensation_ = false;
    }

    addZeroDriftCompensation(orientation);
  }

  bool ImuZeroDriftCompensation::filterIfOutlier(const Eigen::Quaterniond &orientation)
  {
    const auto &time = std::chrono::steady_clock::now();
    double delta_t = std::chrono::duration<double>(time - last_time_update_compensatoin_).count();  // seconds

    const auto &start = last_compensatoin_end_.matrix().eulerAngles(0, 1, 2);
    const auto &end = orientation.matrix().eulerAngles(0, 1, 2);
    last_compensatoin_end_ = orientation;

    const auto &cur_imu_angular_velocity = (start - end) / delta_t;
    // std::cout << "3 " << cur_imu_angular_velocity << std::endl;

    if (std::abs(cur_imu_angular_velocity.x() - cur_angular_velocity_compensation_.x()) < filter_outlier_threshold_.x() &&
        std::abs(cur_imu_angular_velocity.y() - cur_angular_velocity_compensation_.y()) < filter_outlier_threshold_.y() &&
        std::abs(cur_imu_angular_velocity.z() - cur_angular_velocity_compensation_.z()) < filter_outlier_threshold_.z())
    {
      return false;
    }

    if (debug_)
    {
      std::cout << "3 " << cur_imu_angular_velocity << std::endl;
      std::cout << "=======================================================" << std::endl;
    }
    return true;
  }

  void ImuZeroDriftCompensation::prepareForCalculateAngularVelocityCompensation(const Eigen::Quaterniond &orientation)
  {
    time_start_statistic_ = std::chrono::steady_clock::now();
    last_compensatoin_end_ = quat_start_statistic_ = orientation;

    if (debug_)
    {
      std::cout << "prepareForCalculateAngularVelocityCompensation!" << std::endl;
    }
  }

  void ImuZeroDriftCompensation::updateZeroDriftAngularVelocityCompensation(const Eigen::Quaterniond &orientation)
  {
    const auto &time = std::chrono::steady_clock::now();
    double delta_t = std::chrono::duration<double>(time - last_time_update_compensatoin_).count();  // seconds

    // 角速度乘以时间，然后转化成RotationnQuaternion,这是这段时间的姿态变化量
    const Eigen::Quaterniond zero_drift_compensation =
        AngleAxisVectorToRotationQuaternion(Eigen::Vector3d(cur_angular_velocity_compensation_ * delta_t));
    previous_zero_drift_compensation_ = (previous_zero_drift_compensation_ * zero_drift_compensation).normalized();

    if (filterIfOutlier(orientation))
    {
      prepareForCalculateAngularVelocityCompensation(orientation);
    }
    else
    {
      const auto &start = quat_start_statistic_.matrix().eulerAngles(0, 1, 2);
      const auto &end = orientation.matrix().eulerAngles(0, 1, 2);
      delta_t = std::chrono::duration<double>(time - time_start_statistic_).count();
      const auto &mean_imu_angular_velocity = (start - end) / delta_t;

      // only yaw
      cur_angular_velocity_compensation_.z() = mean_imu_angular_velocity.z();
      // cur_angular_velocity_compensation_ = mean_imu_angular_velocity;
    }

    last_time_update_compensatoin_ = time;

    if (debug_)
    {
      std::cout << "2 " << cur_angular_velocity_compensation_ << std::endl;
      std::cout << "Update zero drift angular velocity compensation!" << std::endl;
    }
  }

  void ImuZeroDriftCompensation::addZeroDriftCompensation(Eigen::Quaterniond &orientation)
  {
    const auto& time = std::chrono::steady_clock::now();
    const double delta_t = std::chrono::duration<double>(time - last_time_update_compensatoin_).count();  // seconds

    // 角速度乘以时间，然后转化成RotationnQuaternion,这是这段时间的姿态变化量
    const Eigen::Quaterniond zero_drift_compensation =
        AngleAxisVectorToRotationQuaternion(Eigen::Vector3d(cur_angular_velocity_compensation_ * delta_t));
    // std::cout << "1 " << delta_t << std::endl;
    // std::cout << "5 " << orientation.matrix().eulerAngles(2, 1, 0) << std::endl;
    orientation = (orientation * previous_zero_drift_compensation_ * zero_drift_compensation).normalized();
    // std::cout << "6 " << orientation.matrix().eulerAngles(2, 1, 0) << std::endl;
  }

  /*
    预备知识：
      惯性测量单元（英文：Inertial measurement unit，简称IMU）是测量物体三轴姿态角(或角速率)以及加速度的装置。
      陀螺仪及加速度计是IMU的主要元件，其精度直接影响到惯性系统的精度。
      在实际工作中，由于不可避免的各种干扰因素，而导致陀螺仪及加速度计产生误差，从初始对准开始，其导航误差就随时间而增长，尤其是位置误差，这是惯导系统的主要缺点。
      所以需要利用外部信息进行辅助，实现组合导航，使其有效地减小误差随时间积累的问题。
      为了提高可靠性，还可以为每个轴配备更多的传感器。一般而言IMU要安装在被测物体的重心上。
      一般情况，一个IMU包含了三个单轴的加速度计和三个单轴的陀螺仪，加速度计检测物体在载体坐标系统独立三轴的加速度信号，
      而陀螺仪检测载体相对于导航坐标系的角速度信号，测量物体在三维空间中的角速度和加速度，并以此解算出物体的姿态。在导航中有着很重要的应用价值。
      IMU大多用在需要进行运动控制的设备，如汽车和机器人上。也被用在需要用姿态进行精密位移推算的场合，如潜艇、飞机、导弹和航天器的惯性导航设备等。

      利用三轴地磁解耦和三轴加速度计，受外力加速度影响很大，在运动/振动等环境中，输出方向角误差较大,此外地磁传感器有缺点，
      它的绝对参照物是地磁场的磁力线,地磁的特点是使用范围大，但强度较低，约零点几高斯，非常容易受到其它磁体的干扰， 如果融合了Z轴陀螺仪的瞬时角度，就可以使系统数据更加稳定。
      加速度测量的是重力方向，在无外力加速度的情况下，能准确输出ROLL/PITCH两轴姿态角度 并且此角度不会有累积误差，在更长的时间尺度内都是准确的。
      但是加速度传感器测角度的缺点是加速度传感器实际上是用MEMS技术检测惯性力造成的微小形变，而惯性力与重力本质是一样的,所以加速度计就不会区分重力加速度与外力加速度，当系统在三维空间做变速运动时，它的输出就不正确了。
      陀螺仪输出角速度是瞬时量，角速度在姿态平衡上不能直接使用， 需要角速度与时间积分计算角度，得到的角度变化量与初始角度相加，就得到目标角度，其中积分时间Dt越小输出的角度越精确。
      但陀螺仪的原理决定了它的测量基准是自身，并没有系统外的绝对参照物，加上Dt是不可能无限小的，所以积分的累积误差会随着时间的流逝迅速增加，最终导致输出角度与实际不符，所以陀螺仪只能工作在相对较短的时间尺度内[1]。
      所以在没有其它参照物的基础上，要得到较为真实的姿态角，就要利用加权算法扬长避短，结合两者的优点，摈弃其各自缺点,设计算法在短时间尺度内增加陀螺仪的权值，在更长时间尺度内增加加速度权值，这样系统输出角度就接近真实值了。

      roll/pitch does not drift,though yaw does：
      如果机器人移动缓慢，那么加速度测量影响因素直接来源于G/g：
      f=ma，a=f/m，m=G（重力）/g 。（9.8N/kg)
  */
  ImuGravityCorrection::ImuGravityCorrection(const double imu_gravity_time_constant,
                         const Ctime time)
      : imu_gravity_time_constant_(imu_gravity_time_constant),
        time_(time),
        last_linear_acceleration_time_(Ctime::min()),
        orientation_(Eigen::Quaterniond::Identity()),
        gravity_vector_(Eigen::Vector3d::UnitZ()), // 重力方向初始化为[0,0,1]
        imu_angular_velocity_(Eigen::Vector3d::Zero())
  {
  }

  // 把ImuTracker更新到指定时刻time，并把响应的orientation_,gravity_vector_和time_进行更新
  void ImuGravityCorrection::Advance(const Ctime time)
  {
    if (time < time_)
      return;
    
    const double delta_t = std::chrono::duration<double>(time - time_).count();  // seconds
    // std::cout << delta_t << std::endl;

    // 角速度乘以时间，然后转化成RotationnQuaternion,这是这段时间的姿态变化量
    const Eigen::Quaterniond rotation =
        AngleAxisVectorToRotationQuaternion(Eigen::Vector3d(imu_angular_velocity_ * delta_t));
    orientation_ = (orientation_ * rotation).normalized();
    gravity_vector_ = rotation.conjugate() * gravity_vector_;
    time_ = time;
  }

  void ImuGravityCorrection::Advance()
  {
    Advance(std::chrono::steady_clock::now());
  }

  // 根据读数更新线加速度。这里的线加速度是经过重力校正的。
  /*
    更新imu测量得到的加速度。
    参数：vector3d，测量值
    1),dt=t_-t;
    2),alpha=1-e^(-dt/g);
    3),gravity_vector_=(1-alpha)*gv_+alpha*imu_line;
    4),更新orientation_
  */
  void ImuGravityCorrection::AddImuLinearAccelerationObservation(const Eigen::Vector3d &imu_linear_acceleration)
  {
    // Update the 'gravity_vector_' with an exponential moving average using the 'imu_gravity_time_constant'.
    const double delta_t =
        last_linear_acceleration_time_ > Ctime::min()
            ? (time_ - last_linear_acceleration_time_).count()
            : std::numeric_limits<double>::infinity();
    last_linear_acceleration_time_ = time_;
    const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
    gravity_vector_ = (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
    // Change the 'orientation_' so that it agrees with the current 'gravity_vector_'.
    const Eigen::Quaterniond rotation = 
        Eigen::Quaterniond::FromTwoVectors(gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
    orientation_ = (orientation_ * rotation).normalized();

    assert((orientation_ * gravity_vector_).z() > 0.);
    assert((orientation_ * gravity_vector_).normalized().z() > 0.99);
  }

  void ImuGravityCorrection::AddImuAngularVelocityObservation(const Eigen::Vector3d &imu_angular_velocity)
  {
    imu_angular_velocity_ = imu_angular_velocity;
  }
}
