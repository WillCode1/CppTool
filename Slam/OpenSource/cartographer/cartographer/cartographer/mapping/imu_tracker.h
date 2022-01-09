/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
#define CARTOGRAPHER_MAPPING_IMU_TRACKER_H_

#include "Eigen/Geometry"
#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {

// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
/*
    使用IMU的角速度和线性加速度来跟踪方向。
    因为平均线性加速度(假设运动缓慢)是重力的直接测量，所以横滚/俯仰不会漂移，但偏航会。
 */
class ImuTracker {
 public:
  ImuTracker(double imu_gravity_time_constant, common::Time time);

  // Advances to the given 'time' and updates the orientation to reflect this.
  void Advance(common::Time time);

  // Updates from an IMU reading (in the IMU frame).
  // 根据传感器读数更新传感器的最新状态，得到经过重力校正的线加速度、角速度等。
  void AddImuLinearAccelerationObservation(const Eigen::Vector3d& imu_linear_acceleration);
  void AddImuAngularVelocityObservation(const Eigen::Vector3d& imu_angular_velocity);

  // Query the current time.
  common::Time time() const { return time_; }

  // Query the current orientation estimate.
  Eigen::Quaterniond orientation() const { return orientation_; }

 private:
  const double imu_gravity_time_constant_;  // align重力的时间间隔
  common::Time time_;
  common::Time last_linear_acceleration_time_;
  Eigen::Quaterniond orientation_;  // 当前姿态
  Eigen::Vector3d gravity_vector_;  // 当前重力方向
  Eigen::Vector3d imu_angular_velocity_;  // 角速度
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
