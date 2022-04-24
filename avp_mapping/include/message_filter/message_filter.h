#pragma once
#include "vslam_types.h"
#include <Eigen/Core>
#include <fstream>
#include <mutex>
#include <queue>
class MessageFilter {
public:
  MessageFilter() = delete;
  MessageFilter(const float gps_frequency, const float odom_frequency);
  ~MessageFilter();
  void AddOdomData(uint64_t microsecond,
                   const SemanticSLAM::WheelOdometry &odom);
  void AddGPS(uint64_t microsecond, const Eigen::Vector2d &gps);
  bool GetOdomByTimestamp(const uint64_t &timestamp,
                          SemanticSLAM::WheelOdometry &odom);
  bool GetGpsByTimestamp(const uint64_t &timestamp, Eigen::Vector2d &gps);

private:
  uint32_t max_message_count_;
  uint32_t interval_time_;
  std::mutex mutex_gps_;
  std::mutex mutex_odom_;
  std::queue<SemanticSLAM::WheelOdometry> odom_queue_;
  std::queue<Eigen::Vector2d> gps_queue_;
  std::queue<uint64_t> odom_timestamp_queue_;
  std::queue<uint64_t> gps_timestamp_queue_;
};
