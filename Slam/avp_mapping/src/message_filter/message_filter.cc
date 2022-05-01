#include "message_filter/message_filter.h"
#include <cmath>
#include <iostream>

MessageFilter::MessageFilter(const float gps_frequency, const float odom_frequency)
    : max_message_count_(100), interval_time_(40000)
{
}

MessageFilter::~MessageFilter() {}

void MessageFilter::AddOdomData(uint64_t microsecond, const SemanticSLAM::WheelOdometry &odom)
{
  std::unique_lock<std::mutex> lock(mutex_odom_);
  odom_queue_.push(odom);
  odom_timestamp_queue_.push(microsecond);
  if (odom_queue_.size() > max_message_count_)
  {
    odom_queue_.pop();
    odom_timestamp_queue_.pop();
  }
}

void MessageFilter::AddGPS(uint64_t microsecond, const Eigen::Vector2d &gps)
{
  std::unique_lock<std::mutex> lock(mutex_gps_);
  gps_queue_.push(gps);
  gps_timestamp_queue_.push(microsecond);
  if (gps_queue_.size() > max_message_count_)
  {
    gps_queue_.pop();
    gps_timestamp_queue_.pop();
  }
}

bool MessageFilter::GetOdomByTimestamp(const uint64_t &timestamp, SemanticSLAM::WheelOdometry &odom)
{
  bool found = false;
  std::unique_lock<std::mutex> lock(mutex_odom_);
  uint64_t timestamp_oldest = 0;
  while (!odom_queue_.empty())
  {
    timestamp_oldest = odom_timestamp_queue_.front();
    odom = odom_queue_.front();

    if (timestamp_oldest < timestamp)
    {
      odom_queue_.pop();
      odom_timestamp_queue_.pop();
    }
    else
    {
      found = true;
      break;
    }
  }

  if ((static_cast<uint32_t>(timestamp - timestamp_oldest)) <= interval_time_)
  {
    found = true;
  }

  if (!found)
  {
    std::cout << "\033[1;31m";
    std::cout << " image odom data synchronization interval "
              << static_cast<int64_t>(timestamp - timestamp_oldest) << " us ";
    std::cout << "\033[0m" << std::endl;
  }
  return found;
}

bool MessageFilter::GetGpsByTimestamp(const uint64_t &timestamp, Eigen::Vector2d &gps)
{
  bool found = false;
  std::unique_lock<std::mutex> lock(mutex_gps_);
  uint64_t timestamp_oldest = 0;

  while (!gps_queue_.empty())
  {
    timestamp_oldest = gps_timestamp_queue_.front();
    gps = gps_queue_.front();

    if (timestamp_oldest < timestamp)
    {
      gps_queue_.pop();
      gps_timestamp_queue_.pop();
    }
    else
    {
      found = true;
      break;
    }
  }

  if ((static_cast<uint32_t>(timestamp - timestamp_oldest)) <= interval_time_)
  {
    found = true;
  }

  return found;
}
