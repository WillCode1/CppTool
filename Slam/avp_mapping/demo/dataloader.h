#pragma once
#include "avp_mapping_interface.h"
#include <deque>
#include <fstream>
#include <queue>
#include <string>
using namespace SemanticSLAM;

struct DataFrame {
  std::string str_image_left;
  std::string str_image_front;
  std::string str_image_right;
  std::string str_image_back;
  uint64_t timestamp;
  int id;
};

struct OdometryData {
  WheelOdometry odometry;
  uint64_t timestamp;
};

class DataLoader {
public:
  enum Mode { NOT_ALIGNED = -1, ALIGNED = 0 };

  enum DataType { DATA_NONE = -1, DATA_IMAGE = 0, DATA_ODOM = 1 };
  DataLoader() = delete;
  explicit DataLoader(const Mode mode);

  void SetOdometryFile(const std::string &odom_file);
  void SetImageTimeStampefile(const std::string &image_ts_file);

  DataType NextData(DataFrame &dataframe, OdometryData &odometry_data);
  void SkipImageData(int start_index);
  bool NextFrame(DataFrame &dataframe, OdometryData &odometry_data);

private:
  Mode mode_;
  std::queue<std::pair<uint64_t, OdometryData>> odom_list_;
  std::queue<std::pair<uint64_t, DataFrame>> image_list_;
};
