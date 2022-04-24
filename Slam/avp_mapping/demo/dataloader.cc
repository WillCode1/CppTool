#include "dataloader.h"
#include "colordef.h"
DataLoader::DataLoader(const DataLoader::Mode mode) : mode_(mode) {}

void DataLoader::SetOdometryFile(const std::string &odom_file) {

  std::cout << odom_file << std::endl;

  std::ifstream fin;
  fin.open(odom_file);
  if (!fin.is_open()) {
    std::cout << kColorRed << "File " << odom_file << " open failed "
              << std::endl;
    exit(0);
  }

  if (mode_ == NOT_ALIGNED) {

    std::string line;
    std::getline(fin, line);
    std::getline(fin, line);

    while (!fin.eof()) {
      uint64_t microsecond;
      int seq;
      uint64_t msg_time;
      double x, y, z;
      double qx, qy, qz, qw;

      std::getline(fin, line);
      sscanf(line.c_str(), "%ld,%d,%ld,map,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
             &msg_time, &seq, &microsecond, &x, &y, &z, &qx, &qy, &qz, &qw);

      OdometryData odom_data;
      odom_data.timestamp = microsecond / 1000;
      odom_data.odometry = SemanticSLAM::WheelOdometry(x, y, z, qw, qx, qy, qz);
      odom_list_.push(std::make_pair(microsecond / 1000, odom_data));
    }
    fin.close();

  } else if (mode_ == ALIGNED) {
    std::string line;

    while (!fin.eof()) {
      std::getline(fin, line);
      double timestamp;
      double x, y, z, qx, qy, qz, qw;
      sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &timestamp, &x,
             &y, &z, &qx, &qy, &qz, &qw);
      OdometryData odom_data;
      odom_data.timestamp = timestamp * 1e6;
      odom_data.odometry = SemanticSLAM::WheelOdometry(x, y, z, qw, qx, qy, qz);

      odom_list_.push(std::make_pair(timestamp * 1e6, odom_data));
    }
  }
  fin.close();
}

void DataLoader::SetImageTimeStampefile(const std::string &image_ts_file) {
  std::ifstream fin;
  fin.open(image_ts_file.c_str());
  if (!fin.is_open()) {
    std::cout << kColorRed << " open  " << image_ts_file << " failed "
              << kColorReset << std::endl;
    exit(0);
  }

  std::string line;

  std::getline(fin, line); // skip first line
  while (!fin.eof()) {
    int id;
    uint64_t microsecond;
    std::getline(fin, line);
    sscanf(line.c_str(), "%d %ld", &id, &microsecond);

    DataFrame dataframe;

    dataframe.str_image_left = "/vc0/img" + std::to_string(id) + ".png";
    dataframe.str_image_front = "/vc1/img" + std::to_string(id) + ".png";
    dataframe.str_image_right = "/vc2/img" + std::to_string(id) + ".png";
    dataframe.str_image_back = "/vc3/img" + std::to_string(id) + ".png";
    dataframe.id = id;
    dataframe.timestamp = microsecond;
    image_list_.push(std::make_pair(microsecond, dataframe));
  }
  fin.close();
}

DataLoader::DataType DataLoader::NextData(DataFrame &frame,
                                          OdometryData &odometry_data) {

  DataType dt(DATA_NONE);
  if (!image_list_.empty() && !odom_list_.empty()) {
    auto image_slice = image_list_.front();
    auto odom_slice = odom_list_.front();
    auto odom_timestamp = odom_slice.first;
    auto image_timestamp = image_slice.first;
    if (image_timestamp < odom_timestamp) {
      frame = image_slice.second;
      image_list_.pop();

      dt = DATA_IMAGE;
    } else {
      odometry_data = odom_slice.second;
      odom_list_.pop();
      dt = DATA_ODOM;
    }
  }
  return dt;
}

void DataLoader::SkipImageData(int start_index) {
  while (!image_list_.empty()) {
    auto image_slice = image_list_.front();
    int id = image_slice.second.id;

    if (id < start_index) {
      image_list_.pop();
    } else {
      break;
    }
  }
}

bool DataLoader::NextFrame(DataFrame &dataframe, OdometryData &odometry_data) {

  if (!image_list_.empty() && !odom_list_.empty()) {
    auto image_slice = image_list_.front();
    auto odom_slice = odom_list_.front();
    dataframe = image_slice.second;
    odometry_data = odom_slice.second;

    image_list_.pop();
    odom_list_.pop();
  } else {
    std::cout << " No Next Frame !!!" << std::endl;
    return false;
  }

  return true;
}
