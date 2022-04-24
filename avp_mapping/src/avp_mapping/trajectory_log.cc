#include "trajectory_log.h"
#include "colordef.h"

namespace SemanticSLAM {

/**
 * Export TUM or Kitti Format Trajectory
 *
 * TUM Format:
 * timestamp x y z q_x q_y q_z q_w
 *
 * Kitti Format:
 * Kitti pose matrix
 *   a b c d
 *   e f g h
 *   i j k l
 *   0 0 0 1
 *
 *   a b c d e f g h i j k l
 *
 */

void TrajectoryLog::ExportPose(const uint64_t timestamp, const Mat44_t &pose) {

  if (format_ == Format::TUM) {
    Mat33_t rotation = pose.block(0, 0, 3, 3);
    Quat_t quat(rotation);
    fout_ << timestamp << " " << pose(0, 3) << " " << pose(1, 3) << " "
          << pose(2, 3) << " " << quat.x() << " " << quat.y() << " " << quat.z()
          << " " << quat.w() << std::endl;
  } else if (format_ == Format::KITTI) {

    fout_ << pose(0, 0) << " " << pose(0, 1) << " " << pose(0, 2) << " "
          << pose(0, 3) << " " << pose(1, 0) << " " << pose(1, 1) << " "
          << pose(1, 2) << " " << pose(1, 3) << " " << pose(2, 0) << " "
          << pose(2, 1) << " " << pose(2, 2) << " " << pose(2, 3) << " "
          << pose(3, 0) << " " << pose(3, 1) << " " << pose(3, 2) << " "
          << pose(3, 3) << std::endl;
  } else if (format_ == Format::NULLMAX) {
    timestmaps_.push_back(timestamp);
    //    timestmaps_.push_back(timestamp);
    poses_.push_back(pose);
  }
}

void TrajectoryLog::SaveLog(const std::string &filename) {
  std::ofstream fout;
  fout.open(filename);
  if (!fout.is_open()) {
    std::cout << kColorRed << " open file " << filename << " failed "
              << kColorReset << std::endl;
    return;
  }
  fout.precision(20);
  for (size_t i = 0; i < timestmaps_.size(); i++) {
    fout << timestmaps_[i] << " ";
    auto pose = poses_[i];
    fout << pose(0, 3) << " " << pose(1, 3) << " " << pose(2, 3) << " ";
    Mat33_t rotation = pose.block(0, 0, 3, 3);
    Quat_t quat(rotation);
    fout << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
         << std::endl;
  }
  fout.close();
  std::cout << kColorGreen << " Save log to " << filename << kColorReset
            << std::endl;
}

TrajectoryLog::TrajectoryLog(const std::string &filename, Format format)
    : format_(format) {

  if (format_ == Format::NULLMAX)
    return;

  fout_.open(filename);

  if (!fout_.is_open()) {
    NM_ERROR("Open TRAJECTORY LOG FILE FAILED ")
    exit(0);
  }
  fout_.precision(20);
}

TrajectoryLog::~TrajectoryLog() {

  if (fout_.is_open())
    fout_.close();
}
}
