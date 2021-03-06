#pragma once

#include "colordef.h"
#include "frame.h"
#include "multicam_ext.h"
#include "triangulator.h"
#include "types.h"
#include <opencv2/opencv.hpp>
namespace FeatureSLAM {

class Initializer {
public:
  Initializer(const MCFrame &reference_frame, float reproj_err_thr = 2.0,
              float parallax_deg_thr = 1.0);

  bool Initialize(const MCFrame &current_frame,
                  const std::vector<std::vector<int>> &matches12, cv::Mat &T21,
                  std::vector<std::vector<cv::Point3f>> &init_mappoints,
                  std::vector<std::vector<bool>> &is_triangulated);

private:
  unsigned int CheckPose(const Mat44_t &pose_ref, const Mat44_t &pose_cur,
                         const std::vector<cv::KeyPoint> &keypoints1,
                         const std::vector<cv::KeyPoint> &keypoints2,
                         const std::vector<Vec3_t> &bearings1,
                         const std::vector<Vec3_t> &bearings2,
                         const std::vector<int> &matches12,
                         std::vector<cv::Point3f> &triangulated_pts,
                         std::vector<bool> &is_triangulated,
                         float &parallax_deg);
  bool IsVehicleMoved(const Mat44_t cur_odom);

private:
  // Keypoints from Reference Frame (Frame 1)

  std::vector<std::vector<cv::KeyPoint>> keypoints1_;
  std::vector<std::vector<Vec3_t>> bearings1_;

  // metric threshold

  float reproj_err_thr_;
  float parallax_deg_thr_;

  Mat44_t ref_odom_;
};
}
