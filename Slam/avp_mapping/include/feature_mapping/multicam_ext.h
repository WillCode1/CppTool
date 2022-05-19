#pragma once

#include "converter.h"
#include "g2o/types/types_six_dof_multicam.h"
#include "types.h"
#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>

namespace FeatureSLAM {

struct MultiCam {
  std::vector<cv::Mat> cam_extrinsic_;
  std::vector<Eigen::Matrix<double, 6, 6>> se3_cam_extrinsic_adj_;  // question: 为什么是6d
  std::vector<g2o::SE3Quat> se3quat_cam_extrinsic_;
  MultiCam() {}
  MultiCam(std::vector<cv::Mat> vmTcamExt) {
    if (vmTcamExt.size() != 4)
      std::cout << "Fatal error  ,multi-camera extrinsic parameter size !=4 "
                << std::endl;

    cam_extrinsic_.resize(4);
    se3quat_cam_extrinsic_.resize(4);
    se3_cam_extrinsic_adj_.resize(4);

    for (int i = 0; i < 4; i++) {
      cam_extrinsic_[i] = vmTcamExt[i];
      se3quat_cam_extrinsic_[i] = Converter::toSE3Quat(cam_extrinsic_[i]);
      se3_cam_extrinsic_adj_[i] = se3quat_cam_extrinsic_[i].adj();
    }
  }
};

class MultiCamExt {
public:
  MultiCamExt();
  ~MultiCamExt();
  void Initialize(std::vector<cv::Mat> &cam_ext, cv::Mat &front2wheel);
  static MultiCamExt &GetInstance();
  cv::Mat CameraPose2Baselink(cv::Mat &tcw, int camera_index);

  cv::Mat BaselinkPose2Camera(cv::Mat &tw2baselink, int camera_index);

  cv::Mat BaselinkPose2Camera(const Mat44_t &tw2base, int camera_index);

  cv::Mat GetTargetCamPose(cv::Mat &tcw_src, int src_cam_index,
                           int target_cam_index);
  cv::Mat GetFront2Wheel();

  std::vector<cv::Mat> GetExtrinsic();

private:
  static std::mutex global_multicam_ext_mutex_;

  cv::Mat front2wheel_;
  // cam 2 refcam
  std::vector<cv::Mat> cam_ext_;
};
}
