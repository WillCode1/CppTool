#include "multicam_ext.h"

namespace FeatureSLAM {

std::mutex MultiCamExt::global_multicam_ext_mutex_;

MultiCamExt::MultiCamExt() {}

MultiCamExt::~MultiCamExt() {}

void MultiCamExt::Initialize(std::vector<cv::Mat> &cam_ext,
                             cv::Mat &front2wheel) {
  cam_ext_ = cam_ext;
  front2wheel_ = front2wheel;
}

MultiCamExt &MultiCamExt::GetInstance() {
  static MultiCamExt *instance = nullptr;
  if (!instance) {
    global_multicam_ext_mutex_.lock();
    if (!instance) {
      instance = new MultiCamExt;
    }
    global_multicam_ext_mutex_.unlock();
  }
  return *instance;
}

// 此处相机外参cam_ext_, 表示的是Tcicf(当前相机相对于前置的转换)，不是一般意义的Tcfci(Twc)
// 此处front2wheel_应该是wheel2front_, 即Tcfb
// △Tciw = Tcicf * Tcfb * △Tbw
cv::Mat MultiCamExt::BaselinkPose2Camera(cv::Mat &tw2baselink,
                                         int camera_index) {
  cv::Mat tcw = cam_ext_[camera_index] * front2wheel_ * tw2baselink.inv();
  return tcw;
}

cv::Mat MultiCamExt::BaselinkPose2Camera(const Mat44_t &tw2base,
                                         int camera_index) {

  cv::Mat tw2baselink = Converter::toCvMat(tw2base);
  cv::Mat tcw = cam_ext_[camera_index] * front2wheel_ * tw2baselink.inv();
  return tcw;
}

// question: 2个转换关系
cv::Mat MultiCamExt::GetTargetCamPose(cv::Mat &tcw_src, int src_cam_index,
                                      int target_cam_index) {
  cv::Mat tcw_front = cam_ext_[src_cam_index].inv() * tcw_src;
  cv::Mat tcw = cam_ext_[target_cam_index] * tcw_front;
  return tcw;
}

cv::Mat MultiCamExt::GetFront2Wheel() { return front2wheel_; }

std::vector<cv::Mat> MultiCamExt::GetExtrinsic() { return cam_ext_; }

// question: 2个转换关系
cv::Mat MultiCamExt::CameraPose2Baselink(cv::Mat &tcw, int camera_index) {
  cv::Mat tw2baselink = tcw.inv() * cam_ext_[camera_index] * front2wheel_;
  return tw2baselink;
}
}
