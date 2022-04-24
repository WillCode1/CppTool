#pragma once

#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"
#include "colordef.h"
#include "frame.h"
#include "keyframe.h"
#include "mappoint.h"
#include "mckeyframe.h"
#include "multicam_ext.h"
#include "orbextractor.h"
#include "orbvocabulary.h"
#include "timer.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

namespace FeatureSLAM {
class MCKeyFrame;
enum eCameraID {
  NONE_CAMERA = -1,
  LEFT_CAMERA = 0,
  FRONT_CAMERA = 1,
  RIGHT_CAMERA = 2,
  BACK_CAMERA = 3,
  MULTI_CAMERA = 5

};

class MCFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MCFrame();

  MCFrame(const std::vector<cv::Mat> &image, const Eigen::Vector3d &odom,
          double timestamp, std::vector<ORBextractor *> extractor,
          ORBVocabulary *voc, MultiCam *pMulticam);
  MCFrame(const MCFrame &mcframe);

  Frame *GetFrame(int camera_index);
  void SetPose(cv::Mat Tcw);
  void SetOdometry(const Eigen::Vector3d &odom);
  void UpdatePose();
  void ComputeBoW();

public:
  static long unsigned int next_id_;
  long unsigned int id_;

  double timestamp_;
  cv::Mat tcw_;
  std::vector<Frame> frames_;
  MCKeyFrame *reference_mckeyframe_;
  MultiCam *multicam_;
  // camera extrinsic parameters (relative to the front camera)
  std::vector<cv::Mat> cam_ext_;
  Eigen::Vector3d odom_vector_;
};
}
