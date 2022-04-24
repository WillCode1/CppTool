//
// Created by linminjie on 2020-12-02.
//

#pragma once
#include "vslam_types.h"
#include <Eigen/Core>
#include <memory>
#include <opencv2/opencv.hpp>

namespace SemanticSLAM {

struct PointTyped {
  int type;
  Vec3_t point;
};

class AvpMapper {
public:
  AvpMapper() {}
  virtual ~AvpMapper() {}

  virtual AVPPose GrabSegImages(uint64 microsecond,
                                std::vector<unsigned char *> seg_imgs) = 0;

  virtual void SetCurrentPose(uint64 microsecond, const AVPPose &pose) = 0;

  virtual void SetRawImage(cv::Mat &image_raw) = 0;

  virtual void SetOdometry(uint64 microsecond, WheelOdometry odom) = 0;

  virtual void SaveMap(const std::string &filename) = 0;

  virtual void SaveTrajectory(const std::string &filename) = 0;

  virtual void SaveOdometry(const std::string &filename) = 0;

  virtual bool RemapImageRequired() = 0;

  virtual std::vector<std::pair<cv::Mat, cv::Mat>> GetPanoramicRemap() = 0;

  virtual std::vector<PointTyped> GetSemanticPoints() = 0;

  virtual void Reset() = 0;
};

std::shared_ptr<AvpMapper> CreateAvpMapper(const std::string &config_file);
}

namespace FeatureSLAM {
class FeatureMapper {
public:
  FeatureMapper() {}
  virtual ~FeatureMapper() {}

  virtual cv::Mat TrackMultiCam(const cv::Mat &imleft, const cv::Mat &imfront,
                                const cv::Mat &imright, const cv::Mat &imback,
                                uint64_t timestamp) = 0;

  virtual cv::Mat
  TrackMultiCam(const cv::Mat &imleft, const cv::Mat &imfront,
                const cv::Mat &imright, const cv::Mat &imback,
                const SemanticSLAM::WheelOdometry &odometry) = 0;

  virtual bool SaveMap(const std::string &filename, bool with_bow = false) = 0;
  virtual void InsertOdometry(uint64_t timestamp,
                              const SemanticSLAM::WheelOdometry &odometry) = 0;

  virtual void AlignMaptoTrajectory(const std::string &file_name) = 0;
  virtual void Shutdown() = 0;
};

std::shared_ptr<FeatureMapper>
CreateFeatureMapper(const std::string &config_file);
}
