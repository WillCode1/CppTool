#pragma once

#include "interface/avp_mapping_interface.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
namespace SemanticSLAM
{
  class Frame
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Frame();
    Frame(const WheelOdometry &odom, const Frame &f);
    Frame(const Frame &f);
    ~Frame();

    void Reset();

  private:
    static int next_id_;

  public:
    float visual_loc_confidence_; // question
    int id_;
    Vec3_t odometry_;
    Mat33_t trans_world2base_;
    cv::Mat image_lane_;
    cv::Mat image_slot_;
    cv::Mat image_dash_;
    cv::Mat image_arrow_;
    bool got_odometry_;
    int edge_size_; // question
  };
}
