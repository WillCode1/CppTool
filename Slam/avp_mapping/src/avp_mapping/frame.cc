#include "frame.h"
namespace SemanticSLAM
{
  int Frame::next_id_ = 0;
  Frame::Frame() : visual_loc_confidence_(100)
  {
    trans_world2base_ = Mat33_t::Identity();
    id_ = next_id_++;
    image_lane_ = cv::Mat::zeros(416, 416, CV_8UC1);
    image_slot_ = cv::Mat::zeros(416, 416, CV_8UC1);
    image_dash_ = cv::Mat::zeros(416, 416, CV_8UC1);
    image_arrow_ = cv::Mat::zeros(416, 416, CV_8UC1);
  }

  Frame::Frame(const WheelOdometry &odom, const Frame &f)
      : visual_loc_confidence_(100), image_lane_(f.image_lane_),
        image_slot_(f.image_slot_), image_dash_(f.image_dash_),
        image_arrow_(f.image_arrow_)
  {
    id_ = next_id_++;
    trans_world2base_ = Mat33_t::Identity();
    Quat_t quat(odom.qw, odom.qx, odom.qy, odom.qz);
    Mat33_t rotation = quat.toRotationMatrix();
    double theta = atan2(rotation(1, 0), rotation(0, 0));
    odometry_ = Vec3_t(odom.x, odom.y, theta);
  }

  Frame::Frame(const Frame &f)
      : visual_loc_confidence_(f.visual_loc_confidence_), id_(f.id_),
        odometry_(f.odometry_), trans_world2base_(f.trans_world2base_),
        image_lane_(f.image_lane_.clone()), image_slot_(f.image_slot_.clone()),
        image_dash_(f.image_dash_.clone()), image_arrow_(f.image_arrow_.clone())
  {
  }

  Frame::~Frame() {}

  void Frame::Reset()
  {
    edge_size_ = 0;
    next_id_ = 0;
  }
}
