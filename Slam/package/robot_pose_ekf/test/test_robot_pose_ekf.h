/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Wim Meeussen */
#ifndef __TEST_ROBOT_POSE_EKF__
#define __TEST_ROBOT_POSE_EKF__

#include <string>
#include <cmath>
#include <math.h>
#include <gtest/gtest.h>
#include "Eigen/Dense"

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "map_server/RobotPose.h"

#include "Pose.h"
#include "metrics.h"
using namespace ros;
using namespace Eigen;

using OdomConstPtr = boost::shared_ptr<nav_msgs::Odometry const>;
using ImuConstPtr = boost::shared_ptr<sensor_msgs::Imu const>;
using MapPoseConstPtr = boost::shared_ptr<map_server::RobotPose const>;

geometry_msgs::PoseStamped Imu2PoseStamped(const sensor_msgs::Imu &imu)
{
  geometry_msgs::PoseStamped tmp;
  tmp.header = imu.header;
  tmp.pose.orientation = imu.orientation;
  return tmp;
}

geometry_msgs::PoseStamped Odometry2PoseStamped(const nav_msgs::Odometry &odom)
{
  geometry_msgs::PoseStamped tmp;
  tmp.header = odom.header;
  tmp.pose.orientation = odom.pose.pose.orientation;
  tmp.pose.position = odom.pose.pose.position;
  return tmp;
}

Pose3d Imu2Pose3d(const sensor_msgs::Imu &imu)
{
  Vector3d trans;
  Quaterniond quat(imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
  return Pose3d(trans, quat);
}

Pose3d Odometry2Pose3d(const nav_msgs::Odometry &odom)
{
  Vector3d trans(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  Quaterniond quat(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
  return Pose3d(trans, quat);
}

Pose3d MapPose2Pose3d(const map_server::RobotPose &map_pose)
{
  Vector3d trans(map_pose.pose.pose.position.x, map_pose.pose.pose.position.y, map_pose.pose.pose.position.z);
  Quaterniond quat(map_pose.pose.pose.orientation.w, map_pose.pose.pose.orientation.x, map_pose.pose.pose.orientation.y, map_pose.pose.pose.orientation.z);
  return Pose3d(trans, quat);
}

Pose3d Trans2DstSysTarget(const Pose3d &src_sys_ref, const Pose3d &dst_sys_ref, const Pose3d &src_sys_tag)
{
  return dst_sys_ref * src_sys_ref.inverse() * src_sys_tag;
}

class TestEKF : public testing::Test
{
public:
  int fuse_counter_, sf_counter_, lo_counter_, odom_counter_, imu_counter_, map_counter_;
  double duration_;
  double run_dis_fuse_, run_dis_sf_, run_dis_odom_, run_dis_lo_, run_dis_global_;
  double run_angle_fuse_, run_angle_sf_, run_angle_odom_, run_angle_lo_, run_angle_imu_, run_angle_global_;
  ros::Time last_fuse_time_;

  NodeHandle node_;
  ros::Subscriber fuse_sub_, sf_sub_, lo_sub_, odom_sub_, imu_sub_, map_pose_sub_;
  ros::Publisher odom_pub_, imu_pub_, lo_pub_, vo_pub_, gps_pub_, sf_pub_, map_pose_pub_;
  Pose3d fuse_start_, sf_start_, lo_start_, odom_start_, imu_start_;
  Pose3d fuse_end_, sf_end_, lo_end_, odom_end_, imu_end_;
  int floor_num_;
  Pose3d map_start_, map_end_;
  EvaluateTrajectoryErrorAtEachInterval metrics_;

  void ImuCallback(const ImuConstPtr &imu)
  {
    if (imu_counter_ == 0)
    {
      imu_end_ = imu_start_ = Imu2Pose3d(*imu);
      ROS_INFO("Imu init!");
    }
    else
    {
      const auto &tmp = Imu2Pose3d(*imu);
      run_angle_imu_ += imu_end_.absAngle(tmp)(0);
      imu_end_ = tmp;
    }

    metrics_.addTrajectoryPose("/imu", Imu2PoseStamped(*imu));
    imu_counter_++;
  }

  void OdomCallback(const OdomConstPtr &odom)
  {
    if (odom_counter_ == 0)
    {
      odom_end_ = odom_start_ = Odometry2Pose3d(*odom);
      ROS_INFO("Odom init!");
    }
    else
    {
      const auto &tmp = Odometry2Pose3d(*odom);
      run_dis_odom_ += odom_end_.distance2d(tmp);
      run_angle_odom_ += odom_end_.absAngle(tmp)(0);
      odom_end_ = tmp;
    }

    metrics_.addTrajectoryPose("/odom", Odometry2PoseStamped(*odom));
    odom_counter_++;
  }

  void LoCallback(const OdomConstPtr &lo)
  {
    if (lo_counter_ == 0)
    {
      lo_end_ = lo_start_ = Odometry2Pose3d(*lo);
      ROS_INFO("Lo init!");
    }
    else
    {
      const auto &tmp = Odometry2Pose3d(*lo);
      run_dis_lo_ += lo_end_.distance2d(tmp);
      run_angle_lo_ += lo_end_.absAngle(tmp)(0);
      lo_end_ = tmp;
    }

    metrics_.addTrajectoryPose("/lo", Odometry2PoseStamped(*lo));
    lo_counter_++;
  }

  void SfCallback(const OdomConstPtr &sf)
  {
    if (sf_counter_ == 0)
    {
      sf_end_ = sf_start_ = Odometry2Pose3d(*sf);
      ROS_INFO("Sf init!");
    }
    else
    {
      const auto &tmp = Odometry2Pose3d(*sf);
      run_dis_sf_ += sf_end_.distance2d(tmp);
      run_angle_sf_ += sf_end_.absAngle(tmp)(0);
      sf_end_ = tmp;
    }

    metrics_.addTrajectoryPose("/sf", Odometry2PoseStamped(*sf));
    sf_counter_++;
  }

  void FuseCallback(const OdomConstPtr &fuse)
  {
    // get initial time
    if (fuse_counter_ == 0)
    {
      fuse_end_ = fuse_start_ = Odometry2Pose3d(*fuse);
      ROS_INFO("Fuse init!");
    }
    else
    {
      const auto &tmp = Odometry2Pose3d(*fuse);
      run_dis_fuse_ += fuse_end_.distance2d(tmp);
      run_angle_fuse_ += fuse_end_.absAngle(tmp)(0);
      // ROS_INFO_COND(fuse_end_.distance2d(tmp) > 0.1, "Fuse error!: %f m", fuse_end_.distance2d(tmp));
      fuse_end_ = tmp;
      // ROS_INFO_STREAM("Fuse pose: " << tmp);
    }

    last_fuse_time_ = ros::Time::now();
    // count number of callbacks
    metrics_.addTrajectoryPose("/fuse", Odometry2PoseStamped(*fuse));
    fuse_counter_++;
  }

  void MapPoseCallback(const MapPoseConstPtr &pose)
  {
    static bool havenot_change_coord_flag = false;
    static bool need_change_coord = false;
    static bool ready_change_coord = false;
    static double dis_from_last = 0;

    if (map_counter_ == 0)
    {
      map_end_ = map_start_ = MapPose2Pose3d(*pose);
      floor_num_ = pose->floor;
      metrics_.addGlobalTrajectoryPose(pose->pose);
      ROS_INFO("Map pose init!");
    }
    // 坐标完全校准后更新坐标
    else if (ready_change_coord && MapPose2Pose3d(*pose).distance2d(map_end_) == dis_from_last)
    {
      havenot_change_coord_flag = false;
      need_change_coord = false;
      ready_change_coord = false;
      auto const& tmp = MapPose2Pose3d(*pose);
      map_start_ = Trans2DstSysTarget(map_end_, tmp, map_start_);
      map_end_ = tmp;
      metrics_.addGlobalTrajectoryPose(pose->pose, true);
      ROS_INFO("Change coord!");
    }
    // 准备切换楼层，不更新坐标
    else if (ready_change_coord || need_change_coord && MapPose2Pose3d(*pose).distance2d(map_end_) > 0.1)
    {
      need_change_coord = false;
      ready_change_coord = true;
      dis_from_last = MapPose2Pose3d(*pose).distance2d(map_end_);
      // ROS_INFO("Change coord2!");
    }
    else
    {
      const auto &tmp = MapPose2Pose3d(*pose);
      run_dis_global_ += map_end_.distance2d(tmp);
      run_angle_global_ += map_end_.absAngle(tmp)(0);
      map_end_ = tmp;
      metrics_.addGlobalTrajectoryPose(pose->pose);
      ROS_ERROR_COND(map_end_.distance2d(tmp) > 0.01 && havenot_change_coord_flag, "Have not change coord!");
      // std::cout << map_end_.translation_ << std::endl;
    }

    if (floor_num_ != pose->floor)
    {
      floor_num_ = pose->floor;
      need_change_coord = true;
      havenot_change_coord_flag = true;
      ROS_INFO("Change floor!");
    }

    map_counter_++;
  }

  void readBagAndPublishTopic(const std::string& bag_path, const std::vector<std::string>& topics, float duration = 10.f, int msg_size = 1000)
  {
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    ASSERT_EQ(bag.isOpen(), true);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // check
    ROS_INFO("Number of Sensor Data: %d", view.size());
    EXPECT_GT(view.size(), msg_size);

    ros::Time bag_begin_time = view.getBeginTime();
    ros::Time bag_end_time = view.getEndTime();
    ROS_INFO("ROS bag duration time: %f(s)", (bag_end_time - bag_begin_time).toSec());
    EXPECT_GT((bag_end_time - bag_begin_time).toSec(), duration);
    duration_ += (bag_end_time - bag_begin_time).toSec();

    publishTopic(view);

    ROS_INFO("End time reached");
    bag.close();
  }

  void publishTopic(rosbag::View &view)
  {
    auto time_now = ros::SteadyTime::now();
    ros::Time bag_begin_time = view.getBeginTime();

    for (rosbag::MessageInstance const &m : view)
    {
      const std::string &topic = m.getTopic();
      nav_msgs::Odometry::ConstPtr odom;
      sensor_msgs::Imu::ConstPtr imu;
      map_server::RobotPose::ConstPtr map_pose;

      if (topic.compare("/imu") == 0)
      {
        imu = m.instantiate<sensor_msgs::Imu>();
      }
      else if (topic.compare("/map_server/robot_pose") == 0)
      {
        map_pose = m.instantiate<map_server::RobotPose>();
      }
      else
      {
        odom = m.instantiate<nav_msgs::Odometry>();
      }

      ASSERT_EQ(imu == nullptr && odom == nullptr && map_pose == nullptr, false);

      while ((ros::SteadyTime::now() - time_now).toSec() < (m.getTime() - bag_begin_time).toSec())
      {
      }

      if (topic.compare("/imu") == 0)
      {
        imu_pub_.publish(*imu);
      }
      else if (topic.compare("/peter_motor_core/odom") == 0)
      {
        odom_pub_.publish(*odom);
      }
      else if (topic.compare("/odom_rf2o") == 0)
      {
        lo_pub_.publish(*odom);
      }
      else if (topic.compare("/sf") == 0)
      {
        sf_pub_.publish(*odom);
      }
      else if (topic.compare("/map_server/robot_pose") == 0)
      {
        map_pose_pub_.publish(*map_pose);
      }
    }
  }

  void evaluateError(const std::string &what, const Pose3d &fuse, const Pose3d &truth,
                     const double &run_dis, const double &run_angle,
                     const double &dis_error_mu_threshold = 10000,
                     const double &angle_error_mu_threshold = 10000)
  {
    auto total_dis_error = fuse.distance2d(truth);
    auto total_angle_error = fuse.absAngle(truth)(0);

    double dis_error_mu = total_dis_error / run_dis_sf_;
    double angle_error_mu = total_angle_error / run_angle_sf_;

    ROS_INFO("%s metrics:", what.c_str());
    ROS_ERROR_COND(run_dis == 0 && run_angle == 0, "Maybe %s not activated!", what.c_str());
    ROS_INFO("=======================================");
    ROS_INFO("total_dis_error: %f, run_dis: %f, error_mu: %f, threshold: %f", total_dis_error, run_dis, dis_error_mu, dis_error_mu_threshold);
    ROS_INFO("total_ang_error: %f, run_ang: %f, error_mu: %f, threshold: %f", total_angle_error, run_angle, angle_error_mu, angle_error_mu_threshold);

    ROS_INFO("standard deviation = error / time: (%f, %f, %f)", total_dis_error/duration_, total_dis_error/duration_, total_angle_error/duration_);

    if (dis_error_mu_threshold < 1000 && angle_error_mu_threshold < 1000)
    {
      EXPECT_LE(dis_error_mu, dis_error_mu_threshold);
      EXPECT_LE(angle_error_mu, angle_error_mu_threshold);
    }

    ROS_INFO("=======================================");
  }

  void WaitForBagFinished(const std::string& bag_path)
  {
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    ASSERT_EQ(bag.isOpen(), true);

    rosbag::View view(bag);

    ros::Time bag_begin_time = view.getBeginTime();
    ros::Time bag_end_time = view.getEndTime();
    bag.close();

    ROS_INFO("ROS bag duration time: %f(s)", (bag_end_time - bag_begin_time).toSec());
    duration_ += (bag_end_time - bag_begin_time).toSec();

    Duration d(0.01);
    // wait while bag is played back
    ROS_INFO("Waiting for bag to start playing");
    while (fuse_counter_ == 0)
      d.sleep();
    ROS_INFO("Detected that bag is playing");

    ros::Time start = ros::Time::now();
    ROS_INFO("Waiting untile end time is reached");
    while ((ros::Time::now() - start).toSec() < duration_)
    {
      if ((ros::Time::now() - last_fuse_time_).toSec() > 1)
      {
        ROS_ERROR("Detected Abnormal termination!");
        break;
      }
      d.sleep();
    }
    ROS_INFO("End time reached");
    // give filter some time to catch up
    WallDuration(2.0).sleep();
  }

protected:
  void SetUp()
  {
    fuse_counter_ = sf_counter_ = lo_counter_ = odom_counter_ = imu_counter_ = map_counter_ = duration_ = 0;
    run_dis_fuse_ = run_dis_sf_ = run_dis_odom_ = run_dis_lo_ = run_dis_global_ = 0;
    run_angle_fuse_ = run_angle_sf_ = run_angle_odom_ = run_angle_lo_ = run_angle_imu_ = run_angle_global_ = 0;

    odom_pub_ = node_.advertise<nav_msgs::Odometry>("/peter_motor_core/odom", 10);
    imu_pub_ = node_.advertise<sensor_msgs::Imu>("/imu", 10);
    lo_pub_ = node_.advertise<nav_msgs::Odometry>("/odom_rf2o", 10);
    // vo_pub_ = node_.advertise<nav_msgs::Odometry>("/testvo", 10);
    // gps_pub_ = node_.advertise<nav_msgs::Odometry>("/testgps", 10);
    sf_pub_ = node_.advertise<nav_msgs::Odometry>("/sf", 10);
    map_pose_pub_ = node_.advertise<map_server::RobotPose>("/map_server/robot_pose", 10);

    ROS_INFO("Subscribing to /robot_pose_ekf/odom");
    fuse_sub_ = node_.subscribe("/robot_pose_ekf/odom", 10, &TestEKF::FuseCallback, (TestEKF *)this);

    ROS_INFO("Subscribing to /peter_motor_core/odom");
    odom_sub_ = node_.subscribe("/peter_motor_core/odom", 10, &TestEKF::OdomCallback, (TestEKF *)this);

    ROS_INFO("Subscribing to /odom_rf2o");
    lo_sub_ = node_.subscribe("/odom_rf2o", 10, &TestEKF::LoCallback, (TestEKF *)this);

    ROS_INFO("Subscribing to /imu");
    imu_sub_ = node_.subscribe("/imu", 10, &TestEKF::ImuCallback, (TestEKF *)this);

    ROS_INFO("Subscribing to /sf");
    sf_sub_ = node_.subscribe("/sf", 10, &TestEKF::SfCallback, (TestEKF *)this);

    ROS_INFO("Subscribing to /map_server/robot_pose");
    map_pose_sub_ = node_.subscribe("/map_server/robot_pose", 10, &TestEKF::MapPoseCallback, (TestEKF *)this);
  }

  void TearDown()
  {
    metrics_.reset();

    odom_pub_.shutdown();
    imu_pub_.shutdown();
    lo_pub_.shutdown();
    // vo_pub_.shutdown();
    // gps_pub_.shutdown();
    sf_pub_.shutdown();
    map_pose_pub_.shutdown();
  
    map_pose_sub_.shutdown();
    sf_sub_.shutdown();
    fuse_sub_.shutdown();
    odom_sub_.shutdown();
    imu_sub_.shutdown();
    lo_sub_.shutdown();
  }
};

#endif
