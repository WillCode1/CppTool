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
#include "test_robot_pose_ekf.h"
#include "test_robot_pose_ekf_separate_from_time.h"
#include <string>
#include <boost/thread.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <errno.h>
using namespace ros;

static const double EPS_trans_x = 0.02;
static const double EPS_trans_y = 0.04;
static const double EPS_trans_z = 0.00001;
static const double EPS_rot_x = 0.005;
static const double EPS_rot_y = 0.005;
static const double EPS_rot_z = 0.005;
static const double EPS_rot_w = 0.005;


#if 0
TEST_F(TestEKF_separate_from_time, run_bag)
{
  std::string bags_path;
  ros::NodeHandle nh_local("~");
  nh_local.param("bag_path", bags_path, std::string("/home/will/bagfiles/test/"));

  std::vector<std::string> topics;
  topics.emplace_back(std::string("/peter_motor_core/odom"));
  topics.emplace_back(std::string("/imu"));
  topics.emplace_back(std::string("/odom_rf2o"));
  topics.emplace_back(std::string("/sf"));
  topics.emplace_back(std::string("/map_server/robot_pose"));
  topics.emplace_back(std::string("/scan"));

  ROS_INFO_STREAM("test bag file: " << bags_path);
  loadBagAndSendMsg(bags_path, topics, 30);

  const auto &tmp = getTotalCompensationRadianAbs();
  ROS_WARN("Total Zero Drift Compensation Radian Abs: [%f, %f, %f]", tmp.x(), tmp.y(), tmp.z());
  evaluateError("odom", Trans2DstSysTarget(odom_start_, map_start_, odom_end_), map_end_, run_dis_odom_, run_angle_odom_);
  evaluateError("lo", Trans2DstSysTarget(lo_start_, map_start_, lo_end_), map_end_, run_dis_lo_, run_angle_lo_);
  evaluateError("imu", Trans2DstSysTarget(imu_start_, map_start_, imu_end_), map_end_, 0, run_angle_imu_);
  evaluateError("sf", Trans2DstSysTarget(sf_start_, map_start_, sf_end_), map_end_, run_dis_sf_, run_angle_sf_, 0.03, 0.005);
  evaluateError("fuse", Trans2DstSysTarget(fuse_start_, map_start_, fuse_end_), map_end_, run_dis_fuse_, run_angle_fuse_, 0.02, 0.002);
}
#endif

#if 1
TEST_F(TestEKF_separate_from_time, all_successful_bag_lo)
{
  std::string bags_path;
  ros::NodeHandle nh_local("~");
  nh_local.param("test_path", bags_path, std::string("/home/will/bagfiles/test/"));

  std::vector<std::string> topics;
  topics.emplace_back(std::string("/peter_motor_core/odom"));
  topics.emplace_back(std::string("/imu"));
  topics.emplace_back(std::string("/odom_rf2o"));
  topics.emplace_back(std::string("/sf"));
  topics.emplace_back(std::string("/map_server/robot_pose"));
  topics.emplace_back(std::string("/standalone_tracker/costmap_obstacles"));
  topics.emplace_back(std::string("/scan"));
  // metrics_.setLocalTrajectoryName(std::vector<std::string>{"/odom", "/imu", "/lo", "/sf", "/fuse"});
  // metrics_.setLocalTrajectoryName(std::vector<std::string>{"/sf", "/fuse"});

  std::vector<string> bags;
  getFiles(bags_path, bags, "suc", ".bag");
  ROS_WARN("Total %lu bags!", bags.size());
  for (auto i = 0; i < bags.size(); ++i)
  {
    SetUp();
    ROS_INFO("test bag [%d/%lu], file: %s", i+1, bags.size(), bags[i].c_str());
    loadBagAndSendMsg(bags[i], topics, 30);

    // metrics_.evaluate();
    // metrics_.traverseError("/fuse");
    // metrics_.traverseError("/sf");
    // std::cout << "/fuse = " << metrics_.getDisErrorTopMean("/fuse") << std::endl;
    // std::cout << "/fuse = " << metrics_.getAngErrorTopMean("/fuse") << std::endl;
    // std::cout << "/sf = " << metrics_.getDisErrorTopMean("/sf") << std::endl;
    // std::cout << "/sf = " << metrics_.getAngErrorTopMean("/sf") << std::endl;
    // std::cout << "/fuse = " << metrics_.getDisErrorMean("/fuse") << std::endl;
    // std::cout << "/fuse = " << metrics_.getAngErrorMean("/fuse") << std::endl;
    // std::cout << "/sf = " << metrics_.getDisErrorMean("/sf") << std::endl;
    // std::cout << "/sf = " << metrics_.getAngErrorMean("/sf") << std::endl;

    const auto& tmp = getTotalCompensationRadianAbs();
    ROS_WARN("drop lo times percent %f%%!", getDropLoTimesPercent());
    ROS_WARN("dynamic scene times percent %f%%!", getDynamicScenePercent());
    ROS_WARN("Total Zero Drift Compensation Radian Abs: [%f, %f, %f]", tmp.x(), tmp.y(), tmp.z());
    ROS_WARN_COND(run_dis_sf_ < 10, "run_dis_sf too short: [%f, %f]", run_dis_sf_, run_angle_sf_);
    evaluateError("odom", Trans2DstSysTarget(odom_start_, map_start_, odom_end_), map_end_, run_dis_odom_, run_angle_odom_);
    evaluateError("lo", Trans2DstSysTarget(lo_start_, map_start_, lo_end_), map_end_, run_dis_lo_, run_angle_lo_);
    evaluateError("imu", Trans2DstSysTarget(imu_start_, map_start_, imu_end_), map_end_, 0, run_angle_imu_);
    evaluateError("sf", Trans2DstSysTarget(sf_start_, map_start_, sf_end_), map_end_, run_dis_sf_, run_angle_sf_, 0.03, 0.005);
    evaluateError("fuse", Trans2DstSysTarget(fuse_start_, map_start_, fuse_end_), map_end_, run_dis_fuse_, run_angle_fuse_, 0.02, 0.002);
    
    ROS_INFO_STREAM("one bag end!");
    TearDown();
  }
}
#endif

#if 1
TEST_F(TestEKF_separate_from_time, all_failed_bag_lo)
{
  std::string bags_path;
  ros::NodeHandle nh_local("~");
  nh_local.param("test_path", bags_path, std::string("/home/will/bagfiles/test/"));

  std::vector<std::string> topics;
  topics.emplace_back(std::string("/peter_motor_core/odom"));
  topics.emplace_back(std::string("/imu"));
  topics.emplace_back(std::string("/odom_rf2o"));
  topics.emplace_back(std::string("/sf"));
  topics.emplace_back(std::string("/map_server/robot_pose"));
  topics.emplace_back(std::string("/standalone_tracker/costmap_obstacles"));
  topics.emplace_back(std::string("/scan"));
  // metrics_.setLocalTrajectoryName(std::vector<std::string>{"/odom", "/imu", "/lo", "/sf", "/fuse"});
  // metrics_.setLocalTrajectoryName(std::vector<std::string>{"/sf", "/fuse"});

  std::vector<string> bags;
  getFiles(bags_path, bags, "fail", ".bag");
  ROS_WARN("Total %lu bags!", bags.size());
  for (auto i = 0; i < bags.size(); ++i)
  {
    SetUp();
    ROS_INFO("test bag [%d/%lu], file: %s", i+1, bags.size(), bags[i].c_str());
    loadBagAndSendMsg(bags[i], topics, 30);

    // metrics_.evaluate();
    // metrics_.traverseError("/fuse");
    // metrics_.traverseError("/sf");
    // std::cout << "/fuse = " << metrics_.getDisErrorTopMean("/fuse") << std::endl;
    // std::cout << "/fuse = " << metrics_.getAngErrorTopMean("/fuse") << std::endl;
    // std::cout << "/sf = " << metrics_.getDisErrorTopMean("/sf") << std::endl;
    // std::cout << "/sf = " << metrics_.getAngErrorTopMean("/sf") << std::endl;
    // std::cout << "/fuse = " << metrics_.getDisErrorMean("/fuse") << std::endl;
    // std::cout << "/fuse = " << metrics_.getAngErrorMean("/fuse") << std::endl;
    // std::cout << "/sf = " << metrics_.getDisErrorMean("/sf") << std::endl;
    // std::cout << "/sf = " << metrics_.getAngErrorMean("/sf") << std::endl;
    
    const auto& tmp = getTotalCompensationRadianAbs();
    ROS_WARN("drop lo times percent %f%%!", getDropLoTimesPercent());
    ROS_WARN("dynamic scene times percent %f%%!", getDynamicScenePercent());
    ROS_WARN("Total Zero Drift Compensation Radian Abs: [%f, %f, %f]", tmp.x(), tmp.y(), tmp.z());
    evaluateError("odom", Trans2DstSysTarget(odom_start_, map_start_, odom_end_), map_end_, run_dis_odom_, run_angle_odom_);
    evaluateError("lo", Trans2DstSysTarget(lo_start_, map_start_, lo_end_), map_end_, run_dis_lo_, run_angle_lo_);
    evaluateError("imu", Trans2DstSysTarget(imu_start_, map_start_, imu_end_), map_end_, 0, run_angle_imu_);
    evaluateError("sf", Trans2DstSysTarget(sf_start_, map_start_, sf_end_), map_end_, run_dis_sf_, run_angle_sf_, 0.03, 0.005);
    evaluateError("fuse", Trans2DstSysTarget(fuse_start_, map_start_, fuse_end_), map_end_, run_dis_fuse_, run_angle_fuse_, 0.02, 0.002);
    
    ROS_INFO_STREAM("one bag end!");
    TearDown();
  }
}
#endif


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "testEKF");

  auto res = setpriority(PRIO_PROCESS, getpid(), 0);
  if (res != 0)
  {
    ROS_INFO_STREAM("setpriority failed!, errno: " << errno);
    return 0;
  }

  boost::thread spinner(boost::bind(&ros::spin));

  res = RUN_ALL_TESTS();
  spinner.interrupt();
  spinner.join();

  return res;
}
