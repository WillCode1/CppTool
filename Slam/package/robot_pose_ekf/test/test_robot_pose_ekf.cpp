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
#include <string>
#include <boost/thread.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>
using namespace ros;

static const double EPS_trans_x = 0.02;
static const double EPS_trans_y = 0.04;
static const double EPS_trans_z = 0.00001;
static const double EPS_rot_x = 0.005;
static const double EPS_rot_y = 0.005;
static const double EPS_rot_z = 0.005;
static const double EPS_rot_w = 0.005;

/*
  方案1：error = fuse_end - fuse_start
  方案2：error = global_end - toGlobal(fuse_end)

  运行的数据，可能一开始就有累计误差。所以，如果单次数据回到起始位置可以使用方案1。

  坐标变换
  offset = [fuse0 - global0]
  error = [fusen - offset] - globaln
 */
/*
  方案1： 闭环轨迹。首尾位置应该重合
  error = fuse_end - fuse_start
 */
#if 0
TEST_F(TestEKF, closed_loop_trajectory_method1)
{
  std::string bag_path;
  std::vector<std::string> topics;
  topics.emplace_back(std::string("/peter_motor_core/odom"));
  topics.emplace_back(std::string("/imu"));
  topics.emplace_back(std::string("/odom_rf2o"));
  topics.emplace_back(std::string("/sf"));
  topics.emplace_back(std::string("/map_server/robot_pose"));

  bag_path = "/home/will/catkin_ws/src/robot_pose_ekf/bagfiles/oneloop/8309_2022-01-03-21-01-39.bag";
  readBagAndPublishTopic(bag_path, topics, 30.f, 1000);

  bag_path = "/home/will/catkin_ws/src/robot_pose_ekf/bagfiles/oneloop/front_desk_1F_1_2022-01-03-21-04-03.bag";
  readBagAndPublishTopic(bag_path, topics, 30.f, 1000);

  evaluateError("odom", odom_end_, odom_start_, run_dis_odom_, run_angle_odom_, 0.1, 0.01);
  evaluateError("lo", lo_end_, lo_start_, run_dis_lo_, run_angle_lo_, 0.03, 0.007);
  evaluateError("imu", imu_end_, imu_start_, 0, run_angle_imu_, 1000, 0.001);
  evaluateError("sf", sf_end_, sf_start_, run_dis_sf_, run_angle_sf_, 0.03, 0.003);
  evaluateError("fuse", fuse_end_, fuse_start_, run_dis_fuse_, run_angle_fuse_, 0.022, 0.001);
}
#endif

/*
  方案2： 使用粒子滤波评估累计误差
  error = global_end - toGlobal(fuse_end)
 */
#if 1
TEST_F(TestEKF, long_corridor_method2)
{
  std::string bag_path = "/home/will/bagfiles/test/HOTYC04SZ202101153683684/suc/1101_2022-02-16-12-06-37.bag";

  WaitForBagFinished(bag_path);

  // map_end_.translation_ += Vector3d(-0.42, -0.76, 0);
  // map_end_.translation_ += Vector3d(-0.72, -0.3, 0);

  evaluateError("odom", Trans2DstSysTarget(odom_start_, map_start_, odom_end_), map_end_, run_dis_odom_, run_angle_odom_);
  evaluateError("lo", Trans2DstSysTarget(lo_start_, map_start_, lo_end_), map_end_, run_dis_lo_, run_angle_lo_);
  evaluateError("imu", Trans2DstSysTarget(imu_start_, map_start_, imu_end_), map_end_, 0, run_angle_imu_);
  evaluateError("sf", Trans2DstSysTarget(sf_start_, map_start_, sf_end_), map_end_, run_dis_sf_, run_angle_sf_, 0.03, 0.005);
  evaluateError("fuse", Trans2DstSysTarget(fuse_start_, map_start_, fuse_end_), map_end_, run_dis_fuse_, run_angle_fuse_, 0.02, 0.002);
}
#endif


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "testEKF");

  boost::thread spinner(boost::bind(&ros::spin));

  int res = RUN_ALL_TESTS();
  spinner.interrupt();
  spinner.join();

  return res;
}
