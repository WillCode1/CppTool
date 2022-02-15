#include "bridge_separate_from_time.h"
#include "stub_odom_estimation_node.h"
#include <memory>

std::shared_ptr<Test_OdomEstimationNode> node = nullptr;

void test_bridge_separate_from_time::init()
{
    node = std::make_shared<Test_OdomEstimationNode>();
}

void test_bridge_separate_from_time::reset()
{
    node.reset();
}

void test_bridge_separate_from_time::imuCallback(const ImuConstPtr &imu)
{
    node->imuCallback(imu);
    node->imu_time_ = imu->header.stamp;
}

void test_bridge_separate_from_time::odomCallback(const OdomConstPtr &odom)
{
    node->odomCallback(odom);
    node->odom_time_ = odom->header.stamp;
}

void test_bridge_separate_from_time::loCallback(const OdomConstPtr &odom)
{
    node->loCallback(odom);
    node->lo_time_ = odom->header.stamp;
}

void test_bridge_separate_from_time::voCallback(const OdomConstPtr &odom)
{
    node->voCallback(odom);
    node->vo_time_ = odom->header.stamp;
}

void test_bridge_separate_from_time::gpsCallback(const OdomConstPtr &odom)
{
    node->gpsCallback(odom);
    node->gps_time_ = odom->header.stamp;
}

void test_bridge_separate_from_time::costmapTrackerCallback(const costmap_tracker::ObstacleArrayMsg::ConstPtr &obstacles)
{
    node->costmapTrackerCallback(obstacles);
}

double test_bridge_separate_from_time::getDynamicScenePercent()
{
    return 100. * node->is_dynamic_scene_cnt / node->total;
}

bool test_bridge_separate_from_time::ekfFuseCallback(nav_msgs::Odometry::Ptr &fuse, const ros::Time& fuse_time)
{
    return node->spin(*fuse, fuse_time);
}

Eigen::Vector3d test_bridge_separate_from_time::getTotalCompensationRadianAbs()
{
    return node->getTotalCompensationRadianAbs();
}
