#ifndef __BRIDGE_SEPARATE_FROM_TIME__
#define __BRIDGE_SEPARATE_FROM_TIME__
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include <costmap_tracker/ObstacleArrayMsg.h>

using OdomConstPtr = boost::shared_ptr<nav_msgs::Odometry const>;
using ImuConstPtr = boost::shared_ptr<sensor_msgs::Imu const>;

class test_bridge_separate_from_time
{
public:
    void init();
    void reset();

    void imuCallback(const ImuConstPtr &odom);
    void odomCallback(const OdomConstPtr &odom);
    void loCallback(const OdomConstPtr &odom);
    void voCallback(const OdomConstPtr &odom);
    void gpsCallback(const OdomConstPtr &odom);
    void costmapTrackerCallback(const costmap_tracker::ObstacleArrayMsg::ConstPtr &obstacles);

    bool ekfFuseCallback(nav_msgs::Odometry::Ptr &fuse, const ros::Time& fuse_time);

    Eigen::Vector3d getTotalCompensationRadianAbs();
    double getDynamicScenePercent();
};

#endif
