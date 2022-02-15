#ifndef __TEST_ROBOT_POSE_EKF_SEPARATE_FROM_TIME__
#define __TEST_ROBOT_POSE_EKF_SEPARATE_FROM_TIME__

#include "test_robot_pose_ekf.h"
#include "bridge_separate_from_time.h"
#include <memory>
#include <string>
#include <vector>
#include <sys/types.h>
#include <dirent.h>
#include "rf2o_laser_odometry/CLaserOdometry2D.h"
using namespace ros;
using namespace Eigen;

using OdomConstPtr = boost::shared_ptr<nav_msgs::Odometry const>;
using ImuConstPtr = boost::shared_ptr<sensor_msgs::Imu const>;


class TestEKF_separate_from_time: public TestEKF
{
    std::shared_ptr<rf2o::CLaserOdometry2DNode> rf2o;
    test_bridge_separate_from_time bridge;
public:
    void getFiles(const std::string &bag_path, std::vector<string> &files, const std::string& path_key, const std::string& fileType)
    {
        DIR *dir;
        if ((dir = opendir(bag_path.c_str())) == nullptr)
        {
            throw std::runtime_error("directory " + bag_path + " does not exist");
        }
        for (dirent *dp = readdir(dir); dp != nullptr; dp = readdir(dir))
        {
            std::string fileName(dp->d_name);

            if ((dp->d_type == DT_REG) &&
                (bag_path.find(path_key) != std::string::npos) &&
                (fileType.compare(strchr(fileName.c_str(), '.')) == 0))
            {
                files.push_back(bag_path + "/" + fileName.c_str());
            }
            else if ((dp->d_type == DT_DIR) && (fileName.compare(".") != 0) && (fileName.compare("..") != 0))
            {
                getFiles(bag_path + "/" + fileName.c_str(), files, path_key, fileType);
            }
        }
        closedir(dir);

        std::sort(files.begin(), files.end());
    }
    
    void loadBagAndSendMsg(const std::string &bag_path, const std::vector<std::string> &topics, const double& fuse_freq)
    {
        rosbag::Bag bag;
        bag.open(bag_path, rosbag::bagmode::Read);
        ASSERT_EQ(bag.isOpen(), true);

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        ros::Time bag_begin_time = view.getBeginTime();
        ros::Time bag_end_time = view.getEndTime();
        ROS_WARN("ROS bag duration time: %f(s)", (bag_end_time - bag_begin_time).toSec());
        duration_ += (bag_end_time - bag_begin_time).toSec();

        sendMsgAndDealByTimeStep(view, fuse_freq);
        bag.close();
    }

    void sendMsgAndDealByTimeStep(rosbag::View &view, const double& fuse_freq)
    {
        nav_msgs::Odometry::Ptr lo = boost::make_shared<nav_msgs::Odometry>();
        nav_msgs::Odometry::Ptr fuse = boost::make_shared<nav_msgs::Odometry>();
        auto step = ros::Duration(1.0 / fuse_freq);
        auto fuse_time = view.getBeginTime();

        for (rosbag::MessageInstance const &m : view)
        {
            if (m.getTime() >= fuse_time)
            {
                if (bridge.ekfFuseCallback(fuse, fuse_time))
                {
                    FuseCallback(fuse);
                }
                fuse_time += step;
            }

            const std::string &topic = m.getTopic();
            nav_msgs::Odometry::ConstPtr odom;
            sensor_msgs::Imu::ConstPtr imu;
            map_server::RobotPose::ConstPtr map_pose;
            sensor_msgs::LaserScan::ConstPtr scan;
            costmap_tracker::ObstacleArrayMsg::ConstPtr obstacles;

            if (topic.compare("/imu") == 0)
            {
                imu = m.instantiate<sensor_msgs::Imu>();
            }
            else if (topic.compare("/map_server/robot_pose") == 0)
            {
                map_pose = m.instantiate<map_server::RobotPose>();
            }
            else if (topic.compare("/scan") == 0)
            {
                scan = m.instantiate<sensor_msgs::LaserScan>();
            }
            else if (topic.compare("/standalone_tracker/costmap_obstacles") == 0)
            {
                obstacles = m.instantiate<costmap_tracker::ObstacleArrayMsg>();
            }
            else
            {
                odom = m.instantiate<nav_msgs::Odometry>();
            }

            // ASSERT_EQ(imu == nullptr && odom == nullptr && map_pose == nullptr && obstacles == nullptr && scan == nullptr, false);

            if (topic.compare("/imu") == 0)
            {
                bridge.imuCallback(imu);
                ImuCallback(imu);
            }
            else if (topic.compare("/peter_motor_core/odom") == 0)
            {
                bridge.odomCallback(odom);
                OdomCallback(odom);
            }
            // else if (topic.compare("/odom_rf2o") == 0)
            // {
            //     bridge.loCallback(odom);
            //     LoCallback(odom);
            // }
            else if (topic.compare("/sf") == 0)
            {
                SfCallback(odom);
            }
            else if (topic.compare("/map_server/robot_pose") == 0)
            {
                MapPoseCallback(map_pose);
            }
            else if (topic.compare("/standalone_tracker/costmap_obstacles") == 0)
            {
                bridge.costmapTrackerCallback(obstacles);
            }
            else if (topic.compare("/scan") == 0)
            {
                rf2o->LaserCallBack(scan);
                if (rf2o->process(lo))
                {
                    bridge.loCallback(lo);
                    LoCallback(lo);
                }
            }
        }

        if (bridge.ekfFuseCallback(fuse, fuse_time))
        {
            FuseCallback(fuse);
        }
    }

    Eigen::Vector3d getTotalCompensationRadianAbs() { return bridge.getTotalCompensationRadianAbs(); }
    double getDropLoTimesPercent() { return rf2o->drop_lo_times_per; }
    double getDynamicScenePercent() { return bridge.getDynamicScenePercent(); }

protected:
    void SetUp()
    {
        TestEKF::SetUp();
        bridge.init();
        rf2o = std::make_shared<rf2o::CLaserOdometry2DNode>();
    }

    void TearDown()
    {
        TestEKF::TearDown();
        bridge.reset();
        rf2o.reset();
    }
};

#endif
