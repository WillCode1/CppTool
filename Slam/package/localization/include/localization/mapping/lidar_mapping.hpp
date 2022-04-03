/*
* @Description: lidar mapping
* @Author: fei
* @Data: 2021-09-23
*/

#ifndef LIDAR_MAPPING_HPP_
#define LIDAR_MAPPING_HPP_

#include <deque>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "../sensor_data/sensor_data.hpp"

using namespace std;

namespace localization_node
{
    class LidarMapping
    {
    public:
        LidarMapping(ros::NodeHandle& nh, std::string topic_name, size_t buffer_size);
        LidarMapping() = default;
        ~LidarMapping();
        void setOdometry(string input_file);
        void runMapping();
        bool LookupData();
        PointCloudXYZIPtr getMapCloud();

    private:
        PointCloudXYZIPtr matchingMap(PointCloudXYZIPtr sweepCloud, OdomPoint curOdom);
        void msgCallBackHandler(const sensor_msgs::PointCloud2::ConstPtr & cloud_msg_ptr);
        std::vector<string> splitLineData(const std::string& str, const std::string& delim);

    private:
        ros::NodeHandle nh_;
        std::deque<OdomPoint> lidar_odom_deque_;
        std::deque<CloudData> cloud_data_deque_;
        PointCloudXYZIPtr map_cloud_ptr_;
        bool init_calibrate_;
        Eigen::Isometry3f lidar_imu_tran_;

        ros::Subscriber lidar_cloud_subscriber_;
        tf::TransformListener tf_listener_;
    };
}

#endif
