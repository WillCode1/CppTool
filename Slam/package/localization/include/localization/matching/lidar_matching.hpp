/*
 * @Description: lidar matchiing
 * @Author: fei
 * @Data: 2021-09-23
 */

#ifndef LIDAR_MATCHING_HPP_
#define LIDAR_MATCHING_HPP_

#include <deque>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/registration/ndt.h>

#include "../sensor_data/sensor_data.hpp"

using namespace std;

namespace localization_node
{
    class LidarMatching
    {
    public:
        LidarMatching(ros::NodeHandle &nh, std::string topic_name, size_t buffserSize);
        LidarMatching() = default;
        ~LidarMatching();

        void InitGlobalMap(string cloud_file_name);
        void InitNDTRegistration(float res, float step_size, float trans_eps, int max_iter);
        bool ResetLocalMap(float x, float y, float z, float box_size);
        void setOdometry(string input_file);
        void runMatching();
        std::deque<OdomPoint> getLocatePose();

    private:
        void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr);
        std::vector<string> splitLineData(const std::string &str, const std::string &delim);
        PointCloudXYZIPtr boxFilter(PointCloudXYZIPtr input_cloud, vector<float> origin, float box_size);
        vector<float> calculateEdge(vector<float> origin, float box_size);
        vector<float> getEdge();
        bool setInitPose(const Eigen::Matrix4f &init_pose);
        bool setGNSSPose(const Eigen::Matrix4f &gnss_pose);
        bool UpatePose(const CloudData &cloud_data, Eigen::Matrix4f &cloud_pose);
        void PublishCloudData(PointCloudXYZIPtr input_cloud, ros::Publisher cloud_pub, float sample_size);
        void PublistOdomData(const Eigen::Matrix4f &transform_matrix, ros::Time time);

    private:
        ros::NodeHandle nh_;
        PointCloudXYZIPtr global_map_cloud_;
        PointCloudXYZIPtr local_map_cloud_;
        PointCloudXYZIPtr current_sweep_cloud_;
        std::deque<OdomPoint> lidar_odom_deque_;
        std::deque<CloudData> cloud_data_deque_;

        ros::Subscriber lidar_cloud_subscriber_;
        ros::Publisher global_map_pub_;
        ros::Publisher local_map_pub_;
        ros::Publisher current_scan_pub_;
        ros::Publisher laser_odom_pub_;

        tf::StampedTransform transform_;
        tf::TransformBroadcaster broadcaster_;

        pcl::NormalDistributionsTransform<pointXYZI, pointXYZI>::Ptr ndt_ptr_;

        bool init_global_map_;
        bool has_new_local_map_;
        bool init_pose_finished_;
        vector<float> origin_;
        vector<float> edge_;
        float box_size_;

        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f current_gnss_pose_ = Eigen::Matrix4f::Identity();

        // locate result
        std::deque<OdomPoint> locate_pose_deque_;
    };
}

#endif