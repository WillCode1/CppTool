#pragma once
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
using namespace std;

struct Pose
{
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

struct Twist
{
    Eigen::Vector3d linear;
    Eigen::Vector3d angular;
};

struct OdometryData
{
    double stamp;
    Pose pose;
    Twist twist;
};

struct ImuData
{
    double stamp;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d linear_acceleration;
    Eigen::Quaterniond orientation;
};

struct PointCloudData
{
    double stamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sweepCloudPtr;
};
