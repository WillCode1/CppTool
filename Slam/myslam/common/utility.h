#pragma once
#include <Eigen/Dense>
#include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
using namespace std;

/*************************************************************
 * The custom struct
*************************************************************/
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

/*************************************************************
 * pcl struct
*************************************************************/
/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
 */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

using PointType = pcl::PointXYZI;
using PointTypePose = PointXYZIRPYT;

/*************************************************************
 * Tool
*************************************************************/
class Tool{
public:
    /*
    imuConverter函数，这个函数之后会被频繁调用。它主要的作用，是把IMU的信息，从IMU坐标系，转换到雷达坐标系。
    注意，这个函数只旋转，没有平移，和真正的雷达坐标系之间还是差了一个平移的。
    至于为什么没有平移，先提前剧透一下，在imuPreintegration.cpp文件中，还有两个imu2Lidar，lidar2imu变量，这俩变量只有平移，没有旋转。
    事实上，作者后续是把imu数据先用imuConverter旋转到雷达系下（但其实还差了个平移）。然后他把雷达数据又根据lidar2Imu反向平移了一下，和转换以后差了个平移的imu数据在“中间系”对齐，之后算完又从中间系通过imu2Lidar挪回了雷达系进行publish。
    */
    static ImuData imuConverter(const ImuData &imu_in, const Eigen::Matrix3d& extRot, const Eigen::Quaterniond& extQRPY)
    {
        ImuData imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration);
        acc = extRot * acc;
        imu_out.linear_acceleration = acc;
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity);
        gyr = extRot * gyr;
        imu_out.angular_velocity = gyr;
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation = q_final;

        if (sqrt(q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() + q_final.w() * q_final.w()) < 0.1)
        {
            std::cout << "Invalid quaternion, please use a 9-axis IMU!" << std::endl;
        }

        return imu_out;
    }

    static float pointDistance(PointType p)
    {
        return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    }

    static float pointDistance(PointType p1, PointType p2)
    {
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
    }
};
