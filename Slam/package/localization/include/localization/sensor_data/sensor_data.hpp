/*
* @Description: definition of sensor data
* @Author: fei
* @Data: 2021-09-26
*/

#ifndef SENSOR_DATA_HPP_
#define SENSOR_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

using pointXYZI = pcl::PointXYZI;
using PointCloudXYZI = pcl::PointCloud<pointXYZI>;
using PointCloudXYZIPtr = PointCloudXYZI::Ptr;

struct OdomPoint
{
    double timestamp;

    double x;
    double y;
    double z;

    double q_x;
    double q_y;
    double q_z;
    double q_w;
};

struct CloudData
{
    double timestamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sweepCloudPtr;
};

#endif