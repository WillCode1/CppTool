#pragma once
#include <Eigen/Dense>

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
#include <deque>
#include <iostream>
#include <fstream>
#include <string>
#include "Random.hpp"
#include <pcl/visualization/cloud_viewer.h>
using pointXYZ = pcl::PointXYZ;
using namespace std;
/*
    Mean Map Entropy (MME) is proposed to measure the compactness of a point cloud.
    It has been explored as a standard metric to assess the quality of registration if ground truth is unavailable.
    Given a calibrated point cloud, the normalized mean map entropy is where m is the size of the point cloud and Cpi is the sampling covariance within a local radius r around pi.
    For each calibration case, we select 10 consecutive frames of point clouds that contain many planes and compute their average MME values for evaluation.
 */
class MeanMapEntropyMetrics
{
public:
    MeanMapEntropyMetrics()
    {
        gt_map_cloud_.reset(new pcl::PointCloud<pointXYZ>());
        update_map_cloud_.reset(new pcl::PointCloud<pointXYZ>());
        gt_map_cloud_overlap_.reset(new pcl::PointCloud<pointXYZ>());
        update_map_cloud_overlap_.reset(new pcl::PointCloud<pointXYZ>());

        kdtree.reset(new pcl::KdTreeFLANN<pointXYZ>());
    }

    void LoadGtMap(const std::string &cloud_file_name)
    {
        pcl::io::loadPCDFile(cloud_file_name, *gt_map_cloud_);
    }

    void LoadCurMap(const std::string &cloud_file_name)
    {
        pcl::io::loadPCDFile(cloud_file_name, *update_map_cloud_);
    }

    double Calculate(pcl::PointCloud<pointXYZ>::Ptr check_map, const double& searchRadius)
    {
        kdtree->setInputCloud(check_map);

        int m = check_map->size();
        int cnt = 0;
        double mme = 0;
        Eigen::Matrix3d covariance;
        double e = 2.718281828459045;

        for (auto i = 0; i < m; ++i)
        {
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtree->radiusSearch(check_map->points[i], searchRadius, pointSearchInd, pointSearchSqDis, 0);

            if (pointSearchInd.size() < 2)
                continue;

            double mean_x = 0;
            double mean_y = 0;
            double mean_z = 0;

            for (int j : pointSearchInd)
            {
                mean_x += check_map->points[j].x;
                mean_y += check_map->points[j].y;
                mean_z += check_map->points[j].z;
            }

            mean_x /= pointSearchInd.size();
            mean_y /= pointSearchInd.size();
            mean_z /= pointSearchInd.size();

            double xx = 0;
            double xy = 0;
            double xz = 0;
            double yy = 0;
            double yz = 0;
            double zz = 0;

            for (int j : pointSearchInd)
            {
                double x = check_map->points[j].x - mean_x;
                double y = check_map->points[j].y - mean_y;
                double z = check_map->points[j].z - mean_z;

                xx += x * x;
                xy += x * y;
                xz += x * z;
                yy += y * y;
                yz += y * z;
                zz += z * z;
            }

            xx /= pointSearchInd.size() - 1;
            xy /= pointSearchInd.size() - 1;
            xz /= pointSearchInd.size() - 1;
            yy /= pointSearchInd.size() - 1;
            yz /= pointSearchInd.size() - 1;
            zz /= pointSearchInd.size() - 1;

            covariance << xx, xy, xz, xy, yy, yz, xz, yz, zz;
            covariance *= 2 * M_PI * e;
            auto tmp = std::log(covariance.determinant()) / std::log(e);

            if (!isfinite(tmp))
                continue;

//            std::cout << covariance.determinant() << std::endl;
//            std::cout << tmp << std::endl;

            cnt++;
            mme += tmp;
        }

        std::cout << cnt << std::endl;
        std::cout << m << std::endl;

        mme /= cnt;
        return mme;
    }

    pcl::PointCloud<pointXYZ>::Ptr CalculateOverlap2(pcl::PointCloud<pointXYZ>::Ptr tag,
                                                     pcl::PointCloud<pointXYZ>::Ptr cur,
                                                     const double &sigma = 0.0)
    {
        pcl::PointCloud<pointXYZ>::Ptr res;
        res.reset(new pcl::PointCloud<pointXYZ>());

        double searchRadius = 0.3;
        kdtree->setInputCloud(tag);
        int m = cur->size();
        for (auto i = 0; i < m; ++i)
        {
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtree->radiusSearch(cur->points[i], searchRadius, pointSearchInd, pointSearchSqDis, 0);

            if (pointSearchInd.empty())
            {
                continue;
            }

            if (sigma != 0)
                res->emplace_back(addNoise(cur->points[i], sigma));
            else
                res->emplace_back(cur->points[i]);
        }

        return res;
    }

    void boxFilter(pcl::PointCloud<pointXYZ>::Ptr input_cloud, const pcl::PointXYZ& min, const pcl::PointXYZ& max)
    {
        pcl::CropBox<pointXYZ> cropBoxFilter;
        cropBoxFilter.setMin(Eigen::Vector4f(min.x, min.y, min.z, 1.));
        cropBoxFilter.setMax(Eigen::Vector4f(max.x, max.y, max.z, 1.));
        cropBoxFilter.setInputCloud(input_cloud);
        cropBoxFilter.filter(*input_cloud);
    }

    bool CalculateOverlap(pcl::PointCloud<pointXYZ>::Ptr cur1, pcl::PointCloud<pointXYZ>::Ptr cur2)
    {
        pcl::PointXYZ minpt, maxpt;
        pcl::getMinMax3D(*cur1, minpt, maxpt);
        pcl::PointXYZ minpt2, maxpt2;
        pcl::getMinMax3D(*cur2, minpt2, maxpt2);

        if (!(maxpt.x >= minpt2.x && maxpt.y >= maxpt2.y && maxpt.z >= maxpt2.z &&
              maxpt2.x >= minpt.x && maxpt2.y >= minpt.y && maxpt2.z >= minpt.z))
        {
            return false;
        }

        pcl::PointXYZ minpt3, maxpt3;
        minpt3.x = minpt.x > minpt2.x ? minpt.x : minpt2.x;
        minpt3.y = minpt.y > minpt2.y ? minpt.y : minpt2.y;
        minpt3.z = minpt.z > minpt2.z ? minpt.z : minpt2.z;

        maxpt3.x = maxpt.x < maxpt2.x ? maxpt.x : maxpt2.x;
        maxpt3.y = maxpt.y < maxpt2.y ? maxpt.y : maxpt2.y;
        maxpt3.z = maxpt.z < maxpt2.z ? maxpt.z : maxpt2.z;

        boxFilter(cur1, minpt3, maxpt3);
        boxFilter(cur2, minpt3, maxpt3);
        return true;
    }

    void cloudAddNoise(pcl::PointCloud<pointXYZ>::Ptr cur, const double &sigma)
    {
        for (auto i = 0; i < cur->size(); ++i)
        {
            cur->points[i] = addNoise(cur->points[i], sigma);
        }
    }

    pointXYZ addNoise(const pointXYZ &asd, const double &sigma)
    {
        pointXYZ res;
        res.x = asd.x + Random::RandNormal(0, sigma);
        res.y = asd.y + Random::RandNormal(0, sigma);
        res.z = asd.z + Random::RandNormal(0, sigma);
        return res;
    }

    void draw(pcl::PointCloud<pointXYZ>::Ptr cloud)
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);

        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> RandomColor(cloud);//设置随机颜色
        viewer->addPointCloud<pcl::PointXYZ>(cloud, RandomColor, "points");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points");
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
        }
    }

    void Metrics(const double& searchRadius, const double &sigma)
    {
//        CalculateOverlap(update_map_cloud_, gt_map_cloud_);
//        cloudAddNoise(update_map_cloud_, 0.1);
        gt_map_cloud_ = CalculateOverlap2(update_map_cloud_, gt_map_cloud_);
        update_map_cloud_ = CalculateOverlap2(gt_map_cloud_, update_map_cloud_, sigma);
//        draw(gt_map_cloud_);
//        draw(update_map_cloud_);
        std::cout << "gt = " << Calculate(gt_map_cloud_, searchRadius) << std::endl;
        std::cout << "new = " << Calculate(update_map_cloud_, searchRadius) << std::endl;
    }

private:
    pcl::PointCloud<pointXYZ>::Ptr gt_map_cloud_;
    pcl::PointCloud<pointXYZ>::Ptr update_map_cloud_;

    pcl::PointCloud<pointXYZ>::Ptr gt_map_cloud_overlap_;
    pcl::PointCloud<pointXYZ>::Ptr update_map_cloud_overlap_;

    pcl::KdTreeFLANN<pointXYZ>::Ptr kdtree;
};

// 0, new = -4.35793
// 0.05, new = -3.39328
// 0.10, new = -2.96566
// 0.20, new = -2.76492
// 0.30, new = -2.90125
// 0.40, new = -3.14307
// 0.50, new = -3.41055
// 0.60, new = -3.67453
// 0.70, new = -3.9725
// 0.80, new = -4.2561
// 1.00, new = -4.85561

// 0., new = 0.613592
// 1.0, new = 3.35743
// 0.8, new = 3.3204
// 0.5, new = 3.13729
// 0.2, new = 2.4161
// 0.1, new = 1.78008
// 0.05, new = 1.30424
// 0.02, new = 0.921232

