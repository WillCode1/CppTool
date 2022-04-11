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

    double Calculate(pcl::PointCloud<pointXYZ>::Ptr check_map, const double& searchRadius, const std::vector<int>& index)
    {
        kdtree->setInputCloud(check_map);
        int m = index.size();
        int cnt = 0;
        double mme = 0;
        Eigen::Matrix3d covariance;
        double e = 2.718281828459045;

        for (auto i = 0; i < m; ++i)
        {
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtree->radiusSearch(check_map->points[index[i]], searchRadius, pointSearchInd, pointSearchSqDis, 0);

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

    std::vector<int> CalculateOverlap(const double& searchRadius, pcl::PointCloud<pointXYZ>::Ptr tag, pcl::PointCloud<pointXYZ>::Ptr cur)
    {
        std::vector<int> res;
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

            res.emplace_back(i);
        }

        return res;
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
        std::vector<int> gt_index, update_index;
        gt_index = CalculateOverlap(searchRadius, update_map_cloud_, gt_map_cloud_);
        update_index = CalculateOverlap(searchRadius, gt_map_cloud_, update_map_cloud_);
//        cloudAddNoise(update_map_cloud_, 0.1);
        std::cout << "gt = " << Calculate(gt_map_cloud_, searchRadius, gt_index) << std::endl;
        std::cout << "new = " << Calculate(update_map_cloud_, searchRadius, update_index) << std::endl;
    }

private:
    pcl::PointCloud<pointXYZ>::Ptr gt_map_cloud_;
    pcl::PointCloud<pointXYZ>::Ptr update_map_cloud_;

    pcl::PointCloud<pointXYZ>::Ptr gt_map_cloud_overlap_;
    pcl::PointCloud<pointXYZ>::Ptr update_map_cloud_overlap_;

    pcl::KdTreeFLANN<pointXYZ>::Ptr kdtree;
};
