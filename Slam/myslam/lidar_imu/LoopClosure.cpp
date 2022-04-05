#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

using PointType = pcl::PointXYZI;
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
typedef PointXYZIRPYT  PointTypePose;


namespace lio
{
    bool LoopClosureByDiatance(pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D,
                               pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D,
                               const std::vector<pcl::PointCloud<PointType>::Ptr> &cloudKeyFrames,
                               Eigen::Affine3f& correctionLidarFrame)
    {
        double historyKeyframeSearchRadius = 15;     // m
        double historyKeyframeSearchFrameDiff = 300; // keyframe

        int loopKeyCur = cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;
        {
            std::vector<int> pointSearchIndLoop;
            std::vector<float> pointSearchSqDisLoop;
            pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;
            kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

            kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
            kdtreeHistoryKeyPoses->radiusSearch(cloudKeyPoses3D->back(), historyKeyframeSearchRadius,
                                                pointSearchIndLoop, pointSearchSqDisLoop, 0);

            for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
            {
                if (abs(pointSearchIndLoop[i] - loopKeyCur) > historyKeyframeSearchFrameDiff)
                {
                    loopKeyPre = pointSearchIndLoop[i];
                    break;
                }
            }

            if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
                return false;
        }

        // icp correction
        {
            int historyKeyframeSearchNum = 25;
            pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());

            double mappingSurfLeafSize = 0.4;
            pcl::VoxelGrid<PointType> downSizeFilterICP;
            downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

            auto transformPointCloud = [&](pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
            {
                pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
                int cloudSize = cloudIn->size();
                cloudOut->resize(cloudSize);
                Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(8)
                for (int i = 0; i < cloudSize; ++i)
                {
                    const auto &pointFrom = cloudIn->points[i];
                    cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
                    cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
                    cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
                    cloudOut->points[i].intensity = pointFrom.intensity;
                }
                return cloudOut;
            };

            auto loopFindNearKeyframes = [&](pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum)
            {
                nearKeyframes->clear();
                for (int i = -searchNum; i <= searchNum; ++i)
                {
                    int keyNear = key + i;
                    if (keyNear < 0 || keyNear >= cloudKeyPoses6D->size())
                        continue;

                    *nearKeyframes += *transformPointCloud(cloudKeyFrames[keyNear], &cloudKeyPoses6D->points[keyNear]);
                }

                if (nearKeyframes->empty())
                    return;

                pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
                downSizeFilterICP.setInputCloud(nearKeyframes);
                downSizeFilterICP.filter(*cloud_temp);
                *nearKeyframes = *cloud_temp;
            };

            double historyKeyframeFitnessScore = 0.3;
            static pcl::IterativeClosestPoint<PointType, PointType> icp;
            auto icpCorrect = [&](pcl::PointCloud<PointType>::Ptr cureKeyframeCloud, pcl::PointCloud<PointType>::Ptr prevKeyframeCloud)
            {
                icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
                icp.setMaximumIterations(100);
                icp.setTransformationEpsilon(1e-6);
                icp.setEuclideanFitnessEpsilon(1e-6);
                icp.setRANSACIterations(0);

                icp.setInputSource(cureKeyframeCloud);
                icp.setInputTarget(prevKeyframeCloud);
                pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
                icp.align(*unused_result);

                if (!icp.hasConverged() || icp.getFitnessScore() > historyKeyframeFitnessScore)
                    return false;

                return true;
            };

            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return false;

            if (!icpCorrect(cureKeyframeCloud, prevKeyframeCloud))
                return false;

            // Eigen::Affine3f tWrong = pcl::getTransformation(cloudKeyPoses6D->points[loopKeyCur].x,
            //                                                 cloudKeyPoses6D->points[loopKeyCur].y,
            //                                                 cloudKeyPoses6D->points[loopKeyCur].z,
            //                                                 cloudKeyPoses6D->points[loopKeyCur].roll,
            //                                                 cloudKeyPoses6D->points[loopKeyCur].pitch,
            //                                                 cloudKeyPoses6D->points[loopKeyCur].yaw);
            correctionLidarFrame = icp.getFinalTransformation();
            // Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
        }
        return true;
    }
}
