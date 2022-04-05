#include "utility.h"
#include "lio_sam/cloud_info.h"
#include "featureExtraction.cpp"

class FeatureExtractionROS : public ParamServer
{

public:
    ros::Subscriber subLaserCloudInfo;

    // 发布当前激光帧提取特征之后的点云信息
    ros::Publisher pubLaserCloudInfo;
    // 发布当前激光帧提取的角点点云
    ros::Publisher pubCornerPoints;
    // 发布当前激光帧提取的平面点点云
    ros::Publisher pubSurfacePoints;

    pcl::PointCloud<PointType>::Ptr extractedCloud;
    // 当前激光帧角点点云集合
    pcl::PointCloud<PointType>::Ptr cornerCloud;
    // 当前激光帧平面点点云集合
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    std::shared_ptr<FeatureExtraction> feature_extraction;

    // 当前激光帧点云信息，包括的历史数据有：运动畸变校正，点云数据，初始位姿，姿态角，有效点云数据，角点点云，平面点点云等
    lio_sam::cloud_info cloudInfo;
    std_msgs::Header cloudHeader;

    FeatureExtractionROS()
    {
        feature_extraction = std::make_shared<FeatureExtraction>(N_SCAN, Horizon_SCAN, edgeThreshold, surfThreshold, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());

        // 订阅当前激光帧运动畸变校正后的点云信息
        subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtractionROS::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        // 发布当前激光帧提取特征之后的点云信息
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1);
        // 发布当前激光帧的角点点云
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
        // 发布当前激光帧的面点点云
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);
    }

    //接收imageProjection.cpp中发布的去畸变的点云，实时处理的回调函数
    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr &msgIn)
    {
        // msgIn即为回调函数获取的去畸变点云信息
        cloudInfo = *msgIn;                                      // new cloud info
        cloudHeader = msgIn->header;                             // new cloud header
        pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction

        feature_extraction->laserCloudHandler(extractedCloud, cloudInfo.pointRange, cloudInfo.pointColInd, cloudInfo.startRingIndex, cloudInfo.endRingIndex, cornerCloud, surfaceCloud);

        // 发布角点、面点点云，发布带特征点云数据的当前激光帧点云信息
        publishFeatureCloud();
    }

    /**
     * 清理
     */
    void freeCloudInfoMemory()
    {
        cloudInfo.startRingIndex.clear();
        cloudInfo.endRingIndex.clear();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointRange.clear();
    }

    /**
     * 发布角点、面点点云，发布带特征点云数据的当前激光帧点云信息
     */
    void publishFeatureCloud()
    {
        // free cloud info memory
        freeCloudInfoMemory();
        // save newly extracted features
        // 发布角点、面点点云，用于rviz展示
        cloudInfo.cloud_corner = publishCloud(&pubCornerPoints, cornerCloud, cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_surface = publishCloud(&pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        // publish to mapOptimization
        // 发布当前激光帧点云信息，加入了角点、面点点云数据，发布给mapOptimization
        // 和imageProjection.cpp发布的不是同一个话题，
        // image发布的是"lio_sam/deskew/cloud_info"，
        // 这里发布的是"lio_sam/feature/cloud_info"，
        // 因此不用担心地图优化部分的冲突
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_sam");

    FeatureExtractionROS FE;

    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");

    ros::spin();

    return 0;
}
