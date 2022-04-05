#ifndef AW_MAPPING_ROBOT_SCAN2MAP_H
#define AW_MAPPING_ROBOT_SCAN2MAP_H
#include "aw_mapping_robot/lio/utility.h"
#include "aw_mapping_robot/lio/map_optimization.h"

namespace aw
{
    namespace mapping_robot
    {
        namespace lio
        {
            //TODO
            class Scan2Map : public ParamServer
            {
                public:
                    using Ptr = std::shared_ptr<Scan2Map>;
                    Scan2Map();

                    void updatePrePose(const gtsam::Pose3 &pose);
                    void updatePose(const gtsam::Pose3 &pose);
                    void updateMap();

                    bool run();
                    void setDeltaTInit(const gtsam::Pose3 &deltaT);
                    void correctPoses(const gtsam::Values &isamCurrentEstimate);

                    void extractNearby();
                    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract);
                    void extractSurroundingKeyFrames(); 

                    void downsampleCurrentScan();
                    void updatePointAssociateToMap();
                    void cornerOptimization();

                    void surfOptimization();
                    void combineOptimizationCoeffs();
                    bool LMOptimization(int iterCount);

                    void scan2MapOptimization();
                    void transformUpdate();
                    void pointAssociateToMap(PointType const *const pi, PointType *const po);

                    float constraintTransformation(float value, float limit);
                    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);
                    void publishOdometry();
                    void publishGlobalMap();
                    void updatePath(const PointTypePose &pose_in);
                public:

                    std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
                    std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

                    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
                    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

                    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
                    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;   // downsampled surf featuer set from odoOptimization

                    pcl::PointCloud<PointType>::Ptr laserCloudOri;
                    pcl::PointCloud<PointType>::Ptr coeffSel;

                    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
                    std::vector<PointType> coeffSelCornerVec;
                    std::vector<bool> laserCloudOriCornerFlag;
                    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
                    std::vector<PointType> coeffSelSurfVec;
                    std::vector<bool> laserCloudOriSurfFlag;

                    std::map<int, std::pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
                    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
                    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
                    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
                    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

                    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
                    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

                    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
                    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

                    pcl::VoxelGrid<PointType> downSizeFilterCorner;
                    pcl::VoxelGrid<PointType> downSizeFilterSurf;
                    pcl::VoxelGrid<PointType> downSizeFilterICP;
                    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

                    ros::Time timeLaserInfoStamp;
                    double timeLaserInfoCur;

                    std::vector<double> transformTobeMapped;
                    std::vector<double> deltaTInit;
                    std::vector<double> T;// incre transformation
                    Eigen::Affine3f transPointAssociateToMap;
                    bool isDegenerate = false;
                    cv::Mat matP;

                    int laserCloudCornerFromMapDSNum = 0;
                    int laserCloudSurfFromMapDSNum = 0;
                    int laserCloudCornerLastDSNum = 0;
                    int laserCloudSurfLastDSNum = 0;

                    ros::Publisher pubLaserOdometryIncremental;
                    ros::Publisher pubLaserOdometryGlobal;
                    ros::Publisher pubLaserCloudSurround;
                    ros::Publisher pubPath;

                    std::mutex mtx;
                    tf::TransformListener tfListener;
                    tf::StampedTransform lidar2Baselink;
                    nav_msgs::Path globalPath;

            };

        }
    }
}
#endif
