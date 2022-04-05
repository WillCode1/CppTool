#include "aw_mapping_robot/map_appender/scan2map.h"
namespace aw
{
    namespace mapping_robot
    {
        namespace lio
        {

            Scan2Map::Scan2Map() : ParamServer()
            {

                downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
                downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
                downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
                downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

                cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
                cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

                kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
                kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

                laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
                laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());   // downsampled surf featuer set from odoOptimization

                laserCloudOri.reset(new pcl::PointCloud<PointType>());
                coeffSel.reset(new pcl::PointCloud<PointType>());

                laserCloudOriCornerVec.resize(TOTAL_SCAN_NUM * Horizon_SCAN);
                coeffSelCornerVec.resize(TOTAL_SCAN_NUM * Horizon_SCAN);
                laserCloudOriCornerFlag.resize(TOTAL_SCAN_NUM * Horizon_SCAN);
                laserCloudOriSurfVec.resize(TOTAL_SCAN_NUM * Horizon_SCAN);
                coeffSelSurfVec.resize(TOTAL_SCAN_NUM * Horizon_SCAN);
                laserCloudOriSurfFlag.resize(TOTAL_SCAN_NUM * Horizon_SCAN);

                std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
                std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

                laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
                laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
                laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
                laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

                kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
                kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

                transformTobeMapped.resize(6,0);
                T.resize(6,0);

                matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

                pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 1);
                pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry", 1);
                pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1);
                pubPath = nh.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);

                if (lidarFrame != baselinkFrame)
                {
                    try
                    {
                        tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                        tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
                    }
                    catch (tf::TransformException const& ex)
                    {
                        ROS_ERROR("%s", ex.what());
                    }
                }


            }

            bool Scan2Map::run()
            {
                extractSurroundingKeyFrames();
                downsampleCurrentScan();
                scan2MapOptimization();
                return true;
            }


            void Scan2Map::setDeltaTInit(const gtsam::Pose3 &deltaT)
            {
                deltaTInit.resize(6,0);
                deltaTInit[0] = deltaT.rotation().roll();
                deltaTInit[1] = deltaT.rotation().pitch();
                deltaTInit[2] = deltaT.rotation().yaw();

                deltaTInit[3] = deltaT.translation().x();
                deltaTInit[4] = deltaT.translation().y();
                deltaTInit[5] = deltaT.translation().z();
            }

            void Scan2Map::publishGlobalMap()
            {
                if (pubLaserCloudSurround.getNumSubscribers() == 0)
                    return;

                if (cloudKeyPoses3D->points.empty() == true)
                    return;

                pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
                ;
                pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

                // kd-tree to find near key frames to visualize
                std::vector<int> pointSearchIndGlobalMap;
                std::vector<float> pointSearchSqDisGlobalMap;
                // search near key frames to visualize
                mtx.lock();
                kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
                kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
                mtx.unlock();

                for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
                    globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
                // downsample near selected key frames
                pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;                                                                                            // for global map visualization
                downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
                downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
                downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
                for(auto& pt : globalMapKeyPosesDS->points)
                {
                    pointSearchIndGlobalMap.resize(1);
                    pointSearchSqDisGlobalMap.resize(1);
                    kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
                    pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
                }

                // extract visualized and downsampled key frames
                for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
                {
                    if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                        continue;
                    int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
                    *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
                    *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
                }
                // downsample visualized points
                pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;                                                                                   // for global map visualization
                downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
                downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
                downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
                publishCloud(&pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
            }
            void Scan2Map::publishOdometry()
            {
                if(cloudKeyPoses3D->empty())
                {
                    return;
                }
                // Publish odometry for ROS (global)

                nav_msgs::Odometry laserOdometryROS;
                ros::Time timeLaserInfoStamp = ros::Time::now();
                laserOdometryROS.header.stamp = timeLaserInfoStamp;
                laserOdometryROS.header.frame_id = odometryFrame;
                laserOdometryROS.child_frame_id = "odom_mapping";
                laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
                laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
                laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
                laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
                pubLaserOdometryGlobal.publish(laserOdometryROS);

                // Publish TF
                static tf::TransformBroadcaster br;
                tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                        tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
                tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, "lidar_link");
                br.sendTransform(trans_odom_to_lidar);

                int frame_id = cloudKeyPoses6D->size() - 1;

                static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
                laserOdomIncremental.header.stamp = timeLaserInfoStamp;
                laserOdomIncremental.header.frame_id = odometryFrame;
                laserOdomIncremental.child_frame_id = "odom_mapping";
                laserOdomIncremental.pose.pose.position.x = cloudKeyPoses6D->points[frame_id].x;
                laserOdomIncremental.pose.pose.position.y = cloudKeyPoses6D->points[frame_id].y;
                laserOdomIncremental.pose.pose.position.z = cloudKeyPoses6D->points[frame_id].z;
                laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(cloudKeyPoses6D->points[frame_id].roll, cloudKeyPoses6D->points[frame_id].pitch, cloudKeyPoses6D->points[frame_id].yaw);
                if (isDegenerate)
                    laserOdomIncremental.pose.covariance[0] = 1;
                else
                    laserOdomIncremental.pose.covariance[0] = 0;
                pubLaserOdometryIncremental.publish(laserOdomIncremental);

                static tf::TransformBroadcaster tfOdom2BaseLink;
                tf::Transform tCur;
                tf::poseMsgToTF(laserOdomIncremental.pose.pose, tCur);
                if (lidarFrame != baselinkFrame)
                    tCur = tCur * lidar2Baselink;
                tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, timeLaserInfoStamp, odometryFrame, baselinkFrame);
                tfOdom2BaseLink.sendTransform(odom_2_baselink);

                if(pubPath.getNumSubscribers() != 0)
                {
                    globalPath.header.stamp = timeLaserInfoStamp;
                    globalPath.header.frame_id = odometryFrame;
                    pubPath.publish(globalPath);
                }
            }

            void Scan2Map::correctPoses(const gtsam::Values &isamCurrentEstimate)
            {
                std::lock_guard<std::mutex> lock(mtx);
                laserCloudMapContainer.clear();
                globalPath.poses.clear();
                int numPoses = isamCurrentEstimate.size();
                for (int i = 0; i < numPoses; ++i)
                {
                    cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
                    cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
                    cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

                    cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                    cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                    cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                    cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
                    cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
                    cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();

                    updatePath(cloudKeyPoses6D->points[i]);
                }
            }

            void Scan2Map::updatePath(const PointTypePose &pose_in)
            {
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
                pose_stamped.header.frame_id = odometryFrame;
                pose_stamped.pose.position.x = pose_in.x;
                pose_stamped.pose.position.y = pose_in.y;
                pose_stamped.pose.position.z = pose_in.z;
                tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
                pose_stamped.pose.orientation.x = q.x();
                pose_stamped.pose.orientation.y = q.y();
                pose_stamped.pose.orientation.z = q.z();
                pose_stamped.pose.orientation.w = q.w();
            
                globalPath.poses.push_back(pose_stamped);
            }

            void Scan2Map::updateMap()
            {

                std::lock_guard<std::mutex> lock(mtx);
                pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
                pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
                pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

                // save key frame cloud
                cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
                surfCloudKeyFrames.push_back(thisSurfKeyFrame);
            }

            void Scan2Map::updatePose(const gtsam::Pose3 &pose)
            {

                std::lock_guard<std::mutex> lock(mtx);
                PointType thisPose3D;

                thisPose3D.x = pose.translation().x();
                thisPose3D.y = pose.translation().y();
                thisPose3D.z = pose.translation().z();

                thisPose3D.intensity = cloudKeyPoses3D->size();
                cloudKeyPoses3D->push_back(thisPose3D);

                PointTypePose thisPose6D;

                thisPose6D.x = thisPose3D.x;
                thisPose6D.y = thisPose3D.y;
                thisPose6D.z = thisPose3D.z;

                thisPose6D.intensity = thisPose3D.intensity;
                thisPose6D.roll = pose.rotation().roll();
                thisPose6D.pitch = pose.rotation().pitch();
                thisPose6D.yaw = pose.rotation().yaw();
                thisPose6D.time = timeLaserInfoCur;

                cloudKeyPoses6D->push_back(thisPose6D);
                updatePath(thisPose6D);

            }

            void Scan2Map::updatePrePose(const gtsam::Pose3 &pose)
            {
                transformTobeMapped[0] = pose.rotation().roll();
                transformTobeMapped[1] = pose.rotation().pitch();
                transformTobeMapped[2] = pose.rotation().yaw();
                transformTobeMapped[3] = pose.translation().x();
                transformTobeMapped[4] = pose.translation().y();
                transformTobeMapped[5] = pose.translation().z();
            }

            pcl::PointCloud<PointType>::Ptr Scan2Map::transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
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
            }
            void Scan2Map::pointAssociateToMap(PointType const *const pi, PointType *const po)
            {
                po->x = transPointAssociateToMap(0, 0) * pi->x + transPointAssociateToMap(0, 1) * pi->y + transPointAssociateToMap(0, 2) * pi->z + transPointAssociateToMap(0, 3);
                po->y = transPointAssociateToMap(1, 0) * pi->x + transPointAssociateToMap(1, 1) * pi->y + transPointAssociateToMap(1, 2) * pi->z + transPointAssociateToMap(1, 3);
                po->z = transPointAssociateToMap(2, 0) * pi->x + transPointAssociateToMap(2, 1) * pi->y + transPointAssociateToMap(2, 2) * pi->z + transPointAssociateToMap(2, 3);
                po->intensity = pi->intensity;
            }

            void Scan2Map::extractNearby()
            {
                pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                // extract all the nearby key poses and downsample them
                kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
                kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
                for (int i = 0; i < (int)pointSearchInd.size(); ++i)
                {
                    int id = pointSearchInd[i];
                    surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
                }

                downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
                downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
                for(auto& pt : surroundingKeyPosesDS->points)
                {
                    pointSearchInd.resize(1);
                    pointSearchSqDis.resize(1);
                    kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
                    pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
                }

                // also extract some latest key frames in case the robot rotates in one position
                int numPoses = cloudKeyPoses3D->size();
                for (int i = numPoses - 1; i >= 0; --i)
                {
                    if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                        surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
                    else
                        break;
                }

                extractCloud(surroundingKeyPosesDS);
            }

            void Scan2Map::extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
            {
                // fuse the map
                laserCloudCornerFromMap->clear();
                laserCloudSurfFromMap->clear();
                for (int i = 0; i < (int)cloudToExtract->size(); ++i)
                {
                    if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                        continue;

                    int thisKeyInd = (int)cloudToExtract->points[i].intensity;
                    if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end())
                    {
                        // transformed cloud available
                        *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                        *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
                    }
                    else
                    {
                        // transformed cloud not available
                        pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
                        pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
                        *laserCloudCornerFromMap += laserCloudCornerTemp;
                        *laserCloudSurfFromMap += laserCloudSurfTemp;
                        laserCloudMapContainer[thisKeyInd] = std::make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
                    }
                }

                // Downsample the surrounding corner key frames (or map)
                downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
                downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
                laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
                // Downsample the surrounding surf key frames (or map)
                downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
                downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
                laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

                // clear map cache if too large
                if (laserCloudMapContainer.size() > 1000)
                    laserCloudMapContainer.clear();
            }

            void Scan2Map::extractSurroundingKeyFrames()
            {
                if (cloudKeyPoses3D->points.empty() == true)
                    return;

                // if (loopClosureEnableFlag == true)
                // {
                //     extractForLoopClosure();
                // } else {
                //     extractNearby();
                // }

                extractNearby();
            }

            void Scan2Map::downsampleCurrentScan()
            {
                // 
                // Downsample cloud from current scan
                laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

                laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
            }

            void Scan2Map::updatePointAssociateToMap()
            {
                transPointAssociateToMap = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
            }

            void Scan2Map::cornerOptimization()
            {
                updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
                for (int i = 0; i < laserCloudCornerLastDSNum; i++)
                {
                    PointType pointOri, pointSel, coeff;
                    std::vector<int> pointSearchInd;
                    std::vector<float> pointSearchSqDis;

                    pointOri = laserCloudCornerLastDS->points[i];
                    pointAssociateToMap(&pointOri, &pointSel);
                    kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                    cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
                    cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
                    cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

                    if (pointSearchSqDis[4] < 1.0)
                    {
                        float cx = 0, cy = 0, cz = 0;
                        for (int j = 0; j < 5; j++)
                        {
                            cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                            cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                            cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                        }
                        cx /= 5;
                        cy /= 5;
                        cz /= 5;

                        float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                        for (int j = 0; j < 5; j++)
                        {
                            float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                            float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                            float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                            a11 += ax * ax;
                            a12 += ax * ay;
                            a13 += ax * az;
                            a22 += ay * ay;
                            a23 += ay * az;
                            a33 += az * az;
                        }
                        a11 /= 5;
                        a12 /= 5;
                        a13 /= 5;
                        a22 /= 5;
                        a23 /= 5;
                        a33 /= 5;

                        matA1.at<float>(0, 0) = a11;
                        matA1.at<float>(0, 1) = a12;
                        matA1.at<float>(0, 2) = a13;
                        matA1.at<float>(1, 0) = a12;
                        matA1.at<float>(1, 1) = a22;
                        matA1.at<float>(1, 2) = a23;
                        matA1.at<float>(2, 0) = a13;
                        matA1.at<float>(2, 1) = a23;
                        matA1.at<float>(2, 2) = a33;

                        cv::eigen(matA1, matD1, matV1);

                        if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
                        {

                            float x0 = pointSel.x;
                            float y0 = pointSel.y;
                            float z0 = pointSel.z;
                            float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                            float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                            float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                            float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                            float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                            float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                            float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

                            float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

                            float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

                            float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                            float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                            float ld2 = a012 / l12;

                            float s = 1 - 0.9 * fabs(ld2);

                            coeff.x = s * la;
                            coeff.y = s * lb;
                            coeff.z = s * lc;
                            coeff.intensity = s * ld2;

                            if (s > 0.1)
                            {
                                laserCloudOriCornerVec[i] = pointOri;
                                coeffSelCornerVec[i] = coeff;
                                laserCloudOriCornerFlag[i] = true;
                            }
                        }
                    }
                }
            }

            void Scan2Map::surfOptimization()
            {
                updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
                for (int i = 0; i < laserCloudSurfLastDSNum; i++)
                {
                    PointType pointOri, pointSel, coeff;
                    std::vector<int> pointSearchInd;
                    std::vector<float> pointSearchSqDis;

                    pointOri = laserCloudSurfLastDS->points[i];
                    pointAssociateToMap(&pointOri, &pointSel);
                    kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                    Eigen::Matrix<float, 5, 3> matA0;
                    Eigen::Matrix<float, 5, 1> matB0;
                    Eigen::Vector3f matX0;

                    matA0.setZero();
                    matB0.fill(-1);
                    matX0.setZero();

                    if (pointSearchSqDis[4] < 1.0)
                    {
                        for (int j = 0; j < 5; j++)
                        {
                            matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                            matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                            matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                        }

                        matX0 = matA0.colPivHouseholderQr().solve(matB0);

                        float pa = matX0(0, 0);
                        float pb = matX0(1, 0);
                        float pc = matX0(2, 0);
                        float pd = 1;

                        float ps = sqrt(pa * pa + pb * pb + pc * pc);
                        pa /= ps;
                        pb /= ps;
                        pc /= ps;
                        pd /= ps;

                        bool planeValid = true;
                        for (int j = 0; j < 5; j++)
                        {
                            if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                                        pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                                        pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2)
                            {
                                planeValid = false;
                                break;
                            }
                        }

                        if (planeValid)
                        {
                            float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                            float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                            coeff.x = s * pa;
                            coeff.y = s * pb;
                            coeff.z = s * pc;
                            coeff.intensity = s * pd2;

                            if (s > 0.1)
                            {
                                laserCloudOriSurfVec[i] = pointOri;
                                coeffSelSurfVec[i] = coeff;
                                laserCloudOriSurfFlag[i] = true;
                            }
                        }
                    }
                }
            }

            void Scan2Map::combineOptimizationCoeffs()
            {
                // combine corner coeffs
                for (int i = 0; i < laserCloudCornerLastDSNum; ++i)
                {
                    if (laserCloudOriCornerFlag[i] == true)
                    {
                        laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                        coeffSel->push_back(coeffSelCornerVec[i]);
                    }
                }
                // combine surf coeffs
                for (int i = 0; i < laserCloudSurfLastDSNum; ++i)
                {
                    if (laserCloudOriSurfFlag[i] == true)
                    {
                        laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                        coeffSel->push_back(coeffSelSurfVec[i]);
                    }
                }
                // reset flag for next iteration
                std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
                std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
            }

            bool Scan2Map::LMOptimization(int iterCount)
            {
                // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
                // lidar <- camera      ---     camera <- lidar
                // x = z                ---     x = y
                // y = x                ---     y = z
                // z = y                ---     z = x
                // roll = yaw           ---     roll = pitch
                // pitch = roll         ---     pitch = yaw
                // yaw = pitch          ---     yaw = roll

                // lidar -> camera
                float srx = sin(transformTobeMapped[1]);
                float crx = cos(transformTobeMapped[1]);
                float sry = sin(transformTobeMapped[2]);
                float cry = cos(transformTobeMapped[2]);
                float srz = sin(transformTobeMapped[0]);
                float crz = cos(transformTobeMapped[0]);

                int laserCloudSelNum = laserCloudOri->size();
                if (laserCloudSelNum < 50)
                {
                    return false;
                }


                cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
                cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
                cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
                cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
                cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
                cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

                if(!deltaTInit.empty())
                {
                    matA = cv::Mat(laserCloudSelNum + 6,6,CV_32F, cv::Scalar::all(0));
                    matAt = cv::Mat(6, laserCloudSelNum + 6,CV_32F, cv::Scalar::all(0));
                    matB = cv::Mat(laserCloudSelNum + 6,1,CV_32F, cv::Scalar::all(0));
                }

                PointType pointOri, coeff;

                for (int i = 0; i < laserCloudSelNum; i++)
                {
                    // lidar -> camera
                    pointOri.x = laserCloudOri->points[i].y;
                    pointOri.y = laserCloudOri->points[i].z;
                    pointOri.z = laserCloudOri->points[i].x;
                    // lidar -> camera
                    coeff.x = coeffSel->points[i].y;
                    coeff.y = coeffSel->points[i].z;
                    coeff.z = coeffSel->points[i].x;
                    coeff.intensity = coeffSel->points[i].intensity;
                    // in camera
                    float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

                    float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

                    float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;
                    // lidar -> camera
                    matA.at<float>(i, 0) = arz;
                    matA.at<float>(i, 1) = arx;
                    matA.at<float>(i, 2) = ary;
                    matA.at<float>(i, 3) = coeff.z;
                    matA.at<float>(i, 4) = coeff.x;
                    matA.at<float>(i, 5) = coeff.y;
                    matB.at<float>(i, 0) = -coeff.intensity;
                }

                if(!deltaTInit.empty())
                {
                    float weight = 1e-3;
                    for(int i = 0;i < 6; ++i)
                    {
                        matA.at<float>(i + laserCloudSelNum,i) = 2 * weight * (T[i] - deltaTInit[i]);
                        matB.at<float>(i + laserCloudSelNum,0) = -weight * (T[i] - deltaTInit[i]) * (T[i] - deltaTInit[i]);
                    }
                }

                cv::transpose(matA, matAt);
                matAtA = matAt * matA;
                matAtB = matAt * matB;
                cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

                if (iterCount == 0)
                {

                    cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
                    cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
                    cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

                    cv::eigen(matAtA, matE, matV);
                    matV.copyTo(matV2);

                    isDegenerate = false;
                    float eignThre[6] = {100, 100, 100, 100, 100, 100};
                    for (int i = 5; i >= 0; i--)
                    {
                        if (matE.at<float>(0, i) < eignThre[i])
                        {
                            for (int j = 0; j < 6; j++)
                            {
                                matV2.at<float>(i, j) = 0;
                            }
                            isDegenerate = true;
                        }
                        else
                        {
                            break;
                        }
                    }
                    matP = matV.inv() * matV2;
                }

                if (isDegenerate)
                {
                    cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
                    matX.copyTo(matX2);
                    matX = matP * matX2;
                }

                transformTobeMapped[0] += matX.at<float>(0, 0);
                transformTobeMapped[1] += matX.at<float>(1, 0);
                transformTobeMapped[2] += matX.at<float>(2, 0);
                transformTobeMapped[3] += matX.at<float>(3, 0);
                transformTobeMapped[4] += matX.at<float>(4, 0);
                transformTobeMapped[5] += matX.at<float>(5, 0);

                T[0] += matX.at<float>(0, 0);
                T[1] += matX.at<float>(1, 0);
                T[2] += matX.at<float>(2, 0);
                T[3] += matX.at<float>(3, 0);
                T[4] += matX.at<float>(4, 0);
                T[5] += matX.at<float>(5, 0);


                float deltaR = sqrt(
                        pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
                float deltaT = sqrt(
                        pow(matX.at<float>(3, 0) * 100, 2) +
                        pow(matX.at<float>(4, 0) * 100, 2) +
                        pow(matX.at<float>(5, 0) * 100, 2));

                if (deltaR < 0.05 && deltaT < 0.05)
                {
                    if(isDegenerate)
                        std::cout << "isDegenerate: " << isDegenerate << std::endl;
                    return true; // converged
                }
                return false; // keep optimizing
            }

            void Scan2Map::scan2MapOptimization()
            {
                if (cloudKeyPoses3D->points.empty())
                    return;

                if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
                {
                    kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
                    kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

                    for (int iterCount = 0; iterCount < 30; iterCount++)
                    {
                        laserCloudOri->clear();
                        coeffSel->clear();

                        cornerOptimization();
                        surfOptimization();

                        combineOptimizationCoeffs();

                        if (LMOptimization(iterCount) == true)
                            break;
                    }

                    transformUpdate();
                }
                else
                {
                    ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
                }
            }

            void Scan2Map::transformUpdate()
            {

                transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
                transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
                transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

            }

            float Scan2Map::constraintTransformation(float value, float limit)
            {
                if (value < -limit)
                    value = -limit;
                if (value > limit)
                    value = limit;

                return value;
            }

        }
    }
}
