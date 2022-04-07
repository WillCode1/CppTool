#include "utility.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

// gtsam
NonlinearFactorGraph gtSAMgraph;
Values initialEstimate;
Values optimizedEstimate;
ISAM2 *isam;
Eigen::MatrixXd poseCovariance;

std::deque<OdometryData> gpsQueue;
pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

double timeLaserInfoCur;
float transformTobeMapped[6];   // cur pose

bool aLoopIsClosed = false;
vector<pair<int, int>> loopIndexQueue;
vector<gtsam::Pose3> loopPoseQueue;
vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;

void init()
{
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
}

gtsam::Pose3 pclPointTogtsamPose3(const PointTypePose& thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

/**
 * 添加激光里程计因子
 */
void addOdomFactor()
{
    gtsam::Pose3 curPose(gtsam::Rot3::RzRyRx(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                         gtsam::Point3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));

    if (cloudKeyPoses6D->points.empty())
    {
        // 第一帧初始化先验因子
        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
        gtSAMgraph.add(PriorFactor<Pose3>(0, curPose, priorNoise));
        // 变量节点设置初始值
        initialEstimate.insert(0, curPose);
    }
    else
    {
        // 添加激光里程计因子
        noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
        gtsam::Pose3 poseTo = curPose;
        // 参数：前一帧id，当前帧id，前一帧与当前帧的位姿变换（作为观测值），噪声协方差
        gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses6D->size() - 1, cloudKeyPoses6D->size(), poseFrom.between(poseTo), odometryNoise));
        // 变量节点设置初始值
        initialEstimate.insert(cloudKeyPoses6D->size(), poseTo);
    }
}

/**
 * 添加GPS因子
 */
void addGPSFactor()
{
    if (gpsQueue.empty())
        return;

    // wait for system initialized and settles down
    // 如果没有关键帧，或者首尾关键帧距离小于5m，不添加gps因子
    if (cloudKeyPoses6D->points.empty())
        return;

    while (!gpsQueue.empty())
    {
        // 删除当前帧0.2s之前的里程计
        if (gpsQueue.front().stamp < timeLaserInfoCur - 0.2)
        {
            // message too old
            gpsQueue.pop_front();
        }
        // 超过当前帧0.2s之后，退出
        else if (gpsQueue.front().stamp > timeLaserInfoCur + 0.2)
        {
            // message too new
            break;
        }
        else
        {
            OdometryData thisGPS = gpsQueue.front();
            gpsQueue.pop_front();

            float noise_x;
            float noise_y;
            float noise_z;

            // GPS噪声协方差太大，不能用
            // float noise_x = thisGPS.pose.covariance[0];
            // float noise_y = thisGPS.pose.covariance[7];
            // float noise_z = thisGPS.pose.covariance[14];

            // GPS里程计位置
            float gps_x = thisGPS.pose.position.x();
            float gps_y = thisGPS.pose.position.y();
            float gps_z = thisGPS.pose.position.z();

            // Add GPS every a few meters
            // 每隔5m添加一个GPS里程计

            // 添加GPS因子
            gtsam::Vector Vector3(3);
            Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
            noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(cloudKeyPoses6D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
            gtSAMgraph.add(gps_factor);

            aLoopIsClosed = true;
            break;
        }
    }
}

/**
 * 添加闭环因子
 */
void addLoopFactor()
{
    if (loopIndexQueue.empty())
        return;

    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        // 闭环边对应两帧的索引
        int indexFrom = loopIndexQueue[i].first;
        int indexTo = loopIndexQueue[i].second;
        // 闭环边的位姿变换
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    aLoopIsClosed = true;
}

/**
 * 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹
 */
void correctPoses()
{
    if (cloudKeyPoses6D->points.empty())
        return;

    if (aLoopIsClosed)
    {
        // clear path
        // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿
        int numPoses = optimizedEstimate.size();
        for (int i = 0; i < numPoses; ++i)
        {
            cloudKeyPoses6D->points[i].x = optimizedEstimate.at<Pose3>(i).translation().x();
            cloudKeyPoses6D->points[i].y = optimizedEstimate.at<Pose3>(i).translation().y();
            cloudKeyPoses6D->points[i].z = optimizedEstimate.at<Pose3>(i).translation().z();
            cloudKeyPoses6D->points[i].roll = optimizedEstimate.at<Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = optimizedEstimate.at<Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw = optimizedEstimate.at<Pose3>(i).rotation().yaw();
        }

        aLoopIsClosed = false;
    }
}

void saveKeyFramesAndFactor()
{
    // odom factor
    addOdomFactor();

    // gps factor
    addGPSFactor();

    // loop factor
    addLoopFactor();

    // update iSAM
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    if (aLoopIsClosed)
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }
    // update之后要清空一下保存的因子图，注：历史数据不会清掉，ISAM保存起来了
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    // save key poses
    Pose3 latestEstimate;

    // 优化结果
    optimizedEstimate = isam->calculateEstimate();
    // 当前帧位姿结果
    latestEstimate = optimizedEstimate.at<Pose3>(optimizedEstimate.size() - 1);
    // 位姿协方差
    poseCovariance = isam->marginalCovariance(optimizedEstimate.size() - 1);

    // transformTobeMapped更新当前帧位姿
    transformTobeMapped[0] = latestEstimate.rotation().roll();
    transformTobeMapped[1] = latestEstimate.rotation().pitch();
    transformTobeMapped[2] = latestEstimate.rotation().yaw();
    transformTobeMapped[3] = latestEstimate.translation().x();
    transformTobeMapped[4] = latestEstimate.translation().y();
    transformTobeMapped[5] = latestEstimate.translation().z();
}

int main()
{
    init();

    saveKeyFramesAndFactor();

    correctPoses();

    return 0;
}
