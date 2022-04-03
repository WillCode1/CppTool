#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "../include/localization/matching/lidar_matching.hpp"

using namespace std;
using namespace localization_node;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber_node");
    ros::NodeHandle matching_node("~");

    string odometryFilePath, globalMapFile;

    if (matching_node.getParam("lidar_odometry_file", odometryFilePath) &&
        matching_node.getParam("global_map_file", globalMapFile))
    {
        std::cout << "parameter init successful" << std::endl;
    }

    LidarMatching lidarMatching(matching_node, "/kitti/velo/pointcloud", 1000000);
    lidarMatching.setOdometry(odometryFilePath);

    std::cout << "start load global map cloud" << std::endl;
    lidarMatching.InitGlobalMap(globalMapFile);
    std::cout << "load gloab map cloud finished" << std::endl;

    // NDT Parameter
    float res = 1.0;
    float step_size = 0.1;
    float trans_eps = 0.01;
    int max_iter = 30;

    lidarMatching.InitNDTRegistration(res, step_size, trans_eps, max_iter);

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();
        lidarMatching.runMatching();
        rate.sleep();
    }

    std::deque<OdomPoint> locate_pose = lidarMatching.getLocatePose();
    std::cout << "final locate_pose size: " << locate_pose.size() << std::endl;

    string outFileName = "/home/luffy/work/data/KITTI_VELODYNE/final_class/locate_pose.txt";
    FILE *fp;
    fp = fopen(outFileName.c_str(), "wb");
    while (locate_pose.size() > 0)
    {
        OdomPoint pose_odom = locate_pose.front();
        double timestamp = pose_odom.timestamp;
        double x = pose_odom.x;
        double y = pose_odom.y;
        double z = pose_odom.z;
        fprintf(fp, "%f %f %f %f\n", timestamp, x, y, z);
        fflush(fp);
        locate_pose.pop_front();
    }
    fclose(fp);

    return 0;
}
