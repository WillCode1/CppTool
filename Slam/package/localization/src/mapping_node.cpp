#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>

#include "../include/localization/mapping/lidar_mapping.hpp"

using namespace std;
using namespace localization_node;

namespace bpo = boost::program_options;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber_node");
    ros::NodeHandle mapping_node("~");

    bpo::options_description opts("all options");
    bpo::variables_map vm;

    opts.add_options()("lidar_odometry_file", bpo::value<string>(), "the lidar odometry file path");
    opts.add_options()("result_map_file", bpo::value<string>(), "the result map file path");

    bpo::store(bpo::parse_command_line(argc, argv, opts), vm);

    string odometryFilePath = vm["lidar_odometry_file"].as<string>();
    string resultMapFile = vm["result_map_file"].as<string>();

    LidarMapping lidarMapping(mapping_node, "/kitti/velo/pointcloud", 1000000);
    lidarMapping.setOdometry(odometryFilePath);

    ros::Rate rate(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        lidarMapping.runMapping();
        rate.sleep();
    }

    PointCloudXYZIPtr result_map = lidarMapping.getMapCloud();
    std::cout << "total map cloud size: " << result_map->size() << std::endl;

    pcl::io::savePCDFileBinary(resultMapFile, *result_map);
    std::cout << "write finished" << std::endl;

    return 0;
}