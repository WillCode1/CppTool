/*
 * @Description: lidar mapping
 * @Author: fei
 * @Data: 2021-09-23
 */

#include "../../include/localization/mapping/lidar_mapping.hpp"

namespace localization_node
{
    LidarMapping::LidarMapping(ros::NodeHandle &nh, std::string topic_name, size_t buffer_size)
        : nh_(nh)
    {
        lidar_cloud_subscriber_ = nh_.subscribe(topic_name, buffer_size, &LidarMapping::cloudHandler, this);
        map_cloud_ptr_ = boost::shared_ptr<PointCloudXYZI>(new PointCloudXYZI());
        lidar_imu_tran_ = Eigen::Isometry3f::Identity();
        init_calibrate_ = false;
        std::cout << "Lidar Mapping Constructed()" << std::endl;
    }

    LidarMapping::~LidarMapping()
    {
        std::cout << "Lidar Mapping Deconstructed()" << std::endl;
    }

    void LidarMapping::cloudHandler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr)
    {
        CloudData cloud_data;
        cloud_data.timestamp = cloud_msg_ptr->header.stamp.toSec();
        cloud_data.sweepCloudPtr = boost::shared_ptr<PointCloudXYZI>(new PointCloudXYZI());
        pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.sweepCloudPtr));
        cloud_data_deque_.push_back(cloud_data);
        std::cout << "current cloud msg size: " << cloud_data_deque_.size() << std::endl;
    }

    std::vector<string> LidarMapping::splitLineData(const std::string &str, const std::string &delim)
    {
        std::vector<string> res;
        if (str.empty() || delim.empty())
        {
            return res;
        }
        char *strs = new char[str.length() + 1];
        strcpy(strs, str.c_str());
        char *d = new char[delim.length() + 1];
        strcpy(d, delim.c_str());

        char *p = strtok(strs, d);
        while (p != NULL)
        {
            string s = p;
            res.push_back(s);
            p = strtok(NULL, d);
        }

        if (strs != NULL)
        {
            delete[] strs;
            strs = NULL;
        }

        if (d != NULL)
        {
            delete[] d;
            d = NULL;
        }

        return res;
    }

    PointCloudXYZIPtr LidarMapping::matchingMap(PointCloudXYZIPtr sweepCloud, OdomPoint curOdom)
    {
        PointCloudXYZIPtr transformCloud(new PointCloudXYZI());
        pcl::transformPointCloud(*sweepCloud, *transformCloud, lidar_imu_tran_);

        transformCloud->points.erase(
            std::remove_if(transformCloud->points.begin(), transformCloud->points.end(),
                           [](pointXYZI &p)
                           {
                               return sqrt(p.x * p.x + p.y * p.y) < 1.0 || fabs(p.x) > 12.0 || fabs(p.y) > 12.0;
                           }),
            transformCloud->points.end());

        double roll, pitch, yaw;
        tf::Matrix3x3(tf::Quaternion(curOdom.q_x, curOdom.q_y, curOdom.q_z, curOdom.q_w)).getRPY(roll, pitch, yaw);
        // Eigen::Quaterniond quaternion(curOdom.q_w, curOdom.q_x, curOdom.q_y, curOdom.q_z);
        // Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2,1,0);

        Eigen::Vector3f translation(curOdom.x, curOdom.y, curOdom.z);

        Eigen::Isometry3f tran = Eigen::Isometry3f::Identity();
        tran.translate(translation);
        tran.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                    Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                    Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

        pcl::transformPointCloud(*transformCloud, *transformCloud, tran);

        // voxel filter
        pcl::VoxelGrid<pointXYZI> sor;
        sor.setInputCloud(transformCloud);
        sor.setLeafSize(0.5, 0.5, 0.5);
        sor.setMinimumPointsNumberPerVoxel(1);
        sor.filter(*transformCloud);

        return transformCloud;
    }

    bool LidarMapping::LookupData()
    {
        try
        {
            tf::StampedTransform transform;
            tf_listener_.lookupTransform("velo_link", "imu_link", ros::Time(0), transform);
            Eigen::Vector3f tf_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
            double roll, pitch, yaw;
            tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);

            lidar_imu_tran_.translate(tf_btol);
            lidar_imu_tran_.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                                   Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
                                   Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
            std::cout << "lidar_imu_tran_: " << lidar_imu_tran_.matrix() << std::endl;
            return true;
        }
        catch (tf::TransformException &ex)
        {
            return false;
        }
    }

    void LidarMapping::setOdometry(string input_file)
    {
        ifstream inFile(input_file);
        if (!inFile.is_open())
        {
            std::cout << "input_file: " << input_file << " open errored" << std::endl;
            return;
        }

        const std::string seprator(" ");
        while (inFile.peek() != EOF)
        {
            std::string currentLine;
            getline(inFile, currentLine);
            vector<string> data = splitLineData(currentLine.c_str(), seprator);
            OdomPoint op;
            for (size_t idx = 0; idx < data.size(); idx++)
            {
                switch (idx)
                {
                case 0:
                    char *endT;
                    op.timestamp = std::strtod(data[0].c_str(), &endT);
                    break;
                case 1:
                    char *endX;
                    op.x = std::strtod(data[1].c_str(), &endX);
                    break;
                case 2:
                    char *endY;
                    op.y = std::strtod(data[2].c_str(), &endY);
                    break;
                case 3:
                    char *endZ;
                    op.z = std::strtod(data[3].c_str(), &endZ);
                    break;
                case 4:
                    char *endQx;
                    op.q_x = std::strtod(data[4].c_str(), &endQx);
                    break;
                case 5:
                    char *endQy;
                    op.q_y = std::strtod(data[5].c_str(), &endQy);
                    break;
                case 6:
                    char *endQz;
                    op.q_z = std::strtod(data[6].c_str(), &endQz);
                    break;
                case 7:
                    char *endQw;
                    op.q_w = std::strtod(data[7].c_str(), &endQw);
                    break;
                default:
                    break;
                }
            }
            lidar_odom_deque_.push_back(op);
        }
    }

    void LidarMapping::runMapping()
    {
        if (cloud_data_deque_.size() > 0)
        {
            if (!init_calibrate_)
            {
                if (LookupData())
                {
                    init_calibrate_ = true;
                }
                else
                {
                    std::cout << "Wait for init calibrating" << std::endl;
                    return;
                }
            }
            CloudData cloud_data = cloud_data_deque_.front();
            double timestamp = cloud_data.timestamp;
            bool isFind = false;
            OdomPoint targetOdom;
            for (auto iter = lidar_odom_deque_.begin(); iter != lidar_odom_deque_.end(); ++iter)
            {
                OdomPoint op = (OdomPoint)(*iter);
                double odom_time = op.timestamp;
                if (fabs(timestamp - odom_time) < 0.05)
                {
                    isFind = true;
                    targetOdom = op;
                    std::cout << to_string(timestamp) << " " << to_string(odom_time) << std::endl;
                    break;
                }
            }

            if (isFind)
            {
                PointCloudXYZIPtr segmentCloud = matchingMap(cloud_data.sweepCloudPtr, targetOdom);
                *map_cloud_ptr_ += *segmentCloud;
                // pcl::copyPointCloud(*segmentCloud, *map_cloud_ptr_);

                std::cout << "current map cloud size: " << map_cloud_ptr_->points.size() << std::endl;
                std::cout << "current lidar odometry size: " << lidar_odom_deque_.size() << std::endl;
                std::cout << "current cloud msgs size: " << cloud_data_deque_.size() << std::endl;
            }
            cloud_data_deque_.pop_front();
        }
    }

    PointCloudXYZIPtr LidarMapping::getMapCloud()
    {
        return map_cloud_ptr_;
    }
}
