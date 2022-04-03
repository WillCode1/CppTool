/*
 * @Description: lidar matching
 * @Author: fei
 * @Data: 2021-09-26
 */

#include "../../include/localization/matching/lidar_matching.hpp"

namespace localization_node
{
    LidarMatching::LidarMatching(ros::NodeHandle &nh, std::string topic_name, size_t buffer_size)
        : nh_(nh), ndt_ptr_(new pcl::NormalDistributionsTransform<pointXYZI, pointXYZI>())
    {
        lidar_cloud_subscriber_ = nh_.subscribe(topic_name, buffer_size, &LidarMatching::msgCallBackHandler, this);
        global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_map", 100);
        local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_map", 100);
        current_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/current_scan", 100);
        laser_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/laser_localization", 100);

        transform_.frame_id_ = "/map";
        transform_.child_frame_id_ = "/vehicle_link";

        global_map_cloud_ = boost::shared_ptr<PointCloudXYZI>(new PointCloudXYZI());
        local_map_cloud_ = boost::shared_ptr<PointCloudXYZI>(new PointCloudXYZI());
        current_sweep_cloud_ = boost::shared_ptr<PointCloudXYZI>(new PointCloudXYZI());

        init_global_map_ = false;
        has_new_local_map_ = false;
        init_pose_finished_ = false;

        origin_ = vector<float>(3);
        edge_ = vector<float>(6);
        box_size_ = 150.0;

        std::cout << "Lidar Matching Constructed()" << std::endl;
    }

    LidarMatching::~LidarMatching()
    {
        std::cout << "Lidar Matching Deconstructed()" << std::endl;
    }

    void LidarMatching::msgCallBackHandler(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr)
    {
        CloudData cloud_data;
        cloud_data.timestamp = cloud_msg_ptr->header.stamp.toSec();
        cloud_data.sweepCloudPtr = boost::shared_ptr<PointCloudXYZI>(new PointCloudXYZI());
        pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.sweepCloudPtr));
        cloud_data_deque_.push_back(cloud_data);
        std::cout << "current cloud msg size: " << cloud_data_deque_.size() << std::endl;
        return;
    }

    std::vector<string> LidarMatching::splitLineData(const std::string &str, const std::string &delim)
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

    vector<float> LidarMatching::calculateEdge(vector<float> origin, float box_size)
    {
        vector<float> edge(6);
        for (size_t i = 0; i < origin.size(); ++i)
        {
            edge.at(2 * i) = origin.at(i) - box_size;
            edge.at(2 * i + 1) = origin.at(i) + box_size;
        }
        return edge;
    }

    vector<float> LidarMatching::getEdge()
    {
        return edge_;
    }

    PointCloudXYZIPtr LidarMatching::boxFilter(
        PointCloudXYZIPtr input_cloud,
        vector<float> origin,
        float box_size)
    {
        PointCloudXYZIPtr cropCloud(new PointCloudXYZI());
        pcl::CropBox<pointXYZI> cropBoxFilter;

        edge_ = calculateEdge(origin, box_size);
        cropBoxFilter.setMin(Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6));
        cropBoxFilter.setMax(Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6));
        cropBoxFilter.setInputCloud(input_cloud);
        cropBoxFilter.filter(*cropCloud);
        return cropCloud;
    }

    bool LidarMatching::setGNSSPose(const Eigen::Matrix4f &gnss_pose)
    {
        current_gnss_pose_ = gnss_pose;
        static int gnss_cnt = 0;
        if (gnss_cnt == 0)
        {
            setInitPose(gnss_pose);
        }
        else if (gnss_cnt > 3)
        {
            init_pose_finished_ = true;
        }
        gnss_cnt++;
        return true;
    }

    bool LidarMatching::setInitPose(const Eigen::Matrix4f &init_pose)
    {
        init_pose_ = init_pose;
        bool ret = ResetLocalMap(init_pose(0, 3), init_pose(1, 3), init_pose(2, 3), box_size_);
        if (!ret)
        {
            return false;
        }
        return true;
    }

    bool LidarMatching::UpatePose(const CloudData &cloud_data, Eigen::Matrix4f &cloud_pose)
    {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_data.sweepCloudPtr, *cloud_data.sweepCloudPtr, indices);

        PointCloudXYZIPtr filterCloud(new PointCloudXYZI());
        pcl::VoxelGrid<pointXYZI> sor;
        sor.setInputCloud(cloud_data.sweepCloudPtr);
        sor.setLeafSize(0.5, 0.5, 0.5);
        sor.setMinimumPointsNumberPerVoxel(1);
        sor.filter(*filterCloud);

        // update everytime
        static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
        static Eigen::Matrix4f last_pose = init_pose_;
        static Eigen::Matrix4f predict_pose = init_pose_;

        if (!init_pose_finished_)
        {
            predict_pose = current_gnss_pose_;
        }

        PointCloudXYZIPtr resultCloud(new PointCloudXYZI());
        ndt_ptr_->setInputSource(filterCloud);
        ndt_ptr_->align(*resultCloud, predict_pose);
        cloud_pose = ndt_ptr_->getFinalTransformation();
        pcl::transformPointCloud(*cloud_data.sweepCloudPtr, *current_sweep_cloud_, cloud_pose);

        step_pose = last_pose.inverse() * cloud_pose;
        predict_pose = cloud_pose * step_pose;
        last_pose = cloud_pose;

        std::vector<float> curEdge = getEdge();
        for (int i = 0; i < 3; i++)
        {
            if (fabs(cloud_pose(i, 3) - curEdge.at(2 * i)) > 50.0 &&
                fabs(cloud_pose(i, 3) - curEdge.at(2 * i + 1)) > 50.0)
            {
                continue;
            }
            bool ret = ResetLocalMap(cloud_pose(0, 3), cloud_pose(1, 3), cloud_pose(2, 3), box_size_);
            if (!ret)
            {
                return false;
            }
            break;
        }
        return true;
    }

    void LidarMatching::runMatching()
    {
        std::cout << "waiting data..." << std::endl;
        if (cloud_data_deque_.size() > 0 && init_global_map_)
        {
            CloudData cloud_data = cloud_data_deque_.front();
            if (!init_pose_finished_ || !has_new_local_map_)
            {
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
                    Eigen::Matrix4f init_pose;
                    Eigen::Quaterniond q(targetOdom.q_w, targetOdom.q_x, targetOdom.q_y, targetOdom.q_z);
                    Eigen::Matrix3f rotate_matrix = q.matrix().cast<float>();
                    init_pose(0, 3) = targetOdom.x;
                    init_pose(1, 3) = targetOdom.y;
                    init_pose(2, 3) = targetOdom.z;
                    init_pose.block<3, 3>(0, 0) = rotate_matrix;
                    if (setGNSSPose(init_pose))
                    {
                        std::cout << "Init pose finished" << std::endl;
                    }
                    else
                    {
                        cloud_data_deque_.pop_front();
                        return;
                    }
                }
                else
                {
                    cloud_data_deque_.pop_front();
                    return;
                }
            }

            PublishCloudData(global_map_cloud_, global_map_pub_, 0.3);
            PublishCloudData(local_map_cloud_, local_map_pub_, 0.3);

            Eigen::Matrix4f current_pose;
            if (UpatePose(cloud_data, current_pose))
            {
                OdomPoint pose_odom;
                pose_odom.timestamp = cloud_data.timestamp;
                pose_odom.x = current_pose(0, 3);
                pose_odom.y = current_pose(1, 3);
                pose_odom.z = current_pose(2, 3);
                locate_pose_deque_.push_back(pose_odom);
                std::cout << "locate_pose_deque size: " << locate_pose_deque_.size() << std::endl;
                PublishCloudData(current_sweep_cloud_, current_scan_pub_, 0.0);
                ros::Time rosTime = ros::Time().fromSec(cloud_data.timestamp);
                PublistOdomData(current_pose, rosTime);
            }
            else
            {
                std::cout << "locate failed" << std::endl;
            }
            cloud_data_deque_.pop_front();
        }
        return;
    }

    std::deque<OdomPoint> LidarMatching::getLocatePose()
    {
        return locate_pose_deque_;
    }

    void LidarMatching::setOdometry(string input_file)
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
        return;
    }

    void LidarMatching::InitGlobalMap(string cloud_file_name)
    {
        pcl::io::loadPCDFile(cloud_file_name, *global_map_cloud_);
        std::cout << "load global map size: " << global_map_cloud_->size() << std::endl;

        init_global_map_ = true;

        return;
    }

    void LidarMatching::InitNDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    {
        ndt_ptr_->setResolution(res);
        ndt_ptr_->setStepSize(step_size);
        ndt_ptr_->setTransformationEpsilon(trans_eps);
        ndt_ptr_->setMaximumIterations(max_iter);

        std::cout << "Init NDT Registrarion finished" << std::endl;

        return;
    }

    bool LidarMatching::ResetLocalMap(float x, float y, float z, float box_size)
    {
        origin_.at(0) = x;
        origin_.at(1) = y;
        origin_.at(2) = z;

        local_map_cloud_ = boxFilter(global_map_cloud_, origin_, box_size);
        std::cout << "local map cloud size: " << local_map_cloud_->size() << std::endl;
        if (local_map_cloud_->size() == 0)
        {
            std::cout << "local map cloud is empty" << std::endl;
            return false;
        }
        has_new_local_map_ = true;
        ndt_ptr_->setInputTarget(local_map_cloud_);
        return true;
    }

    void LidarMatching::PublistOdomData(const Eigen::Matrix4f &transform_matrix, ros::Time time)
    {
        nav_msgs::Odometry odometry;
        odometry.header.stamp = time;
        odometry.header.frame_id = "/map";
        odometry.child_frame_id = "/lidar";
        // set the position
        odometry.pose.pose.position.x = transform_matrix(0, 3);
        odometry.pose.pose.position.y = transform_matrix(1, 3);
        odometry.pose.pose.position.z = transform_matrix(2, 3);

        Eigen::Quaternionf q;
        q = transform_matrix.block<3, 3>(0, 0);
        odometry.pose.pose.orientation.x = q.x();
        odometry.pose.pose.orientation.y = q.y();
        odometry.pose.pose.orientation.z = q.z();
        odometry.pose.pose.orientation.w = q.w();

        laser_odom_pub_.publish(odometry);

        transform_.stamp_ = time;
        transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        transform_.setOrigin(tf::Vector3(transform_matrix(0, 3), transform_matrix(1, 3), transform_matrix(2, 3)));
        broadcaster_.sendTransform(transform_);

        return;
    }

    void LidarMatching::PublishCloudData(PointCloudXYZIPtr input_cloud, ros::Publisher cloud_pub, float sample_size)
    {
        ros::Time rosTime = ros::Time::now();
        sensor_msgs::PointCloud2 laserCloudMsg;
        // sample size = 0.0
        if (sample_size < 1.0e-10)
        {
            pcl::toROSMsg(*input_cloud, laserCloudMsg);
        }
        else
        {
            PointCloudXYZIPtr filterCloud(new PointCloudXYZI());
            pcl::VoxelGrid<pointXYZI> sor;
            sor.setInputCloud(input_cloud);
            sor.setLeafSize(sample_size, sample_size, sample_size);
            sor.setMinimumPointsNumberPerVoxel(1);
            sor.filter(*filterCloud);
            pcl::toROSMsg(*filterCloud, laserCloudMsg);
        }
        laserCloudMsg.header.stamp = rosTime;
        laserCloudMsg.header.frame_id = "/map";
        cloud_pub.publish(laserCloudMsg);
    }
}
