#ifndef __STUB_ODOM_ESTIMATION_NODE__
#define __STUB_ODOM_ESTIMATION_NODE__

#include <algorithm>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

#define private public
#include "robot_pose_ekf/odom_estimation_node.h"

using namespace tf;
using namespace ros;
using namespace std;


class Test_OdomEstimationNode: public estimation::OdomEstimationNode
{
public:
    int total = 0;
    int is_dynamic_scene_cnt = 0;

    Test_OdomEstimationNode()
    {
        timer_.stop();
    }

    Eigen::Vector3d getTotalCompensationRadianAbs()
    {
        if (imu_zero_drift_compensation_)
        {
            return imu_zero_drift_compensation_->getTotalCompensationRadianAbs();
        }
        else
        {
            return Eigen::Vector3d::Zero();
        }
    }

    void costmapTrackerCallback(const costmap_tracker::ObstacleArrayMsg::ConstPtr &obstacles)
    {
        estimation::OdomEstimationNode::costmapTrackerCallback(obstacles);
        if (is_dynamic_scene_)
            ++is_dynamic_scene_cnt;
        ++total;
    }

    bool spin(nav_msgs::Odometry& odom_fuse, const ros::Time& fuse_time)
    {
        bool is_update = false;
        ROS_DEBUG("Spin function at time %f", fuse_time.toSec());

        // check for timing problems
        if ((odom_initializing_ || odom_active_) && (imu_initializing_ || imu_active_))
        {
            double diff = fabs(Duration(odom_stamp_ - imu_stamp_).toSec());
            if (diff > 1.0)
                ROS_ERROR("Timestamps of odometry and imu are %f seconds apart.", diff);
        }

        // initial value for filter stamp; keep this stamp when no sensors are active
        filter_stamp_ = fuse_time;

        // check which sensors are still active
        if ((odom_active_ || odom_initializing_) &&
            (filter_stamp_ - odom_time_).toSec() > timeout_)
        {
            odom_active_ = false;
            odom_initializing_ = false;
            ROS_INFO("Odom sensor not active any more");
        }
        if ((imu_active_ || imu_initializing_) &&
            (filter_stamp_ - imu_time_).toSec() > timeout_)
        {
            imu_active_ = false;
            imu_initializing_ = false;
            ROS_INFO("Imu sensor not active any more");
        }
        if ((lo_active_ || lo_initializing_) &&
            (filter_stamp_ - lo_time_).toSec() > timeout_)
        {
            lo_active_ = false;
            lo_initializing_ = false;
            ROS_INFO("Lo sensor not active any more");
        }
        if ((vo_active_ || vo_initializing_) &&
            (filter_stamp_ - vo_time_).toSec() > timeout_)
        {
            vo_active_ = false;
            vo_initializing_ = false;
            ROS_INFO("VO sensor not active any more");
        }

        if ((gps_active_ || gps_initializing_) &&
            (filter_stamp_ - gps_time_).toSec() > timeout_)
        {
            gps_active_ = false;
            gps_initializing_ = false;
            ROS_INFO("GPS sensor not active any more");
        }

        // only update filter when one of the sensors is active
        if (odom_active_ || imu_active_ || vo_active_ || gps_active_ || lo_active_)
        {
            // update filter at time where all sensor measurements are available
            if (odom_active_)
                filter_stamp_ = min(filter_stamp_, odom_stamp_);
            if (imu_active_)
                filter_stamp_ = min(filter_stamp_, imu_stamp_);
            if (lo_active_)
                filter_stamp_ = min(filter_stamp_, lo_stamp_);
            if (vo_active_)
                filter_stamp_ = min(filter_stamp_, vo_stamp_);
            if (gps_active_)
                filter_stamp_ = min(filter_stamp_, gps_stamp_);

            // update filter
            if (my_filter_.isInitialized())
            {
                bool diagnostics = true;
                if (my_filter_.update(odom_active_, imu_active_, gps_active_, vo_active_, lo_active_, filter_stamp_, diagnostics))
                {
                    is_update = true;
                    my_filter_.getEstimate(odom_fuse);
                    ekf_sent_counter_++;

                    if (false)
                    {
                        // output most recent estimate and relative covariance
                        my_filter_.getEstimate(output_);
                        pose_pub_.publish(output_);
                        my_filter_.getEstimate(odom_output_);
                        odom_pub_.publish(odom_output_);

                        // broadcast most recent estimate to TransformArray
                        StampedTransform tmp;
                        my_filter_.getEstimate(tmp);
                        if (!vo_active_ && !gps_active_)
                            tmp.getOrigin().setZ(0.0);
                        odom_broadcaster_.sendTransform(StampedTransform(tmp, tmp.stamp_, output_frame_, base_footprint_frame_));
                    }
                }
                if (self_diagnose_ && !diagnostics)
                    ROS_WARN("Robot pose ekf diagnostics discovered a potential problem");
            }

            // initialize filter with odometry frame
            if (imu_active_ && gps_active_ && !my_filter_.isInitialized())
            {
                Quaternion q = imu_meas_.getRotation();
                Vector3 p = gps_meas_.getOrigin();
                Transform init_meas_ = Transform(q, p);
                my_filter_.initialize(init_meas_, gps_stamp_);
                ROS_INFO("Kalman filter initialized with gps and imu measurement");
            }
            else if (odom_active_ && gps_active_ && !my_filter_.isInitialized())
            {
                Quaternion q = odom_meas_.getRotation();
                Vector3 p = gps_meas_.getOrigin();
                Transform init_meas_ = Transform(q, p);
                my_filter_.initialize(init_meas_, gps_stamp_);
                ROS_INFO("Kalman filter initialized with gps and odometry measurement");
            }
            else if (vo_active_ && gps_active_ && !my_filter_.isInitialized())
            {
                Quaternion q = vo_meas_.getRotation();
                Vector3 p = gps_meas_.getOrigin();
                Transform init_meas_ = Transform(q, p);
                my_filter_.initialize(init_meas_, gps_stamp_);
                ROS_INFO("Kalman filter initialized with gps and visual odometry measurement");
            }
            else if (odom_active_ && !gps_used_ && !my_filter_.isInitialized())
            {
                my_filter_.initialize(odom_meas_, odom_stamp_);
                ROS_INFO("Kalman filter initialized with odom measurement");
            }
            else if (vo_active_ && !gps_used_ && !my_filter_.isInitialized())
            {
                my_filter_.initialize(vo_meas_, vo_stamp_);
                ROS_INFO("Kalman filter initialized with vo measurement");
            }
        }
        return is_update;
    }
};

#endif
