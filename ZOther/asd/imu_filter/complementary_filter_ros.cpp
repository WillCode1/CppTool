/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

        @section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
        All rights reserved.

        Redistribution and use in source and binary forms, with or without
        modification, are permitted provided that the following conditions are
  met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. Neither the name of the City College of New York nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND
        ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED
        WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
        DISCLAIMED. IN NO EVENT SHALL the CCNY ROBOTICS LAB BE LIABLE FOR ANY
        DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES
        (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES;
        LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  AND
        ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
  TORT
        (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
  THIS
        SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "aw_mapping_robot/imu_filter/complementary_filter_ros.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <Eigen/Dense>

namespace aw{
namespace mapping_robot{

namespace {

void ExtrinsicRotImuMsg(sensor_msgs::Imu* imu_msg) {

Eigen::Matrix3d extRot;
extRot <<
1.00000000e+00,   0,  0,
0,   1.00000000e+00,  0, 
0,   0,   1.00000000e+00;  
    

Eigen::Quaterniond extQRPY(extRot);

	// rotate acceleration
	Eigen::Vector3d acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    acc = extRot * acc;
    imu_msg->linear_acceleration.x = acc.x();
    imu_msg->linear_acceleration.y = acc.y();
    imu_msg->linear_acceleration.z = acc.z();

	// rotate gyroscope
	Eigen::Vector3d gyr(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
	gyr = extRot * gyr;
    imu_msg->angular_velocity.x = gyr.x();
    imu_msg->angular_velocity.y = gyr.y();
    imu_msg->angular_velocity.z = gyr.z();

	// rotate orientation
	Eigen::Quaterniond q_from(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
	Eigen::Quaterniond q_final = extQRPY * q_from;
	imu_msg->orientation.x = q_final.x();
	imu_msg->orientation.y = q_final.y();
	imu_msg->orientation.z = q_final.z();
	imu_msg->orientation.w = q_final.w();

}

sensor_msgs::Imu CreateImuFromAwImu(const aw_idl::IMU& aw_imu_msg) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header = aw_imu_msg.header;
    imu_msg.header.frame_id = "imu_link";

    imu_msg.angular_velocity.x = aw_imu_msg.gyroscopes[0];
    imu_msg.angular_velocity.y = -aw_imu_msg.gyroscopes[1];
    imu_msg.angular_velocity.z = -aw_imu_msg.gyroscopes[2];
    imu_msg.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01,
                                           0.0,  0.0, 0.0, 0.01};

    imu_msg.linear_acceleration.x = aw_imu_msg.accelerometers[0];
    imu_msg.linear_acceleration.y = -aw_imu_msg.accelerometers[1];
    imu_msg.linear_acceleration.z = -aw_imu_msg.accelerometers[2];
    imu_msg.linear_acceleration_covariance = {0.01, 0.0, 0.0, 0.0, 0.01,
                                              0.0,  0.0, 0.0, 0.01};

    imu_msg.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01,
                                      0.0,  0.0, 0.0, 0.01};
    return imu_msg;
}

}  // namespace

ComplementaryFilterROS::ComplementaryFilterROS(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), initialized_filter_(false) {
    ROS_INFO("Starting ComplementaryFilterROS");
    initializeParams();

    int queue_size = 5;

    if (publish_)
    {
        // Register publishers:
        imu_publisher_ = nh_.advertise<sensor_msgs::Imu>(
            ros::names::resolve("imu") + "/data", queue_size);
        // Register IMU raw data subscriber.
        imu_subscriber_.reset(new ImuSubscriber(nh_, "/aw/imu", queue_size));

        // Register magnetic data subscriber.
        imu_subscriber_->registerCallback(&ComplementaryFilterROS::imuCallback,
                                          this);
    }

    if (publish_debug_topics_) {
        rpy_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
            ros::names::resolve("imu") + "/rpy/filtered", queue_size);

        if (filter_.getDoBiasEstimation()) {
            state_publisher_ = nh_.advertise<std_msgs::Bool>(
                ros::names::resolve("imu") + "/steady_state", queue_size);
        }
    }
}

ComplementaryFilterROS::~ComplementaryFilterROS() {
    ROS_INFO("Destroying ComplementaryFilterROS");
}

void ComplementaryFilterROS::initializeParams() {
    double gain_acc;
    double gain_mag;
    bool do_bias_estimation;
    double bias_alpha;
    bool do_adaptive_gain;

    if (!nh_private_.getParam("fixed_frame", fixed_frame_))
        fixed_frame_ = "odom";
    if (!nh_private_.getParam("use_mag", use_mag_)) use_mag_ = true;
    if (!nh_private_.getParam("publish_tf", publish_tf_)) publish_tf_ = false;
    if (!nh_private_.getParam("reverse_tf", reverse_tf_)) reverse_tf_ = false;
    if (!nh_private_.getParam("constant_dt", constant_dt_)) constant_dt_ = 0.0;
    if (!nh_private_.getParam("publish_debug_topics", publish_debug_topics_))
        publish_debug_topics_ = false;
    if (!nh_private_.getParam("gain_acc", gain_acc)) gain_acc = 0.001;
    if (!nh_private_.getParam("gain_mag", gain_mag)) gain_mag = 0.001;
    if (!nh_private_.getParam("do_bias_estimation", do_bias_estimation))
        do_bias_estimation = false;
    if (!nh_private_.getParam("bias_alpha", bias_alpha)) bias_alpha = 0.01;
    if (!nh_private_.getParam("do_adaptive_gain", do_adaptive_gain))
        do_adaptive_gain = true;
    if (!nh_private_.getParam("publish", publish_))
        publish_= true;
    filter_.setDoBiasEstimation(do_bias_estimation);
    filter_.setDoAdaptiveGain(do_adaptive_gain);

    if (!filter_.setGainAcc(gain_acc))
        ROS_WARN("Invalid gain_acc passed to ComplementaryFilter.");
    if (use_mag_) {
        if (!filter_.setGainMag(gain_mag))
            ROS_WARN("Invalid gain_mag passed to ComplementaryFilter.");
    }
    if (do_bias_estimation) {
        if (!filter_.setBiasAlpha(bias_alpha))
            ROS_WARN("Invalid bias_alpha passed to ComplementaryFilter.");
    }

    // check for illegal constant_dt values
    if (constant_dt_ < 0.0) {
        // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
        // otherwise, it will be constant
        ROS_WARN("constant_dt parameter is %f, must be >= 0.0. Setting to 0.0",
                 constant_dt_);
        constant_dt_ = 0.0;
    }
}

/*
void ComplementaryFilterROS::imuCallback(const ImuMsg::ConstPtr& imu_msg_raw) {
    const geometry_msgs::Vector3& a = imu_msg_raw->linear_acceleration;
    const geometry_msgs::Vector3& w = imu_msg_raw->angular_velocity;
    const ros::Time& time = imu_msg_raw->header.stamp;

    // Initialize.
    if (!initialized_filter_) {
        time_prev_ = time;
        initialized_filter_ = true;
        return;
    }

    // determine dt: either constant, or from IMU timestamp
    double dt;
    if (constant_dt_ > 0.0)
        dt = constant_dt_;
    else
        dt = (time - time_prev_).toSec();

    time_prev_ = time;

    // Update the filter.
    filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);

    // Publish state.
    publish(imu_msg_raw);
}
*/

void ComplementaryFilterROS::imuCallback(
    const aw_idl::IMU::ConstPtr& imu_msg_raw) {
    const boost::array<double, 3>& a = imu_msg_raw->accelerometers;
    const boost::array<double, 3>& w = imu_msg_raw->gyroscopes;
    const boost::array<double, 3>& m = imu_msg_raw->magnetometers;
    const ros::Time& time = imu_msg_raw->header.stamp;

    // Initialize.
    if (!initialized_filter_) {
        time_prev_ = time;
        initialized_filter_ = true;
        skip_cnt_ = 0;
        return;
    }

    //lwy
    bool valid = true;
    for(int i = 0; i < 3; ++i){
        if(a[i] > -20. || a[i] < 20)
            continue;
        else{
            valid = false;
            ROS_WARN_STREAM("IMU Accelerometers: " << a[0] << " " << a[1] << " " << a[2]);
            break;
        }
    }
    if(!valid){
        ++skip_cnt_;
        return;
    }

    // Calculate dt.
    //double dt = 1.0 / 200 * (skip_cnt_ + 1);
    double dt = (time - time_prev_).toSec();
    //time_prev_ = time;
    dt = (dt < 1e-4) ? 1.0 / 200 * (skip_cnt_ + 1): dt;

    // ros::Time t_in, t_out;
    // t_in = ros::Time::now();
    // Update the filter.

    if (!use_mag_) {
        filter_.update(a.at(0), -a.at(1), -a.at(2), w.at(0), -w.at(1), -w.at(2),
        dt);
    } else {
        filter_.update(a.at(0), -a.at(1), -a.at(2), w.at(0), -w.at(1), -w.at(2),
                   m.at(0), -m.at(1), -m.at(2), dt);
    }
    // t_out = ros::Time::now();
    // float dt_tot = (t_out - t_in).toSec() * 1000.0; // In msec.
    // printf("%.6f\n", dt_tot);
    // Publish state.
    publish(imu_msg_raw);
}

tf::Quaternion ComplementaryFilterROS::hamiltonToTFQuaternion(double q0,
                                                              double q1,
                                                              double q2,
                                                              double q3) const {
    // ROS uses the Hamilton quaternion convention (q0 is the scalar). However,
    // the ROS quaternion is in the form [x, y, z, w], with w as the scalar.
    return tf::Quaternion(q1, q2, q3, q0);
}

void ComplementaryFilterROS::publish(const aw_idl::IMU::ConstPtr& imu_msg_raw) {
    // Get the orientation:
    double q0, q1, q2, q3;
    filter_.getOrientation(q0, q1, q2, q3);

    tf::Quaternion q = hamiltonToTFQuaternion(q0, q1, q2, q3);

    // Create and publish fitlered IMU message.

    boost::shared_ptr<sensor_msgs::Imu> imu_msg =
        boost::make_shared<sensor_msgs::Imu>(CreateImuFromAwImu(*imu_msg_raw));
    tf::quaternionTFToMsg(q, imu_msg->orientation);

    // Account for biases.
    if (filter_.getDoBiasEstimation()) {
        imu_msg->angular_velocity.x -= filter_.getAngularVelocityBiasX();
        imu_msg->angular_velocity.y -= filter_.getAngularVelocityBiasY();
        imu_msg->angular_velocity.z -= filter_.getAngularVelocityBiasZ();
    }

    const ros::Time &time = imu_msg_raw->header.stamp;
    double dt = (time - time_prev_).toSec();
    dt = (dt < 1e-5) ? 1.0 / 200 * (skip_cnt_ + 1) : dt;
    ros::Time compensate_time(time_prev_.toSec() + dt);
    time_prev_ = compensate_time;
    imu_msg->header.stamp = time_prev_;
    skip_cnt_ = 0;

    /*
    Eigen::Quaterniond qq1(imu_msg->orientation.w, imu_msg->orientation.x,
                         imu_msg->orientation.y, imu_msg->orientation.z);
    Eigen::Vector3d euler1 = qq1.toRotationMatrix().eulerAngles(0, 1, 2);
    std::cout << "before q: " << euler1(0) << ", " << euler1(1) << ", " << euler1(2) << std::endl;
    */
    //ExtrinsicRotImuMsg(imu_msg.get());

    /*
    Eigen::Quaterniond qq2(imu_msg->orientation.w, imu_msg->orientation.x,
                         imu_msg->orientation.y, imu_msg->orientation.z);
    Eigen::Vector3d euler2 = qq2.toRotationMatrix().eulerAngles(0, 1, 2);
    std::cout << "after q: " << euler2(0) << ", " << euler2(1) << ", " << euler2(2) << std::endl;
    */
    imu_publisher_.publish(imu_msg);

    if (publish_debug_topics_) {
        // Create and publish roll, pitch, yaw angles
        geometry_msgs::Vector3Stamped rpy;
        rpy.header = imu_msg_raw->header;

        tf::Matrix3x3 M;
        M.setRotation(q);
        M.getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
        rpy_publisher_.publish(rpy);

        // Publish whether we are in the steady state, when doing bias
        // estimation
        if (filter_.getDoBiasEstimation()) {
            std_msgs::Bool state_msg;
            state_msg.data = filter_.getSteadyState();
            state_publisher_.publish(state_msg);
        }
    }

    if (publish_tf_) {
        // Create and publish the ROS tf.
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform.setRotation(q);

        if (reverse_tf_) {
            tf_broadcaster_.sendTransform(tf::StampedTransform(
                transform.inverse(), imu_msg_raw->header.stamp,
                imu_msg_raw->header.frame_id, fixed_frame_));
        } else {
            tf_broadcaster_.sendTransform(tf::StampedTransform(
                transform, imu_msg_raw->header.stamp, fixed_frame_,
                imu_msg_raw->header.frame_id));
        }
    }
}

bool
ComplementaryFilterROS::updateAndConvertImu(const aw_idl::IMU::ConstPtr &imu_msg_raw,
                                        sensor_msgs::Imu& imu_msg)
{
    const boost::array<double, 3> &a = imu_msg_raw->accelerometers;
    const boost::array<double, 3> &w = imu_msg_raw->gyroscopes;
    const boost::array<double, 3> &m = imu_msg_raw->magnetometers;
    const ros::Time &time = imu_msg_raw->header.stamp;

    // Initialize.
    if (!initialized_filter_)
    {
        time_prev_ = time;
        initialized_filter_ = true;
        return false;
    }

    // Calculate dt.
    double dt = 1.0 / 200;
    //double dt = (time - time_prev_).toSec();
    //time_prev_ = time;
    //dt = (dt < 1e-4) ? 1.0 / 200 : dt;

    // ros::Time t_in, t_out;
    // t_in = ros::Time::now();
    // Update the filter.

    if (!use_mag_)
    {
        filter_.update(a.at(0), -a.at(1), -a.at(2), w.at(0), -w.at(1), -w.at(2),
                       dt);
    }
    else
    {
        filter_.update(a.at(0), -a.at(1), -a.at(2), w.at(0), -w.at(1), -w.at(2),
                       m.at(0), -m.at(1), -m.at(2), dt);
    }
    // t_out = ros::Time::now();
    // float dt_tot = (t_out - t_in).toSec() * 1000.0; // In msec.
    // printf("%.6f\n", dt_tot);
    // Publish state.
    double q0, q1, q2, q3;
    filter_.getOrientation(q0, q1, q2, q3);

    tf::Quaternion q = hamiltonToTFQuaternion(q0, q1, q2, q3);

    // Create and publish fitlered IMU message.

    imu_msg =CreateImuFromAwImu(*imu_msg_raw);
    tf::quaternionTFToMsg(q, imu_msg.orientation);

    // Account for biases.
    if (filter_.getDoBiasEstimation()) {
        imu_msg.angular_velocity.x -= filter_.getAngularVelocityBiasX();
        imu_msg.angular_velocity.y -= filter_.getAngularVelocityBiasY();
        imu_msg.angular_velocity.z -= filter_.getAngularVelocityBiasZ();
    }

    dt = (time - time_prev_).toSec();
    dt = (dt < 1e-5) ? 1.0 / 200 : dt;
    ros::Time compensate_time(time_prev_.toSec() + dt);
    time_prev_ = compensate_time;
    imu_msg.header.stamp = time_prev_;

    return true;
}

}
}
