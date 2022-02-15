#pragma once
#include <string>
#include <vector>
#include <list>
#include <unordered_map>
#include <algorithm>
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "Pose.h"
using namespace std;
using namespace ros;
using namespace Eigen;


class EvaluateTrajectoryErrorAtEachInterval
{
public:
    struct MapPoseStamp
    {
        MapPoseStamp(const geometry_msgs::PoseStamped &map_pose, bool _change_floor)
            : change_floor(_change_floor), pose_stamped(map_pose) {}

        bool change_floor; // new start
        geometry_msgs::PoseStamped pose_stamped;
    };

    void setLocalTrajectoryName(const std::vector<std::string>& trajectory_list)
    {
        local_trajectory_name_.clear();

        for (const auto& name : trajectory_list)
        {
            local_trajectory_name_.emplace(name);
        }

        reset();
    }

    void reset()
    {
        for (auto& name : local_trajectory_name_)
        {
            dis_error_[name].clear();
            ang_error_[name].clear();
            local_trajectory_list_[name].clear();
            local_trajectory_list_interpolation_[name].clear();
            dis_exception_time_[name] = 0;
            ang_exception_time_[name] = 0;
        }

        global_trajectory_stamps_.clear();
    }

    bool hasTrajectory(const std::string& trajectory)
    {
        return local_trajectory_name_.find(trajectory) != local_trajectory_name_.end();
    }

    void addGlobalTrajectoryPose(const geometry_msgs::PoseStamped& pose, bool change_floor = false)
    {
        global_trajectory_stamps_.emplace_back(pose, change_floor);
    }

    void addTrajectoryPose(const std::string& sensor, const geometry_msgs::PoseStamped &pose)
    {
        if (hasTrajectory(sensor))
        {
            local_trajectory_list_[sensor].emplace_back(pose);
        }
    }

    void evaluate()
    {
        // 1.clip
        clipGlobalTrajectory();

        // 2.插值计算
        calculateInterpolationByGlobalStamps();

        // 3.evaluateError
        evaluateError();
    }

    void traverseError(const std::string& trajectory)
    {
        std::cout << trajectory << " dis_error: ";
        for (auto &error : dis_error_[trajectory])
        {
            std::cout << error << " ";
        }
        std::cout << std::endl;
        std::cout << std::endl;

        std::cout << trajectory << " ang_error: ";
        for (auto &error : ang_error_[trajectory])
        {
            std::cout << error << " ";
        }
        std::cout << std::endl;
    }

    double getDisErrorMax(const std::string& trajectory)
    {
        return dis_error_[trajectory].front();
    }

    double getAngErrorMax(const std::string &trajectory)
    {
        return ang_error_[trajectory].front();
    }

    double getDisErrorTopMean(const std::string& trajectory, int top = 10)
    {
        if (top > dis_error_[trajectory].size())
        {
            top = dis_error_[trajectory].size();
        }

        double total = 0;
        auto iter = dis_error_[trajectory].begin();
        for (auto i = top; i > 0; --i)
        {
            total += *iter;
            ++iter;
        }
        return total / top;
    }

    double getAngErrorTopMean(const std::string &trajectory, int top = 10)
    {
        if (top > ang_error_[trajectory].size())
        {
            top = ang_error_[trajectory].size();
        }

        double total = 0;
        auto iter = ang_error_[trajectory].begin();
        for (auto i = top; i > 0; --i)
        {
            total += *iter;
            ++iter;
        }
        return total / top;
    }

    double getDisErrorMean(const std::string& trajectory)
    {
        return getDisErrorTopMean(trajectory, -1);
    }

    double getAngErrorMean(const std::string& trajectory)
    {
        return getAngErrorTopMean(trajectory, -1);
    }

private:
    // map_pose_stamp_ 的时间需要裁减为：起始时间大于其他，终止时间小于其他
    void clipGlobalTrajectory()
    {
        ros::Time time_start = global_trajectory_stamps_.front().pose_stamped.header.stamp;
        ros::Time time_end = global_trajectory_stamps_.back().pose_stamped.header.stamp;
        for (auto& pair : local_trajectory_list_)
        {
            if (pair.second.front().header.stamp > time_start)
            {
                time_start = pair.second.front().header.stamp;
            }
            if (pair.second.back().header.stamp < time_end)
            {
                time_end = pair.second.back().header.stamp;
            }
        }

        while (global_trajectory_stamps_.back().pose_stamped.header.stamp > time_end)
        {
            global_trajectory_stamps_.pop_back();
        }

        while (global_trajectory_stamps_.front().pose_stamped.header.stamp < time_start)
        {
            global_trajectory_stamps_.pop_front();
        }
    }

    void calculateInterpolationForTrajectory(const std::string& trajectory)
    {
        auto linear_interpolation = [](const ros::Time &timeS, const geometry_msgs::Pose &poseS,
                                       const ros::Time &timeE, const geometry_msgs::Pose &poseE,
                                       const ros::Time &time_tag)
        {
            if (time_tag == timeS)
            {
                return poseS;
            }

            geometry_msgs::Pose pose_tag;
            Eigen::Vector3d pointS, pointE;
            tf::pointMsgToEigen(poseS.position, pointS);
            tf::pointMsgToEigen(poseE.position, pointE);
            Eigen::Quaterniond quatS, quatE;
            tf::quaternionMsgToEigen(poseS.orientation, quatS);
            tf::quaternionMsgToEigen(poseE.orientation, quatE);

            const double& t = (time_tag - timeS).toSec() / (timeE - timeS).toSec();
            tf::pointEigenToMsg(pointS + t * (pointE - pointS), pose_tag.position);
            tf::quaternionEigenToMsg(quatS.slerp(t, quatE), pose_tag.orientation);
            return pose_tag;
        };

        auto iterS = local_trajectory_list_[trajectory].begin();
        auto iterE = iterS;

        for (auto &map_pose : global_trajectory_stamps_)
        {
            while (iterE->header.stamp < map_pose.pose_stamped.header.stamp)
            {
                ++iterE;
            }

            geometry_msgs::PoseStamped pose_tag;
            pose_tag.header = iterE->header;
            pose_tag.header.stamp = map_pose.pose_stamped.header.stamp;
            pose_tag.pose = linear_interpolation(iterS->header.stamp, iterS->pose, iterE->header.stamp, iterE->pose, map_pose.pose_stamped.header.stamp);
            local_trajectory_list_interpolation_[trajectory].emplace_back(pose_tag);
            iterS = iterE;
            ++iterE;
        }
    }

    void calculateInterpolationByGlobalStamps()
    {
        // 10个为一个间隔(1s)
        int cnt = 0;
        decltype(global_trajectory_stamps_) tmp;
        for (auto iter = global_trajectory_stamps_.rbegin(); iter != global_trajectory_stamps_.rend(); ++iter)
        {
            if (iter->change_floor)
            {
                cnt = 0;
            }
            if (cnt++ % 10 == 0)
            {
                tmp.emplace_back(*iter);
            }
        }
        global_trajectory_stamps_ = tmp;
        for (auto& name : local_trajectory_name_)
        {
            calculateInterpolationForTrajectory(name);
            assert(local_trajectory_list_interpolation_[name].size() == global_trajectory_stamps_.size());
        }
    }

    void evaluateError()
    {
        auto poseStamped2Pose3d = [](const geometry_msgs::PoseStamped &pose)
        {
            Vector3d trans(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            Quaterniond quat(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
            return Pose3d(trans, quat);
        };

        for (auto& name : local_trajectory_name_)
        {
            auto iter_gs = global_trajectory_stamps_.begin();
            auto iter_ge = global_trajectory_stamps_.begin();
            auto iter_ls = local_trajectory_list_interpolation_[name].begin();
            auto iter_le = local_trajectory_list_interpolation_[name].begin();

#define DEBUG 0

#if DEBUG
            std::cout << name << std::endl;
#endif
            while (++iter_ge != global_trajectory_stamps_.end())
            {
                ++iter_le;
                if (iter_ge->change_floor)
                {
                    iter_gs = iter_ge;
                    iter_ls = iter_le;
                    continue;
                }

                // 此处统计的是线速度和角速度的误差
                // l:local, g:global
                auto dis_l = poseStamped2Pose3d(*iter_ls).distance2d(poseStamped2Pose3d(*iter_le));
                auto ang_l = Pose3d::NormalizeAngle(poseStamped2Pose3d(*iter_ls).deltaAngle(poseStamped2Pose3d(*iter_le))(0), -M_PI, M_PI);
                auto dis_g = poseStamped2Pose3d(iter_gs->pose_stamped).distance2d(poseStamped2Pose3d(iter_ge->pose_stamped));
                auto ang_g = Pose3d::NormalizeAngle(poseStamped2Pose3d(iter_gs->pose_stamped).deltaAngle(poseStamped2Pose3d(iter_ge->pose_stamped))(0), -M_PI, M_PI);

                double thld_d = 5e-3;   // 5mm
                double thld_a = 1e-2;   // ~0.5度
#if DEBUG
                if (std::abs(dis_g) < thld_d || std::abs(dis_l) < thld_d)
                {
                    std::cout << "dis_g == 0 && dis_l != 0: " << dis_l << std::endl;
                }
                else if (std::abs(dis_g) > thld_d)
                {
                    std::cout << "std::abs(dis_rr) = " << std::abs((dis_g - dis_l) / dis_g) << ", dis_g = " << dis_g << ", dis_l = " << dis_l << std::endl;
                }
                if (std::abs(ang_g) < thld_a || std::abs(ang_l) < thld_a)
                {
                    std::cout << "ang_g == 0 && ang_l != 0: " << ang_l << std::endl;
                }
                else if (std::abs(ang_g) > thld_a)
                {
                    std::cout << "std::abs(ang_rr) = " << std::abs((ang_g - ang_l) / ang_g) << ", ang_g = " << ang_g << ", ang_l = " << ang_l << std::endl;
                }

                if (std::abs(ang_l) > 3)
                {
                    std::cout << "std::abs(ang_l) > 3: " << poseStamped2Pose3d(*iter_ls) << poseStamped2Pose3d(*iter_le) << std::endl;
                }
                if (std::abs(ang_g) > 3)
                {
                    std::cout << "std::abs(ang_g) > 3: " << poseStamped2Pose3d(iter_gs->pose_stamped) << poseStamped2Pose3d(iter_ge->pose_stamped) << std::endl;
                }
#endif
                // auto dis_error_mu = ;
                // dis: 多走少走
                // ang: 多转少转，正转反转
                if (std::abs(dis_g) < thld_d || std::abs(dis_l) < thld_d)
                {
                    ++dis_exception_time_[name];
                }
                else if (std::abs(dis_l) > 1.5 || std::abs(dis_g) > 1.5)
                {
                    ++dis_exception_time_[name];
                }
                else if (std::abs(dis_g) > thld_d)
                {
                    dis_error_[name].emplace_back(std::abs((dis_g - dis_l) / dis_g));
                }
                if (std::abs(ang_g) < thld_a || std::abs(ang_l) < thld_a)
                {
                    ++ang_exception_time_[name];
                }
                else if (std::abs(ang_l) > 3 || std::abs(ang_g) > 3)
                {
                    ++ang_exception_time_[name];
                }
                else if (std::abs(ang_g) > thld_a)
                {
                    ang_error_[name].emplace_back(std::abs((ang_g - ang_l) / ang_g));
                }

                iter_gs = iter_ge;
                iter_ls = iter_le;
            }

            dis_error_[name].sort(greater<double>());
            ang_error_[name].sort(greater<double>());

#if DEBUG
            std::cout << "dis_error_ size: " << dis_error_[name].size() << std::endl;
            std::cout << "ang_error_ size: " << ang_error_[name].size() << std::endl;

            std::cout << "dis_exception_time_=" << dis_exception_time_[name] << std::endl;
            std::cout << "ang_exception_time_=" << ang_exception_time_[name] << std::endl;
#endif
        }
    }

private:
    std::set<std::string> local_trajectory_name_;

    // 记录异常次数
    std::unordered_map<std::string, int> dis_exception_time_;
    std::unordered_map<std::string, int> ang_exception_time_;

    std::unordered_map<std::string, std::list<double>> dis_error_;
    std::unordered_map<std::string, std::list<double>> ang_error_;
    
    std::unordered_map<std::string, std::list<geometry_msgs::PoseStamped>> local_trajectory_list_;
    std::unordered_map<std::string, std::list<geometry_msgs::PoseStamped>> local_trajectory_list_interpolation_;

    std::list<MapPoseStamp> global_trajectory_stamps_;
};

