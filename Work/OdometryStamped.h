#pragma once
#include <cmath>
#include <string>
#include <list>
#include <unordered_map>
#include <algorithm>

#include <Eigen/Dense>
#include <bfl/wrappers/matrix/matrix_wrapper.h>

using namespace std;
// using namespace Eigen;
using namespace MatrixWrapper;

class OdometryStamped
{
public:
    using Time = double;
    
    struct PoseStamped
    {
        PoseStamped() = default;
        PoseStamped(const Time &time_, const Eigen::Vector3d &position_, const Eigen::Quaterniond &orientation_, const SymmetricMatrix &covariance_)
            : time(time_), position(position_), orientation(orientation_), covariance(covariance_) {}

        Time time;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        SymmetricMatrix covariance;
    };

    void reset()
    {
        sensor_list_.clear();
    }

    bool hasTrajectory(const std::string& sensor)
    {
        return sensor_list_.find(sensor) != sensor_list_.end();
    }

    void addMeasurement(const std::string &sensor, const Time &time, const Eigen::Vector3d &position,
                        const Eigen::Quaterniond &orientation, const SymmetricMatrix &covariance)
    {
        sensor_list_[sensor].emplace_back(time, position, orientation, covariance);
    }

    bool lookupMeasurementByStamp(const std::string &sensor, const Time &time, Eigen::Vector3d &position,
                                  Eigen::Quaterniond &orientation, SymmetricMatrix &covariance)
    {
        dropUselessMeasurement(sensor, time);
        return lookupMeasurementOrLerp(sensor, time, position, orientation, covariance);
    }

private:
    void dropUselessMeasurement(const std::string &sensor, const Time &time)
    {
        if (sensor_list_[sensor].size() <= 1)
        {
            return;
        }

        auto iterS = sensor_list_[sensor].begin();
        auto iterE = ++sensor_list_[sensor].begin();
        if (time <= iterS->time)
        {
            return;
        }

        while (true)
        {
            if (iterE == sensor_list_[sensor].end())
            {
                break;
            }

            if (time <= iterE->time)
            {
                break;
            }
            else
            {
                sensor_list_[sensor].erase(iterS++);
                ++iterE;
            }
        }
    }

    bool lookupMeasurementOrLerp(const std::string &sensor, const Time &time, Eigen::Vector3d &position,
                                 Eigen::Quaterniond &orientation, SymmetricMatrix &covariance)
    {
        auto size = sensor_list_[sensor].size();
        if (size == 0)
        {
            return false;
        }

        auto iterS = sensor_list_[sensor].begin();
        if (size == 1)
        {
            if (time == iterS->time)
            {
                position = iterS->position;
                orientation = iterS->orientation;
                covariance = iterS->covariance;
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            if (time < iterS->time)
            {
                return false;
            }

            auto iterE = ++sensor_list_[sensor].begin();
            PoseStamped tag;
            lerp(*iterS, *iterE, time, tag);
            position = tag.position;
            orientation = tag.orientation;
            covariance = tag.covariance;
            return true;
        }
    }

    void lerp(const PoseStamped& poseS, const PoseStamped& poseE, const Time& time, PoseStamped& tag)
    {
        double t = (time - poseS.time) / (poseE.time - poseS.time);
        tag.position = poseS.position + t * (poseE.position - poseS.position);
        tag.orientation = poseS.orientation.slerp(t, poseE.orientation);
        tag.covariance.resize(poseS.covariance.size());

        for (auto i = 1; i <= poseS.covariance.size(); ++i)
        {
            tag.covariance(i, i) = std::max(poseS.covariance(i, i), poseE.covariance(i, i));
        }
    }

private:
    std::unordered_map<std::string, std::list<PoseStamped>> sensor_list_;
};

