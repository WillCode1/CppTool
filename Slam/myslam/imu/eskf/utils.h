#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace aw {

inline Eigen::Matrix3d skew_matrix(const Eigen::Vector3d &v) {
    Eigen::Matrix3d w;
    w << 0., -v(2), v(1), v(2), 0., -v(0), -v(1), v(0), 0.;
    return w;
}

inline Eigen::Matrix4d quat_left_matrix(const Eigen::Quaterniond &q) {
    Eigen::Matrix4d m4 = Eigen::Matrix4d::Zero();
    m4.block<3, 1>(1, 0) = q.vec();
    m4.block<1, 3>(0, 1) = -q.vec();
    m4.block<3, 3>(1, 1) = skew_matrix(q.vec());
    m4 += Eigen::Matrix4d::Identity() * q.w();
    return m4;
}

inline Eigen::Matrix4d quat_right_matrix(const Eigen::Quaterniond &q) {
    Eigen::Matrix4d m4 = Eigen::Matrix4d::Zero();
    m4.block<3, 1>(1, 0) = q.vec();
    m4.block<1, 3>(0, 1) = -q.vec();
    m4.block<3, 3>(1, 1) = -skew_matrix(q.vec());
    m4 += Eigen::Matrix4d::Identity() * q.w();
    return m4;
}

}  // namespace aw
