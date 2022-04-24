#include "g2o/equirectangular_pose_edge.h"

namespace g2o {

EquirectangularPoseEdge::EquirectangularPoseEdge(SE3Quat mVertexSE3,
                                                 Matrix<double, 6, 6> adj) {

  se3_camdext_adj_ = adj;
  se3_cam_ext_ = mVertexSE3;
  img_low_bound_ = 200.0;
  rad2pixel_ = 640.0 / M_PI;
}

bool EquirectangularPoseEdge::read(std::istream &) {
  std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
  return false;
}

bool EquirectangularPoseEdge::write(std::ostream &) const {
  std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
  return false;
}

void EquirectangularPoseEdge::reset() {
  if (_robustKernel) {
    delete _robustKernel;
    _robustKernel = nullptr;
  }

  _dimension = -1;
  _level = 0;
  _internalId = -1;
  _cacheIds.clear();
  _parameterTypes.clear();
  _parameters.clear();
  _parameterIds.clear();
  _vertices.clear();
  _vertices.resize(1);
  _id = -1;
}

float EquirectangularPoseEdge::getError() {
  const VertexSE3Expmap *sw = static_cast<const VertexSE3Expmap *>(
      _vertices[0]); // Multi Camera System Pose
  Vector2d obs(_measurement);
  Vector2d error = obs - cam_project(se3_cam_ext_.map(sw->estimate().map(pw_)));
  return error.norm();
}

void EquirectangularPoseEdge::computeError() {
  const VertexSE3Expmap *sw = static_cast<const VertexSE3Expmap *>(
      _vertices[0]); // Multi Camera System Pose
  Vector2d obs(_measurement);
  _error = obs - cam_project(se3_cam_ext_.map(sw->estimate().map(pw_)));
}

void EquirectangularPoseEdge::linearizeOplus() {
  const VertexSE3Expmap *sw =
      static_cast<VertexSE3Expmap *>(_vertices[0]); // Multi Camera System Pose
  SE3Quat se3quat_sw(sw->estimate());
  Vector3d pc = se3_cam_ext_.map(se3quat_sw.map(pw_));

  double x = pc.x();
  double y = pc.y();
  double z = pc.z();

  double r2 = x * x + y * y + z * z;

  Matrix<double, 2, 3> tmp;
  tmp(0, 0) = rad2pixel_ * z / (x * x + z * z);
  tmp(0, 1) = 0;
  tmp(0, 2) = -rad2pixel_ * x / (x * x + z * z);

  tmp(1, 0) = -rad2pixel_ * x * y / (sqrt(x * x + z * z) * r2);
  tmp(1, 1) = rad2pixel_ * sqrt(x * x + z * z) / r2;
  tmp(1, 2) = -rad2pixel_ * y * z / (sqrt(x * x + z * z) * r2);

  Matrix<double, 3, 6> jac3;
  jac3(0, 0) = 0;
  jac3(0, 1) = z;
  jac3(0, 2) = -y;
  jac3(0, 3) = 1;
  jac3(0, 4) = 0;
  jac3(0, 5) = 0;
  jac3(1, 0) = -z;
  jac3(1, 1) = 0;
  jac3(1, 2) = x;
  jac3(1, 3) = 0;
  jac3(1, 4) = 1;
  jac3(1, 5) = 0;
  jac3(2, 0) = y;
  jac3(2, 1) = -x;
  jac3(2, 2) = 0;
  jac3(2, 3) = 0;
  jac3(2, 4) = 0;
  jac3(2, 5) = 1;

  Matrix<double, 2, 6> jaco_mcs;
  jaco_mcs = -tmp * jac3 * se3_camdext_adj_;
  _jacobianOplusXi = jaco_mcs;
}

Eigen::Vector2d
EquirectangularPoseEdge::cam_project(const Eigen::Vector3d &pos_c) const {

  Vector2d reproj;
  Vector3d point = pos_c / pos_c.norm();

  const auto latitude = -std::asin(point.y());
  auto longitude = std::atan2(point.z(), point.x());

  if (longitude < -M_PI / 2.0f)
    longitude += 2.0f * M_PI;

  reproj.x() = rad2pixel_ * (-longitude + M_PI);
  reproj.y() = rad2pixel_ * (-latitude + M_PI / 2.0f) - img_low_bound_;
  return reproj;
}
}
