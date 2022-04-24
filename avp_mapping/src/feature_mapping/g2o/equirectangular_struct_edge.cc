#include "g2o/equirectangular_struct_edge.h"

namespace g2o {

EquirectangularStructEdge::EquirectangularStructEdge(SE3Quat sys_pose,
                                                     g2o::SE3Quat mVertexSE3,
                                                     Matrix<double, 6, 6> adj) {
  se3_sys_pose_ = sys_pose;
  se3_camdext_adj_ = adj;
  se3_cam_ext_ = mVertexSE3;
  img_low_bound_ = 200.0;
  rad2pixel_ = 640.0 / M_PI;
}

float EquirectangularStructEdge::getError() {
  const VertexSBAPointXYZ *pt3 =
      static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
  Vector2d obs(_measurement);
  Vector2d error =
      obs - cam_project(se3_cam_ext_.map(se3_sys_pose_.map(pt3->estimate())));
  return error.norm();
}

void EquirectangularStructEdge::linearizeOplus() {

  const VertexSBAPointXYZ *pt3 =
      static_cast<VertexSBAPointXYZ *>(_vertices[0]); // World Coordinate Points

  Vector3d xyz = pt3->estimate();
  Vector3d xyz_trans = se3_cam_ext_.map(se3_sys_pose_.map(xyz));

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];

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

  if (!pt3->fixed()) {
    SE3Quat se3_cw = se3_cam_ext_ * se3_sys_pose_;
    Matrix<double, 2, 3> jac_points;
    jac_points = -tmp * se3_cw.rotation().toRotationMatrix();
    _jacobianOplusXi = jac_points;
  } else {
    Matrix<double, 2, 3> jac_points;
    jac_points << 0, 0, 0, 0, 0, 0;
    _jacobianOplusXi = jac_points;
  }
}

void EquirectangularStructEdge::computeError() {
  const VertexSBAPointXYZ *pt3 =
      static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
  Vector2d obs(_measurement);
  _error =
      obs - cam_project(se3_cam_ext_.map(se3_sys_pose_.map(pt3->estimate())));
}

Vector2d EquirectangularStructEdge::cam_project(const Vector3d &pos_c) const {

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
