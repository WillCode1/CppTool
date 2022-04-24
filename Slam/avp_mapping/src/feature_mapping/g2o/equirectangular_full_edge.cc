#include "g2o/equirectangular_full_edge.h"

namespace g2o {

EquirectangularFullEdge::EquirectangularFullEdge() {

  information().setIdentity();
  resize(3);
  img_low_bound_ = 200.0;
  rad2pixel_ = 640.0 / M_PI;
}

bool EquirectangularFullEdge::read(std::istream &) {
  std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
  return false;
}

bool EquirectangularFullEdge::write(std::ostream &) const {
  std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
  return false;
}

void EquirectangularFullEdge::reset() {
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

float EquirectangularFullEdge::getError() {
  const VertexSBAPointXYZ *pt3 = static_cast<const VertexSBAPointXYZ *>(
      _vertices[0]); // World Coordinate Points
  const VertexSE3Expmap *sw = static_cast<const VertexSE3Expmap *>(
      _vertices[1]); // Multi Camera System Pose

  const VertexSE3Expmap *cs =
      static_cast<const VertexSE3Expmap *>(_vertices[2]);

  Vector2d obs(_measurement);
  Vector2d error =
      obs -
      cam_project(cs->estimate().map(sw->estimate().map(pt3->estimate())));
  return error.norm();
}

void EquirectangularFullEdge::computeError() {
  const VertexSBAPointXYZ *pt3 = static_cast<const VertexSBAPointXYZ *>(
      _vertices[0]); // World Coordinate Points
  const VertexSE3Expmap *sw = static_cast<const VertexSE3Expmap *>(
      _vertices[1]); // Multi Camera System Pose

  const VertexSE3Expmap *cs =
      static_cast<const VertexSE3Expmap *>(_vertices[2]);

  Vector2d obs(_measurement);
  _error = obs -
           cam_project(cs->estimate().map(sw->estimate().map(pt3->estimate())));
}

void EquirectangularFullEdge::linearizeOplus() {

  const VertexSBAPointXYZ *pt3 =
      static_cast<VertexSBAPointXYZ *>(_vertices[0]); // World Coordinate Points
  const VertexSE3Expmap *sw =
      static_cast<VertexSE3Expmap *>(_vertices[1]); // Multi Camera System Pose

  const VertexSE3Expmap *cs = static_cast<VertexSE3Expmap *>(_vertices[2]);

  SE3Quat se3_quat_sw(sw->estimate());
  SE3Quat se3_quat_ext(cs->estimate());
  Vector3d xyz = pt3->estimate();
  Vector3d pc = cs->estimate().map(se3_quat_sw.map(xyz));

  double x = pc[0];
  double y = pc[1];
  double z = pc[2];

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

  Matrix<double, 2, 6> jacobianMCS_;
  jacobianMCS_ = -tmp * jac3 * se3_quat_ext.adj();

  _jacobianOplus[0].resize(2, 3);
  _jacobianOplus[1].resize(2, 6);
  _jacobianOplus[2].resize(2, 6);

  // system pose
  _jacobianOplus[1] = jacobianMCS_;

  if (!pt3->fixed()) {
    SE3Quat se3_cw = se3_quat_ext * se3_quat_sw;
    Matrix<double, 2, 3> jac_points;
    jac_points = -tmp * se3_cw.rotation().toRotationMatrix();
    _jacobianOplus[0] = jac_points;
  } else {
    Matrix<double, 2, 3> jac_points;
    jac_points << 0, 0, 0, 0, 0, 0;
    _jacobianOplus[0] = jac_points;
  }

  // extrinsic
  _jacobianOplus[2] = -tmp * jac3;
}

Vector2d EquirectangularFullEdge::cam_project(const Vector3d &pos_c) const {

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
