#include "camera_parameter.h"

namespace g2o {

Vector2D project2d(const Vector3D &v) {
  Vector2D res;
  res(0) = v(0) / v(2);
  res(1) = v(1) / v(2);
  return res;
}

CameraParameters::CameraParameters()
    : focal_length_x_(1.0), focal_length_y_(1.0),
      principle_point_(Vector2D(0, 0)) {}

Vector2D CameraParameters::cam_map(const Vector3D &trans_xyz) const {
  Vector2D proj = project2d(trans_xyz);
  Vector2D res;
  res[0] = proj[0] * focal_length_x_ + principle_point_[0];
  res[1] = proj[1] * focal_length_y_ + principle_point_[1];
  return res;
}
}
