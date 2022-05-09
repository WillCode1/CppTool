#include "edge_se2_semantic_project.h"

namespace g2o {

// 判断当前是否还在，与车运行方向相反
bool EdgeSE2SemanticProject::IsWithinimage() {
  const VertexSE2 *v = static_cast<const VertexSE2 *>(_vertices[0]);

  Eigen::Vector3d pose = v->estimate().toVector();
  double theta = pose.z();
  double inv_x = -cos(theta) * pose.x() - sin(theta) * pose.y();
  double inv_y = sin(theta) * pose.x() - cos(theta) * pose.y();
  double inv_theta = -theta;

  Eigen::Matrix3d inv_pose;
  inv_pose << cos(inv_theta), -sin(inv_theta), inv_x, sin(inv_theta), cos(inv_theta), inv_y, 0, 0, 1;

  Eigen::Vector3d point_vehicle = inv_pose * wp_;
  Vector3D point_camera(-point_vehicle.y(), -point_vehicle.x() + baselink2cam_, cam_height_);
  Vector2D uv = cam_->cam_map(point_camera);

  if (uv.x() < 0 || uv.y() < 0 || uv.x() > image_->cols || uv.y() > image_->rows)
    return false;
  return true;
}

void EdgeSE2SemanticProject::linearizeOplus() {
  if (level() == 1) {
    _jacobianOplusXi = Eigen::Matrix<double, 1, 3>::Zero();
    return;
  }

  const VertexSE2 *vi = static_cast<const VertexSE2 *>(_vertices[0]);
  Eigen::Vector3d pose = vi->estimate().toVector();

  double theta = pose.z();
  double x = pose.x();
  double y = pose.y();
  double ci = cos(theta);
  double si = sin(theta);

  double px = wp_.x();
  double py = wp_.y();

  double fx = cam_->focal_length_x_;
  double fy = cam_->focal_length_y_;
  double cx = cam_->principle_point_.x();
  double cy = cam_->principle_point_.y();

  double h = cam_height_;
  Eigen::Matrix<double, 2, 3> jacobian_proj;
  jacobian_proj(0, 0) = -si * fx / h;                                   // du/dx
  jacobian_proj(0, 1) = ci * fx / h;                                    // du/dy
  jacobian_proj(0, 2) = (ci * px + si * py - ci * x - si * y) * fx / h; // du/d_theta

  jacobian_proj(1, 0) = ci * fy / h;                                    // dv/dx
  jacobian_proj(1, 1) = si * fy / h;                                    // dv/dy
  jacobian_proj(1, 2) = (si * px - ci * py - si * x + ci * y) * fy / h; // dv/d_theta

  // u = x / h * fx + cx
  // v = y / h * fy + cy
  double u = (si * px - ci * py - si * x + ci * y) / h * fx + cx;
  double v = (baselink2cam_ - ci * px - si * py + ci * x + si * y) / h * fy + cy;
  Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

  jacobian_pixel_uv(0, 0) = (getPixelValue(u + 1, v) - getPixelValue(u - 1, v)) / 2;  // derror/du
  jacobian_pixel_uv(0, 1) = (getPixelValue(u, v + 1) - getPixelValue(u, v - 1)) / 2;  // derror/dv

  if (jacobian_pixel_uv(0, 0) == 0 || jacobian_pixel_uv(0, 1) == 0)
    this->setLevel(1);
  _jacobianOplusXi = coef_ * jacobian_pixel_uv * jacobian_proj;
}
}
