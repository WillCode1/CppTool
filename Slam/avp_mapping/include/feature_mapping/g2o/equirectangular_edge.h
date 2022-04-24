#pragma once

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/g2o_core_api.h"
#include "g2o/types/se3_ops.h"
#include "g2o/types/types_six_dof_expmap.h"
#include <iostream>

namespace g2o {
class G2O_CORE_API EquirectangularEdge final
    : public BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EquirectangularEdge(SE3Quat mVertexSE3, Matrix<double, 6, 6> adj);

  virtual bool read(std::istream & /*is*/) {
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
  }

  virtual bool write(std::ostream & /*os*/) const {
    std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
    return false;
  }

  float getError();
  void linearizeOplus();

  void computeError() {
    const VertexSBAPointXYZ *pt3 = static_cast<const VertexSBAPointXYZ *>(
        _vertices[0]); // World Coordinate Points
    const VertexSE3Expmap *sw = static_cast<const VertexSE3Expmap *>(
        _vertices[1]); // Multi Camera System Pose
    Vector2d obs(_measurement);
    _error = obs -
             cam_project(se3_cam_ext_.map(sw->estimate().map(pt3->estimate())));
  }
  inline Vector2d cam_project(const Vector3d &pos_c) const {

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

private:
  float rad2pixel_;
  float img_low_bound_;
  Matrix<double, 6, 6> se3_camdext_adj_;
  SE3Quat se3_cam_ext_;
};
}
