#pragma once

#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/g2o_core_api.h"
#include "g2o/types/se3_ops.h"
#include "g2o/types/types_six_dof_expmap.h"
#include <iostream>

namespace g2o {
class G2O_CORE_API EquirectangularStructEdge final
    : public BaseUnaryEdge<2, Vector2d, VertexSBAPointXYZ> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EquirectangularStructEdge(SE3Quat sys_pose, SE3Quat mVertexSE3,
                            Matrix<double, 6, 6> adj);

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

  void computeError();
  inline Vector2d cam_project(const Vector3d &pos_c) const;

private:
  float rad2pixel_;
  float img_low_bound_;
  Matrix<double, 6, 6> se3_camdext_adj_;
  SE3Quat se3_cam_ext_;
  SE3Quat se3_sys_pose_;
};
}
