#pragma once

#include "g2o/core/base_multi_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/g2o_core_api.h"
#include "g2o/types/se3_ops.h"
#include "g2o/types/types_six_dof_expmap.h"
#include <iostream>

namespace g2o {
class G2O_CORE_API EquirectangularFullEdge final
    : public BaseMultiEdge<2, Vector2d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EquirectangularFullEdge();

  virtual bool read(std::istream & /*is*/);

  virtual bool write(std::ostream & /*os*/) const;

  virtual void reset();

  float getError();
  void computeError();
  void linearizeOplus();
  inline Vector2d cam_project(const Vector3d &pos_c) const;

public:
  Vector3d pw_;

private:
  float rad2pixel_;

  float img_low_bound_;
  Matrix<double, 6, 6> se3_camdext_adj_;
  SE3Quat se3_cam_ext_;
};
}
