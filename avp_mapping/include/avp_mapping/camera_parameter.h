#ifndef G2O_CAMERA_PARAMETER_H
#define G2O_CAMERA_PARAMETER_H
#include "g2o/core/eigen_types.h"
#include "g2o/core/parameter.h"
#include <Eigen/Geometry>

namespace g2o {
class G2O_CORE_API CameraParameters : public g2o::Parameter {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CameraParameters();

  CameraParameters(double focal_length_x, double focal_length_y,
                   const Vector2D &principle_point)
      : focal_length_x_(focal_length_x), focal_length_y_(focal_length_y),
        principle_point_(principle_point) {}

  Vector2D cam_map(const Vector3D &trans_xyz) const;

  virtual bool read(std::istream &is) {
    is >> focal_length_x_;
    is >> focal_length_y_;
    is >> principle_point_[0];
    is >> principle_point_[1];
    return true;
  }

  virtual bool write(std::ostream &os) const {
    os << focal_length_x_ << " " << focal_length_y_ << " ";
    os << principle_point_.x() << " ";
    os << principle_point_.y() << " ";
    return true;
  }

  double focal_length_x_;
  double focal_length_y_;
  Vector2D principle_point_;
};
}

#endif
