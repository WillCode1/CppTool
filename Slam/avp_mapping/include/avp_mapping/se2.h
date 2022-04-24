#ifndef G2O_SE2_H_
#define G2O_SE2_H_

#include "g2o/core/eigen_types.h"
#include "g2o/stuff/misc.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace g2o {

class SE2 {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SE2() : _R(0), _t(0, 0) {}

  SE2(const Isometry2D &iso) : _R(0), _t(iso.translation()) {
    _R.fromRotationMatrix(iso.linear());
  }

  SE2(const Vector3D &v) : _R(v[2]), _t(v[0], v[1]) {}

  SE2(double x, double y, double theta) : _R(theta), _t(x, y) {}

  //! translational component
  inline const Vector2D &translation() const { return _t; }
  void setTranslation(const Vector2D &t_) { _t = t_; }

  //! rotational component
  inline const Eigen::Rotation2Dd &rotation() const { return _R; }
  void setRotation(const Eigen::Rotation2Dd &R_) { _R = R_; }

  //! concatenate two SE2 elements (motion composition)
  inline SE2 operator*(const SE2 &tr2) const {
    SE2 result(*this);
    result *= tr2;
    return result;
  }

  //! motion composition operator
  inline SE2 &operator*=(const SE2 &tr2) {
    _t += _R * tr2._t;
    _R.angle() += tr2._R.angle();
    _R.angle() = normalize_theta(_R.angle());
    return *this;
  }

  //! project a 2D vector
  inline Vector2D operator*(const Vector2D &v) const { return _t + _R * v; }

  //! invert :-)
  inline SE2 inverse() const {
    SE2 ret;
    ret._R = _R.inverse();
    ret._R.angle() = normalize_theta(ret._R.angle());
    ret._t = ret._R * (_t * -1.);
    return ret;
  }

  inline double operator[](int i) const {
    assert(i >= 0 && i < 3);
    if (i < 2)
      return _t(i);
    return _R.angle();
  }

  //! assign from a 3D vector (x, y, theta)
  inline void fromVector(const Vector3D &v) { *this = SE2(v[0], v[1], v[2]); }

  //! convert to a 3D vector (x, y, theta)
  inline Vector3D toVector() const {
    return Vector3D(_t.x(), _t.y(), _R.angle());
  }

  inline Isometry2D toIsometry() const {
    Isometry2D iso = Isometry2D::Identity();
    iso.linear() = _R.toRotationMatrix();
    iso.translation() = _t;
    return iso;
  }

protected:
  Eigen::Rotation2Dd _R;
  Vector2D _t;
};

} // end namespace

#endif
