#ifndef G2O_VERTEX_SE2_H
#define G2O_VERTEX_SE2_H

#include "g2o/core/base_vertex.h"
#include "g2o/core/cache.h"
#include "se2.h"

namespace g2o {

/**
 * \brief 2D pose Vertex, (x,y,theta)
 */
class G2O_CORE_API VertexSE2 : public BaseVertex<3, SE2> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexSE2() {}

  virtual void setToOriginImpl() { _estimate = SE2(); }

  virtual void oplusImpl(const double *update) {
    Vector2D t = _estimate.translation();
    t += Eigen::Map<const Vector2D>(update);
    double angle = normalize_theta(_estimate.rotation().angle() + update[2]);
    _estimate.setTranslation(t);
    _estimate.setRotation(Eigen::Rotation2Dd(angle));
  }
  virtual bool read(std::istream &is) override {
    Vector3D p;
    is >> p[0] >> p[1] >> p[2];
    setEstimate(p);
    return true;
  }
  virtual bool write(std::ostream &os) const override {
    Vector3D p = estimate().toVector();
    os << p[0] << " " << p[1] << " " << p[2];
    return os.good();
  }
  void Clear() {
    _id = 0;
    _edges.clear();
    _graph = nullptr;
    if (_cacheContainer)
      delete (_cacheContainer);
    if (_userData)
      delete _userData;
    _hessianIndex = -1;
    _fixed = false;
    _marginalized = false;
    _colInHessian = -1;
  }
};

} // end namespace

#endif
