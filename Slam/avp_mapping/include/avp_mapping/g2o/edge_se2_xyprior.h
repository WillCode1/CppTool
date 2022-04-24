#ifndef G2O_EDGE_SE2_PRIOR_XY_H
#define G2O_EDGE_SE2_PRIOR_XY_H

#include "g2o/core/base_unary_edge.h"
#include "vertex_se2.h"

namespace g2o {

class G2O_CORE_API EdgeSE2XYPrior
    : public BaseUnaryEdge<2, Vector2D, VertexSE2> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE2XYPrior();

  virtual bool setMeasurementData(const double *d) {
    _measurement[0] = d[0];
    _measurement[1] = d[1];
    return true;
  }

  virtual bool getMeasurementData(double *d) const {
    d[0] = _measurement[0];
    d[1] = _measurement[1];
    return true;
  }

  virtual int measurementDimension() const { return 2; }

  virtual void linearizeOplus();

  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;

  virtual void computeError() {
    const VertexSE2 *v = static_cast<const VertexSE2 *>(_vertices[0]);
    _error = v->estimate().translation() - _measurement;
  }
};

} // end namespace

#endif
