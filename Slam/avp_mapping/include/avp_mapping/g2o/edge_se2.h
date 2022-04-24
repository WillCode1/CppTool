#ifndef G2O_EDGE_SE2_H
#define G2O_EDGE_SE2_H

#include "g2o/core/base_binary_edge.h"
#include "vertex_se2.h"

namespace g2o {

class G2O_CORE_API EdgeSE2
    : public BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE2();

  void computeError() {
    const VertexSE2 *v1 = static_cast<const VertexSE2 *>(_vertices[0]);
    const VertexSE2 *v2 = static_cast<const VertexSE2 *>(_vertices[1]);
    SE2 delta =
        _inverseMeasurement * (v1->estimate().inverse() * v2->estimate());
    _error = delta.toVector();
  }
  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;

  virtual void setMeasurement(const SE2 &m) {
    _measurement = m;
    _inverseMeasurement = m.inverse();
  }

  virtual bool setMeasurementData(const double *d) {
    _measurement = SE2(d[0], d[1], d[2]);
    _inverseMeasurement = _measurement.inverse();
    return true;
  }

  virtual bool getMeasurementData(double *d) const {
    Vector3D v = _measurement.toVector();
    d[0] = v[0];
    d[1] = v[1];
    d[2] = v[2];
    return true;
  }

  virtual int measurementDimension() const { return 3; }

  virtual bool setMeasurementFromState() {
    const VertexSE2 *v1 = static_cast<const VertexSE2 *>(_vertices[0]);
    const VertexSE2 *v2 = static_cast<const VertexSE2 *>(_vertices[1]);
    _measurement = v1->estimate().inverse() * v2->estimate();
    _inverseMeasurement = _measurement.inverse();
    return true;
  }

  virtual double initialEstimatePossible(const OptimizableGraph::VertexSet &,
                                         OptimizableGraph::Vertex *) {
    return 1.;
  }
  virtual void initialEstimate(const OptimizableGraph::VertexSet &from,
                               OptimizableGraph::Vertex *to);
  virtual void linearizeOplus();

protected:
  SE2 _inverseMeasurement;
};

} // end namespace

#endif
