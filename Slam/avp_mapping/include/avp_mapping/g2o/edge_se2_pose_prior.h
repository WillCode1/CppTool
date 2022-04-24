#ifndef G2O_EDGE_SE2_PRIOR_POSE_H
#define G2O_EDGE_SE2_PRIOR_POSE_H

#include "g2o/core/base_unary_edge.h"
#include "vertex_se2.h"
#include <opencv2/opencv.hpp>

namespace g2o {

class G2O_CORE_API EdgeSE2PosePrior
    : public BaseUnaryEdge<3, Vector3D, VertexSE2> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE2PosePrior();

  virtual void linearizeOplus();

  virtual bool read(std::istream &is);
  virtual bool write(std::ostream &os) const;

  virtual void reset() {
    if (_robustKernel) {
      delete _robustKernel;
      _robustKernel = nullptr;
    }

    _dimension = -1;
    _level = 0;
    _internalId = -1;
    _cacheIds.clear();
    _parameterTypes.clear();
    _parameters.clear();
    _parameterIds.clear();
    _vertices.clear();
    _vertices.resize(1);
    _id = -1;
  }

  virtual void computeError() {
    const VertexSE2 *v = static_cast<const VertexSE2 *>(_vertices[0]);
    _error = v->estimate().toVector() - _measurement;

    if (_error[2] > CV_PI) {
      _error[2] -= 2 * CV_PI;
    } else if (_error[2] < -CV_PI) {
      _error[2] += 2 * CV_PI;
    }
  }
};

} // end namespace

#endif
