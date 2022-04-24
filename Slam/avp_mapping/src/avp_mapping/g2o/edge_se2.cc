#include "edge_se2.h"
namespace g2o {

EdgeSE2::EdgeSE2() : BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2>() {}

bool EdgeSE2::read(std::istream &is) { return false; }

bool EdgeSE2::write(std::ostream &os) const { return false; }

void EdgeSE2::initialEstimate(const OptimizableGraph::VertexSet &from,
                              OptimizableGraph::Vertex * /* to */) {
  VertexSE2 *fromEdge = static_cast<VertexSE2 *>(_vertices[0]);
  VertexSE2 *toEdge = static_cast<VertexSE2 *>(_vertices[1]);
  if (from.count(fromEdge) > 0)
    toEdge->setEstimate(fromEdge->estimate() * _measurement);
  else
    fromEdge->setEstimate(toEdge->estimate() * _inverseMeasurement);
}

void EdgeSE2::linearizeOplus() {
  const VertexSE2 *vi = static_cast<const VertexSE2 *>(_vertices[0]);
  const VertexSE2 *vj = static_cast<const VertexSE2 *>(_vertices[1]);
  double thetai = vi->estimate().rotation().angle();

  Vector2D dt = vj->estimate().translation() - vi->estimate().translation();
  double si = sin(thetai), ci = cos(thetai);

  _jacobianOplusXi(0, 0) = -ci;
  _jacobianOplusXi(0, 1) = -si;
  _jacobianOplusXi(0, 2) = -si * dt.x() + ci * dt.y();
  _jacobianOplusXi(1, 0) = si;
  _jacobianOplusXi(1, 1) = -ci;
  _jacobianOplusXi(1, 2) = -ci * dt.x() - si * dt.y();
  _jacobianOplusXi(2, 0) = 0;
  _jacobianOplusXi(2, 1) = 0;
  _jacobianOplusXi(2, 2) = -1;

  _jacobianOplusXj(0, 0) = ci;
  _jacobianOplusXj(0, 1) = si;
  _jacobianOplusXj(0, 2) = 0;
  _jacobianOplusXj(1, 0) = -si;
  _jacobianOplusXj(1, 1) = ci;
  _jacobianOplusXj(1, 2) = 0;
  _jacobianOplusXj(2, 0) = 0;
  _jacobianOplusXj(2, 1) = 0;
  _jacobianOplusXj(2, 2) = 1;

  const SE2 &rmean = _inverseMeasurement;
  Matrix3D z = Matrix3D::Zero();
  z.block<2, 2>(0, 0) = rmean.rotation().toRotationMatrix();
  z(2, 2) = 1.;
  _jacobianOplusXi = z * _jacobianOplusXi;
  _jacobianOplusXj = z * _jacobianOplusXj;
}

} // end namespace
