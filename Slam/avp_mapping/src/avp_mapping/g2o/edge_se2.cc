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

/*
         ci,  si,  xi
  vi = [-si,  ci,  yi]
          0,   0,  1
  _error = _inverseMeasurement * vi.inverse() * vj;

  vi.inv = (ti*Ri, -Ri)

  Ri((xi,yi)+Rj(xj,yj)) = Ri(xi+Rj*xj, yi+Rj*yj)
  vi.inv*vj = (Ri*(ti+Rj*tj), Rj-Ri) = (, , Rj-Ri)

  inline SE2 inverse() const {
    SE2 ret;
    ret._R = _R.inverse();
    ret._R.angle() = normalize_theta(ret._R.angle());
    ret._t = ret._R * (_t * -1.);
    return ret;
  }
  inline SE2 &operator*=(const SE2 &tr2) {
    _t += _R * tr2._t;
    _R.angle() += tr2._R.angle();
    _R.angle() = normalize_theta(_R.angle());
    return *this;
  }
 */
/* 李代数SE2 error(x,y,theta)对VertexSE2 vi(x,y,theta)求导, jacob是3x3矩阵 */
void EdgeSE2::linearizeOplus() {
  const VertexSE2 *vi = static_cast<const VertexSE2 *>(_vertices[0]);
  const VertexSE2 *vj = static_cast<const VertexSE2 *>(_vertices[1]);
  double thetai = vi->estimate().rotation().angle();

  Vector2D dt = vj->estimate().translation() - vi->estimate().translation();  // 平移增量
  double si = sin(thetai), ci = cos(thetai);

  _jacobianOplusXi(0, 0) = -ci;  _jacobianOplusXi(0, 1) = -si;  _jacobianOplusXi(0, 2) = -si * dt.x() + ci * dt.y();
  _jacobianOplusXi(1, 0) = si;  _jacobianOplusXi(1, 1) = -ci;  _jacobianOplusXi(1, 2) = -ci * dt.x() - si * dt.y();
  _jacobianOplusXi(2, 0) = 0;  _jacobianOplusXi(2, 1) = 0;  _jacobianOplusXi(2, 2) = -1;

  _jacobianOplusXj(0, 0) = ci;  _jacobianOplusXj(0, 1) = si;  _jacobianOplusXj(0, 2) = 0;
  _jacobianOplusXj(1, 0) = -si;  _jacobianOplusXj(1, 1) = ci;  _jacobianOplusXj(1, 2) = 0;
  _jacobianOplusXj(2, 0) = 0;  _jacobianOplusXj(2, 1) = 0;  _jacobianOplusXj(2, 2) = 1;

  const SE2 &rmean = _inverseMeasurement;
  Matrix3D z = Matrix3D::Zero();
  z.block<2, 2>(0, 0) = rmean.rotation().toRotationMatrix();
  z(2, 2) = 1.;
  _jacobianOplusXi = z * _jacobianOplusXi;
  _jacobianOplusXj = z * _jacobianOplusXj;
}

} // end namespace
