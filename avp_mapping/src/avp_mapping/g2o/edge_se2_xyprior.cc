#include "edge_se2_xyprior.h"

namespace g2o {

EdgeSE2XYPrior::EdgeSE2XYPrior()
    : BaseUnaryEdge<2, Vector2D, g2o::VertexSE2>() {}

bool EdgeSE2XYPrior::read(std::istream &is) { return false; }

bool EdgeSE2XYPrior::write(std::ostream &os) const { return false; }

void EdgeSE2XYPrior::linearizeOplus() { _jacobianOplusXi << 1, 0, 0, 0, 1, 0; }

} // end namespace
