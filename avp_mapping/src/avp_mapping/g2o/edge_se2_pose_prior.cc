#include "edge_se2_pose_prior.h"

namespace g2o {
g2o::EdgeSE2PosePrior::EdgeSE2PosePrior() {}

void EdgeSE2PosePrior::linearizeOplus() {
  _jacobianOplusXi << Eigen::Matrix3d::Identity();
}

bool EdgeSE2PosePrior::read(std::istream &is) { return false; }

bool EdgeSE2PosePrior::write(std::ostream &os) const { return false; }
}
