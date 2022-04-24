#pragma once

#include "camera_config.h"
#include "camera_parameter.h"
#include "frame.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/edge_se2_pose_prior.h"
#include "g2o/edge_se2_semantic_project.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/vertex_se2.h"
#include "keyframe.h"
#include "system_config.h"

#include "utils.h"
#include <Eigen/Dense>
namespace SemanticSLAM {
class Optimizer {
public:
  Optimizer();
  ~Optimizer();

  Optimizer(const Optimizer &) = delete;
  Optimizer(const Optimizer &&) = delete;
  Optimizer &operator=(const Optimizer &) = delete;
  Optimizer &operator=(const Optimizer &&) = delete;

  static Optimizer &GetInstance();

  bool OptimizeFramePose(KeyFrame *last_keyframe, Frame &current_frame,
                         const CameraConfig &camera_config);

private:
  g2o::EdgeSE2PosePrior *CreatePosePriorEdge(const Vec3_t &trans_lc_vector);
  g2o::EdgeSE2SemanticProject *GetEdgeResource();

  void ResetResource();

private:
  g2o::SparseOptimizer *optimizer_;
  g2o::VertexSE2 *vse2_;
  std::unordered_map<g2o::EdgeSE2SemanticProject *, bool> edge_resources_;
  static std::mutex global_optimizer_mutex_;
};
}
