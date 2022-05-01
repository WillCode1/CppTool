#pragma once

#include <Eigen/Core>
#include <chrono>
#include <iostream>
#include <queue>
#include <vector>

#include "colordef.h"
#include "frame.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/edge_se2.h"
#include "g2o/edge_se2_pose_prior.h"
#include "g2o/edge_se2_xyprior.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/vertex_se2.h"
#include "log.h"
#include "utils.h"

namespace SemanticSLAM
{
  // question: 什么功能
  class TrajectorySmoother
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    TrajectorySmoother() = delete;
    TrajectorySmoother(double odom_x_sigma, double odom_y_sigma, double odom_theta_sigma);
    ~TrajectorySmoother();
    bool Smoother(Frame *frame);
    void Reset();
    bool ScaleOdometrySigma(double scale);

  private:
    void InsertNewVertex(Frame *frame);
    void InsertVisualLocPriorEdge(Frame *frame);
    void InsertOdometryEdge(Frame *frame);
    void RemoveFrontestData();
    bool NeedFuse(Frame *frame);
    void OptimizeNormalFrame(Frame *frame);

  private:
    int current_id_;

    double odom_x_sigma_;
    double odom_y_sigma_;
    double odom_theta_sigma_;

    std::shared_ptr<g2o::SparseOptimizer> optimizer_;
    g2o::BlockSolverX::LinearSolverType *linearsolver_;
    g2o::BlockSolverX *solverptr_;
    g2o::OptimizationAlgorithmLevenberg *solver_;

    Eigen::Vector3d last_odometry_;
    std::queue<g2o::VertexSE2 *> vertex_queue_;
    std::queue<g2o::EdgeSE2 *> edge_se2_queue_;
    std::queue<g2o::EdgeSE2PosePrior *> edge_se2_prior_queue_;
    size_t max_vertex_;
  };
}
