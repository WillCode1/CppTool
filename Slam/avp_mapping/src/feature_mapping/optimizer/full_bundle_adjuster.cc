#include "optimizer/full_bundle_adjuster.h"
#include "colordef.h"
#include "converter.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/equirectangular_edge.h"
#include "g2o/equirectangular_full_edge.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/types/types_six_dof_expmap.h"
namespace FeatureSLAM {

FullBundleAdjuster::FullBundleAdjuster(Map *map, unsigned int iterations,
                                       MultiCam *multicam)
    : map_(map), num_iter_(iterations), multicam_(multicam) {}

void FullBundleAdjuster::Optimize() const {
  std::vector<MCKeyFrame *> mckeyfrms = map_->GetAllMCKeyFrames();

  std::vector<MapPoint *> mps = map_->GetAllMapPoints();

  vector<bool> not_included_mps;
  not_included_mps.resize(mps.size());
  std::cout << kColorYellow << "  Run Bundle Adjustment " << kColorReset
            << std::endl;

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  long unsigned int max_vertex_id = 0;
  std::set<unsigned long int> pose_vertex_id;

  // add extrinisc vertex

  for (auto mckeyfrm : mckeyfrms) {

    g2o::VertexSE3Expmap *pose_vertex_se3 = new g2o::VertexSE3Expmap();
    pose_vertex_se3->setEstimate(Converter::toSE3Quat(mckeyfrm->GetPose()));
    pose_vertex_se3->setId(mckeyfrm->id_);
    pose_vertex_se3->setFixed(mckeyfrm->id_ == 0);
    pose_vertex_id.insert(mckeyfrm->id_);

    optimizer.addVertex(pose_vertex_se3);
    if (mckeyfrm->id_ > max_vertex_id) {
      max_vertex_id = mckeyfrm->id_;
    }
  }

  MultiCam *multi_cam = multicam_;

  const unsigned long int kMaxPoseId = max_vertex_id;

  for (int cam_index = 0; cam_index < 4; cam_index++) {
    g2o::VertexSE3Expmap *pose_vertex_se3 = new g2o::VertexSE3Expmap();
    pose_vertex_se3->setEstimate(
        Converter::toSE3Quat(multi_cam->cam_extrinsic_[cam_index]));
    long unsigned int id = kMaxPoseId + 1 + cam_index;
    pose_vertex_se3->setId(id);
    pose_vertex_se3->setFixed(cam_index == 1);
    pose_vertex_id.insert(id);

    optimizer.addVertex(pose_vertex_se3);
    if (id > max_vertex_id) {
      max_vertex_id = id;
    }
  }

  //  const float thHuber2D = sqrt(3.99);
  constexpr float chi_sqr_2d = 5.99146;

  std::vector<g2o::EquirectangularFullEdge *> edges;
  for (size_t i = 0; i < mps.size(); i++) {
    MapPoint *mp = mps[i];
    if (mp->isBad())
      continue;
    g2o::VertexSBAPointXYZ *point_vertex_xyz = new g2o::VertexSBAPointXYZ();
    point_vertex_xyz->setEstimate(Converter::toVector3d(mp->GetWorldPos()));
    const int id = mp->id_ + max_vertex_id + 1;
    point_vertex_xyz->setId(id);

    // use marginalized to speed up optimization
    point_vertex_xyz->setMarginalized(true);
    optimizer.addVertex(point_vertex_xyz);

    const std::map<KeyFrame *, size_t> observations = mp->GetObservations();

    int edge_num = 0;

    // Add edges

    for (auto obs_iter : observations) {
      auto keyfrm = obs_iter.first;

      auto mckeyfrm = keyfrm->GetMCKeyFrame();

      if (!mckeyfrm || mckeyfrm->id_ > kMaxPoseId)
        continue;

      const cv::KeyPoint &kp_pt = keyfrm->keypts_[obs_iter.second];
      Vec2_t obs(kp_pt.pt.x, kp_pt.pt.y);
      const int cam_index = keyfrm->camera_index_;

      g2o::EquirectangularFullEdge *e = new g2o::EquirectangularFullEdge();
      e->setMeasurement(obs);

      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex(id)));
      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex(mckeyfrm->id_)));
      e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex(kMaxPoseId + 1 + cam_index)));

      const float &inv_sigma2 = keyfrm->inv_level_sigma2_[kp_pt.octave];
      e->setInformation(Eigen::Matrix2d::Identity() * inv_sigma2);
      g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
      e->setRobustKernel(rk);
      rk->setDelta(chi_sqr_2d);
      optimizer.addEdge(e);
      edges.push_back(e);
      edge_num++;
    }

    if (edge_num == 0) {
      optimizer.removeVertex(point_vertex_xyz);
      not_included_mps[i] = true;
    } else {
      not_included_mps[i] = false;
    }
  }

  float error_sum = 0;
  for (size_t i = 0; i < edges.size(); i++) {
    error_sum += edges[i]->getError();
  }
  std::cout << kColorGreen << "before  full bundle adjustment optimization "
            << "total error : " << kColorYellow << error_sum << kColorReset
            << kColorReset << std::endl;

  // Start Optimization

  optimizer.initializeOptimization();
  optimizer.optimize(num_iter_);

  float error_sum_after = 0;
  for (size_t i = 0; i < edges.size(); i++) {
    error_sum_after += edges[i]->getError();
  }
  std::cout << kColorGreen << "after full bundle adjustment optimization "
            << "total error : " << kColorYellow << error_sum_after
            << kColorReset << std::endl;

  //  Update MCKeyframes' pose
  for (auto mckeyfrm : mckeyfrms) {

    if (!pose_vertex_id.count(mckeyfrm->id_))
      continue;

    g2o::VertexSE3Expmap *vertex_pose_se3 =
        static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(mckeyfrm->id_));
    g2o::SE3Quat SE3quat = vertex_pose_se3->estimate();
    mckeyfrm->SetPose(Converter::toCvMat(SE3quat));
  }

  // update extrinsic

  for (int cam_index = 0; cam_index < 4; cam_index++) {

    g2o::VertexSE3Expmap *vertex_ext_se3 = static_cast<g2o::VertexSE3Expmap *>(
        optimizer.vertex(kMaxPoseId + 1 + cam_index));
    g2o::SE3Quat SE3quat = vertex_ext_se3->estimate();

    std::cout << " new ext for cam (" << cam_index << "_" << std::endl
              << Converter::toCvMat(SE3quat) << std::endl;
    std::cout << " old ext" << multi_cam->cam_extrinsic_[cam_index]
              << std::endl;
  }

  //  Update Points' position
  for (size_t i = 0; i < mps.size(); i++) {
    if (not_included_mps[i])
      continue;

    MapPoint *mp = mps[i];

    if (mp->isBad())
      continue;
    g2o::VertexSBAPointXYZ *vertex_point =
        static_cast<g2o::VertexSBAPointXYZ *>(
            optimizer.vertex(mp->id_ + max_vertex_id + 1));
    mp->SetWorldPos(Converter::toCvMat(vertex_point->estimate()));
    mp->UpdateNormalAndDepth();
  }
}
}
