#include "optimizer/struct_bundle_adjuster.h"
#include "colordef.h"
#include "converter.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/equirectangular_edge.h"
#include "g2o/equirectangular_struct_edge.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/types/types_six_dof_expmap.h"
namespace FeatureSLAM {
StructBundleAdjuster::StructBundleAdjuster(unsigned int num_first_iter,
                                           unsigned int num_second_iter)
    : num_first_iter_(num_first_iter), num_second_iter_(num_second_iter) {}

void StructBundleAdjuster::Optimize(MCKeyFrame *current_mc_keyfrm,
                                    Map *map) const {
  Timer timer("Struct only bundle adjustment ");

  std::list<MCKeyFrame *> local_mc_keyfrms;
  local_mc_keyfrms.push_back(current_mc_keyfrm);
  current_mc_keyfrm->local_ba_for_keyfrm_ = current_mc_keyfrm->id_;

  const std::vector<MCKeyFrame *> ngh_mc_keyfrms =
      current_mc_keyfrm->GetVectorCovisibleKeyFrames();

  for (auto ngh_mc_keyfrm : ngh_mc_keyfrms) {
    if (ngh_mc_keyfrm->local_ba_for_keyfrm_ == current_mc_keyfrm->id_) {
      continue;
    }
    ngh_mc_keyfrm->local_ba_for_keyfrm_ = current_mc_keyfrm->id_;
    local_mc_keyfrms.push_back(ngh_mc_keyfrm);
  }

  std::list<MapPoint *> local_mps;

  for (auto local_mc_keyfrm : local_mc_keyfrms) {

    for (int cam_index = 0; cam_index < 4; cam_index++) {
      std::vector<MapPoint *> mps =
          local_mc_keyfrm->keyframes_[cam_index]->GetMapPointMatches();
      for (auto mp : mps) {

        if (!mp)
          continue;

        if (mp->isBad())
          continue;

        if (mp->local_ba_keyfrm_id_ != current_mc_keyfrm->id_) {
          local_mps.push_back(mp);
          mp->local_ba_keyfrm_id_ = current_mc_keyfrm->id_;
        }
      }
    }
  }

  // Find fixed Keyframes, Keyframes that see Local MapPoints but that are not
  // Local Keyframes

  std::list<MCKeyFrame *> fixed_mc_keyfrms;

  for (auto mp : local_mps) {
    std::map<KeyFrame *, size_t> observations = mp->GetObservations();

    for (auto obs_iter : observations) {
      KeyFrame *obs_keyfrm = obs_iter.first;

      MCKeyFrame *obs_mc_keyfrm = obs_keyfrm->GetMCKeyFrame();

      if (obs_mc_keyfrm->local_ba_for_keyfrm_ != current_mc_keyfrm->id_ &&
          obs_mc_keyfrm->ba_fixed_for_keyfrm_ != current_mc_keyfrm->id_) {
        obs_mc_keyfrm->ba_fixed_for_keyfrm_ = current_mc_keyfrm->id_;
        fixed_mc_keyfrms.push_back(obs_mc_keyfrm);
      }
    }
  }

#ifdef ENABLE_VIEWER

  std::vector<MCKeyFrame *> ba_mc_keyfrms(local_mc_keyfrms.begin(),
                                          local_mc_keyfrms.end());
  std::vector<MCKeyFrame *> ba_fixed_keyfrms(fixed_mc_keyfrms.begin(),
                                             fixed_mc_keyfrms.end());
  map->SetBaKeyFrame(ba_mc_keyfrms, ba_fixed_keyfrms);

#endif

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  std::map<int, g2o::SE3Quat> index_mc_keyfrm_pose;

  // Add Local MC Keyframes
  for (auto mc_keyfrm : local_mc_keyfrms) {
    auto mc_keyfrm_pose = Converter::toSE3Quat(mc_keyfrm->GetPose());
    index_mc_keyfrm_pose[mc_keyfrm->id_] = mc_keyfrm_pose;
  }

  // Add Local MC Keyframes
  for (auto mc_keyfrm : fixed_mc_keyfrms) {
    auto mc_keyfrm_pose = Converter::toSE3Quat(mc_keyfrm->GetPose());
    index_mc_keyfrm_pose[mc_keyfrm->id_] = mc_keyfrm_pose;
  }

  MultiCam *multicam = current_mc_keyfrm->multicam_;
  const int expected_edge_num = local_mps.size();

  std::vector<g2o::EquirectangularStructEdge *> edge_equ_project;
  edge_equ_project.reserve(expected_edge_num);
  std::vector<KeyFrame *> edge_keyfrm;
  edge_keyfrm.reserve(expected_edge_num);
  vector<MapPoint *> edge_map_point;
  edge_map_point.reserve(expected_edge_num);

  constexpr float chi_sq_2D = 5.99146;

  for (auto mp : local_mps) {

    g2o::VertexSBAPointXYZ *vertex_point = new g2o::VertexSBAPointXYZ();
    vertex_point->setEstimate(Converter::toVector3d(mp->GetWorldPos()));

    int id = mp->id_;
    vertex_point->setId(id);
    vertex_point->setMarginalized(true);
    optimizer.addVertex(vertex_point);

    const std::map<KeyFrame *, size_t> observations = mp->GetObservations();

    for (auto obs_iter : observations) {
      KeyFrame *keyfrm = obs_iter.first;

      if (keyfrm->isBad())
        continue;

      MCKeyFrame *mc_keyfrm = keyfrm->GetMCKeyFrame();

      if (!mc_keyfrm) {
        std::cout << kColorRed << " impossible  " << kColorReset << std::endl;
        continue;
      }

      if (index_mc_keyfrm_pose.count(mc_keyfrm->id_) == 0) {
        std::cout << kColorRed << " missing mc keyframe , impossible "
                  << kColorReset << std::endl;
        continue;
      }

      const int kCamIndex = keyfrm->camera_index_;

      const cv::KeyPoint &kp_pt = keyfrm->keypts_[obs_iter.second];
      Vec2_t obs(kp_pt.pt.x, kp_pt.pt.y);

      g2o::EquirectangularStructEdge *e = new g2o::EquirectangularStructEdge(
          index_mc_keyfrm_pose[mc_keyfrm->id_],
          multicam->se3quat_cam_extrinsic_[kCamIndex],
          multicam->se3_cam_extrinsic_adj_[kCamIndex]);

      e->setMeasurement(obs);
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex(id)));

      const float &inv_sigma2 = keyfrm->inv_level_sigma2_[kp_pt.octave];
      e->setInformation(Eigen::Matrix2d::Identity() * inv_sigma2);
      g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
      e->setRobustKernel(rk);
      rk->setDelta(chi_sq_2D);
      optimizer.addEdge(e);
      edge_equ_project.push_back(e);

      edge_keyfrm.push_back(keyfrm);
      edge_map_point.push_back(mp);
    }
  }

  optimizer.initializeOptimization();
  optimizer.optimize(num_first_iter_);

  bool run_robust_BA = true;

  if (run_robust_BA)

  {
    for (size_t i = 0; i < edge_equ_project.size(); i++) {
      auto e = edge_equ_project[i];
      auto mp = edge_map_point[i];

      if (mp->isBad())
        continue;
      if (e->chi2() > chi_sq_2D) {
        e->setLevel(1);
      }
      // Do do use robust kernel
      e->setRobustKernel(nullptr);
    }

    optimizer.initializeOptimization(0);
    optimizer.optimize(num_second_iter_);
  }

  std::vector<std::pair<KeyFrame *, MapPoint *>> outlier_observations;
  outlier_observations.reserve(edge_equ_project.size());
  for (size_t i = 0; i < edge_equ_project.size(); i++) {
    auto e = edge_equ_project[i];
    auto mp = edge_map_point[i];
    if (mp->isBad())
      continue;

    if (e->chi2() > chi_sq_2D) {
      auto keyfrm = edge_keyfrm[i];
      outlier_observations.push_back(std::make_pair(keyfrm, mp));
    }
  }

  // mutexMapUpdate is lock in tracking::track()
  //  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

  if (!outlier_observations.empty()) {
    for (auto outlier_iter : outlier_observations) {
      auto keyfrm = outlier_iter.first;
      auto mp = outlier_iter.second;
      keyfrm->EraseMapPointMatch(mp);
      mp->EraseObservation(keyfrm);
    }
  }

  for (auto mp : local_mps) {
    g2o::VertexSBAPointXYZ *vertex_point =
        static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(mp->id_));

    mp->SetWorldPos(Converter::toCvMat(vertex_point->estimate()));
    mp->UpdateNormalAndDepth();
  }

  std::cout << kColorGreen;
  timer.Print();
  std::cout << kColorReset;
}
}
