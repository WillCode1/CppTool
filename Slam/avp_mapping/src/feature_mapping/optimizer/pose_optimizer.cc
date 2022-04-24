#include "optimizer/pose_optimizer.h"
#include "colordef.h"
#include "converter.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/equirectangular_pose_edge.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_seven_dof_expmap.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/types/types_six_dof_multicam.h"
#include "multicam_ext.h"
#include <Eigen/StdVector>

#include <fstream>
#include <mutex>

namespace FeatureSLAM {

PoseOptimizer::PoseOptimizer(const unsigned int num_trials,
                             const unsigned int num_each_iter)
    : num_trials_(num_trials), num_each_iter_(num_each_iter) {}

int PoseOptimizer::PoseOptimizationMC(MCFrame *mc_frm) const {

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  int num_init_obs = 0;

  g2o::VertexSE3Expmap *frm_vtx = new g2o::VertexSE3Expmap();
  frm_vtx->setEstimate(Converter::toSE3Quat(mc_frm->tcw_));
  frm_vtx->setId(0);
  frm_vtx->setFixed(false);
  optimizer.addVertex(frm_vtx);

  const unsigned int num_keypts = mc_frm->frames_[LEFT_CAMERA].num_feature_ +
                                  mc_frm->frames_[FRONT_CAMERA].num_feature_ +
                                  mc_frm->frames_[RIGHT_CAMERA].num_feature_ +
                                  mc_frm->frames_[BACK_CAMERA].num_feature_;

  std::vector<g2o::EquirectangularPoseEdge *> pose_opt_edges;
  pose_opt_edges.reserve(num_keypts);

  std::vector<int> edge_cam_index;
  edge_cam_index.reserve(num_keypts);

  std::vector<size_t> edge_keypt_index;
  edge_keypt_index.reserve(num_keypts);

  auto multi_cam = mc_frm->multicam_;

  constexpr float chi_sq_2D = 5.99146;

  const int kCamNum = 4;
  for (int cam_index = 0; cam_index < kCamNum; cam_index++) {
    const int cam_kpt_num = mc_frm->frames_[cam_index].num_feature_;

    for (int i = 0; i < cam_kpt_num; i++) {
      auto mp = mc_frm->frames_[cam_index].mappoints_[i];
      if (!mp)
        continue;
      num_init_obs++;
      cv::Mat pos = mp->GetWorldPos();
      mc_frm->frames_[cam_index].outliers_[i] = false;
      const cv::KeyPoint &kp_pt = mc_frm->frames_[cam_index].keypts_[i];
      Vec2_t obs(kp_pt.pt.x, kp_pt.pt.y);

      g2o::EquirectangularPoseEdge *e = new g2o::EquirectangularPoseEdge(
          multi_cam->se3quat_cam_extrinsic_[cam_index],
          multi_cam->se3_cam_extrinsic_adj_[cam_index]);
      e->setMeasurement(obs);
      e->pw_ = Converter::toVector3d(pos);
      const float inv_sigma2 =
          mc_frm->frames_[cam_index].inv_level_sigma2_[kp_pt.octave];
      e->setInformation(Eigen::Matrix2d::Identity() * inv_sigma2);

      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex(0)));
      g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
      e->setRobustKernel(rk);
      rk->setDelta(chi_sq_2D);
      optimizer.addEdge(e);
      pose_opt_edges.push_back(e);
      edge_keypt_index.push_back(i);
      edge_cam_index.push_back(cam_index);
    }
  }

  if (num_init_obs < 5) {
    std::cout << kColorRed << " too few  obs " << kColorReset << std::endl;
    return 0;
  }

  unsigned int num_bad_obs = 0;
  for (unsigned int trial = 0; trial < num_trials_; trial++) {

    // reset pose initial value, which is different in openvslam

    frm_vtx->setEstimate(Converter::toSE3Quat(mc_frm->tcw_));
    optimizer.initializeOptimization(0);
    optimizer.optimize(num_each_iter_);
    num_bad_obs = 0;

    for (size_t i = 0; i < pose_opt_edges.size(); i++) {
      auto edge = pose_opt_edges[i];
      int cam_index = edge_cam_index[i];
      const size_t idx = edge_keypt_index[i];

      if (mc_frm->frames_[cam_index].outliers_[idx]) {
        edge->computeError();
      }

      // Re-projection error is too large
      if (edge->chi2() > chi_sq_2D) {
        mc_frm->frames_[cam_index].outliers_[idx] = true;
        edge->setLevel(1);
        num_bad_obs++;
      } else {
        mc_frm->frames_[cam_index].outliers_[idx] = false;
        edge->setLevel(0);
      }

      if (trial == num_trials_ - 2) {
        edge->setRobustKernel(0);
      }

      if (num_init_obs - num_bad_obs < 5) {
        std::cout << kColorRed << " too few inliers " << kColorReset
                  << std::endl;
        break;
      }
    }
  }

  g2o::VertexSE3Expmap *pose_se3_vtx =
      static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
  cv::Mat tcw = Converter::toCvMat(pose_se3_vtx->estimate());

  mc_frm->SetPose(tcw);
  mc_frm->UpdatePose();

  return num_init_obs - num_bad_obs;
}

int PoseOptimizer::PoseOptimization(Frame *frame) const {

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

  linearSolver =
      new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  int num_init_obs = 0;
  g2o::VertexSE3Expmap *frm_vtx = new g2o::VertexSE3Expmap();
  frm_vtx->setEstimate(Converter::toSE3Quat(frame->Tcw_));
  frm_vtx->setId(0);
  frm_vtx->setFixed(false);
  optimizer.addVertex(frm_vtx);

  // Set MapPoint vertices
  const int num_keypts = frame->num_feature_;

  // for Monocular
  vector<g2o::EquirectangularPoseEdge *> pose_opt_edges;
  vector<size_t> edge_keypt_index;
  pose_opt_edges.reserve(num_keypts);
  edge_keypt_index.reserve(num_keypts);

  cv::Mat cam_ext = cv::Mat::eye(4, 4, CV_32FC1);
  auto cam_ext_se3quat = Converter::toSE3Quat(cam_ext);
  auto cam_ext_adj = cam_ext_se3quat.adj();

  constexpr float chi_sq_2D = 5.99146;
  {
    //    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < num_keypts; i++) {
      MapPoint *mp = frame->mappoints_[i];
      if (!mp)
        continue;
      num_init_obs++;
      frame->outliers_[i] = false;
      const cv::KeyPoint &kp_pt = frame->keypts_[i];
      Vec2_t obs(kp_pt.pt.x, kp_pt.pt.y);

      g2o::EquirectangularPoseEdge *e =
          new g2o::EquirectangularPoseEdge(cam_ext_se3quat, cam_ext_adj);
      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
                          optimizer.vertex(0)));
      e->setMeasurement(obs);
      cv::Mat pos = mp->GetWorldPos();

      e->pw_ = Converter::toVector3d(pos);
      const float inv_sigma2 = frame->inv_level_sigma2_[kp_pt.octave];
      e->setInformation(Eigen::Matrix2d::Identity() * inv_sigma2);
      g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
      e->setRobustKernel(rk);
      rk->setDelta(chi_sq_2D);
      optimizer.addEdge(e);
      pose_opt_edges.push_back(e);
      edge_keypt_index.push_back(i);
    }
  }

  if (num_init_obs < 5)
    return 0;

  optimizer.initializeOptimization();

  int num_bad_obs = 0;
  for (size_t trial = 0; trial < num_trials_; trial++) {

    frm_vtx->setEstimate(Converter::toSE3Quat(frame->Tcw_));
    optimizer.initializeOptimization(0);
    optimizer.optimize(num_each_iter_);

    num_bad_obs = 0;
    for (size_t i = 0; i < pose_opt_edges.size(); i++) {
      g2o::EquirectangularPoseEdge *e = pose_opt_edges[i];

      const size_t idx = edge_keypt_index[i];

      if (frame->outliers_[idx]) {
        e->computeError();
      }
      const float chi2 = e->chi2();
      if (chi2 > chi_sq_2D) {
        frame->outliers_[idx] = true;
        e->setLevel(1);
        num_bad_obs++;
      } else {
        frame->outliers_[idx] = false;
        e->setLevel(0);
      }

      if (trial == num_trials_ - 2)
        e->setRobustKernel(0);
    }
    if (num_init_obs - num_bad_obs < 5)
      break;
  }

  // Recover optimized pose and return number of inliers
  g2o::VertexSE3Expmap *pose_se3_vtx =
      static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
  cv::Mat tcw = Converter::toCvMat(pose_se3_vtx->estimate());
  frame->SetPose(tcw);
  return num_init_obs - num_bad_obs;
}

// void Optimizer::OptimizeEssentialGraph(
//    Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
//    const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
//    const LoopClosing::KeyFrameAndPose &CorrectedSim3,
//    const map<KeyFrame *, set<KeyFrame *>> &LoopConnections,
//    const bool &bFixScale) {
//  // Setup optimizer
//  // 步骤1：构造优化器

//  if (!pCurKF->mbGotMCConnected || !pLoopKF->mbGotMCConnected) {
//    cout << " ######### Fatal error ,Curent KF  or Matched KF  don't have "
//            "MCkeyframe pointer "
//         << endl;
//    return;
//  }
//  const int nMatchedKFCamIndex = pLoopKF->camera_index_;
//  const int nCurrentKFCamIndex = pCurKF->camera_index_;
//  bool bSameCam = (nMatchedKFCamIndex == nCurrentKFCamIndex);
//  std::cout << " #####  Loop Found with same cam? : " << bSameCam << endl;

//  const int nFixed =
//      pLoopKF->GetMCKeyFrameConnection()->keyframes_[FRONT_CAMERA]->mnId;

//  //  Relative Extrinsic Parameters

//  MCKeyFrame *pMCKFCur = pCurKF->GetMCKeyFrameConnection();
//  cv::Mat mCO = pMCKFCur->cam_exts_[nCurrentKFCamIndex];
//  cv::Mat mSim3FC = pMCKFCur->cam_exts_[FRONT_CAMERA] * mCO.inv();
//  cv::Mat RFC = mSim3FC.rowRange(0, 3).colRange(0, 3);
//  cv::Mat tFC = mSim3FC.rowRange(0, 3).col(3);
//  g2o::Sim3 g2oSFC(Converter::toMatrix3d(RFC), Converter::toVector3d(tFC),
//  1.0);

//  cv::Mat mMO = pMCKFCur->cam_exts_[nMatchedKFCamIndex];
//  cv::Mat mSim3FM = pMCKFCur->cam_exts_[FRONT_CAMERA] * mMO.inv();
//  cv::Mat RFM = mSim3FM.rowRange(0, 3).colRange(0, 3);
//  cv::Mat tFM = mSim3FM.rowRange(0, 3).col(3);
//  g2o::Sim3 g2oSFM(Converter::toMatrix3d(RFM), Converter::toVector3d(tFM),
//  1.0);

//  g2o::SparseOptimizer optimizer;
//  optimizer.setVerbose(false);
//  g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
//      new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
//  g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
//  g2o::OptimizationAlgorithmLevenberg *solver =
//      new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

//  solver->setUserLambdaInit(1e-16);
//  optimizer.setAlgorithm(solver);

//  const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
//  const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

//  const unsigned int nMaxKFid = pMap->GetMaxKFid();

//  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
//  vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(
//      nMaxKFid + 1);

//  const int minFeat = 100;

//  set<unsigned long int> sg2oVertexId;

//  // %  Add Vertex

//  for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
//    KeyFrame *pKF = vpKFs[i];
//    if (pKF->isBad())
//      continue;
//    //        if(pKF->mnCamIndex!= nCurrentKFCamIndex)
//    if (pKF->camera_index_ !=
//        FRONT_CAMERA) // pose graph only cantain front camera pose
//      continue;
//    if (!pKF->mbGotMCConnected)
//      continue;
//    MCKeyFrame *pMCKF = pKF->GetMCKeyFrameConnection();
//    KeyFrame *pKFLoopCurrent = pMCKF->keyframes_[nCurrentKFCamIndex];
//    g2o::VertexSim3Expmap *VSim3 =
//        new g2o::VertexSim3Expmap(); // 一直new，不用释放？(wubo???)
//    const int nIDi = pKF->mnId;
//    LoopClosing::KeyFrameAndPose::const_iterator it =
//        CorrectedSim3.find(pKFLoopCurrent);
//    // 如果该关键帧在闭环时通过Sim3传播调整过，用校正后的位姿
//    if (it != CorrectedSim3.end()) {
//      vScw[nIDi] = g2oSFC * it->second;
//      VSim3->setEstimate(g2oSFC * it->second);
//    } else // 如果该关键帧在闭环时没有通过Sim3传播调整过，用自身的位姿
//    {
//      Eigen::Matrix<double, 3, 3> Rcw =
//          Converter::toMatrix3d(pKFLoopCurrent->GetRotation());
//      Eigen::Matrix<double, 3, 1> tcw =
//          Converter::toVector3d(pKFLoopCurrent->GetTranslation());
//      g2o::Sim3 Siw(Rcw, tcw, 1.0);
//      vScw[nIDi] = g2oSFC * Siw;
//      VSim3->setEstimate(g2oSFC * Siw);
//    }

//    // 闭环匹配上的帧不进行位姿优化
//    if (pKF->mnId == nFixed)
//      VSim3->setFixed(true);

//    VSim3->setId(nIDi);
//    VSim3->setMarginalized(false);
//    VSim3->_fix_scale = bFixScale;
//    optimizer.addVertex(VSim3);
//    sg2oVertexId.insert(nIDi);
//  }

//  set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

//  const Eigen::Matrix<double, 7, 7> matLambda =
//      Eigen::Matrix<double, 7, 7>::Identity();

//  int nEdgeLoop = 0;
//  for (map<KeyFrame *, set<KeyFrame *>>::const_iterator
//           mit = LoopConnections.begin(),
//           mend = LoopConnections.end();
//       mit != mend; mit++) {
//    KeyFrame *pKFCC = mit->first;

//    if (!pKFCC->mbGotMCConnected)
//      continue;
//    MCKeyFrame *pMCKFCC = pKFCC->GetMCKeyFrameConnection();

//    KeyFrame *pKF = pMCKFCC->keyframes_[FRONT_CAMERA];
//    const long unsigned int nIDi = pKF->mnId;

//    if (!sg2oVertexId.count(nIDi)) {
//      cout << " !!!! Wired  Error , Cam Index error in Optimized Essential "
//              "graph "
//           << endl;
//      continue;
//    }

//    const set<KeyFrame *> &spConnections = mit->second;
//    const g2o::Sim3 Siw = vScw[nIDi];
//    const g2o::Sim3 Swi = Siw.inverse();

//    for (set<KeyFrame *>::const_iterator sit = spConnections.begin(),
//                                         send = spConnections.end();
//         sit != send; sit++) {

//      KeyFrame *pKFi = *sit;
//      const long unsigned int nIDj = pKFi->mnId;

//      //            if((nIDi!=pCurKF->mnId || nIDj!=pLoopKF->mnId) &&
//      //            pKFCC->GetWeight(*sit)<minFeat)
//      if ((pKFCC->mnId != pCurKF->mnId || nIDj != pLoopKF->mnId) &&
//          pKFCC->GetWeight(*sit) < 80)
//        continue;

//      if (!pKFi->mbGotMCConnected)
//        continue;
//      MCKeyFrame *pMCKFi = pKFi->GetMCKeyFrameConnection();
//      KeyFrame *pKFRef = pMCKFi->keyframes_[FRONT_CAMERA];
//      const long unsigned int nIDk = pKFRef->mnId;
//      if (!sg2oVertexId.count(nIDk))
//        continue;
//      const g2o::Sim3 Skw = vScw[nIDk];
//      const g2o::Sim3 Ski = Skw * Swi;

//      g2o::EdgeSim3 *e = new g2o::EdgeSim3();
//      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
//                          optimizer.vertex(nIDk)));
//      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
//                          optimizer.vertex(nIDi)));
//      e->setMeasurement(Ski);
//      e->information() = matLambda;
//      optimizer.addEdge(e);
//      sInsertedEdges.insert(make_pair(min(nIDk, nIDi), max(nIDk, nIDi)));
//      nEdgeLoop++;
//    }
//  }

//  cout << " ## Loop closure edge : " << nEdgeLoop << endl;

//  // Set normal edges
//  // 步骤4：添加跟踪时形成的边、闭环匹配成功形成的边
//  for (size_t i = 0, iend = vpKFs.size(); i < iend; i++) {
//    KeyFrame *pKF = vpKFs[i];
//    const int nIDi = pKF->mnId;
//    if (!sg2oVertexId.count(nIDi))
//      continue;
//    g2o::Sim3 Swi;

//    if (!pKF->mbGotMCConnected)
//      continue;
//    MCKeyFrame *pMCKFCC = pKF->GetMCKeyFrameConnection();
//    KeyFrame *pKFCC = pMCKFCC->keyframes_[nCurrentKFCamIndex];

//    LoopClosing::KeyFrameAndPose::const_iterator iti =
//        NonCorrectedSim3.find(pKFCC);

//    // 尽可能得到未经过Sim3传播调整的位姿
//    if (iti != NonCorrectedSim3.end()) {
//      Swi = (iti->second).inverse() * g2oSFC.inverse();
//    } else {
//      Swi = vScw[nIDi].inverse();
//    }

//    KeyFrame *pParentKF = pKF->GetParent();

//    // Spanning tree edge
//    // 步骤4.1：只添加扩展树的边（有父关键帧）
//    if (pParentKF) {
//      int nIDj = pParentKF->mnId;
//      if (!sg2oVertexId.count(nIDj))
//        continue;

//      if (!pParentKF->mbGotMCConnected)
//        continue;
//      MCKeyFrame *pMCKFParent = pParentKF->GetMCKeyFrameConnection();
//      KeyFrame *pKFParentCC = pMCKFParent->keyframes_[nCurrentKFCamIndex];
//      g2o::Sim3 Sjw;
//      LoopClosing::KeyFrameAndPose::const_iterator itj =
//          NonCorrectedSim3.find(pKFParentCC);
//      // 尽可能得到未经过Sim3传播调整的位姿
//      if (itj != NonCorrectedSim3.end()) {
//        Sjw = g2oSFC * (itj->second);
//      } else
//        Sjw = vScw[nIDj];

//      g2o::Sim3 Sji = Sjw * Swi;
//      g2o::EdgeSim3 *e = new g2o::EdgeSim3();
//      e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
//                          optimizer.vertex(nIDj)));
//      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
//                          optimizer.vertex(nIDi)));
//      e->setMeasurement(Sji);
//      e->information() = matLambda;
//      optimizer.addEdge(e);
//    }

//    // Loop edges
//    //
//    步骤4.2：添加在CorrectLoop函数中AddLoopEdge函数添加的闭环连接边（当前帧与闭环匹配帧之间的连接关系）
//    // 使用经过Sim3调整前关键帧之间的相对关系作为边
//    const set<KeyFrame *> sLoopEdges = pKFCC->GetLoopEdges();
//    for (set<KeyFrame *>::const_iterator sit = sLoopEdges.begin(),
//                                         send = sLoopEdges.end();
//         sit != send; sit++) {
//      KeyFrame *pLKF = *sit;

//      if ((pLKF->mnId < pKFCC->mnId))
//        continue;
//      if (!pLKF->mbGotMCConnected)
//        continue;

//      MCKeyFrame *pMCKFL = pLKF->GetMCKeyFrameConnection();
//      KeyFrame *pLKFF = pMCKFL->keyframes_[FRONT_CAMERA];
//      const int nIDl = pLKFF->mnId;
//      if (!sg2oVertexId.count(nIDl))
//        continue;

//      g2o::Sim3 Slw;
//      LoopClosing::KeyFrameAndPose::const_iterator itl =
//          NonCorrectedSim3.find(pLKF);
//      if (itl != NonCorrectedSim3.end())
//        Slw = g2oSFC * itl->second;
//      else
//        Slw = vScw[nIDl];

//      g2o::Sim3 Sli = Slw * Swi;

//      g2o::EdgeSim3 *el = new g2o::EdgeSim3();
//      el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
//                           optimizer.vertex(nIDl)));
//      el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
//                           optimizer.vertex(nIDi)));
//      el->setMeasurement(Sli);
//      el->information() = matLambda;
//      optimizer.addEdge(el);
//    }

//    // Covisibility graph edges
//    // 步骤4.3：最有很好共视关系的关键帧也作为边进行优化
//    // 使用经过Sim3调整前关键帧之间的相对关系作为边
//    const vector<KeyFrame *> vpConnectedKFs =
//        pKF->GetCovisiblesByWeight(minFeat);
//    for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin();
//         vit != vpConnectedKFs.end(); vit++) {
//      KeyFrame *pKFn = *vit;
//      if (!pKFn->mbGotMCConnected)
//        continue;
//      MCKeyFrame *pMCKFn = pKFn->GetMCKeyFrameConnection();
//      KeyFrame *pKFnC = pMCKFn->keyframes_[nCurrentKFCamIndex];
//      if (!sg2oVertexId.count(pKFn->mnId))
//        continue;
//      if (pKFn && pKFn != pParentKF && !pKF->hasChild(pKFn) &&
//          !sLoopEdges.count(pKFnC)) {
//        if (!pKFn->isBad() && pKFn->mnId < pKF->mnId) {
//          if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId),
//                                             max(pKF->mnId, pKFn->mnId))))
//            continue;

//          g2o::Sim3 Snw;

//          LoopClosing::KeyFrameAndPose::const_iterator itn =
//              NonCorrectedSim3.find(pKFnC);

//          // 尽可能得到未经过Sim3传播调整的位姿
//          if (itn != NonCorrectedSim3.end())
//            Snw = g2oSFC * itn->second;
//          else
//            Snw = vScw[pKFn->mnId];

//          g2o::Sim3 Sni = Snw * Swi;

//          g2o::EdgeSim3 *en = new g2o::EdgeSim3();
//          en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
//                               optimizer.vertex(pKFn->mnId)));
//          en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(
//                               optimizer.vertex(nIDi)));
//          en->setMeasurement(Sni);
//          en->information() = matLambda;
//          optimizer.addEdge(en);
//        }
//      }
//    }
//  }

//  // Optimize!
//  // 步骤5：开始g2o优化
//  optimizer.initializeOptimization();
//  optimizer.optimize(20);

//  unique_lock<mutex> lock(pMap->mMutexMapUpdate);

//  // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
//  // 步骤6：设定优化后的位姿
//  for (size_t i = 0; i < vpKFs.size(); i++) {
//    KeyFrame *pKFi = vpKFs[i];

//    const int nIDi = pKFi->mnId;
//    if (!sg2oVertexId.count(pKFi->mnId))
//      continue;
//    g2o::VertexSim3Expmap *VSim3 =
//        static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
//    g2o::Sim3 CorrectedSiw = VSim3->estimate();
//    vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
//    Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
//    Eigen::Vector3d eigt = CorrectedSiw.translation();
//    double s = CorrectedSiw.scale();

//    eigt *= (1. / s); //[R t/s;0 1]

//    cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

//    pKFi->SetPose(Tiw);
//  }

//  for (size_t i = 0; i < vpKFs.size(); i++) {
//    KeyFrame *pKF = vpKFs[i];
//    if (sg2oVertexId.count(pKF->mnId)) {
//      if (pKF->mbGotMCConnected) {
//        MCKeyFrame *pMCKF = pKF->GetMCKeyFrameConnection();
//        cv::Mat mTcamFront = pMCKF->cam_exts_[FRONT_CAMERA];
//        for (int camId = 0; camId < 4; camId++) {
//          KeyFrame *pKFii = pMCKF->keyframes_[camId];
//          if (pKFii->camera_index_ == FRONT_CAMERA)
//            continue;
//          pKFii->SetPose(pMCKF->cam_exts_[camId] * mTcamFront.inv() *
//                         pMCKF->keyframes_[FRONT_CAMERA]->GetPose());
//        }
//      }
//    }
//  }

//  // Correct points. Transform to "non-optimized" reference keyframe pose and
//  // transform back with optimized pose
//  //
//  步骤7：步骤5和步骤6优化得到关键帧的位姿后，MapPoints根据参考帧优化前后的相对关系调整自己的位置
//  for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
//    MapPoint *pMP = vpMPs[i];

//    if (pMP->isBad())
//      continue;

//    int nIDr;
//    if (pMP->mnCorrectedByKF == pCurKF->mnId &&
//        pMP->mnCorrectedReference != 0) {
//      nIDr = pMP->mnCorrectedReference;
//    } else {
//      KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
//      int nIDref = pRefKF->mnId;
//      if (sg2oVertexId.count(nIDref))
//        nIDr = pRefKF->mnId;
//      else {
//        if (pRefKF->mbGotMCConnected) {
//          MCKeyFrame *pMCKF = pRefKF->GetMCKeyFrameConnection();
//          KeyFrame *pRefCoCamKF = pMCKF->keyframes_[FRONT_CAMERA];
//          nIDr = pRefCoCamKF->mnId;
//        }
//      }
//    }

//    // 得到MapPoint参考关键帧步骤5优化前的位姿
//    g2o::Sim3 Srw = vScw[nIDr];
//    // 得到MapPoint参考关键帧优化后的位姿
//    g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

//    cv::Mat P3Dw = pMP->GetWorldPos();
//    Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
//    Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw =
//        correctedSwr.map(Srw.map(eigP3Dw));

//    cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
//    pMP->SetWorldPos(cvCorrectedP3Dw);
//    pMP->UpdateNormalAndDepth();
//  }
//}

class G2O_CORE_API EdgeSim3ProjectXYZ_RE
    : public g2o::BaseBinaryEdge<2, g2o::Vector2d, g2o::VertexSBAPointXYZ,
                                 g2o::VertexSim3Expmap> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSim3ProjectXYZ_RE(g2o::SE3Quat CamExt1, g2o::SE3Quat CamExt2)
      : BaseBinaryEdge<2, g2o::Vector2d, g2o::VertexSBAPointXYZ,
                       g2o::VertexSim3Expmap>() {

    CamExt1_ = g2o::Sim3(CamExt1.rotation(), CamExt1.translation(), 1);
    CamExt2_ = g2o::Sim3(CamExt2.rotation(), CamExt2.translation(), 1);
  }
  ~EdgeSim3ProjectXYZ_RE() {}
  virtual bool read(std::istream &is) {
    for (int i = 0; i < 2; i++) {
      is >> _measurement[i];
    }

    for (int i = 0; i < 2; i++)
      for (int j = i; j < 2; j++) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }
  virtual bool write(std::ostream &os) const {
    for (int i = 0; i < 2; i++) {
      os << _measurement[i] << " ";
    }

    for (int i = 0; i < 2; i++)
      for (int j = i; j < 2; j++) {
        os << " " << information()(i, j);
      }
    return os.good();
  }

  void computeError() {
    const g2o::VertexSim3Expmap *v1 =
        static_cast<const g2o::VertexSim3Expmap *>(_vertices[1]);
    const g2o::VertexSBAPointXYZ *v2 =
        static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

    g2o::Vector2d obs(_measurement);

    g2o::Sim3 sim3_initial = v1->estimate();

    g2o::Sim3 sim3 = CamExt1_ * sim3_initial * CamExt2_;
    //      _error =
    //      obs-v1->cam_map_panoramic(project(v1->estimate().map(v2->estimate())));
    //     _error =
    //     obs-v1->cam_map_panoramic(project(sim3.map(v2->estimate())));
    _error =
        obs - v1->cam_map_panoramic(g2o::project(sim3.map(v2->estimate())));
  }

private:
  g2o::Sim3 CamExt1_;
  g2o::Sim3 CamExt2_;
};

class G2O_CORE_API EdgeInverseSim3ProjectXYZ_RE
    : public g2o::BaseBinaryEdge<2, g2o::Vector2d, g2o::VertexSBAPointXYZ,
                                 g2o::VertexSim3Expmap> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeInverseSim3ProjectXYZ_RE(g2o::SE3Quat CamExt1, g2o::SE3Quat CamExt2)
      : BaseBinaryEdge<2, g2o::Vector2d, g2o::VertexSBAPointXYZ,
                       g2o::VertexSim3Expmap>() {

    CamExt1_ = g2o::Sim3(CamExt1.rotation(), CamExt1.translation(), 1);
    CamExt2_ = g2o::Sim3(CamExt2.rotation(), CamExt2.translation(), 1);
  }
  ~EdgeInverseSim3ProjectXYZ_RE() {}
  virtual bool read(std::istream &is) {
    for (int i = 0; i < 2; i++) {
      is >> _measurement[i];
    }

    for (int i = 0; i < 2; i++)
      for (int j = i; j < 2; j++) {
        is >> information()(i, j);
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }
  virtual bool write(std::ostream &os) const {
    for (int i = 0; i < 2; i++) {
      os << _measurement[i] << " ";
    }

    for (int i = 0; i < 2; i++)
      for (int j = i; j < 2; j++) {
        os << " " << information()(i, j);
      }
    return os.good();
  }

  void computeError() {
    const g2o::VertexSim3Expmap *v1 =
        static_cast<const g2o::VertexSim3Expmap *>(_vertices[1]);
    const g2o::VertexSBAPointXYZ *v2 =
        static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);

    g2o::Vector2d obs(_measurement);

    g2o::Sim3 sim3_initial = v1->estimate();

    g2o::Sim3 sim3 = CamExt1_ * sim3_initial * CamExt2_;
    _error =
        obs -
        v1->cam_map_panoramic(g2o::project(sim3.inverse().map(v2->estimate())));
  }

private:
  g2o::Sim3 CamExt1_;
  g2o::Sim3 CamExt2_;
};
}
