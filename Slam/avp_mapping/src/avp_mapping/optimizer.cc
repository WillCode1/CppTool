#include "optimizer.h"

#include "viewerconfig.h"
#ifdef ENABLE_VIEWER
#include "hpa_viewer.h"
#include "plot_viewer.h"
#endif
#include "timer.h"
namespace SemanticSLAM
{

  std::mutex Optimizer::global_optimizer_mutex_;
  Optimizer::Optimizer() : optimizer_(nullptr) {}

  Optimizer::~Optimizer() {}

  Optimizer &Optimizer::GetInstance()
  {
    static Optimizer *instance = nullptr;

    if (!instance)
    {
      global_optimizer_mutex_.lock();
      instance = new Optimizer();
      instance->optimizer_ = new g2o::SparseOptimizer;
      g2o::BlockSolverX::LinearSolverType *linearsolver;
      linearsolver =
          new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
      g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearsolver);
      g2o::OptimizationAlgorithmLevenberg *solver =
          new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      instance->optimizer_->setAlgorithm(solver);
      std::unordered_map<g2o::EdgeSE2SemanticProject *, bool> empty_list;
      instance->edge_resources_ = empty_list;
      instance->vse2_ = new g2o::VertexSE2();
      global_optimizer_mutex_.unlock();
    }
    return *instance;
  }

  void DrawProjection(cv::InputArray im, const Vec3_t &trans_tc_vector,
                      const std::vector<Vec3_t> &points,
                      const CameraConfig &camera_config,
                      const cv::Scalar &color)
  {

    const double baselink2cam = camera_config.cam_extrinsic.baselink2cam;
    const double cam_height_ = camera_config.cam_extrinsic.camera_height;
    cv::Mat image = im.getMat();

    double fx = camera_config.cam_intrinsic.fx;
    double fy = camera_config.cam_intrinsic.fy;
    double cx = camera_config.cam_intrinsic.cx;
    double cy = camera_config.cam_intrinsic.cy;

    for (auto p : points)
    {

      p.z() = 1;
      double theta = trans_tc_vector.z();
      double inv_x =
          -cos(theta) * trans_tc_vector.x() - sin(theta) * trans_tc_vector.y();
      double inv_y =
          sin(theta) * trans_tc_vector.x() - cos(theta) * trans_tc_vector.y();
      double inv_theta = -theta;

      Mat33_t trans_ct;
      trans_ct << cos(inv_theta), -sin(inv_theta), inv_x, sin(inv_theta),
          cos(inv_theta), inv_y, 0, 0, 1;

      Vec3_t point_vehicle = trans_ct * p;
      Vec3_t point_camera(-point_vehicle.y(), -point_vehicle.x() + baselink2cam,
                          cam_height_);

      Vec2_t uv(point_camera.x() / point_camera.z() * fx + cx,
                point_camera.y() / point_camera.z() * fy + cy);

      cv::circle(image, cv::Point2d(uv.x(), uv.y()), 2, color, 2, CV_AA);
    }
  }

  /***
   *  optimize current frame pose relative to ref keyframe
   */
  bool Optimizer::OptimizeFramePose(KeyFrame *last_keyframe, Frame &current_frame,

                                    const CameraConfig &camera_config)
  {

    ResetResource();

    Mat33_t current_pose = current_frame.trans_world2base_;
    Mat33_t last_pose = last_keyframe->trans_world2base_;

    Mat33_t trans_lc = last_pose.inverse() * current_pose;
    Vec3_t trans_lc_vector = Utils::Se2Matrix2Vector(trans_lc);
    vse2_->setId(0);
    vse2_->setEstimate(
        g2o::SE2(trans_lc_vector.x(), trans_lc_vector.y(), trans_lc_vector.z()));
    optimizer_->addVertex(vse2_);

    const double baselink2cam = camera_config.cam_extrinsic.baselink2cam;
    const double camera_height = camera_config.cam_extrinsic.camera_height;

    const double fx = camera_config.cam_intrinsic.fx;
    const double fy = camera_config.cam_intrinsic.fy;
    const double cx = camera_config.cam_intrinsic.cx;
    const double cy = camera_config.cam_intrinsic.cy;

    g2o::CameraParameters *cam_param =
        new g2o::CameraParameters(fx, fy, Eigen::Vector2d(cx, cy));

    Timer timer("optimization");

    std::vector<g2o::EdgeSE2SemanticProject *> edges;

    // Add Pose Prior Edge

    g2o::EdgeSE2PosePrior *edge_pose_prior = CreatePosePriorEdge(trans_lc_vector);

#ifdef ENABLE_VIEWER
    std::vector<Vec3_t> keyframe_points;
#endif

    const int kMinSlotProb = SystemConfig::GetSystemConfig()->map_prob_min_slot_;

    for (auto &smt_point : last_keyframe->slot_points_)
    {
      unsigned char measurement = smt_point.prob;
      if (measurement == 0)
        continue;

      if (measurement < kMinSlotProb)
        continue;

      auto edge = GetEdgeResource();
      edge->SetImagePtr(&current_frame.image_slot_);
      edge->SetWorldPoint(Vec3_t(smt_point.x, smt_point.y, 0));
      edge->SetCameraExtrinsic(camera_height, baselink2cam);
      edge->setVertex(0, vse2_);
      edge->setCameraParameter(cam_param);

      if (!edge->IsWithinimage())
        continue;

      double dist = hypot(smt_point.v - cy, smt_point.u - cx);
      double weight = 10000.0 / ((dist + 10) * (dist + 10));
      edge->setInformation(weight * Eigen::Matrix<double, 1, 1>::Identity());
      edge->setMeasurement((double)measurement);
      optimizer_->addEdge(edge);
      edges.push_back(edge);

#ifdef ENABLE_VIEWER
      keyframe_points.push_back(Vec3_t(smt_point.x, smt_point.y, 0));
#endif
    }

    optimizer_->setVerbose(false);
    if (!edges.empty())
    {
      optimizer_->initializeOptimization(0);
      optimizer_->optimize(10);
    }

    current_frame.edge_size_ = edges.size();
    timer.Print();

    Vec3_t optimized_tlc = vse2_->estimate().toVector();
    // update pose
    current_frame.trans_world2base_ =
        last_pose * Utils::Se2Vector2Matrix(optimized_tlc);

#ifdef ENABLE_VIEWER

    PlotViewer::GetInstance().UpdateData("optimization",
                                         timer.GetTimeConsuming());
    cv::Mat image_current_frame = current_frame.image_slot_.clone();
    cv::cvtColor(image_current_frame, image_current_frame, cv::COLOR_GRAY2BGR);

    cv::Scalar color1(255, 156, 0);
    cv::Scalar color2(0, 255, 112);
    // Draw projection before optimization
    DrawProjection(image_current_frame, trans_lc_vector, keyframe_points,
                   camera_config, color1);
    // Draw projection after optimization
    DrawProjection(image_current_frame, optimized_tlc, keyframe_points,
                   camera_config, color2);

    cv::putText(image_current_frame, "before optimization ",
                cv::Point(10, image_current_frame.rows - 50),
                cv::FONT_HERSHEY_COMPLEX, 0.8, color1, 1, 6);

    cv::putText(image_current_frame, "after optimization ",
                cv::Point(10, image_current_frame.rows - 20),
                cv::FONT_HERSHEY_COMPLEX, 0.8, color2, 1, 6);
    HpaViewer::GetInstance().UpdateProjection(image_current_frame);
#endif

    delete cam_param;
    delete edge_pose_prior;
    return true;
  }

  g2o::EdgeSE2PosePrior *
  Optimizer::CreatePosePriorEdge(const Vec3_t &trans_lc_vector)
  {
    g2o::EdgeSE2PosePrior *edge_pose_prior = new g2o::EdgeSE2PosePrior();

    edge_pose_prior->vertices()[0] = vse2_;
    double prior_x = trans_lc_vector(0);
    double prior_y = trans_lc_vector(1);
    double prior_theta = trans_lc_vector(2);
    Mat33_t odom_sigma = Mat33_t::Identity();
    double odom_x_sigma = 10;
    double odom_y_sigma = 10;
    double odom_theta_sigma = 0.001;

    double x_sigma = std::max(odom_x_sigma * fabs(prior_x), 0.1);
    double y_sigma = std::max(odom_y_sigma * fabs(prior_y), 0.05);
    double theta_sigma = std::max(odom_theta_sigma * fabs(prior_theta), 0.0001);

    odom_sigma(0, 0) = x_sigma;
    odom_sigma(1, 1) = y_sigma;
    odom_sigma(2, 2) = theta_sigma;
    edge_pose_prior->setInformation(odom_sigma.inverse() * 100);
    edge_pose_prior->setMeasurement(trans_lc_vector);
    optimizer_->addEdge(edge_pose_prior);
    return edge_pose_prior;
  }

  g2o::EdgeSE2SemanticProject *Optimizer::GetEdgeResource()
  {
    bool found = false;
    g2o::EdgeSE2SemanticProject *e = nullptr;
    for (auto it = edge_resources_.begin(); it != edge_resources_.end(); ++it)
    {
      if (!it->second)
      {
        e = it->first;
        it->second = true;
        found = true;
        break;
      }
    }

    if (!found)
    {
      e = new g2o::EdgeSE2SemanticProject();
      edge_resources_.insert(
          std::pair<g2o::EdgeSE2SemanticProject *, bool>(e, true));
    }
    return e;
  }

  void Optimizer::ResetResource()
  {
    optimizer_->shallowClear();
    vse2_->Clear();
    for (auto it = edge_resources_.begin(); it != edge_resources_.end(); ++it)
    {
      it->second = false;
      it->first->reset();
    }
  }
}
