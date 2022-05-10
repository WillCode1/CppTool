#include <trajectory_smoother.h>
namespace SemanticSLAM
{

  TrajectorySmoother::TrajectorySmoother(double odom_x_sigma, double odom_y_sigma,
                                         double odom_theta_sigma)
      : current_id_(0), odom_x_sigma_(odom_x_sigma), odom_y_sigma_(odom_y_sigma),
        odom_theta_sigma_(odom_theta_sigma), max_vertex_(50)
  {
    // create optimizer
    optimizer_ = std::make_shared<g2o::SparseOptimizer>();
    linearsolver_ = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    solverptr_ = new g2o::BlockSolverX(linearsolver_);
    solver_ = new g2o::OptimizationAlgorithmLevenberg(solverptr_);
    solver_->setUserLambdaInit(1e-16);
    optimizer_->setAlgorithm(solver_);
    optimizer_->setVerbose(false);
  }

  TrajectorySmoother::~TrajectorySmoother()
  {
    optimizer_->clear();

    //  delete solver_;
    //  solver_ = nullptr;

    //  delete solverptr_;
    //  solverptr_ = nullptr;

    //  delete linearsolver_;
    //  linearsolver_ = nullptr;

    // delete vertex
    while (!vertex_queue_.empty())
      vertex_queue_.pop();
    // delete edge
    while (!edge_se2_queue_.empty())
      edge_se2_queue_.pop();
    while (!edge_se2_prior_queue_.empty())
      edge_se2_prior_queue_.pop();
  }

  bool TrajectorySmoother::Smoother(Frame *frame)
  {
    // new vertex
    if (!NeedFuse(frame))
    {
      OptimizeNormalFrame(frame);
      return false;
    }

    InsertNewVertex(frame);
    InsertVisualLocPriorEdge(frame);
    InsertOdometryEdge(frame);

    if (vertex_queue_.size() < 4)
    {
      current_id_++;
      OptimizeNormalFrame(frame);

      return false;
    }

    if (vertex_queue_.size() > max_vertex_)
    {
      RemoveFrontestData();
    }

    optimizer_->initializeOptimization();
    optimizer_->optimize(20);

    //  optimizer.setVerbose(true);

    g2o::VertexSE2 *v = static_cast<g2o::VertexSE2 *>(optimizer_->vertex(current_id_));
    Vec3_t pose = v->estimate().toVector();

    NM_DEBUG(" trajectory smooth  x : {} {}\n", frame->trans_world2base_(0, 2), frame->trans_world2base_(1, 2));
    NM_DEBUG(" after  : {} {}\n ", pose.x(), pose.y());

    frame->trans_world2base_ = Utils::Se2Vector2Matrix(pose);
    current_id_++;
    return true;
  }

  void TrajectorySmoother::Reset()
  {
    current_id_ = 0;
    optimizer_->clear();
    // delete vertex
    while (!vertex_queue_.empty())
      vertex_queue_.pop();
    // delete edge
    while (!edge_se2_queue_.empty())
      edge_se2_queue_.pop();
    while (!edge_se2_prior_queue_.empty())
      edge_se2_prior_queue_.pop();
  }

  bool TrajectorySmoother::ScaleOdometrySigma(double scale)
  {
    NM_ERROR("NOT SUPPORT");
    return true;
  }

  void TrajectorySmoother::InsertNewVertex(Frame *frame)
  {
    double x = frame->trans_world2base_(0, 2);
    double y = frame->trans_world2base_(1, 2);
    double theta = atan2(frame->trans_world2base_(1, 0), frame->trans_world2base_(0, 0));
    g2o::VertexSE2 *v = new g2o::VertexSE2();

    v->setEstimate(g2o::SE2(x, y, theta));
    v->setId(current_id_);
    v->setFixed(current_id_ == 0);
    optimizer_->addVertex(v);
    vertex_queue_.push(v);
  }

  void TrajectorySmoother::InsertVisualLocPriorEdge(Frame *frame)
  {
    //  Mat33_t prior_information;
    //  prior_information << 1, 0, 0, 0, 1, 0, 0, 0, 20;

    Mat33_t prior_information;
    prior_information << 0.005 * frame->visual_loc_confidence_, 0, 0,
        0, 0.005 * frame->visual_loc_confidence_, 0,
        0, 0, 0.1 * frame->visual_loc_confidence_;

    double x = frame->trans_world2base_(0, 2);
    double y = frame->trans_world2base_(1, 2);
    double theta = atan2(frame->trans_world2base_(1, 0), frame->trans_world2base_(0, 0));
    g2o::EdgeSE2PosePrior *edge = new g2o::EdgeSE2PosePrior;
    edge->vertices()[0] = optimizer_->vertex(current_id_);
    g2o::Vector3D measurement(x, y, theta);   // 赋一样的值做为先验

    edge->setInformation(prior_information);
    edge->setMeasurement(measurement);
    optimizer_->addEdge(edge);
    edge_se2_prior_queue_.push(edge);
  }

  void TrajectorySmoother::InsertOdometryEdge(Frame *frame)
  {
    if (vertex_queue_.size() == 1) // first vertex
    {
      last_odometry_ = frame->odometry_;
      // frame->GetOdometry(last_odometry_);
      return;
    }

    Mat33_t last_odom_matrix = Utils::Se2Vector2Matrix(last_odometry_);

    Vec3_t current_odom;
    current_odom = frame->odometry_;
    // frame->GetOdometry(current_odom);
    Mat33_t current_odom_matrix = Utils::Se2Vector2Matrix(current_odom);

    Mat33_t odom_increment = last_odom_matrix.inverse() * current_odom_matrix;

    double x = odom_increment(0, 2);
    double y = odom_increment(1, 2);
    double theta = atan2(odom_increment(1, 0), odom_increment(0, 0));

    // this happen when the vehicle move very slow and the wheel odometry is wrong
    //  if(fabs(x) < 1e-5 &&fabs(y) <1e-5  )
    //    return ;

    Mat33_t odom_sigma;

    double x_sigma = std::max(odom_x_sigma_ * fabs(x), 0.01);
    double y_sigma = std::max(odom_y_sigma_ * fabs(y), 0.005);
    double theta_sigma = std::max(odom_theta_sigma_ * fabs(theta), 0.0001);

    odom_sigma << x_sigma, 0, 0, 0, y_sigma * fabs(y), 0, 0, 0, theta_sigma;
    const int id_end = current_id_;
    const int id_start = id_end - 1;

    g2o::EdgeSE2 *odometry = new g2o::EdgeSE2;
    odometry->vertices()[0] = optimizer_->vertex(id_start);
    odometry->vertices()[1] = optimizer_->vertex(id_end);
    odometry->setMeasurement(g2o::SE2(x, y, theta));
    //  odometry->setInformation(odometry_info);
    odometry->setInformation(odom_sigma.inverse());
    optimizer_->addEdge(odometry);
    edge_se2_queue_.push(odometry);
    // frame->GetOdometry(last_odometry_);
    last_odometry_ = frame->odometry_;
  }

  void TrajectorySmoother::RemoveFrontestData()
  {
    g2o::VertexSE2 *vertex = vertex_queue_.front();
    g2o::EdgeSE2 *edge_se2 = edge_se2_queue_.front();
    g2o::EdgeSE2PosePrior *edge_se2_prior = edge_se2_prior_queue_.front();

    optimizer_->removeVertex(vertex);       // hyper graph will delete the pointer
    optimizer_->removeEdge(edge_se2);       // hyper graph will delete the pointer
    optimizer_->removeEdge(edge_se2_prior); // hyper graph will delete the pointer

    vertex_queue_.pop();
    edge_se2_queue_.pop();
    edge_se2_prior_queue_.pop();
  }

  bool TrajectorySmoother::NeedFuse(Frame *frame)
  {
    if (vertex_queue_.size() == 0)
      return true;

    Vec3_t current_odom;
    current_odom = frame->odometry_;
    //  frame->GetOdometry(current_odom);

    double distance = hypot(current_odom.x() - last_odometry_.x(),
                            current_odom.y() - last_odometry_.y());
    if (distance < 0.1)
      return false;

    return true;
  }

  void TrajectorySmoother::OptimizeNormalFrame(Frame *frame)
  {
    if (vertex_queue_.size() == 0) // first vertex
    {
      return;
    }

    double x = frame->trans_world2base_(0, 2);
    double y = frame->trans_world2base_(1, 2);
    double theta = atan2(frame->trans_world2base_(1, 0), frame->trans_world2base_(0, 0));
    g2o::VertexSE2 *pose_vertex = new g2o::VertexSE2();

    pose_vertex->setEstimate(g2o::SE2(x, y, theta));
    pose_vertex->setId(current_id_);
    pose_vertex->setFixed(current_id_ == 0);
    optimizer_->addVertex(pose_vertex);

    // prior edge
    //  Mat33_t prior_information;
    //  prior_information << 1, 0, 0, 0, 1, 0, 0, 0, 20;
    //  g2o::EdgeSE2PosePrior *edge = new g2o::EdgeSE2PosePrior;
    //  edge->vertices()[0] = optimizer_->vertex(current_id_);
    //  g2o::Vector3D measurement(x, y, theta);
    //  edge->setInformation(prior_information);
    //  edge->setMeasurement(measurement);
    //  optimizer_->addEdge(edge);

    // odometry edge
    Mat33_t last_odom_matrix = Utils::Se2Vector2Matrix(last_odometry_);
    Vec3_t current_odom;
    current_odom = frame->odometry_;
    //  frame->GetOdometry(current_odom);
    Mat33_t current_odom_matrix = Utils::Se2Vector2Matrix(current_odom);

    Mat33_t odom_increment = last_odom_matrix.inverse() * current_odom_matrix;

    double odom_increment_x = odom_increment(0, 2);
    double odom_increment_y = odom_increment(1, 2);
    double odom_increment_theta = atan2(odom_increment(1, 0), odom_increment(0, 0));

    Mat33_t odom_sigma;

    double x_sigma = std::max(odom_x_sigma_ * fabs(x), 0.01);
    double y_sigma = std::max(odom_y_sigma_ * fabs(y), 0.005);
    double theta_sigma = std::max(odom_theta_sigma_ * fabs(theta), 0.0001);

    odom_sigma << x_sigma, 0, 0, 0, y_sigma, 0, 0, 0, theta_sigma;
    const int id_end = current_id_;
    const int id_start = id_end - 1;

    g2o::EdgeSE2 *odometry = new g2o::EdgeSE2;
    odometry->vertices()[0] = optimizer_->vertex(id_start);
    odometry->vertices()[1] = optimizer_->vertex(id_end);
    odometry->setMeasurement(g2o::SE2(odom_increment_x, odom_increment_y, odom_increment_theta));
    //  odometry->setInformation(odometry_info);
    odometry->setInformation(odom_sigma.inverse());
    optimizer_->addEdge(odometry);

    optimizer_->initializeOptimization();
    optimizer_->optimize(20);
    g2o::VertexSE2 *v = static_cast<g2o::VertexSE2 *>(optimizer_->vertex(current_id_));
    Vec3_t pose = v->estimate().toVector();
    frame->trans_world2base_ = Utils::Se2Vector2Matrix(pose);
    optimizer_->removeVertex(pose_vertex);
    optimizer_->removeEdge(odometry);
    //  optimizer_->removeEdge(edge);
  }
}
