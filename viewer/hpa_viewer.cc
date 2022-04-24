#include "hpa_viewer.h"

std::mutex HpaViewer::global_mutex_;

HpaViewer::HpaViewer(int viewer_width, int viewer_height)
    : video_img_changed_(false), projection_changed_(false),
      video_width_(viewer_width), video_height_(viewer_height),
      current_pose_(Eigen::Vector3d(0, 0, 0)), finish_request_(false),
      is_finish_(false) {

  video_data_ = new unsigned char[3 * video_width_ * video_height_];
  projection_data_ = new unsigned char[3 * video_width_ * video_height_];
  simple_car_model_ = std::make_shared<SimpleCarModel>();
}

HpaViewer::~HpaViewer() {}

pangolin::OpenGlMatrix
GetSmoothMatrix(const pangolin::OpenGlMatrix &current_matrix,
                const pangolin::OpenGlMatrix &target_matrix) {
  double x0 = current_matrix.m[12];
  double y0 = current_matrix.m[13];

  double x1 = target_matrix.m[12];
  double y1 = target_matrix.m[13];

  double distance = hypot(x0 - x1, y0 - y1);

  pangolin::OpenGlMatrix m = current_matrix;

  if (distance < 0.02) {
    m = target_matrix;
    return m;
  }

  double x = 0.9 * x0 + 0.1 * x1;
  double y = 0.9 * y0 + 0.1 * y1;

  m.m[12] = x;
  m.m[13] = y;

  return m;
}

void HpaViewer::Run() {
  pangolin::CreateWindowAndBind("Avp Mapper", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  double mViewpointF = 2000;
  double mViewpointX = 0;
  double mViewpointY = 0;
  double mViewpointZ = 100;

  int ui_width = 175;
  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(ui_width));

  pangolin::Var<bool> menu_follow_camera("menu.FollowCamera", true, true);
  pangolin::Var<bool> menu_draw_current_pose("menu.DrawCurrentPose", true,
                                             true);
  pangolin::Var<bool> menu_draw_grid("menu.DrawGrid", false, true);
  pangolin::Var<bool> menu_draw_keyframe("menu.DrawKeyframe", false, true);
  pangolin::Var<bool> menu_draw_quadtree("menu.DrawQuadTree", true, true);
  pangolin::Var<bool> menu_show_video("menu.ShowVideo", true, true);
  pangolin::Var<bool> menu_show_projection("menu.ShowProjection", true, true);
  pangolin::Var<bool> menu_draw_feature_area("menu.ShowFeatureArea", true,
                                             true);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389,
                                 0.1, 1000),
      pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ,
                                mViewpointX, mViewpointY, 0, 0.0, 1.0, 0.0));

  pangolin::View &d_cam =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0 / 768.0)
          .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::View &d_video = pangolin::Display("imgVideo")
                                .SetAspect(video_width_ / (float)video_height_);

  pangolin::View &d_projection =
      pangolin::Display("imgProjection")
          .SetAspect(video_width_ / (float)video_height_);

  pangolin::GlTexture tex_video(video_width_, video_height_, GL_RGB, false, 0,
                                GL_RGB, GL_UNSIGNED_BYTE);
  pangolin::GlTexture tex_projection(video_width_, video_height_, GL_RGB, false,
                                     0, GL_RGB, GL_UNSIGNED_BYTE);

  pangolin::CreateDisplay()
      .SetBounds(0.0, 0.3, pangolin::Attach::Pix(ui_width), 1.0)
      .SetLayout(pangolin::LayoutEqual)
      .AddDisplay(d_video)
      .AddDisplay(d_projection);

  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();

  auto last_view_matrix = GetCurrentCamMatrix();

  while (true) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(0.0156f, 0.1019f, 0.231f, 0.0f);

    // 3D Display
    {
      if (menu_follow_camera) {

        auto target_view_matrix = GetCurrentCamMatrix();
        auto smooth_matrix =
            GetSmoothMatrix(last_view_matrix, target_view_matrix);
        s_cam.Follow(smooth_matrix);
        last_view_matrix = smooth_matrix;
        s_cam.Follow(GetCurrentCamMatrix());
      }

      if (menu_draw_quadtree) {
        DrawQuadtree();
      }

      if (menu_draw_grid) {
        DrawGrid();
        DrawAxis();
      }

      if (menu_draw_keyframe)
        DrawKeyframes();

      if (menu_draw_current_pose)
        DrawCurrentPose();

      if (menu_draw_feature_area)
        DrawMarkerPoints();
    }

    // 2D Display
    {
      std::lock_guard<std::mutex> lock(mutex_texture_);
      if (video_img_changed_)
        tex_video.Upload(video_data_, GL_BGR, GL_UNSIGNED_BYTE);
      video_img_changed_ = false;
      if (projection_changed_)
        tex_projection.Upload(projection_data_, GL_BGR, GL_UNSIGNED_BYTE);
      projection_changed_ = false;
    }
    if (menu_show_video) {
      d_video.Activate();
      glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
      tex_video.RenderToViewportFlipY();
    }

    if (menu_show_projection) {
      d_projection.Activate();
      glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
      tex_projection.RenderToViewportFlipY();
    }
    pangolin::FinishFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    if (CheckFinish()) {
      break;
    }
  }
  SetFinish();
}

HpaViewer &HpaViewer::GetInstance(int width, int height) {
  static HpaViewer *instance = nullptr;
  if (instance == nullptr) {
    global_mutex_.lock();
    instance = new HpaViewer(width, height);
    global_mutex_.unlock();
  }
  return *instance;
}

bool HpaViewer::CheckFinish() {
  std::unique_lock<std::mutex> lock(mutex_finishe_);
  return finish_request_;
}

void HpaViewer::SetFinish() {
  std::unique_lock<std::mutex> lock(mutex_finishe_);
  is_finish_ = true;
}

void HpaViewer::RequestFinish() {
  std::unique_lock<std::mutex> lock(mutex_finishe_);
  finish_request_ = true;
}

bool HpaViewer::IsFinish() {
  std::unique_lock<std::mutex> lock(mutex_finishe_);
  return is_finish_;
}

void HpaViewer::AddMarker() {
  double x = 0, y = 0;
  {
    std::unique_lock<std::mutex> lock(mutex_pose_);
    x = current_pose_.x();
    y = current_pose_.y();
  }
  std::unique_lock<std::mutex> lock2(mutex_marker_);
  marker_points_.push_back(Eigen::Vector2d(x, y));
}

bool HpaViewer::UpdateSemanticMap(
    const std::vector<SemanticSLAM::PointTyped> &semantic_map) {
  std::lock_guard<std::mutex> lock(mutex_map_);
  semantic_map_.insert(semantic_map_.end(), semantic_map.begin(),
                       semantic_map.end());
  return true;
}

void HpaViewer::DrawGrid() {
  glLineWidth(0.4f);
  glBegin(GL_LINES);
  glColor3f(0.2235f, 0.7215f, 0.99215f);
  for (float i = -4; i <= 4; i = i + 0.1f) {
    glVertex3f(4, i, 0);
    glVertex3f(-4, i, 0);
    glVertex3f(i, 4, 0);
    glVertex3f(i, -4, 0);
  }
  glEnd();
}

void HpaViewer::DrawAxis() {
  glColor3f(1, 0, 0);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(10, 0, 0);
  glColor3f(0, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 10, 0);
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 10);
  glEnd();
}

void HpaViewer::SetCurrentPose(const Eigen::Matrix3d &trans_world2base) {
  std::unique_lock<std::mutex> lock(mutex_pose_);
  current_pose_.x() = trans_world2base(0, 2);
  current_pose_.y() = trans_world2base(1, 2);
  current_pose_.z() = atan2(trans_world2base(1, 0), trans_world2base(0, 0));
}

void HpaViewer::AddKeyframePose(const Eigen::Matrix3d &kf_world2base) {
  std::lock_guard<std::mutex> lock(mutex_keyframe_);
  keyframe_poses_.push_back(kf_world2base);
}

void HpaViewer::Reset() {
  std::lock_guard<std::mutex> lock(mutex_keyframe_);
  std::lock_guard<std::mutex> lock2(mutex_pose_);
  std::lock_guard<std::mutex> lock3(mutex_map_);
  keyframe_poses_.clear();
  current_pose_ = Eigen::Vector3d(0, 0, 0);
  semantic_map_.clear();
}

void HpaViewer::UpdateVideo(cv::InputArray video_frame) {

  std::lock_guard<std::mutex> lock(mutex_texture_);

  cv::Mat image = video_frame.getMat();
  if (image.channels() == 1) {
    cv::cvtColor(image, image, CV_GRAY2BGR);
  }

  if (image.rows != video_height_ || image.cols != video_width_)
    cv::resize(image, image, cv::Size(video_width_, video_height_));

  memcpy(video_data_, image.data,
         sizeof(unsigned char) * 3 * image.rows * image.cols);
  video_img_changed_ = true;
}

void HpaViewer::UpdateProjection(cv::InputArray projection_frame) {
  std::lock_guard<std::mutex> lock(mutex_texture_);
  cv::Mat image = projection_frame.getMat();
  if (image.channels() == 1) {
    cv::cvtColor(image, image, CV_GRAY2BGR);
  }

  if (image.rows != video_height_ || image.cols != video_width_)
    cv::resize(image, image, cv::Size(video_width_, video_height_));
  memcpy(projection_data_, image.data,
         sizeof(unsigned char) * 3 * image.rows * image.cols);
  projection_changed_ = true;
}

void HpaViewer::DrawQuadtree() {
  std::vector<SemanticSLAM::PointTyped> semantic_map;
  {
    std::lock_guard<std::mutex> lock(mutex_map_);
    semantic_map = semantic_map_;
  }
  const float kBoxWidth = 0.14;
  for (auto &p : semantic_map) {

    glLineWidth(2);
    glColor3f(1, 0, 0);
    glBegin(GL_QUADS);

    switch (p.type) {
    case PARKING_SLOT:
      glColor3f(0.447f, 0.623f, 0.8117f);
      break;
    case DASH:
      glColor3f(1.0f, 0.58f, 0.22f);
      break;
    case LANE:
      glColor3f(1, 0, 1);
      break;
    case ARROW:
      glColor3f(0, 1, 0);
      break;
    default:
      glColor3f(0, 0.5, 0.5);
    }
    glVertex3d(p.point.x() - 0.5 * kBoxWidth, p.point.y() - 0.5 * kBoxWidth, 0);
    glVertex3d(p.point.x() + 0.5 * kBoxWidth, p.point.y() - 0.5 * kBoxWidth, 0);
    glVertex3d(p.point.x() + 0.5 * kBoxWidth, p.point.y() + 0.5 * kBoxWidth, 0);
    glVertex3d(p.point.x() - 0.5 * kBoxWidth, p.point.y() + 0.5 * kBoxWidth, 0);
    glEnd();
  }
}

void HpaViewer::DrawKeyframes() {
  std::vector<Eigen::Matrix3d> kf_world2base;
  {
    std::lock_guard<std::mutex> lock(mutex_keyframe_);
    kf_world2base = keyframe_poses_;
  }

  for (auto world2base : kf_world2base) {
    std::vector<Eigen::Vector3d> base_corners{
        Eigen::Vector3d(6.8, 5.5, 1), Eigen::Vector3d(6.8, -5.5, 1),
        Eigen::Vector3d(-4.2, -5.5, 1), Eigen::Vector3d(-4.2, 5.5, 1)};

    std::vector<Eigen::Vector3d> world_corners;
    for (auto pb : base_corners) {
      auto wp = world2base * pb;
      world_corners.push_back(wp);
    }

    // draw
    glLineWidth(2);
    glColor4f(238.0f / 255, 188.0f / 255, 128.0f / 255, 0.1f);
    glBegin(GL_QUADS);

    for (auto p : world_corners) {
      glVertex3d(p.x(), p.y(), 0);
    }
    glEnd();

    glBegin(GL_LINE_STRIP);
    glLineWidth(2);
    glColor3f(1.0, 1.0, 1.0);

    if (!world_corners.empty())
      world_corners.push_back(world_corners.front());
    for (auto p : world_corners) {
      glVertex3d(p.x(), p.y(), 0);
    }
    glEnd();
  }
}

void HpaViewer::DrawMarkerPoints() {
  std::vector<Eigen::Vector2d> points;
  {
    std::unique_lock<std::mutex> lock(mutex_marker_);
    points = marker_points_;
  }
  glLineWidth(20.0);
  glBegin(GL_LINE_STRIP);
  glColor4f(0, 1, 0, 0.2);
  for (auto p : points) {
    glVertex3f(p.x(), p.y(), 0);
  }
  glEnd();
}

pangolin::OpenGlMatrix HpaViewer::GetCurrentCamMatrix() {

  Eigen::Matrix4d trans_world2cam = Eigen::Matrix4d::Identity();

  {
    std::unique_lock<std::mutex> lock(mutex_pose_);

    trans_world2cam(0, 3) = current_pose_.x();
    trans_world2cam(1, 3) = current_pose_.y();
  }

  pangolin::OpenGlMatrix twc;
  twc.SetIdentity();
  twc.m[12] = trans_world2cam(0, 3);
  twc.m[13] = trans_world2cam(1, 3);
  twc.m[14] = trans_world2cam(2, 3);

  twc.m[0] = trans_world2cam(0, 0);
  twc.m[1] = trans_world2cam(1, 0);
  twc.m[2] = trans_world2cam(2, 0);

  twc.m[4] = trans_world2cam(0, 1);
  twc.m[5] = trans_world2cam(1, 1);
  twc.m[6] = trans_world2cam(2, 1);

  twc.m[8] = trans_world2cam(0, 2);
  twc.m[9] = trans_world2cam(1, 2);
  twc.m[10] = trans_world2cam(2, 2);

  return twc;
}

void HpaViewer::DrawCurrentPose() {

  Eigen::Matrix4d twc = Eigen::Matrix4d::Identity();
  {
    std::unique_lock<std::mutex> lock(mutex_pose_);

    double tx = current_pose_.x();
    double ty = current_pose_.y();
    double theta = current_pose_.z();

    twc(0, 0) = cos(theta);
    twc(0, 1) = -sin(theta);
    twc(1, 0) = sin(theta);
    twc(1, 1) = cos(theta);
    twc(0, 3) = tx;
    twc(1, 3) = ty;
  }
  glPushMatrix();
  glMultMatrixd(twc.data());

  glColor4f(0, 0.67f, 0.9f, 0.5f);
  for (auto &face : simple_car_model_->model_faces_) {
    glLineWidth(4.0);
    glBegin(GL_TRIANGLES);
    auto &p0 = face[0];
    auto &p1 = face[1];
    auto &p2 = face[2];
    glVertex3f(p0.x(), p0.y(), p0.z());
    glVertex3f(p1.x(), p1.y(), p1.z());
    glVertex3f(p2.x(), p2.y(), p2.z());
    glEnd();
  }
  glPopMatrix();
}
