#include "viewer/viewer.h"
#include <mutex>
#include <pangolin/pangolin.h>

namespace FeatureSLAM {

Viewer::Viewer(System *system, FrameDrawer *frame_drawer, MapDrawer *pMapDrawer,
               Tracking *tracker, const string &setting_file)
    : system_(system), frame_drawer_(frame_drawer), map_drawer_(pMapDrawer),
      tracker_(tracker), finish_requested_(false), is_finished_(true),
      is_stopped_(false), stop_requested_(false) {
  cv::FileStorage settings(setting_file, cv::FileStorage::READ);

  image_width_ = settings["Camera.width"];
  img_height_ = settings["Camera.height"];
  if (image_width_ < 1 || img_height_ < 1) {
    image_width_ = 640;
    img_height_ = 480;
  }

  //  viewpoint_.x() = settings["Viewer.ViewpointX"];
  //  viewpoint_.y() = settings["Viewer.ViewpointY"];
  //  viewpoint_.z() = settings["Viewer.ViewpointZ"];
  //  viewpoint_focal_length_ = settings["Viewer.ViewpointF"];

  viewpoint_ = Vec3_t(0, 0, 100);
  viewpoint_focal_length_ = 2000;

  // load camera extrinsic parameter
  cv::Mat tcam_01 = settings["Tcam_01"].mat();
  cv::Mat tcam_11 = settings["Tcam_11"].mat();
  cv::Mat tcam_21 = settings["Tcam_21"].mat();
  cv::Mat tcam_31 = settings["Tcam_31"].mat();
  cam_ext_ = std::vector<cv::Mat>{tcam_01, tcam_11, tcam_21, tcam_31};

  settings.release();
}

void Viewer::Run() {

  const int kUiWidth = 175;
  is_finished_ = false;
  pangolin::CreateWindowAndBind(" Map Viewer", 1280, 800);
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("config").SetBounds(0.0, 1.0, 0.0,
                                            pangolin::Attach::Pix(kUiWidth));
  pangolin::Var<bool> menu_follow_cam("config.Follow Camera", false, true);
  pangolin::Var<bool> menu_show_points("config.Show Points", true, true);
  pangolin::Var<bool> menu_show_keyfrm("config.Show KeyFrames", true, true);
  pangolin::Var<bool> menu_draw_graph("config.Draw Graph", true, true);
  pangolin::Var<bool> menu_darwaxis("config.DrawAxis", true, true);
  pangolin::Var<bool> menu_drawodom("config.Show Odom", true, true);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, viewpoint_focal_length_,
                                 viewpoint_focal_length_, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(viewpoint_.x(), viewpoint_.y(), viewpoint_.z(),
                                0, 0, 0, 1.0, 0.0, 0.0));

  pangolin::CreateDisplay()
      .SetBounds(0.0, 0.25, pangolin::Attach::Pix(kUiWidth), 1.0, true)
      .SetLayout(pangolin::LayoutEqual);

  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175),
                                         1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::OpenGlMatrix Twc;
  cv::Mat mTwc = cv::Mat::eye(4, 4, CV_32F);
  Twc.SetIdentity();
  cv::namedWindow("current frame ");
  bool follow_cam = true;
  while (1) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    map_drawer_->GetCurrentOpenGLCameraMatrix(Twc, mTwc);
    if (menu_follow_cam && follow_cam) {
      s_cam.Follow(Twc);
    } else if (menu_follow_cam && !follow_cam) {
      s_cam.SetModelViewMatrix(
          pangolin::ModelViewLookAt(viewpoint_.x(), viewpoint_.y(),
                                    viewpoint_.z(), 0, 0, 0, 0.0, -1.0, 0.0));
      s_cam.Follow(Twc);
      follow_cam = true;
    } else if (!menu_follow_cam && follow_cam) {
      follow_cam = false;
    }

    d_cam.Activate(s_cam);

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    map_drawer_->DrawCurrentCamera(mTwc, cam_ext_);
    //    map_drawer_->DrawTrajectory();

    if (menu_drawodom)
      map_drawer_->DrawOdometry();

    map_drawer_->DrawKeyFrames(menu_show_keyfrm, menu_draw_graph);

    if (menu_show_points)
      map_drawer_->DrawMapPoints();

    if (menu_darwaxis) {
      map_drawer_->DrawAxis();
    }

    cv::Mat im = frame_drawer_->DrawFrame();

    cv::imshow("current frame ", im);
    cv::waitKey(1);

    pangolin::FinishFrame();

    if (Stop()) {
      while (isStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
      }
    }

    if (CheckFinish())
      break;
  }

  SetFinish();
}

void Viewer::RequestFinish() {
  unique_lock<mutex> lock(mutex_finish_);
  finish_requested_ = true;
}

bool Viewer::CheckFinish() {
  unique_lock<mutex> lock(mutex_finish_);
  return finish_requested_;
}

void Viewer::SetFinish() {
  unique_lock<mutex> lock(mutex_finish_);
  is_finished_ = true;
}

bool Viewer::isFinished() {
  unique_lock<mutex> lock(mutex_finish_);
  return is_finished_;
}

void Viewer::RequestStop() {
  unique_lock<mutex> lock(mutex_stop_);
  if (!is_stopped_)
    stop_requested_ = true;
}

bool Viewer::isStopped() {
  unique_lock<mutex> lock(mutex_stop_);
  return is_stopped_;
}

bool Viewer::Stop() {
  unique_lock<mutex> lock(mutex_stop_);
  unique_lock<mutex> lock2(mutex_finish_);

  if (finish_requested_)
    return false;
  else if (stop_requested_) {
    is_stopped_ = true;
    stop_requested_ = false;
    return true;
  }

  return false;
}

void Viewer::Release() {
  unique_lock<mutex> lock(mutex_stop_);
  is_stopped_ = false;
}
}
