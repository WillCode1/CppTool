#include "mapviewer.h"
#include "frame.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>

using namespace FeatureSLAM;

MapViewer::MapViewer(Map *map, std::vector<cv::Mat> cam_extrinsic)
    : map_(map), tracked_inliers_(0), state_(false), show_frame_(false),
      finish_request_(false), is_finish_(false) {

  std::vector<MapPoint *> all_mps = map_->GetAllMapPoints();

  for (auto mp : all_mps) {
    if (mp->isBad())
      continue;
    auto pos = Converter::toVector3d(mp->GetWorldPos());
    map_points_.push_back(pos);
  }

  cam_extrinsic_.resize(4);
  for (int i = 0; i < 4; i++) {
    cam_extrinsic_[i] = cam_extrinsic[i];
  }
  cam_center_.resize(4);
  Tcw_ = cv::Mat();
  tracked_mappoints_.resize(4);
  frame_mappoints_.resize(4);
  cur_mcfrm_keypts_.resize(4);

  Eigen::MatrixXf m(4, 3);
  m << 0.3565, 0.7962, 0.3566, 0.255, 0.412, 1.0, 0.722, 0.525, 0.043, 0.592,
      0.267, 0.745;
  ray_color_ = m;

  simple_car_model_ = std::make_shared<SimpleCarModel>();
  front2wheel_ = MultiCamExt::GetInstance().GetFront2Wheel();
}

void MapViewer::Run() {

  pangolin::CreateWindowAndBind("Map Localization", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  double viewpoint_f = 2000;
  double viewpoint_x = 0;
  double viewpoint_y = 0;
  double viewpoint_z = 100;

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(175));
  pangolin::Var<bool> menu_draw_mappoint("menu.Draw Map Points", true, true);
  pangolin::Var<bool> menu_draw_rays("menu.Draw Tracking Rays ", true, true);
  pangolin::Var<bool> menu_draw_current_cam("menu.Draw Current Cam", true,
                                            true);
  pangolin::Var<bool> menu_draw_graph("menu.DrawGraph ", false, true);
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, viewpoint_f, viewpoint_f, 512, 389,
                                 0.1, 1000),
      pangolin::ModelViewLookAt(viewpoint_x, viewpoint_y, viewpoint_z, 0, 0, 0,
                                0.0, -1.0, 0.0));

  pangolin::View &d_cam =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0 / 768.0)
          .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::DataLog log;

  // Optionally add named labels
  std::vector<std::string> labels{"num inliers"};
  log.SetLabels(labels);

  pangolin::Plotter plotter(&log, 0.0f, 2000, 0.0f, 1000.0f, 30, 0.5);
  plotter.SetBounds(0.0, 0.25, 0.75, 1.0)
      .SetLock(pangolin::LockBottom, pangolin::LockRight);

  pangolin::DisplayBase().AddDisplay(plotter);
  plotter.SetBackgroundColour(pangolin::Colour(0.1f, 0.1f, 0.1f));
  plotter.SetAxisColour(pangolin::Colour(0, 1, 0));
  plotter.Track("$i", "");

  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();

  while (1) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    if (menu_draw_mappoint) {
      DrawWholeMap();
    }

    if (menu_draw_rays) {
      DrawRays();
    }
    if (menu_draw_current_cam) {
      DrawCurrentCam();
    }
    DrawFrame();

    if (menu_draw_graph)
      DrawGraph();

    plotter.Activate();
    log.Log(tracked_inliers_);

    pangolin::FinishFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(3));

    if (CheckFinish()) {
      break;
    }
  }
  SetFinish();
}
void MapViewer::DrawGraph() {
  const vector<KeyFrame *> all_keyfrms = map_->GetAllKeyFrames();
  glLineWidth(1);
  glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
  glBegin(GL_LINES);
  for (size_t i = 0; i < all_keyfrms.size(); i++) {
    const vector<KeyFrame *> cos_keyfrms =
        all_keyfrms[i]->GetCovisiblesByWeight(100);
    cv::Mat cam_center = all_keyfrms[i]->GetCameraCenter();
    if (!cos_keyfrms.empty()) {
      for (vector<KeyFrame *>::const_iterator vit = cos_keyfrms.begin(),
                                              vend = cos_keyfrms.end();
           vit != vend; vit++) {
        if ((*vit)->id_ < all_keyfrms[i]->id_)
          continue;
        cv::Mat cam_center2 = (*vit)->GetCameraCenter();
        glVertex3f(cam_center.at<float>(0), cam_center.at<float>(1),
                   cam_center.at<float>(2));
        glVertex3f(cam_center2.at<float>(0), cam_center2.at<float>(1),
                   cam_center2.at<float>(2));
      }
    }
    KeyFrame *parent_keyfrm = all_keyfrms[i]->GetParent();
    if (parent_keyfrm) {
      cv::Mat cam_center_parent = parent_keyfrm->GetCameraCenter();
      glVertex3f(cam_center.at<float>(0), cam_center.at<float>(1),
                 cam_center.at<float>(2));
      glVertex3f(cam_center_parent.at<float>(0), cam_center_parent.at<float>(1),
                 cam_center_parent.at<float>(2));
    }
  }

  glEnd();
}

bool MapViewer::CheckFinish() {
  std::unique_lock<std::mutex> lock(mutex_finishe_);
  return finish_request_;
}

void MapViewer::SetFinish() {
  std::unique_lock<std::mutex> lock(mutex_finishe_);
  is_finish_ = true;
}

inline cv::Vec3b MakeJet3B(float id) {
  if (id <= 0)
    return cv::Vec3b(128, 0, 0);
  if (id >= 1)
    return cv::Vec3b(0, 0, 128);

  int icP = (id * 8);
  float ifP = (id * 8) - icP;

  if (icP == 0)
    return cv::Vec3b(255 * (0.5 + 0.5 * ifP), 0, 0);
  if (icP == 1)
    return cv::Vec3b(255, 255 * (0.5 * ifP), 0);
  if (icP == 2)
    return cv::Vec3b(255, 255 * (0.5 + 0.5 * ifP), 0);
  if (icP == 3)
    return cv::Vec3b(255 * (1 - 0.5 * ifP), 255, 255 * (0.5 * ifP));
  if (icP == 4)
    return cv::Vec3b(255 * (0.5 - 0.5 * ifP), 255, 255 * (0.5 + 0.5 * ifP));
  if (icP == 5)
    return cv::Vec3b(0, 255 * (1 - 0.5 * ifP), 255);
  if (icP == 6)
    return cv::Vec3b(0, 255 * (0.5 - 0.5 * ifP), 255);
  if (icP == 7)
    return cv::Vec3b(0, 0, 255 * (1 - 0.5 * ifP));
  return cv::Vec3b(255, 255, 255);
}

void MapViewer::DrawWholeMap() {

  glPointSize(2);
  glBegin(GL_POINTS);
  glColor3f(1, 1, 1);
  for (auto pos : map_points_) {

    auto color = MakeJet3B((pos.z()) / 3.f);
    glColor3f(color[0] / 255.f, color[1] / 255.f, color[2] / 255.f);
    glVertex3f(pos.x(), pos.y(), pos.z());
  }
  glEnd();
}

void MapViewer::DrawGrid() {
  glLineWidth(0.4);
  glBegin(GL_LINES);
  glColor3f(0, 0, 1.0);
  for (int i = -10; i <= 10; i++) {
    glVertex3f(10, i, 0);
    glVertex3f(-10, i, 0);

    glVertex3f(i, 10, 0);
    glVertex3f(i, -10, 0);
  }

  glEnd();
}

void MapViewer::DrawRays() {
  if (!state_)
    return;
  unique_lock<mutex> lock(mutex_tracked_points_);
  glLineWidth(0.5);
  glBegin(GL_LINES);
  glColor4f(0.498039f, 1, 0.498039f, 0.1f);
  for (size_t cam_index = 0; cam_index < 4; cam_index++) {

    if (cam_center_[cam_index].empty())
      continue;

    for (auto mp : tracked_mappoints_[cam_index]) {
      if (mp) {
        if (mp->isBad())
          continue;
        cv::Mat pos = mp->GetWorldPos();

        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
        glVertex3f(cam_center_[cam_index].at<float>(0),
                   cam_center_[cam_index].at<float>(1),
                   cam_center_[cam_index].at<float>(2));
      }
    }
  }

  glEnd();
}

void MapViewer::DrawTrackedMapPoints() {
  if (!state_)
    return;
  unique_lock<mutex> lock(mutex_tracked_points_);
  glPointSize(4);
  glBegin(GL_POINTS);
  for (int cam_index = 0; cam_index < 4; cam_index++) {

    glColor4f(ray_color_(cam_index, 0), ray_color_(cam_index, 1),
              ray_color_(cam_index, 2), 0.7);

    for (auto mp : tracked_mappoints_[cam_index]) {
      if (mp) {
        if (mp->isBad())
          continue;
        cv::Mat p = mp->GetWorldPos();
        glVertex3f(p.at<float>(0), p.at<float>(1), p.at<float>(2));
      }
    }
  }

  glEnd();
}

void MapViewer::DrawCurrentCam() {
  if (!state_)
    return;
  unique_lock<mutex> lock(mutex_tracked_points_);
  const float &w = 0.3f;
  const float h = w * 0.75f;
  const float z = w * 0.6f;
  if (!Tcw_.empty()) {
    for (int cam_index = 0; cam_index < 4; cam_index++) {
      cv::Mat twc = Tcw_.inv() * cam_extrinsic_[cam_index].inv();
      cv::Mat twc_t = twc.t();
      glPushMatrix();
      glMultMatrixf(twc_t.ptr<GLfloat>(0));
      glLineWidth(3);
      glColor3f(ray_color_(cam_index, 0), ray_color_(cam_index, 1),
                ray_color_(cam_index, 2));
      glBegin(GL_LINES);
      glVertex3f(0, 0, 0);
      glVertex3f(w, h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(w, -h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(-w, -h, z);
      glVertex3f(0, 0, 0);
      glVertex3f(-w, h, z);
      glVertex3f(w, h, z);
      glVertex3f(w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(-w, -h, z);

      glVertex3f(-w, h, z);
      glVertex3f(w, h, z);
      glVertex3f(-w, -h, z);
      glVertex3f(w, -h, z);
      glEnd();
      glPopMatrix();
    }

    cv::Mat two = Tcw_.inv() * front2wheel_;
    cv::Mat two_t = two.t();
    glPushMatrix();
    glMultMatrixf(two_t.ptr<GLfloat>(0));

    glColor4f(0, 0.67, 0.94, 0.5);
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
}

void MapViewer::Update(MultiCamTracking *mtracker) {
  std::unique_lock<mutex> lock(mutex_tracked_points_);

  im = mtracker->img_gray_.clone();
  Tcw_ = mtracker->mCurrentMcFrame.tcw_;
  if (mtracker->state_ != MultiCamTracking::OK) {
    state_ = false;
    return;
  }
  state_ = true;
  show_frame_ = true;
  for (int cam_index = 0; cam_index < 4; cam_index++) {
    tracked_mappoints_[cam_index].assign(
        mtracker->mCurrentMcFrame.frames_[cam_index].mappoints_.begin(),
        mtracker->mCurrentMcFrame.frames_[cam_index].mappoints_.end());
    frame_mappoints_[cam_index] = vector<bool>(
        mtracker->mCurrentMcFrame.frames_[cam_index].num_feature_, false);
    cur_mcfrm_keypts_[cam_index] =
        mtracker->mCurrentMcFrame.frames_[cam_index].keypts_;
    for (int i = 0;
         i < mtracker->mCurrentMcFrame.frames_[cam_index].num_feature_; i++) {
      MapPoint *mp = mtracker->mCurrentMcFrame.frames_[cam_index].mappoints_[i];
      if (mp) {
        if (!mtracker->mCurrentMcFrame.frames_[cam_index].outliers_[i]) {
          if (mp->Observations() > 0)
            frame_mappoints_[cam_index][i] = true;
          else
            frame_mappoints_[cam_index][i] = false;
        }
      }
    }
    cv::Mat twci = Tcw_.inv() * cam_extrinsic_[cam_index].inv();
    cam_center_[cam_index] = twci.rowRange(0, 3).col(3);
  }

  tracked_inliers_ = mtracker->num_matches_inliers_;
}

void MapViewer::RequestFinish() {
  std::unique_lock<std::mutex> lock(mutex_finishe_);
  finish_request_ = true;
}

bool MapViewer::IsFinish() {
  std::unique_lock<std::mutex> lock(mutex_finishe_);
  return is_finish_;
}

void MapViewer::DrawFrame() {
  if (im.empty())
    return;
  vector<vector<cv::KeyPoint>> cur_mcfrm_keypts;
  vector<vector<bool>> frame_mappoint;

  if (state_ && show_frame_) {
    cv::cvtColor(im, im, CV_GRAY2BGR);
    {
      std::unique_lock<mutex> lock(mutex_tracked_points_);
      cur_mcfrm_keypts = cur_mcfrm_keypts_;
      frame_mappoint = frame_mappoints_;
    }

    const float r = 5;
    for (int cam_index = 0; cam_index < 4; cam_index++) {
      const int n = frame_mappoint[cam_index].size();
      for (int i = 0; i < n; i++) {
        float xdevi = 0;
        float ydevi = 0;

        if (cam_index == LEFT_CAMERA) {
          xdevi = kImgWidth;
          ydevi = 0.0;
        } else if (cam_index == FRONT_CAMERA) {
          xdevi = 0.0;
          ydevi = 0.0;
        } else if (cam_index == RIGHT_CAMERA) {
          xdevi = kImgWidth;
          ydevi = kImgUpBound - kImgLowBound;
        }

        else if (cam_index == BACK_CAMERA) {
          xdevi = 0.0;
          ydevi = kImgUpBound - kImgLowBound;
        }

        if (frame_mappoint[cam_index][i]) {
          cv::Point2f pt1, pt2;
          cv::Point2f pt0;
          pt0.x = xdevi + cur_mcfrm_keypts[cam_index][i].pt.x;
          pt0.y = ydevi + cur_mcfrm_keypts[cam_index][i].pt.y;

          pt1.x = cur_mcfrm_keypts[cam_index][i].pt.x - r + xdevi;
          pt1.y = cur_mcfrm_keypts[cam_index][i].pt.y - r + ydevi;
          pt2.x = cur_mcfrm_keypts[cam_index][i].pt.x + r + xdevi;
          pt2.y = cur_mcfrm_keypts[cam_index][i].pt.y + r + ydevi;

          cv::circle(im, pt0, 2, cv::Scalar(11, 239, 255), -1);
        }
      }
    }

    std::string s0 = std::string(" Left Camera ");
    std::string s1 = std::string(" Front Camera ");
    std::string s2 = std::string(" Right Camera ");
    std::string s3 = std::string(" Back Camera ");
    cv::putText(im, s1, cv::Point(15 + kImgWidth / 3, 45),
                cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
    cv::putText(im, s0, cv::Point2f(kImgWidth * 4 / 3 + 15, 45),
                cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
    cv::putText(im, s2, cv::Point2f(kImgWidth * 4 / 3 + 15,
                                    kImgUpBound - kImgLowBound + 45),
                cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
    cv::putText(im, s3, cv::Point2f(15 + kImgWidth / 3,
                                    kImgUpBound - kImgLowBound + 45),
                cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);
    show_frame_ = false;
  }
  cv::resize(im, im, cv::Size(1024, 690));
  cv::imshow("current frame ", im);
  cv::waitKey(10);
}
