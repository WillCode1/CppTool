#include "viewer/mapdrawer.h"
#include "keyframe.h"
#include "mappoint.h"
#include <mutex>
#include <pangolin/pangolin.h>

namespace FeatureSLAM {

MapDrawer::MapDrawer(Map *pMap, const string &strSettingPath) : map_(pMap) {
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

  //  keyfrm_size_ = fSettings["Viewer.KeyFrameSize"];
  //  keyfrm_line_width_ = fSettings["Viewer.KeyFrameLineWidth"];
  //  graph_line_width_ = fSettings["Viewer.GraphLineWidth"];
  //  point_size_ = fSettings["Viewer.PointSize"];
  //  cam_size_ = fSettings["Viewer.CameraSize"];
  //  cam_line_width_ = fSettings["Viewer.CameraLineWidth"];

  keyfrm_size_ = 1.5f;
  keyfrm_line_width_ = 1.0f;
  graph_line_width_ = 1.0f;
  point_size_ = 1.0f;
  cam_size_ = 1.5f;
  cam_line_width_ = 1.0f;
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

void MapDrawer::DrawMapPoints() {
  const vector<MapPoint *> &mps = map_->GetAllMapPoints();
  const std::vector<MapPoint *> &ref_mps = map_->GetReferenceMapPoints();
  std::set<MapPoint *> ref_mps_list(ref_mps.begin(), ref_mps.end());

  if (mps.empty())
    return;

  // draw old map points

  glPointSize(point_size_);
  glBegin(GL_POINTS);
  glColor3f(0.255f, 0.412f, 1.0f);

  for (auto mp : mps) {
    if (mp->isBad() || ref_mps_list.count(mp))
      continue;

    cv::Mat pw = mp->GetWorldPos();
    glVertex3f(pw.at<float>(0), pw.at<float>(1), pw.at<float>(2));
  }
  glEnd();

  // draw reference map points
  glPointSize(point_size_);
  glBegin(GL_POINTS);
  glColor3f(0, 1, 0);

  int ref_mp_num = ref_mps.size();

  for (int i = 0; i < ref_mps.size(); i++) {
    auto mp = ref_mps[i];
    if (mp->isBad())
      continue;

    auto color = MakeJet3B(i * 0.5 / ref_mp_num);
    cv::Mat pw = mp->GetWorldPos();
    glColor3f(color[0] / 255.0f, color[1] / 255.0f, color[2] / 255.0f);
    glVertex3f(pw.at<float>(0), pw.at<float>(1), pw.at<float>(2));
  }
  glEnd();
}

void DrawCircle(float x, float y, const cv::Vec3b &color) {
  glColor3f(color[0] / 255.0f, color[1] / 255.0f, color[2] / 255.0f);
  glLineWidth(2.0);
  glBegin(GL_LINE_STRIP);
  float r = 0.2f;
  for (float i = 0; i < 2 * M_PI; i = i + 0.2f) {

    float px = x + r * sin(i);
    float py = y + r * cos(i);
    glVertex3f(px, py, 0);
  }
  glEnd();
}

void MapDrawer::DrawOdometry() {

  std::vector<MCKeyFrame *> mc_keyfrms = map_->GetAllMCKeyFrames();

  std::sort(mc_keyfrms.begin(), mc_keyfrms.end(), MCKeyFrame::lId);

  if (mc_keyfrms.size() < 2)
    return;
  glColor3f(252.0f / 255.0f, 212.0f / 255.0f, 54.0f / 255.0f);
  glLineWidth(2.0);
  glBegin(GL_LINE_STRIP);
  for (auto mc_keyfrm : mc_keyfrms) {
    glVertex3f(mc_keyfrm->odometry_vector_.x(), mc_keyfrm->odometry_vector_.y(),
               0);
  }
  glEnd();

  if (!mc_keyfrms.empty()) {
    auto last_mc_keyfrm = mc_keyfrms[mc_keyfrms.size() - 1];
    auto last_odom = last_mc_keyfrm->odometry_vector_;
    DrawCircle(last_odom.x(), last_odom.y(), cv::Vec3b(252, 212, 54));
    auto keyfrm_pose = last_mc_keyfrm->GetPose();
    auto cal_odom = MultiCamExt::GetInstance().CameraPose2Baselink(
        keyfrm_pose, FRONT_CAMERA);
    DrawCircle(cal_odom.at<float>(0, 3), cal_odom.at<float>(1, 3),
               cv::Vec3b(54, 212, 252));
  }
}

void MapDrawer::DrawAxis() {
  glColor3f(1, 0, 0);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(3, 0, 0);
  glColor3f(0, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 3, 0);
  glColor3f(0, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 3);
  glEnd();
}

void MapDrawer::DrawKeyFrames(const bool draw_keyfrm, const bool draw_graph) {
  const float &w = keyfrm_size_ * 0.15f;
  const float h = w * 0.75f;
  const float z = w * 0.6f;

  std::vector<KeyFrame *> keyfrms = map_->GetAllKeyFrames();
  std::vector<KeyFrame *> ba_keyfrms = map_->GetBAKeyFrames();
  std::sort(ba_keyfrms.begin(), ba_keyfrms.end(), KeyFrame::lId);
  std::vector<KeyFrame *> ba_fixed_keyfrms = map_->GetBAKeyFramesFixed();
  std::sort(ba_fixed_keyfrms.begin(), ba_fixed_keyfrms.end(), KeyFrame::lId);

  if (draw_keyfrm) {

    for (auto keyfrm : keyfrms) {
      cv::Mat Twc = keyfrm->GetPoseInverse().t();
      glPushMatrix();
      glMultMatrixf(Twc.ptr<GLfloat>(0));
      glLineWidth(keyfrm_line_width_);

      auto it = find(ba_keyfrms.begin(), ba_keyfrms.end(), keyfrm);
      auto it_fixed =
          find(ba_fixed_keyfrms.begin(), ba_fixed_keyfrms.end(), keyfrm);
      if (it != ba_keyfrms.end()) {
        auto color = MakeJet3B(
            (it - ba_keyfrms.begin()) * 0.55f / ba_keyfrms.size() + 0.2f);
        glColor3f(color[0] / 255.0f, color[1] / 255.0f, color[2] / 255.0f);
        glLineWidth(keyfrm_line_width_ * 2);
      } else if (it_fixed != ba_fixed_keyfrms.end()) {
        glColor3f(0.0f, 0.7490196078431373f, 1.0f);
        glLineWidth(keyfrm_line_width_);
      } else {
        glColor3f(176.0 / 255, 196.0 / 255, 222.0 / 255);
        glLineWidth(keyfrm_line_width_);
      }

      //      int cam_index = pkf->camera_index_;
      //      switch (cam_index) {
      //      case LEFT_CAMERA:
      //        glColor3f(0.3565, 0.7962, 0.3566);
      //        break;
      //      case FRONT_CAMERA:
      //        glColor3f(0.255f, 0.412f, 1.0f);
      //        break;
      //      case RIGHT_CAMERA:
      //        glColor3f(0.722, 0.525, 0.043);
      //        break;
      //      case BACK_CAMERA:
      //        glColor3f(0.592, 0.267, 0.745);
      //        break;
      //      }

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
  }

  if (draw_graph) {
    glLineWidth(graph_line_width_);
    glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
    glBegin(GL_LINES);

    for (size_t i = 0; i < keyfrms.size(); i++) {

      // draw cosvisible keyframes

      const vector<KeyFrame *> cos_keyfrms =
          keyfrms[i]->GetCovisiblesByWeight(100);
      cv::Mat camera_center = keyfrms[i]->GetCameraCenter();
      if (!cos_keyfrms.empty()) {

        for (auto cos_kf : cos_keyfrms) {
          if (cos_kf->isBad() < keyfrms[i]->id_)
            continue;
          cv::Mat camera_center2 = cos_kf->GetCameraCenter();

          if (keyfrms[i]->camera_index_ != cos_kf->camera_index_) {
            glColor4f(1.0f, 0.0f, 0.0f, 0.3f);
            glVertex3f(camera_center.at<float>(0), camera_center.at<float>(1),
                       camera_center.at<float>(2));
            glVertex3f(camera_center2.at<float>(0), camera_center2.at<float>(1),
                       camera_center2.at<float>(2));
          } else {

            glColor4f(0, 1.0, 0, 0.6);
            glVertex3f(camera_center.at<float>(0), camera_center.at<float>(1),
                       camera_center.at<float>(2));
            glVertex3f(camera_center2.at<float>(0), camera_center2.at<float>(1),
                       camera_center2.at<float>(2));
          }
        }
      }

      // draw parent keyframes

      KeyFrame *parent_keyfrm = keyfrms[i]->GetParent();
      if (parent_keyfrm) {
        cv::Mat camera_center_parent = parent_keyfrm->GetCameraCenter();

        if (parent_keyfrm->camera_index_ != keyfrms[i]->camera_index_)
          glColor4f(1.0f, 0.0f, 0.0f, 0.3f);
        else {
          glColor4f(0, 1.0, 0, 0.6);
        }

        glVertex3f(camera_center.at<float>(0), camera_center.at<float>(1),
                   camera_center.at<float>(2));
        glVertex3f(camera_center_parent.at<float>(0),
                   camera_center_parent.at<float>(1),
                   camera_center_parent.at<float>(2));
      }
    }

    glEnd();
  }
}
void MapDrawer::DrawCurrentCamera(cv::Mat trans_w2frm,
                                  const std::vector<cv::Mat> &cam_ext) {
  const float &w = cam_size_ * 0.17f;
  const float h = w * 0.75f;
  const float z = w * 0.6f;

  glColor3f(1.0f, 1.0f, 1.0f);

  for (int cam_index = 0; cam_index < 4; cam_index++) {

    cv::Mat trans_w2c = trans_w2frm * cam_ext[cam_index].inv();
    cv::Mat trans_w2c_t = trans_w2c.t();
    glPushMatrix();
    glMultMatrixf(trans_w2c_t.ptr<GLfloat>(0));
    glLineWidth(1.5 * cam_line_width_);

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
}

void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw) {
  unique_lock<mutex> lock(mutex_cam_);
  cam_pose_ = Tcw.clone();
}

pangolin::OpenGlMatrix MapDrawer::ToGLMatrix(cv::Mat T) {

  cv::Mat rwc = T.rowRange(0, 3).colRange(0, 3).t();
  cv::Mat twc = -rwc * T.rowRange(0, 3).col(3);

  pangolin::OpenGlMatrix m;
  m.SetIdentity();
  m.m[0] = rwc.at<float>(0, 0);
  m.m[1] = rwc.at<float>(1, 0);
  m.m[2] = rwc.at<float>(2, 0);
  m.m[3] = 0.0;

  m.m[4] = rwc.at<float>(0, 1);
  m.m[5] = rwc.at<float>(1, 1);
  m.m[6] = rwc.at<float>(2, 1);
  m.m[7] = 0.0;

  m.m[8] = rwc.at<float>(0, 2);
  m.m[9] = rwc.at<float>(1, 2);
  m.m[10] = rwc.at<float>(2, 2);
  m.m[11] = 0.0;

  m.m[12] = twc.at<float>(0);
  m.m[13] = twc.at<float>(1);
  m.m[14] = twc.at<float>(2);
  m.m[15] = 1.0;

  return m;
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &m,
                                             cv::Mat &Twc) {
  if (!cam_pose_.empty()) {
    cv::Mat rwc(3, 3, CV_32F);
    cv::Mat twc(3, 1, CV_32F);
    {
      unique_lock<mutex> lock(mutex_cam_);
      Twc = cam_pose_.inv();
      rwc = cam_pose_.rowRange(0, 3).colRange(0, 3).t();
      twc = -rwc * cam_pose_.rowRange(0, 3).col(3);
    }

    m.m[0] = rwc.at<float>(0, 0);
    m.m[1] = rwc.at<float>(1, 0);
    m.m[2] = rwc.at<float>(2, 0);
    m.m[3] = 0.0;

    m.m[4] = rwc.at<float>(0, 1);
    m.m[5] = rwc.at<float>(1, 1);
    m.m[6] = rwc.at<float>(2, 1);
    m.m[7] = 0.0;

    m.m[8] = rwc.at<float>(0, 2);
    m.m[9] = rwc.at<float>(1, 2);
    m.m[10] = rwc.at<float>(2, 2);
    m.m[11] = 0.0;

    m.m[12] = twc.at<float>(0);
    m.m[13] = twc.at<float>(1);
    m.m[14] = twc.at<float>(2);
    m.m[15] = 1.0;
  } else {
    m.SetIdentity();
    Twc = cv::Mat::eye(4, 4, CV_32F);
  }
}
}
