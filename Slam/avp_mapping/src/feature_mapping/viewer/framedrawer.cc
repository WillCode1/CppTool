#include "viewer/framedrawer.h"
#include "tracking.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mutex>

namespace FeatureSLAM {

const int kWindowWidth = 1980;
const int kWindowHeight = 360;

unsigned long int GetCurrentSec() {
  std::time_t nowtime;
  struct tm *p;
  time(&nowtime);
  p = localtime(&nowtime);
  char name_prefix[100];
  sprintf(name_prefix, "%04d-%02d-%02d-%02d-%02d-%02d", p->tm_year + 1900,
          p->tm_mon + 1, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);

  return p->tm_hour * 3600 + p->tm_min * 60 + p->tm_sec;
}

FrameDrawer::FrameDrawer(Map *map) : map_(map) {
  state_ = Tracking::SYSTEM_NOT_READY;
  image_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  start_time_sec_ = GetCurrentSec();
}

cv::Mat FrameDrawer::DrawFrame() {
  cv::Mat im;
  vector<cv::KeyPoint>
      init_keypts; // Initialization: KeyPoints in reference frame
  vector<int>
      matches; // Initialization: correspondeces with reference keypoints
  vector<cv::KeyPoint> cur_keypts; // KeyPoints in current frame
  int state;                       // Tracking state

  vector<vector<cv::KeyPoint>> cur_mc_keypts;
  vector<vector<bool>> map_point;

  {
    unique_lock<mutex> lock(mutex_);
    state = state_;
    if (state_ == Tracking::SYSTEM_NOT_READY)
      state_ = Tracking::NO_IMAGES_YET;

    image_.copyTo(im);

    if (state_ == Tracking::NOT_INITIALIZED) {
      cur_keypts = cur_keypts_;
      init_keypts = init_keypts_;
      matches = init_matches_;
    } else if (state_ == Tracking::LOST) {
      cur_keypts = cur_keypts_;
    } else if (state_ == Tracking::OK) {

      cur_mc_keypts = cur_mc_keypts_;
      map_point = map_point_;
    }
  }                      // destroy scoped mutex -> release mutex
  if (im.channels() < 3) // this should be always true
    cvtColor(im, im, CV_GRAY2BGR);

  if (state == Tracking::NOT_INITIALIZED) // INITIALIZING
  {
    for (unsigned int i = 0; i < matches.size(); i++) {
      if (matches[i] >= 0) {
        cv::Point2f pt1 = init_keypts[i].pt;
        pt1.x += kImgWidth;
        cv::Point2f pt2 = cur_keypts[matches[i]].pt;
        pt2.x += kImgWidth;
        cv::line(im, pt1, pt2, cv::Scalar(0, 255, 0));
      }
    }
  } else if (state == Tracking::OK) {
    mnTracked = 0;
    const float r = 5;
    for (int cam_index = 0; cam_index < 4; cam_index++) {
      const int n = cur_mc_keypts[cam_index].size();
      for (int i = 0; i < n; i++) {

        float xdevi = 0;
        float ydevi = 0;

        if (cam_index == LEFT_CAMERA) {
          xdevi = 0.0;
          ydevi = 0.0;
        } else if (cam_index == FRONT_CAMERA) {
          xdevi = kImgWidth * 1.0;
          ydevi = 0.0;
        } else if (cam_index == RIGHT_CAMERA) {
          xdevi = kImgWidth * 2.0;
          ydevi = 0.0;
        }

        else if (cam_index == BACK_CAMERA) {
          xdevi = kImgWidth * 3.0;
          ydevi = 0.0;
        }

        if (map_point[cam_index][i]) {
          cv::Point2f pt1, pt2;
          cv::Point2f pt0;
          pt0.x = xdevi + cur_mc_keypts[cam_index][i].pt.x;
          pt0.y = ydevi + cur_mc_keypts[cam_index][i].pt.y;

          pt1.x = cur_mc_keypts[cam_index][i].pt.x - r + xdevi;
          pt1.y = cur_mc_keypts[cam_index][i].pt.y - r + ydevi;
          pt2.x = cur_mc_keypts[cam_index][i].pt.x + r + xdevi;
          pt2.y = cur_mc_keypts[cam_index][i].pt.y + r + ydevi;

          cv::circle(im, pt0, 2, cv::Scalar(0, 255, 0), -1);
          mnTracked++;
        }
      }
    }
  }
  cv::Mat img_with_info;
  cv::resize(im, im, cv::Size(kWindowWidth, kWindowHeight));
  DrawTextInfo(im, state, img_with_info);
  return img_with_info;
}

void FrameDrawer::DrawTextInfo(cv::Mat &im, int state, cv::Mat &im_text) {

  stringstream s;
  if (state == Tracking::NO_IMAGES_YET)
    s << " WAITING FOR IMAGES";
  else if (state == Tracking::NOT_INITIALIZED)
    s << " TRYING TO INITIALIZE ";
  else if (state == Tracking::OK) {

    s << "Mapping |  ";

    int num_keyfrms = map_->KeyFramesInMap();
    int num_mappoints = map_->MapPointsInMap();
    s << "KFs: " << num_keyfrms << ", MPs: " << num_mappoints
      << ", Matches: " << mnTracked;
    s << " | Time Consuming(s): " << GetCurrentSec() - start_time_sec_;

  } else if (state == Tracking::LOST) {
    s << " TRACK LOST. TRYING TO RELOCALIZE ";
  } else if (state == Tracking::SYSTEM_NOT_READY) {
    s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
  }

  int baseline = 0;
  cv::Size text_size =
      cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

  im_text = cv::Mat(im.rows + text_size.height + 10, im.cols, im.type());
  im.copyTo(
      im_text.rowRange(text_size.height + 10, im.rows + text_size.height + 10)
          .colRange(0, im.cols));
  im_text.rowRange(0, text_size.height + 10) =
      cv::Mat::zeros(text_size.height + 10, im.cols, im.type());
  cv::putText(im_text, s.str(), cv::Point(5, text_size.height + 5),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);

  std::string left_camera("Left Camera");
  std::string front_camera("Front Camera");
  std::string right_camera("Right Camera");
  std::string back_camera("Back Camera");

  int single_image_width = im_text.cols / 4;
  int text_margin = int(0.25f * single_image_width);

  cv::Scalar text_color(207, 159, 114);

  cv::putText(im_text, left_camera,
              cv::Point(text_margin, text_size.height * 2 + 45),
              cv::FONT_HERSHEY_COMPLEX, 1, text_color, 1, 8);
  cv::putText(im_text, front_camera, cv::Point(text_margin + single_image_width,
                                               text_size.height * 2 + 45),
              cv::FONT_HERSHEY_COMPLEX, 1, text_color, 1, 8);
  cv::putText(im_text, right_camera,
              cv::Point(text_margin + 2 * single_image_width,
                        text_size.height * 2 + 45),
              cv::FONT_HERSHEY_COMPLEX, 1, text_color, 1, 8);
  cv::putText(im_text, back_camera,
              cv::Point(text_margin + 3 * single_image_width,
                        text_size.height * 2 + 45),
              cv::FONT_HERSHEY_COMPLEX, 1, text_color, 1, 8);
}

void FrameDrawer::Update(Tracking *tracker) {
  unique_lock<mutex> lock(mutex_);
  tracker->img_gray_.copyTo(image_);
  cur_keypts_ = tracker->current_mc_frame_.frames_[FRONT_CAMERA].keypts_;
  cur_mc_keypts_.resize(4);
  for (int i = 0; i < 4; i++) {
    cur_mc_keypts_[i] = tracker->current_mc_frame_.frames_[i].keypts_;
  }

  num_feature_ = cur_keypts_.size();
  num_feature_left_ = cur_mc_keypts_[0].size();
  num_feature_front_ = cur_mc_keypts_[1].size();
  num_feature_right_ = cur_mc_keypts_[2].size();
  num_feature_back_ = cur_mc_keypts_[3].size();

  map_point_.resize(4);

  map_point_[0] = vector<bool>(num_feature_left_, false);
  map_point_[1] = vector<bool>(num_feature_front_, false);
  map_point_[2] = vector<bool>(num_feature_right_, false);
  map_point_[3] = vector<bool>(num_feature_back_, false);

  if (tracker->last_process_state_ == Tracking::NOT_INITIALIZED) {
    init_keypts_ = tracker->initial_mc_frame_.frames_[FRONT_CAMERA].keypts_;
    init_matches_ = tracker->init_matches_[FRONT_CAMERA];
  } else if (tracker->last_process_state_ == Tracking::OK) {
    for (int i = 0; i < num_feature_left_; i++) {
      MapPoint *mp =
          tracker->current_mc_frame_.frames_[LEFT_CAMERA].mappoints_[i];
      if (mp) {
        if (!tracker->current_mc_frame_.frames_[LEFT_CAMERA].outliers_[i]) {
          if (mp->Observations() > 0)
            map_point_[LEFT_CAMERA][i] = true;
        }
      }
    }

    for (int i = 0; i < num_feature_front_; i++) {
      MapPoint *mp =
          tracker->current_mc_frame_.frames_[FRONT_CAMERA].mappoints_[i];
      if (mp) {
        if (!tracker->current_mc_frame_.frames_[FRONT_CAMERA].outliers_[i]) {
          if (mp->Observations() > 0)
            map_point_[FRONT_CAMERA][i] = true;
        }
      }
    }

    for (int i = 0; i < num_feature_right_; i++) {
      MapPoint *mp =
          tracker->current_mc_frame_.frames_[RIGHT_CAMERA].mappoints_[i];
      if (mp) {
        if (!tracker->current_mc_frame_.frames_[RIGHT_CAMERA].outliers_[i]) {
          if (mp->Observations() > 0)
            map_point_[RIGHT_CAMERA][i] = true;
        }
      }
    }

    for (int i = 0; i < num_feature_back_; i++) {
      MapPoint *mp =
          tracker->current_mc_frame_.frames_[BACK_CAMERA].mappoints_[i];
      if (mp) {
        if (!tracker->current_mc_frame_.frames_[BACK_CAMERA].outliers_[i]) {
          if (mp->Observations() > 0)
            map_point_[BACK_CAMERA][i] = true;
        }
      }
    }
  }
  state_ = static_cast<int>(tracker->last_process_state_);
}
}
