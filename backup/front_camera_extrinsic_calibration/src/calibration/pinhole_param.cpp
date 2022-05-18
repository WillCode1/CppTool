#include "pinhole_param.h"
#define DISTORTION_COEF 5
using namespace std;

namespace nullmax_perception {

void PinholeParam::Load(const std::string &calibration_file) {
  ifstream input_file;
  std::string s;
  input_file.open(calibration_file.c_str(), ifstream::in);

  getline(input_file, s);
  stringstream sfx(s);
  std::string str_fx;
  sfx >> str_fx; // read std::string "fx:"
  sfx >> str_fx; // read fx value
  camera_model_param_.pinhole.intrinsic.fx = atof(str_fx.c_str());

  getline(input_file, s);
  stringstream sfy(s);
  std::string str_fy;
  sfy >> str_fy; // read std::string "fy:"
  sfy >> str_fy; // read fx value
  camera_model_param_.pinhole.intrinsic.fy = atof(str_fy.c_str());

  getline(input_file, s);
  stringstream scx(s);
  std::string str_cx;
  scx >> str_cx; // read std::string "cx:"
  scx >> str_cx; // read cx value
  camera_model_param_.pinhole.intrinsic.cx = atof(str_cx.c_str());

  getline(input_file, s);
  stringstream scy(s);
  std::string str_cy;
  scy >> str_cy; // read std::string "cy:"
  scy >> str_cy; // read cy value
  camera_model_param_.pinhole.intrinsic.cy = atof(str_cy.c_str());

  getline(input_file, s);
  stringstream sdist_coef(s);
  std::string str_dist_coef;
  sdist_coef >> str_dist_coef; // read std::string "dist_coef:"
  for (int i = 0; i < DISTORTION_COEF; ++i) {
    sdist_coef >> str_dist_coef;
    camera_model_param_.pinhole.distort.coef[i] = atof(str_dist_coef.c_str());
  }

  getline(input_file, s);
  stringstream swidth(s);
  std::string str_width;
  swidth >> str_width; // read std::string "width:"
  swidth >> str_width; // read width value
  camera_model_param_.pinhole.width = atoi(str_width.c_str());

  getline(input_file, s);
  stringstream sheight(s);
  std::string str_height;
  sheight >> str_height; // read std::string "height:"
  sheight >> str_height; // read height value
  camera_model_param_.pinhole.height = atof(str_height.c_str());

  input_file.close();

  float scale = 1.0f;
  camera_model_param_.pinhole.intrinsic_undistort.fx =
      camera_model_param_.pinhole.intrinsic.fx / scale;
  camera_model_param_.pinhole.intrinsic_undistort.fy =
      camera_model_param_.pinhole.intrinsic.fy / scale;
  camera_model_param_.pinhole.intrinsic_undistort.cx =
      camera_model_param_.pinhole.intrinsic.cx;
  camera_model_param_.pinhole.intrinsic_undistort.cy =
      camera_model_param_.pinhole.intrinsic.cy;
  // OutputParam();
  camera_matrix_in_ = cv::Mat(3, 3, CV_32FC1);
  camera_matrix_in_.at<float>(0, 0) = camera_model_param_.pinhole.intrinsic.fx;
  camera_matrix_in_.at<float>(0, 1) = 0;
  camera_matrix_in_.at<float>(0, 2) = camera_model_param_.pinhole.intrinsic.cx;
  camera_matrix_in_.at<float>(1, 0) = 0;
  camera_matrix_in_.at<float>(1, 1) = camera_model_param_.pinhole.intrinsic.fy;
  camera_matrix_in_.at<float>(1, 2) = camera_model_param_.pinhole.intrinsic.cy;
  camera_matrix_in_.at<float>(2, 0) = 0;
  camera_matrix_in_.at<float>(2, 1) = 0;
  camera_matrix_in_.at<float>(2, 2) = 1;

  distortion_coeffs_ = cv::Mat(5, 1, CV_32FC1);
  for (int i = 0; i < DISTORTION_COEF; ++i) {
    distortion_coeffs_.at<float>(i, 0) =
        camera_model_param_.pinhole.distort.coef[i];
  }

  camera_matrix_out_ = cv::Mat(3, 3, CV_32FC1);
  camera_matrix_out_.at<float>(0, 0) =
      camera_model_param_.pinhole.intrinsic_undistort.fx;
  camera_matrix_out_.at<float>(0, 1) = 0;
  camera_matrix_out_.at<float>(0, 2) =
      camera_model_param_.pinhole.intrinsic_undistort.cx;
  camera_matrix_out_.at<float>(1, 0) = 0;
  camera_matrix_out_.at<float>(1, 1) =
      camera_model_param_.pinhole.intrinsic_undistort.fy;
  camera_matrix_out_.at<float>(1, 2) =
      camera_model_param_.pinhole.intrinsic_undistort.cy;
  camera_matrix_out_.at<float>(2, 0) = 0;
  camera_matrix_out_.at<float>(2, 1) = 0;
  camera_matrix_out_.at<float>(2, 2) = 1;
}

PinholeParam::PinholeParam(const std::string &calibration_file) {
  Load(calibration_file);
}

int PinholeParam::Undistortion(const cv::Mat &src, cv::Mat &dst) {
  PinholeModel &pin = camera_model_param_.pinhole;
  int width = src.cols;  // pin.width;
  int height = src.rows; // pin.height;
  dst = cv::Mat(src.rows, src.cols, CV_8UC1, cv::Scalar(0));
  double camcord[3];
  double imgcord[2];
  double r1, r2, r3;
  double tep1, tep2, tep3, tep4;
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      camcord[0] = (j - pin.intrinsic.cx) / pin.intrinsic.fx;
      camcord[1] = (i - pin.intrinsic.cy) / pin.intrinsic.fy;
      r1 = camcord[0] * camcord[0] + camcord[1] * camcord[1];
      r2 = r1 * r1;
      r3 = r1 * r2;
      tep1 = 2 * camcord[0] * camcord[1];
      tep2 = r1 + 2 * camcord[0] * camcord[0];
      tep3 = r1 + 2 * camcord[1] * camcord[1];
      tep4 = 1 + pin.distort.coef[0] * r1 + pin.distort.coef[1] * r2 +
             pin.distort.coef[4] * r3;
      imgcord[0] = camcord[0] * tep4 + tep1 * pin.distort.coef[2] +
                   tep2 * pin.distort.coef[3];
      imgcord[0] = pin.intrinsic.fx * imgcord[0] + pin.intrinsic.cx;
      imgcord[1] = camcord[1] * tep4 + tep1 * pin.distort.coef[3] +
                   tep3 * pin.distort.coef[2];
      imgcord[1] = pin.intrinsic.fy * imgcord[1] + pin.intrinsic.cy;
      if (round(imgcord[0]) >= width || round(imgcord[0]) < 0 ||
          round(imgcord[1]) >= height || round(imgcord[1]) < 0) {
        continue;
      }
      float ui = imgcord[0];
      float vi = imgcord[1];
      int x1 = int(ui), x2 = int(ui + 1);
      int ky1 = int(vi), y2 = int(vi + 1);
      float x = ui - x1, y = vi - ky1;
      uchar val = src.at<uchar>(ky1, x1) * (1 - x) * (1 - y) +
                  src.at<uchar>(ky1, x2) * x * (1 - y) +
                  src.at<uchar>(y2, x1) * (1 - x) * y +
                  src.at<uchar>(y2, x2) * x * y;

      dst.at<uchar>(i, j) = val;
    }
  }
  // update undistort param
  pin.intrinsic_undistort = pin.intrinsic;
  return 0;
}

void PinholeParam::OpencvPinholeUndistortion(const cv::Mat &src, cv::Mat &dst) {
  cv::Size image_size = src.size();
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
  cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
  // TODO: here can do one time
  cv::initUndistortRectifyMap(camera_matrix_in_, distortion_coeffs_, R,
                              camera_matrix_out_, image_size, CV_32FC1, mapx,
                              mapy);
  cv::remap(src, dst, mapx, mapy, cv::INTER_LINEAR);
}

void PinholeParam::OpencvFisheyeUndistortion(const cv::Mat &src, cv::Mat &dst) {
  cv::Size image_size = src.size();
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
  cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
  cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
  camera_matrix_out_ =
      cv::getOptimalNewCameraMatrix(camera_matrix_in_, distortion_coeffs_,
                                    image_size, 0, image_size, 0, false);
  cv::Vec4d distortion_coeffs(camera_model_param_.pinhole.distort.coef[0],
                              camera_model_param_.pinhole.distort.coef[1],
                              camera_model_param_.pinhole.distort.coef[2],
                              camera_model_param_.pinhole.distort.coef[3]);
  cv::fisheye::initUndistortRectifyMap(camera_matrix_in_, distortion_coeffs, R,
                                       camera_matrix_out_, image_size, CV_32FC1,
                                       mapx, mapy);
  cv::remap(src, dst, mapx, mapy, cv::INTER_LINEAR);
  camera_model_param_.pinhole.intrinsic_undistort.fx =
      camera_matrix_out_.at<float>(0, 0);
  camera_model_param_.pinhole.intrinsic_undistort.fy =
      camera_matrix_out_.at<float>(1, 1);
  camera_model_param_.pinhole.intrinsic_undistort.cx =
      camera_matrix_out_.at<float>(0, 2);
  camera_model_param_.pinhole.intrinsic_undistort.cy =
      camera_matrix_out_.at<float>(1, 2);
}

cv::Point2f PinholeParam::World2Camera(cv::Point3f X) {
  PinholeModel &pin = camera_model_param_.pinhole;
  if (X.z != 0) {
    X.x /= X.z;
    X.y /= X.z;
  }
  return cv::Point2f(
      X.x * pin.intrinsic_undistort.fx + pin.intrinsic_undistort.cx,
      X.y * pin.intrinsic_undistort.fy + pin.intrinsic_undistort.cy);
}

cv::Point3f PinholeParam::Camera2World(cv::Point2f pt) {
  PinholeModel &pin = camera_model_param_.pinhole;
  double x = (pt.x - pin.intrinsic_undistort.cx) / pin.intrinsic_undistort.fx;
  double y = (pt.y - pin.intrinsic_undistort.cy) / pin.intrinsic_undistort.fy;
  double z = 1.0f;
  double norm = sqrt(x * x + y * y + z * z);
  return cv::Point3f(x / norm, y / norm, z / norm);
}
}