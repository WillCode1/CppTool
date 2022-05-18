#include "fisheye_ocam_param.h"
#include "ocam_functions.h"

using namespace std;

namespace nullmax_perception {

void FisheyeOcamParam::Load(const string &calibration_file) {
  ifstream input_file;
  string s;
  input_file.open(calibration_file.c_str(), ifstream::in);
  getline(input_file, s); // first line
  getline(input_file, s); // second line
  getline(input_file, s); // third line
  stringstream ss(s);
  string strlength_pol;
  ss >> strlength_pol;

  camera_model_param_.ocam.length_pol = atoi(strlength_pol.c_str());

  camera_model_param_.ocam.pol.resize(camera_model_param_.ocam.length_pol);
  for (int i = 0; i < camera_model_param_.ocam.length_pol; i++) {
    string coef;
    ss >> coef;
    camera_model_param_.ocam.pol[i] = atof(coef.c_str());
  }

  getline(input_file, s); // 4th
  getline(input_file, s); // 5th
  getline(input_file, s); // 6th
  getline(input_file, s); // 7th
  stringstream ssinv(s);
  string strlength_invpol;
  ssinv >> strlength_invpol;
  camera_model_param_.ocam.length_invpol = atoi(strlength_invpol.c_str());
  camera_model_param_.ocam.invpol.resize(
      camera_model_param_.ocam.length_invpol);
  for (int i = 0; i < camera_model_param_.ocam.length_invpol; i++) {
    string coef;
    ssinv >> coef;
    camera_model_param_.ocam.invpol[i] = atof(coef.c_str());
  }

  getline(input_file, s); // 8th
  getline(input_file, s); // 9th
  getline(input_file, s); // 10th
  getline(input_file, s); // 11th

  stringstream uv(s);
  string stry;
  string strx;

  uv >> stry;
  uv >> strx;
  camera_model_param_.ocam.yc = atof(stry.c_str());
  camera_model_param_.ocam.xc = atof(strx.c_str());

  getline(input_file, s); // 12th
  getline(input_file, s); // 13th
  getline(input_file, s); // 14th
  getline(input_file, s); // 15th

  stringstream affinecoef(s);
  string strc;
  string strd;
  string stre;
  affinecoef >> strc;
  affinecoef >> strd;
  affinecoef >> stre;

  camera_model_param_.ocam.c = atof(strc.c_str());
  camera_model_param_.ocam.d = atof(strd.c_str());
  camera_model_param_.ocam.e = atof(stre.c_str());

  getline(input_file, s); // 16th
  getline(input_file, s); // 17th
  getline(input_file, s); // 18th
  getline(input_file, s); // 19th

  stringstream imagesize(s);
  string strwidth;
  string strheight;
  imagesize >> strheight;
  imagesize >> strwidth;
  camera_model_param_.ocam.height = atoi(strheight.c_str());
  camera_model_param_.ocam.width = atoi(strwidth.c_str());
}

FisheyeOcamParam::FisheyeOcamParam(const std::string &calibration_file) {
  Load(calibration_file);
}

cv::Point2f FisheyeOcamParam::World2Camera(cv::Point3f pt3) {
  OcamModel &ocam = camera_model_param_.ocam;
  double norm = sqrt(pt3.x * pt3.x + pt3.y * pt3.y);
  if (norm == 0.0)
    norm = 1e-14;

  double M[3] = {pt3.x, pt3.y, pt3.z};
  double m[2];
  world2cam(m, M, ocam);
  return cv::Point2f(m[0], m[1]);
}

cv::Point3f FisheyeOcamParam::Camera2World(cv::Point2f pt2) {
  OcamModel &ocam = camera_model_param_.ocam;
  double M[3];
  double m[2] = {pt2.x, pt2.y};
  cam2world(M, m, ocam);
  return cv::Point3f(M[0], M[1], M[2]);
}

int FisheyeOcamParam::Undistortion(const cv::Mat &src, cv::Mat &dst) {
  // TODO: can do better
  // auto define fx,fy,cx,cy
  dst = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);

  if (mapx_persp_.empty() || mapy_persp_.empty()) {
    mapx_persp_ = cv::Mat::zeros(src.rows, src.cols, CV_32FC1);
    mapy_persp_ = cv::Mat::zeros(src.rows, src.cols, CV_32FC1);

    // float sf = 2.4;//2.4
    // float focal = src.cols / sf;
    // float cx = src.cols / 2.0;
    // float cy = src.rows / 2.0 + 70;
    PinholeModel &pinhole = camera_model_param_.pinhole;
    float focal = pinhole.intrinsic_undistort.fx;
    float cx = pinhole.intrinsic_undistort.cx;
    float cy = pinhole.intrinsic_undistort.cy;
    create_perspecive_undistortion_LUT(mapx_persp_, mapy_persp_,
                                       camera_model_param_.ocam, focal, cx, cy);

    camera_model_param_.pinhole.intrinsic.fx = focal;
    camera_model_param_.pinhole.intrinsic.fy = focal;
    camera_model_param_.pinhole.intrinsic.cx = cx;
    camera_model_param_.pinhole.intrinsic.cy = cy;
    camera_model_param_.pinhole.distort.k1 = 0;
    camera_model_param_.pinhole.distort.k2 = 0;
    camera_model_param_.pinhole.distort.k3 = 0;
    camera_model_param_.pinhole.distort.p1 = 0;
    camera_model_param_.pinhole.distort.p2 = 0;
  }
  cv::remap(src, dst, mapx_persp_, mapy_persp_, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT);

  // cv::imshow("Original fisheye camera image", src);
  // cv::imshow("Undistorted Perspective Image", dst);
  // cv::imwrite("undistorted_perspective.png", dst);
}
}