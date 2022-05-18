#include "ocam.h"
#include <cmath>

namespace nullmax_perception {
namespace calibration {
bool Ocam::RunCalibration(
    const std::vector<std::vector<cv::Point2f>> &image_points) {
  if (image_points.empty()) {
    return false;
  } else {
    image_points_ = image_points;
  }

  object_points_.resize(1);
  for (int i = 0; i < config_.board_size.height; ++i) {
    for (int j = 0; j < config_.board_size.width; ++j) {
      object_points_[0].push_back(cv::Point3f(
          j * config_.board_unit_length, i * config_.board_unit_length, 0));
    }
  }

  object_points_.resize(image_points_.size(), object_points_[0]);

  intrinsic_ = std::make_shared<OcamIntrinsics>();
  OcamAlgrithom::Ptr ocam_calib = std::make_shared<OcamAlgrithom>();
  double rms = ocam_calib->Calibrate(object_points_, image_points,
                                     config_.img_size, intrinsic_);
  repro_errors_ = ocam_calib->GetReproErrors();
  pixel_repro_errors_ = ocam_calib->GetPixelReproErrors();

  std::cout << "\033[0;33m";
  std::cout << "***** Re-projection error: " << rms << std::endl;

  std::cout << "principal point: " << std::endl;
  std::cout << intrinsic_->cx << " " << intrinsic_->cy << std::endl;
  std::cout << "cam to world (poly): " << std::endl;
  for (auto v : intrinsic_->pol) {
    std::cout << v << " ";
  }
  std::cout << std::endl;

  std::cout << "world to cam (inv poly): " << std::endl;
  for (auto v : intrinsic_->invpol) {
    std::cout << v << " ";
  }
  std::cout << std::endl;
  std::cout << "stretch matrix (c d e): " << std::endl;
  std::cout << intrinsic_->c << " " << intrinsic_->d << " " << intrinsic_->e
            << std::endl;
  std::cout << "\033[0m";

  bool ok = true;

  return ok;
}

bool Ocam::SaveResult() {
  std::ofstream out_file("../result/ocam_intrinsics.txt");
  if (!out_file.is_open()) {
    std::cerr << "save file failed" << std::endl;
    return false;
  }

  out_file << "#polynomial coefficients for the DIRECT mapping function "
              "(ocam_model.ss in MATLAB). These are used by cam2world"
           << std::endl
           << std::endl;
  out_file << (int)intrinsic_->pol.size() << " ";
  out_file.precision(10);
  for (auto &p : intrinsic_->pol) {
    out_file << p << " ";
  }
  out_file << std::endl << std::endl;

  out_file << "#polynomial coefficients for the inverse mapping function "
              "(ocam_model.invpol in MATLAB). These are used by world2cam"
           << std::endl
           << std::endl;

  out_file.precision(2);
  out_file << (int)intrinsic_->invpol.size() << " ";
  out_file.precision(10);
  for (auto &p : intrinsic_->invpol) {
    out_file << p << " ";
  }
  out_file << std::endl << std::endl;

  out_file << "#center: \"row\" and \"column\", starting from 0 (C convention)"
           << std::endl
           << std::endl;
  out_file << intrinsic_->cy << " " << intrinsic_->cx << std::endl << std::endl;

  out_file << "#affine parameters \"c\", \"d\", \"e\"" << std::endl
           << std::endl;
  out_file << intrinsic_->c << " " << intrinsic_->d << " " << intrinsic_->e
           << std::endl
           << std::endl;

  out_file << "#image size: \"height\" and \"width\"" << std::endl << std::endl;
  out_file << config_.img_size.height << " " << config_.img_size.width
           << std::endl
           << std::endl;

  out_file << "#vitual focal length of undistortion: \"virtual_fx\" and "
              "\"virtual_fy\""
           << std::endl
           << std::endl;
  out_file << config_.specified_fx << " " << config_.specified_fy << std::endl;

  out_file.close();
  std::cout << "save txt file success: ocam_intrinsics.txt" << std::endl;

  SaveRemapFile();
  SavePixelError();

  return true;
}

bool Ocam::ReadIntrinsics(const std::string &intrinsic_file) {
  std::ifstream in_file;
  std::string s;
  in_file.open(intrinsic_file.c_str(), std::ifstream::in);
  if (!in_file.is_open()) {
    std::cerr << "read intrinsic file error" << std::endl;
    return false;
  }

  getline(in_file, s);
  getline(in_file, s);
  getline(in_file, s);
  std::stringstream ss(s);
  std::string strlength_pol;
  ss >> strlength_pol;

  int pol_size = atoi(strlength_pol.c_str());

  intrinsic_->pol.resize(pol_size);
  for (int i = 0; i < pol_size; i++) {
    std::string coef;
    ss >> coef;
    intrinsic_->pol[i] = atof(coef.c_str());
  }

  getline(in_file, s);
  getline(in_file, s);
  getline(in_file, s);
  getline(in_file, s);
  std::stringstream ssinv(s);
  std::string strlength_invpol;
  ssinv >> strlength_invpol;
  int invpol_size = atoi(strlength_invpol.c_str());
  intrinsic_->invpol.resize(invpol_size);
  for (int i = 0; i < invpol_size; i++) {
    std::string coef;
    ssinv >> coef;
    intrinsic_->invpol[i] = atof(coef.c_str());
  }

  getline(in_file, s);
  getline(in_file, s);
  getline(in_file, s);
  getline(in_file, s);

  std::stringstream uv(s);
  std::string stry;
  std::string strx;

  uv >> stry;
  uv >> strx;
  intrinsic_->cy = atof(stry.c_str());
  intrinsic_->cx = atof(strx.c_str());

  getline(in_file, s);
  getline(in_file, s);
  getline(in_file, s);
  getline(in_file, s);

  std::stringstream affinecoef(s);
  std::string strc;
  std::string strd;
  std::string stre;
  affinecoef >> strc;
  affinecoef >> strd;
  affinecoef >> stre;

  intrinsic_->c = atof(strc.c_str());
  intrinsic_->d = atof(strd.c_str());
  intrinsic_->e = atof(stre.c_str());

  in_file.close();
  return true;
}

cv::Mat Ocam::Undistort(const cv::Mat &distorted_img) {
  double cx = distorted_img.cols / 2;
  double cy = distorted_img.rows / 2;
  if (undistort_remap_.first.empty() || undistort_remap_.second.empty()) {
    if (!CalVirtualf()) {
      return cv::Mat();
    }

    double &fx = config_.specified_fx;
    double &fy = config_.specified_fy;

    undistort_remap_.first =
        cv::Mat::zeros(distorted_img.rows, distorted_img.cols, CV_32FC1);
    undistort_remap_.second =
        cv::Mat::zeros(distorted_img.rows, distorted_img.cols, CV_32FC1);

    for (int i = 0; i < distorted_img.cols; i++) {
      for (int j = 0; j < distorted_img.rows; j++) {
        float u = 1.0 * i;
        float v = 1.0 * j;
        cv::Point3f point((u - cx) / fx, (v - cy) / fy, 1.0);

        cv::Point2f pixel = CameraToPixel(point);

        undistort_remap_.first.at<float>(j, i) = pixel.x;
        undistort_remap_.second.at<float>(j, i) = pixel.y;
      }
    }

    if (config_.undistort_type == "auto" && !config_.use_FOV &&
        !config_.use_specified_focal_length) {
      cv::Mat remapx = undistort_remap_.first(cv::Rect(
          left_max_, up_max_, right_min_ - left_max_, down_min_ - up_max_));
      cv::Mat remapy = undistort_remap_.second(cv::Rect(
          left_max_, up_max_, right_min_ - left_max_, down_min_ - up_max_));
      std::cout << "*** remapx.cols: " << remapx.cols << " " << remapx.rows
                << std::endl;
      undistort_remap_.first = remapx;
      undistort_remap_.second = remapy;
    }
  }

  cv::Mat undistorted_img;
  cv::remap(distorted_img, undistorted_img, undistort_remap_.first,
            undistort_remap_.second, cv::INTER_LINEAR);

  return undistorted_img;
}

bool Ocam::CalVirtualf() {
  double fx = 100.;
  double fy = 100.;
  double cx = config_.img_size.width / 2;
  double cy = config_.img_size.height / 2;

  if (config_.use_FOV) {
    cx = (config_.img_size.width - intrinsic_->cx) < intrinsic_->cx
             ? (config_.img_size.width - intrinsic_->cx)
             : intrinsic_->cx;
    config_.specified_fx = cx / std::tan(config_.target_FOV * CV_PI / 360.);
    config_.specified_fy = config_.specified_fx;
    std::cout << "specified fx: " << config_.specified_fx << std::endl;
    std::cout << "specified fy: " << config_.specified_fy << std::endl;
    return true;
  } else if (config_.use_specified_focal_length) {
    cx = (config_.img_size.width - intrinsic_->cx) < intrinsic_->cx
             ? (config_.img_size.width - intrinsic_->cx)
             : intrinsic_->cx;
    config_.target_FOV = 360. * std::atan(cx / config_.specified_fx) / CV_PI;
    std::cout << "undistorted FOV: " << config_.target_FOV << std::endl;
    return true;
  }

  float left_max(-FLT_MAX), right_min(FLT_MAX), up_max(-FLT_MAX),
      down_min(FLT_MAX);
  for (int i = 0; i < config_.img_size.width; i++) {
    for (int j = 0; j < config_.img_size.height; j++) {
      float u = 1.0 * i;
      float v = 1.0 * j;
      cv::Point3f point((u - cx) / fx, (v - cy) / fy, 1.0);

      cv::Point2f pixel = CameraToPixel(point);

      if (pixel.x < 2 && u > left_max) {
        left_max = u;
      } else if (pixel.x >= (config_.img_size.width - 2) && u < right_min) {
        right_min = u;
      }

      if (pixel.y < 2 && v > up_max) {
        up_max = v;
      } else if (pixel.y >= (config_.img_size.height - 2) && v < down_min) {
        down_min = v;
      }
    }
  }

  left_max = std::max(float(0), left_max);
  right_min = std::min(float(config_.img_size.width), right_min);
  if (config_.undistort_type == "ratio") {
    float W = std::min(cx - left_max, right_min - cx);
    float H = std::min(cy - up_max, down_min - cy);
    if ((W / H) >
        (float(config_.img_size.width) / float(config_.img_size.height))) {
      if (cy - up_max < down_min - cy) {
        config_.specified_fy = (0. - cy) / (up_max - cy) * fy;
      } else {
        config_.specified_fy =
            (config_.img_size.height - cy) / (down_min - cy) * fy;
      }
      config_.specified_fx = config_.specified_fy;
    } else {
      if (cx - left_max < right_min - cx) {
        config_.specified_fx = (0. - cx) / (left_max - cx) * fx;
      } else {
        config_.specified_fx =
            (config_.img_size.width - cx) / (right_min - cx) * fx;
      }
      config_.specified_fy = config_.specified_fx;
    }
  } else if (config_.undistort_type == "auto") {
    float W = std::max(cx - left_max, right_min - cx);
    float H = std::max(cy - up_max, down_min - cy);
    if ((W / H) <
        (float(config_.img_size.width) / float(config_.img_size.height))) {
      if (cy - up_max < down_min - cy) {
        config_.specified_fy = (0. - cy) / (up_max - cy) * fy;
      } else {
        config_.specified_fy =
            (config_.img_size.height - cy) / (down_min - cy) * fy;
      }
      config_.specified_fx = config_.specified_fy;
    } else {
      if (cx - left_max > right_min - cx) {
        config_.specified_fx = (0. - cx) / (left_max - cx) * fx;
      } else {
        config_.specified_fx =
            (config_.img_size.width - cx) / (right_min - cx) * fx;
      }
      config_.specified_fy = config_.specified_fx;
    }
    left_max_ = cx + (left_max - cx) / fx * config_.specified_fx;
    right_min_ = cx + (right_min - cx) / fx * config_.specified_fx;
    up_max_ = cy + (up_max - cy) / fy * config_.specified_fy;
    down_min_ = cy + (down_min - cy) / fy * config_.specified_fy;
  } else {
    std::cerr << "No support undistort_type!" << std::endl;
    return false;
  }

  std::cout << "specified fx " << config_.specified_fx << std::endl;
  std::cout << "specified fy " << config_.specified_fy << std::endl;
  return true;
}

void Ocam::CalReprojectError() {
  for (int i = 0, size = repro_errors_.size(); i < size; ++i) {
    std::cout << "No." << i + 1 << " image error: " << repro_errors_[i]
              << std::endl;
  }
}

cv::Point2f Ocam::CameraToPixel(const cv::Point3f &point) {
  double norm = sqrt(point.x * point.x + point.y * point.y);
  if (norm == 0.0)
    norm = 1e-14;
  const double theta = std::atan2(-point.z, norm);
  double rho = 0.0;
  double theta_i = 1.0;
  for (int i = 0; i < intrinsic_->invpol.size(); ++i) {
    rho += theta_i * intrinsic_->invpol[i];
    theta_i *= theta;
  }

  const double uu = point.x / norm * rho;
  const double vv = point.y / norm * rho;
  cv::Point2f pixel;
  pixel.x = uu * intrinsic_->c + vv * intrinsic_->d + intrinsic_->cx;
  pixel.y = uu * intrinsic_->e + vv + intrinsic_->cy;
  return pixel;
}
} // calibration
} // nullmax_perception