#include "fisheye.h"
#include "json/json.h"

namespace nullmax_perception {
namespace calibration {
bool Fisheye::RunCalibration(
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

  // choose optimize mode
  int calib_flag =
      cv::fisheye::CALIB_FIX_SKEW | cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
  //   calib_flag |= cv::fisheye::CALIB_FIX_K1;
  //   calib_flag |= cv::fisheye::CALIB_FIX_K2;
  //   calib_flag |= cv::fisheye::CALIB_FIX_K3;
  //   calib_flag |= cv::fisheye::CALIB_FIX_K4;
  //   calib_flag |= cv::fisheye::CALIB_FIX_PRINCIPAL_POINT;

  double rms = cv::fisheye::calibrate(object_points_, image_points_,
                                      config_.img_size, intrinsic_,
                                      dist_coeffs_, rvecs_, tvecs_, calib_flag);

  bool ok = cv::checkRange(intrinsic_) && cv::checkRange(dist_coeffs_);

  if (ok) {
    std::cout << "\033[0;33m";
    std::cout << "***** Re-projection error: " << rms << std::endl;
    std::cout << "intrinsic" << std::endl;
    std::cout << intrinsic_ << std::endl;
    std::cout << "dist_coeffs_" << std::endl;
    std::cout << dist_coeffs_ << std::endl;
    std::cout << "\033[0m";
  }

  return ok;
}

bool Fisheye::SaveResult() {
  std::ofstream out_file("../result/fisheye_intrinsics.json");
  if (!out_file.is_open()) {
    std::cerr << "save file failed" << std::endl;
    return false;
  }

  Json::Value root, distortion, intrinsic;

  distortion["k1"] = dist_coeffs_.at<double>(0, 0);
  distortion["k2"] = dist_coeffs_.at<double>(0, 1);
  distortion["k3"] = dist_coeffs_.at<double>(0, 2);
  distortion["k4"] = dist_coeffs_.at<double>(0, 3);

  intrinsic["cx"] = intrinsic_.at<double>(0, 2);
  intrinsic["cy"] = intrinsic_.at<double>(1, 2);
  intrinsic["fx"] = intrinsic_.at<double>(0, 0);
  intrinsic["fy"] = intrinsic_.at<double>(1, 1);

  root["distortion"] = Json::Value(distortion);
  root["intrinsic"] = Json::Value(intrinsic);

  out_file << root.toStyledString();

  out_file.close();
  std::cout << "save json file success: fisheye_intrinsics.json" << std::endl;

  SaveRemapFile();
  SavePixelError();

  return true;
}

bool Fisheye::ReadIntrinsics(const std::string &intrinsic_file) {
  Json::Reader reader;
  Json::Value root;

  std::ifstream in_file(intrinsic_file, std::ios::binary);
  if (!in_file.is_open()) {
    std::cerr << "read intrinsic file error" << std::endl;
    return false;
  }

  if (reader.parse(in_file, root)) {
    intrinsic_ = cv::Mat::eye(3, 3, CV_64F);
    intrinsic_.at<double>(0, 0) = root["intrinsic"]["fx"].asDouble();
    intrinsic_.at<double>(1, 1) = root["intrinsic"]["fy"].asDouble();
    intrinsic_.at<double>(0, 2) = root["intrinsic"]["cx"].asDouble();
    intrinsic_.at<double>(1, 2) = root["intrinsic"]["cy"].asDouble();

    dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
    dist_coeffs_.at<double>(0, 0) = root["distortion"]["k1"].asDouble();
    dist_coeffs_.at<double>(0, 1) = root["distortion"]["k2"].asDouble();
    dist_coeffs_.at<double>(0, 2) = root["distortion"]["k3"].asDouble();
    dist_coeffs_.at<double>(0, 3) = root["distortion"]["k4"].asDouble();
  } else {
    std::cerr << "json parse error" << std::endl;
    return false;
  }
  in_file.close();

  return true;
}

cv::Mat Fisheye::Undistort(const cv::Mat &distorted_img) {
  cv::Mat undistorted_img, newCamMat;
  // 使用指定fx/fy 或 指定FOV微调CamMat
  if (undistort_remap_.first.empty() || undistort_remap_.second.empty()) {
    newCamMat = cv::Mat::eye(3, 3, CV_64F);
    newCamMat.at<double>(0, 2) = config_.img_size.width / 2;
    newCamMat.at<double>(1, 2) = config_.img_size.height / 2;
    if (config_.use_FOV) {
      double intrinsic_cx = intrinsic_.at<double>(0, 2);
      double cx = (config_.img_size.width - intrinsic_cx) < intrinsic_cx
                      ? (config_.img_size.width - intrinsic_cx)
                      : intrinsic_cx;
      config_.specified_fx = cx / std::tan(config_.target_FOV * CV_PI / 360.);
      config_.specified_fy = config_.specified_fx;
      newCamMat.at<double>(0, 0) = config_.specified_fx;
      newCamMat.at<double>(1, 1) = config_.specified_fy;
      std::cout << "specified fx: " << config_.specified_fx << std::endl;
      std::cout << "specified fy: " << config_.specified_fy << std::endl;
    } else if (config_.use_specified_focal_length) {
      double intrinsic_cx = intrinsic_.at<double>(0, 2);
      double cx = (config_.img_size.width - intrinsic_cx) < intrinsic_cx
                      ? (config_.img_size.width - intrinsic_cx)
                      : intrinsic_cx;
      config_.target_FOV = 360. * std::atan(cx / config_.specified_fx) / CV_PI;
      newCamMat.at<double>(0, 0) = config_.specified_fx;
      newCamMat.at<double>(1, 1) = config_.specified_fy;
      std::cout << "undistorted FOV: " << config_.target_FOV << std::endl;
    } else if (config_.undistort_type == "ratio" ||
               config_.undistort_type == "auto") {
      cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
          intrinsic_, dist_coeffs_, config_.img_size, cv::Matx33d::eye(),
          newCamMat, 1);
      if (config_.undistort_type == "ratio") {
        newCamMat = cv::getOptimalNewCameraMatrix(
            intrinsic_, dist_coeffs_, config_.img_size, 1, config_.img_size, 0);
      }

    } else {
      std::cerr << "No support undistort_type!" << std::endl;
      return cv::Mat();
    }

    cv::fisheye::initUndistortRectifyMap(
        intrinsic_, dist_coeffs_, cv::Mat(), newCamMat, config_.img_size,
        CV_32FC1, undistort_remap_.first, undistort_remap_.second);
  }

  cv::remap(distorted_img, undistorted_img, undistort_remap_.first,
            undistort_remap_.second, cv::INTER_LINEAR);

  return undistorted_img;
}

void Fisheye::CalReprojectError() {
  std::vector<float> reproject_errors(object_points_.size());
  std::vector<cv::Point2f> projected_points;
  size_t point_num = 0;
  double total_error = 0, error;

  for (size_t i = 0; i < object_points_.size(); ++i) {
    cv::fisheye::projectPoints(object_points_[i], projected_points, rvecs_[i],
                               tvecs_[i], intrinsic_, dist_coeffs_);

    std::vector<cv::Point2f> pixel_error;
    for (int p = 0; p < image_points_[i].size(); ++p) {
      pixel_error.push_back(image_points_[i][p] - projected_points[p]);
    }
    pixel_repro_errors_.push_back(pixel_error);

    error = cv::norm(image_points_[i], projected_points, cv::NORM_L2);

    size_t n = object_points_[i].size();
    reproject_errors[i] = (float)std::sqrt(error * error / n);
    total_error += error * error;
    point_num += n;

    std::cout << "error of " << i << ": " << reproject_errors[i] << std::endl;
  }

  double total_avg_error = std::sqrt(total_error / point_num);
  std::cout << "total avg error: " << total_avg_error << std::endl;
}
} // calibration
} // nullmax_perception