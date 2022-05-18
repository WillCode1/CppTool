#include "pinhole.h"
#include "json/json.h"

namespace nullmax_perception {
namespace calibration {
bool Pinhole::RunCalibration(
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
  int calib_flag = 0;
  // calib_flag |= cv::CALIB_FIX_PRINCIPAL_POINT;
  // calib_flag |= cv::CALIB_FIX_ASPECT_RATIO;
  if (config_.pinhole_fix_p) {
    calib_flag |= cv::CALIB_ZERO_TANGENT_DIST;
  }
  if (config_.pinhole_fix_k3) {
    calib_flag |= cv::CALIB_FIX_K3;
  }

  calib_flag |= cv::CALIB_FIX_K4;
  calib_flag |= cv::CALIB_FIX_K5;
  calib_flag |= cv::CALIB_FIX_K6;

  double init_rms =
      cv::calibrateCamera(object_points_, image_points_, config_.img_size,
                          intrinsic_, dist_coeffs_, rvecs_, tvecs_, calib_flag);

  bool ok = cv::checkRange(intrinsic_) && cv::checkRange(dist_coeffs_);

  if (ok) {
    double rms = IterativeCalib(init_rms, calib_flag);

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

double Pinhole::IterativeCalib(const double &init_rms, const int &calib_flag) {
  std::cout << "init rms: " << init_rms << std::endl;
  double rms, pre_rms(init_rms);
  cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
  for (int iter = 1; iter <= 10; ++iter) {
    std::vector<double> dist_param = {
        dist_coeffs_.at<double>(0, 0), dist_coeffs_.at<double>(0, 1),
        dist_coeffs_.at<double>(0, 2), dist_coeffs_.at<double>(0, 3),
        dist_coeffs_.at<double>(0, 4)};
    // 1.calculate transform R of paralla views
    std::vector<cv::Mat> paralla_R = CalParallalR();

    // 2.convert corners to paralla views and extract new subpix corners
    std::vector<std::vector<cv::Point2f>> paralla_cornres(image_points_.size());
    for (int i = 0; i < image_points_.size(); ++i) {
      paralla_cornres[i] = ConvertCorners(
          image_points_[i], intrinsic_, rvecs_[i], tvecs_[i], dist_param,
          intrinsic_, identity, tvecs_[i], std::vector<double>());
    }

    for (int i = 0; i < paralla_R.size(); ++i) {
      cv::Mat gary_image;
      cv::cvtColor(images_[i], gary_image, CV_BGR2GRAY);
      cv::Mat paralla_img =
          cv::Mat(2 * gary_image.rows, 2 * gary_image.cols, gary_image.type());

      cv::Mat map1, map2;
      cv::initUndistortRectifyMap(intrinsic_, dist_coeffs_, paralla_R[i],
                                  intrinsic_, paralla_img.size(), CV_32FC1,
                                  map1, map2);
      cv::remap(gary_image, paralla_img, map1, map2, cv::INTER_LINEAR,
                cv::BORDER_CONSTANT);
      cv::cornerSubPix(
          paralla_img, paralla_cornres[i], cv::Size(15, 15), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                           0.1));
    }

    // 3.convert new corners to origin views
    for (int i = 0; i < image_points_.size(); ++i) {
      auto new_corners = ConvertCorners(
          paralla_cornres[i], intrinsic_, identity, tvecs_[i],
          std::vector<double>(), intrinsic_, rvecs_[i], tvecs_[i], dist_param);
      paralla_cornres[i] = new_corners;
    }

    // 4.using new corners to calibration
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat intrinsic, dist_coeffs;
    rms = cv::calibrateCamera(object_points_, paralla_cornres, config_.img_size,
                              intrinsic, dist_coeffs, rvecs, tvecs, calib_flag);
    std::cout << "iterative: " << iter << " refine rms: " << rms << std::endl;

    // 5.end condition: whether reprojection error becomes larger
    if (rms > pre_rms) {
      return pre_rms;
    }
    pre_rms = rms;
    intrinsic_ = intrinsic;
    dist_coeffs_ = dist_coeffs;
    rvecs_ = rvecs;
    tvecs_ = tvecs;
    image_points_ = paralla_cornres;
  }
  return rms;
}

std::vector<cv::Mat> Pinhole::CalParallalR() {
  std::vector<cv::Mat> paralla_R;
  for (int i = 0; i < rvecs_.size(); ++i) {
    cv::Mat R;
    cv::Rodrigues(rvecs_[i], R);
    cv::Mat t = tvecs_[i];

    cv::Mat H0(3, 3, CV_64F);
    R.copyTo(H0);
    t.col(0).copyTo(H0.col(2));

    cv::Mat H1 = cv::Mat::eye(3, 3, CV_64F);
    t.col(0).copyTo(H1.col(2));
    R = H1 * H0.inv();
    paralla_R.push_back(R);
  }
  return paralla_R;
}

std::vector<cv::Point2f> Pinhole::ConvertCorners(
    const std::vector<cv::Point2f> &pixels1, const cv::Mat &intrinsic1,
    const cv::Mat &r1, const cv::Mat &t1,
    const std::vector<double> &dist_coeffs1, const cv::Mat &intrinsic2,
    const cv::Mat &r2, const cv::Mat &t2,
    const std::vector<double> &dist_coeffs2) {
  // 1.Get 3D point using cam1
  std::vector<cv::Point3f> points;
  cv::Mat R1;
  r1.copyTo(R1);
  if (R1.rows == 1 || R1.cols == 1) {
    cv::Rodrigues(r1, R1);
  }
  t1.col(0).copyTo(R1.col(2));
  std::vector<cv::Point2f> undist_pixels1;
  cv::undistortPoints(pixels1, undist_pixels1, intrinsic1, dist_coeffs1,
                      R1.inv());
  for (auto &p : undist_pixels1) {
    points.push_back(cv::Point3f(p.x, p.y, 0));
  }

  // 2.Poject 2D pixel using cam2
  std::vector<cv::Point2f> pixels2;
  cv::Mat R2;
  r2.copyTo(R2);
  if (R2.cols == 3 && R2.rows == 3) {
    cv::Rodrigues(r2, R2);
  }
  cv::projectPoints(points, R2, t2, intrinsic2, dist_coeffs2, pixels2);

  return pixels2;
}

bool Pinhole::SaveResult() {
  std::ofstream out_file("../result/pinhole_intrinsics.json");
  if (!out_file.is_open()) {
    std::cerr << "save file failed" << std::endl;
    return false;
  }

  Json::Value root, distortion, intrinsic;

  distortion["k1"] = dist_coeffs_.at<double>(0, 0);
  distortion["k2"] = dist_coeffs_.at<double>(0, 1);
  distortion["p1"] = dist_coeffs_.at<double>(0, 2);
  distortion["p2"] = dist_coeffs_.at<double>(0, 3);
  if (!config_.pinhole_fix_k3) {
    distortion["k3"] = dist_coeffs_.at<double>(0, 4);
  }

  intrinsic["cx"] = intrinsic_.at<double>(0, 2);
  intrinsic["cy"] = intrinsic_.at<double>(1, 2);
  intrinsic["fx"] = intrinsic_.at<double>(0, 0);
  intrinsic["fy"] = intrinsic_.at<double>(1, 1);

  root["distortion"] = Json::Value(distortion);
  root["intrinsic"] = Json::Value(intrinsic);

  out_file << root.toStyledString();

  out_file.close();
  std::cout << "save json file success: pinhole_intrinsics.json" << std::endl;

  SaveRemapFile();
  SavePixelError();

  return true;
}

bool Pinhole::ReadIntrinsics(const std::string &intrinsic_file) {
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

    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    dist_coeffs_.at<double>(0, 0) = root["distortion"]["k1"].asDouble();
    dist_coeffs_.at<double>(0, 1) = root["distortion"]["k2"].asDouble();
    dist_coeffs_.at<double>(0, 2) = root["distortion"]["p1"].asDouble();
    dist_coeffs_.at<double>(0, 3) = root["distortion"]["p2"].asDouble();
    dist_coeffs_.at<double>(0, 4) = root["distortion"]["k3"].asDouble();
  } else {
    std::cerr << "json parse error" << std::endl;
    return false;
  }
  in_file.close();

  return true;
}

cv::Mat Pinhole::Undistort(const cv::Mat &distorted_img) {
  cv::Mat undistorted_img;
  if (undistort_remap_.first.empty() || undistort_remap_.second.empty()) {
    cv::initUndistortRectifyMap(
        intrinsic_, dist_coeffs_, cv::Mat(), intrinsic_, config_.img_size,
        CV_32FC1, undistort_remap_.first, undistort_remap_.second);
  }
  cv::remap(distorted_img, undistorted_img, undistort_remap_.first,
            undistort_remap_.second, cv::INTER_LINEAR);

  return undistorted_img;
}

void Pinhole::CalReprojectError() {
  std::vector<float> reproject_errors(object_points_.size());
  std::vector<cv::Point2f> projected_points;
  size_t point_num = 0;
  double total_error = 0, error;

  for (size_t i = 0; i < object_points_.size(); ++i) {
    cv::projectPoints(object_points_[i], rvecs_[i], tvecs_[i], intrinsic_,
                      dist_coeffs_, projected_points);

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