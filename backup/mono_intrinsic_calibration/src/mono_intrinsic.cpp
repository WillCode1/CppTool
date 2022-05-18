#include "mono_intrinsic.h"
namespace nullmax_perception {

namespace calibration {

bool MonoIntrinsic::Init(const std::string &config_file) {
  if (!ReadConfigFile(config_file)) {
    return false;
  }
  InitDatabase();
  camera_model_ = ModelFactory::CreateModel(config_.camera_cfg);

  return (camera_model_ != nullptr);
}

void MonoIntrinsic::InitDatabase() {
  database_.progress = cv::Vec4f(0, 0, 0, 0);
  database_.progress_bins.resize(4);
  database_.progress_bins[0].resize(20, false);
  database_.progress_bins[1].resize(20, false);
  database_.progress_bins[2].resize(12, false);
  database_.progress_bins[3].resize(15, false);
  database_.bin_flags.resize(4);
  for (auto &bin_flag : database_.bin_flags) {
    bin_flag.resize(3, false);
  }
}

std::vector<std::string>
MonoIntrinsic::GetFileNames(const std::string &file_path) {
  std::vector<std::string> file_names;
  DIR *pDir;
  struct dirent *ptr;
  if (!(pDir = opendir(file_path.c_str()))) {
    std::cerr << "image file path error" << std::endl;
    return file_names;
  }
  while ((ptr = readdir(pDir)) != 0) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
      file_names.push_back(ptr->d_name);
    }
  }
  closedir(pDir);
  return file_names;
}

bool MonoIntrinsic::ReadConfigFile(const std::string &config_file) {
  cv::FileStorage fs(config_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "config file open failed" << std::endl;
    return false;
  }

  config_.camera_cfg.calib_type = static_cast<std::string>(fs["calib_type"]);
  config_.image_path = static_cast<std::string>(fs["image_path"]) + "/";

  cv::FileNode node = fs["image_size"];
  config_.camera_cfg.img_size.width = static_cast<int>(node["width"]);
  config_.camera_cfg.img_size.height = static_cast<int>(node["height"]);

  node = fs["board_size"];
  config_.camera_cfg.board_size.width = static_cast<int>(node["width"]);
  config_.camera_cfg.board_size.height = static_cast<int>(node["height"]);

  config_.camera_cfg.board_unit_length =
      static_cast<float>(fs["board_unit_length"]);
  config_.is_show_undistorted = static_cast<bool>((int)fs["show_undistort"]);

  config_.camera_cfg.pinhole_fix_p =
      static_cast<bool>((int)fs["pinhole_fix_p"]);
  config_.camera_cfg.pinhole_fix_k3 =
      static_cast<bool>((int)fs["pinhole_fix_k3"]);

  config_.camera_cfg.undistort_type =
      static_cast<std::string>(fs["undistort_type"]);

  config_.camera_cfg.use_FOV = static_cast<bool>((int)fs["use_FOV"]);
  config_.camera_cfg.target_FOV = static_cast<float>(fs["target_FOV"]);

  config_.camera_cfg.use_specified_focal_length =
      static_cast<bool>((int)fs["use_specified_focal_length"]);
  config_.camera_cfg.specified_fx = static_cast<double>(fs["specified_fx"]);
  config_.camera_cfg.specified_fy = static_cast<double>(fs["specified_fy"]);

  fs.release();

  std::cout << config_ << std::endl;

  return ConfigValidCheck();
}

std::ostream &operator<<(std::ostream &os,
                         const MonoIntrinsic::CalibrationConfig &cfg) {
  os << "mono intrinsic parameters" << std::endl;
  os << "calib_type: " << cfg.camera_cfg.calib_type << std::endl;
  os << "image_path: " << cfg.image_path << std::endl;
  os << "image width: " << cfg.camera_cfg.img_size.width << std::endl;
  os << "image height: " << cfg.camera_cfg.img_size.height << std::endl;
  os << "board width: " << cfg.camera_cfg.board_size.width << std::endl;
  os << "board height: " << cfg.camera_cfg.board_size.height << std::endl;
  os << "board unit length: " << cfg.camera_cfg.board_unit_length << std::endl;
  os << "show_undistor: " << cfg.is_show_undistorted << std::endl;
  if (cfg.camera_cfg.calib_type == "pinhole" && cfg.camera_cfg.pinhole_fix_p) {
    os << "enable fix p1,p2" << std::endl;
  }
  if (cfg.camera_cfg.calib_type == "pinhole" && cfg.camera_cfg.pinhole_fix_k3) {
    os << "enable fix k3" << std::endl;
  }
  os << "undistort_type: " << cfg.camera_cfg.undistort_type << std::endl;

  os << "use_FOV: " << cfg.camera_cfg.use_FOV << std::endl;
  os << "target_FOV: " << cfg.camera_cfg.target_FOV << std::endl;
  os << "use_specified_focal_length: "
     << cfg.camera_cfg.use_specified_focal_length << std::endl;
  os << "specified_fx: " << cfg.camera_cfg.specified_fx << std::endl;
  os << "specified_fy: " << cfg.camera_cfg.specified_fy << std::endl;

  return os;
}

bool MonoIntrinsic::ConfigValidCheck() {
  bool ok = false;
  ok |= (config_.camera_cfg.calib_type == "pinhole") |
        (config_.camera_cfg.calib_type == "fisheye") |
        (config_.camera_cfg.calib_type == "ocam");
  ok &= (config_.camera_cfg.img_size.width > 0) &
        (config_.camera_cfg.img_size.height > 0);
  ok &= (config_.camera_cfg.board_size.width > 0) &
        (config_.camera_cfg.board_size.height > 0);
  ok &= (config_.camera_cfg.board_unit_length > 0);
  ok &= (config_.camera_cfg.undistort_type == "auto") |
        (config_.camera_cfg.undistort_type == "ratio");
  ok &= !(config_.camera_cfg.use_FOV &&
          config_.camera_cfg.use_specified_focal_length);
  if (config_.camera_cfg.use_FOV) {
    ok &= (config_.camera_cfg.target_FOV > 0);
  }
  if (config_.camera_cfg.use_specified_focal_length) {
    ok &= (config_.camera_cfg.specified_fx > 0);
    ok &= (config_.camera_cfg.specified_fy > 0);
  }

  if (!ok) {
    std::cerr << "config param invaild, please check" << std::endl;
  }

  return ok;
}

bool MonoIntrinsic::Calib() {
  InputData();

  std::cout << "calibrating ..." << std::endl;
  if (!camera_model_->RunCalibration(database_.corners)) {
    return false;
  }

  camera_model_->CalReprojectError();

  if (config_.is_show_undistorted) {
    for (int i = 0; i < database_.images.size(); ++i) {
      cv::Mat undistorted_img = Undistort(database_.images[i]);
      if (undistorted_img.empty()) {
        continue;
      }
      std::cout << "showing undistorted image " << i + 1 << " ..." << std::endl;
      cv::imshow("compare", undistorted_img);
      cv::waitKey(0);
    }
  } else {
    // to generate remap
    camera_model_->Undistort(database_.images.front());
  }

  return camera_model_->SaveResult();
}

bool MonoIntrinsic::ViewUndistort(const std::string &distorted_image_file,
                                  const std::string &intrinsic_file) {
  std::string suffix = intrinsic_file.substr(intrinsic_file.length() - 4,
                                             intrinsic_file.length());
  bool ok;
  if (suffix == ".bin") {
    ok = camera_model_->ReadRemapFile(intrinsic_file);
  } else if (suffix == ".txt" || suffix == "json") {
    ok = camera_model_->ReadIntrinsics(intrinsic_file);
  } else {
    std::cerr << "not support " << suffix << " file" << std::endl;
    return false;
  }
  if (ok) {
    cv::Mat raw_img = cv::imread(distorted_image_file);
    if (raw_img.empty()) {
      std::cerr << "can not read img: " << std::endl;
      std::cerr << distorted_image_file << std::endl;
      return false;
    }
    cv::Mat undistorted_img = Undistort(raw_img);
    if (undistorted_img.empty()) {
      std::cerr << "undistorted_img empty" << std::endl;
      ok = false;
    } else {
      cv::imshow("undistorted_img", undistorted_img);
      cv::waitKey(0);
    }
  }

  return ok;
}

cv::Mat MonoIntrinsic::Undistort(const cv::Mat &distorted_img) {
  cv::Mat raw_img = distorted_img.clone();
  cv::Mat undistorted_img = camera_model_->Undistort(raw_img);
  if (undistorted_img.empty()) {
    return cv::Mat();
  }

  cv::resize(raw_img, raw_img, cv::Size(raw_img.cols / 3, raw_img.rows / 3));
  cv::resize(undistorted_img, undistorted_img,
             cv::Size(undistorted_img.cols / 3, undistorted_img.rows / 3));

  cv::putText(raw_img, "Before", cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(0, 0, 255), 4, 8);
  cv::putText(undistorted_img, "After", cv::Point(50, 60),
              cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 4, 8);

  int compare_width = 2 * std::max(raw_img.cols, undistorted_img.cols);
  int compare_height = std::max(raw_img.rows, undistorted_img.rows);
  cv::Mat compare_image = cv::Mat(compare_height, compare_width, raw_img.type(),
                                  cv::Scalar(0, 0, 0));
  cv::Mat part1 = compare_image(cv::Rect(0, 0, raw_img.cols, raw_img.rows));
  cv::Mat part2 = compare_image(
      cv::Rect(raw_img.cols, 0, undistorted_img.cols, undistorted_img.rows));
  raw_img.copyTo(part1);
  undistorted_img.copyTo(part2);
  return compare_image;
}

void MonoIntrinsic::InputData() {
  std::vector<std::string> file_names = GetFileNames(config_.image_path);
  for (int img_idx = 0, valid_count = 0, size = file_names.size();
       img_idx < size; ++img_idx) {
    cv::Mat cur_img =
        cv::imread(config_.image_path + file_names[img_idx], cv::IMREAD_COLOR);

    if (cur_img.empty()) {
      std::cerr << "read image error, skip file: " + file_names[img_idx]
                << std::endl;
      continue;
    }

    std::vector<cv::Point2f> corners = DetectCorners(cur_img);
    if (corners.empty()) {
      std::cerr << "image can not detect corners: " << file_names[img_idx]
                << std::endl;
      continue;
    }
    std::cout << "No." << ++valid_count << " image: " << file_names[img_idx]
              << std::endl;

    UpdateDatabase(cur_img, corners);
    cv::waitKey(0);
  }
  camera_model_->SetImages(database_.images);
  cv::destroyAllWindows();
}

std::vector<cv::Point2f> MonoIntrinsic::DetectCorners(const cv::Mat &img) {
  int &width = config_.camera_cfg.img_size.width;
  int &height = config_.camera_cfg.img_size.height;
  float scale = std::sqrt(static_cast<float>(width * height) / (640. * 480.));
  cv::Mat sampled_img;
  if (scale > 1.0) {
    cv::resize(img, sampled_img,
               cv::Size(int(width / scale), int(height / scale)));
  } else {
    sampled_img = img.clone();
  }

  std::vector<cv::Point2f> sampled_corners, corners;
  int chessboard_flags = cv::CALIB_CB_ADAPTIVE_THRESH |
                         cv::CALIB_CB_NORMALIZE_IMAGE |
                         cv::CALIB_CB_FILTER_QUADS | cv::CALIB_CB_FAST_CHECK;
  bool find_corners =
      cv::findChessboardCorners(sampled_img, config_.camera_cfg.board_size,
                                sampled_corners, chessboard_flags);

  int &board_rows = config_.camera_cfg.board_size.height;
  int &board_cols = config_.camera_cfg.board_size.width;
  if (find_corners) {
    float min_dist(FLT_MAX);
    for (int row = 0; row < board_rows; ++row) {
      for (int col = 0; col < board_cols - 1; ++col) {
        int index = row * board_rows + col;
        float dx = sampled_corners[index].x - sampled_corners[index + 1].x;
        float dy = sampled_corners[index].y - sampled_corners[index + 1].y;
        float dist = std::sqrt(dx * dx + dy * dy);
        min_dist = std::min(min_dist, dist);
      }
    }
    for (int row = 0; row < board_rows - 1; ++row) {
      for (int col = 0; col < board_cols; ++col) {
        int index = row * board_rows + col;
        float dx =
            sampled_corners[index].x - sampled_corners[index + board_cols].x;
        float dy =
            sampled_corners[index].y - sampled_corners[index + board_cols].y;
        float dist = std::sqrt(dx * dx + dy * dy);
        min_dist = std::min(min_dist, dist);
      }
    }
    int radius = cvCeil(min_dist * 0.5);
    cv::Mat sampled_gary;
    cv::cvtColor(sampled_img, sampled_gary, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(
        sampled_gary, sampled_corners, cv::Size(radius, radius),
        cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                         0.1));

    cv::drawChessboardCorners(sampled_img, config_.camera_cfg.board_size,
                              cv::Mat(sampled_corners), find_corners);

    cv::imshow("view corners", sampled_img);

    if (scale > 1.0) {
      float scale_x = float(width) / float(sampled_img.cols);
      float scale_y = float(height) / float(sampled_img.rows);

      for (auto &corner : sampled_corners) {
        corner.x *= scale_x;
        corner.y *= scale_y;
      }
      cv::Mat gary;
      cv::cvtColor(img, gary, cv::COLOR_BGR2GRAY);
      int radius = cvCeil(scale);
      cv::cornerSubPix(
          gary, sampled_corners, cv::Size(radius, radius), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                           0.1));
    }
    corners = sampled_corners;
  }

  return corners;
}

void MonoIntrinsic::UpdateDatabase(const cv::Mat &img,
                                   const std::vector<cv::Point2f> &corners) {
  cv::Vec4f pose = GetBoardPose(corners);

  // choose image according to pose
  // if (IsGoodPose(pose)) {
  // database_.images.push_back(img);
  // database_.corners.push_back(corners);
  // database_.poses.push_back(pose);
  // }

  database_.images.push_back(img);
  database_.corners.push_back(corners);
  database_.poses.push_back(pose);

  GetProgress(pose);
}

cv::Vec4f MonoIntrinsic::GetBoardPose(const std::vector<cv::Point2f> &corners) {
  int &board_width = config_.camera_cfg.board_size.width;
  int &board_height = config_.camera_cfg.board_size.height;
  cv::Vec2f up_left(corners.front().x, corners.front().y);
  cv::Vec2f up_right(corners[board_width - 1].x, corners[board_width - 1].y);
  cv::Vec2f down_left(corners[board_width * (board_height - 1)].x,
                      corners[board_width * (board_height - 1)].y);
  cv::Vec2f down_right(corners.back().x, corners.back().y);

  // compute corners area, |p X q|/2
  // reference: http://mathworld.wolfram.com/Quadrilateral.html
  cv::Vec2f a = up_right - up_left;
  cv::Vec2f b = down_right - up_right;
  cv::Vec2f c = down_left - down_right;
  cv::Vec2f p = b + c;
  cv::Vec2f q = a + b;
  float area = std::fabs(p[0] * q[1] - p[1] * q[0]) / 2.;
  float border = std::sqrt(area);

  // compute X Y
  float mean_x(0.), mean_y(0.);
  for (auto &corner : corners) {
    mean_x += corner.x;
    mean_y += corner.y;
  }
  mean_x /= static_cast<float>(corners.size());
  mean_y /= static_cast<float>(corners.size());
  int &img_width = config_.camera_cfg.img_size.width;
  int &img_height = config_.camera_cfg.img_size.height;
  float X =
      std::min(float(1.0), std::max(float(0.0), (mean_x - border / 2) /
                                                    (img_width - border)));
  float Y =
      std::min(float(1.0), std::max(float(0.0), (mean_y - border / 2) /
                                                    (img_height - border)));

  // compute size
  float size = std::sqrt(area / float(img_width * img_height));

  // compute skew
  cv::Vec2f ab = up_left - up_right;
  cv::Vec2f cb = down_right - up_right;
  float angle = std::acos(ab.dot(cb) / (cv::norm(ab) * cv::norm(cb)));
  float skew = angle * 180. / CV_PI;
  // ROS method
  // float skew = std::min(float(1.0), 2 * (float)std::abs(CV_PI / 2. - angle));

  return cv::Vec4f(X, Y, size, skew);
}

bool MonoIntrinsic::IsGoodPose(const cv::Vec4f &pose) {
  // cal min dist of all poses in database
  float min_dist(FLT_MAX);
  for (auto &db_pose : database_.poses) {
    cv::Vec4f delta = db_pose - pose;
    float dist = std::fabs(delta[0]) + std::fabs(delta[1]) +
                 std::fabs(delta[2]) + std::fabs(delta[3]);
    if (dist < min_dist) {
      min_dist = dist;
    }
  }

  return (min_dist > 0.2);
}

bool MonoIntrinsic::GetProgress(const cv::Vec4f &pose) {
  if (database_.poses.empty()) {
    return false;
  }
  bool ok = ComputeProgress(pose);

  // show progress info
  std::vector<std::string> str = {"X: ", "Y: ", "Size: ", "Skew: "};
  if (database_.last_progress != database_.progress) {
    database_.last_progress = database_.progress;
    for (int i = 0; i < 4; ++i) {
      int score = int(database_.progress[i] * 100);
      if (score <= 50) {
        std::cout << "\033[0;31m";
      } else if (score >= 90) {
        std::cout << "\033[0;32m";
      } else {
        std::cout << "\033[0;33m";
      }
      std::cout << str[i] << std::setw(3) << score << "    ";
      std::cout << "\033[0m";
    }
    std::cout << std::endl;
  }
  return ((int)database_.poses.size() >= 40) || ok;
}

bool MonoIntrinsic::ComputeProgress(const cv::Vec4f &pose) {
  // vote for score bin
  std::array<int, 4> bin_idx_set = {
      int(pose[0] * 100) / 5, int(pose[1] * 100) / 5,
      int((pose[2] * 100 - 20.) / 2.5), int((pose[3] - 75.) / 2.)};
  std::array<double, 4> bin_base_score = {0.2, 0.2, 0.4, 0.4};
  std::array<int, 4> low_bound = {6, 6, 2, 5};
  std::array<int, 4> up_bound = {13, 13, 9, 9};
  for (int i = 0; i < 4; ++i) {
    int bin_idx = bin_idx_set[i];

    // situation if out of vote range
    if (bin_idx < 0) {
      bin_idx = 0;
    } else if (bin_idx >= database_.progress_bins[i].size()) {
      bin_idx = int(database_.progress_bins[i].size()) - 1;
    }

    if (!database_.progress_bins[i][bin_idx]) {
      database_.progress_bins[i][bin_idx] = true;

      int bin_flag;
      if (bin_idx < low_bound[i]) {
        bin_flag = 0;
      } else if (bin_idx > up_bound[i]) {
        bin_flag = 2;
      } else {
        bin_flag = 1;
      }
      if (database_.bin_flags[i][bin_flag]) {
        database_.progress[i] = std::min(1.0, database_.progress[i] + 0.05);
      } else {
        database_.progress[i] =
            std::min(1.0, database_.progress[i] + bin_base_score[i]);
        database_.bin_flags[i][bin_flag] = true;
      }
    }
  }

  bool ok = true;
  for (int i = 0; i < 4; ++i) {
    ok &= (database_.progress[i] >= 1.);
  }

  return ok;
}

bool MonoIntrinsic::ComputeProgress_ROS() {
  auto cal_min = [&](const cv::Vec4f &v1, const cv::Vec4f &v2) -> cv::Vec4f {
    float p0 = std::min(v1[0], v2[0]);
    float p1 = std::min(v1[1], v2[1]);
    float p2 = std::min(v1[2], v2[2]);
    float p3 = std::min(v1[3], v2[3]);
    return cv::Vec4f(p0, p1, p2, p3);
  };
  auto cal_max = [&](const cv::Vec4f &v1, const cv::Vec4f &v2) -> cv::Vec4f {
    float p0 = std::max(v1[0], v2[0]);
    float p1 = std::max(v1[1], v2[1]);
    float p2 = std::max(v1[2], v2[2]);
    float p3 = std::max(v1[3], v2[3]);
    return cv::Vec4f(p0, p1, p2, p3);
  };

  if (database_.poses.empty()) {
    return false;
  }
  cv::Vec4f min_pose = database_.poses.front();
  cv::Vec4f max_pose = database_.poses.front();
  for (auto &each_pose : database_.poses) {
    min_pose = cal_min(each_pose, min_pose);
    max_pose = cal_max(each_pose, max_pose);
  }
  min_pose[2] = 0.0;
  min_pose[3] = 0.0;
  cv::Vec4f pose_range(0.7, 0.7, 0.4, 0.5);
  std::vector<std::string> str = {"X: ", "Y: ", "Size: ", "Skew: "};
  bool ok = true;
  for (int i = 0; i < 4; ++i) {
    database_.progress[i] =
        std::min(float(1.0), (max_pose[i] - min_pose[i]) / pose_range[i]);
    ok &= (database_.progress[i] == 1.0);
  }
  if (database_.last_progress != database_.progress) {
    database_.last_progress = database_.progress;
    for (int i = 0; i < 4; ++i) {
      if (database_.progress[i] <= 0.5) {
        std::cout << "\033[0;31m";
      } else if (database_.progress[i] >= 0.9) {
        std::cout << "\033[0;32m";
      } else {
        std::cout << "\033[0;33m";
      }
      std::cout << str[i] << std::setw(3) << int(database_.progress[i] * 100)
                << "    ";
      std::cout << "\033[0m";
    }
    std::cout << std::endl;
  }
  return ((int)database_.poses.size() >= 40) || ok;
}

std::shared_ptr<MonoIntrinsicInterface> CreateCalibrator() {
  std::shared_ptr<MonoIntrinsicInterface> interface_ptr =
      std::make_shared<MonoIntrinsic>();
  return interface_ptr;
}

} // calibration
} // nullmax_perception