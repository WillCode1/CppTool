#include "extrinsic_calibrator.h"

ExtrinsicCalibrator::ExtrinsicCalibrator() {
  cc_ = Eigen::MatrixXf::Zero(1, 2);
  fc_ = Eigen::MatrixXf::Zero(1, 2);
  distcoeffs_ = Eigen::MatrixXf::Zero(1, 5);

  scale_ = 1.0f;
}
ExtrinsicCalibrator::~ExtrinsicCalibrator() {}

void ExtrinsicCalibrator::LoadImage(std::string filename) {
  cv::Mat image_color = cv::imread(filename, cv::IMREAD_COLOR);

  cv::cvtColor(image_color, image_gray_, cv::COLOR_BGR2GRAY);
  image_height_ = image_gray_.rows;
  image_width_ = image_gray_.cols;
}

void ExtrinsicCalibrator::LoadImage(const cv::Mat &image_color) {
  cv::cvtColor(image_color, image_gray_, cv::COLOR_BGR2GRAY);
  image_height_ = image_gray_.rows;
  image_width_ = image_gray_.cols;
}

void ExtrinsicCalibrator::LoadParameter(std::string str_camera_info) {
  char str[1024];

  float focal_length_x, focal_length_y;
  float optical_center_x, optical_center_y;
  float camera_height;

  float r1, r2, t1, t2;

  FILE *pfile = fopen(str_camera_info.c_str(), "rt");
  if (pfile != NULL) {

    fscanf(pfile, "%s %f\n", str, &focal_length_x);
    std::cout << "read focal_length_x = " << focal_length_x << std::endl;
    fscanf(pfile, "%s %f\n", str, &focal_length_y);
    std::cout << "read focal_length_y = " << focal_length_y << std::endl;
    fscanf(pfile, "%s %f\n", str, &optical_center_x);
    std::cout << "read optical_center_x = " << optical_center_x << std::endl;
    fscanf(pfile, "%s %f\n", str, &optical_center_y);
    std::cout << "read optical_center_y = " << optical_center_y << std::endl;

    fscanf(pfile, "%s %f\n", str, &camera_height);
    std::cout << "read camera_height = " << camera_height << std::endl;

    fscanf(pfile, "%s %f %f\n", str, &r1, &r2);
    std::cout << "read radial_distortion = " << r1 << " " << r2 << std::endl;
    fscanf(pfile, "%s %f %f\n", str, &t1, &t2);
    std::cout << "read tangential_distortion = " << t1 << " " << t2
              << std::endl;

    fclose(pfile);

  } else {
    std::cout << "cannot open file " << str_camera_info << std::endl;
    exit(-1);
  }

  fc_ = Eigen::MatrixXf::Constant(1, 2, 0);
  cc_ = Eigen::MatrixXf::Constant(1, 2, 0);
  distcoeffs_ = Eigen::MatrixXf::Constant(1, 5, 0);
  fc_(0, 0) = focal_length_x;
  fc_(0, 1) = focal_length_y;
  cc_(0, 0) = optical_center_x;
  cc_(0, 1) = optical_center_y;
  distcoeffs_(0, 0) = r1;
  distcoeffs_(0, 1) = r2;
  distcoeffs_(0, 2) = t1;
  distcoeffs_(0, 3) = t2;
  scale_ = 1.0f;

  std::cout << fc_ << std::endl << cc_ << std::endl << distcoeffs_ << std::endl;
}

void ExtrinsicCalibrator::LoadParameter(
    const float &camera_height, nullmax_perception::CameraIntrinsic &intrinsic,
    nullmax_perception::CameraDistortCoef &distort_coef) {

  fc_ = Eigen::MatrixXf::Constant(1, 2, 0);
  cc_ = Eigen::MatrixXf::Constant(1, 2, 0);
  distcoeffs_ = Eigen::MatrixXf::Constant(1, 5, 0);
  fc_(0, 0) = intrinsic.fx;
  fc_(0, 1) = intrinsic.fy;
  cc_(0, 0) = intrinsic.cx;
  cc_(0, 1) = intrinsic.cy;
  distcoeffs_(0, 0) = distort_coef.coef[0];
  distcoeffs_(0, 1) = distort_coef.coef[1];
  distcoeffs_(0, 2) = distort_coef.coef[2];
  distcoeffs_(0, 3) = distort_coef.coef[3];
  scale_ = 1.0f;

  std::cout << fc_ << std::endl << cc_ << std::endl << distcoeffs_ << std::endl;
}

cv::Mat ExtrinsicCalibrator::Undistortion(float alpha, float max_range,
                                          int mode) {
  cv::Mat image_gray = image_gray_;
  float scale = scale_;
  int image_height = image_gray.rows;
  int image_width = image_gray.cols;

  int new_image_height = std::floor(image_height * scale);
  int new_image_width = std::floor(image_width * scale);

  cv::Mat undistortion_image =
      cv::Mat::zeros(new_image_height, new_image_width, CV_32FC1);

  float camera_coord_i, camera_coord_j, delta_camera_coord_i,
      delta_camera_coord_j, weight_h = 0.0f, weight_v = 0.0f;
  float image_coord_i, image_coord_j, r2, r4, r6, t1, t2, t3, temp, temp1,
      temp2;

  for (int i = 0; i < undistortion_image.rows; ++i)
    for (int j = 0; j < undistortion_image.cols; ++j) {
      camera_coord_i =
          (i - cc_(0, 1) - (new_image_height - image_height) / 2) / fc_(0, 1);
      camera_coord_j = (j - cc_(0, 0) - (new_image_width - image_width) / 2 -
                        alpha * camera_coord_i) /
                       fc_(0, 0);

      r2 =
          (camera_coord_i * camera_coord_i) + (camera_coord_j * camera_coord_j);
      r4 = r2 * r2;
      r6 = r2 * r4;

      float cdist = 1 + distcoeffs_(0, 0) * r2 + distcoeffs_(0, 1) * r4 +
                    distcoeffs_(0, 4) * r6;

      t1 = 2 * camera_coord_i * camera_coord_j;
      t2 = r2 + 2 * camera_coord_j * camera_coord_j;
      t3 = r2 + 2 * camera_coord_i * camera_coord_i;

      delta_camera_coord_j = distcoeffs_(0, 2) * t1 + distcoeffs_(0, 3) * t2;
      delta_camera_coord_i = distcoeffs_(0, 2) * t3 + distcoeffs_(0, 3) * t1;

      camera_coord_j = camera_coord_j * cdist + delta_camera_coord_j;
      camera_coord_i = camera_coord_i * cdist + delta_camera_coord_i;

      // reconvert to image coordinate system
      image_coord_j =
          fc_(0, 0) * camera_coord_j + alpha * camera_coord_i + cc_(0, 0);
      image_coord_i = fc_(0, 1) * camera_coord_i + cc_(0, 1);

      float j1 = std::floor(image_coord_j);
      float i1 = std::floor(image_coord_i);

      if (mode == 1) {
        weight_h = image_coord_j - j1;
        weight_v = image_coord_i - i1;
      } else if (mode == 2) {
        weight_h = std::floor(image_coord_j - j1 + 0.5);
        weight_v = std::floor(image_coord_i - i1 + 0.5);
      }

      if ((j1 >= 0) && (j1 < image_width - 1) && (i1 >= 0) &&
          (i1 < image_height - 1)) {
        temp1 = (float)image_gray.at<uchar>(i1 + 0, j1 + 0) * (1.0 - weight_h) +
                (float)image_gray.at<uchar>(i1 + 0, j1 + 1) * weight_h;
        temp2 = (float)image_gray.at<uchar>(i1 + 1, j1 + 0) * (1.0 - weight_h) +
                (float)image_gray.at<uchar>(i1 + 1, j1 + 1) * weight_h;
        temp = temp1 * (1.0 - weight_v) + temp2 * weight_v;
        undistortion_image.at<float>(i, j) =
            std::max(0.0f, std::min(max_range, roundf(temp))); // / 255.0f;
      }
    }
  undistortion_image_ = undistortion_image;
  return undistortion_image;
}
