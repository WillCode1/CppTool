#include "model_base.h"

namespace nullmax_perception {
namespace calibration {

bool ModelBase::SavePixelError() {
  std::ofstream out_file("../result/pixel_error.txt");
  if (!out_file.is_open()) {
    std::cerr << "save pixel error failed" << std::endl;
    return false;
  }

  for (auto &image : pixel_repro_errors_) {
    for (auto &error : image) {
      out_file << error.x << " " << error.y << std::endl;
    }
  }
  out_file.close();
  std::cout << "save txt file success: pixel_error.txt" << std::endl;

  ShowPixelError();

  return true;
}

void ModelBase::ShowPixelError() {
  cv::Mat plot = cv::Mat(700, 650, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Rect rect(100, 100, 500, 500);
  cv::rectangle(plot, rect, cv::Scalar(255, 255, 255), 1);

  float interval = 500. / 6.;
  std::vector<std::string> text = {"-1.0", "-0.5", "0.0", "0.5", "1.0"};
  for (int i = 1; i <= 5; ++i) {
    // X
    cv::line(plot, cv::Point2f(100. + i * interval, 590),
             cv::Point2f(100. + i * interval, 600), cv::Scalar(255, 255, 255));
    int offset = i > 2 ? 10 : 25;
    cv::putText(plot, text[i - 1], cv::Point(100 + i * interval - offset, 620),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255));
    // Y
    cv::line(plot, cv::Point(100, 100. + i * interval),
             cv::Point(110, 100. + i * interval), cv::Scalar(255, 255, 255));
    offset = i > 2 ? 25 : 10;
    cv::putText(plot, text[i - 1],
                cv::Point(45 + offset, 105 + (6 - i) * interval),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255));
  }
  cv::putText(plot, "-1.5", cv::Point(100 - 25, 620), cv::FONT_HERSHEY_COMPLEX,
              0.5, cv::Scalar(255, 255, 255));
  cv::putText(plot, "1.5", cv::Point(600 - 15, 620), cv::FONT_HERSHEY_COMPLEX,
              0.5, cv::Scalar(255, 255, 255));
  cv::putText(plot, "-1.5", cv::Point(45 + 10, 100 + 6 * interval),
              cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255));
  cv::putText(plot, "1.5", cv::Point(45 + 25, 105), cv::FONT_HERSHEY_COMPLEX,
              0.5, cv::Scalar(255, 255, 255));
  cv::putText(plot, "Pixel Reprojection Error", cv::Point(140, 50),
              cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
  cv::putText(plot, "X / pixel", cv::Point(270, 670), cv::FONT_HERSHEY_COMPLEX,
              1, cv::Scalar(255, 255, 255));
  cv::putText(plot, "Y /", cv::Point(20, 335), cv::FONT_HERSHEY_COMPLEX, 1,
              cv::Scalar(255, 255, 255));
  cv::putText(plot, "pixel", cv::Point(0, 385), cv::FONT_HERSHEY_COMPLEX, 1,
              cv::Scalar(255, 255, 255));

  srand(time(NULL));
  int outlier_count(0);
  for (auto &image : pixel_repro_errors_) {
    int r = rand() % 256;
    int g = rand() % 256;
    int b = rand() % 256;
    for (auto &error : image) {
      if (error.x <= 1.5 && error.x >= -1.5 && error.y <= 1.5 &&
          error.y >= -1.5) {
        cv::circle(plot,
                   cv::Point2f(error.x * interval / 0.5 + 100 + 3 * interval,
                               error.y * interval / 0.5 + 100 + 3 * interval),
                   1, cv::Scalar(r, g, b), -1);
      } else {
        outlier_count++;
      }
    }
  }
  cv::putText(plot, "(", cv::Point(175, 80), cv::FONT_HERSHEY_COMPLEX, 0.6,
              cv::Scalar(255, 255, 255));
  cv::putText(plot, std::to_string(outlier_count), cv::Point(185, 80),
              cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 165, 255));
  cv::putText(plot, "outliers out of plot range)", cv::Point(225, 80),
              cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(255, 255, 255));
  std::cout << "\033[0;33m" << outlier_count << "\033[0m";
  std::cout << " outliers, error over 1.5 pixel" << std::endl;
  cv::imwrite("../result/pixel_repro_error.bmp", plot);
  cv::imshow("pixel_repro_error", plot);
  cv::waitKey(0);
}

bool ModelBase::ReadRemapFile(const std::string &remap_file) {
  std::ifstream in_file;
  in_file.open(remap_file.c_str(), std::ios_base::in | std::ios::binary);
  if (!in_file.is_open()) {
    std::cerr << "read undistort remap failed" << std::endl;
    return false;
  }
  undistort_remap_.first =
      cv::Mat::zeros(config_.img_size.height, config_.img_size.width, CV_32FC1);
  undistort_remap_.second =
      cv::Mat::zeros(config_.img_size.height, config_.img_size.width, CV_32FC1);

  for (int row = 0; row < config_.img_size.height; ++row) {
    for (int col = 0; col < config_.img_size.width; ++col) {
      float value;
      in_file.read((char *)&value, sizeof(float));
      undistort_remap_.first.at<float>(row, col) = value;
    }
  }
  for (int row = 0; row < config_.img_size.height; ++row) {
    for (int col = 0; col < config_.img_size.width; ++col) {
      float value;
      in_file.read((char *)&value, sizeof(float));
      undistort_remap_.second.at<float>(row, col) = value;
    }
  }

  return true;
}

bool ModelBase::SaveRemapFile() {
  std::ofstream out_file;
  out_file.open("../result/undistort_remap.bin",
                std::ios_base::out | std::ios::binary);
  if (!out_file.is_open()) {
    std::cerr << "save undistort remap failed" << std::endl;
    return false;
  }
  int &rows = undistort_remap_.first.rows;
  int &cols = undistort_remap_.first.cols;
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      float value = undistort_remap_.first.at<float>(row, col);
      out_file.write((char *)&value, sizeof(float));
    }
  }
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      float value = undistort_remap_.second.at<float>(row, col);
      out_file.write((char *)&value, sizeof(float));
    }
  }
  std::cout << "save bin file success: undistort_remap.bin" << std::endl;
  return true;
}
} // calibration
} // nullmax_perception
