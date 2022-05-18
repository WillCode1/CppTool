#include "eigen_cv_utils.h"
#include <iostream>

bool CvMat2EigenMat(const Mat &mat_cv, EigenMat &mat_eigen) {

  const float *pcdata = mat_cv.ptr<float>(0);
  float *pdata = const_cast<float *>(pcdata);

  mat_eigen = Eigen::Map<EigenMat>(pdata, mat_cv.rows, mat_cv.cols);

  return true;
}

bool EigenMat2CvMat(const EigenMat &mat_eigen, Mat &mat_cv) {

  const float *pcdata = mat_eigen.data();
  float *pdata = const_cast<float *>(pcdata);
  // Eigen::Map<EigenMat>(pdata, mat_cv.rows, mat_cv.cols) = mat_eigen;

  Mat mat = Mat(mat_eigen.rows(), mat_eigen.cols(), CV_32F, pdata);
  mat_cv = mat.clone();

  return true;
}

bool SaveImageTxt(const EigenMat &img, const std::string &str_file) {

  Mat img_cv;
  EigenMat2CvMat(img, img_cv);
  SaveImageTxt(img_cv, str_file);
  return true;
}

// save image to disk
bool SaveImage(const EigenMat &img, const std::string &str_file) {

  Mat img_cv;
  EigenMat2CvMat(img, img_cv);
  SaveImage(img_cv, str_file);
  return true;
}

bool SaveImage(const EigenMat &img1, const EigenMat &img2,
               const std::string &str_file, const int window_height,
               const int window_width) {

  Mat img_cv1, img_cv2;
  EigenMat2CvMat(img1, img_cv1);
  EigenMat2CvMat(img2, img_cv2);
  SaveImage(img_cv1, img_cv2, str_file, window_height, window_width);
  return true;
}

bool SaveImage(const EigenMat &img1, const EigenMat &img2, const EigenMat &img3,
               const std::string &str_file, const int window_height,
               const int window_width) {

  Mat img_cv1, img_cv2, img_cv3;
  EigenMat2CvMat(img1, img_cv1);
  EigenMat2CvMat(img2, img_cv2);
  EigenMat2CvMat(img3, img_cv3);
  SaveImage(img_cv1, img_cv2, img_cv3, str_file, window_height, window_width);

  return true;
}

bool SaveImage(const EigenMat &img1, const EigenMat &img2, const EigenMat &img3,
               const EigenMat &img4, const std::string &str_file,
               const int window_height, const int window_width) {

  Mat img_cv1, img_cv2, img_cv3, img_cv4;
  EigenMat2CvMat(img1, img_cv1);
  EigenMat2CvMat(img2, img_cv2);
  EigenMat2CvMat(img3, img_cv3);
  EigenMat2CvMat(img3, img_cv4);
  SaveImage(img_cv1, img_cv2, img_cv3, img_cv4, str_file, window_height,
            window_width);

  return true;
}

void ShowMat(const EigenMat &mat, const std::string &str) {
  std::cout << str << std::endl << mat << std::endl;
}

void ShowMatDebug(const EigenMat &mat) {
  std::cout << "mat" << std::endl << mat << std::endl;
}

void ShowMatDebug(const Eigen::Map<EigenMat> &mat) {
  std::cout << "map-mat" << std::endl << mat << std::endl;
}

void SaveMatTxt(const EigenMat &mat, const std::string &str) {
  Mat mat_cv;
  EigenMat2CvMat(mat, mat_cv);
  SaveImageTxt(mat_cv, str);
}

// save image to disk
bool ShowImage(const EigenMat &img, const std::string &str_window,
               const int delay_ms, const int window_height,
               const int window_width) {

  Mat img_cv;
  EigenMat2CvMat(img, img_cv);
  ShowImage(img_cv, str_window, delay_ms, window_height, window_width);
  return true;
}

bool ShowImage(const EigenMat &img1, const EigenMat &img2,
               const std::string &str_window, const int delay_ms,
               const int window_height, const int window_width) {

  Mat img_cv1, img_cv2;
  EigenMat2CvMat(img1, img_cv1);
  EigenMat2CvMat(img2, img_cv2);
  ShowImage(img_cv1, img_cv2, str_window, delay_ms, window_height,
            window_width);
  return true;
}

bool ShowImage(const EigenMat &img1, const EigenMat &img2, const EigenMat &img3,
               const std::string &str_window, const int delay_ms,
               const int window_height, const int window_width) {

  Mat img_cv1, img_cv2, img_cv3;
  EigenMat2CvMat(img1, img_cv1);
  EigenMat2CvMat(img2, img_cv2);
  EigenMat2CvMat(img3, img_cv3);
  ShowImage(img_cv1, img_cv2, img_cv3, str_window, delay_ms, window_height,
            window_width);

  return true;
}

bool ShowImage(const EigenMat &img1, const EigenMat &img2, const EigenMat &img3,
               const EigenMat &img4, const std::string &str_window,
               const int delay_ms, const int window_height,
               const int window_width) {

  Mat img_cv1, img_cv2, img_cv3, img_cv4;
  EigenMat2CvMat(img1, img_cv1);
  EigenMat2CvMat(img2, img_cv2);
  EigenMat2CvMat(img3, img_cv3);
  EigenMat2CvMat(img4, img_cv4);
  ShowImage(img_cv1, img_cv2, img_cv3, img_cv4, str_window, delay_ms,
            window_height, window_width);
  return true;
}

bool Convert8U3C(const EigenMat &img, Mat &img_8u3c, const int window_height,
                 const int window_width) {

  Mat img_cv;
  EigenMat2CvMat(img, img_cv);
  return Convert8U3C(img_cv, img_8u3c, window_height, window_width);
}
