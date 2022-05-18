#pragma once

#include <Eigen/Dense>

#include "utils/cv_utils.h"

//#include "../src/LaneDetEigen.h"

// try to use cv_utils as much as poosible

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    EigenMat;

bool CvMat2EigenMat(const Mat &mat_cv, EigenMat &mat_eigen);
bool EigenMat2CvMat(const EigenMat &mat_eigen, Mat &mat_cv);

bool SaveImageTxt(const EigenMat &img, const std::string &str_file);

// save image to disk
// for float image, simply convert to [0, 255] by mapping [val_min->0,
// val_max->255]
bool SaveImage(const EigenMat &img, const std::string &str_file);
bool SaveImage(const EigenMat &img1, const EigenMat &img2,
               const std::string &str_file, const int window_height = 480,
               const int window_width = 640);
bool SaveImage(const EigenMat &img1, const EigenMat &img2, const Mat &img3,
               const std::string &str_file, const int window_height = 480,
               const int window_width = 640);
bool SaveImage(const EigenMat &img1, const EigenMat &img2, const EigenMat &img3,
               const EigenMat &img4, const std::string &str_file,
               const int window_height = 480, const int window_width = 640);

void ShowMat(const EigenMat &mat, const std::string &str);
void ShowMatDebug(const EigenMat &mat);
void ShowMatDebug(const Eigen::Map<EigenMat> &mat);
void SaveMatTxt(const EigenMat &mat,
                const std::string &str); // the same as SaveImageTxt

// in order to display all the types of image
// convert image to 8UC3
// dispaly the image
bool ShowImage(const EigenMat &img, const std::string &str_window,
               const int delay_ms = 10, const int window_height = 480,
               const int window_width = 640);

// dispaly the image
bool ShowImage(const EigenMat &img1, const EigenMat &img2,
               const std::string &str_window, const int delay_ms = 10,
               const int window_height = 480, const int window_width = 640);

// dispaly the image
bool ShowImage(const EigenMat &img1, const EigenMat &img2, const EigenMat &img3,
               const std::string &str_window, const int delay_ms = 10,
               const int window_height = 480, const int window_width = 640);

// dispaly the image
bool ShowImage(const EigenMat &img1, const EigenMat &img2, const EigenMat &img3,
               const EigenMat &img4, const std::string &str_window,
               const int delay_ms = 10, const int window_height = 480,
               const int window_width = 640);

// convert {8u,32f} c{1,3} to 8uc3 image
bool Convert8U3C(const EigenMat &img, Mat &img_8u3c,
                 const int window_height = -1, const int window_width = -1);
