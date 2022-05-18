#pragma once

#include <iostream>
#include <string>

//#include <cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// load image
bool LoadImage(const std::string &str_img, Mat &img, const bool is_color);

// load image as color; split to single channel
// idx_channel: -1: enforce load to gray; 0,1,2: bgr channel
bool LoadImage(const std::string &str_img, Mat &img, Mat &img_color,
               const int idx_channel = -1);

bool SaveImageTxt(const Mat &img, const std::string &str_file);

// save image to disk
// for float image, simply convert to [0, 255] by mapping [val_min->0,
// val_max->255]
bool SaveImage(const Mat &img, const std::string &str_file);
bool SaveImage(const Mat &img1, const Mat &img2, const std::string &str_file,
               const int window_height = 480, const int window_width = 640);
bool SaveImage(const Mat &img1, const Mat &img2, const Mat &img3,
               const std::string &str_file, const int window_height = 480,
               const int window_width = 640);
bool SaveImage(const Mat &img1, const Mat &img2, const Mat &img3,
               const Mat &img4, const std::string &str_file,
               const int window_height = 480, const int window_width = 640);

void ShowMat(const Mat &mat, const std::string &str);
void SaveMatTxt(const Mat &mat,
                const std::string &str); // the same as SaveImageTxt

// void ShowPts(const Point &pts, const std::string &str);
// void ShowPts(const Point2f &pts, const std::string &str);

// in order to display all the types of image
// convert image to 8UC3
// dispaly the image
bool ShowImage(const Mat &img, const std::string &str_window,
               const int delay_ms = 10, const int window_height = 480,
               const int window_width = 640);

// dispaly the image
bool ShowImage(const Mat &img1, const Mat &img2, const std::string &str_window,
               const int delay_ms = 10, const int window_height = 480,
               const int window_width = 640);

// dispaly the image
bool ShowImage(const Mat &img1, const Mat &img2, const Mat &img3,
               const std::string &str_window, const int delay_ms = 10,
               const int window_height = 480, const int window_width = 640);

// dispaly the image
bool ShowImage(const Mat &img1, const Mat &img2, const Mat &img3,
               const Mat &img4, const std::string &str_window,
               const int delay_ms = 10, const int window_height = 480,
               const int window_width = 640);

// convert {8u,32f} c{1,3} to 8uc3 image
bool Convert8U3C(const Mat &img, Mat &img_8u3c, const int window_height = -1,
                 const int window_width = -1);

#if 0

float GetQuantile(const Mat& mat, const float qtile);

// Return y difference of two rects: ex: difference of rect_top_y
// rect: (top_x, top_y, rect_width, rect_height)
// rect_prev/curr: rect in previous/current frame
// image_prev/curr: image of previous/current frame; only used for visulization
int ComputeRectYDiff(const Mat& image_prev, const Mat& image_curr, const Rect& rect_prev, const Rect& rect_curr);


template <class T>
inline T quantile(const T* d, const int size, const double q) {
  if (size == 0) return T(0);
  if (size == 1) return d[0];
  if (q <= 0) return *std::min_element(d, d + size);
  if (q >= 1) return *std::max_element(d, d + size);

  double pos = (size - 1) * q;
  int ind = int(pos);
  double delta = pos - ind;
  std::vector<T> w(size); std::copy(d, d + size, w.begin());
  std::nth_element(w.begin(), w.begin() + ind, w.end());
  T i1 = *(w.begin() + ind);
  T i2 = *std::min_element(w.begin() + ind + 1, w.end());
  return (T) (i1 * (1.0 - delta) + i2 * delta);
}

template <class T>
inline T quantile(const std::vector<T>& d, const int size, const double q) {
    quantile(&d[0], size, q);
}

#endif

void Test();
