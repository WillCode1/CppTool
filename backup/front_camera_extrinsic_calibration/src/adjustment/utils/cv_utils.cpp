#include "cv_utils.h"
#include <fstream>

bool LoadImage(const std::string &str_img, Mat &img, const bool is_color) {
  int cv_read_flag = (is_color ? CV_LOAD_IMAGE_COLOR : CV_LOAD_IMAGE_GRAYSCALE);

  // load gray image or color image
  img = imread(str_img, cv_read_flag);

  if (!img.data) {
    std::cout << "No image data" << std::endl;
    return false;
  }

  return true;
}

// load image as color; split to single channel
// idx_channel: -1: enforce load to gray; 0,1,2: bgr channel
bool LoadImage(const std::string &str_img, Mat &img_gray, Mat &img_color,
               const int idx_channel) {
  // force to load as color image
  int cv_read_flag = CV_LOAD_IMAGE_COLOR;
  img_color = imread(str_img, cv_read_flag);
  if (!img_color.data) {
    std::cout << "No image data" << std::endl;
    return false;
  }

  // load gray-scale image
  // enforce load to gray
  if (idx_channel == -1) {
    cv_read_flag = CV_LOAD_IMAGE_GRAYSCALE;
    img_gray = imread(str_img, cv_read_flag);
    if (!img_gray.data) {
      std::cout << "No image data" << std::endl;
      return false;
    }
  } else if (idx_channel >= 0 && idx_channel <= 2) {
    Mat img_bgr[3];            // destination array
    split(img_color, img_bgr); // split source
    img_gray = img_bgr[idx_channel];
  } else {
    std::cout << "invalide channel idx" << std::endl;
    return false;
  }

  return true;
}

bool SaveImageTxt(const Mat &img, const std::string &str_file) {
  if (img.empty()) {
    std::cout << "No image data" << std::endl;
    return false;
  }

  // output image to txt file
  std::ofstream strm(str_file);

  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {

      if (img.type() == CV_32F) {
        strm << img.at<float>(i, j) << " ";
      }
      if (img.type() == CV_8U) {
        strm << (int)img.at<uchar>(i, j) << " ";
      }
      if (img.type() == CV_32S) {
        strm << img.at<int>(i, j) << " ";
      }
      if (img.type() == CV_64F) {
        strm << img.at<double>(i, j) << " ";
      }
    }
    strm << std::endl;
  }

  return true;
}

// save image to disk
bool SaveImage(const Mat &img, const std::string &str_file) {
  if (img.empty()) {
    std::cout << "No image data" << std::endl;
    return false;
  }

  if (img.channels() == 1) {

    if (img.type() == CV_8U) {
      imwrite(str_file, img);
    } else if (img.type() == CV_32F) {

      /*
      Mat image = Mat(img.size(), CV_8U);

      double val_min, val_max;
      minMaxLoc(img, &val_min, &val_max);
      double scale = 255.0 / val_max;

      img.convertTo(image, CV_8U, scale, -1*val_min);
      */

      Mat image;
      normalize(img, image, 0, 255, NORM_MINMAX, CV_8U);
      imwrite(str_file, image);
    }

  } else if (img.channels() == 3) {
    if (img.type() == CV_8UC3) {
      imwrite(str_file, img);
    } else {
      std::cout << "does not support current type" << std::endl;
      return false;
    }
  } else {
    std::cout << "does not support current type" << std::endl;
    return false;
  }

  return true;
}

bool SaveImage(const Mat &img1, const Mat &img2, const std::string &str_file,
               const int window_height, const int window_width) {
  Mat img1_8uc3;
  Mat img2_8uc3;

  Convert8U3C(img1, img1_8uc3, window_height, window_width);
  Convert8U3C(img2, img2_8uc3, window_height, window_width);

  Mat img_show;
  hconcat(img1_8uc3, img2_8uc3, img_show);
  SaveImage(img_show, str_file);
  return true;
}

bool SaveImage(const Mat &img1, const Mat &img2, const Mat &img3,
               const std::string &str_file, const int window_height,
               const int window_width) {
  Mat img1_8uc3;
  Mat img2_8uc3;
  Mat img3_8uc3;

  Convert8U3C(img1, img1_8uc3, window_height, window_width);
  Convert8U3C(img2, img2_8uc3, window_height, window_width);
  Convert8U3C(img3, img3_8uc3, window_height, window_width);

  Mat img_show_tmp, img_show;
  hconcat(img1_8uc3, img2_8uc3, img_show_tmp);
  hconcat(img_show_tmp, img3_8uc3, img_show);
  SaveImage(img_show, str_file);
  return true;
}

bool SaveImage(const Mat &img1, const Mat &img2, const Mat &img3,
               const Mat &img4, const std::string &str_file,
               const int window_height, const int window_width) {
  Mat img1_8uc3;
  Mat img2_8uc3;
  Mat img3_8uc3;
  Mat img4_8uc3;

  Convert8U3C(img1, img1_8uc3, window_height, window_width);
  Convert8U3C(img2, img2_8uc3, window_height, window_width);
  Convert8U3C(img3, img3_8uc3, window_height, window_width);
  Convert8U3C(img4, img4_8uc3, window_height, window_width);

  Mat img_show_h1, img_show_h2, img_show;
  hconcat(img1_8uc3, img2_8uc3, img_show_h1);
  hconcat(img3_8uc3, img4_8uc3, img_show_h2);
  vconcat(img_show_h1, img_show_h2, img_show);
  SaveImage(img_show, str_file);

  return true;
}

void ShowMat(const Mat &mat, const std::string &str) {
  std::cout << str << std::endl << mat << std::endl;
}

void SaveMatTxt(const Mat &mat, const std::string &str) {
  SaveImageTxt(mat, str);
}

void ShowPts(const Point &pts, const std::string &str) {
  std::cout << str << std::endl << pts << std::endl;
}

void ShowPts(const Point2f &pts, const std::string &str) {
  std::cout << str << std::endl << pts << std::endl;
}

// save image to disk
bool ShowImage(const Mat &img, const std::string &str_window,
               const int delay_ms, const int window_height,
               const int window_width) {
  if (img.empty()) {
    std::cout << "No image data" << std::endl;
    return false;
  }

  // choose WINDOW_NORMAL other than WINDOW_AUTOSIZE to set window size
  namedWindow(str_window, WINDOW_NORMAL);
  resizeWindow(str_window, window_width, window_height);

  if (img.channels() == 1) {

    if (img.type() == CV_8U) {
      imshow(str_window, img);
    } else if (img.type() == CV_32F) {

      /*
      Mat image = Mat(img.size(), CV_8U);

      double val_min, val_max;
      minMaxLoc(img, &val_min, &val_max);
      double scale = 255.0 / val_max;

      img.convertTo(image, CV_8U, scale, -1*val_min);
      */

      Mat image;
      normalize(img, image, 0, 255, NORM_MINMAX, CV_8U);
      imshow(str_window, image);
    }

  } else if (img.channels() == 3) {
    if (img.type() == CV_8UC3) {
      imshow(str_window, img);
    } else {
      std::cout << "does not support current type" << std::endl;
      return false;
    }
  } else {
    std::cout << "does not support current type" << std::endl;
    return false;
  }

  waitKey(delay_ms);
  return true;
}

bool ShowImage(const Mat &img1, const Mat &img2, const std::string &str_window,
               const int delay_ms, const int window_height,
               const int window_width) {
  Mat img1_8uc3;
  Mat img2_8uc3;

  Convert8U3C(img1, img1_8uc3, window_height, window_width);
  Convert8U3C(img2, img2_8uc3, window_height, window_width);

  Mat img_show;
  hconcat(img1_8uc3, img2_8uc3, img_show);
  ShowImage(img_show, str_window, delay_ms, window_height, window_width * 2);
  return true;
}

bool ShowImage(const Mat &img1, const Mat &img2, const Mat &img3,
               const std::string &str_window, const int delay_ms,
               const int window_height, const int window_width) {
  Mat img1_8uc3;
  Mat img2_8uc3;
  Mat img3_8uc3;

  Convert8U3C(img1, img1_8uc3, window_height, window_width);
  Convert8U3C(img2, img2_8uc3, window_height, window_width);
  Convert8U3C(img3, img3_8uc3, window_height, window_width);

  Mat img_show_tmp, img_show;
  hconcat(img1_8uc3, img2_8uc3, img_show_tmp);
  hconcat(img_show_tmp, img3_8uc3, img_show);
  ShowImage(img_show, str_window, delay_ms, window_height, window_width * 3);
  return true;
}

bool ShowImage(const Mat &img1, const Mat &img2, const Mat &img3,
               const Mat &img4, const std::string &str_window,
               const int delay_ms, const int window_height,
               const int window_width) {
  Mat img1_8uc3;
  Mat img2_8uc3;
  Mat img3_8uc3;
  Mat img4_8uc3;

  Convert8U3C(img1, img1_8uc3, window_height, window_width);
  Convert8U3C(img2, img2_8uc3, window_height, window_width);
  Convert8U3C(img3, img3_8uc3, window_height, window_width);
  Convert8U3C(img4, img4_8uc3, window_height, window_width);

  Mat img_show_h1, img_show_h2, img_show;
  hconcat(img1_8uc3, img2_8uc3, img_show_h1);
  hconcat(img3_8uc3, img4_8uc3, img_show_h2);
  vconcat(img_show_h1, img_show_h2, img_show);
  ShowImage(img_show, str_window, delay_ms, window_height * 2,
            window_width * 2);

  return true;
}

bool Convert8U3C(const Mat &img, Mat &img_8u3c, const int window_height,
                 const int window_width) {
  if (img.empty()) {
    std::cout << "No image data" << std::endl;
    return false;
  }

  Mat image;
  if (img.channels() == 1) {

    if (img.type() == CV_8U) {
      image = img;
    } else if (img.type() == CV_32F) {

      /*
      Mat image = Mat(img.size(), CV_8U);

      double val_min, val_max;
      minMaxLoc(img, &val_min, &val_max);
      double scale = 255.0 / val_max;

      img.convertTo(image, CV_8U, scale, -1*val_min);
      */
      normalize(img, image, 0, 255, NORM_MINMAX, CV_8U);
    }

  } else if (img.channels() == 3) {
    if (img.type() == CV_8UC3) {
      Mat img_8u3c_tmp = img.clone();

      if (window_height > 0 && window_width > 0) {
        resize(img_8u3c_tmp, img_8u3c, Size(window_width, window_height));
      }
      return true;

    } else {
      std::cout << "does not support current type" << std::endl;
      return false;
    }
  } else {
    std::cout << "does not support current type" << std::endl;
    return false;
  }

  // image.convertTo(img_8u3c, CV_8UC3);
  Mat img_8u3c_tmp;
  cvtColor(image, img_8u3c_tmp, COLOR_GRAY2BGR);
  if (window_height > 0 && window_width > 0) {
    resize(img_8u3c_tmp, img_8u3c, Size(window_width, window_height));
  } else {
    img_8u3c = img_8u3c_tmp.clone();
  }

  return true;
}

/*
float GetQuantile(const Mat& mat, const float qtile) {
  //make it a row vector
  //CvMat rowMat;
  //cvReshape(mat, &rowMat, 0, 1);

  //get the quantile
  float qval;
  const float * data = mat.ptr<float>(0);
  qval = quantile(data, mat.rows * mat.cols, qtile);

  //std::cout<<"qval = "<<qval<<std::endl;

#if 0
    // convert to vector and verify again
    int num = mat.rows * mat.cols;
    std::vector<float> vdata;
    for (int i=0; i<mat.rows; i++) {
        for (int j=0; j<mat.cols; j++) {
            vdata.push_back(mat.at<float>(i,j));
        }
    }
    float qval_check;
    //const float * data = mat.ptr<float>(0);
    qval_check = quantile(vdata, num, qtile);

    std::cout<<"qval_check = "<<qval_check<<std::endl;
#endif

  return qval;
}
*/