#pragma once
#include <opencv2/opencv.hpp>

#define FLT_TO_FIX(x, n) cvRound((x) * (1 << (n)))

namespace SemanticSLAM {

const int kInitDist0 = (INT_MAX >> 2);
const int kDistShift = 16;

class AvpDistanceTransformer {
public:
  AvpDistanceTransformer() = delete;
  AvpDistanceTransformer(int height, int width)
      : pixel_thresh_(50), image_height_(height), image_width_(width) {
    int boarder = 1;
    temp_ = cv::Mat::zeros(image_height_ + 2 * boarder,
                           image_width_ + 2 * boarder, CV_32SC1);
    temp_inverse_ = cv::Mat::zeros(image_height_ + 2 * boarder,
                                   image_width_ + 2 * boarder, CV_32SC1);
  }

  void DistranceTransform(uchar *input, unsigned char *output) {
    int boarder = 1;
    initTopBottom(temp_, boarder);
    initTopBottom(temp_inverse_, boarder);
    //    cv::Mat image(image_height_, image_width_, CV_8UC1, input);
    //    cv::Mat dst(image_height_, image_width_, CV_8UC1, output);
    DistanceTransform3x3(input, temp_, temp_inverse_, output, metrics_);
  }

  void DistanceTransform3x3(const uchar *src, cv::Mat &_temp,
                            cv::Mat &_temp_inverse, uchar *dst,
                            const float *metrics) {
    const int kBoarder = 1;
    int i, j;
    const int kHVDistCost = FLT_TO_FIX(metrics[0], kDistShift);
    const int kDiagDistCost = FLT_TO_FIX(metrics[1], kDistShift);
    const float kScale = 1.f / (1 << kDistShift);

    CV_Assert(_temp.rows == _temp_inverse.rows);
    CV_Assert(_temp.cols == _temp_inverse.cols);

    int *temp = _temp.ptr<int>();
    int *temp_inverse = _temp_inverse.ptr<int>();
    uchar *dist = dst;
    int srcstep = image_width_;
    int step = (int)(_temp.step / sizeof(temp[0]));
    int dststep = image_width_;
    cv::Size size(image_width_, image_height_);
    // forward pass
    for (i = 0; i < size.height; i++) {
      const uchar *s = src + i * srcstep;
      int *tmp = (int *)(temp + (i + kBoarder) * step) + kBoarder;
      int *tmp_inverse =
          (int *)(temp_inverse + (i + kBoarder) * step) + kBoarder;

      for (j = 0; j < kBoarder; j++) {
        tmp[-j - 1] = tmp[size.width + j] = kInitDist0;
        tmp_inverse[-j - 1] = tmp_inverse[size.width + j] = kInitDist0;
      }

      for (j = 0; j < size.width; j++) {
        if (s[j] < pixel_thresh_) {
          tmp[j] = 0;

          int t0 = tmp_inverse[j - step - 1] + kDiagDistCost;
          int t = tmp_inverse[j - step] + kHVDistCost;
          if (t0 > t)
            t0 = t;
          //          t = tmp_inverse[j - step + 1] + kDiagDistCost;
          //          if (t0 > t)
          //            t0 = t;
          t = tmp_inverse[j - 1] + kHVDistCost;
          if (t0 > t)
            t0 = t;
          tmp_inverse[j] = t0;

        } else {

          tmp_inverse[j] = 0;
          int t0 = tmp[j - step - 1] + kDiagDistCost;
          int t = tmp[j - step] + kHVDistCost;
          if (t0 > t)
            t0 = t;
          //          t = tmp[j - step + 1] + kDiagDistCost;
          //          if (t0 > t)
          //            t0 = t;
          t = tmp[j - 1] + kHVDistCost;
          if (t0 > t)
            t0 = t;
          tmp[j] = t0;
        }
      }
    }

    // backward pass
    for (i = size.height - 1; i >= 0; i--) {
      unsigned char *d = (unsigned char *)(dist + i * dststep);
      int *tmp = (int *)(temp + (i + kBoarder) * step) + kBoarder;
      int *tmp_inverse =
          (int *)(temp_inverse + (i + kBoarder) * step) + kBoarder;

      for (j = size.width - 1; j >= 0; j--) {

        // for tmp;
        int t0 = tmp[j];
        if (t0 > kHVDistCost) {
          int t = tmp[j + step + 1] + kDiagDistCost;
          if (t0 > t)
            t0 = t;
          t = tmp[j + step] + kHVDistCost;
          if (t0 > t)
            t0 = t;
          //          t = tmp[j + step - 1] + kDiagDistCost;
          //          if (t0 > t)
          //            t0 = t;
          t = tmp[j + 1] + kHVDistCost;
          if (t0 > t)
            t0 = t;
          tmp[j] = t0;
        }
        //      d[j] = (float)(t0 * kScale);

        // for tmp_inverse ;
        int t1 = tmp_inverse[j];
        if (t1 > kHVDistCost) {
          int t = tmp_inverse[j + step + 1] + kDiagDistCost;
          if (t1 > t)
            t1 = t;
          t = tmp_inverse[j + step] + kHVDistCost;
          if (t1 > t)
            t1 = t;
          //          t = tmp_inverse[j + step - 1] + kDiagDistCost;
          //          if (t1 > t)
          //            t1 = t;
          t = tmp_inverse[j + 1] + kHVDistCost;
          if (t1 > t)
            t1 = t;
          tmp_inverse[j] = t1;
        }
        float val = (float)((t0 - t1) * kScale) + 40;
        d[j] = val < 0 ? 0 : cv::saturate_cast<uchar>(val * 5.1f);
      }
    }
  }

private:
  void initTopBottom(cv::Mat &temp, int border) {
    cv::Size size = temp.size();
    for (int i = 0; i < border; i++) {
      int *ttop = temp.ptr<int>(i);
      int *tbottom = temp.ptr<int>(size.height - i - 1);

      for (int j = 0; j < size.width; j++) {
        ttop[j] = kInitDist0;
        tbottom[j] = kInitDist0;
      }
    }
  }

private:
  unsigned char pixel_thresh_;
  int image_height_;
  int image_width_;
  cv::Mat temp_;
  cv::Mat temp_inverse_;
  float metrics_[2] = {0.955f, 1.3693f};
};
}
