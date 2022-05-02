#pragma once

#include <opencv2/opencv.hpp>

namespace FeatureSLAM {
class BasicTool {
public:
  static std::vector<cv::Mat>
  LoadExtrinsicParam(const cv::FileStorage &fSettings);
  static void LoadMultiCamImages(std::vector<std::vector<std::string>> &image_name_lists, std::vector<double> &timestamps);
};
}
