#include "avp_mapping_interface.h"
#include "colordef.h"
#include "viewerconfig.h"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cout << kColorRed << " args error " << std::endl;
    std::cout << " args : config_file bmp_folder start_index  " << kColorReset
              << std::endl;
    return 1;
  }

  std::string config_file = std::string(argv[1]);
  std::string bmp_folder = std::string(argv[2]);
  int start_index = atoi(argv[3]);

  auto avp_mapper = SemanticSLAM::CreateAvpMapper(config_file);
  auto remaps = avp_mapper->GetPanoramicRemap();

  for (int i = start_index;; i++) {
    cv::Mat img_left =
        cv::imread(bmp_folder + "/frame_vc9_" + std::to_string(i) + ".bmp",
                   cv::IMREAD_GRAYSCALE);
    cv::Mat img_front =
        cv::imread(bmp_folder + "/frame_vc10_" + std::to_string(i) + ".bmp",
                   cv::IMREAD_GRAYSCALE);
    cv::Mat img_right =
        cv::imread(bmp_folder + "/frame_vc11_" + std::to_string(i) + ".bmp",
                   cv::IMREAD_GRAYSCALE);
    cv::Mat img_back =
        cv::imread(bmp_folder + "/frame_vc12_" + std::to_string(i) + ".bmp",
                   cv::IMREAD_GRAYSCALE);
    if (img_left.empty() || img_front.empty() || img_right.empty() ||
        img_back.empty()) {
      std::cout << " image " << i << " is not found " << std::endl;
      break;
    }

    cv::Mat img_pano_left, img_pano_front, img_pano_right, img_pano_back;
    cv::remap(img_left, img_pano_left, remaps.at(0).first, remaps.at(0).second,
              CV_INTER_LINEAR);
    cv::remap(img_front, img_pano_front, remaps.at(1).first,
              remaps.at(1).second, CV_INTER_LINEAR);
    cv::remap(img_right, img_pano_right, remaps.at(2).first,
              remaps.at(2).second, CV_INTER_LINEAR);
    cv::remap(img_back, img_pano_back, remaps.at(3).first, remaps.at(3).second,
              CV_INTER_LINEAR);

#ifdef ENABLE_VIEWER
    cv::imshow("left", img_pano_left);
    cv::imshow("front", img_pano_front);
    cv::imshow("right", img_pano_right);
    cv::imshow("back", img_pano_back);
    cv::waitKey(0);
#endif
  }

  return 1;
}
