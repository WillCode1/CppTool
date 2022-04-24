#include "basictool.h"
#include "colordef.h"
#include "converter.h"
#include "map.h"
#include "mapviewer/mapviewer.h"
#include "multicam_ext.h"
#include "multicamtracking.h"
#include "system.h"
#include <algorithm>
#include <chrono>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <termios.h>
#include <thread>
#include <unistd.h>

const unsigned char KEYCODE_ESC = 0x1B;
const unsigned char KEYCODE_LEFT = 0x61;
const unsigned char KEYCODE_FRONT = 0x77;
const unsigned char KEYCODE_RIGHT = 0x64;
const unsigned char KEYCODE_BACK = 0x73;
const unsigned char KEYCODE_SPACE = 0x20;

using namespace std;
using namespace FeatureSLAM;

int kbhit(void) {
  struct termios oldt, newt;
  int ch;
  int oldf;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

const int kImageRow0 = 200;
const int kImageRow1 = 570;

int main(int argc, char **argv) {
  if (argc != 5) {
    std::cout << kColorRed << " Args error " << std::endl;
    std::cerr << " Usage :  config.yaml map.bin  path_to_sequence start_index"
              << kColorReset << std::endl;
    return 1;
  }

  const std::string str_config = std::string(argv[1]);
  const std::string map_file = std::string(argv[2]);
  const std::string image_path = std::string(argv[3]);
  const unsigned int kStartIndex = static_cast<unsigned int>(atoi(argv[4]));

  // init settings

  const size_t bar_index = str_config.find_last_of('/');
  std::string config_path = str_config.substr(0, bar_index + 1);
  cv::FileStorage fsettings(str_config, cv::FileStorage::READ);
  string front_mask_path = fsettings["MaskFront"];
  cv::Mat front2wheel = fsettings["toWheelCoordinate"].mat();
  std::vector<cv::Mat> cam_extrinsic = BasicTool::LoadExtrinsicParam(fsettings);
  std::string str_voc = config_path + fsettings["ORBVoc"].string();

  fsettings.release();
  MultiCamExt::GetInstance().Initialize(cam_extrinsic, front2wheel);

  // load voc

  ORBVocabulary voc;
  voc.loadFromBinaryFile(str_voc);

  // load map
  Map *map = new Map();
  map->Load(map_file, voc, front_mask_path, cam_extrinsic);
  KeyFrameDatabase *keyfrm_database = new KeyFrameDatabase(voc);

  for (auto keyfrm : map->GetAllKeyFrames()) {
    keyfrm_database->add(keyfrm);
  }

  // setup viewer
  MapViewer *viewer = new MapViewer(map, cam_extrinsic);
  std::thread *viewer_thread = new thread(&MapViewer::Run, viewer);
  viewer_thread->detach();

  // prepare data
  std::vector<std::vector<std::string>> image_lists;
  std::vector<double> timestamps;
  std::string data_path = std::string(image_path);

  BasicTool::LoadMultiCamImages(image_lists, timestamps);
  const unsigned long kImageNum = image_lists[0].size();

  bool use_crop_image = true;
  bool bockleftcam = false, blockfrontcam = false, blockrightcam = false,
       blockbackcam = false;

  MultiCamTracking *multi_cam_tracker =
      new MultiCamTracking(&voc, map, keyfrm_database, str_config, viewer);

  for (size_t i = kStartIndex; i < kImageNum; i++) {
    cv::Mat imgleft =
        cv::imread(data_path + image_lists[0][i], cv::IMREAD_GRAYSCALE);
    cv::Mat imgfront =
        cv::imread(data_path + image_lists[1][i], cv::IMREAD_GRAYSCALE);
    cv::Mat imgright =
        cv::imread(data_path + image_lists[2][i], cv::IMREAD_GRAYSCALE);
    cv::Mat imgback =
        cv::imread(data_path + image_lists[3][i], cv::IMREAD_GRAYSCALE);

    if (imgleft.empty() || imgfront.empty() || imgright.empty() ||
        imgback.empty())
      continue;

    if (!use_crop_image) {
      cv::resize(imgleft, imgleft, cv::Size(kImgWidth, kImgWidth));
      cv::resize(imgfront, imgfront, cv::Size(kImgWidth, kImgWidth));
      cv::resize(imgright, imgright, cv::Size(kImgWidth, kImgWidth));
      cv::resize(imgback, imgback, cv::Size(kImgWidth, kImgWidth));
    }

    if (bockleftcam)
      imgleft = cv::Mat::zeros(imgleft.rows, imgleft.cols, imgleft.type());
    if (blockfrontcam)
      imgfront = cv::Mat::zeros(imgfront.rows, imgfront.cols, imgfront.type());
    if (blockrightcam)
      imgright = cv::Mat::zeros(imgright.rows, imgright.cols, imgright.type());
    if (blockbackcam)
      imgback = cv::Mat::zeros(imgback.rows, imgback.cols, imgback.type());

    if (!use_crop_image) {
      imgleft = imgleft.rowRange(kImageRow0, kImageRow1);
      imgfront = imgfront.rowRange(kImageRow0, kImageRow1);
      imgright = imgright.rowRange(kImageRow0, kImageRow1);
      imgback = imgback.rowRange(kImageRow0, kImageRow1);
    }

    Timer timer_total("Total time cost");

    cv::Mat pose = multi_cam_tracker->GrabImageMultiCam(
        imgleft, imgfront, imgright, imgback, timestamps[i]);

    std::cout << kColorRed;
    timer_total.Print();
    std::cout << kColorReset;

    if (kbhit()) {
      int ch = getchar();
      if (ch == KEYCODE_LEFT) {
        bockleftcam = !bockleftcam;
      } else if (ch == KEYCODE_FRONT) {
        blockfrontcam = !blockfrontcam;
      } else if (ch == KEYCODE_RIGHT) {
        blockrightcam = !blockrightcam;
      } else if (ch == KEYCODE_BACK) {
        blockbackcam = !blockbackcam;
      } else if (ch == KEYCODE_ESC)
        break;
    }
  }

  // shutdown the viewer
  viewer->RequestFinish();
  while (!viewer->IsFinish()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 1;
}
