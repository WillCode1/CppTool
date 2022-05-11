#include "avp_mapping_interface.h"
#include "colordef.h"
#include "dataloader.h"
#include "timer.h"
#include "vslam_types.h"
#include <algorithm>
#include <chrono>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

#define KEYCODE_ESC 0x1B

using namespace FeatureSLAM;
using namespace SemanticSLAM;
using namespace std;

int kbhit(void)
{
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
  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

const int kImageRow0 = 200;
const int kImageRow1 = 570;

int main(int argc, char **argv)
{

  if (argc != 2 && argc != 3)
  {
    std::cerr << " args error " << std::endl;
    std::cout << " usage  ./mc_slam config_file  [aligned_odom ] " << std::endl;
    return 1;
  }

  // loading config
  std::string str_config = std::string(argv[1]);

  // const size_t bar_index = str_config.find_last_of('/');
  // std::string config_path = str_config.substr(0, bar_index + 1);

  cv::FileStorage fs(str_config, cv::FileStorage::READ);

  std::string data_path = fs["data_path"].string();
  std::string image_timestamp = fs["image_timestamp"].string();
  std::string odometry_file = fs["wheel_odom"].string();
  int start_index = int(fs["start_index"].real());

  DataLoader::Mode mode(DataLoader::NOT_ALIGNED);

  if (argc == 3)
  {
    odometry_file = std::string(argv[2]);
    mode = DataLoader::ALIGNED;
  }

  DataLoader dataloader(mode);
  dataloader.SetOdometryFile(odometry_file);
  dataloader.SetImageTimeStampefile(image_timestamp);

  if (mode == DataLoader::ALIGNED)
    dataloader.SkipImageData(start_index);

  bool use_crop_image = false;
  int crop_image = int(fs["image_crop"].real());
  if (crop_image != 0)
    use_crop_image = true;

  auto feature_mapper = CreateFeatureMapper(str_config);
  Timer timer("Total Mapping");

  while (true)
  {
    DataFrame frame_data;
    OdometryData odom_data;

    if (mode == DataLoader::ALIGNED)
    {

      bool ret = dataloader.NextFrame(frame_data, odom_data);

      if (!ret)
        break;

      cv::Mat imgleft = cv::imread(data_path + frame_data.str_image_left, cv::IMREAD_GRAYSCALE);
      cv::Mat imgfront = cv::imread(data_path + frame_data.str_image_front, cv::IMREAD_GRAYSCALE);
      cv::Mat imgright = cv::imread(data_path + frame_data.str_image_right, cv::IMREAD_GRAYSCALE);
      cv::Mat imgback = cv::imread(data_path + frame_data.str_image_back, cv::IMREAD_GRAYSCALE);

      if (imgleft.empty() || imgfront.empty() || imgright.empty() || imgback.empty())
        continue;

      if (!use_crop_image)
      {
        imgleft = imgleft.rowRange(kImageRow0, kImageRow1);
        imgfront = imgfront.rowRange(kImageRow0, kImageRow1);
        imgright = imgright.rowRange(kImageRow0, kImageRow1);
        imgback = imgback.rowRange(kImageRow0, kImageRow1);
      }
      Timer timer(" total cost ");
      feature_mapper->TrackMultiCam(imgleft, imgfront, imgright, imgback, odom_data.odometry);
      timer.Print();
    }
    else
    {
      DataLoader::DataType dt = dataloader.NextData(frame_data, odom_data);

      if (dt == DataLoader::DATA_IMAGE)
      {

        if (frame_data.id < start_index)
          continue;

        cv::Mat imgleft = cv::imread(data_path + frame_data.str_image_left, cv::IMREAD_GRAYSCALE);
        cv::Mat imgfront = cv::imread(data_path + frame_data.str_image_front, cv::IMREAD_GRAYSCALE);
        cv::Mat imgright = cv::imread(data_path + frame_data.str_image_right, cv::IMREAD_GRAYSCALE);
        cv::Mat imgback = cv::imread(data_path + frame_data.str_image_back, cv::IMREAD_GRAYSCALE);

        if (imgleft.empty() || imgfront.empty() || imgright.empty() || imgback.empty())
          continue;

        if (!use_crop_image)
        {
          imgleft = imgleft.rowRange(kImageRow0, kImageRow1);
          imgfront = imgfront.rowRange(kImageRow0, kImageRow1);
          imgright = imgright.rowRange(kImageRow0, kImageRow1);
          imgback = imgback.rowRange(kImageRow0, kImageRow1);
        }
        Timer timer(" total cost ");
        feature_mapper->TrackMultiCam(imgleft, imgfront, imgright, imgback, frame_data.timestamp);
        timer.Print();
      }
      else if (dt == DataLoader::DATA_ODOM)
      {
        feature_mapper->InsertOdometry(odom_data.timestamp, odom_data.odometry);
      }
      else
      {
        break;
      }
    }

    if (kbhit())
    {
      uchar ch = getchar();
      if (ch == KEYCODE_ESC)
      {

        break;
      }
    }
  }

  std::cout << kColorGreen;
  timer.Print();
  std::cout << kColorReset << std::endl;
  std::cout << kColorYellow << "Saving Maps " << kColorReset << std::endl;

  const std::string traj_file = "trajectory.txt";
  if ((access("trajectory.txt", F_OK)) != -1)
  {
    feature_mapper->AlignMaptoTrajectory(traj_file);
  }
  else
  {
    std::cout << kColorRed << " Can not find " << traj_file << kColorReset << std::endl;
  }

  feature_mapper->SaveMap("map_multicam.bin");

  std::this_thread::sleep_for(std::chrono::seconds(5));
  feature_mapper->Shutdown();

  return 0;
}
