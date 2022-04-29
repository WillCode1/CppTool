// Have read
#include "avp_mapping_interface.h"
#include "colordef.h"
#include "dataloader.h"
#include "timer.h"
#include "viewerconfig.h"
#include <Eigen/Dense>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <termios.h>
#include <thread>
#include <unistd.h>

#ifdef ENABLE_VIEWER
#include "hpa_viewer.h"
#include "plot_viewer.h"
#endif
const int kKeyCodeR = 114;
const int kKeyCodeS = 115;
const int kKeyCodeSpace = 32;
const int kKeyCodeEsc = 0x1B;

// AVP : Automated Valet Parking // 自动代客泊车
// HPA : Home zone Parking Assistant // 家庭区域停车助理
using namespace SemanticSLAM;

struct DemoOption
{
  int start_index;
  int image_width;
  int image_height;
  bool use_compress;

  double x_offset;
  double y_offset;
  double init_x;
  double init_y;
  double init_theta;
  std::string str_map;
  std::string str_odom;
  std::string str_image_folder;
  std::string str_image_timestamp;
  std::string str_bmp_folder;
};

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

DemoOption LoadInitOption(const std::string &filename);

bool LoadImageFromFolder(cv::Mat &image_slot, cv::Mat &image_dash, cv::Mat &image_arrow, cv::Mat &image_lane,
                         cv::Mat &image_raw, int image_id, const DemoOption &demo_option);

void SkipImage(int start_index, std::queue<std::pair<uint64_t, int>> &image_id)
{
  for (int i = 0; i < start_index; i++)
  {
    if (image_id.empty())
      break;
    image_id.pop();
  }
}

void ShowDemoInfo()
{
  std::cout << kColorYellow << "keyboard control:  " << std::endl;
  std::cout << "   press 'r'  to reset system " << std::endl;
  std::cout << "   press 'space' to pause/resume system " << std::endl;
  std::cout << "   press 's'  to save map " << kColorReset << std::endl;
}

const std::string GetCarModelPath(const std::string &exe_path)
{
  const size_t bar_index = exe_path.find_last_of('/');
  std::string model_path = exe_path.substr(0, bar_index + 1) + "../../model/car.obj";
  return model_path;
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cout << " usage : ./test_hpa_mapping  config_file  " << std::endl;
    return 1;
  }

  const int kFps = 30;
  const int kTimeInterval = 1000 / kFps;

  DemoOption demo_option = LoadInitOption(argv[1]);

  DataLoader dataloader(DataLoader::NOT_ALIGNED);

  dataloader.SetOdometryFile(demo_option.str_odom);
  dataloader.SetImageTimeStampefile(demo_option.str_image_timestamp);

  cv::Mat image_slot, image_dash, image_arrow, image_lane, image_raw;

  std::ofstream fout;
  fout.open("test_hpa_mapping_result.txt");
  if (!fout.is_open())
  {
    std::cout << " Create result.txt failed " << std::endl;
    return -1;
  }
  fout.precision(20);
  std::cout.precision(15);

  //  STEP 4 CREATE AVP LOCATOR
  std::shared_ptr<AvpMapper> avp_mapper_ptr = CreateAvpMapper(argv[1]);
  // main loop

  ShowDemoInfo();

  bool stop = false;

  while (true)
  {

    // Key control
    if (kbhit())
    {
      uchar key_code = getchar();
      if (key_code == kKeyCodeS)
      {
        std::cout << kColorGreen;
        avp_mapper_ptr->SaveMap("map.bin");
        std::cout << kColorReset;
      }
      else if (key_code == kKeyCodeSpace)
      {
        std::cout << kColorRed << " Stop" << kColorReset << std::endl;
        stop = !stop;
      }
      else if (key_code == kKeyCodeR)
      {
        std::cout << kColorYellow << " Rest System " << kColorReset << std::endl;
        avp_mapper_ptr->Reset();
      }
      else if (key_code == kKeyCodeEsc)
      {
        std::cout << kColorGreen << " Esc" << kColorReset << std::endl;
        avp_mapper_ptr->SaveMap("map.bin");
        avp_mapper_ptr->SaveTrajectory("trajectory.txt");
        avp_mapper_ptr->SaveOdometry("odometry.txt");
        avp_mapper_ptr.reset();
        break;
      }
    }
    if (stop)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(kTimeInterval));
      continue;
    }

    DataFrame frame_data;
    OdometryData odom_data;
    DataLoader::DataType data_type = dataloader.NextData(frame_data, odom_data);

    if (data_type == DataLoader::DataType::DATA_IMAGE)
    {

      if (frame_data.id < demo_option.start_index)
        continue;

      std::cout << " ======================================== " << std::endl;
      std::cout << " current image id  " << frame_data.id << std::endl;

      if (!LoadImageFromFolder(image_slot, image_dash, image_arrow, image_lane, image_raw, frame_data.id, demo_option))
        continue;

      std::vector<unsigned char *> segmentation_imgs{image_slot.data, image_dash.data, image_arrow.data, image_lane.data};

#ifdef ENABLE_VIEWER
      avp_mapper_ptr->SetRawImage(image_raw);
#endif

      Timer timer_mapping_duration("mapping time cost per frame ");

      // Main functions
      AVPPose current_pose = avp_mapper_ptr->GrabSegImages(frame_data.timestamp, segmentation_imgs);

      timer_mapping_duration.Print();
      double mapping_duration = timer_mapping_duration.GetTimeConsuming();

#ifdef ENABLE_VIEWER

      PlotViewer::GetInstance().UpdateData("total", timer_mapping_duration.GetTimeConsuming());

      if (avp_mapper_ptr->RemapImageRequired())
      {
        HpaViewer::GetInstance().AddMarker();
      }

      // Draw  Semantic Map

      if (frame_data.id == demo_option.start_index || frame_data.id % 10 == 0)
      {
        std::cout << " frame id " << frame_data.id << std::endl;
        Timer timer_update("update map ");
        HpaViewer::GetInstance().UpdateSemanticMap(avp_mapper_ptr->GetSemanticPoints());
        timer_update.Print();
        PlotViewer::GetInstance().UpdateData("update semantic points", timer_update.GetTimeConsuming());
      }
#endif
      fout << current_pose.x << " " << current_pose.y << " " << mapping_duration << std::endl;

      if (mapping_duration < kTimeInterval) // ms
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(kTimeInterval - (int)mapping_duration));
      }
    }
    else if (data_type == DataLoader::DataType::DATA_ODOM)
    {
      avp_mapper_ptr->SetOdometry(odom_data.timestamp, odom_data.odometry);
    }
  }
  fout.close();
  std::cout << " Done " << std::endl;

  return 0;
}

DemoOption LoadInitOption(const std::string &filename)
{

  DemoOption option;

  cv::FileStorage settings(filename, cv::FileStorage::READ);
  const size_t bar_index = filename.find_last_of('/');
  std::string str_data_path = filename.substr(0, bar_index + 1);

  const std::string str_map = settings["map"];
  option.str_map = str_data_path + str_map;

  std::string str_odometry = settings["wheel_odom"];
  option.str_odom = str_odometry;

  const std::string str_image_folder = settings["image_folder"];
  option.str_image_folder = str_image_folder;

  option.start_index = settings["start_index"];
  option.image_width = settings["image_width"];
  option.image_height = settings["image_height"];

  option.x_offset = settings["map_offset_x"];
  option.y_offset = settings["map_offset_y"];

  option.use_compress = settings["use_compress"].real() > 0;

  const std::string timestamp = settings["image_timestamp"];
  option.str_image_timestamp = timestamp;

  const std::string bmp_folder = settings["bmp_folder"];
  option.str_bmp_folder = bmp_folder;

  std::cout << " use compressed data : ";
  if (option.use_compress)
  {
    std::cout << " yes " << std::endl;
  }
  else
  {
    std::cout << " no " << std::endl;
  }
  std::cout << " image width " << option.image_width << std::endl;
  std::cout << " image height " << option.image_height << std::endl;
  settings.release();
  return option;
}

bool LoadImageFromFolder(cv::Mat &image_slot, cv::Mat &image_dash,
                         cv::Mat &image_arrow, cv::Mat &image_lane,
                         cv::Mat &image_raw, int image_id,
                         const DemoOption &demo_option)
{

  std::string data_path = demo_option.str_image_folder;

  if (!demo_option.use_compress)
  {
    image_slot = cv::imread(data_path + "/slot/" + std::to_string(image_id) + "_slot.bmp", cv::IMREAD_GRAYSCALE);
    image_dash = cv::imread(data_path + "/dash/" + std::to_string(image_id) + "_dash.bmp", cv::IMREAD_GRAYSCALE);
    image_arrow = cv::imread(data_path + "/arrow/" + std::to_string(image_id) + "_arrow.bmp", cv::IMREAD_GRAYSCALE);

    image_raw = cv::imread(data_path + "/result/" + std::to_string(image_id) + ".bmp");

    image_lane = cv::imread(data_path + "/lane/" + std::to_string(image_id) + "_lane.bmp", cv::IMREAD_GRAYSCALE);

    if (image_lane.empty())
    {
      image_lane = cv::Mat::zeros(demo_option.image_height, demo_option.image_width, CV_8UC1);
    }
  }
  else
  {
    cv::Mat image = cv::imread(data_path + "/compress/seg_" + std::to_string(image_id) + ".png", cv::IMREAD_GRAYSCALE);

    if (image.empty())
    {
      std::cout << " open compressed seg image failed " << std::endl;
      return false;
    }

    if (image.rows != demo_option.image_height || image.cols != demo_option.image_width)
    {
      cv::resize(image, image, cv::Size(demo_option.image_width, demo_option.image_height));
    }

    cv::inRange(image, 1, 1, image_slot);
    cv::inRange(image, 2, 2, image_dash);
    cv::inRange(image, 3, 3, image_arrow);
    cv::inRange(image, 4, 4, image_lane);

    image_raw = image.clone();
    cv::cvtColor(image_raw, image_raw, cv::COLOR_GRAY2BGR);

    std::vector<cv::Mat> mbgr(3);
    mbgr[0] = image_slot + image_lane;
    mbgr[1] = image_arrow + image_lane + 0.6 * image_dash;
    mbgr[2] = image_dash;
    cv::merge(mbgr, image_raw);
  }

  if (image_slot.empty() || image_dash.empty() || image_arrow.empty() || image_raw.empty() || image_lane.empty())
  {
    std::cout << data_path + "/result/" + std::to_string(image_id) + ".bmp" << std::endl;
    std::cout << " image empty " << std::endl;
    return false;
  }

  if (image_slot.rows != demo_option.image_height || image_slot.cols != demo_option.image_width)
  {
    cv::resize(image_slot, image_slot, cv::Size(demo_option.image_width, demo_option.image_height));
    cv::resize(image_dash, image_dash, cv::Size(demo_option.image_width, demo_option.image_height));
    cv::resize(image_arrow, image_arrow, cv::Size(demo_option.image_width, demo_option.image_height));
    cv::resize(image_raw, image_raw, cv::Size(demo_option.image_width, demo_option.image_height));
  }
  return true;
}
