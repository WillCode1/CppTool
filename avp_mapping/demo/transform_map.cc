#include "basictool.h"
#include "map.h"
#include <iostream>

using namespace FeatureSLAM;

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

  cv::Mat transform = cv::Mat::eye(4,4,CV_32FC1);


  double theta = 1.7593465054845188  + M_PI;
  double x = 66;
  double y = 50;

  transform .at<float> (0,3) = x;
  transform.at<float>(1,3) = y;
  transform .at<float>(0,0 )  =cos(theta);
  transform.at<float>(1,1) = cos(theta);
  transform.at<float>(0,1) = -sin(theta);
  transform.at<float>(1,0) = sin(theta);

  cv::Mat tt = transform.inv();
   map->MoveMap( tt );
   map->Save("moved_map.bin.bow", true);
  return 1;
}
