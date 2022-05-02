#include "basictool.h"
#include "mcframe.h"

namespace FeatureSLAM
{
  std::vector<cv::Mat> BasicTool::LoadExtrinsicParam(const cv::FileStorage &fsettings)
  {
    std::vector<cv::Mat> cam_extrinsic;
    cam_extrinsic.resize(4);
    cam_extrinsic[LEFT_CAMERA] = fsettings["Tcam_01"].mat();
    cam_extrinsic[FRONT_CAMERA] = fsettings["Tcam_11"].mat();
    cam_extrinsic[RIGHT_CAMERA] = fsettings["Tcam_21"].mat();
    cam_extrinsic[BACK_CAMERA] = fsettings["Tcam_31"].mat();
    return cam_extrinsic;
  }

  void BasicTool::LoadMultiCamImages(std::vector<std::vector<string>> &image_name_lists, std::vector<double> &timestamps)
  {
    image_name_lists.resize(4);
    for (int i = 0; i < 10000; i++)
    {
      image_name_lists[0].push_back("/vc0/img" + std::to_string(i) + ".png");
      image_name_lists[1].push_back("/vc1/img" + std::to_string(i) + ".png");
      image_name_lists[2].push_back("/vc2/img" + std::to_string(i) + ".png");
      image_name_lists[3].push_back("/vc3/img" + std::to_string(i) + ".png");
      timestamps.push_back(0.001 * i);
    }
  }
}
