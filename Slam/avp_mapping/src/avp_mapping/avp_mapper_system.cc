// Have read
#include "avp_mapper_system.h"

namespace SemanticSLAM
{
  AvpMapperSystem::AvpMapperSystem(const std::string &config_file)
  {
    ShowInfo();

    hpa_map_ = std::make_shared<HpaMap>();
    tracker_ = std::make_shared<BevTracker>(config_file, hpa_map_);
    message_filter_ = std::make_shared<MessageFilter>(10, 50);
    LoadRemap(config_file);
  }

  AvpMapperSystem::~AvpMapperSystem() {}

  AVPPose AvpMapperSystem::GrabSegImages(uint64 microsecond, std::vector<unsigned char *> seg_imgs)
  {
    AVPPose current_pose;
    WheelOdometry odometry;

    // find correspond odometry data
    bool ret = message_filter_->GetOdomByTimestamp(microsecond, odometry);

    if (!ret)
    {
      std::cout << kColorRed << " Can't Find Correspond Odometry " << kColorReset << std::endl;
      return current_pose;
    }
    tracker_->GrabSegImages(microsecond, seg_imgs, odometry, current_pose);
    return current_pose;
  }

  void AvpMapperSystem::SetCurrentPose(uint64 microsecond, const AVPPose &pose) {}

  void AvpMapperSystem::SetRawImage(cv::Mat &image_raw)
  {
    tracker_->SetRawImage(image_raw);
  }

  void AvpMapperSystem::SetOdometry(uint64 microsecond, WheelOdometry odom)
  {
    message_filter_->AddOdomData(microsecond, odom);
  }

  void AvpMapperSystem::SaveMap(const std::string &filename)
  {
    hpa_map_->SaveMap(filename);
  }

  void AvpMapperSystem::SaveTrajectory(const std::string &filename)
  {
    tracker_->SaveTrajectory(filename);
  }

  void AvpMapperSystem::SaveOdometry(const std::string &filename)
  {
    tracker_->SaveOdometry(filename);
  }

  bool AvpMapperSystem::RemapImageRequired()
  {
    return tracker_->RemapImageRequired();
  }

  std::vector<std::pair<cv::Mat, cv::Mat>> AvpMapperSystem::GetPanoramicRemap()
  {
    if (remaps_.size() != 4)
    {
      NM_ERROR("REMAP SIZE IS WRONG")
      std::cout << kColorRed << " REMAP SIZE IS WRONG " << kColorReset << std::endl;
    }
    return remaps_;
  }

  void AvpMapperSystem::Reset()
  {
    tracker_->Reset();
    NM_INFO("RESET !!!! ");
  }

  std::vector<PointTyped> AvpMapperSystem::GetSemanticPoints()
  {
    return hpa_map_->GetSemanticPoints();
  }

  void AvpMapperSystem::ShowInfo()
  {
    std::cout << kColorGreen << std::endl;
    std::cout << "  █░█ █▀█ ▄▀█   █▀▄▀█ ▄▀█ █▀█ █▀█ █ █▄░█ █▀▀ " << std::endl;
    std::cout << "  █▀█ █▀▀ █▀█   █░▀░█ █▀█ █▀▀ █▀▀ █ █░▀█ █▄█ " << std::endl;
    std::cout << kColorReset << std::endl;
  }

  void AvpMapperSystem::LoadRemap(const std::string &config)
  {
    const size_t bar_index = config.find_last_of('/');
    std::string str_data_path = config.substr(0, bar_index + 1);

    // skill: c++11读取配置文件技巧
    cv::FileStorage fs(config, cv::FileStorage::READ);
    cv::FileNode node_remap = fs["remap"];  // remap.yaml

    if (node_remap.empty())
    {
      NM_ERROR("CAN NOT FIND REMAP IN CONFIG");
      std::cout << kColorRed << "CAN NOT FIND REMAP IN CONFIG" << kColorReset << std::endl;
      return;
    }
    std::string remap_file = str_data_path + node_remap.string();
    fs.release();

    cv::FileStorage fs_remap(remap_file, cv::FileStorage::READ);
    if (!fs_remap.isOpened())
    {
      NM_ERROR("REMAP FILE DOES NOT EXIST");
      std::cout << kColorRed << "REMAP FILE DOES NOT EXIST" << kColorReset << std::endl;
      return;
    }

    // question: 这些都是什么矩阵，remap.yaml都是怎么生成的
    remaps_.push_back(std::make_pair(fs_remap["mapx_left"].mat(), fs_remap["mapy_left"].mat()));
    remaps_.push_back(std::make_pair(fs_remap["mapx_front"].mat(), fs_remap["mapy_front"].mat()));
    remaps_.push_back(std::make_pair(fs_remap["mapx_right"].mat(), fs_remap["mapy_right"].mat()));
    remaps_.push_back(std::make_pair(fs_remap["mapx_back"].mat(), fs_remap["mapy_back"].mat()));
    fs_remap.release();
  }

  std::shared_ptr<AvpMapper> CreateAvpMapper(const std::string &config_file)
  {
    std::shared_ptr<AvpMapper> shared_mapper_ptr = std::make_shared<AvpMapperSystem>(config_file);
    return shared_mapper_ptr;
  }
}
