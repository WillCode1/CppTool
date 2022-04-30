#pragma once
#include "interface/avp_mapping_interface.h"
#include <memory>
#include <string>

#include "bevtracker.h"
#include "message_filter/message_filter.h"
namespace SemanticSLAM
{
  class AvpMapperSystem : public AvpMapper
  {
  public:
    AvpMapperSystem() = delete;
    AvpMapperSystem(const std::string &config_file);
    virtual ~AvpMapperSystem();

    virtual AVPPose GrabSegImages(uint64 microsecond, std::vector<unsigned char *> seg_imgs) override;

    virtual void SetCurrentPose(uint64 microsecond, const AVPPose &pose) override;

    virtual void SetRawImage(cv::Mat &image_raw) override;

    virtual void SetOdometry(uint64 microsecond, WheelOdometry odom) override;

    virtual void SaveMap(const std::string &filename) override;

    virtual void SaveTrajectory(const std::string &filename) override;

    virtual void SaveOdometry(const std::string &filename) override;

    virtual bool RemapImageRequired() override;

    virtual std::vector<std::pair<cv::Mat, cv::Mat>> GetPanoramicRemap() override;

    virtual void Reset() override;

    virtual std::vector<PointTyped> GetSemanticPoints() override;

    void ShowInfo();

  private:
    void LoadRemap(const std::string &config);

  private:
    std::shared_ptr<HpaMap> hpa_map_;
    std::vector<std::pair<cv::Mat, cv::Mat>> remaps_;   // 全景重映射PanoramicRemap
    std::shared_ptr<BevTracker> tracker_;
    std::shared_ptr<MessageFilter> message_filter_;
  };
}
