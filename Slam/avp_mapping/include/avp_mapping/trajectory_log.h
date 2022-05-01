#pragma once
#include "log.h"
#include "vslam_types.h"
#include <fstream>
namespace SemanticSLAM
{
  class TrajectoryLog
  {
  public:
    enum class Format
    {
      TUM = 0,
      KITTI = 1,
      NULLMAX
    };

    TrajectoryLog() = delete;
    TrajectoryLog(const std::string &filename, Format format = Format::TUM);
    ~TrajectoryLog();
    void ExportPose(const uint64_t timestamp, const Mat44_t &pose);
    void SaveLog(const std::string &filename);

  private:
    std::vector<uint64_t> timestmaps_;
    std::vector<Mat44_t> poses_;

    Format format_;
    std::ofstream fout_;
  };
}
