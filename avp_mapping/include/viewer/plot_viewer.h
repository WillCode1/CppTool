#pragma once

#include "colordef.h"
#include <Eigen/Dense>
#include <mutex>
#include <pangolin/pangolin.h>
#include <thread>

class PlotViewer {

public:
  static PlotViewer &GetInstance();
  void Run();
  ~PlotViewer();

  void SetLabel(const std::string &label);
  void UpdateData(const std::string &label, float data);

  void SetTitle(const std::string &title);
  void SetViewRange(float min_x, float max_x, float min_y, float max_y);

  void RequestFinish();
  bool IsFinish();

private:
  PlotViewer();
  bool CheckFinish();
  void SetFinish();

private:
  static std::mutex global_mutex_;

  float min_x_;
  float max_x_;
  float min_y_;
  float max_y_;
  std::string title_;
  bool running_;
  std::mutex mutex_label_;
  std::mutex mutex_data_;
  bool data_change_;
  std::map<std::string, unsigned int> label_index_;
  std::vector<std::string> labels_;
  std::vector<float> data_;

  std::mutex mutex_finishe_;
  bool finish_request_;
  bool is_finish_;
};
