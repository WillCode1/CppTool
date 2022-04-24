#include "plot_viewer.h"

std::mutex PlotViewer::global_mutex_;

PlotViewer::PlotViewer()
    : min_x_(0), max_x_(1000), min_y_(0), max_y_(50), running_(false),
      finish_request_(false), is_finish_(false) {}

PlotViewer &PlotViewer::GetInstance() {
  static PlotViewer *instance = nullptr;
  if (instance == nullptr) {
    global_mutex_.lock();
    instance = new PlotViewer();
    global_mutex_.unlock();
  }
  return *instance;
}

void PlotViewer::Run() {
  {
    std::unique_lock<std::mutex> lock(mutex_label_);
    std::unique_lock<std::mutex> lock2(mutex_data_);
    data_.resize(labels_.size());
    running_ = true;
  }

  if (title_.size() <= 0) {
    title_ = std::string("plotviewer");
  }
  pangolin::CreateWindowAndBind(title_, 480, 240);

  // Data logger object
  pangolin::DataLog log;

  if (labels_.size() == 0) {
    std::cout << kColorRed << " label size is 0 " << kColorReset << std::endl;
    return;
  }

  log.SetLabels(labels_);

  const float tinc = 1;

  // OpenGL 'view' of data. We might have many views of the same data.
  pangolin::Plotter plotter(&log, min_x_, max_x_, min_y_, max_y_, 30, 0.5);
  plotter.SetBounds(0.0, 1.0, 0.0, 1.0);
  plotter.Track("$i");
  // Add some sample annotations to the plot
  plotter.AddMarker(pangolin::Marker::Vertical, -1000,
                    pangolin::Marker::LessThan,
                    pangolin::Colour::Blue().WithAlpha(0.2f));
  plotter.AddMarker(pangolin::Marker::Horizontal, 100,
                    pangolin::Marker::GreaterThan,
                    pangolin::Colour::Red().WithAlpha(0.2f));
  plotter.AddMarker(pangolin::Marker::Horizontal, 10, pangolin::Marker::Equal,
                    pangolin::Colour::Green().WithAlpha(0.2f));

  pangolin::DisplayBase().AddDisplay(plotter);

  float t = 0;

  // Default hooks for exiting (Esc) and fullscreen (tab).
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    switch (labels_.size()) {
    case 1:
      log.Log(data_[0]);
      break;
    case 2:
      log.Log(data_[0], data_[1]);
      break;
    case 3:
      log.Log(data_[0], data_[1], data_[2]);
      break;

    case 4:
      log.Log(data_[0], data_[1], data_[2], data_[3]);
      break;
    case 5:
      log.Log(data_[0], data_[1], data_[2], data_[3], data_[4]);
      break;
    }

    if (data_change_) {
      data_change_ = false;
      t += tinc;
    }
    // Render graph, Swap frames and Process Events
    pangolin::FinishFrame();

    if (CheckFinish()) {
      break;
    }
  }
  log.Save(title_ + ".log");
  SetFinish();
}

bool PlotViewer::CheckFinish() {
  std::unique_lock<std::mutex> lock(mutex_finishe_);
  return finish_request_;
}

void PlotViewer::SetFinish() {
  std::unique_lock<std::mutex> lock(mutex_finishe_);
  is_finish_ = true;
}

void PlotViewer::RequestFinish() {
  std::unique_lock<std::mutex> lock(mutex_finishe_);
  finish_request_ = true;
}

bool PlotViewer::IsFinish() {
  std::unique_lock<std::mutex> lock(mutex_finishe_);
  return is_finish_;
}

PlotViewer::~PlotViewer() {}

void PlotViewer::SetLabel(const std::string &label) {

  std::unique_lock<std::mutex> lock(mutex_label_);

  if (running_)
    return;

  if (label_index_.count(label) != 0) {
    std::cout << kColorRed << " label " << label << " has already existed  "
              << kColorReset << std::endl;

  } else if (labels_.size() >= 5) {
    std::cout << kColorRed << " too many labels , skip  " << kColorReset
              << std::endl;
  } else {
    label_index_[label] = labels_.size();
    labels_.push_back(label);
  }
}

void PlotViewer::UpdateData(const std::string &label, float data) {

  std::unique_lock<std::mutex> lock(mutex_label_);
  std::unique_lock<std::mutex> lock2(mutex_data_);
  if (label_index_.count(label) == 0) {
    std::cout << kColorRed << " label " << label << " not found " << kColorReset
              << std::endl;
  } else {
    data_[label_index_[label]] = data;
    data_change_ = true;
  }
}

void PlotViewer::SetTitle(const std::string &title) { title_ = title; }

void PlotViewer::SetViewRange(float min_x, float max_x, float min_y,
                              float max_y) {
  min_x_ = min_x;
  max_x_ = max_x;
  min_y_ = min_y;
  max_y_ = max_y;
}
