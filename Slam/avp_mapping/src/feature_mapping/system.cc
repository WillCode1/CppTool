#include "system.h"
#include "converter.h"
#include "map_alignment.h"
#include "optimizer/full_bundle_adjuster.h"
#include "optimizer/global_bundle_adjuster.h"
#include <iomanip>
#include <iostream>
#include <thread>
bool has_suffix(const std::string &str, const std::string &suffix) {
  std::size_t index = str.find(suffix, str.size() - suffix.size());
  return (index != std::string::npos);
}

namespace FeatureSLAM {

void ShowInfo() {

  std::cout << kColorGreen << std::endl;
  std::cout << "      █▄░█ █░█ █░░ █░░ █▀▄▀█ ▄▀█ ▀▄▀" << std::endl;
  std::cout << "      █░▀█ █▄█ █▄▄ █▄▄ █░▀░█ █▀█ █░█" << std::endl << std::endl;
  std::cout << "-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄"
            << std::endl
            << std::endl;
  std::cout << "   🄼 🅄 🄻 🅃 🄸 🄲 🄰 🄼  🄵 🄴 🄰 🅃 🅄 🅁 🄴  🄼 🄰 🄿 🄿 🄸 🄽 🄶" << std::endl
            << std::endl;
  std::cout << "-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄-̄"
            << std::endl
            << std::endl;
  std::cout << kColorReset << std::endl;
}

System::System(const string &setting_file) {

  const size_t bar_index = setting_file.find_last_of('/');
  std::string config_path = setting_file.substr(0, bar_index + 1);

  // Check settings file
  cv::FileStorage fsettings(setting_file, cv::FileStorage::READ);
  if (!fsettings.isOpened()) {

    std::cerr << kColorRed
              << "Failed to open settings file at: " << setting_file
              << kColorReset << std::endl;
    exit(-1);
  }

  ShowInfo();

  // Load ORB Vocabulary
  std::cout << kColorYellow << "Loading ORB Vocabulary ... " << kColorReset
            << std::endl;

  std::string voc_file = config_path + fsettings["ORBVoc"].string();

  vocabulary_ = new ORBVocabulary();
  bool voc_loaded = false; // chose loading method based on file extension
  if (has_suffix(voc_file, ".txt"))
    voc_loaded = vocabulary_->loadFromTextFile(voc_file);
  else if (has_suffix(voc_file, ".bin"))
    voc_loaded = vocabulary_->loadFromBinaryFile(voc_file);
  else
    voc_loaded = false;
  if (!voc_loaded) {
    std::cerr << "Wrong path to vocabulary. " << std::endl;
    std::cerr << "Failed to open at: " << voc_file << std::endl;
    exit(-1);
  }
  std::cout << kColorYellow << "Vocabulary loaded!" << kColorReset << std::endl;

  // Create KeyFrame Database
  keyfrm_database_ = new KeyFrameDatabase(*vocabulary_);

  // Create the Map
  map_ = new Map();

  tracker_ =
      new Tracking(this, vocabulary_, map_, keyfrm_database_, setting_file);
#ifdef ENABLE_VIEWER
  frame_drawer_ = new FrameDrawer(map_);
  map_drawer_ = new MapDrawer(map_, setting_file);

  // initialize viewer
  tracker_->SetViewer(frame_drawer_, map_drawer_);
  viewer_ =
      new Viewer(this, frame_drawer_, map_drawer_, tracker_, setting_file);
  viewer_thread_ = new thread(&Viewer::Run, viewer_);
  tracker_->SetViewer(viewer_);

#endif

  message_filter_ = std::make_shared<MessageFilter>(100, 100);
}

System::System(const string &map_file, const string &voc_file,
               const string &setting_file, const bool use_viewer) {

  // Check settings file
  cv::FileStorage settings(setting_file.c_str(), cv::FileStorage::READ);
  if (!settings.isOpened()) {
    cerr << "Failed to open settings file at: " << setting_file << endl;
    exit(-1);
  }

  // Load ORB Vocabulary
  std::cout << endl << "Loading ORB Vocabular..." << std::endl;

  vocabulary_ = new ORBVocabulary();
  bool voc_loaded = false; // chose loading method based on file extension
  if (has_suffix(voc_file, ".txt"))
    voc_loaded = vocabulary_->loadFromTextFile(voc_file);
  else if (has_suffix(voc_file, ".bin"))
    voc_loaded = vocabulary_->loadFromBinaryFile(voc_file);
  else
    voc_loaded = false;
  if (!voc_loaded) {
    std::cerr << "Wrong path to vocabulary. " << std::endl;
    std::cerr << "Failed to open at: " << voc_file << std::endl;
    exit(-1);
  }
  std::cout << "Vocabulary loaded!" << std::endl;

  keyfrm_database_ = new KeyFrameDatabase(*vocabulary_);

  string front_camera_mask = settings["MaskFront"];
  std::vector<cv::Mat> cam_exts = BasicTool::LoadExtrinsicParam(settings);
  map_ = new Map();
  map_->Load(map_file, *vocabulary_, front_camera_mask, cam_exts);

  std::vector<KeyFrame *> vpAllKFs = map_->GetAllKeyFrames();

  for (size_t i = 0; i < vpAllKFs.size(); i++)
    keyfrm_database_->add(vpAllKFs[i]);

  tracker_ =
      new Tracking(this, vocabulary_, map_, keyfrm_database_, setting_file);
  tracker_->state_ = Tracking::LOST;

#ifdef ENABLE_VIEWER

  frame_drawer_ = new FrameDrawer(map_);
  map_drawer_ = new MapDrawer(map_, setting_file);
  viewer_ =
      new Viewer(this, frame_drawer_, map_drawer_, tracker_, setting_file);
  if (use_viewer)
    viewer_thread_ = new thread(&Viewer::Run, viewer_);
  tracker_->SetViewer(viewer_);

#endif
}

cv::Mat System::TrackMultiCam(const cv::Mat &imleft, const cv::Mat &imfront,
                              const cv::Mat &imright, const cv::Mat &imback,
                              uint64_t timestamp) {
  // find  correspond   odometry

  SemanticSLAM::WheelOdometry odom;
  bool ret = message_filter_->GetOdomByTimestamp(timestamp, odom);

  if (!ret) {

    return cv::Mat();
  }
  double time_sec = timestamp / 1e6;
  return tracker_->GrabImageMultiCam(imleft, imfront, imright, imback, time_sec,
                                     odom);
}

cv::Mat System::TrackMultiCam(const cv::Mat &imleft, const cv::Mat &imfront,
                              const cv::Mat &imright, const cv::Mat &imback,
                              const SemanticSLAM::WheelOdometry &odometry) {

  // question: timestamp=0? 区别
  double time_sec = 0.0;
  return tracker_->GrabImageMultiCam(imleft, imfront, imright, imback, time_sec,
                                     odometry);
}

void System::InsertOdometry(uint64_t timestamp,
                            const SemanticSLAM::WheelOdometry &odometry) {
  message_filter_->AddOdomData(timestamp, odometry);
}

void System::Shutdown() {

#ifdef ENABLE_VIEWER
  if (viewer_) {
    viewer_->RequestFinish();
    while (!viewer_->isFinished())
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (viewer_)
    pangolin::BindToContext("ORB-SLAM2: Map Viewer");
#endif
}

bool System::SaveMap(const std::string &filename, bool with_bow) {
  // question: 这里本身是干什么的?

  //  std::cout << "System Saving to " << filename << endl;

  //  map_->SaveOdometry("odom_inc.txt");

  //  map_->Save(filename, with_bow);
  //  map_->SaveKeyFrameOdom("keyodom_pose.txt");

  //  // alignment pose to odometry
  //  map_->AlignToOdometry();
  //  map_->Save(filename + ".alignment2odom", with_bow);
  //  map_->SaveKeyFrameOdom("keyodom_pose_after_alignment.txt");

  //  // aplly bundle adjustment

  //  MultiCam *multicam = new
  //  MultiCam(MultiCamExt::GetInstance().GetExtrinsic());
  //  const auto global_ba = GlobalBundleAdjuster(map_, 10, multicam);
  //  global_ba.Optimize();

  //  //  const auto full_ba = FullBundleAdjuster(map_, 10, multicam);
  //  //  full_ba.Optimize();

  map_->Save(filename, with_bow);
  map_->Save(filename + ".bow", true);
  //  map_->SaveKeyFrameOdom("keyodom_pose_after_bundle_adjustment.txt");

  return true;
}

void System::AlignMaptoTrajectory(const string &file_name) {
  const auto map_align = MapAlignment(map_);
  map_align.Align(file_name);
}

std::shared_ptr<FeatureMapper>
CreateFeatureMapper(const std::string &config_file) {
  std::shared_ptr<FeatureMapper> shared_mapper_ptr =
      std::make_shared<System>(config_file);
  return shared_mapper_ptr;
}
}
