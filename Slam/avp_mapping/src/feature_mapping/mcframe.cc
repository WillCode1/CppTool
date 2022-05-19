#include "mcframe.h"
#include "converter.h"
#include "orbmatcher.h"
#include <thread>
namespace FeatureSLAM {

long unsigned int MCFrame::next_id_ = 0;

MCFrame::MCFrame() { id_ = next_id_++; }

MCFrame::MCFrame(const MCFrame &mcframe)
    : id_(mcframe.id_), timestamp_(mcframe.timestamp_), tcw_(mcframe.tcw_),
      reference_mckeyframe_(mcframe.reference_mckeyframe_),
      multicam_(mcframe.multicam_), odom_vector_(mcframe.odom_vector_) {

  size_t cam_num = 4;

  if (frames_.size() != cam_num) {
    frames_.resize(cam_num);
    cam_ext_.resize(cam_num);
  }
  for (size_t i = 0; i < cam_num; i++) {
    frames_[i] = Frame(mcframe.frames_[i]);
    cam_ext_[i] = mcframe.cam_ext_[i];
  }
}

Frame *MCFrame::GetFrame(int camera_index) { return &frames_[camera_index]; }

MCFrame::MCFrame(const std::vector<cv::Mat> &image, const Eigen::Vector3d &odom,
                 double timestamp, std::vector<ORBextractor *> extractor,
                 ORBVocabulary *voc, MultiCam *pMulticam)
    : timestamp_(timestamp), reference_mckeyframe_(nullptr),
      multicam_(pMulticam), odom_vector_(odom) {

  id_ = next_id_++;
  size_t cam_num = 4;

  cam_ext_.resize(cam_num);
  if (image.size() != cam_num || extractor.size() != cam_num) {
    std::cerr << " fatal error , dimension error " << std::endl;
  }
  if (frames_.size() != cam_num)
    frames_.resize(cam_num);

  Timer timer("4 camera orb feature extraction ");
  for (size_t cam_index = 0; cam_index < cam_num; cam_index++) {

    frames_[cam_index] = Frame(timestamp, extractor[cam_index], voc);
    frames_[cam_index].ExtractORB(image[cam_index]);
    frames_[cam_index].Initialization();
    cam_ext_[cam_index] = multicam_->cam_extrinsic_[cam_index];
  }
  timer.Print();
  tcw_ = cv::Mat();
}
void MCFrame::SetPose(cv::Mat Tcw) { tcw_ = Tcw.clone(); }

void MCFrame::SetOdometry(const Eigen::Vector3d &odom) { odom_vector_ = odom; }

void MCFrame::ComputeBoW() {
  for (int i = 0; i < 4; i++) {
    frames_[i].ComputeBoW();
  }
}
// 这里的Tcw应该为odom增量转相机移动增量
void MCFrame::UpdatePose() {
  if (tcw_.empty()) {
    cout << " fatal error  MCframe pose empty " << endl;
    return;
  }
  for (int i = 0; i < 4; i++) {
    frames_[i].SetPose(cam_ext_[i] * tcw_);
  }
}
}
