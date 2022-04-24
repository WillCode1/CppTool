#include "map_alignment.h"

namespace FeatureSLAM {

MapAlignment::MapAlignment(Map *map) : map_(map) {}

void MapAlignment::Align(const string &trajectory_file) const {

  std::vector<MCKeyFrame *> all_mc_keyfrm = map_->GetAllMCKeyFrames();
  std::map<unsigned long, MCKeyFrame *> index_mc_keyfrms;

  std::map<unsigned long, cv::Mat> mc_keyfrm_tcw;
  std::map<unsigned long, cv::Mat> correct_mc_keyfrm_twc;

  for (auto mc_keyfrm : all_mc_keyfrm) {
    index_mc_keyfrms[mc_keyfrm->frame_id_] = mc_keyfrm;
  }

  std::ifstream fin;
  fin.open(trajectory_file);
  if (!fin.is_open()) {
    std::cout << kColorRed << " Can not find file " << trajectory_file
              << kColorReset << std::endl;
    exit(0);
  }

  unsigned long frame_id = 0;
  while (!fin.eof()) {
    std::string line;
    std::getline(fin, line);
    double timestamp;
    double x, y, z, qx, qy, qz, qw;
    sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &timestamp, &x, &y,
           &z, &qx, &qy, &qz, &qw);

    if (index_mc_keyfrms.count(frame_id) == 0) {
      frame_id++;
      continue;
    }

    auto mc_keyfrm = index_mc_keyfrms[frame_id];

    // set old pose
    mc_keyfrm_tcw[mc_keyfrm->id_] = mc_keyfrm->GetPose();

    // set correct pose

    Mat44_t trans_w2base = Mat44_t::Identity();
    trans_w2base(0, 3) = x;
    trans_w2base(1, 3) = y;
    trans_w2base(2, 3) = z;

    Quat_t quat(qw, qx, qy, qz);
    trans_w2base.block(0, 0, 3, 3) = quat.toRotationMatrix();

    cv::Mat correct_tcw = MultiCamExt::GetInstance().BaselinkPose2Camera(
        trans_w2base, FRONT_CAMERA);
    mc_keyfrm->SetPose(correct_tcw);
    correct_mc_keyfrm_twc[mc_keyfrm->id_] = correct_tcw.inv();

    frame_id++;
  }
  fin.close();

  // correct map points
  std::vector<MapPoint *> all_mps = map_->GetAllMapPoints();
  for (auto mp : all_mps) {

    if (mp->isBad())
      continue;
    auto mc_keyfrm = mp->GetReferenceKeyFrame()->GetMCKeyFrame();
    unsigned long mc_keyfrm_id = mc_keyfrm->id_;

    if (mc_keyfrm_tcw.count(mc_keyfrm_id) == 0 ||
        correct_mc_keyfrm_twc.count(mc_keyfrm_id) == 0) {
      std::cout << kColorRed
                << " impossible : can not find corresponding mc_keyfrm "
                << kColorReset << std::endl;
      continue;
    }

    cv::Mat tcw = mc_keyfrm_tcw[mc_keyfrm_id];
    cv::Mat correct_twc = correct_mc_keyfrm_twc[mc_keyfrm_id];

    cv::Mat pos = mp->GetWorldPos();
    cv::Mat pos4d = (cv::Mat_<float>(4, 1) << pos.at<float>(0),
                     pos.at<float>(1), pos.at<float>(2), 1);

    cv::Mat new_pos = correct_twc * tcw * pos4d;
    cv::Mat new_pos3d = new_pos.rowRange(0, 3);
    mp->SetWorldPos(new_pos3d);
    mp->UpdateNormalAndDepth();
  }

  std::cout << kColorGreen << " Done !!! " << kColorReset << std::endl;
}
}
