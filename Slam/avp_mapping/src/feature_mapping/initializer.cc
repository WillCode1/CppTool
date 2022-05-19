#include "initializer.h"
#include "DUtils/Random.h"
#include "converter.h"
#include "orbmatcher.h"

#include <thread>

namespace FeatureSLAM {

Initializer::Initializer(const MCFrame &reference_frame, float reproj_err_thr,
                         float parallax_deg_thr)
    : reproj_err_thr_(reproj_err_thr), parallax_deg_thr_(parallax_deg_thr) {

  keypoints1_.resize(4);
  bearings1_.resize(4);

  for (int cam_index = 0; cam_index < 4; cam_index++) {
    keypoints1_[cam_index] = reference_frame.frames_[cam_index].keypts_;
    for (size_t i = 0; i < keypoints1_[cam_index].size(); i++) {
      Vec3_t bearing = Converter::KeypointToBearing(keypoints1_[cam_index][i]);
      bearings1_[cam_index].push_back(bearing);
    }
  }

  ref_odom_ = Converter::toOdometryMatrix(reference_frame.odom_vector_);
}

bool Initializer::Initialize(
    const MCFrame &current_frame,
    const std::vector<std::vector<int>> &matches12, cv::Mat &T21,
    std::vector<std::vector<cv::Point3f>> &init_mappoints,
    std::vector<std::vector<bool>> &is_triangulated) {

  // Check whether the vehicle is moving

  Mat44_t cur_odom = Converter::toOdometryMatrix(current_frame.odom_vector_);

  if (!IsVehicleMoved(cur_odom)) {
    std::cout << kColorYellow << " the vehicle is not moving " << kColorReset
              << std::endl;
    return false;
  }

  float parallax_deg = 0;

  Mat44_t tco_ref = Converter::toMatrix4d(
      MultiCamExt::GetInstance().BaselinkPose2Camera(ref_odom_, FRONT_CAMERA));
  Mat44_t tco_cur = Converter::toMatrix4d(
      MultiCamExt::GetInstance().BaselinkPose2Camera(cur_odom, FRONT_CAMERA));
  Mat44_t pose_ref_to_cur = tco_cur * tco_ref.inverse();

  // Force set ref and cur frame pose
  T21 = Converter::toCvMat(pose_ref_to_cur);
  unsigned int num_valid_pts = 0;

  for (int cam_index = 0; cam_index < 4; cam_index++) {

    cv::Mat ref_front_pose = cv::Mat::eye(4, 4, CV_32FC1);

    cv::Mat trans_ref_w = MultiCamExt::GetInstance().GetTargetCamPose(
        ref_front_pose, FRONT_CAMERA, cam_index);
    // inc?
    cv::Mat trans_cur_w = MultiCamExt::GetInstance().GetTargetCamPose(
        T21, FRONT_CAMERA, cam_index);

    Mat44_t tcw_ref = Converter::toMatrix4d(trans_ref_w);
    Mat44_t tcw_cur = Converter::toMatrix4d(trans_cur_w);

    auto keypoints2 = current_frame.frames_[cam_index].keypts_;

    vector<Vec3_t> bearings2;
    for (size_t i = 0; i < keypoints2.size(); i++) {
      Vec3_t bearing = Converter::KeypointToBearing(keypoints2[i]);
      bearings2.push_back(bearing);
    }
    float frame_parallax_deg = 0;

    unsigned int frame_num_valid_pts =
        CheckPose(tcw_ref, tcw_cur, keypoints1_[cam_index], keypoints2,
                  bearings1_[cam_index], bearings2, matches12[cam_index],
                  init_mappoints[cam_index], is_triangulated[cam_index],
                  frame_parallax_deg);

    // Only use front camera to check
    if (cam_index == FRONT_CAMERA) {
      num_valid_pts = frame_num_valid_pts;
      parallax_deg = frame_parallax_deg;
    }

    std::cout << kColorYellow << " Cam " << cam_index
              << " parallax deg (50th smallest) :" << frame_parallax_deg
              << " num of valid points :" << frame_num_valid_pts << std::endl;
  }

  // check valid points
  unsigned int min_num_triangulated_ = 100;

  if (num_valid_pts < min_num_triangulated_)
    return false;

  if (parallax_deg < parallax_deg_thr_)
    return false;

  return true;
}

unsigned int Initializer::CheckPose(const Mat44_t &ref_pose,
                                    const Mat44_t &cur_pose,
                                    const std::vector<cv::KeyPoint> &keypoints1,
                                    const std::vector<cv::KeyPoint> &keypoints2,
                                    const std::vector<Vec3_t> &bearings1,
                                    const std::vector<Vec3_t> &bearings2,
                                    const std::vector<int> &matches12,
                                    std::vector<cv::Point3f> &triangulated_pts,
                                    std::vector<bool> &is_triangulated,
                                    float &parallax_deg) {

  // basic threshold
  constexpr float cos_parallax_thr = 0.99996192306;
  bool depth_is_positive = true;

  const float reproj_err_thr_sq = reproj_err_thr_ * reproj_err_thr_;

  std::vector<float> cos_parallaxes;
  cos_parallaxes.reserve(keypoints2.size());

  is_triangulated.resize(keypoints2.size(), false);
  triangulated_pts.resize(keypoints2.size());

  Mat44_t ref_pos_w1 = ref_pose.inverse();
  Mat44_t cur_pos_w1 = cur_pose.inverse();

  Vec3_t ref_cam_center = ref_pos_w1.block(0, 3, 3, 1);
  Vec3_t cur_cam_center = cur_pos_w1.block(0, 3, 3, 1);

  Mat44_t pose_ref_to_cur = cur_pose * ref_pos_w1;

  Mat33_t rot_ref_to_cur = pose_ref_to_cur.block(0, 0, 3, 3);
  Vec3_t trans_ref_to_cur = pose_ref_to_cur.block(0, 3, 3, 1);

  int num_valid_pts = 0;

  for (size_t i = 0; i < matches12.size(); i++) {
    if (matches12[i] < 0)
      continue;

    int idx1 = i;
    int idx2 = matches12[i];
    // generate bearings

    Vec3_t ref_bearing = bearings1[idx1];
    ref_bearing.normalize();

    Vec3_t cur_bearing = bearings2[idx2];
    cur_bearing.normalized();

    const Vec3_t pos_c_in_ref = Triangulator::Triangulate(
        ref_bearing, cur_bearing, rot_ref_to_cur, trans_ref_to_cur);

    if (!std::isfinite(pos_c_in_ref(0)) || !std::isfinite(pos_c_in_ref(1)) ||
        !std::isfinite(pos_c_in_ref(2))) {
      continue;
    }

    // compute a parallax
    const Vec3_t ref_normal = pos_c_in_ref - ref_cam_center;
    const float ref_norm = ref_normal.norm();
    const Vec3_t cur_normal = pos_c_in_ref - cur_cam_center;
    const float cur_norm = cur_normal.norm();
    const float cos_parallax =
        ref_normal.dot(cur_normal) / (ref_norm * cur_norm);

    const bool parallax_is_small = cos_parallax_thr < cos_parallax;

    // reject if the 3D point is in front of the cameras
    if (depth_is_positive) {
      if (!parallax_is_small && pos_c_in_ref(2) <= 0) {
        continue;
      }
      const Vec3_t pos_c_in_cur =
          rot_ref_to_cur * pos_c_in_ref + trans_ref_to_cur;
      if (!parallax_is_small && pos_c_in_cur(2) <= 0) {
        continue;
      }
    }

    Vec2_t ref_undist_keypt(keypoints1[idx1].pt.x, keypoints1[idx1].pt.y);
    Vec2_t cur_undist_keypt(keypoints2[idx2].pt.x, keypoints2[idx2].pt.y);

    // compute a reprojection error in the reference
    Vec2_t reproj_in_ref;
    const auto is_valid_ref = Converter::ReprojectToImage(
        Mat33_t::Identity(), Vec3_t::Zero(), pos_c_in_ref, reproj_in_ref);
    if (!parallax_is_small && !is_valid_ref) {
      continue;
    }

    const float ref_reproj_err_sq =
        (reproj_in_ref - ref_undist_keypt).squaredNorm();
    if (reproj_err_thr_sq < ref_reproj_err_sq) {
      continue;
    }

    // compute a reprojection error in the current
    Vec2_t reproj_in_cur;
    const auto is_valid_cur = Converter::ReprojectToImage(
        rot_ref_to_cur, trans_ref_to_cur, pos_c_in_ref, reproj_in_cur);
    if (!parallax_is_small && !is_valid_cur) {
      continue;
    }
    const float cur_reproj_err_sq =
        (reproj_in_cur - cur_undist_keypt).squaredNorm();
    if (reproj_err_thr_sq < cur_reproj_err_sq) {
      continue;
    }

    // triangulation is valid!
    ++num_valid_pts;
    cos_parallaxes.push_back(cos_parallax);
    if (!parallax_is_small) {

      cv::Point3f pw(pos_c_in_ref.x(), pos_c_in_ref.y(), pos_c_in_ref.z());
      triangulated_pts[i] = pw;
      is_triangulated[i] = true;
    }
  }

  if (0 < num_valid_pts) {
    // return the 50th smallest parallax
    std::sort(cos_parallaxes.begin(), cos_parallaxes.end());
    const auto idx = std::min(50, static_cast<int>(cos_parallaxes.size() - 1));
    parallax_deg = std::acos(cos_parallaxes.at(idx)) * 180.0 / M_PI;
  } else {
    parallax_deg = 0.0;
  }
  return num_valid_pts;
}

bool Initializer::IsVehicleMoved(const Mat44_t cur_odom) {
  // check the vehicle is moving or not
  double odom_distance = hypot(ref_odom_(0, 3) - cur_odom(0, 3), ref_odom_(1, 3) - cur_odom(1, 3));

  if (odom_distance < 0.3) {
    return false;
  }
  return true;
}
}
