#include "transform_ipm.h"

namespace tf_ipm {

void TransformGround2Image(const EigenMat &mat_ground,
                           const EigenMat &mat_tf_ground2uv,
                           const float ground_height, EigenMat &mat_uv) {

  // add two rows to the input points
  // EigenMat mat_ground3 = EigenMat(mat_ground.rows+1, mat_ground.cols,
  // mat_ground.type());
  EigenMat mat_ground3(mat_ground.rows() + 1, mat_ground.cols());

  mat_ground3.block(0, 0, mat_ground.rows(), mat_ground.cols()) = mat_ground;
  mat_ground3.row(2) = EigenMat::Constant(1, mat_ground.cols(), ground_height);

  mat_ground3 = mat_tf_ground2uv * mat_ground3; // (3,3) x (3,n) = (3,n)

  // divide by last row of mat_ground4
  for (int i = 0; i < mat_ground.cols(); i++) {
    float div = mat_ground3(2, i);
    mat_ground3(0, i) = mat_ground3(0, i) / div;
    mat_ground3(1, i) = mat_ground3(1, i) / div;
  }

  // put back the result into mat_uv
  mat_uv = mat_ground3.block(0, 0, 2, mat_ground.cols());
}

void TransformImage2Ground(const EigenMat &mat_uv,
                           const EigenMat &mat_tf_uv2ground,
                           EigenMat &mat_ground) {

  EigenMat mat_uv3(mat_uv.rows() + 1, mat_uv.cols());

  mat_uv3.block(0, 0, 2, mat_uv.cols()) = mat_uv;
  mat_uv3.row(2) = EigenMat::Constant(1, mat_uv.cols(), 1.0);

  // create the transformation matrix

  // mat_uv4 = mat_tf_uv2ground * mat_uv3; // (4,3) x (3,n) = (4,n)

  EigenMat mat_uv4 = mat_tf_uv2ground * mat_uv3;

  // divide by last row of mat_uv4
  for (int i = 0; i < mat_uv4.cols(); i++) {
    float div = mat_uv4(3, i);
    mat_uv4(0, i) = mat_uv4(0, i) / div;
    mat_uv4(1, i) = mat_uv4(1, i) / div;
  }

  // put back the result into mat_ground
  // mat_uv2.copyTo(mat_ground);
  mat_ground = mat_uv4.block(0, 0, 2, mat_uv4.cols());
}

Point2f GetVanishingPoint(const CameraInfo &camera_info) {
  // get the mat_vp in world coordinates
  float pts_vp[] = {sinf(camera_info.angle_yaw) / cosf(camera_info.angle_pitch),
                    cosf(camera_info.angle_yaw) / cosf(camera_info.angle_pitch),
                    0.0f};
  // EigenMat mat_vp = EigenMat(3, 1, CV_32F, pts_vp);
  Eigen::Map<EigenMat> mat_vp(pts_vp, 3, 1);

  // transform from world to camera coordinates
  // rotation matrix for yaw
  float pts_yaw[] = {cosf(camera_info.angle_yaw),
                     -sinf(camera_info.angle_yaw),
                     0.0f,
                     sinf(camera_info.angle_yaw),
                     cosf(camera_info.angle_yaw),
                     0.0f,
                     0.0f,
                     0.0f,
                     1.0f};
  // EigenMat mat_tf_yaw = EigenMat(3, 3, CV_32F, pts_yaw);
  Eigen::Map<EigenMat> mat_tf_yaw(pts_yaw, 3, 3);
  // rotation matrix for pitch
  float pts_pitch[] = {1.0f,
                       0.0f,
                       0.0f,
                       0.0f,
                       -sinf(camera_info.angle_pitch),
                       -cosf(camera_info.angle_pitch),
                       0.0f,
                       cosf(camera_info.angle_pitch),
                       -sinf(camera_info.angle_pitch)};
  // EigenMat mat_transform = EigenMat(3, 3, CV_32F, pts_pitch);
  Eigen::Map<EigenMat> mat_transform(pts_pitch, 3, 3);

  // combined transform
  mat_transform = mat_transform * mat_tf_yaw;

  // transformation from (xc, yc) in camra coordinates
  // to (u,v) in image frame
  // matrix to shift optical center and focal length
  float pts_tf_camera[] = {camera_info.focal_length.x,
                           0.0f,
                           camera_info.optical_center.x,
                           0.0f,
                           camera_info.focal_length.y,
                           camera_info.optical_center.y,
                           0.0f,
                           0.0f,
                           1.0f};
  // EigenMat mat_ft_camera = EigenMat(3, 3, CV_32F, pts_tf_camera);
  Eigen::Map<EigenMat> mat_ft_camera(pts_tf_camera, 3, 3);

  // add scale and cut
  float pts_resize_cut_pts[] = {camera_info.scale_x,
                                0.0f,
                                -camera_info.cut_x,
                                0.0f,
                                camera_info.scale_y,
                                -camera_info.cut_y,
                                0.0f,
                                0.0f,
                                1.0f};
  // EigenMat mat_tf_resize_cut = EigenMat(3, 3, CV_32F, pts_resize_cut_pts);
  Eigen::Map<EigenMat> mat_tf_resize_cut(pts_resize_cut_pts, 3, 3);

  EigenMat mat_tf_camera_resize_cut = mat_tf_resize_cut * mat_ft_camera;

  // combine transform
  // mat_transform = mat_ft_camera * mat_transform;
  mat_transform = mat_tf_camera_resize_cut * mat_transform;

  // transform
  mat_vp = mat_transform * mat_vp;

  // clean and return
  Point2f ret;
  ret.x = mat_vp(0, 0);
  ret.y = mat_vp(1, 0);

  return ret;
}

Point2f GetVanishingPointOpt(const CameraInfo &camera_info) {
  // get the mat_vp in world coordinates
  float pts_vp[] = {0, 1, 0};
  // EigenMat mat_vp = EigenMat(3, 1, CV_32F, pts_vp);
  Eigen::Map<EigenMat> mat_vp(pts_vp, 3, 1);

  float pts_yaw[] = {cosf(camera_info.angle_yaw),
                     -sinf(camera_info.angle_yaw),
                     0.0f,
                     sinf(camera_info.angle_yaw),
                     cosf(camera_info.angle_yaw),
                     0.0f,
                     0.0f,
                     0.0f,
                     1.0f};
  Eigen::Map<EigenMat> mat_tf_yaw_z(pts_yaw, 3, 3);

  // rotation matrix for pitch
  float pts_pitch[] = {1.0f,
                       0.0f,
                       0.0f,
                       0.0f,
                       cosf(camera_info.angle_pitch),
                       -sinf(camera_info.angle_pitch),
                       0.0f,
                       sinf(camera_info.angle_pitch),
                       cosf(camera_info.angle_pitch)};
  Eigen::Map<EigenMat> mat_tf_pitch_x(pts_pitch, 3, 3);

  float pts_roll[] = {cosf(camera_info.angle_roll),
                      0.0f,
                      sinf(camera_info.angle_roll),
                      0.0f,
                      1.0f,
                      0.0f,
                      -sinf(camera_info.angle_roll),
                      0.0f,
                      cosf(camera_info.angle_roll)};
  Eigen::Map<EigenMat> mat_tf_roll_y(pts_roll, 3, 3);

  EigenMat mat_transform_yz = mat_tf_roll_y * mat_tf_yaw_z;

  // combined transform
  EigenMat mat_transform = mat_tf_pitch_x * mat_transform_yz;

  // combined transform
  // EigenMat mat_transform = mat_tf_pitch_x * mat_tf_yaw_z;

  float pts_r1[] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f};
  // EigenMat mat_r1 = EigenMat(3, 3, CV_32F, pts_r1);
  Eigen::Map<EigenMat> mat_r1(pts_r1, 3, 3);

  mat_transform = mat_r1 * mat_transform;

  // transformation from (xc, yc) in camra coordinates
  // to (u,v) in image frame
  // matrix to shift optical center and focal length
  float pts_tf_camera[] = {camera_info.focal_length.x,
                           0.0f,
                           camera_info.optical_center.x,
                           0.0f,
                           camera_info.focal_length.y,
                           camera_info.optical_center.y,
                           0.0f,
                           0.0f,
                           1.0f};
  // EigenMat mat_tf_camera = EigenMat(3, 3, CV_32F, pts_tf_camera);
  Eigen::Map<EigenMat> mat_tf_camera(pts_tf_camera, 3, 3);

  float pts_resize_cut_pts[] = {camera_info.scale_x,
                                0.0f,
                                -camera_info.cut_x,
                                0.0f,
                                camera_info.scale_y,
                                -camera_info.cut_y,
                                0.0f,
                                0.0f,
                                1.0f};
  // EigenMat mat_tf_resize_cut = EigenMat(3, 3, CV_32F, pts_resize_cut_pts);
  Eigen::Map<EigenMat> mat_tf_resize_cut(pts_resize_cut_pts, 3, 3);

  EigenMat mat_tf_camera_resize_cut = mat_tf_resize_cut * mat_tf_camera;
  // combine transform
  mat_transform = mat_tf_camera_resize_cut * mat_transform;

  // transform
  mat_vp = mat_transform * mat_vp;

  // clean and return
  Point2f ret;
  ret.x = mat_vp(0, 0) / mat_vp(2, 0);
  ret.y = mat_vp(1, 0) / mat_vp(2, 0);

  return ret;
}

void TransformImIPM2Ground(const EigenMat &mat_ipm, EigenMat &mat_ground,
                           const IPMInfo &ipm_info) {

  mat_ground = mat_ipm;

  mat_ground.row(0) = mat_ground.row(0).array() * 1.0f / ipm_info.scale_x +
                      ipm_info.limits_x[0];
  mat_ground.row(1) = mat_ground.row(1).array() * (-1.0f / ipm_info.scale_y) +
                      ipm_info.limits_y[1];

  /*
  //work on the x-direction i.e. first row
  EigenMat row = mat_ground.row(0);
  row.convertTo(row, CV_32F, 1./ipm_info.scale_x, ipm_info.limits_x[0]);

  //work on y-direction
  row = mat_ground.row(1);
  row.convertTo(row, CV_32F, -1./ipm_info.scale_y, ipm_info.limits_y[1]);
  */
}

void TransformGround2ImIPM(const EigenMat &mat_ground, EigenMat &mat_ipm,
                           const IPMInfo &ipm_info) {

  mat_ipm = mat_ground;
  mat_ipm.row(0) = mat_ipm.row(0).array() * ipm_info.scale_x -
                   1.0f * ipm_info.limits_x[0] * ipm_info.scale_x;
  mat_ipm.row(1) = mat_ipm.row(1).array() * (-1.0f * ipm_info.scale_y) +
                   ipm_info.limits_y[1] * ipm_info.scale_y;

  /*
  //work on the x-direction i.e. first row
  EigenMat row = mat_ipm.row(0);
  row.convertTo(row, CV_32F, ipm_info.scale_x, -1.0 * ipm_info.limits_x[0] *
  ipm_info.scale_x);

  //work on y-direction
  row = mat_ipm.row(1);
  row.convertTo(row, CV_32F, -1.0 * ipm_info.scale_y, ipm_info.limits_y[1] *
  ipm_info.scale_y);
  */
}

EigenMat PrepareTfGround2UV(const CameraInfo &camera_info_) {
  // transform from world to camera coordinates
  // rotation matrix for yaw
  float pts_yaw[] = {cosf(camera_info_.angle_yaw),
                     -sinf(camera_info_.angle_yaw),
                     0.0f,
                     sinf(camera_info_.angle_yaw),
                     cosf(camera_info_.angle_yaw),
                     0.0f,
                     0.0f,
                     0.0f,
                     1.0f};
  // EigenMat mat_tf_yaw_z = EigenMat(3, 3, CV_32F, pts_yaw);
  Eigen::Map<EigenMat> mat_tf_yaw_z(pts_yaw, 3, 3);
  // rotation matrix for pitch
  float pts_pitch[] = {1.0f,
                       0.0f,
                       0.0f,
                       0.0f,
                       cosf(camera_info_.angle_pitch),
                       -sinf(camera_info_.angle_pitch),
                       0.0f,
                       sinf(camera_info_.angle_pitch),
                       cosf(camera_info_.angle_pitch)};

  // EigenMat mat_tf_pitch_x = EigenMat(3, 3, CV_32F, pts_pitch);
  Eigen::Map<EigenMat> mat_tf_pitch_x(pts_pitch, 3, 3);
  // EigenMat mat_transform = EigenMat(3, 3, CV_32F, pts_pitch);

  float pts_roll[] = {cosf(camera_info_.angle_roll),
                      0.0f,
                      sinf(camera_info_.angle_roll),
                      0.0f,
                      1.0f,
                      0.0f,
                      -sinf(camera_info_.angle_roll),
                      0.0f,
                      cosf(camera_info_.angle_roll)};

  // EigenMat mat_tf_pitch_x = EigenMat(3, 3, CV_32F, pts_pitch);
  Eigen::Map<EigenMat> mat_tf_roll_y(pts_roll, 3, 3);

  EigenMat mat_transform_yz = mat_tf_roll_y * mat_tf_yaw_z;

  // combined transform
  // EigenMat mat_transform = mat_tf_pitch_x * mat_tf_yaw_z;
  EigenMat mat_transform = mat_tf_pitch_x * mat_transform_yz;

  float pts_r1[] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f};
  // EigenMat mat_r1 = EigenMat(3, 3, CV_32F, pts_r1);
  Eigen::Map<EigenMat> mat_r1(pts_r1, 3, 3);

  mat_transform = mat_r1 * mat_transform;

  // transformation from (xc, yc) in camra coordinates
  // to (u,v) in image frame
  // matrix to shift optical center and focal length
  float pts_tf_camera[] = {camera_info_.focal_length.x,
                           0.0f,
                           camera_info_.optical_center.x,
                           0.0f,
                           camera_info_.focal_length.y,
                           camera_info_.optical_center.y,
                           0.0f,
                           0.0f,
                           1.0f};
  // EigenMat mat_tf_camera = EigenMat(3, 3, CV_32F, pts_tf_camera);
  Eigen::Map<EigenMat> mat_tf_camera(pts_tf_camera, 3, 3);

  float pts_resize_cut_pts[] = {camera_info_.scale_x,
                                0.0f,
                                -camera_info_.cut_x,
                                0.0f,
                                camera_info_.scale_y,
                                -camera_info_.cut_y,
                                0.0f,
                                0.0f,
                                1.0f};
  // EigenMat mat_tf_resize_cut = EigenMat(3, 3, CV_32F, pts_resize_cut_pts);
  Eigen::Map<EigenMat> mat_tf_resize_cut(pts_resize_cut_pts, 3, 3);

  EigenMat mat_tf_camera_resize_cut = mat_tf_resize_cut * mat_tf_camera;
  // combine transform
  EigenMat mat_tf_ground2uv_ = mat_tf_camera_resize_cut * mat_transform;
  return mat_tf_ground2uv_;
}

EigenMat PrepareTfUV2Ground(const CameraInfo &camera_info_) {

  // rotation matrix for yaw (rotation along z-axis)
  float pts_yaw_z_inv[] = {cosf(camera_info_.angle_yaw),
                           sinf(camera_info_.angle_yaw),
                           0.0f,
                           -sinf(camera_info_.angle_yaw),
                           cosf(camera_info_.angle_yaw),
                           0.0f,
                           0.0f,
                           0.0f,
                           1.0f};
  Eigen::Map<EigenMat> mat_tf_yaw_z_inv(pts_yaw_z_inv, 3, 3);

  // rotation matrix for pitch (rotation along x-axis)
  float pts_pitch_x_inv[] = {1.0f,
                             0.0f,
                             0.0f,
                             0.0f,
                             cosf(camera_info_.angle_pitch),
                             sinf(camera_info_.angle_pitch),
                             0.0f,
                             -sinf(camera_info_.angle_pitch),
                             cosf(camera_info_.angle_pitch)};
  Eigen::Map<EigenMat> mat_tf_pitch_x_inv(pts_pitch_x_inv, 3, 3);

  // rotation matrix for roll (rotation along y-axis)
  float pts_roll_y_inv[] = {cosf(camera_info_.angle_roll),
                            0.0f,
                            -sinf(camera_info_.angle_roll),
                            0.0f,
                            1.0f,
                            0.0f,
                            sinf(camera_info_.angle_roll),
                            0.0f,
                            cosf(camera_info_.angle_roll)};
  Eigen::Map<EigenMat> mat_tf_roll_y_inv(pts_roll_y_inv, 3, 3);

  EigenMat mat_transform_zy =
      mat_tf_yaw_z_inv * mat_tf_roll_y_inv; // mat_tf_pitch_x_inv;

  // combined transform
  // EigenMat mat_transform = mat_tf_yaw_z_inv * mat_tf_pitch_x_inv;
  EigenMat mat_transform = mat_transform_zy * mat_tf_pitch_x_inv;

  float pts_r1_inv[] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f};
  // EigenMat mat_r1_inv = EigenMat(3, 3, CV_32F, pts_r1_inv);
  Eigen::Map<EigenMat> mat_r1_inv(pts_r1_inv, 3, 3);

  mat_transform = mat_transform * mat_r1_inv;

  // transformation from (xc, yc) in camra coordinates
  // to (u,v) in image frame
  // matrix to shift optical center and focal length
  float pts_tf_camera_inv[] = {
      1.0f / camera_info_.focal_length.x,
      0.0f,
      -camera_info_.optical_center.x / camera_info_.focal_length.x,
      0.0f,
      1.0f / camera_info_.focal_length.y,
      -camera_info_.optical_center.y / camera_info_.focal_length.y,
      0.0f,
      0.0f,
      1.0f};
  // EigenMat mat_ft_camera_inv = EigenMat(3, 3, CV_32F, pts_tf_camera_inv);
  Eigen::Map<EigenMat> mat_ft_camera_inv(pts_tf_camera_inv, 3, 3);

  float pts_resize_cut_pts_inv[] = {1.0f / camera_info_.scale_x,
                                    0.0f,
                                    camera_info_.cut_x / camera_info_.scale_x,
                                    0.0f,
                                    1.0f / camera_info_.scale_y,
                                    camera_info_.cut_y / camera_info_.scale_y,
                                    0.0f,
                                    0.0f,
                                    1.0f};
  // EigenMat mat_tf_resize_cut_inv = EigenMat(3, 3, CV_32F,
  // pts_resize_cut_pts_inv);
  Eigen::Map<EigenMat> mat_tf_resize_cut_inv(pts_resize_cut_pts_inv, 3, 3);

  EigenMat mat_tf_camera_resize_cut_inv =
      mat_ft_camera_inv * mat_tf_resize_cut_inv;

  // combine transform
  // EigenMat mat_tf_uv_ground_3x3 = mat_transform * mat_ft_camera_inv;
  EigenMat mat_tf_uv_ground_3x3 = mat_transform * mat_tf_camera_resize_cut_inv;

  EigenMat mat_tf_uv2ground_ = EigenMat(4, 3);
  mat_tf_uv2ground_.block<3, 3>(0, 0) = mat_tf_uv_ground_3x3;
  mat_tf_uv2ground_.row(3) = mat_tf_uv_ground_3x3.row(2);
  mat_tf_uv2ground_.row(3) /= -camera_info_.camera_height;
  return mat_tf_uv2ground_;
}

} // namespace tf_ipm
