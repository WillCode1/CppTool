#include "utils/cv_utils.h"
#include "utils/eigen_cv_utils.h"

#include "verify_extrinsic.h"

namespace nullmax_calibration {

void ShowMat(const EigenMat &mat, const std::string &str) {
  std::cout << str << std::endl << mat << std::endl;
}

void VerifyCalibration::Init(const std::string &str_camera_info,
                             const std::string &str_ipm_info) {
  std::cout << "loadCameraInfo" << std::endl;
  loadCameraInfo(str_camera_info);

  std::cout << "prepareIpmInfo" << std::endl;
  loadIpmInfo(str_ipm_info);

  std::cout << "prepareTf" << std::endl;
  prepareTf();

  std::cout << "updateIpm" << std::endl;
  updateIpm();

  std::cout << "after update" << std::endl;
  ipm_info_.PrintSelf();

  // prepareDistanceLimit();
}

void VerifyCalibration::Init(
    const nullmax_perception::CameraIntrinsic &intrinsic,
    const nullmax_perception::CameraDistortCoef &distort_coef,
    const float &camera_height, const int &image_width, const int &image_height,
    const nullmax_perception::CameraRotationEuler &rotation_angle,
    const nullmax_perception::UVIPMRoi &uv_ipm_roi) {
  std::cout << "loadCameraInfo" << std::endl;
  loadCameraInfo(camera_height, image_width, image_height, intrinsic,
                 distort_coef, rotation_angle);

  std::cout << "prepareIpmInfo" << std::endl;
  loadIpmInfo(uv_ipm_roi);

  std::cout << "prepareTf" << std::endl;
  prepareTf();

  std::cout << "updateIpm" << std::endl;
  updateIpm();

  std::cout << "after update" << std::endl;
  ipm_info_.PrintSelf();
}

void VerifyCalibration::AdjustPitch(bool is_add) {
  float step = 0.05 * CV_PI / 180.0f;
  if (is_add) {
    camera_info_.angle_pitch += step;
  } else {
    camera_info_.angle_pitch -= step;
  }
  prepareTf();
  updateIpm();
  printCameraAngle();
}

void VerifyCalibration::AdjustYaw(bool is_add) {
  float step = 0.05 * CV_PI / 180.0f;
  if (is_add) {
    camera_info_.angle_yaw += step;
  } else {
    camera_info_.angle_yaw -= step;
  }
  prepareTf();
  updateIpm();
  printCameraAngle();
}

void VerifyCalibration::AdjustRoll(bool is_add) {
  float step = 0.05 * CV_PI / 180.0f;
  if (is_add) {
    camera_info_.angle_roll += step;
  } else {
    camera_info_.angle_roll -= step;
  }
  prepareTf();
  updateIpm();
  printCameraAngle();
}

void VerifyCalibration::loadIpmInfo(const std::string &str_ipm_info) {

  // full size parameters
  // ipm_info_.roi_uv_left = 10;
  // ipm_info_.roi_uv_right = 1270;
  // ipm_info_.roi_uv_top = 500;
  // ipm_info_.roi_uv_bottom = 850;

  char str[1024];
  int roi_uv_left, roi_uv_right, roi_uv_top, roi_uv_bottom;

  FILE *pfile = fopen(str_ipm_info.c_str(), "rt");
  int err;
  if (pfile != NULL) {

    err = fscanf(pfile, "%s %d\n", str, &roi_uv_left);
    std::cout << "read roi_uv_left = " << roi_uv_left << std::endl;
    err = fscanf(pfile, "%s %d\n", str, &roi_uv_right);
    std::cout << "read roi_uv_right = " << roi_uv_right << std::endl;
    err = fscanf(pfile, "%s %d\n", str, &roi_uv_top);
    std::cout << "read roi_uv_top = " << roi_uv_top << std::endl;
    err = fscanf(pfile, "%s %d\n", str, &roi_uv_bottom);
    std::cout << "read roi_uv_bottom = " << roi_uv_bottom << std::endl;

    fclose(pfile);

  } else {
    std::cout << "cannot open file " << str_ipm_info << std::endl;
    exit(-1);
  }

  ipm_info_.roi_uv_left = roi_uv_left;
  ipm_info_.roi_uv_right = roi_uv_right;
  ipm_info_.roi_uv_top = roi_uv_top;
  ipm_info_.roi_uv_bottom = roi_uv_bottom;

  // set ipm info
  ipm_info_.width_ipm = 960;
  ipm_info_.height_ipm = 480;

  ipm_info_.vp_portion = 0.01;
  ipm_info_.ipm_interpolation = 0;

  ipm_info_.PrintSelf();
}

void VerifyCalibration::loadIpmInfo(
    const nullmax_perception::UVIPMRoi &uv_ipm_roi) {
  ipm_info_.roi_uv_left = uv_ipm_roi.left;
  ipm_info_.roi_uv_right = uv_ipm_roi.right;
  ipm_info_.roi_uv_top = uv_ipm_roi.top;
  ipm_info_.roi_uv_bottom = uv_ipm_roi.bottom;

  // set ipm info
  ipm_info_.width_ipm = 960;
  ipm_info_.height_ipm = 480;

  ipm_info_.vp_portion = 0.01;
  ipm_info_.ipm_interpolation = 0;

  ipm_info_.PrintSelf();
}

void VerifyCalibration::loadCameraInfo(const std::string &str_camera_info) {
  char str[1024];

  float focal_length_x, focal_length_y;
  float optical_center_x, optical_center_y;
  float camera_height;

  float angle_pitch, angle_yaw, angle_roll;
  int image_width = 1920, image_height = 1208;

  float r1, r2, t1, t2;

  FILE *pfile = fopen(str_camera_info.c_str(), "rt");
  if (pfile != NULL) {
    int err;
    err = fscanf(pfile, "%s %f\n", str, &focal_length_x);
    std::cout << "read focal_length_x = " << focal_length_x << std::endl;
    err = fscanf(pfile, "%s %f\n", str, &focal_length_y);
    std::cout << "read focal_length_y = " << focal_length_y << std::endl;
    err = fscanf(pfile, "%s %f\n", str, &optical_center_x);
    std::cout << "read optical_center_x = " << optical_center_x << std::endl;
    err = fscanf(pfile, "%s %f\n", str, &optical_center_y);
    std::cout << "read optical_center_y = " << optical_center_y << std::endl;

    err = fscanf(pfile, "%s %f\n", str, &camera_height);
    std::cout << "read camera_height = " << camera_height << std::endl;

    err = fscanf(pfile, "%s %f %f\n", str, &r1, &r2);
    err = fscanf(pfile, "%s %f %f\n", str, &t1, &t2);

    err = fscanf(pfile, "%s %d\n", str, &image_width);
    std::cout << "read image_width = " << image_width << std::endl;
    err = fscanf(pfile, "%s %d\n", str, &image_height);
    std::cout << "read image_height = " << image_height << std::endl;

    err = fscanf(pfile, "%s %f\n", str, &angle_pitch);
    std::cout << "read angle_pitch = " << angle_pitch << std::endl;
    err = fscanf(pfile, "%s %f\n", str, &angle_yaw);
    std::cout << "read angle_yaw = " << angle_yaw << std::endl;
    err = fscanf(pfile, "%s %f\n", str, &angle_roll);
    std::cout << "read angle_roll = " << angle_roll << std::endl;

    fclose(pfile);

  } else {
    std::cout << "cannot open file " << str_camera_info << std::endl;
    exit(-1);
  }

  int cut_top = 0;

  // set camera info
  camera_info_.focal_length.x = focal_length_x;
  camera_info_.focal_length.y = focal_length_y;
  camera_info_.optical_center.x = optical_center_x;
  camera_info_.optical_center.y = optical_center_y;

  camera_info_.camera_height = camera_height;
  camera_info_.angle_pitch = angle_pitch * CV_PI / 180.0;
  camera_info_.angle_yaw = angle_yaw * CV_PI / 180.0;
  camera_info_.angle_roll = angle_roll * CV_PI / 180.0;

  camera_info_.width_image = image_width;
  camera_info_.height_image = image_height;

  camera_info_.scale_x = 1.0;
  camera_info_.scale_y = 1.0;
  camera_info_.cut_x = 0;
  camera_info_.cut_y = cut_top;

  camera_info_.PrintSelf();
}

void VerifyCalibration::loadCameraInfo(
    const float &camera_height, const int &image_width, const int &image_height,
    const nullmax_perception::CameraIntrinsic &intrinsic,
    const nullmax_perception::CameraDistortCoef &distort_coef,
    const nullmax_perception::CameraRotationEuler &rotation_angle) {
  int cut_top = 0;

  // set camera info
  camera_info_.focal_length.x = intrinsic.fx;
  camera_info_.focal_length.y = intrinsic.fy;
  camera_info_.optical_center.x = intrinsic.cx;
  camera_info_.optical_center.y = intrinsic.cy;

  camera_info_.camera_height = camera_height;
  camera_info_.angle_pitch = rotation_angle.pitch * CV_PI / 180.0;
  camera_info_.angle_yaw = rotation_angle.yaw * CV_PI / 180.0;
  camera_info_.angle_roll = rotation_angle.roll * CV_PI / 180.0;

  camera_info_.width_image = image_width;
  camera_info_.height_image = image_height;

  camera_info_.scale_x = 1.0;
  camera_info_.scale_y = 1.0;
  camera_info_.cut_x = 0;
  camera_info_.cut_y = cut_top;

  camera_info_.PrintSelf();
}

void VerifyCalibration::Verify(const Mat &image_uv_color,
                               const EigenMat &image_uv,
                               vector<float> dist_verticals,
                               const int &window_width,
                               const int &window_height,
                               cv::Mat &image_in_out) {
  image_uv_ = image_uv;
  image_ipm_ = EigenMat((int)ipm_info_.height_ipm, (int)ipm_info_.width_ipm);
  ComputeIpm(image_uv_, image_ipm_);
  // draw lines on image_uv
  Mat image_uv_show;
  image_uv_color_ = image_uv_color.clone();
  image_uv_show = image_uv_color_.clone();

  Mat image_ipm_show;
  Convert8U3C(image_ipm_, image_ipm_show);

  float width_horizontal = 1.0;

  int num_dist_vertical = dist_verticals.size();
  EigenMat mat_pts_ground_vertical(2, num_dist_vertical);
  for (int i = 0; i < num_dist_vertical; i++) {
    mat_pts_ground_vertical(0, i) = 0.0;
    mat_pts_ground_vertical(1, i) = dist_verticals[i] * 1000.0;
  }

  EigenMat mat_pts_uv_vertical;
  transformGround2Image(mat_pts_ground_vertical, mat_pts_uv_vertical);

  for (int i = 0; i < num_dist_vertical - 1; i++) {
    int x0 = (int)(mat_pts_uv_vertical(0, i) + 0.5);
    int y0 = (int)(mat_pts_uv_vertical(1, i) + 0.5);
    cv::Point pt0(x0, y0);

    int x1 = (int)(mat_pts_uv_vertical(0, i + 1) + 0.5);
    int y1 = (int)(mat_pts_uv_vertical(1, i + 1) + 0.5);
    cv::Point pt1(x1, y1);

    line(image_uv_show, pt0, pt1, Scalar(0, 0, 255), 1);
  }

  EigenMat mat_pts_ipm_vertical;
  transformGround2ImIPM(mat_pts_ground_vertical, mat_pts_ipm_vertical);
  for (int i = 0; i < num_dist_vertical - 1; i++) {
    int x0 = (int)(mat_pts_ipm_vertical(0, i) + 0.5);
    int y0 = (int)(mat_pts_ipm_vertical(1, i) + 0.5);
    cv::Point pt0(x0, y0);

    int x1 = (int)(mat_pts_ipm_vertical(0, i + 1) + 0.5);
    int y1 = (int)(mat_pts_ipm_vertical(1, i + 1) + 0.5);
    cv::Point pt1(x1, y1);

    line(image_ipm_show, pt0, pt1, Scalar(0, 0, 255), 1);
  }

  // draw h lines
  EigenMat mat_pts_ground_horizontal_left(2, num_dist_vertical);
  EigenMat mat_pts_ground_horizontal_right(2, num_dist_vertical);

  for (int i = 0; i < num_dist_vertical; i++) {
    mat_pts_ground_horizontal_left(0, i) = -2.0 * width_horizontal * 1000.0;
    mat_pts_ground_horizontal_left(1, i) = dist_verticals[i] * 1000.0;

    mat_pts_ground_horizontal_right(0, i) = 2.0 * width_horizontal * 1000.0;
    mat_pts_ground_horizontal_right(1, i) = dist_verticals[i] * 1000.0;
    // dist_vertical_i += step_vertical;
  }

  EigenMat mat_pts_uv_horizontal_left;
  EigenMat mat_pts_uv_horizontal_right;
  transformGround2Image(mat_pts_ground_horizontal_left,
                        mat_pts_uv_horizontal_left);
  transformGround2Image(mat_pts_ground_horizontal_right,
                        mat_pts_uv_horizontal_right);

  for (int i = 0; i < num_dist_vertical; i++) {
    int x0 = (int)(mat_pts_uv_horizontal_left(0, i) + 0.5);
    int y0 = (int)(mat_pts_uv_horizontal_left(1, i) + 0.5);
    cv::Point pt0(x0, y0);

    int x1 = (int)(mat_pts_uv_horizontal_right(0, i) + 0.5);
    int y1 = (int)(mat_pts_uv_horizontal_right(1, i) + 0.5);
    cv::Point pt1(x1, y1);

    line(image_uv_show, pt0, pt1, Scalar(0, 255, 0), 1);

    // print dist on screen
    std::stringstream ss;
    ss << dist_verticals[i] << " m";
    std::string str_dist = ss.str();
    int font_face = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double font_scale = 0.25;

    cv::Point pt_text;
    if (i % 2 == 0) {
      pt_text.x = x0 - 30;
      pt_text.y = y0;
    } else {
      pt_text.x = x1 + 10;
      pt_text.y = y0;
    }
    putText(image_uv_show, str_dist, pt_text, font_face, font_scale,
            Scalar(0, 255, 255), 1);
  }

  EigenMat mat_pts_ipm_horizontal_left;
  EigenMat mat_pts_ipm_horizontal_right;
  transformGround2ImIPM(mat_pts_ground_horizontal_left,
                        mat_pts_ipm_horizontal_left);
  transformGround2ImIPM(mat_pts_ground_horizontal_right,
                        mat_pts_ipm_horizontal_right);

  for (int i = 0; i < num_dist_vertical; i++) {
    int x0 = (int)(mat_pts_ipm_horizontal_left(0, i) + 0.5);
    int y0 = (int)(mat_pts_ipm_horizontal_left(1, i) + 0.5);
    cv::Point pt0(x0, y0);

    int x1 = (int)(mat_pts_ipm_horizontal_right(0, i) + 0.5);
    int y1 = (int)(mat_pts_ipm_horizontal_right(1, i) + 0.5);
    cv::Point pt1(x1, y1);

    line(image_ipm_show, pt0, pt1, Scalar(0, 255, 0), 1);

    // print dist on screen
    std::stringstream ss;
    ss << dist_verticals[i] << " m";
    std::string str_dist = ss.str();
    int font_face = FONT_HERSHEY_SCRIPT_SIMPLEX;
    double font_scale = 0.25;

    cv::Point pt_text;
    if (i % 2 == 0) {
      pt_text.x = x0 - 30;
      pt_text.y = y0;
    } else {
      pt_text.x = x1 + 10;
      pt_text.y = y0;
    }
    putText(image_ipm_show, str_dist, pt_text, font_face, font_scale,
            Scalar(0, 255, 255), 1);
  }

  // draw the ROI to compute ipm
  cv::Point pt1;
  pt1.x = ipm_info_.roi_uv_left;
  pt1.y = ipm_info_.roi_uv_top;

  cv::Point pt2;
  pt2.x = ipm_info_.roi_uv_right;
  pt2.y = ipm_info_.roi_uv_bottom;

  cv::rectangle(image_uv_show, pt1, pt2, Scalar(0, 0, 255));

  std::string str_image_uv_save = "image_dist.png";
  std::string str_image_ipm_save = "image_ipm.png";
  SaveImage(image_uv_show, str_image_uv_save);
  SaveImage(image_ipm_show, str_image_ipm_save);
  int line_num = 24;
  for (int i = 1; i < line_num; ++i) {
    cv::line(
        image_ipm_show, cv::Point2f(image_ipm_show.cols * i / line_num, 0),
        cv::Point2f(image_ipm_show.cols * i / line_num, image_ipm_show.rows),
        cv::Scalar(214, 238, 247));
  }

  Mat image_uv_show_8uc3;
  Mat image_ipm_show_8uc3;
  Convert8U3C(image_uv_show, image_uv_show_8uc3, window_height, window_width);
  Convert8U3C(image_ipm_show, image_ipm_show_8uc3, window_height, window_width);
  hconcat(image_uv_show_8uc3, image_ipm_show_8uc3, image_in_out);
  // ShowImage(image_uv_show, image_ipm_show, "image-uv-ipm", 5);

  // ShowImage(image_uv_, image_ipm_, "image-uv-ipm", 0);
}

void VerifyCalibration::prepareTf() {
  prepareTfGround2UV();
  prepareTfUV2Ground();
}

void VerifyCalibration::prepareTfGround2UV() {
  mat_tf_ground2uv_ = tf_ipm::PrepareTfGround2UV(camera_info_);
}

void VerifyCalibration::prepareTfUV2Ground() {

  mat_tf_uv2ground_ = tf_ipm::PrepareTfUV2Ground(camera_info_);
  ShowMat(mat_tf_uv2ground_, "mat_tf_uv2ground_");
}

void VerifyCalibration::prepareDistanceLimit() {
  // after update ipm, compute the maximum distance
  Point2f pt_vp;
  pt_vp = getVanishingPoint();
  pt_vp.y = std::max<float>(0, pt_vp.y);

  std::cout << "vp " << pt_vp << std::endl;
  // float eps = ipm_info_.vp_portion * v;
  float eps = 0.0;
  int max_uv_y_ideal = (int)(pt_vp.y + eps);

  // add debug to set reasonable distance
  int height_uv = 960;
  int width_uv = 1280;
  int num_pts = height_uv - max_uv_y_ideal + 1;
  // EigenMat mat_pts_uv = EigenMat::Zero(2, num_pts);
  mat_pts_uv_ = EigenMat::Zero(2, num_pts);
  for (int i = 0; i < num_pts; i++) {
    mat_pts_uv_(0, i) = width_uv / 2;
    mat_pts_uv_(1, i) = height_uv - 1 - i;
  }

  // EigenMat mat_pts_ground;
  transformImage2Ground(mat_pts_uv_, mat_pts_ground_);
}

void VerifyCalibration::transformGround2Image(const EigenMat &mat_ground,
                                              EigenMat &mat_uv) {
  tf_ipm::TransformGround2Image(mat_ground, mat_tf_ground2uv_,
                                -1.0 * camera_info_.camera_height, mat_uv);
}

void VerifyCalibration::transformImage2Ground(const EigenMat &mat_uv,
                                              EigenMat &mat_ground) {
  tf_ipm::TransformImage2Ground(mat_uv, mat_tf_uv2ground_, mat_ground);
}

void VerifyCalibration::transformImIPM2Im(const EigenMat &mat_ipm,
                                          EigenMat &mat_uv) {
  // convert from im-ipm to (ground) world coordinates
  EigenMat mat_ground;
  tf_ipm::TransformImIPM2Ground(mat_ipm, mat_ground, ipm_info_);

  // convert from ground to image coordinates
  transformGround2Image(mat_ground, mat_uv);
}

void VerifyCalibration::transformIm2ImIPM(const EigenMat &mat_uv,
                                          EigenMat &mat_ipm) {
  // convert from image to (ground) world coordinates
  EigenMat mat_ground;
  transformImage2Ground(mat_uv, mat_ground);

  // convert from ground to im-ipm coordinates
  tf_ipm::TransformGround2ImIPM(mat_ground, mat_ipm, ipm_info_);
}

void VerifyCalibration::transformImIPM2Ground(const EigenMat &mat_ipm,
                                              EigenMat &mat_ground) {
  tf_ipm::TransformImIPM2Ground(mat_ipm, mat_ground, ipm_info_);
}

void VerifyCalibration::transformGround2ImIPM(const EigenMat &mat_ground,
                                              EigenMat &mat_ipm) {
  tf_ipm::TransformGround2ImIPM(mat_ground, mat_ipm, ipm_info_);
}

Point2f VerifyCalibration::getVanishingPoint() {
  // return tf_ipm::GetVanishingPoint(camera_info_);
  return tf_ipm::GetVanishingPointOpt(camera_info_);
}

void VerifyCalibration::printCameraAngle() {
  std::cout << "camera_pitch[deg]: "
            << camera_info_.angle_pitch * 180.0f / CV_PI << std::endl;
  std::cout << "camera_yaw[deg]: " << camera_info_.angle_yaw * 180.0f / CV_PI
            << std::endl;
  std::cout << "camera_rol[deg]: " << camera_info_.angle_roll * 180.0f / CV_PI
            << std::endl;
}

void VerifyCalibration::ComputeIpm(const EigenMat &image_uv,
                                   EigenMat &image_ipm) {

  int rows_ipm = ipm_info_.height_ipm;
  int cols_ipm = ipm_info_.width_ipm;

  // now loop and find the nearest pixel value for each position
  // that's inside the image, otherwise put it zero
  float ui, vi;
  // get mean of the input image
  // Scalar means = mean(image_uv);
  // double mean = means.val[0];
  double mean = 0.0;

  // generic loop to work for both float and int matrix types
  for (int i = 0; i < rows_ipm; i++) {
    for (int j = 0; j < cols_ipm; j++) {
      // get pixel coordiantes
      ui = mat_grid_uv_(0, i * cols_ipm + j);
      vi = mat_grid_uv_(1, i * cols_ipm + j);

      // check if out-of-bounds
      if (ui < ipm_info_.roi_uv_left || ui > ipm_info_.roi_uv_right ||
          vi < ipm_info_.roi_uv_top || vi > ipm_info_.roi_uv_bottom) {
        image_ipm(i, j) = mean;
      } else {
        // not out of bounds, then get nearest neighbor
        // Bilinear interpolation
        if (ipm_info_.ipm_interpolation == 0) {
          int x1 = int(ui), x2 = int(ui + 1);
          int y1 = int(vi), y2 = int(vi + 1);
          float x = ui - x1, y = vi - y1;

          float val = image_uv(y1, x1) * (1 - x) * (1 - y) +
                      image_uv(y1, x2) * x * (1 - y) +
                      image_uv(y2, x1) * (1 - x) * y + image_uv(y2, x2) * x * y;

          image_ipm(i, j) = val;
        } else {
          // nearest-neighbor interpolation
          image_ipm(i, j) = image_uv(int(vi + 0.5f), int(ui + 0.5f));
        }
      }
    }
  }
}

void VerifyCalibration::updateIpm() {

  float v = camera_info_.height_image;
  float u = camera_info_.width_image;

  // get the vanishing point
  Point2f pt_vp;
  pt_vp = getVanishingPoint();
  pt_vp.y = std::max<float>(0, pt_vp.y);

  // get extent of the image in the xfyf plane -> update IPM info
  float eps = ipm_info_.vp_portion * v; // VP_PORTION*v;
  ipm_info_.roi_uv_left = std::max<float>(0, ipm_info_.roi_uv_left);
  ipm_info_.roi_uv_right = std::min<float>(u - 1, ipm_info_.roi_uv_right);
  ipm_info_.roi_uv_top = std::max<float>(pt_vp.y + eps, ipm_info_.roi_uv_top);
  ipm_info_.roi_uv_bottom = std::min<float>(v - 1, ipm_info_.roi_uv_bottom);

  max_uv_y_ = pt_vp.y + eps;

  float pts_limits_uv[] = {pt_vp.x,
                           ipm_info_.roi_uv_right,
                           ipm_info_.roi_uv_left,
                           pt_vp.x,
                           ipm_info_.roi_uv_top,
                           ipm_info_.roi_uv_top,
                           ipm_info_.roi_uv_top,
                           ipm_info_.roi_uv_bottom};

  Eigen::Map<EigenMat> mat_limits_uv(pts_limits_uv, 2, 4);

  EigenMat mat_limits_ground;
  transformImage2Ground(mat_limits_uv, mat_limits_ground);

  float x_ground_max = mat_limits_ground.row(0).maxCoeff();
  float x_ground_min = mat_limits_ground.row(0).minCoeff();
  float y_ground_max = mat_limits_ground.row(1).maxCoeff();
  float y_ground_min = mat_limits_ground.row(1).minCoeff();

  int rows_ipm = ipm_info_.height_ipm;
  int cols_ipm = ipm_info_.width_ipm;

  float step_row = (y_ground_max - y_ground_min) / rows_ipm;
  float step_col = (x_ground_max - x_ground_min) / cols_ipm;

  // construct the grid to sample
  EigenMat mat_grid_ground = EigenMat(2, rows_ipm * cols_ipm);
  int i, j;
  float x, y;
  // fill it with x-y values on the ground plane in world frame
  for (i = 0, y = y_ground_max - .5 * step_row; i < rows_ipm;
       i++, y -= step_row) {
    for (j = 0, x = x_ground_min + .5 * step_col; j < cols_ipm;
         j++, x += step_col) {
      mat_grid_ground(0, i * cols_ipm + j) = x;
      mat_grid_ground(1, i * cols_ipm + j) = y;
    }
  }

  // get their pixel values in image frame
  mat_grid_uv_ = EigenMat(2, rows_ipm * cols_ipm);
  transformGround2Image(mat_grid_ground, mat_grid_uv_);

  // update ipm info according to ipm image info
  ipm_info_.limits_x[0] = mat_grid_ground(0, 0);
  ipm_info_.limits_x[1] =
      mat_grid_ground(0, (rows_ipm - 1) * cols_ipm + cols_ipm - 1);
  ipm_info_.limits_y[1] = mat_grid_ground(1, 0);
  ipm_info_.limits_y[0] =
      mat_grid_ground(1, (rows_ipm - 1) * cols_ipm + cols_ipm - 1);
  ipm_info_.scale_x = 1.0 / step_col;
  ipm_info_.scale_y = 1.0 / step_row;
  ipm_info_.width_ipm = cols_ipm;
  ipm_info_.height_ipm = rows_ipm;
}

} // namespace nullmax_calibration
