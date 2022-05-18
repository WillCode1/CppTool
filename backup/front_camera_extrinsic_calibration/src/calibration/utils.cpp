#include <iostream>
#include <utils.h>

using namespace std;
using namespace cv;

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::NumericDiffCostFunction;
using ceres::Problem;
// using ceres::Solve;
using ceres::Solver;

cv::Mat FindLane(const cv::Mat &img, const cv::Mat &mask,
                 std::vector<double> &vx, std::vector<double> &vy,
                 bool is_left) {
  vx.clear();
  vy.clear();

  cv::Mat edges;
  // Scharr edge detection,detection x axis gradient,then save in edges
  cv::Scharr(img, edges, CV_8UC1, 1, 0, 1, 0, cv::BORDER_DEFAULT);

  /*devide the img to left and right*/
  int col_start = 0;
  int col_end = img.cols / 2 - 2;
  if (!is_left) {
    col_start = img.cols / 2 + 2;
    col_end = img.cols;
  }

  cv::Mat output = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
  for (int i = 500; i < img.rows; i++) {
    for (int j = col_start; j < col_end; j++) {
      // reject the 0 pixel,using mask that pre-estimated
      uchar value_mask = mask.at<uchar>(i, j);
      if (value_mask < 200)
        continue;
      uchar value = edges.at<uchar>(i, j);
      // The gradient is bigger than 200,and save.Else reject
      if (value > 200) {
        output.at<uchar>(i, j) = 255;
        double x = 1.0 * j;
        double y = 1.0 * i;
        vx.push_back(x);
        vy.push_back(y);
      }
    }
  }
  return output;
}

void CalculateLine(const std::vector<double> &vx, const std::vector<double> &vy,
                   double &k, double &b) {
  k = 0.0;
  b = 0.0;

  {
    Problem problem;
    for (int i = 0; i < vx.size(); ++i) {
      CostFunction *cost_function =
          new AutoDiffCostFunction<LineFittingResidual, 1, 1, 1>(
              new LineFittingResidual(vx[i], vy[i]));
      problem.AddResidualBlock(cost_function, new CauchyLoss(0.5), &k, &b);
    }
    Solver::Options options;
    options.logging_type = ceres::SILENT;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
  }

  // Select the point that meet the line.
  // Error is defined 10 with the equation y=kx+b
  std::vector<double> vx_, vy_;
  for (int i = 0; i < vx.size(); i++) {
    double x = vx[i];
    double y = vy[i];
    double error = k * x + b;
    if (fabs(error) > 10)
      continue;

    vx_.push_back(x);
    vy_.push_back(y);
  }

  // Two step least square method
  {

    Problem problem;
    for (int i = 0; i < vx_.size(); ++i) {
      CostFunction *cost_function =
          new AutoDiffCostFunction<LineFittingResidual, 1, 1, 1>(
              new LineFittingResidual(vx_[i], vy_[i]));
      problem.AddResidualBlock(cost_function, new CauchyLoss(0.5), &k, &b);
    }
    Solver::Options options;
    options.logging_type = ceres::SILENT;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "Initial k: " << 0.0 << " b: " << 0.0 << "\n";
    std::cout << "Final   k: " << k << " c: " << b << "\n";
  }
}

int CalculateRawPitch(const nullmax_perception::CameraIntrinsic &intrinsic,
                      std::vector<nullmax_perception::LaneCoef> &lane_data,
                      cv::Point2f &vanish_point, float &pitch_raw) {
  if (lane_data.size() != 2) {
    std::cout << "not input two lane data \n";
    return 1;
  }

  float k1 = lane_data[0].coef_k;
  float b1 = lane_data[0].coef_b;
  float k2 = lane_data[1].coef_k;
  float b2 = lane_data[1].coef_b;

  // calculate raw_pitch
  // note the k1 k2 will not be nan
  // intersetion point--vanishing point
  float x = -(b1 - b2) / (k1 - k2);
  float y = k1 * x + b1;
  vanish_point.x = x;
  vanish_point.y = y;
  // calculate homo point
  float cam_z = 1;
  float cam_y = (y - cam_z * intrinsic.cy) / intrinsic.fy;
  cv::Mat ground_inter = (cv::Mat_<float>(3, 1) << 0, 0, 1);
  cv::Mat camera_inter = (cv::Mat_<float>(3, 1) << 0, cam_y, cam_z);
  camera_inter = camera_inter / cv::norm(camera_inter);
  cv::Mat rotation = ground_inter.cross(camera_inter);

  float sina = float(cv::norm(rotation));
  pitch_raw = -asin(sina);
  std::cout << "\033[1;31m"
            << " raw pitch " << pitch_raw << "\033[0m" << std::endl;

  return 0;
}

void OptimizePitchYaw(const std::vector<LaneVanishPoint> &lane_data,
                      const float &camera_height,
                      const nullmax_perception::CameraIntrinsic &intrinsic,
                      float &pitch_optimize, float &yaw_optimize) {

  double pitch_average = 0;
  for (size_t i = 0; i < lane_data.size(); i++) {
    LaneVanishPoint data = lane_data[i];
    pitch_average += double(data.pitch_raw);
  }
  pitch_average /= lane_data.size();
  std::cout << " Pitch initial value : " << pitch_average << std::endl;

  double pitch = pitch_average;
  double yaw = 0;
  double height = double(camera_height); // TODO: why not -camera_height

  Problem problem;
  for (int i = 0; i < int(lane_data.size()); ++i) {

    LaneVanishPoint data = lane_data[i];

    double k1 = double(data.lane_coef[0].coef_k);
    double b1 = double(data.lane_coef[0].coef_b);

    double k2 = double(data.lane_coef[1].coef_k);
    double b2 = double(data.lane_coef[1].coef_b);

    Eigen::Vector3d bearing1;
    Eigen::Vector3d bearing2;
    Eigen::Vector3d bearing3;
    Eigen::Vector3d bearing4;
    CalBearings(intrinsic, k1, b1, bearing1, bearing2);
    CalBearings(intrinsic, k2, b2, bearing3, bearing4);

    CostFunction *cost_function =
        new NumericDiffCostFunction<CostFatorPitchYaw, ceres::CENTRAL, 1, 1, 1>(
            new CostFatorPitchYaw(bearing1, bearing2, bearing3, bearing4,
                                  height));
    problem.AddResidualBlock(cost_function, NULL, &pitch, &yaw);
  }
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.logging_type = ceres::SILENT;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  std::cout << " result  " << std::endl;
  std::cout << "  pitch " << pitch_average << " -> " << pitch << std::endl;
  std::cout << "  yaw " << 0.0 << " -> " << yaw << std::endl;

  pitch_optimize = float(pitch);
  yaw_optimize = float(yaw);
}

void CalBearings(const nullmax_perception::CameraIntrinsic &intrinsic,
                 const double k, const double b, Eigen::Vector3d &b1,
                 Eigen::Vector3d &b2) {
  {

    double x = (600 - b) / k;
    double y = 600;

    double cam_x = (x - intrinsic.cx) / intrinsic.fx;
    double cam_y = (y - intrinsic.cy) / intrinsic.fy;
    double cam_z = 1;
    double n = sqrt(cam_x * cam_x + cam_y * cam_y + cam_z * cam_z);
    b1 = Eigen::Vector3d(cam_x / n, cam_y / n, cam_z / n);
  }
  {
    double x = (960 - b) / k;
    double y = 960;
    double cam_x = (x - intrinsic.cx) / intrinsic.fx;
    double cam_y = (y - intrinsic.cy) / intrinsic.fy;
    double cam_z = 1;
    double n = sqrt(cam_x * cam_x + cam_y * cam_y + cam_z * cam_z);
    b2 = Eigen::Vector3d(cam_x / n, cam_y / n, cam_z / n);
  }
}

void OptimizePitchYawSingle(
    const LaneVanishPoint &lane_data, const float &camera_height,
    const nullmax_perception::CameraIntrinsic &intrinsic,
    float &pitch_optimize) {

  double pitch = double(pitch_optimize);
  double height = double(camera_height); // TODO: why not -camera_height
  Problem problem;
  double k1 = double(lane_data.lane_coef[0].coef_k);
  double b1 = double(lane_data.lane_coef[0].coef_b);
  double k2 = double(lane_data.lane_coef[1].coef_k);
  double b2 = double(lane_data.lane_coef[1].coef_b);
  Eigen::Vector3d bearing1;
  Eigen::Vector3d bearing2;
  Eigen::Vector3d bearing3;
  Eigen::Vector3d bearing4;
  CalBearings(intrinsic, k1, b1, bearing1, bearing2);
  CalBearings(intrinsic, k2, b2, bearing3, bearing4);

  CostFunction *cost_function =
      new NumericDiffCostFunction<CostFatorPitch, ceres::CENTRAL, 1, 1>(
          new CostFatorPitch(bearing1, bearing2, bearing3, bearing4, height));
  problem.AddResidualBlock(cost_function, NULL, &pitch);

  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.logging_type = ceres::SILENT;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << " result  " << std::endl;
  std::cout << "  pitch " << pitch_optimize << " -> " << pitch << std::endl;
  pitch_optimize = float(pitch);
}

cv::Mat
CreateBirdView(const cv::Mat &img,
               const nullmax_perception::CameraRotationEuler &rotation_angle,
               const nullmax_perception::CameraIntrinsic &intrinsic,
               const float &camera_height) {
  cv::Mat bv = cv::Mat::zeros(940, 700, CV_8UC1);
  cv::Mat Twc = cv::Mat::zeros(4, 4, CV_32F);
  Twc.at<float>(0, 3) = 0;
  Twc.at<float>(1, 3) = -camera_height;
  Twc.at<float>(2, 3) = 0;
  Twc.at<float>(3, 3) = 1.0;
  Eigen::Matrix3f Rx =
      (Eigen::AngleAxisf(rotation_angle.pitch, Eigen::Vector3f::UnitX()))
          .matrix();
  Eigen::Matrix3f Ry =
      (Eigen::AngleAxisf(-rotation_angle.yaw, Eigen::Vector3f::UnitY()))
          .matrix();
  Eigen::Matrix3f Rz =
      (Eigen::AngleAxisf(rotation_angle.roll, Eigen::Vector3f::UnitZ()))
          .matrix();
  Eigen::Matrix3f R = Rx * Rz * Ry;

  cv::Mat Rcw = (cv::Mat_<float>(3, 3) << R(0, 0), R(0, 1), R(0, 2), R(1, 0),
                 R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));

  cv::Mat Rwc = Rcw.inv();
  Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
  cv::Mat Tcw = Twc.inv();
  for (int i = 0; i < bv.rows; i++) {
    for (int j = 0; j < bv.cols; j++) {
      float x = float((j - 350) * 0.03);
      float y = 0;
      float z = float(40 - 0.03 * i);

      cv::Mat wp = (cv::Mat_<float>(3, 1) << x, y, z);
      cv::Mat camp =
          Tcw.rowRange(0, 3).colRange(0, 3) * wp + Tcw.rowRange(0, 3).col(3);

      float camx = camp.at<float>(0, 0);
      float camy = camp.at<float>(1, 0);
      float camz = camp.at<float>(2, 0);

      float u = (camx / camz) * intrinsic.fx + intrinsic.cx;
      float v = (camy / camz) * intrinsic.fy + intrinsic.cy;

      int rows = int(v);
      int cols = int(u);
      if (rows < 0 || rows >= img.rows)
        continue;
      if (cols < 0 || cols >= img.cols)
        continue;
      bv.at<uchar>(i, j) = img.at<uchar>(rows, cols);
    }
  }
  cv::line(bv, cv::Point2f(10, 0), cv::Point2f(10, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(110, 0), cv::Point2f(110, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(210, 0), cv::Point2f(210, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(310, 0), cv::Point2f(310, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(410, 0), cv::Point2f(410, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(510, 0), cv::Point2f(510, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(610, 0), cv::Point2f(610, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(710, 0), cv::Point2f(710, 940), cv::Scalar(255), 1);
  cv::line(bv, cv::Point2f(810, 0), cv::Point2f(810, 940), cv::Scalar(255), 1);
  return bv;
}
