#include "ocam_algrithom.h"

namespace nullmax_perception {
namespace calibration {
template <typename T>
T PolyVal2(const std::vector<T> &ss, const T &theta, const T &radius) {
  T m = ceres::tan(theta);
  std::vector<T> poly_coeff_tmp = ss;
  poly_coeff_tmp[1] = poly_coeff_tmp[1] - m;
  std::vector<T> p_deriv_tmp(poly_coeff_tmp.size() - 1);
  for (size_t i = 0; i < p_deriv_tmp.size(); ++i) {
    p_deriv_tmp[i] = T(i + 1) * poly_coeff_tmp[i + 1];
  }
  int max_iter_count = 300;
  T x0 = radius;
  T tolerance = T(10e-14);
  T epsilon = T(10e-14);
  bool ok = false;
  // same as matlab roots()
  for (int i = 0; i < max_iter_count; ++i) {
    T y = PolyVal<T>(poly_coeff_tmp, x0);
    T yprime = PolyVal<T>(p_deriv_tmp, x0);

    // denominator is too small
    if (ceres::abs(yprime) < epsilon) {
      break;
    }

    T x1 = x0 - y / yprime;

    if (ceres::abs(x1 - x0) <= tolerance * ceres::abs(x1)) {
      ok = true;
      break;
    }
    x0 = x1;
  }
  return x0;
  // TODO how to judge NAN in template function
  // if (ok) {
  //   return x0;
  // } else
  //   return numeric_limits<T>::quiet_NaN();
}

template <typename T> T PolyVal(const std::vector<T> &ss, const T &x) {
  T res = T(0);
  T m = T(1);
  for (auto &c : ss) {
    res += c * m;
    m *= x;
  }
  return res;
}

template <typename T>
Eigen::Matrix<T, 2, 1>
ReprojectPoint(const std::vector<T> &ss, const T &c, const T &d, const T &e,
               const T &xc, const T &yc, const Eigen::Matrix<T, 3, 1> &xx_,
               const T &width, const T &height) {
  Eigen::Matrix<T, 3, 1> xx = xx_;
  if (xx(0) == T(0) && xx(1) == T(0)) {
    xx(0) = T(1e-14);
    xx(1) = T(1e-14);
  }
  T d_s = ceres::sqrt(xx(0) * xx(0) + xx(1) * xx(1));
  T m = ceres::atan2(xx(2), d_s);
  T radius = ceres::sqrt(width * width / 4.0 + height * height / 4.0);
  T rho = PolyVal2<T>(ss, m, radius);
  T x = xx(0) / d_s * rho;
  T y = xx(1) / d_s * rho;

  return Eigen::Matrix<T, 2, 1>(x * c + y * d + xc, x * e + y + yc);
}

double OcamAlgrithom::Calibrate(
    const std::vector<std::vector<cv::Point3f>> &object_points,
    const std::vector<std::vector<cv::Point2f>> &image_corners,
    const cv::Size &image_size, std::shared_ptr<OcamIntrinsics> intrinsic) {
  // 1.init param
  InitParam(object_points, image_corners, image_size, intrinsic);

  // 2.estimate extrinsic and intrinsic
  EstimateParam();

  // 3.estimate cx,cy and calibrate again
  FindCenter();
  EstimateParam();

  // 4.refine all param including c,d,e
  double avg_repro_error = RefineParameters();

  return avg_repro_error;
}

std::vector<double> OcamAlgrithom::GetReproErrors() const {
  return repro_errors_;
}

std::vector<std::vector<cv::Point2f>>
OcamAlgrithom::GetPixelReproErrors() const {
  return pixel_repro_errors_;
}

void OcamAlgrithom::InitParam(
    const std::vector<std::vector<cv::Point3f>> &object_points,
    const std::vector<std::vector<cv::Point2f>> &image_corners,
    const cv::Size &image_size, std::shared_ptr<OcamIntrinsics> intrinsic) {
  // swap x and y according to Matlab
  intrinsic_ = intrinsic;
  intrinsic_->img_size = image_size;
  intrinsic_->cx = image_size.height / 2.0;
  intrinsic_->cy = image_size.width / 2.0;

  // init 3D world points
  intrinsic_->Yt = Eigen::MatrixXd(object_points[0].size(), 1);
  intrinsic_->Xt = Eigen::MatrixXd(object_points[0].size(), 1);
  for (int j = 0; j < object_points[0].size(); ++j) {
    intrinsic_->Yt(j, 0) = object_points[0][j].x;
    intrinsic_->Xt(j, 0) = object_points[0][j].y;
  }

  // init 2D corners
  intrinsic_->image_corners = image_corners;

  // init RT and stretch matrix
  intrinsic_->RRfin.resize(object_points.size());
  intrinsic_->c = 1.;
  intrinsic_->d = 0.;
  intrinsic_->e = 0.;
}

double OcamAlgrithom::EstimateParam() {
  if (!EstimateExtrisics() || !EstimateIntrisics()) {
    return std::numeric_limits<double>::infinity();
  }

  double avg_reprojection_error = ReprojectError();

  return avg_reprojection_error;
}

std::vector<double> OcamAlgrithom::PolyRootD2(const double &a, const double &b,
                                              const double &c) {
  std::vector<double> ret;
  double d = b * b - 4 * a * c;
  if (d > std::numeric_limits<double>::epsilon()) {
    d = sqrt(d);
    double r = (-b + d) / (2 * a);
    ret.push_back(r);

    r = (-b - d) / (2 * a);
    ret.push_back(r);
  } else if (std::fabs(d) < std::numeric_limits<double>::epsilon()) {
    double r = -b / (2 * a);
    ret.push_back(r);
  } else {
    assert(false);
  }
  return ret;
}

int OcamAlgrithom::Plot_RR(const std::vector<Eigen::Matrix3d> &RR,
                           const Eigen::MatrixXd &Xt, const Eigen::MatrixXd &Yt,
                           const Eigen::MatrixXd &Xpt,
                           const Eigen::MatrixXd &Ypt) {
  int index = -1;

  for (size_t i = 0; i < RR.size(); ++i) {
    Eigen::Matrix3d RRdef = RR[i];
    double R11 = RRdef(0, 0);
    double R21 = RRdef(1, 0);
    double R31 = RRdef(2, 0);
    double R12 = RRdef(0, 1);
    double R22 = RRdef(1, 1);
    double R32 = RRdef(2, 1);
    double T1 = RRdef(0, 2);
    double T2 = RRdef(1, 2);

    Eigen::MatrixXd MA = (R21 * Xt + R22 * Yt).array() + T2;
    Eigen::MatrixXd MB = Ypt.cwiseProduct(R31 * Xt + R32 * Yt);
    Eigen::MatrixXd MC = (R11 * Xt + R12 * Yt).array() + T1;
    Eigen::MatrixXd MD = Xpt.cwiseProduct(R31 * Xt + R32 * Yt);
    Eigen::MatrixXd rho =
        (Xpt.cwiseProduct(Xpt) + Ypt.cwiseProduct(Ypt)).cwiseSqrt();
    Eigen::MatrixXd rho2 = rho.cwiseProduct(rho);

    Eigen::MatrixXd PP1(2 * Xt.rows(), 3);
    PP1.block(0, 0, Xt.rows(), 1) = MA;
    PP1.block(Xt.rows(), 0, Xt.rows(), 1) = MC;
    PP1.block(0, 1, Xt.rows(), 1) = MA.cwiseProduct(rho);
    PP1.block(Xt.rows(), 1, Xt.rows(), 1) = MC.cwiseProduct(rho);
    PP1.block(0, 2, Xt.rows(), 1) = MA.cwiseProduct(rho2);
    PP1.block(Xt.rows(), 2, Xt.rows(), 1) = MC.cwiseProduct(rho2);

    Eigen::MatrixXd PP(2 * Xt.rows(), 4);
    PP.block(0, 0, PP1.rows(), 3) = PP1;
    PP.block(0, 3, Ypt.rows(), 1) = -Ypt;
    PP.block(Ypt.rows(), 3, Ypt.rows(), 1) = -Xpt;

    Eigen::MatrixXd QQ(MB.rows() * 2, 1);
    QQ.block(0, 0, MB.rows(), 1) = MB;
    QQ.block(MB.rows(), 0, MB.rows(), 1) = MD;
    Eigen::MatrixXd s =
        PP.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(QQ);
    if (s(2) >= 0)
      index = (int)i;
  }
  return index;
}
bool OcamAlgrithom::EstimateExtrisics() {
  for (int kk = 0; kk < intrinsic_->image_corners.size(); ++kk) {
    int image_number = intrinsic_->image_corners[kk].size();
    Eigen::MatrixXd Ypt(image_number, 1);
    Eigen::MatrixXd Xpt(image_number, 1);
    for (int j = 0; j < image_number; ++j) {
      Ypt(j, 0) = intrinsic_->image_corners[kk][j].x - intrinsic_->cy;
      Xpt(j, 0) = intrinsic_->image_corners[kk][j].y - intrinsic_->cx;
    }

    Eigen::MatrixXd A(image_number, 6);

    A.col(0) = intrinsic_->Xt.cwiseProduct(Ypt);
    A.col(1) = intrinsic_->Yt.cwiseProduct(Ypt);
    A.col(2) = -intrinsic_->Xt.cwiseProduct(Xpt);
    A.col(3) = -intrinsic_->Yt.cwiseProduct(Xpt);
    A.col(4) = Ypt;
    A.col(5) = -Xpt;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);

    double R11 = svd.matrixV()(0, 5);
    double R12 = svd.matrixV()(1, 5);
    double R21 = svd.matrixV()(2, 5);
    double R22 = svd.matrixV()(3, 5);
    double T1 = svd.matrixV()(4, 5);
    double T2 = svd.matrixV()(5, 5);

    double AA = ((R11 * R12) + (R21 * R22)) * ((R11 * R12) + (R21 * R22));
    double BB = R11 * R11 + R21 * R21;
    double CC = R12 * R12 + R22 * R22;
    std::vector<double> R32_2 = PolyRootD2(1, CC - BB, -AA);

    auto IsVectorZero = [&](const std::vector<double> &v) -> bool {
      for (auto e : v) {
        if (e != 0)
          return false;
      }
      return true;
    };

    std::vector<double> R31, R32;
    std::vector<double> sg = {1.0, -1.0};
    for (auto r : R32_2) {
      if (r > 0) {
        for (auto c : sg) {
          double sqrtR32_2 = c * std::sqrt(r);
          R32.push_back(sqrtR32_2);
          if (IsVectorZero(R32_2)) {
            R31.push_back(std::sqrt(CC - BB));
            R31.push_back(-std::sqrt(CC - BB));
            R32.push_back(sqrtR32_2);
          } else {
            R31.push_back((R11 * R12 + R21 * R22) / -sqrtR32_2);
          }
        }
      }
    }

    std::vector<Eigen::Matrix3d> RR(R32.size() * 2, Eigen::Matrix3d::Zero());

    int count = -1;
    for (size_t i1 = 0; i1 < R32.size(); ++i1) {
      for (size_t i2 = 0; i2 < 2; ++i2) {
        count++;
        double Lb = 1 / std::sqrt(R11 * R11 + R21 * R21 + R31[i1] * R31[i1]);
        RR[count] << R11, R12, T1, R21, R22, T2, R31[i1], R32[i1], 0.0;
        RR[count] *= sg[i2] * Lb;
      }
    }

    double minRR = std::numeric_limits<double>::infinity();
    int minRR_ind = -1;
    for (size_t min_count = 0; min_count < RR.size(); ++min_count) {
      if ((Eigen::Vector2d(RR[min_count](0, 2), RR[min_count](1, 2)) -
           Eigen::Vector2d(Xpt(0), Ypt(0)))
              .norm() < minRR) {
        minRR = (Eigen::Vector2d(RR[min_count](0, 2), RR[min_count](1, 2)) -
                 Eigen::Vector2d(Xpt(0), Ypt(0)))
                    .norm();
        minRR_ind = (int)min_count;
      }
    }

    auto Sign = [&](const double &val) -> int {
      return (0.0 < val) - (val < 0.0);
    };
    std::vector<Eigen::Matrix3d> RR1;
    if (minRR_ind != -1) {
      for (size_t count = 0; count < RR.size(); ++count) {
        if (Sign(RR[count](0, 2)) == Sign(RR[minRR_ind](0, 2)) &&
            Sign(RR[count](1, 2)) == Sign(RR[minRR_ind](1, 2))) {
          RR1.push_back(RR[count]);
        }
      }
    }

    if (RR1.empty()) {
      return false;
    }

    int nm = Plot_RR(RR1, intrinsic_->Xt, intrinsic_->Yt, Xpt, Ypt);
    Eigen::Matrix3d RRdef = RR1[nm];
    intrinsic_->RRfin[kk] = RRdef;
  }
  return true;
}

bool OcamAlgrithom::EstimateIntrisics() {
  double xc = intrinsic_->cx;
  double yc = intrinsic_->cy;

  int count = -1;
  Eigen::MatrixXd PP = Eigen::MatrixXd::Zero(
      2 * intrinsic_->Xt.rows() * intrinsic_->image_corners.size(),
      4 + intrinsic_->image_corners.size());
  Eigen::MatrixXd QQ = Eigen::MatrixXd::Zero(
      2 * intrinsic_->Xt.rows() * intrinsic_->image_corners.size(), 1);

  if (intrinsic_->image_corners.empty()) {
    return false;
  }

  for (size_t i = 0; i < intrinsic_->image_corners.size(); ++i) {
    Eigen::MatrixXd Ypt(intrinsic_->image_corners[i].size(), 1);
    Eigen::MatrixXd Xpt(intrinsic_->image_corners[i].size(), 1);
    for (int j = 0; j < intrinsic_->image_corners[i].size(); ++j) {
      Ypt(j, 0) = intrinsic_->image_corners[i][j].x - yc;
      Xpt(j, 0) = intrinsic_->image_corners[i][j].y - xc;
    }
    count++;

    Eigen::Matrix3d RRdef = intrinsic_->RRfin[i];

    double R11 = RRdef(0, 0);
    double R21 = RRdef(1, 0);
    double R31 = RRdef(2, 0);
    double R12 = RRdef(0, 1);
    double R22 = RRdef(1, 1);
    double R32 = RRdef(2, 1);
    double T1 = RRdef(0, 2);
    double T2 = RRdef(1, 2);

    Eigen::MatrixXd MA =
        (R21 * intrinsic_->Xt + R22 * intrinsic_->Yt).array() + T2;
    Eigen::MatrixXd MB =
        Ypt.cwiseProduct(R31 * intrinsic_->Xt + R32 * intrinsic_->Yt);
    Eigen::MatrixXd MC =
        (R11 * intrinsic_->Xt + R12 * intrinsic_->Yt).array() + T1;
    Eigen::MatrixXd MD =
        Xpt.cwiseProduct(R31 * intrinsic_->Xt + R32 * intrinsic_->Yt);
    std::vector<Eigen::MatrixXd> rho(4);

    Eigen::MatrixXd tmp =
        (Xpt.cwiseProduct(Xpt) + Ypt.cwiseProduct(Ypt)).cwiseSqrt();
    rho[0] = tmp;
    for (size_t j = 1; j < 4; ++j) {
      rho[j] = tmp.cwiseProduct(rho[j - 1]);
    }
    rho[0] = Eigen::MatrixXd::Zero(tmp.rows(), tmp.cols());

    Eigen::MatrixXd PP1(intrinsic_->Xt.rows() * 2, 4);
    PP1.block(0, 0, intrinsic_->Xt.rows(), 1) = MA;
    PP1.block(intrinsic_->Xt.rows(), 0, intrinsic_->Xt.rows(), 1) = MC;
    for (int j = 1; j < 4; ++j) {
      PP1.block(0, j, intrinsic_->Xt.rows(), 1) = MA.cwiseProduct(rho[j]);
      PP1.block(intrinsic_->Xt.rows(), j, intrinsic_->Xt.rows(), 1) =
          MC.cwiseProduct(rho[j]);
    }

    PP.block(PP1.rows() * i, 0, PP1.rows(), PP1.cols()) = PP1;
    PP.block(PP1.rows() * i, 4 + i, Ypt.rows(), 1) = -Ypt;
    PP.block(PP1.rows() * i + Ypt.rows(), 4 + i, Ypt.rows(), 1) = -Xpt;

    QQ.block(PP1.rows() * i, 0, MB.rows(), 1) = MB;
    QQ.block(PP1.rows() * i + MB.rows(), 0, MB.rows(), 1) = MD;
  }

  Eigen::MatrixXd s =
      PP.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(QQ);

  intrinsic_->pol.resize(5);
  intrinsic_->pol[0] = s(0);
  intrinsic_->pol[1] = 0.0;
  for (int i = 2; i < 5; ++i) {
    intrinsic_->pol[i] = s(i - 1);
  }

  for (size_t i = 0; i < intrinsic_->image_corners.size(); ++i) {
    intrinsic_->RRfin[i](2, 2) = s(4 + i);
  }
  return true;
}

double OcamAlgrithom::ReprojectError() {
  double rms2 = 0;
  int count = 0;
  repro_errors_.clear();
  pixel_repro_errors_.clear();
  for (auto k = 0; k < intrinsic_->image_corners.size(); ++k) {
    double error(0.0);
    int single_count(0);
    std::vector<cv::Point2f> single_pixel_repro_error;
    for (int i = 0; i < intrinsic_->Xt.rows(); ++i) {
      Eigen::Vector3d w =
          intrinsic_->RRfin[k] *
          Eigen::Vector3d(intrinsic_->Xt(i), intrinsic_->Yt(i), 1);
      Eigen::Vector2d reprojected = ReprojectPoint<double>(
          intrinsic_->pol, intrinsic_->c, intrinsic_->d, intrinsic_->e,
          intrinsic_->cx, intrinsic_->cy, w, intrinsic_->img_size.width,
          intrinsic_->img_size.height);
      Eigen::Vector2d orig(intrinsic_->image_corners[k][i].y,
                           intrinsic_->image_corners[k][i].x);

      Eigen::Vector2d delta = orig - reprojected;
      cv::Point2f single_pixel_error;
      single_pixel_error.x = delta[0];
      single_pixel_error.y = delta[1];
      single_pixel_repro_error.push_back(single_pixel_error);

      error += delta.norm();
      ++single_count;

      rms2 += delta.norm();
      ++count;
    }
    repro_errors_.push_back(error / single_count);
    pixel_repro_errors_.push_back(single_pixel_repro_error);
  }
  return rms2 / count;
}

void OcamAlgrithom::FindCenter() {
  double pxc = intrinsic_->cx;
  double pyc = intrinsic_->cy;
  double width = intrinsic_->img_size.width;
  double height = intrinsic_->img_size.height;
  double regwidth = (width / 2);
  double regheight = (height / 2);
  double yceil = 5;
  double xceil = 5;

  double xregstart = pxc - (regheight / 2);
  double xregstop = pxc + (regheight / 2);
  double yregstart = pyc - (regwidth / 2);
  double yregstop = pyc + (regwidth / 2);

  // iterative search accurate cx,cy
  for (int glc = 0; glc < 9; ++glc) {
    double s = ((yregstop - yregstart) / yceil);
    int c = int((yregstop - yregstart) / s + 1);

    Eigen::MatrixXd yreg(c, c);
    for (int i = 0; i < c; ++i) {
      for (int j = 0; j < c; ++j) {
        yreg(i, j) = yregstart + j * s;
      }
    }

    s = ((xregstop - xregstart) / xceil);

    Eigen::MatrixXd xreg(c, c);
    for (int i = 0; i < c; ++i) {
      for (int j = 0; j < c; ++j) {
        xreg(i, j) = xregstart + i * s;
      }
    }

    int ic_proc = (int)xreg.rows();
    int jc_proc = (int)xreg.cols();
    int min_idx1, min_idx2;
    double min_MSEA = std::numeric_limits<double>::max();
    int a = 0;
    for (int ic = 0; ic < ic_proc; ++ic) {
      for (int jc = 0; jc < jc_proc; ++jc) {
        intrinsic_->cx = xreg(ic, jc);
        intrinsic_->cy = yreg(ic, jc);

        EstimateParam();

        auto isMatricesZeros =
            [&](const std::vector<Eigen::Matrix3d> &ms) -> bool {
          for (auto &m : ms) {
            if (m.norm() < std::numeric_limits<double>::epsilon())
              return true;
          }
          return false;
        };
        if (isMatricesZeros(intrinsic_->RRfin)) {
          continue;
        }
        double MSE = ReprojectError();

        if (!std::isnan(MSE) && MSE < min_MSEA) {
          min_MSEA = MSE;
          min_idx1 = ic;
          min_idx2 = jc;
        }
      }
    }

    intrinsic_->cx = xreg(min_idx1, min_idx2);
    intrinsic_->cy = yreg(min_idx1, min_idx2);
    double dx_reg = std::abs((xregstop - xregstart) / xceil);
    double dy_reg = std::abs((yregstop - yregstart) / yceil);
    xregstart = intrinsic_->cx - dx_reg;
    xregstop = intrinsic_->cx + dx_reg;
    yregstart = intrinsic_->cy - dy_reg;
    yregstop = intrinsic_->cy + dy_reg;
  }
}

double OcamAlgrithom::RefineParameters() {
  double MSE_tol = 1e-12;
  double MSE_old = 0.;
  double MSE_new = std::numeric_limits<double>::max();
  std::cout << "refine parameters ..." << std::endl;

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_num_iterations = 10000;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;

  // iterative refine all parameters
  for (int iter = 0; iter < 100 && std::fabs(MSE_new - MSE_old) > MSE_tol;
       ++iter) {
    // refine extrinsics firstly
    for (int kk = 0; kk < intrinsic_->image_corners.size(); ++kk) {
      Eigen::Matrix3d R = intrinsic_->RRfin[kk];
      R.col(2) = R.col(0).cross(R.col(1));
      Eigen::Vector3d t = intrinsic_->RRfin[kk].col(2);
      Eigen::Vector3d r;
      ceres::RotationMatrixToAngleAxis(R.data(), r.data());

      Eigen::Matrix<double, 6, 1> refined_extri;
      refined_extri.block(0, 0, 3, 1) = r;
      refined_extri.block(3, 0, 3, 1) = t;

      ceres::Problem problem_extri;
      for (int j = 0; j < intrinsic_->image_corners[kk].size(); ++j) {
        Eigen::Vector3d p(intrinsic_->Xt(j), intrinsic_->Yt(j), 1.0);

        ceres::CostFunction *cost_function = ExtrinsicCost::Create(
            intrinsic_->image_corners[kk][j].x,
            intrinsic_->image_corners[kk][j].y, *intrinsic_, p.x(), p.y());
        problem_extri.AddResidualBlock(cost_function, nullptr,
                                       refined_extri.data());
      }

      ceres::Solve(options, &problem_extri, &summary);

      r = refined_extri.block(0, 0, 3, 1);
      t = refined_extri.block(3, 0, 3, 1);

      ceres::AngleAxisToRotationMatrix(r.data(), intrinsic_->RRfin[kk].data());
      intrinsic_->RRfin[kk].col(2) = t;
    }

    // only refine c,d,e secondly
    Eigen::Matrix<double, 3, 1> refined_stretch;
    refined_stretch[0] = intrinsic_->c;
    refined_stretch[1] = intrinsic_->d;
    refined_stretch[2] = intrinsic_->e;

    ceres::Problem problem_stretch;
    ceres::CostFunction *stretch_cost = StretchCost::Create(*intrinsic_);
    problem_stretch.AddResidualBlock(stretch_cost, NULL,
                                     refined_stretch.data());
    ceres::Solve(options, &problem_stretch, &summary);

    intrinsic_->c = refined_stretch[0];
    intrinsic_->d = refined_stretch[1];
    intrinsic_->e = refined_stretch[2];

    // finally refine the coefficient of all pol params
    ceres::CostFunction *intinsics_cost = IntinsicsCost::Create(*intrinsic_);
    ceres::Problem problem_intri;

    Eigen::Matrix<double, 6, 1> refined_intri =
        Eigen::Matrix<double, 6, 1>::Ones();

    problem_intri.AddResidualBlock(intinsics_cost, NULL, refined_intri.data());
    ceres::Solve(options, &problem_intri, &summary);

    intrinsic_->cx = refined_intri[0] * intrinsic_->cx;
    intrinsic_->cy = refined_intri[1] * intrinsic_->cy;
    intrinsic_->pol[0] = refined_intri[2] * intrinsic_->pol[0];
    for (size_t i = 2; i < intrinsic_->pol.size(); ++i) {
      intrinsic_->pol[i] = refined_intri[1 + i] * intrinsic_->pol[i];
    }

    MSE_old = MSE_new;
    MSE_new = ReprojectError();
  }
  intrinsic_->invpol = GetInvPol(intrinsic_->pol, intrinsic_->img_size.width,
                                 intrinsic_->img_size.height);

  for (auto v : intrinsic_->RRfin) {
    Eigen::Matrix3d R = v;
    R.col(2) = R.col(0).cross(R.col(1));

    intrinsic_->R.push_back({R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1),
                             R(1, 2), R(2, 0), R(2, 1), R(2, 2)});

    Eigen::Vector3d t = v.col(2);
    intrinsic_->T.push_back({t(0), t(1), t(2)});
  }
  // reverse cx,cy
  double change = intrinsic_->cx;
  intrinsic_->cx = intrinsic_->cy;
  intrinsic_->cy = change;

  return MSE_new;
}

std::vector<double> OcamAlgrithom::GetInvPol(const std::vector<double> &p,
                                             double w, double h) {
  double max_err = std::numeric_limits<double>::infinity();
  std::vector<double> res;
  int poly_degree = 5;
  while (max_err > 1e-2) {

    std::vector<double> theta;
    std::vector<double> r;

    double step = 0.01;
    double radius = sqrt(w * w / 4 + h * h / 4);
    for (double x = -M_PI_2; x < 1.2; x += step) {
      // matlab source: invFUN(ss, theta, radius)
      double y = PolyVal2<double>(p, x, radius);
      if (y != std::numeric_limits<double>::quiet_NaN() && y > 0 &&
          y < radius) {
        theta.push_back(x);
        r.push_back(y);
      }
    }

    // like matlab polyfit()
    Eigen::MatrixXd A(theta.size(), poly_degree + 1);
    Eigen::MatrixXd b(theta.size(), 1);

    for (size_t i = 0; i < theta.size(); ++i) {
      A(i, 0) = 1;
      b(i) = r[i];
      for (size_t j = 1; j < poly_degree + 1; ++j) {
        A(i, j) = A(i, j - 1) * theta[i];
      }
    }

    Eigen::MatrixXd x =
        A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    res.resize(poly_degree + 1);
    for (int i = 0; i < poly_degree + 1; ++i) {
      res[i] = x(i);
    }

    double tmp_max_err = 0;
    for (size_t i = 0; i < r.size(); ++i) {
      double err = std::fabs(r[i] - PolyVal<double>(res, theta[i]));
      if (err > tmp_max_err) {
        tmp_max_err = err;
      }
    }
    max_err = tmp_max_err;
    ++poly_degree;
  }
  return res;
}
} // calibration
} // nullmax_perception