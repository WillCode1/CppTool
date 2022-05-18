#pragma once

#include <Eigen/Core>
#include <Eigen/SVD>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <limits>
#include <math.h>
#include <opencv2/opencv.hpp>

namespace nullmax_perception {
namespace calibration {
struct OcamIntrinsics {
  std::vector<double> pol;
  std::vector<double> invpol;
  double cx;
  double cy;
  double c;
  double d;
  double e;
  std::vector<std::vector<double>> R;
  std::vector<std::vector<double>> T;
  cv::Size img_size;
  std::vector<Eigen::Matrix3d> RRfin;
  Eigen::MatrixXd Xt;
  Eigen::MatrixXd Yt;
  std::vector<std::vector<cv::Point2f>> image_corners;
};

template <typename T>
static Eigen::Matrix<T, 2, 1>
ReprojectPoint(const std::vector<T> &ss, const T &c, const T &d, const T &e,
               const T &xc, const T &yc, const Eigen::Matrix<T, 3, 1> &xx_,
               const T &width, const T &height);
template <typename T>
static T PolyVal2(const std::vector<T> &ss, const T &theta, const T &radius);
template <typename T> static T PolyVal(const std::vector<T> &ss, const T &x);

class OcamAlgrithom {
public:
  using Ptr = std::shared_ptr<OcamAlgrithom>;
  OcamAlgrithom() {}
  OcamAlgrithom &operator=(const OcamAlgrithom &) = delete;
  OcamAlgrithom(const OcamAlgrithom &) = delete;
  ~OcamAlgrithom() {}

  double Calibrate(const std::vector<std::vector<cv::Point3f>> &object_points,
                   const std::vector<std::vector<cv::Point2f>> &image_corners,
                   const cv::Size &image_size,
                   std::shared_ptr<OcamIntrinsics> intrinsic);

  std::vector<double> GetReproErrors() const;
  std::vector<std::vector<cv::Point2f>> GetPixelReproErrors() const;

private:
  // calibration function
  void InitParam(const std::vector<std::vector<cv::Point3f>> &object_points,
                 const std::vector<std::vector<cv::Point2f>> &image_corners,
                 const cv::Size &image_size,
                 std::shared_ptr<OcamIntrinsics> intrinsic);
  double EstimateParam();
  bool EstimateExtrisics();
  bool EstimateIntrisics();
  double ReprojectError();
  void FindCenter();
  double RefineParameters();

  // tool function
  std::vector<double> PolyRootD2(const double &a, const double &b,
                                 const double &c);
  int Plot_RR(const std::vector<Eigen::Matrix3d> &RR, const Eigen::MatrixXd &Xt,
              const Eigen::MatrixXd &Yt, const Eigen::MatrixXd &Xpt,
              const Eigen::MatrixXd &Ypt);
  std::vector<double> GetInvPol(const std::vector<double> &p, double w,
                                double h);

private:
  std::shared_ptr<OcamIntrinsics> intrinsic_;
  std::vector<double> repro_errors_;
  std::vector<std::vector<cv::Point2f>> pixel_repro_errors_;
};

struct ExtrinsicCost {
public:
  ExtrinsicCost(const double &pixel_u, const double &pixel_v,
                const OcamIntrinsics &intrinsic, const double &point_x,
                const double &point_y)
      : pixel_u_(pixel_u), pixel_v_(pixel_v), intrinsic_(intrinsic),
        point_x_(point_x), point_y_(point_y) {}

  template <typename T> bool operator()(const T *const RT, T *residuals) const {
    Eigen::Matrix<T, 3, 1> p;
    Eigen::Matrix<T, 3, 3> R;
    ceres::AngleAxisToRotationMatrix<T>(RT, R.data());
    R(0, 2) = RT[3];
    R(1, 2) = RT[4];
    R(2, 2) = RT[5];
    p(0) = T(point_x_);
    p(1) = T(point_y_);
    p(2) = T(1.0);
    p = R * p;

    T width = T(intrinsic_.img_size.width);
    T height = T(intrinsic_.img_size.height);
    T c = T(1.0);
    T d = T(0.0);
    T e = T(0.0);
    T xc = T(intrinsic_.cx);
    T yc = T(intrinsic_.cy);
    std::vector<T> ss;
    for (auto &s : intrinsic_.pol) {
      ss.push_back(T(s));
    }

    Eigen::Matrix<T, 2, 1> ret =
        ReprojectPoint<T>(ss, c, d, e, xc, yc, p, width, height);

    residuals[0] = T(pixel_v_ - ret.x());
    residuals[1] = T(pixel_u_ - ret.y());
    return true;
  }

  static ceres::CostFunction *Create(const double &pixel_u,
                                     const double &pixel_v,
                                     const OcamIntrinsics &intrinsic,
                                     const double &point_x,
                                     const double &point_y) {
    return (new ceres::AutoDiffCostFunction<ExtrinsicCost, 2, 6>(
        new ExtrinsicCost(pixel_u, pixel_v, intrinsic, point_x, point_y)));
  }

private:
  double pixel_u_;
  double pixel_v_;
  double point_x_;
  double point_y_;
  OcamIntrinsics intrinsic_;
};

struct IntinsicsCost {
public:
  IntinsicsCost(const OcamIntrinsics &intrinsic) : intrinsic_(intrinsic) {}

  template <typename T>
  bool operator()(const T *const param, T *residuals) const {
    T xc = param[0] * T(intrinsic_.cx);
    T yc = param[1] * T(intrinsic_.cy);
    std::vector<T> ss{param[2] * T(intrinsic_.pol[0]), T(0.0)};
    for (size_t i = 3; i <= 5; ++i) {
      ss.push_back(param[i] * T(intrinsic_.pol[i - 1]));
    }

    T c = T(intrinsic_.c);
    T d = T(intrinsic_.d);
    T e = T(intrinsic_.e);

    T rms2 = T(0.0);
    for (int k = 0; k < intrinsic_.image_corners.size(); ++k) {
      for (int i = 0; i < intrinsic_.Xt.rows(); ++i) {
        Eigen::Vector3d w =
            intrinsic_.RRfin[k] *
            Eigen::Vector3d(intrinsic_.Xt(i), intrinsic_.Yt(i), 1);
        Eigen::Matrix<T, 3, 1> Pw = w.template cast<T>();
        Eigen::Matrix<T, 2, 1> reprojected = ReprojectPoint<T>(
            ss, c, d, e, xc, yc, Pw, T(intrinsic_.img_size.width),
            T(intrinsic_.img_size.height));
        Eigen::Matrix<T, 2, 1> orig(T(intrinsic_.image_corners[k][i].y),
                                    T(intrinsic_.image_corners[k][i].x));
        rms2 += (orig - reprojected).template squaredNorm();
      }
    }

    residuals[0] = rms2;
    return true;
  }

  static ceres::CostFunction *Create(const OcamIntrinsics &intrinsic) {
    return (new ceres::AutoDiffCostFunction<IntinsicsCost, 1, 6>(
        new IntinsicsCost(intrinsic)));
  }

private:
  OcamIntrinsics intrinsic_;
};

struct StretchCost {
public:
  StretchCost(const OcamIntrinsics &intrinsic) : intrinsic_(intrinsic) {}

  template <typename T>
  bool operator()(const T *const stretch, T *residuals) const {
    T xc = T(intrinsic_.cx);
    T yc = T(intrinsic_.cy);
    std::vector<T> ss{T(intrinsic_.pol[0]), T(0.0)};
    for (size_t i = 3; i <= 5; ++i) {
      ss.push_back(T(intrinsic_.pol[i - 1]));
    }

    T c = stretch[0];
    T d = stretch[1];
    T e = stretch[2];

    T rms2 = T(0.0);
    for (int k = 0; k < intrinsic_.image_corners.size(); ++k) {
      for (int i = 0; i < intrinsic_.Xt.rows(); ++i) {
        Eigen::Vector3d w =
            intrinsic_.RRfin[k] *
            Eigen::Vector3d(intrinsic_.Xt(i), intrinsic_.Yt(i), 1);
        Eigen::Matrix<T, 3, 1> Pw = w.template cast<T>();
        Eigen::Matrix<T, 2, 1> reprojected = ReprojectPoint<T>(
            ss, c, d, e, xc, yc, Pw, T(intrinsic_.img_size.width),
            T(intrinsic_.img_size.height));
        Eigen::Matrix<T, 2, 1> orig(T(intrinsic_.image_corners[k][i].y),
                                    T(intrinsic_.image_corners[k][i].x));
        rms2 += (orig - reprojected).template squaredNorm();
      }
    }

    residuals[0] = rms2;
    return true;
  }

  static ceres::CostFunction *Create(const OcamIntrinsics &intrinsic) {
    return (new ceres::AutoDiffCostFunction<StretchCost, 1, 3>(
        new StretchCost(intrinsic)));
  }

private:
  OcamIntrinsics intrinsic_;
};
} // calibration
} // nullmax_perception
