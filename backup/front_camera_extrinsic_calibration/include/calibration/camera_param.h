#ifndef CAMERA_PARAM_H_
#define CAMERA_PARAM_H_

#include "camera_calib_utils.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace nullmax_perception {

struct PinholeModel {
  CameraIntrinsic intrinsic;
  CameraIntrinsic intrinsic_undistort;
  CameraDistortCoef distort;
  int width;  // image width
  int height; // image height
};

struct OcamModel {
  std::vector<double> pol;    // the polynomial coefficients: pol[0] + x"pol[1]
                              // + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
  int length_pol;             // length of polynomial
  std::vector<double> invpol; // the coefficients of the inverse polynomial
  int length_invpol;          // length of inverse polynomial
  double xc;                  // row coordinate of the center
  double yc;                  // column coordinate of the center
  double c;                   // affine parameter
  double d;                   // affine parameter
  double e;                   // affine parameter
  int width;                  // image width
  int height;                 // image height
};

struct CameraModelParam {
  OcamModel ocam;
  PinholeModel pinhole;
};

class CameraParam {
public:
  virtual ~CameraParam(){};

  virtual void Load(const std::string &calbration_file) = 0;

  virtual cv::Point2f World2Camera(cv::Point3f pt3) = 0;
  virtual cv::Point3f Camera2World(cv::Point2f pt2) = 0;

  // NOTE:
  // if fisheye ocam model call Undistortion() will reproject in pinhole model
  // if fisheye opencv model call Undistortion() will reproject in new pinhole
  // model
  virtual int Undistortion(const cv::Mat &src, cv::Mat &dst) = 0;

  virtual CameraModel getCameraModelType() = 0;
  virtual CameraModelParam getCameraModelParam() = 0;
};

CameraParam *CreateCameraParamHandle(const CameraModel &camera_model,
                                     CameraModelParam &camera_model_param);
void DestroyCameraParamHandle(CameraParam *handle);

} // namespace nullmax_perception
#endif // CAMERA_PARAM_H_
