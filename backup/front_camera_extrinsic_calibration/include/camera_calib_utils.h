#ifndef CAMERA_CALIB_UTILS_
#define CAMERA_CALIB_UTILS_
#include <vector>

namespace nullmax_perception {

union CameraIntrinsic {
  float data[4];
  struct {
    float fx; // focal_length_x
    float fy; // focal_length_y
    float cx; // row coordinate of the center
    float cy; // column coordinate of the center
  };
};

union CameraDistortCoef {
  float coef[5]; // distortion coefficient
  struct {
    float k1;
    float k2;
    float p1;
    float p2;
    float k3;
  };
};

struct CameraPosition {
  float x;
  float y;
  float z;
};

struct CameraRotationEuler {
  CameraRotationEuler() : pitch(0.0), yaw(0.0), roll(0.0) {}

  float pitch;
  float yaw;
  float roll;
};

struct RoiData {
  // Fields for Calibration module
  int width_top;
  int width_bottom;
  int height_min;
  int height_max;
  // Fields for Adjustment module
  float roi1_left;
  float roi1_right;
  float roi1_top;
  float roi1_bottom;
  float roi2_theta;
  float roi2_rho;
};

struct LaneCoef {
  bool valid;
  float coef_k;
  float coef_b;
};

struct VanishPoint {
  bool valid;
  float x;
  float y;
};



enum CameraModel {
  FISHEYE_OCAM_MODEL = 0,
  FISHEYE_OPENCV_MODEL = 1,
  PINHOLE_MODEL = 2,
};

struct UVIPMRoi {
  float left;
  float right;
  float top;
  float bottom;
};

} // namespace nullmax_perception
#endif // CAMERA_CALIB_UTILS_
