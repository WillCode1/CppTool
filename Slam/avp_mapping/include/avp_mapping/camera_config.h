#pragma once

struct CameraIntrinsic {
  int image_width;
  int image_height;

  double fx;
  double fy;
  double cx;
  double cy;
};

struct CameraExtrinsic {
  // for bev
  double scale;
  double baselink2cam;
  double camera_height;
};

struct CameraConfig {
  CameraIntrinsic cam_intrinsic;
  CameraExtrinsic cam_extrinsic;
};

struct VehicleBodyMask {
  int left;
  int top;
  int width;
  int height;
};
