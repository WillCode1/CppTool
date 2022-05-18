#ifndef UTIL_STRUCTURE_H
#define UTIL_STRUCTURE_H

#include <Eigen/Dense>
#include <iostream>

#define PI 3.14159265

namespace nullmax_calibration {

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    EigenMat;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    Matd;

struct Size {
  Size() : width(0), height(0) {}
  Size(int _w, int _h) : width(_w), height(_h) {}

  int width;
  int height;
};

template <typename _Tp> struct Point_ {
  Point_() : x(0), y(0) {}
  Point_(_Tp _x, _Tp _y) : x(_x), y(_y) {}

  _Tp x;
  _Tp y;
};

struct Rect {
  Rect() : x(0), y(0), width(0), height(0) {}
  Rect(int _x, int _y, int _w, int _h) : x(_x), y(_y), width(_w), height(_h) {}
  int x, y, width, height;
};

template <typename _Tp>
std::ostream &operator<<(std::ostream &os, const Point_<_Tp> &pt) {
  os << "[" << pt.x << ", " << pt.y << "]" << std::endl;
  return os;
}

typedef Point_<int> Point;
typedef Point_<float> Point2f;

template <typename _Tp>
class CmpPtsY : public std::binary_function<Point, Point, bool> {
public:
  CmpPtsY() {}
  inline bool operator()(const Point_<_Tp> &pts1,
                         const Point_<_Tp> &pts2) const {
    if (pts1.y < pts2.y)
      return true;
    else
      return false;
  }
};

/**
 * Structure to hold the info about IPM transformation
 */
typedef struct IPMInfo {
  /// min and max x-value on ground in world coordinates
  float limits_x[2];
  /// min and max y-value on ground in world coordinates
  float limits_y[2];
  /// conversion between mm in world coordinate on the ground
  /// in x-direction and pixel in image
  float scale_x;
  /// conversion between mm in world coordinate on the ground
  /// in y-direction and pixel in image
  float scale_y;
  /// width of ipm image
  int width_ipm;
  /// height of ipm image
  int height_ipm;
  /// portion of image height to add to y-coordinate of
  /// vanishing point
  float vp_portion;
  /// Left point in original image of region to make IPM for
  float roi_uv_left;
  /// Right point in original image of region to make IPM for
  float roi_uv_right;
  /// Top point in original image of region to make IPM for
  float roi_uv_top;
  /// Bottom point in original image of region to make IPM for
  float roi_uv_bottom;
  /// interpolation to use for IPM (0: bilinear, 1:nearest neighbor)
  int ipm_interpolation;

  void PrintSelf() {
    std::cout << "-------------ipm info -------------" << std::endl;
    std::cout << " limits_x: " << limits_x[0] << " - " << limits_x[1]
              << std::endl;
    std::cout << " limits_y: " << limits_y[0] << " - " << limits_y[1]
              << std::endl;
    std::cout << " scale_x = " << scale_x << std::endl;
    std::cout << " scale_y = " << scale_x << std::endl;
    std::cout << " width_ipm = " << width_ipm << std::endl;
    std::cout << " height_ipm = " << height_ipm << std::endl;

    std::cout << " roi_uv_left = " << roi_uv_left << std::endl;
    std::cout << " roi_uv_right = " << roi_uv_right << std::endl;
    std::cout << " roi_uv_top = " << roi_uv_top << std::endl;
    std::cout << " roi_uv_bottom = " << roi_uv_bottom << std::endl;

    std::cout << " vp_portion = " << vp_portion << std::endl;
    std::cout << " ipm_interpolation = " << ipm_interpolation << std::endl;
    std::cout << "------------------------------------" << std::endl;
  }

} IPMInfo;

// Camera Calibration info
typedef struct CameraInfo {
  /// focal length in x and y
  Point2f focal_length;
  /// optical center coordinates in image frame (origin is (0,0) at top left)
  Point2f optical_center;
  /// height of camera above ground
  float camera_height;
  /// pitch angle in radians (+ve downwards)
  float angle_pitch;
  /// yaw angle in radians (+ve clockwise)
  float angle_yaw;
  /// roll rangle in radians
  float angle_roll;
  /// width of images
  float width_image;
  /// height of images
  float height_image;

  // resize and cut the image
  float scale_x;
  float scale_y;
  float cut_x;
  float cut_y;

  void PrintSelf() {
    std::cout << "------------- camera info -------------" << std::endl;
    std::cout << " focal_length.x = " << focal_length.x << std::endl;
    std::cout << " focal_length.y = " << focal_length.y << std::endl;
    std::cout << " optical_center.x = " << optical_center.x << std::endl;
    std::cout << " optical_center.y = " << optical_center.y << std::endl;
    std::cout << " camera_height = " << camera_height << std::endl;
    std::cout << " angle_pitch = " << angle_pitch * 180.0 / PI << std::endl;
    std::cout << " angle_yaw = " << angle_yaw * 180.0 / PI << std::endl;
    std::cout << " angle_roll = " << angle_roll * 180.0 / PI << std::endl;
    std::cout << " width_image = " << width_image << std::endl;
    std::cout << " height_image = " << height_image << std::endl;
    std::cout << "---------------------------------------" << std::endl;
  }
} CameraInfo;
/// Line structure with start and end points
typedef struct Line {
  Line(){};

  Line(int x_start, int y_start, int x_end, int y_end) {
    start_point.x = x_start;
    start_point.y = y_start;
    end_point.x = x_end;
    end_point.y = y_end;
  }
  Line(Point2f start_point_in, Point2f end_point_in) {
    start_point = start_point_in;
    end_point = end_point_in;
  }
  /// start/end point
  Point2f start_point, end_point;
  /// color of line
  // LineColor color;
  /// score of line
  float score;
} Line;
} // namespace nullmax_calibration

#endif // UTIL_STRUCTURE_H
