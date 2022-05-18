#ifndef TRANSFORM_IPM_H
#define TRANSFORM_IPM_H

#include "util_structure.h"
using namespace nullmax_calibration;

namespace tf_ipm {

// tf-(ground <-> ipm) is already efficient in tf_ipm
// pts(ground <-> ipm) is already efficient in tf_ipm

// Transforms points from the ground plane (z=-h) in the world frame
// into points on the image in image frame (uv-coordinates)
// mat_ground, mat_uv: matrix 2xN: 1st row: x, 2nd row: y
// void TransformGround2Image(const EigenMat &mat_ground, EigenMat &mat_uv,
//                            const CameraInfo &camera_info);

// specify the ground height (-1.0*cemera_height)
void TransformGround2Image(const EigenMat &mat_ground,
                           const EigenMat &mat_tf_ground2uv,
                           const float ground_height, EigenMat &mat_uv);

// Transforms points from the image frame (uv-coordinates)
// into the real world frame on the ground plane (z=-height)
// mat_ground, mat_uv: matrix 2xN: 1st row: x, 2nd row: y
// void TransformImage2Ground(const EigenMat &mat_uv, EigenMat &mat_ground,
//                            const CameraInfo &camera_info);

void TransformImage2Ground(const EigenMat &mat_uv,
                           const EigenMat &mat_tf_uv2ground,
                           EigenMat &mat_ground);

// converts from IPM pixel coordinates into world coordinates
// mat_ground, mat_ipm: matrix 2xN: 1st row: x, 2nd row: y
void TransformImIPM2Ground(const EigenMat &mat_ipm, EigenMat &mat_ground,
                           const IPMInfo &ipm_info);

// converts from world coordinates into IPM pixel coordinates
// mat_ground, mat_ipm: matrix 2xN: 1st row: x, 2nd row: y
void TransformGround2ImIPM(const EigenMat &mat_ground, EigenMat &mat_ipm,
                           const IPMInfo &ipm_info);

// converts from IPM pixel coordinates into Image coordinates
// mat_uv, mat_ipm: matrix 2xN: 1st row: x, 2nd row: y
// void TransformImIPM2Im(const EigenMat &mat_ipm, EigenMat &mat_uv,
//                        const IPMInfo &ipm_info, const CameraInfo
//                        &camera_info);

// Converts from Image coordinates into IPM pixel coordinates
// mat_uv, mat_ipm: matrix 2xN: 1st row: x, 2nd row: y
// void TransformIm2ImIPM(const EigenMat &mat_uv, EigenMat &mat_ipm,
//                        const IPMInfo &ipm_info, const CameraInfo
//                        &camera_info);

// Computes the vanishing point in the image plane uv. It is
// the point of intersection of the image plane with the line
// in the XY-plane in the world coordinates that makes an
// angle yaw clockwise (form Y-axis) with Y-axis
nullmax_calibration::Point2f GetVanishingPoint(const CameraInfo &camera_info);
nullmax_calibration::Point2f
GetVanishingPointOpt(const CameraInfo &camera_info);

EigenMat PrepareTfUV2Ground(const CameraInfo &camera_info_);
EigenMat PrepareTfGround2UV(const CameraInfo &camera_info_);

// compute ipm image from uv image by Inverse Perspective Mapping
// of the input image, assuming a flat ground plane
// image_ipm is created outside; then will update ipm_info inside
// void GetIPM(const EigenMat& image_uv, EigenMat& image_ipm, IPMInfo &ipm_info,
// const CameraInfo &camera_info);

} // namespace tf_ipm

#endif // TRANSFORM_IPM_H
