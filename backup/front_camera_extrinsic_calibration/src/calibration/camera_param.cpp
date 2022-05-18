#include "camera_param.h"
#include "fisheye_ocam_param.h"
#include "pinhole_param.h"
#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

namespace nullmax_perception {

CameraParam *CreateCameraParamHandle(const CameraModel &camera_model,
                                     CameraModelParam &camera_model_param) {
  if (FISHEYE_OCAM_MODEL == camera_model) {
    return new FisheyeOcamParam(camera_model_param);
  } else if (FISHEYE_OPENCV_MODEL == camera_model ||
             PINHOLE_MODEL == camera_model)
  // TODO: depart this two model
  {
    return new PinholeParam(camera_model_param);
  } else {
    std::cout << "CreateCameraParamHandle failed \n";
  }
  return NULL;
}

void DestroyCameraParamHandle(CameraParam *handle) {
  if (handle == NULL) {
    return;
  }
  delete handle;
}

} // namespace nullmax_perception
