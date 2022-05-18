#include "front_camera_extrinsic_calib_base.h"

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

#include "front_camera_extrinsic_calib_use_lane.h"

namespace nullmax_perception {

FrontCameraExtrinsicCalibBase *
CreateFrontCameraExtrinsicCalibHandle(const std::string &method) {
  std::string method_lower = method;
  transform(method.begin(), method.end(), method_lower.begin(), ::tolower);
  if ("uselane" == method_lower) {
    return new FrontCameraExtrinsicCalibUseLane();
  } else {
    std::cout << "CreateFrontCameraExtrinsicCalibHandle failed \n";
  }

  return NULL;
}

void DestroyFrontCameraExtrinsicCalibHandle(
    FrontCameraExtrinsicCalibBase *handle) {
  if (handle == NULL) {
    return;
  }
  delete handle;
}

} // namespace nullmax_perception
