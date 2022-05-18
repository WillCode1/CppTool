#include "front_camera_extrinsic_adjustment_base.h"
#include "front_camera_extrinsic_adjustment_manual.h"
#include <algorithm>
#include <string>
#include <vector>

namespace nullmax_perception {

FrontCameraExtrinsicAdjustmentBase *
CreateFrontCameraExtrinsicAdjustmentHandle(const std::string &method) {
  std::string method_lower = method;
  transform(method.begin(), method.end(), method_lower.begin(), ::tolower);
  if ("manual" == method_lower) {
    return new FrontCameraExtrinsicAdjustmentManual();
  }
  return NULL;
}

void DestroyFrontCameraExtrinsicAdjustmentHandle(
    FrontCameraExtrinsicAdjustmentBase *handle) {
  if (handle == NULL) {
    return;
  }
  delete handle;
}

} // namespace nullmax_perception
