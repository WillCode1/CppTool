#include "mono_intrinsic_interface.h"
#include <iostream>
#include <string>
using namespace nullmax_perception::calibration;

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cout << "Usage: view_undistorted IMAGE_FILE INTRINSIC_FILE"
              << std::endl;
    return 0;
  }

  std::string config_path = "../config/config.cfg";
  auto calibrator = CreateCalibrator();

  if (!calibrator->Init(config_path)) {
    std::cerr << "calibrator init failed" << std::endl;
    return 0;
  }

  return calibrator->ViewUndistort(argv[1], argv[2]);
}