#include "mono_intrinsic_interface.h"
#include <iostream>
#include <string>
using namespace nullmax_perception::calibration;

int main(int argc, char **argv) {
  std::string config_path = "../config/config.cfg";

  auto calibrator = CreateCalibrator();

  if (!calibrator->Init(config_path)) {
    std::cerr << "calibrator init failed" << std::endl;
    return 0;
  }

  if (!calibrator->Calib()) {
    std::cerr << "calibration failed" << std::endl;
    return 0;
  }

  return 1;
}