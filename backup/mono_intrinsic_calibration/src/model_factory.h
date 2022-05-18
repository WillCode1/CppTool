#pragma once
#include "fisheye.h"
#include "model_base.h"
#include "ocam.h"
#include "pinhole.h"

namespace nullmax_perception {
namespace calibration {
class ModelFactory {
public:
  ModelFactory() {}
  ~ModelFactory() {}
  ModelFactory &operator=(const ModelFactory &) = delete;
  ModelFactory(const ModelFactory &) = delete;

  static ModelBase::Ptr CreateModel(const ModelBase::CameraConfig &config) {
    ModelBase::Ptr camera_model = nullptr;
    if (config.calib_type == "pinhole") {
      camera_model = std::make_shared<Pinhole>(config);
    } else if (config.calib_type == "fisheye") {
      camera_model = std::make_shared<Fisheye>(config);
    } else if (config.calib_type == "ocam") {
      camera_model = std::make_shared<Ocam>(config);
    } else {
      std::cerr << "unknown mode type" << std::endl;
    }
    return camera_model;
  }
};
} // calibration
} // nullmax_perception