#pragma once

#include <iostream>
#include <memory>
#include <string>

namespace nullmax_perception {
namespace calibration {

class MonoIntrinsicInterface {
public:
  virtual ~MonoIntrinsicInterface(){};

  /* Brief: when calibrating, init calibrator form .yaml file
   * Param: config_file: file path and name .yaml
   * Return: if true, initialization success
   */
  virtual bool Init(const std::string &config_file) = 0;

  /* Brief: do mono intrinsic calibration
   * Return: if true, calibration is finshed
   */
  virtual bool Calib() = 0;

  /* Brief: view undistort images
   * Param: distorted_image_file: absolute path and name of distorted image
   *        intrinsic_file: absolute path and name of intrinsic file
   * Return: if true, no file error
   */
  virtual bool ViewUndistort(const std::string &distorted_image_file,
                             const std::string &intrinsic_file) = 0;
};

/* Brief: create interface object
 */
std::shared_ptr<MonoIntrinsicInterface> CreateCalibrator();

} // namespace calibration
} // namespace nullmax_perception
