#pragma once

#include "frame.h"
#include "g2o/types/se3_ops.h"
#include "g2o/types/types_seven_dof_expmap.h"
#include "keyframe.h"
#include "map.h"
#include "mappoint.h"
#include "mcframe.h"
#include "multicam_ext.h"
namespace FeatureSLAM {

class Map;

class FullBundleAdjuster {

public:
  explicit FullBundleAdjuster(Map *map, unsigned int iterations,
                              MultiCam *multicam);
  ~FullBundleAdjuster() = default;

  void Optimize() const;

private:
  Map *map_;
  unsigned int num_iter_;
  MultiCam *multicam_;
};
}
