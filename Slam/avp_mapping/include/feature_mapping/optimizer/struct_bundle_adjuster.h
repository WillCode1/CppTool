#pragma once

#include "frame.h"
#include "g2o/types/se3_ops.h"
#include "keyframe.h"
#include "map.h"
#include "mappoint.h"
#include "mcframe.h"
#include "multicam_ext.h"
#include "timer.h"
#include "viewerconfig.h"

namespace FeatureSLAM {

class StructBundleAdjuster {
public:
  explicit StructBundleAdjuster(unsigned int num_first_iter = 5,
                                unsigned int num_second_iter = 10);
  virtual ~StructBundleAdjuster() = default;

  void Optimize(MCKeyFrame *current_mc_keyfrm, Map *map) const;

private:
  unsigned int num_first_iter_;
  unsigned int num_second_iter_;
};
}
