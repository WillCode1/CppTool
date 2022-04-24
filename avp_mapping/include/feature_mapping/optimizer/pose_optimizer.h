#pragma once

#include "frame.h"
#include "keyframe.h"
#include "map.h"
#include "mappoint.h"
#include "mcframe.h"

#include "g2o/types/se3_ops.h"
#include "g2o/types/types_seven_dof_expmap.h"
namespace FeatureSLAM {

class PoseOptimizer {
public:
  explicit PoseOptimizer(const unsigned int num_trials,
                         const unsigned int num_each_iter);
  virtual ~PoseOptimizer() = default;
  int PoseOptimization(Frame *frame) const;
  int PoseOptimizationMC(MCFrame *mc_frm) const;

private:
  unsigned int num_trials_;
  int num_each_iter_;

  // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise
  // (mono)
  //  void static OptimizeEssentialGraph(
  //      Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
  //      const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
  //      const LoopClosing::KeyFrameAndPose &CorrectedSim3,
  //      const map<KeyFrame *, set<KeyFrame *>> &LoopConnections,
  //      const bool &bFixScale);
};
}
