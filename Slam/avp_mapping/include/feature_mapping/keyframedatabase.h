#pragma once

#include <list>
#include <set>
#include <vector>

#include "frame.h"
#include "keyframe.h"
#include "orbvocabulary.h"
#include <mutex>
namespace FeatureSLAM {

class KeyFrame;
class Frame;

class KeyFrameDatabase {
public:
  KeyFrameDatabase(const ORBVocabulary &voc);

  void add(KeyFrame *pKF);
  //   void addMC(MCKeyFrame *pMCKF);

  void erase(KeyFrame *pKF);

  void clear();

  // Loop Detection
  std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame *pKF, float minScore);
  //   std::vector<MCKeyFrame*> DetectLoopCandidatesMC(MCKeyFrame *pKF, float
  //   minScore);

  // Relocalization
  std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame *F);
  std::vector<KeyFrame *> DetectMapFusionCandidates(KeyFrame *pKF);

protected:
  // Associated vocabulary
  const ORBVocabulary *mpVoc; ///< 预先训练好的词典

  // Inverted file
  std::vector<list<KeyFrame *>>
      mvInvertedFile; ///< 倒排索引，mvInvertedFile[i]表示包含了第i个word
                      /// id的所有关键帧
  //  std::vector<list<MCKeyFrame*> >mvInvertedMCFile;
  //  std::set<MCKeyFrame* > mspMCKFs;

  // Mutex
  std::mutex mMutex;
};
}
