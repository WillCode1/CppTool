#pragma once
#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"

namespace FeatureSLAM {

// question: 为什么是这个词袋模型, TDescriptor?? DBoW2::FORB?? 词袋模型的使用训练??
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
    ORBVocabulary;
}
