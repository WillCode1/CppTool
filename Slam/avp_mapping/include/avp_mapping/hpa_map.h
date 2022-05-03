#pragma once
#include "box.h"
#include "keyframe.h"
#include "quadtree.h"
#include <fstream>
#include <mutex>
#include <vector>

namespace SemanticSLAM
{
  // map point
  struct Node
  {
    int label;
    //  quadtree::Box<float> box;
    Eigen::Vector2f centroid;
    std::size_t id;
    int measurement;
  };

  class HpaMap
  {
  public:
    HpaMap();
    ~HpaMap();

    void Reset();
    void InsertKeyframe(KeyFrame *keyframe);
    void SaveMap(const std::string &filename);

    std::vector<KeyFrame *> GetAllKeyframes();
    void AddSlotPoints(const std::vector<Vec3_t> &points);
    void AddDashPoints(const std::vector<Vec3_t> &points);
    void AddArrowPoint(const std::vector<Vec3_t> &points);
    void AddLanePoint(const std::vector<Vec3_t> &points);
    std::vector<quadtree::SemanticBox> GetQuadTreeBox();
    std::vector<PointTyped> GetSemanticPoints();

  private:
    void SaveBox(std::ofstream &fout, const quadtree::Box &box, const int prob, const int label);
    void WriteKeyframe(std::ofstream &fout, KeyFrame *kf);
    void SaveQuadtreeMap(std::ofstream &fout);
    void SaveKeyframeMap(std::ofstream &fout);

  private:
    bool map_changed_;
    std::mutex mutex_map_;
    std::mutex mutex_keyframe_;
    std::vector<KeyFrame *> keyframes_;
    std::unique_ptr<quadtree::Quadtree<SemanticSLAM::Node *>> quadtree_;
  };
}
