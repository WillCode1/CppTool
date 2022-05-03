// Have read
#include "hpa_map.h"
#include "colordef.h"
#include "log.h"

namespace SemanticSLAM
{
  HpaMap::HpaMap() : map_changed_(false)
  {
    quadtree::Box map_region = quadtree::Box(-100.0f, -100.0f, 300.0f, 300.0f);
    quadtree_ = std::unique_ptr<quadtree::Quadtree<SemanticSLAM::Node *>>(new quadtree::Quadtree<SemanticSLAM::Node *>(map_region));
  }

  HpaMap::~HpaMap() {}

  /**
   * @brief Map::Reset  reset the map: delete all keyframe and quadtree
   *
   */
  void HpaMap::Reset()
  {
    std::lock_guard<std::mutex> lock(mutex_keyframe_);
    std::lock_guard<std::mutex> lock2(mutex_map_);

    for (auto kf : keyframes_)
    {
      if (kf != nullptr)
      {
        delete kf;
      }
    }
    keyframes_.clear();
    map_changed_ = false;

    // Reset the quadtree
    quadtree_.reset();
    quadtree::Box map_region = quadtree::Box(-100.0f, -100.0f, 300.0f, 300.0f);
    quadtree_ = std::unique_ptr<quadtree::Quadtree<SemanticSLAM::Node *>>(new quadtree::Quadtree<SemanticSLAM::Node *>(map_region));
  }

  void HpaMap::InsertKeyframe(KeyFrame *keyframe)
  {
    std::lock_guard<std::mutex> lock(mutex_keyframe_);
    keyframes_.push_back(keyframe);
    map_changed_ = true;
  }

  void HpaMap::SaveMap(const std::string &filename)
  {
    std::lock_guard<std::mutex> lock(mutex_keyframe_);
    std::lock_guard<std::mutex> lock2(mutex_map_);
    if (!map_changed_)
    {
      std::cout << kColorRed << " map has already saved " << kColorReset << std::endl;
      return;
    }

    std::ofstream fout;
    fout.open(filename, std::ios_base::out | std::ios::binary);
    if (!fout.is_open())
    {
      NM_ERROR("Save HPA MAP ERROR ")
      return;
    }

    SaveQuadtreeMap(fout);
    SaveKeyframeMap(fout);
    fout.close();

    map_changed_ = false;
  }

  std::vector<KeyFrame *> HpaMap::GetAllKeyframes()
  {
    std::lock_guard<std::mutex> lock(mutex_keyframe_);
    return keyframes_;
  }

  void HpaMap::AddSlotPoints(const std::vector<Vec3_t> &points)
  {
    std::lock_guard<std::mutex> lock(mutex_map_);

    for (auto &p : points)
    {
      SemanticSLAM::Node node;
      node.centroid = Eigen::Vector2f(p.x(), p.y());
      node.measurement = 1;
      node.label = PARKING_SLOT; // parking slot
      quadtree_->Add(&node);
    }
  }

  void HpaMap::AddDashPoints(const std::vector<Vec3_t> &points)
  {
    std::lock_guard<std::mutex> lock(mutex_map_);

    for (auto &p : points)
    {
      SemanticSLAM::Node node;
      node.centroid = Eigen::Vector2f(p.x(), p.y());
      node.measurement = 1;
      node.label = DASH; // dash
      quadtree_->Add(&node);
    }
  }

  void HpaMap::AddArrowPoint(const std::vector<Vec3_t> &points)
  {
    std::lock_guard<std::mutex> lock(mutex_map_);

    for (auto &p : points)
    {
      SemanticSLAM::Node node;
      node.centroid = Eigen::Vector2f(p.x(), p.y());
      node.measurement = 1;
      node.label = ARROW; // arrow
      quadtree_->Add(&node);
    }
  }

  void HpaMap::AddLanePoint(const std::vector<Vec3_t> &points)
  {
    std::lock_guard<std::mutex> lock(mutex_map_);

    for (auto &p : points)
    {
      SemanticSLAM::Node node;
      node.centroid = Eigen::Vector2f(p.x(), p.y());
      node.measurement = 1;
      node.label = LANE; // arrow
      quadtree_->Add(&node);
    }
  }

  void HpaMap::SaveBox(std::ofstream &fout, const quadtree::Box &box, const int prob, const int label)
  {
    auto tl = box.GetTopLeft();
    auto size = box.getSize();

    double x0 = tl.x();
    double y0 = tl.y();
    double x1 = tl.x() + size.x();
    double y1 = tl.y() + size.y();

    fout.write((char *)&x0, sizeof(double));
    fout.write((char *)&y0, sizeof(double));
    fout.write((char *)&x1, sizeof(double));
    fout.write((char *)&y1, sizeof(double));
    fout.write((char *)&prob, sizeof(int));
    fout.write((char *)&label, sizeof(int));
  }

  void HpaMap::SaveQuadtreeMap(std::ofstream &fout)
  {
    std::vector<quadtree::SemanticBox> semantic_box = quadtree_->GetAllBoxes();

    long int node_size = semantic_box.size();
    fout.write((char *)&node_size, sizeof(node_size));

    for (auto &single_box : semantic_box)
    {
      int prob = single_box.prob_;
      auto box = single_box.box_;
      int label = single_box.label_;
      SaveBox(fout, box, prob, label);
    }
  }

  void HpaMap::SaveKeyframeMap(std::ofstream &fout)
  {
    const int keyframe_count = static_cast<int>(keyframes_.size());
    fout.write((char *)&keyframe_count, sizeof(int));

    for (auto kf : keyframes_)
    {
      WriteKeyframe(fout, kf);
    }
    // Save keyframe connections
    // id  num_connected_keyframes
    for (auto kf : keyframes_)
    {
      fout.write((char *)&kf->id_, sizeof(kf->id_));
      auto connected_keyframes = kf->GetConnectedKeyframes();
      int num_connected_keyframes = connected_keyframes.size();
      fout.write((char *)&num_connected_keyframes, sizeof(int));
      for (auto ckf : connected_keyframes)
      {
        fout.write((char *)&ckf->id_, sizeof(ckf->id_));
      }
    }
  }

  void HpaMap::WriteKeyframe(std::ofstream &fout, KeyFrame *kf)
  {
    const int id = kf->id_;
    fout.write((char *)&id, sizeof(int));
    // write pose
    Mat33_t trans_world2base = kf->trans_world2base_;
    double tx = trans_world2base(0, 2);
    double ty = trans_world2base(1, 2);
    double theta = atan2(trans_world2base(1, 0), trans_world2base(0, 0));

    fout.write((char *)&tx, sizeof(double));
    fout.write((char *)&ty, sizeof(double));
    fout.write((char *)&theta, sizeof(double));

    // question: arrow_points_这个不用保存?

    // save slot point
    int slot_point_num = kf->slot_point_num_;
    fout.write((char *)&slot_point_num, sizeof(int));
    for (auto &p : kf->slot_points_)
    {
      fout.write((char *)&p.u, sizeof(p.u));
      fout.write((char *)&p.v, sizeof(p.v));
      fout.write((char *)&p.x, sizeof(p.x));
      fout.write((char *)&p.y, sizeof(p.y));
      fout.write((char *)&p.prob, sizeof(p.prob));
    }

    // save dash point
    int dash_point_num = kf->dash_point_num_;
    fout.write((char *)&dash_point_num, sizeof(int));
    for (auto &p : kf->dash_points_)
    {
      fout.write((char *)&p.u, sizeof(p.u));
      fout.write((char *)&p.v, sizeof(p.v));
      fout.write((char *)&p.x, sizeof(p.x));
      fout.write((char *)&p.y, sizeof(p.y));
      fout.write((char *)&p.prob, sizeof(p.prob));
    }

    // save lane point
    int lane_point_num = kf->lane_point_num_;
    fout.write((char *)&lane_point_num, sizeof(int));
    for (auto &p : kf->lane_points_)
    {
      fout.write((char *)&p.u, sizeof(p.u));
      fout.write((char *)&p.v, sizeof(p.v));
      fout.write((char *)&p.x, sizeof(p.x));
      fout.write((char *)&p.y, sizeof(p.y));
      fout.write((char *)&p.prob, sizeof(p.prob));
    }
  }

  std::vector<quadtree::SemanticBox> HpaMap::GetQuadTreeBox()
  {
    std::lock_guard<std::mutex> lock(mutex_map_);
    return quadtree_->GetAllBoxes();
  }

  std::vector<PointTyped> HpaMap::GetSemanticPoints()
  {
    std::lock_guard<std::mutex> lock(mutex_map_);
    const std::vector<quadtree::SemanticBox> boxes = quadtree_->GetIncrementalBoxes();
    std::vector<PointTyped> semantic_points;
    for (auto &box : boxes)
    {
      if (box.prob_ < 0)
        continue;
      PointTyped p;
      auto origin = box.box_.GetTopLeft();
      auto size = box.box_.getSize();
      p.point = Vec3_t(origin.x() + 0.5 * size.x(), origin.y() + 0.5 * size.y(), 0);
      p.type = box.label_;
      semantic_points.push_back(p);
    }
    return semantic_points;
  }
}
