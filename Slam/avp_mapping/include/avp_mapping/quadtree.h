#pragma once

#include "box.h"
#include "colordef.h"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <memory>
#include <type_traits>
#include <vector>

namespace quadtree
{

  template <typename T>
  class Quadtree
  {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Quadtree(const Box &box) : box_(box), root_(std::unique_ptr<Node>(new Node))
    {
      root_->box = box;
      root_->prob = -1;
      root_->label = -1;
    }

    void Add(const T &value) { Add(root_.get(), 0, box_, value); }
    void Remove(const T &value) { Remove(root_.get(), nullptr, box_, value); }

    std::vector<SemanticBox> GetAllBoxes() { return GetAllBoxes(root_.get()); }
    std::vector<SemanticBox> GetIncrementalBoxes()
    {
      return GetIncrementalBoxes(root_.get());
    }

  private:
    static constexpr auto Threshold = std::size_t(16);
    static constexpr auto MaxDepth = std::size_t(13);

    struct Node
    {
      std::array<std::unique_ptr<Node>, 4> children;
      //    std::vector<T> values;
      Box box;
      int prob;
      int label;
      bool sent;
      Node() : prob(-1), label(-1), sent(false) {}
    };

    Box box_;
    std::unique_ptr<Node> root_;
    bool IsLeaf(const Node *node) const
    {
      return !static_cast<bool>(node->children[0]);
    }

    std::vector<SemanticBox> GetAllBoxes(Node *node)
    {
      std::vector<SemanticBox> boxes;
      boxes.push_back(SemanticBox(node->box, node->label, node->prob));
      if (!IsLeaf(node))
      {
        for (auto &child : node->children)
        {
          std::vector<SemanticBox> child_boxes = GetAllBoxes(child.get());
          boxes.insert(boxes.begin(), child_boxes.begin(), child_boxes.end());
        }
      }
      return boxes;
    }

    std::vector<SemanticBox> GetIncrementalBoxes(Node *node)
    {
      std::vector<SemanticBox> boxes;
      if (!node->sent)
      {
        boxes.push_back(SemanticBox(node->box, node->label, node->prob));
        node->sent = true;
      }
      if (!IsLeaf(node))
      {
        for (auto &child : node->children)
        {
          std::vector<SemanticBox> child_boxes = GetIncrementalBoxes(child.get());
          boxes.insert(boxes.begin(), child_boxes.begin(), child_boxes.end());
        }
      }
      return boxes;
    }

    Box ComputeBox(const Box &box, int i) const
    {
      auto origin = box.GetTopLeft();
      Eigen::Vector2f box_size = box.getSize();
      auto child_size = Eigen::Vector2f(box_size.x() / 2.0f, box_size.y() / 2.0f);
      switch (i)
      {
      // North West
      case 0:
        return Box(origin, child_size);
      // Norst East
      case 1:
        return Box(Eigen::Vector2f(origin.x() + child_size.x(), origin.y()),
                   child_size);
      // South West
      case 2:
        return Box(Eigen::Vector2f(origin.x(), origin.y() + child_size.y()),
                   child_size);
      // South East
      case 3:
        return Box(origin + child_size, child_size);
      default:
        assert(false && "Invalid child index");
        return Box();
      }
    }

    int GetQuadrant(const Box &nodeBox, const Eigen::Vector2f &value_centroid)
    {
      auto center = nodeBox.GetCenter();
      // West
      if (value_centroid.x() < center.x())
      {
        // North West
        if (value_centroid.y() < center.y())
          return 0;
        // South West
        else if (value_centroid.y() >= center.y())
          return 2;
        // Not contained in any quadrant
        else
          return -1;
      }
      // East
      else if (value_centroid.x() >= center.x())
      {
        // North East
        if (value_centroid.y() < center.y())
          return 1;
        // South East
        else if (value_centroid.y() >= center.y())
          return 3;
        // Not contained in any quadrant
        else
          return -1;
      }
      // Not contained in any quadrant
      else
        return -1;
    }

    void Add(Node *node, std::size_t depth, const Box &box, const T &value)
    {
      assert(node != nullptr);
      if (!box.Contains(value->centroid))

        return;

      // assert(box.contains(value->centroid));

      if (IsLeaf(node))
      {

        // Insert the value in this node if possible
        if (depth >= MaxDepth)
        {

          // the final leave

          if (node->label == value->label)
            node->prob += value->measurement;
          else
          {
            node->label = value->label;
            node->prob = value->measurement;
          }
        }
        // Otherwise, we split and we try again
        else
        {
          Split(node, box);
          Add(node, depth, box, value);
        }
      }
      else
      {
        auto i = GetQuadrant(box, value->centroid);
        // Add the value in a child if the value is entirely contained in it
        if (i != -1)
        {
          auto child_box = ComputeBox(box, i);
          Add(node->children[i].get(), depth + 1, child_box, value);
        }
        else
        {
          // this would not happen is this case
          //        node->values.push_back(value);
        }
      }
    }

    void Split(Node *node, const Box &box)
    {
      assert(node != nullptr);
      assert(IsLeaf(node) && "Only leaves can be split");
      // Create children
      for (auto &child : node->children)
      {
        child = std::unique_ptr<Node>(new Node());
      }

      auto node_box_origin = box.GetTopLeft();
      Eigen::Vector2f box_size = box.getSize();
      auto child_box_size = Eigen::Vector2f(box_size.x() / 2.0f, box_size.y() / 2.0f);
      node->children[0]->box = Box(node_box_origin, child_box_size);
      node->children[1]->box = Box(Eigen::Vector2f(node_box_origin.x() + child_box_size.x(), node_box_origin.y()), child_box_size);
      node->children[2]->box = Box(Eigen::Vector2f(node_box_origin.x(), node_box_origin.y() + child_box_size.y()), child_box_size);
      node->children[3]->box = Box(node_box_origin + child_box_size, child_box_size);
    }
  };
}
