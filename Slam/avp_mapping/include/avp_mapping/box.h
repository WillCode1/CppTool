#pragma once

#include "Eigen/Core"

namespace quadtree
{
  class Box
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    float left_;
    float top_;
    float width_;  // Must be positive
    float height_; // Must be positive

    Box(float left = 0, float top = 0, float width = 0, float height = 0) noexcept
        : left_(left),
          top_(top),
          width_(width),
          height_(height) {}

    Box(const Eigen::Vector2f &position, const Eigen::Vector2f &size) noexcept
        : left_(position.x()),
          top_(position.y()),
          width_(size.x()),
          height_(size.y()) {}

    float GetRight() const noexcept { return left_ + width_; }

    float GetBottom() const noexcept { return top_ + height_; }

    Eigen::Vector2f GetTopLeft() const noexcept
    {
      return Eigen::Vector2f(left_, top_);
    }

    Eigen::Vector2f GetCenter() const noexcept
    {
      return Eigen::Vector2f(left_ + width_ / 2.0f, top_ + height_ / 2.0f);
    }

    Eigen::Vector2f getSize() const noexcept
    {
      return Eigen::Vector2f(width_, height_);
    }

    bool Contains(const Box &box) const noexcept
    {
      return left_ <= box.left_ && box.GetRight() <= GetRight() &&
             top_ <= box.top_ && box.GetBottom() <= GetBottom();
    }

    bool Contains(const Eigen::Vector2f &centroid) const noexcept
    {
      return left_ <= centroid.x() && centroid.x() <= GetRight() &&
             top_ <= centroid.y() && centroid.y() <= GetBottom();
    }

    bool Intersects(const Box &box) const noexcept
    {
      return !(left_ >= box.GetRight() || GetRight() <= box.left_ ||
               top_ >= box.GetBottom() || GetBottom() <= box.top_);
    }
  };

  class SemanticBox
  {
  public:
    SemanticBox(const Box &box, int label, int prob)
        : box_(box), label_(label), prob_(prob){};
    ~SemanticBox(){};
    Box box_;
    int label_;
    int prob_;
  };
}
