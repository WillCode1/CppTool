#ifndef G2O_EDGE_SE2_SEMANTIC_PROJECT_H
#define G2O_EDGE_SE2_SEMANTIC_PROJECT_H

#include "camera_parameter.h"
#include "g2o/core/base_unary_edge.h"
#include "se2.h"
#include "vertex_se2.h"
#include <opencv2/opencv.hpp>

namespace g2o
{

  class EdgeSE2SemanticProject : public BaseUnaryEdge<1, double, VertexSE2>
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE2SemanticProject(cv::Mat *image, Eigen::Vector3d wp, double cam_height, double baselink2cam)
        : image_(image), wp_(wp), cam_height_(cam_height),
          baselink2cam_(baselink2cam)
    {
      wp_.z() = 1;
      coef_ = 1.0 / 255;
    }
    EdgeSE2SemanticProject()
        : image_(nullptr), cam_height_(4.0), baselink2cam_(1.3), coef_(1.0 / 255) {}
    void SetImagePtr(cv::Mat *image) { image_ = image; };
    void SetWorldPoint(Eigen::Vector3d wp)
    {
      wp_ = wp;
      wp_.z() = 1;
    }
    void SetCameraExtrinsic(double cam_height, double baselink2cam)
    {
      cam_height_ = cam_height;
      baselink2cam_ = baselink2cam;
    }
    ~EdgeSE2SemanticProject() {}

    virtual void computeError()
    {
      const VertexSE2 *v = static_cast<const VertexSE2 *>(_vertices[0]);

      Eigen::Vector3d pose = v->estimate().toVector();
      double theta = pose.z();
      double inv_x = -cos(theta) * pose.x() - sin(theta) * pose.y();
      double inv_y = sin(theta) * pose.x() - cos(theta) * pose.y();
      double inv_theta = -theta;

      Eigen::Matrix3d inv_pose;
      inv_pose << cos(inv_theta), -sin(inv_theta), inv_x, sin(inv_theta), cos(inv_theta), inv_y, 0, 0, 1;

      Eigen::Vector3d point_vehicle = inv_pose * wp_;

      Vector3D point_camera(-point_vehicle.y(), -point_vehicle.x() + baselink2cam_, cam_height_);
      Vector2D uv = cam_->cam_map(point_camera);

      if (uv.x() - 4 < 0 || (uv.x() + 4) > image_->cols || (uv.y() - 4) < 0 || (uv.y() + 4) > image_->rows)
      {
        _error(0, 0) = 0.0;
        this->setLevel(1);
      }
      else
      {
        float pixel_value = getPixelValue(uv.x(), uv.y());
        if (pixel_value < 10)
        {
          _error(0, 0) = 0.0;
          this->setLevel(1);
        }
        else
        {
          _error(0, 0) = coef_ * (pixel_value - _measurement);
        }
      }
    }

    double GetError() { return _error(0, 0); }

    // plus in manifold
    virtual void linearizeOplus();

    // dummy read and write functions because we don't care...
    virtual bool read(std::istream &in) { return false; }
    virtual bool write(std::ostream &out) const { return false; }

    bool IsWithinimage();

    inline void setCameraParameter(g2o::CameraParameters *cam) { cam_ = cam; }

    virtual void reset()
    {
      if (_robustKernel)
      {
        delete _robustKernel;
        _robustKernel = nullptr;
      }

      _dimension = -1;
      _level = 0;
      _internalId = -1;
      _cacheIds.clear();
      _parameterTypes.clear();
      _parameters.clear();
      _parameterIds.clear();
      _vertices.clear();
      _vertices.resize(1);
      _id = -1;
    }

  protected:
    inline float getPixelValue(float x, float y)
    {
      uchar *data = &image_->data[int(y) * image_->step + int(x)];
      float xx = x - floor(x);
      float yy = y - floor(y);
      return float((1 - xx) * (1 - yy) * data[0] + xx * (1 - yy) * data[1] +
                   (1 - xx) * yy * data[image_->step] + xx * yy * data[image_->step + 1]);
    }

  public:
    CameraParameters *cam_;
    cv::Mat *image_ = nullptr; // reference image
    Eigen::Vector3d wp_;       // 3D point in world frame
    double cam_height_;
    double baselink2cam_; // x direction (default : 1.383)
    double coef_;
  };
}

#endif
