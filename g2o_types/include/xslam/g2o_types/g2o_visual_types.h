/**
 * @file g2o_visual_types.h
 * @author hyhuang hhuangat@gmail.com
 * @brief 
 * @version 0.1
 * @date 2023-03-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <aslam/cameras/camera.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "./g2o_types.h"

namespace xslam {
class EdgeProjectionPoseOnly
    : public g2o::BaseUnaryEdge<2, Vector2, VertexSE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeProjectionPoseOnly();

  EdgeProjectionPoseOnly(
      const Vector3& pt3d_, const aslam::Camera::ConstPtr& camera);
  // {
  //   // resizeParameters(1);
  //   // installParameter(_cam, 0);
  // }

  bool read(std::istream& is) override {}

  bool write(std::ostream& os) const override {}

  void computeError() override;

  virtual void linearizeOplus() override;

  Vector3 pt3d_;
  aslam::Camera::ConstPtr camera_ = nullptr;
};

class EdgeSE3Prior : public g2o::BaseUnaryEdge<6, SE3, VertexSE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3Prior();
  // EdgeSE3Prior(const SE3 & prior_pose);

  virtual void setMeasurement(const SE3 &m) {
    _measurement = m;
    _inverseMeasurement = m.inverse();
  }

  void computeError();

  virtual void linearizeOplus();

  bool read(std::istream &is) {}

  bool write(std::ostream &os) const {}

  SE3 _inverseMeasurement;
};

}  // namespace xslam
