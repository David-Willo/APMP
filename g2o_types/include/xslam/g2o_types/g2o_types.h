/**
 * @file g2o_types.h
 * @author hyhuang hhuangat@gmail.com
 * @brief 
 * @version 0.1
 * @date 2023-03-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include <g2o/core/base_vertex.h>

#include "xslam/common/common.h"
#include "xslam/common/eigen_types.h"

namespace xslam {

class VertexVector3 : public g2o::BaseVertex<3, Vector3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexVector3() : g2o::BaseVertex<3, Vector3>() {}

  virtual bool read(std::istream& is) {
    return false;
  }
  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void setToOriginImpl() {
    setEstimate(Vector3::Zero());
  }

  virtual void oplusImpl(const double* update_) {
    Eigen::Map<const Vector3> uv(update_);
    setEstimate(estimate() + uv);
  }
};

class VertexSO3 : public g2o::BaseVertex<3, SO3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexSO3() : g2o::BaseVertex<3, SO3>() {}

  virtual bool read(std::istream& is) {
    return false;
  }
  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void setToOriginImpl() {
    setEstimate(SO3());
  }

  virtual void oplusImpl(const double* update_) {
    Eigen::Map<const Vector3> uv(update_);
    setEstimate(estimate() * SO3::exp(uv));
  }
};

class VertexSE3 : public g2o::BaseVertex<6, SE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexSE3() : g2o::BaseVertex<6, SE3>() {}

  explicit VertexSE3(const SE3& T) : g2o::BaseVertex<6, SE3>() {
    setEstimate(T);
  }

  virtual bool read(std::istream& is) {
    return false;
  }
  virtual bool write(std::ostream& os) const {
    return false;
  }

  virtual void setToOriginImpl() {
    setEstimate(SE3());
  }

  virtual void oplusImpl(const double* update_) {
    Eigen::Map<const Vector6> uv(update_);
    setEstimate(SE3::exp(uv) * estimate());
  }
};

}  // namespace xslam