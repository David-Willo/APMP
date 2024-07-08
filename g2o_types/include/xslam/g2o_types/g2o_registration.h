/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2023-09-15 09:27:53
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2023 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#pragma once

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>

#include "./g2o_types.h"

namespace xslam {

class EdgePriorPoseSE3 : public g2o::BaseUnaryEdge<6, SE3, VertexSE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgePriorPoseSE3() = default;

  //   explicit EdgePGORotation(const SO3 &rot_ji) : rot_ji_(rot_ji) {}

  bool read(std::istream& is) {}

  bool write(std::ostream& os) const {}

  void computeError() override;

  // virtual void linearizeOplus() override;
};

class EdgeRegistrationPt2LineSE3
    : public g2o::BaseUnaryEdge<3, Vector3, VertexSE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeRegistrationPt2LineSE3() = default;

  //   explicit EdgePGORotation(const SO3 &rot_ji) : rot_ji_(rot_ji) {}

  bool read(std::istream& is) {}

  bool write(std::ostream& os) const {}

  void computeError() override;

  // virtual void linearizeOplus() override;

  Vector3 mean_, normal_;
  Vector3 pt_;
};

class EdgeRegistrationPt2PlaneSE3
    : public g2o::BaseUnaryEdge<1, Vector3, VertexSE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeRegistrationPt2PlaneSE3() = default;

  //   explicit EdgePGORotation(const SO3 &rot_ji) : rot_ji_(rot_ji) {}

  bool read(std::istream& is) {}

  bool write(std::ostream& os) const {}

  void computeError() override;

  virtual void linearizeOplus() override;

  Vector3 mean_, normal_;
  Vector3 pt_;
};

}  // namespace xslam
