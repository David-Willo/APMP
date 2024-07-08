/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 06:07:52
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/g2o_types/g2o_registration.h"

#include "xslam/common/math.h"

namespace xslam {

using namespace std;

void EdgePriorPoseSE3::computeError() {
  const SE3& Twb = static_cast<const VertexSE3*>(_vertices[0])->estimate();
  // const SE3& Twb_prior = measurement();
  SE3 Trel = measurement() * Twb;
  _error = Trel.log();
}

void EdgeRegistrationPt2LineSE3::computeError() {
  const SE3& Twb = static_cast<const VertexSE3*>(_vertices[0])->estimate();

  _error = normal_.cross(Twb * pt_ - mean_);
}

// void EdgeRegistrationPt2LineSE3::linearizeOplus() {
// }

void EdgeRegistrationPt2PlaneSE3::computeError() {
  const SE3& Twb = static_cast<const VertexSE3*>(_vertices[0])->estimate();

  _error(0) = normal_.dot(Twb * pt_ - mean_);

  // if (isnan(_error(0))) {
  //   throw std::runtime_error("nan");
  // }
}

void EdgeRegistrationPt2PlaneSE3::linearizeOplus() {
  const SE3& Twb = static_cast<const VertexSE3*>(_vertices[0])->estimate();

  _jacobianOplusXi.leftCols(3) = normal_.transpose();
  _jacobianOplusXi.rightCols(3) = -normal_.transpose() * math::skew(Twb * pt_);
  // if (_jacobianOplusXi.hasNaN()) {
  //   throw std::runtime_error("nan");
  // }
}

}  // namespace xslam
