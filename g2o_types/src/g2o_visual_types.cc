/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 06:08:23
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/g2o_types/g2o_visual_types.h"

#include "xslam/common/logging.h"
#include "xslam/common/math.h"

namespace xslam {
using namespace std;

EdgeProjectionPoseOnly::EdgeProjectionPoseOnly()
    : BaseUnaryEdge<2, Vector2, VertexSE3>() {}

EdgeProjectionPoseOnly::EdgeProjectionPoseOnly(
    const Vector3& pt3d, const aslam::Camera::ConstPtr& camera)
    : BaseUnaryEdge<2, Vector2, VertexSE3>(), pt3d_(pt3d), camera_(camera) {}

void EdgeProjectionPoseOnly::computeError() {
  const SE3& Tcw = static_cast<const VertexSE3*>(_vertices[0])->estimate();
  Vector3d ptc = Tcw * pt3d_;
  Vector2d uv;

  camera_->project3(ptc, &uv);
  _error = uv - measurement();
}

void EdgeProjectionPoseOnly::linearizeOplus() {
  const SE3& Tcw = static_cast<const VertexSE3*>(_vertices[0])->estimate();

  Vector3d ptc = Tcw * pt3d_;
  Vector2d uv;

  Eigen::Matrix<double, 2, 3> jacob_proj;

  camera_->project3(ptc, &uv, &jacob_proj);

  // _jacobianOplusXi = jacob_proj * Tcw.so3().matrix();

  Matrix3d ptc_skew = math::skew(ptc);

  _jacobianOplusXi.leftCols(3) = jacob_proj;
  _jacobianOplusXi.rightCols(3) = -1. * jacob_proj * ptc_skew;
}

EdgeSE3Prior::EdgeSE3Prior() {}
// EdgeSE3Prior::EdgeSE3Prior(const SE3& prior_pose) : BaseUnaryEdge<6, SE3, VertexSE3>() {
//   this->setMeasurement(prior_pose);
// }



void EdgeSE3Prior::computeError() {
  const VertexSE3 *v1 =
      static_cast<const VertexSE3 *>(_vertices[0]);

  const SE3 &Tj = v1->estimate();
  SE3 diff = _inverseMeasurement * Tj;
  diff.translation() *= 100;
  _error = diff.log();
}


void EdgeSE3Prior::linearizeOplus() {
  const SE3& Tj = static_cast<const VertexSE3*>(_vertices[0])->estimate();
  _jacobianOplusXi = SE3::leftJacobianInverse(_error)*_inverseMeasurement.Adj();
  // _jacobianOplusXi = SE3::leftJacobianInverse(_error)*Tj.inverse().Adj();
}

}  // namespace xslam
