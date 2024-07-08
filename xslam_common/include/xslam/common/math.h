#pragma once

#include "xslam/common/common.h"

namespace xslam {

namespace math {

Matrix3 skew(const Vector3& vec);
}

class Math {
 public:
  //   static Matrix3d computeEssentialMatrix(const SE3Quat &Tcw1,
  //                                          const SE3Quat &Tcw2

  //   );

  //   static Matrix3d computeFundamentalMatrix(const SE3Quat &Tcw1,
  //                                            const Matrix3d &K1,
  //                                            const SE3Quat &Tcw2,
  //                                            const Matrix3d &K2

  //   );
  //   static SE3 averagePoses(
  //       const eigen_aligned_std_vector<SE3>& poses,
  //       const std::vector<double>& weights);

  //   static Matrix3 rightJacobian(const SO3& rot);

  //   static Matrix3 rightJacobian(const Vector3d& rot);

  //   static Matrix3 inverseRightJacobian(const Vector3d& rot);

  //   static Matrix6 invRightJacobian(const SE3& trans);

  //   static Matrix6 invRightJacobianSE3(const Vector6& v);

  //   // static Matrix6 adjoint

  //   // static Matrix6 rightJacobian(const SE3 &rot);

  //   // static void marginalize();
  //   static Eigen::MatrixXd marginalize(
  //       const Eigen::MatrixXd& H, const int& start, const int& end);

  //   template <typename PointT>
  //   static Vector3d pcl2Eigen(const PointT& pt) {
  //     return Vector3d(pt.x, pt.y, pt.z);
  //   }
};

}  // namespace xslam
