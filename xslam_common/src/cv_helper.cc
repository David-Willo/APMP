#include "xslam/common/cv_helper.h"

#include <opencv2/core/eigen.hpp>

namespace xslam {

SE3 CVHelper::convertToSE3(const cv::Mat& tvec, const cv::Mat& rvec) {
  Eigen::Vector3d t, rvec_eigen;
  cv::cv2eigen(tvec, t);
  cv::cv2eigen(rvec, rvec_eigen);

  number_t angle = rvec_eigen.norm();
  Vector3 axis = rvec_eigen / angle;

  Quaternion rot(AngleAxis(angle, axis));
  rot.normalize();

  return SE3(rot, t);
}

}  // namespace xslam
