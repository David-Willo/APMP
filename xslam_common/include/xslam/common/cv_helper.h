#pragma once

#include "common.h"

namespace xslam {
class CVHelper {
 public:
  // CVHelper(/* args */);
  // ~CVHelper();
  static SE3 convertToSE3(const cv::Mat& tvec, const cv::Mat& rvec) ;
};

}  // namespace xslam