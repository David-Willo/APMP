/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2023-08-17 13:37:48
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2023 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#pragma once

#include "xslam/common/arg.h"
#include "xslam/visual-types/visual-frame.h"

namespace xslam {

class Optimization {
 public:
  struct Options {
    ADD_ARG(bool, use_robust_kernel) = true;
    ADD_ARG(std::string, robust_kernel_type) = "Huber";
    ADD_ARG(double, projection_error) = 8.0;

    ADD_ARG(bool, clear_outliers) = true;
    ADD_ARG(bool, add_prior) = false;
  };

 public:
  static void optimizeFramePosePnP(
      VisualFrame::Ptr& frame, const Options& options);

  /* data */
};

}  // namespace xslam
