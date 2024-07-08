/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-07 07:21:03
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include "xslam/common/arg.h"
#include "xslam/common/common.h"
#include "xslam/common/eigen_types.h"

namespace xslam {

class GlobalFeature {
 public:
  using Ptr = std::shared_ptr<GlobalFeature>;

  using ConstPtr = std::shared_ptr<const GlobalFeature>;

  enum Type {
    kNetVLAD = 0,
  };

  struct Options {
    ADD_ARG(Type, feature_type);
    ADD_ARG(std::string, model_path);
    ADD_ARG(int, image_width);
    ADD_ARG(int, image_height);

    // int image_width;
    // int image_height;

    ADD_ARG(bool, gpu) = true;
    ADD_ARG(bool, build_tensorrt) = false;

    // bool b_gpu = true;
    // bool b_build_engine = false;
  };

 public:
  GlobalFeature() = default;
  virtual ~GlobalFeature() = default;

  virtual void extractFeature(const cv::Mat& image, VectorXb& descriptor) = 0;

 protected:
  /* data */
};

}  // namespace xslam
