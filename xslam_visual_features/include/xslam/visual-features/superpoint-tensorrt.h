/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 06:09:39
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once
#include <opencv2/opencv.hpp>

#include "torch/script.h"
#include "torch_tensorrt/torch_tensorrt.h"

#include "xslam/visual-types/keypoint.h"

namespace xslam {

class SuperPointTensorRT {
 public:
  struct Options {
    std::string model_path;

    int image_width;
    int image_height;

    bool b_gpu = true;
  };

 public:
  explicit SuperPointTensorRT(const Options& options);
  virtual ~SuperPointTensorRT() = default;

  virtual void extractFeature(
      const cv::Mat& image, KeyPointContainer& keypoints,
      MatrixXb& descriptors);

  // void initialize(std::string);
  void forward(const torch::Tensor& tensor);

 protected:
  std::shared_ptr<torch::jit::script::Module> module_ = nullptr;

  Options options_;
  /* data */
};

}  // namespace xslam