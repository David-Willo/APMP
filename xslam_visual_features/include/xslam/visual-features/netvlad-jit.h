/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 06:09:17
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once
#include <opencv2/opencv.hpp>

#include "torch/script.h"
#include "torch/torch.h"
#include "torch_tensorrt/torch_tensorrt.h"
#include "xslam/common/arg.h"
#include "xslam/common/eigen_types.h"
#include "xslam/visual-features/global-feature.h"

namespace xslam {

class NetVLADJIT : public GlobalFeature {
 public:
  explicit NetVLADJIT(const Options& options);
  virtual ~NetVLADJIT() = default;

  torch::jit::IValue forward(const torch::Tensor& tensor);

  void saveModule(const std::string& output_path);

  virtual void extractFeature(
      const cv::Mat& image, VectorXb& descriptor) override;

  void convertTensorToNativeType(
      const torch::Tensor& tensor, VectorXb& descriptor);

 protected:
  std::shared_ptr<torch::jit::script::Module> module_ = nullptr;
  Options options_;
  /* data */
};

}  // namespace xslam
