/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-09-26 14:40:52
 * @LastEditTime: 2024-07-02 06:09:34
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
#include "xslam/visual-features/local-feature.h"
#include "xslam/visual-types/keypoint.h"
#include "xslam/visual-features/image_enhancement_worker.h"


namespace xslam {

class SuperPointJIT : public LocalFeature {
  //  public:
 public:
  explicit SuperPointJIT(const Options& options);
  virtual ~SuperPointJIT() = default;

  virtual void extractFeature(
      const cv::Mat& image, KeyPointContainer& keypoints,
      MatrixXb& desc) override;

  // void initialize(std::string);
  torch::jit::IValue forward(const torch::Tensor& tensor);

  void saveModule(const std::string& output_path);

 protected:
  // torch::Tensor normalizeImage(torch::Tensor& tensor);

  // https://github.com/magicleap/SuperGluePretrainedNetwork/blob/master/models/superpoint.py
  torch::Tensor performNonMaximumSuppression(
      torch::Tensor& tensor, int nms_radius);

  // https://github.com/magicleap/SuperGluePretrainedNetwork/blob/master/models/superpoint.py
  std::tuple<torch::Tensor, torch::Tensor> removeBorders(
      const torch::Tensor& keypoints, const torch::Tensor& scores, int border,
      int height, int width);

  // https://github.com/magicleap/SuperGluePretrainedNetwork/blob/master/models/superpoint.py
  torch::Tensor sampleDescriptors(
      const torch::Tensor& keypoints, const torch::Tensor& descriptors,
      int cell_size = 8);

  // TODO: desc
  // std::tuple<KeyPointContainer, MatrixXb>
  void convertTensorToNativeType(
      const torch::Tensor& keypoints_tensor, const torch::Tensor& scores,
      const torch::Tensor& desc_tensor, KeyPointContainer& keypoints,
      MatrixXb& descriptors);

 protected:
  std::shared_ptr<torch::jit::script::Module> module_ = nullptr;
  //   torch::jit::script::Module trt_module_;

  Options options_;
  torch::ScalarType scalar_type_;


  ImageEnhancementWorker enhance_worker_;
  /* data */
};

}  // namespace xslam