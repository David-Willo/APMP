/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:07:59
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-features/superpoint-tensorrt.h"

#include <filesystem>

#include "xslam/common/logging.h"
#include "xslam/visual-features/torch-helper.h"

namespace xslam {

SuperPointTensorRT::SuperPointTensorRT(const Options& options)
    : options_(options) {
  torch::NoGradGuard grad_guard;
  // torch::jit::getProfilingMode() = false;
  // torch::jit::getExecutorMode() = false;
  // torch::jit::setGraphExecutorOptimize(false);
  try {
    module_ = std::make_shared<torch::jit::script::Module>(
        torch::jit::load(options_.model_path));
  } catch (const c10::Error& e) {
    ERRLOG("cound not load model: {}", options_.model_path);
  }

  if (options_.b_gpu) {
    CHECK(torch::cuda::is_available());
    CHECK(torch::cuda::cudnn_is_available());
    module_->to(torch::kCUDA);
  }
  module_->eval();

  // initialization
  {
    auto inp =
        torch::rand({1, 1, 768, 1024}).to(torch::kCUDA).to(torch::kFloat32);
    forward(inp);
  }

  // torch::random
}

void SuperPointTensorRT::forward(const torch::Tensor& tensor) {
  torch::NoGradGuard grad_guard;
  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(tensor);
  auto output = module_->forward(inputs).toTupleRef();
  // output.elements().size()
  // INFOLOG("#tensor in output: {}", output.elements().size());
  // INFOLOG("shape of score: {}", output.elements()[0].toTensor().sizes());
  // INFOLOG("shape of desc: {}", output.elements()[1].toTensor().sizes());
}

// TODO: preprocessing
void SuperPointTensorRT::extractFeature(
    const cv::Mat& image, KeyPointContainer& keypoints, MatrixXb& descriptors) {
  auto inp_tensor = TorchHelper::convertCvMatToTensor(image);

  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(inp_tensor);
  // auto output = module_(inputs).toTensor();
}

}  // namespace xslam