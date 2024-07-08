/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:07:46
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-features/netvlad-jit.h"

#include <filesystem>

#include "xslam/common/logging.h"
#include "xslam/visual-features/torch-helper.h"

namespace xslam {

NetVLADJIT::NetVLADJIT(const Options& options) : options_(options) {
  try {
    module_ = std::make_shared<torch::jit::script::Module>(
        torch::jit::load(options_.model_path()));
  } catch (const c10::Error& e) {
    ERRLOG("cound not load model: {}", options_.model_path());
  }

  if (options_.gpu()) {
    CHECK(torch::cuda::is_available());
    CHECK(torch::cuda::cudnn_is_available());
    module_->to(torch::kCUDA);
  }
  module_->eval();

  if (options_.build_tensorrt()) {
    auto inp =
        torch::rand({1, 3, options_.image_height(), options_.image_width()})
            .to(torch::kCUDA)
            .to(torch::kFloat32);
    std::vector<int64_t> shape{
        1, 3, options_.image_height(), options_.image_width()};
    auto input = torch_tensorrt::Input(shape, torch::kFloat32);
    auto compile_settings = torch_tensorrt::ts::CompileSpec({input});
    // // FP16 execution
    compile_settings.enabled_precisions = {torch::kFloat};
    // Compile module
    *module_ = torch_tensorrt::ts::compile(*module_, compile_settings);
    forward(inp);
  }
}

torch::jit::IValue NetVLADJIT::forward(const torch::Tensor& tensor) {
  torch::NoGradGuard grad_guard;
  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(tensor);
  // return trt_module_.forward(inputs);
  return module_->forward(inputs);
}

void NetVLADJIT::saveModule(const std::string& output_path) {
  module_->save(output_path);
}

void NetVLADJIT::extractFeature(const cv::Mat& image, VectorXb& descriptor) {
  cv::Mat image_input;
  if (image.type() == CV_8UC1) {
    throw std::runtime_error("the input should have three channels");
  } else if (image.type() == CV_8UC3) {
    image_input = TorchHelper::convertImageType(image, false);
  } else {
    image_input = image;
  }
  torch::NoGradGuard grad_guard;
  auto inp_tensor = TorchHelper::convertCvMatToTensor(image_input);
  inp_tensor = inp_tensor.to(torch::kCUDA);

  auto desc_tensor = forward(inp_tensor).toTensor();

  convertTensorToNativeType(desc_tensor.squeeze(0), descriptor);
}

void NetVLADJIT::convertTensorToNativeType(
    const torch::Tensor& tensor, VectorXb& descriptor) {
  // VectorXb desc =
  descriptor.resize(4096 * sizeof(float) / sizeof(unsigned char));
  memcpy(
      descriptor.data(), tensor.cpu().data_ptr(),
      4096 * sizeof(float) / sizeof(unsigned char));
  // Eigen::Map<VectorXb>(
  //     (unsigned char*)tensor.cpu().data_ptr(),
  //     4096 * sizeof(float) / sizeof(unsigned char));
}

}  // namespace xslam