/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:07:53
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-features/superpoint-jit.h"

#include <filesystem>

#include "xslam/common/logging.h"
#include "xslam/visual-features/torch-helper.h"

// #define DO_CORRECTION 1

namespace xslam {
using namespace torch::indexing;

SuperPointJIT::SuperPointJIT(const Options& options) : options_(options) {
  torch::NoGradGuard grad_guard;
  // torch::jit::getProfilingMode() = false;
  // torch::jit::getExecutorMode() = false;
  // torch::jit::setGraphExecutorOptimize(false);

  scalar_type_ = options_.use_fp16() ? torch::kFloat16 : torch::kFloat32;

  try {
    module_ = std::make_shared<torch::jit::script::Module>(
        torch::jit::load(options_.model_path()));
  } catch (const c10::Error& e) {
    ERRLOG("cound not load model: {}", options_.model_path());
  }

  if (options_.b_gpu()) {
    CHECK(torch::cuda::is_available());
    CHECK(torch::cuda::cudnn_is_available());
    module_->to(torch::kCUDA);
    module_->to(scalar_type_);
  }
  module_->eval();

  // auto in = torch::randn({1, 1, 32, 32}, {torch::kCUDA});
  // torch_tensorrt::ts::convert_method_to_trt_engine(*module_, "xx",
  // torch_tensorrt::ts::CompileSpec({in.sizes()}))
  // // auto trt_mod = torch_tensorrt::CompileGraph(
  //     mod,
  //     std::vector<torch_tensorrt::CompileSpec::InputRange>{{in.sizes()}});

  // initialization
  // TODO: magic number
  if (options_.build_tensorrt()) {
    auto inp =
        torch::rand({1, 1, options_.image_height(), options_.image_width()})
            .to(torch::kCUDA)
            .to(scalar_type_);
    std::vector<int64_t> shape{
        1, 1, options_.image_height(), options_.image_width()};
    auto input = torch_tensorrt::Input(shape, scalar_type_);
    auto compile_settings = torch_tensorrt::ts::CompileSpec({input});
    // // FP16 execution
    compile_settings.enabled_precisions = {scalar_type_};
    // compile_settings.truncate_long_and_double = true;
    // Compile module
    *module_ = torch_tensorrt::ts::compile(*module_, compile_settings);
    forward(inp);
  }
  // // Run like normal
  // auto results = trt_mod.forward({in_tensor});
}

void SuperPointJIT::saveModule(const std::string& output_path) {
  module_->save(output_path);
}

torch::jit::IValue SuperPointJIT::forward(const torch::Tensor& tensor) {
  torch::NoGradGuard grad_guard;
  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(tensor);
  // return trt_module_.forward(inputs);
  return module_->forward(inputs);
}

// TODO: preprocessing
void SuperPointJIT::extractFeature(
    const cv::Mat& _image, KeyPointContainer& _keypoints,
    MatrixXb& _descriptor) {

  cv::Mat image_pre_input;
  cv::Mat image_input;

#ifdef DO_CORRECTION
  cv::Mat image_gray;
  cv::cvtColor(_image, image_gray, cv::COLOR_BGR2GRAY);
  cv::imshow("in", image_gray);
  enhance_worker_.Process(_image, &image_pre_input);
  cv::imshow("gamma", image_pre_input);
  // image_input = image_pre_input;
  cv::waitKey(1);
#else
  image_pre_input = _image;
#endif

  // DW: seem no need to judge type here, do it in convertImageType, 
  if (image_pre_input.type() == CV_8UC1) {
    // WARNLOG("gray");
    image_input = TorchHelper::convertImageType(image_pre_input, true);
  } else if (image_pre_input.type() == CV_8UC3) {
    // WARNLOG("color");
    image_input = TorchHelper::convertImageType(image_pre_input, true);
  } else {
    image_input = image_pre_input;
  }



  torch::NoGradGuard grad_guard;
  auto inp_tensor = TorchHelper::convertCvMatToTensor(image_input);
  if (scalar_type_ == torch::kFloat16) {
    inp_tensor = inp_tensor.to(scalar_type_);
  }
  inp_tensor = inp_tensor.to(torch::kCUDA);
  // INFOLOG("inp {}", inp_tensor.dtype());

  auto output = forward(inp_tensor).toTupleRef();
  auto scores = output.elements()[0].toTensor();
  auto desc = output.elements()[1].toTensor();

  INFOLOG("output type: {} {}", scores.sizes(), desc.sizes());
  if (desc.sizes().size() == 5) {
    desc = desc.squeeze(0);
  }

  if (scalar_type_ == torch::kFloat16) {
    desc = desc.to(torch::kFloat16);
  }
  // INFOLOG("desc {}", desc.dtype());

  // std::cout << desc.index({0, Slice(), 0, 0}) << std::endl;
  desc = torch::nn::functional::normalize(
      desc, torch::nn::functional::NormalizeFuncOptions().p(2).dim(1));
  // INFOLOG("desc {}", desc.dtype());

  scores = performNonMaximumSuppression(scores, options_.nms_radius());
  // INFOLOG("score size: {}", scores.sizes());

  // TODO: multiple image
  scores = scores.squeeze(0);
  auto keypoints = torch::nonzero(scores > options_.threshold());
  // INFOLOG("keypoints size: {}", keypoints.sizes());
  // torch::to
  // scores[keypoints.to()];
  scores = scores.index(
      {keypoints.index({Slice(), 0}), keypoints.index({Slice(), 1})});

  auto h = inp_tensor.size(2);
  auto w = inp_tensor.size(3);
  std::tie(keypoints, scores) =
      removeBorders(keypoints, scores, options_.border_width(), h, w);

  // TODO: topk if maximum number set

  keypoints = torch::flip(keypoints, {1}).to(scalar_type_);

  // if ()

  desc = sampleDescriptors(keypoints, desc);
  desc = desc.squeeze(0);

  convertTensorToNativeType(keypoints, scores, desc, _keypoints, _descriptor);

  // INFOLOG("desc shape: {}", desc.sizes());

  // caffe2::TypeMeta

  // # scores = [s[tuple(k.t())] for s, k in zip(scores, keypoints)]

  // for (size_t i = 0; i < count; i++) {

  // }
  // auto keypoints = [
  //     torch.nonzero(s > self.config['keypoint_threshold'])
  //     for s in scores]
  // scores = [s[tuple(k.t())] for s, k in zip(scores, keypoints)]

  // std::vector<torch::jit::IValue> inputs;
  // inputs.push_back(inp_tensor);
  // auto output = module_(inputs).toTensor();
}

void SuperPointJIT::convertTensorToNativeType(
    const torch::Tensor& _keypoints, const torch::Tensor& _scores,
    const torch::Tensor& _descriptors, KeyPointContainer& keypoints,
    MatrixXb& descriptors) {
  // KeyPointContainer keypoints;
  // MatrixXb descriptors;
  auto keypoints_cpu = _keypoints.to(torch::kCPU);
  auto scores_cpu = _scores.to(torch::kCPU);
  auto descriptors_cpu = _descriptors.to(torch::kCPU);
  auto num_keypoints = keypoints_cpu.size(0);

  // TODO: use flag b_offset

  keypoints.reserve(num_keypoints);
  if (scalar_type_ == torch::kFloat16) {
    keypoints_cpu = keypoints_cpu.to(torch::kFloat32);
    scores_cpu = scores_cpu.to(torch::kFloat32);
    descriptors_cpu = descriptors_cpu.to(torch::kFloat32);
  }

  for (decltype(num_keypoints) i = 0; i < num_keypoints; i++) {
    keypoints.emplace_back(
        Vector2(
            keypoints_cpu[i][0].item<float>() + 0.5f,
            keypoints_cpu[i][1].item<float>() + 0.5f),
        scores_cpu[i].item<float>());
  }

  // descriptors_cpu = descriptors_cpu.t();
  // TODO: scalar type
  MatrixXf desc_remap = Eigen::Map<
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      (float*)descriptors_cpu.data_ptr(), 256, num_keypoints);
  descriptors.resize(
      256 * sizeof(float) / sizeof(unsigned char), num_keypoints);
  memcpy(
      descriptors.data(), desc_remap.data(),
      256 * sizeof(float) / sizeof(unsigned char) * num_keypoints);
}

// torch::Tensor SuperPointJIT::normalizeImage(torch::Tensor& tensor) {
//   tensor
// }

torch::Tensor SuperPointJIT::performNonMaximumSuppression(
    torch::Tensor& scores, int nms_radius) {
  auto max_pool = [&nms_radius](const torch::Tensor& x) {
    const int kernel_size = nms_radius * 2 + 1;
    const int stride = 1;
    const int padding = nms_radius;
    return torch::max_pool2d(x, kernel_size, stride, padding);
  };

  auto zeros = torch::zeros_like(scores);

  auto max_mask = (scores == max_pool(scores));
  for (size_t i = 0; i < 2; i++) {
    auto supp_mask = (max_pool(max_mask.to(scalar_type_)) > 0.0f);
    // INFOLOG("{}", max_mask.to(scalar_type_).dtype());
    auto supp_scores = torch::where(supp_mask, zeros, scores);
    auto new_max_mask = (supp_scores == max_pool(supp_scores));
    max_mask = max_mask | (new_max_mask & ~(supp_mask));
  }
  // INFOLOG("mask sum: {}", max_mask.sum());
  return torch::where(max_mask, scores, zeros);

  //   for _ in range(2):
  //     supp_mask = max_pool(max_mask.float()) > 0
  //     supp_scores = torch.where(supp_mask, zeros, scores)
  //     new_max_mask = supp_scores == max_pool(supp_scores)
  //     max_mask = max_mask | (new_max_mask & (~supp_mask))
  // return torch.where(max_mask, scores, zeros)
  // for(auto&& std::optional<int> : )
}

// def remove_borders(keypoints, scores, border: int, height: int, width: int):
//     """ Removes keypoints too close to the border """
//     mask_h = (keypoints[:, 0] >= border) & (
//         keypoints[:, 0] < (height - border))
//     mask_w = (keypoints[:, 1] >= border) & (keypoints[:, 1] < (width -
//     border)) mask = mask_h & mask_w return keypoints[mask], scores[mask]
std::tuple<torch::Tensor, torch::Tensor> SuperPointJIT::removeBorders(
    const torch::Tensor& keypoints, const torch::Tensor& scores, int border,
    int height, int width) {
  auto mask_h =
      ((keypoints.index({Slice(), 0}) >= border) &
       (keypoints.index({Slice(), 0}) < (height - border)));
  auto mask_w =
      ((keypoints.index({Slice(), 1}) >= border) &
       (keypoints.index({Slice(), 1}) < (width - border)));
  auto mask = (mask_h & mask_w);

  // return {keypoints, scores};
  // INFOLOG("mask sum {}", mask.sum());
  // INFOLOG("kp shape {}", keypoints.index({mask, Slice()}).sizes());
  // INFOLOG("x shape {}", scores.index({mask}).sizes());
  return {keypoints.index({mask, Slice()}), scores.index({mask})};

  // return {keypoints.masked_select(mask), scores.masked_select(mask)};
  // return {keypoints[mask], scores[mask]};
  // auto mask_w = (keypoints.index({torch::indexing::Slice(), 1}) >= border);
  // scores
}

// keypoints = keypoints*2 - 1  # normalize to (-1, 1)
// args = {'align_corners': True} if torch.__version__ >= '1.3' else {}
// descriptors = torch.nn.functional.grid_sample(
//     descriptors, keypoints.view(b, 1, -1, 2), mode='bilinear', **args)
// descriptors = torch.nn.functional.normalize(
//     descriptors.reshape(b, c, -1), p=2, dim=1)
// return descriptors
torch::Tensor SuperPointJIT::sampleDescriptors(
    const torch::Tensor& keypoints, const torch::Tensor& descriptors, int s) {
  //
  auto b = descriptors.size(0);
  auto c = descriptors.size(1);
  auto h = descriptors.size(2);
  auto w = descriptors.size(3);
  // INFOLOG("{}", descriptors.sizes());
  // auto&& [b, c, h, w] = descriptors.sizes();

  auto coords = keypoints - s / 2 + 0.5;
  coords /= torch::tensor({w * s - s / 2 - 0.5, h * s - s / 2 - 0.5})
                .to(coords)
                .index({None});
  coords = coords * 2 - 1;
  // INFOLOG("coords type: {}", coords.dtype());
  // INFOLOG("kps type: {}", keypoints.dtype());
  // INFOLOG("descriptor: {}", descriptors.dtype());

  auto desc_sampled = torch::nn::functional::grid_sample(
      descriptors, coords.view({b, 1, -1, 2}),
      torch::nn::functional::GridSampleFuncOptions()
          .mode(torch::kBilinear)
          .align_corners(true));

  desc_sampled = torch::nn::functional::normalize(
      desc_sampled.reshape({b, c, -1}),
      torch::nn::functional::NormalizeFuncOptions().p(2).dim(1));
  // torch::ones({1, 2}).to(coords);
  // keypoints /= torch.tensor([(w*s - s/2 - 0.5), (h*s - s/2 - 0.5)],
  //                           ).to(keypoints)[None]

  return desc_sampled;
}

}  // namespace xslam