/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-09-26 14:40:52
 * @LastEditTime: 2024-07-07 07:25:05
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <torch/csrc/jit/runtime/graph_executor.h>

#include "xslam/common/logging.h"
#include "xslam/common/timing.h"
#include "xslam/visual-features/superpoint-jit.h"
#include "xslam/visual-features/superpoint.h"

namespace xslam {

using namespace std;

TEST(TestFeature, SuperPointJIT) {
  SuperPointJIT::Options options;
  // torch::jit::getBailoutDepth() = 1;
  // torch::jit::getProfilingMode() = false;
  // cout << torch::jit::getBailoutDepth() << endl;
  // torch::jit::getExecutorMode() = false;
  // torch::jit::setGraphExecutorOptimize(false);
  // options.model_path("./test_models/superpoint_1024x768_fp32.ts");

#if (defined(__aarch64__) || defined(__arm__))
  std::string script_path = "./test_models/superpoint_orin_1024x768_fp32.ts";
#else
  std::string script_path =
      "./test_models/superpoint_x86_4080_1024x768_fp32.ts";
#endif
  // options.model_path(script_path);
  options.model_path("./test_models/superpoint.pt");


  options.image_height(768);
  options.image_width(1024);
  SuperPointJIT model(options);

  // timing
  torch::NoGradGuard grad_guard;

  auto inp = torch::rand({1, 1, 768, 1024}).to(torch::kCUDA);
  for (size_t i = 0; i < 50; i++) {
    // auto inp =
    //     torch::rand({1, 1, 768,
    //     1024}).to(torch::kCUDA).to(torch::kFloat32).pin_memory();
    // inp = inp.to(torch::kCUDA);
    // inp = torch::rand({1, 1, 768, 1024});
    // inp = inp.to(torch::kCUDA).set_requires_grad(false);

    torch::cuda::synchronize();
    timing::Timer timer("forward");
    model.forward(torch::autograd::make_variable(inp, false));
    torch::cuda::synchronize();
    timer.Stop();
  }

  timing::Timing::Print(std::cout);
}




TEST(TestFeature, SuperPointJITDistill) {
  SuperPointJIT::Options options;

#if (defined(__aarch64__) || defined(__arm__))
  std::string script_path = "./test_models/superstudent_orin_1024x768_fp32.ts";
#else
  std::string script_path =
      "./test_models/superstudent_x86_3080_1024x768_fp32.ts";
#endif


  // test distill
  options.model_path("./test_models/distill/superstudent.pt");
  options.image_height(768);
  options.image_width(1024);
  SuperPointJIT model(options);

  // timing
  torch::NoGradGuard grad_guard;

  auto inp = torch::rand({1, 1, 768, 1024}).to(torch::kCUDA);
  for (size_t i = 0; i < 50; i++) {
    torch::cuda::synchronize();
    timing::Timer timer("forward_distill");
    model.forward(torch::autograd::make_variable(inp, false));
    torch::cuda::synchronize();
    timer.Stop();
  }

  timing::Timing::Print(std::cout);
}


using namespace torch::indexing;
TEST(TestFeature, SuperPointJIT_Extraction) {
  SuperPointJIT::Options options;
#if (defined(__aarch64__) || defined(__arm__))
  std::string script_path = "./test_models/superpoint_orin_1024x768_fp32.ts";
#else
  std::string script_path =
      "./test_models/superpoint_x86_4080_1024x768_fp32.ts";
#endif
  // options.model_path(script_path).build_tensorrt(true);
  // options.model_path(script_path);
  options.model_path("./test_models/superpoint.pt");
  options.image_height(768);
  options.image_width(1024);
  SuperPointJIT model(options);
  // timing
  torch::NoGradGuard grad_guard;

  cv::Mat image = cv::imread("./test_models/000039.png", cv::IMREAD_COLOR);
  // CHECK(!image.empty());;
  EXPECT_FALSE(image.empty());
  cv::Mat image_gray, image_float;
  cv::cvtColor(image, image_gray, cv::COLOR_RGB2GRAY);
  image_gray.convertTo(image_float, CV_32FC1, 1.f / 255.f, 0);

  // xslam::KeyPointContainer keypoints;
  // xslam::MatrixXb descriptor;
  for (size_t i = 0; i < 100; i++) {
    timing::Timer timer("extract");
    KeyPointContainer keypoints;
    MatrixXb descriptors;
    model.extractFeature(image_float, keypoints, descriptors);
    timer.Stop();
  }

  timing::Timing::Print(std::cout);
}

TEST(TestFeature, SuperPointJIT_Extraction_FP16) {
  SuperPointJIT::Options options;
  // options.model_path("./test_models/superpoint_1024x768_fp32.ts");
  options.model_path("./test_models/superpoint.pt")
      .build_tensorrt(true)
      .use_fp16(true);
  options.image_height(768);
  options.image_width(1024);
  SuperPointJIT model(options);
  // timing
  torch::NoGradGuard grad_guard;

  cv::Mat image = cv::imread("./test_models/000039.png", cv::IMREAD_COLOR);
  // CHECK(!image.empty());;
  EXPECT_FALSE(image.empty());
  cv::Mat image_gray, image_float;
  cv::cvtColor(image, image_gray, cv::COLOR_RGB2GRAY);
  image_gray.convertTo(image_float, CV_32FC1, 1.f / 255.f, 0);

  // xslam::KeyPointContainer keypoints;
  // xslam::MatrixXb descriptor;
  for (size_t i = 0; i < 100; i++) {
    timing::Timer timer("extract_fp16");
    KeyPointContainer keypoints;
    MatrixXb descriptors;
    model.extractFeature(image_float, keypoints, descriptors);
    timer.Stop();
  }

  timing::Timing::Print(std::cout);
}

TEST(TestFeature, SuperStudent_Extraction) {
  SuperPointJIT::Options options;
  // options.model_path("./test_models/superpoint_1024x768_fp32.ts");
  options.model_path("./test_models/distill/superstudent_nobn_att.pt")
      .build_tensorrt(true)
      .use_fp16(true);
  options.image_height(768);
  options.image_width(1024);
  SuperPointJIT model(options);
  // timing
  torch::NoGradGuard grad_guard;

  cv::Mat image = cv::imread("./test_models/000039.png", cv::IMREAD_COLOR);
  // CHECK(!image.empty());;
  EXPECT_FALSE(image.empty());
  cv::Mat image_gray, image_float;
  cv::cvtColor(image, image_gray, cv::COLOR_RGB2GRAY);
  image_gray.convertTo(image_float, CV_32FC1, 1.f / 255.f, 0);

  // xslam::KeyPointContainer keypoints;
  // xslam::MatrixXb descriptor;
  for (size_t i = 0; i < 100; i++) {
    timing::Timer timer("extract_distill");
    KeyPointContainer keypoints;
    MatrixXb descriptors;
    model.extractFeature(image_float, keypoints, descriptors);
    timer.Stop();
  }

  timing::Timing::Print(std::cout);
}

TEST(TestFeature, SuperPoint) {
  torch::NoGradGuard grad_guard;
  SuperPoint::Options options;
  // options.model_path =
  // "/home/hyhuang/Data/vloc/hkustgz0119_sample/torch/sp.pt";
  SuperPoint model(options);
  model.to(torch::kCUDA);
  model.eval();

  // timing

  size_t num_iters = 1;
  auto inp = torch::rand({1, 1, 768, 1024});
  for (size_t i = 0; i < 1; i++) {
    // auto inp =
    //     torch::rand({1, 1, 768,
    //     1024}).to(torch::kCUDA).to(torch::kFloat32).pin_memory();
    // inp = inp.to(torch::kCUDA);
    inp = torch::rand({1, 1, 768, 1024});
    inp = inp.to(torch::kCUDA).set_requires_grad(false);

    torch::cuda::synchronize();
    timing::Timer timer("nn_module");
    model.forward(torch::autograd::make_variable(inp, false));
    torch::cuda::synchronize();
    timer.Stop();
  }

  timing::Timing::Print(std::cout);
}

}  // namespace xslam

int main(int argc, char** argv) {
  // ros::init(argc, argv, "test_map_loading");
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_v = 1;
  // customInit();
  return RUN_ALL_TESTS();
}