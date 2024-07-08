/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 06:13:45
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include <glog/logging.h>

#include "xslam/common/timing.h"
#include "xslam/visual-features/superpoint-jit.h"

using namespace xslam;
using namespace std;
using namespace std::chrono;

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_v = 1;

  // SuperPointJIT::Options options;
  // options.model_path("/home/hyhuang/Data/vloc/hkustgz0119_sample/torch/sp.pt");
  // SuperPointJIT model(options);

  // torch::NoGradGuard grad_guard;
  // {
  //   auto inp =
  //       torch::rand({1, 1, 768, 1024}).to(torch::kCUDA).to(torch::kFloat32);
  //   // timing::Timer timer("init");
  //   model.forward(inp);
  //   // timer.Stop();
  // }
  // // timing::Timing::Print(std::cout);
  // std::vector<float> ds;
  // for (size_t i = 0; i < 100; i++) {
  //   torch::cuda::synchronize();
  //   auto start = high_resolution_clock::now();

  //   auto inp =
  //       torch::rand({1, 1, 768, 1024}).to(torch::kCUDA).to(torch::kFloat32);
  //   // timing::Timer timer("forward");
  //   model.forward(inp);
  //   torch::cuda::synchronize();
  //   // timer.Stop();
  //   auto end = high_resolution_clock::now();
  //   auto duration = duration_cast<milliseconds>(end - start);
  //   ds.push_back(duration.count());
  // }
  // cout << std::reduce(ds.begin(), ds.end()) / ds.size() << endl;

  // timing::Timing::Print(std::cout);
  // customInit();
  return 0;
}
