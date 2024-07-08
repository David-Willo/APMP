/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:08:04
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-features/superpoint.h"

namespace xslam {
// using namespace cv;
using namespace std;

const int c1 = 64;
const int c2 = 64;
const int c3 = 128;
const int c4 = 128;
const int c5 = 256;
const int d1 = 256;

SuperPoint::SuperPoint(const Options& options)
    // :  // : conf_thresh(conf_thresh_),
    //   h(height),
    //   w(width),
    //   hc(height / cell_size),
    //   wc(width / cell_size),
    //   c(cell_size),

    : conv1a(torch::nn::Conv2dOptions(1, c1, 3).stride(1).padding(1)),
      conv1b(torch::nn::Conv2dOptions(c1, c1, 3).stride(1).padding(1)),
      conv2a(torch::nn::Conv2dOptions(c1, c2, 3).stride(1).padding(1)),
      conv2b(torch::nn::Conv2dOptions(c2, c2, 3).stride(1).padding(1)),
      conv3a(torch::nn::Conv2dOptions(c2, c3, 3).stride(1).padding(1)),
      conv3b(torch::nn::Conv2dOptions(c3, c3, 3).stride(1).padding(1)),
      conv4a(torch::nn::Conv2dOptions(c3, c4, 3).stride(1).padding(1)),
      conv4b(torch::nn::Conv2dOptions(c4, c4, 3).stride(1).padding(1)),
      convPa(torch::nn::Conv2dOptions(c4, c5, 3).stride(1).padding(1)),
      convPb(torch::nn::Conv2dOptions(c5, 65, 1).stride(1).padding(0)),
      convDa(torch::nn::Conv2dOptions(c4, c5, 3).stride(1).padding(1)),
      convDb(torch::nn::Conv2dOptions(c5, d1, 1).stride(1).padding(0)) {
  register_module("conv1a", conv1a);
  register_module("conv1b", conv1b);

  register_module("conv2a", conv2a);
  register_module("conv2b", conv2b);

  register_module("conv3a", conv3a);
  register_module("conv3b", conv3b);

  register_module("conv4a", conv4a);
  register_module("conv4b", conv4b);

  register_module("convPa", convPa);
  register_module("convPb", convPb);

  register_module("convDa", convDa);
  register_module("convDb", convDb);

  // auto yx = torch::meshgrid({torch::arange(height),
  // torch::arange(width)}); cout << x[0].sizes() << endl;
  // x[0].unsqueeze_(0)
  // x[1].unsqueeze_(0)
  // auto grid_ = torch::cat({yx[1].unsqueeze(0), yx[0].unsqueeze(0)});
  // grid = grid_.contiguous()
  //            .view({1, 2, height / 8, 8, width / 8, 8})  // TODO:
  //            batch-size .permute({0, 1, 3, 5, 2, 4}) .reshape({1, 2, 64,
  //            hc, wc}) .cuda();

  // torch::masked_select();
}

// -1 is okay for slicing
std::vector<torch::Tensor> SuperPoint::forward(torch::Tensor x) {
  // timing::Timer timer_f0("feat/f0");
  // torch::E
  torch::Tensor semi;
  {
    x = torch::relu(conv1a->forward(x));
    x = torch::relu(conv1b->forward(x));
    x = torch::max_pool2d(x, 2, 2);

    x = torch::relu(conv2a->forward(x));
    x = torch::relu(conv2b->forward(x));
    x = torch::max_pool2d(x, 2, 2);

    x = torch::relu(conv3a->forward(x));
    x = torch::relu(conv3b->forward(x));
    x = torch::max_pool2d(x, 2, 2);

    x = torch::relu(conv4a->forward(x));
    x = torch::relu(conv4b->forward(x));

    semi = torch::relu(convPa->forward(x));
  }
  semi = convPb->forward(semi).squeeze();  // [B, 65, H/8, W/8]

  auto cDa = torch::relu(convDa->forward(x));
  auto coarse = convDb->forward(cDa);  // [B, d1, H/8, W/8]

  // auto dn = torch::norm(coarse, 2, 1);
  // coarse = coarse.div(torch::unsqueeze(dn, 1));

  // semi = torch::softmax(semi, 0);
  // // auto semi_dust = semi[-1];
  // // auto dense_dust = dense[-1];
  // semi = semi.slice(0, 0, -1);

  // torch::cuda::synchronize();
  // timer_f0.Stop();
  // auto score, indices = ;

  // forward score and inlier indices

  return {semi, coarse};
}

}  // namespace xslam
