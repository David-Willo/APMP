/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 06:09:45
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include <Eigen/Dense>
#include <list>
// #include <opencv/cv.h>
#include <torch/script.h>
#include <torch/torch.h>
#include <vector>

// #include "base_extractor.h"

namespace xslam {

class SuperPoint : public torch::nn::Module {
 public:
  struct Options {
    float conf_thresh;
    int height;
    int width;
    int cell_size;
  };

  explicit SuperPoint(const Options& options);

  virtual ~SuperPoint() = default;

  // torch::Tensor grid;
  // float conf_thresh;
  int h, w, hc, wc, c;

  // torch::Device device_;

  std::vector<torch::Tensor> forward(torch::Tensor x);

  torch::nn::Conv2d conv1a;
  torch::nn::Conv2d conv1b;

  torch::nn::Conv2d conv2a;
  torch::nn::Conv2d conv2b;

  torch::nn::Conv2d conv3a;
  torch::nn::Conv2d conv3b;

  torch::nn::Conv2d conv4a;
  torch::nn::Conv2d conv4b;

  torch::nn::Conv2d convPa;
  torch::nn::Conv2d convPb;

  // descriptor
  torch::nn::Conv2d convDa;
  torch::nn::Conv2d convDb;
};

struct SPConfig {
  std::string model_path;
  int height, width;

  float conf_thresh = 0.015;

  bool limit_number = false;
  int num_features = 1000;

  int cell_size = 8;
};

// class SPExtractor {
//  public:
//   // SPExtractor(std::string model_path, float conf_thresh, int nfeatures,
//   //             int height, int width);

//   explicit SPExtractor(const SPConfig& opt);
//   // SPExtractor(int nfeatures, std::string weight_path,
//   //             float conf_thresh = 0.007);

//   virtual ~SPExtractor() = default;

//   // void extract(cv::InputArray image, cv::InputArray mask,
//   //              std::vector<cv::KeyPoint> &keypoints,
//   //              cv::OutputArray descriptors);

//   void extract(
//       cv::InputArray image, cv::InputArray mask,
//       std::vector<cv::KeyPoint>& keypoints, cv::OutputArray descriptors);

//   static float distance(
//       const Eigen::RowVectorXf& f0, const Eigen::RowVectorXf& f1) {
//     return (f0 - f1).norm();
//   }

//   // cv::Mat getMask() { return mask_; }

//   //   void assignFeatureToGrid(()()()()()()()())

//   // cv::Mat getHeatMap() { return heat_; }

//   // const std::vector<Eigen::Vector2f> getCov() { return cov2_; }

//   // const std::vector<Eigen::Vector2f> getCov2Inv() { return cov2_inv_; }

//   // cv::Mat semi_dust_, dense_dust_;

//   // cv::Mat mask_, heat_, heat_inv_;

//   cv::Mat occ_grid_;

//  protected:
//   // std::vector<Eigen::Vector2f> cov2_, cov2_inv_;
//   // std::vector<Eigen::Matrix2f> info_mat_;
//   // std::vector<cv::Point2f> cov2_inv_;

//   SPConfig opt_;

//   // bool limit_num_feature_ = false;

//   // int num_feature_ = 1000;

//   // float conf_thresh_ = 0.015;

//   // int height_, width_;

//   // std::shared_ptr<torch::jit::script::Module> model_;
//   torch::Device device_;

//   std::shared_ptr<SPFrontend> model_;
// };

}  // namespace xslam
