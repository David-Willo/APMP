/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-07 07:21:04
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include <opencv2/opencv.hpp>

#include "xslam/visual-types/keypoint.h"

#include "xslam/common/arg.h"

namespace xslam {

// template <int Dim, typename Scalar>
class LocalFeature {
 public:
  using Ptr = std::shared_ptr<LocalFeature>;

  using ConstPtr = std::shared_ptr<const LocalFeature>;

  enum Type {
    kORB = 0,
    kSuerPoint = 1,
    kSuerPointJIT = 2,
  };

  struct Options {
    ADD_ARG(Type, feature_type);
    ADD_ARG(std::string, model_path);
    ADD_ARG(int, image_width);
    ADD_ARG(int, image_height);

    ADD_ARG(bool, b_gpu) = true;
    ADD_ARG(bool, build_tensorrt) = false;
    ADD_ARG(bool, use_fp16) = false;

    ADD_ARG(bool, b_offset) = false;
    ADD_ARG(int, border_width) = 4;
    ADD_ARG(int, nms_radius) = 4;
    ADD_ARG(int, num_points) = -1;
    ADD_ARG(float, threshold) = 0.005;
  };


 public:
  LocalFeature() = default;

  virtual ~LocalFeature() = default;

  // static inline int dim() {
  //   // return Dim;
  // }

  // virtual void extractFeature(
  //     const cv::Mat& image, KeyPointContainer& keypoints,
  //     MatrixXb& descriptors) = 0;

  // virtual std::tuple<KeyPointContainer, MatrixXb> extractFeature(
  //     const cv::Mat& image);

  virtual void extractFeature(
      const cv::Mat& image, KeyPointContainer& keypoints, MatrixXb& desc) = 0;

  // TODO: batched inference
  // virtual void extractFeature(const std::vector<cv::Mat>& image) = 0;

  //  protected:
};

}  // namespace xslam
