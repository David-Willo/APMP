/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-09-26 14:40:52
 * @LastEditTime: 2024-07-02 06:09:53
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include <opencv2/core/core.hpp>
#include <torch/torch.h>

namespace xslam {

class TorchHelper {
 public:
  static torch::ScalarType getTypeTorch(int mat_type) {
    torch::ScalarType type;
    switch (mat_type) {
      case CV_8UC1:
      case CV_8UC3:
        type = torch::kByte;
        break;
      case CV_32FC1:
      case CV_32FC3:
        type = torch::kFloat32;
        break;
      case CV_64FC1:
      case CV_64FC3:
        type = torch::kFloat64;
        break;
      default:
        throw std::runtime_error("not implemented");
        break;
    }

    return type;
  }
  
  static cv::Mat convertImageType(
      const cv::Mat& image, bool gray_scale = false) {
    cv::Mat image_gray, image_float;
    if (gray_scale) {

      if (image.type()  == CV_8UC3) {
        cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
        image_gray.convertTo(image_float, CV_32FC1, 1.f / 255.f, 0);
      } else if (image.type()  == CV_8UC1) {
        image.convertTo(image_float, CV_32FC1, 1.f / 255.f, 0);
      }
    } else {
      
      cv::Mat image_rgb;
      cv::cvtColor(image, image_rgb, cv::COLOR_BGR2RGB);
      image_rgb.convertTo(image_float, CV_32FC3, 1.f / 255.f, 0);
      
    }

    return image_float;
  }

  static torch::Tensor convertCvMatToTensor(
      const cv::Mat& image, bool b_grad = false) {
    const auto height = image.rows;
    const auto width = image.cols;
    const auto channels = image.channels();
    const auto type = getTypeTorch(image.type());

    std::vector<int64_t> dims = {1, height, width, channels};
    auto img_var = torch::from_blob(image.data, dims, type);
    img_var = img_var.permute({0, 3, 1, 2});
    img_var.set_requires_grad(b_grad);

    return img_var;
  }

  static torch::Tensor convertCvMatToTensor(
      const cv::InputArray images, bool b_grad = false) {
    // images.
    throw std::runtime_error("not implemented");
  }

  static cv::Mat convertTensorToCvMat(torch::Tensor) {
    //
  }
};

}  // namespace xslam