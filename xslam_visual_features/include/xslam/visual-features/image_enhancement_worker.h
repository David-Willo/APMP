/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-14 19:35:11
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-14 23:45:41
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#pragma once

#include <Eigen/Dense>
#include <vector>

#include "opencv2/opencv.hpp"

namespace xslam {

union FifthPolynomial {
  struct {
    double p0;
    double p1;
    double p2;
    double p3;
    double p4;
    double p5;
  } param_struct;
  double param_array[6];
};

class ImageEnhancementWorker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImageEnhancementWorker();
  void Process(const cv::Mat &img_raw, cv::Mat *img_out);
  double ImageGradientMagnitude(const cv::Mat &img);
  void GenerateGammaCorrectionLUT(cv::Mat *lut, const double &gamma);
  void ImageGammaCorrection(const cv::Mat &img_in, const double &gamma,
                            cv::Mat *img_out);
  void FifthPolynomialFitting(const std::vector<double> &x,
                              const std::vector<double> &y,
                              FifthPolynomial *param);
  void FifthPolynomialInference(const std::vector<double> &x,
                                const FifthPolynomial &param,
                                std::vector<double> *y);

 private:
  std::vector<double> gamma_seeds_;
  std::vector<double> gamma_samples_;
  double sample_res_;
  std::vector<cv::Mat> lut_list_;
};

}  // namespace xslam
