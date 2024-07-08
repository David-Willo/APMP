/*
 * @Author: Shuyang Zhang
 * @Date: 2023-07-14 19:35:01
 * @LastEditors: ShuyangUni shuyang.zhang1995@gmail.com
 * @LastEditTime: 2023-07-14 23:50:06
 * @Description:
 *
 * Copyright (c) 2023 by Shuyang Zhang, All Rights Reserved.
 */
#include "xslam/visual-features/image_enhancement_worker.h"

namespace xslam {
ImageEnhancementWorker::ImageEnhancementWorker() {
  sample_res_ = 0.01;
  gamma_seeds_ = {0.1, 0.5, 0.8, 1.0, 1.2, 1.5, 1.9};
  gamma_samples_.clear();
  for (double gamma = 0; gamma <= 2.0; gamma = gamma + sample_res_)
    gamma_samples_.push_back(gamma);
  // lut
  lut_list_.clear();
  for (size_t i = 0; i < gamma_seeds_.size(); ++i) {
    cv::Mat lut;
    GenerateGammaCorrectionLUT(&lut, gamma_seeds_.at(i));
    lut_list_.push_back(lut);
  }
}

void ImageEnhancementWorker::GenerateGammaCorrectionLUT(cv::Mat *lut,
                                                        const double &gamma) {
  double inv_gamma = 1.0 / gamma;
  lut->create(1, 256, CV_8UC1);
  uchar *p = lut->data;
  for (int i = 0; i < 256; ++i) p[i] = pow(i / 255.0, inv_gamma) * 255;
}

void ImageEnhancementWorker::Process(const cv::Mat &img_raw, cv::Mat *img_out) {
  cv::Mat img_in;
  cv::cvtColor(img_raw, img_in, cv::COLOR_BGR2GRAY);
  
  cv::Mat img_gamma;
  std::vector<double> gm_seeds;
  // resize image
  cv::Mat img_in_resize;
  cv::resize(img_in, img_in_resize, cv::Size(480, 270));
  // get gradient magnitude of gamma seed
  for (size_t i = 0; i < gamma_seeds_.size(); ++i) {
    cv::LUT(img_in_resize, lut_list_.at(i), img_gamma);
    gm_seeds.push_back(ImageGradientMagnitude(img_gamma));
  }
  // fitting curve
  FifthPolynomial fp;
  FifthPolynomialFitting(gamma_seeds_, gm_seeds, &fp);
  // get curve values of gamma samples
  std::vector<double> gm_samples;
  FifthPolynomialInference(gamma_samples_, fp, &gm_samples);
  // get maximum value
  int idx_max = std::max_element(gm_samples.begin(), gm_samples.end()) -
                gm_samples.begin();
  // generate target image
  ImageGammaCorrection(img_in, gamma_samples_.at(idx_max), img_out);
}

double ImageEnhancementWorker::ImageGradientMagnitude(const cv::Mat &img) {
  cv::Mat sobel_x, sobel_y, gm;
  cv::Sobel(img, sobel_x, CV_64F, 1, 0);
  cv::Sobel(img, sobel_y, CV_64F, 0, 1);
  cv::magnitude(sobel_x, sobel_y, gm);
  return cv::sum(gm)[0];
}

void ImageEnhancementWorker::ImageGammaCorrection(const cv::Mat &img_in,
                                                  const double &gamma,
                                                  cv::Mat *img_out) {
  cv::Mat lut;
  GenerateGammaCorrectionLUT(&lut, gamma);
  cv::LUT(img_in, lut, *img_out);
}

void ImageEnhancementWorker::FifthPolynomialFitting(
    const std::vector<double> &x, const std::vector<double> &y,
    FifthPolynomial *param) {
  assert(x.size() != 0);
  assert(y.size() != 0);
  assert(x.size() == y.size());

  // prepare least square
  const Eigen::VectorXd vec_b =
      Eigen::Map<const Eigen::VectorXd, Eigen::Aligned>(y.data(), y.size());
  Eigen::MatrixXd mat_a = Eigen::MatrixXd::Zero(x.size(), 6);
  double value = 0.0;
  for (size_t i = 0; i < x.size(); ++i) {
    value = 1.0;
    for (size_t j = 0; j < 6; ++j) {
      mat_a(i, j) = value;
      value *= x.at(i);
    }
  }
  // solve least square
  // direct method, fast but may be singular
  Eigen::VectorXd vec_res =
      (mat_a.transpose() * mat_a).ldlt().solve(mat_a.transpose() * vec_b);

  // for output
  std::copy(vec_res.data(), vec_res.data() + vec_res.size(),
            param->param_array);
}

void ImageEnhancementWorker::FifthPolynomialInference(
    const std::vector<double> &x, const FifthPolynomial &param,
    std::vector<double> *y) {
  y->clear();
  y->resize(x.size());
  double value = 0.0;
  for (size_t i = 0; i < x.size(); ++i) {
    value = 1.0;
    y->at(i) = 0.0;
    for (size_t j = 0; j < 6; ++j) {
      y->at(i) += value * param.param_array[j];
      value = value * x.at(i);
    }
  }
}

}  // namespace img_enh_worker
