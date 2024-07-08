/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 06:13:45
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#pragma once

#include "./types.h"
#include "xslam/common/common.h"
#include "xslam/common/eigen_types.h"

namespace xslam {

/**
 * @brief 2d keypoint extracted from image
 *
 * TODO
 */
struct KeyPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //   KeyPoint(const cv::KeyPoint& kp, const cv::Mat& _desc)
  //       : uv(kp.pt.x, kp.pt.y),
  //         desc(_desc.clone()),
  //         size(kp.size),
  //         responce(kp.response),
  //         angle(kp.angle),
  //         octave(kp.octave),
  //         depth(-1.0f),
  //         u_right(-1.0f)

  //   {}
  KeyPoint() = default;

  KeyPoint(const Vector2& _uv, double _score) : uv(_uv), responce(_score) {}

  /**
   * @brief Construct a new Key Point object
   *
   * @param kp
   */
  explicit KeyPoint(const cv::KeyPoint& kp)
      : uv(kp.pt.x, kp.pt.y),
        size(kp.size),
        responce(kp.response),
        angle(kp.angle),
        octave(kp.octave),
        depth(-1.0f),
        disparity(-1.0f) {}

  /// @brief
  /// @param obs
  /// @return
  inline number_t distance2(const Vector2& obs) const {
    return (uv - obs).squaredNorm();
  }

  inline number_t distance(const Vector2& obs) const {
    return (uv - obs).norm();
  }

  // inline double error(const Vector3& obs) const {
  //   if (u_right < 0.0f) {
  //     return error(Vector2(obs.x(), obs.y()));
  //   } else {
  //     Vector3 uvr(uv.x(), uv.y(), u_right);
  //     return (uvr - obs).squaredNorm();
  //   }
  // }

  Vector2 uv;

  float size = 0.0f;
  float responce = 0.0f;
  float angle = -1.0f;
  int octave = 0;

  // depth and disparity
  number_t depth = -1.0f, disparity = -1.0f;
};

using KeyPointContainer = aligned_std_vector<KeyPoint>;

}  // namespace xslam
