/**
 * @file distances.h
 * @author hyhuang hhuangat@gmail.com
 * @brief
 * @version 0.1
 * @date 2023-03-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "xslam/common/eigen_types.h"

#include <functional>

// TODO: move to common
namespace xslam {

// pointer
class Distance {
 public:
  using DistanceFunc = std::function<float(const VectorXb&, const VectorXb&)>;

  enum DistanceType : uint16_t {
    DISTANCE_NONETYPE,
    DISTANCE_HAMMING,
    DISTANCE_L2_NORM,
    DISTANCE_COSINE
  };

  template <typename Scalar>
  static DistanceFunc createDistanceFunc(const DistanceType& type);

 protected:
  template <typename Scalar>
  static float l2_norm(const VectorXb& lhs, const VectorXb& rhs);

  // virtual float operator()(const VectorXb& a, const VectorXb& b) = 0;
};

}  // namespace xslam

#include "distances-impl.hpp"
