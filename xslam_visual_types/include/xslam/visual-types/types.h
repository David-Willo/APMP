/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-07 07:16:35
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include <limits>
#include <unordered_set>
#include <vector>

#include "aslam/common/unique-id.h"
#include "xslam/common/common.h"
#include "xslam/common/eigen_types.h"
// #include "xslam/common/frame.h"

namespace xslam {

UNIQUE_ID_DEFINE_ID(MapPointId);

using KeyPointId = uint32_t;
constexpr KeyPointId kInvalidKeyPointId = std::numeric_limits<uint32_t>::max();

struct ProjectionStatus {
  bool is_visible;

  Vector3 uvd;

  number_t distance;
  number_t view_cosine;
  number_t scale;
};

// enum LocalFeatureType { ORB, SUPER_POINT  };

// enum GlobalFeatureType { DBOW_ORB, NETVLAD  };

}  // namespace xslam

UNIQUE_ID_DEFINE_ID_HASH(xslam::MapPointId);

// UNIQUE_ID_DEFINE_ID_HASH(MapPointId);