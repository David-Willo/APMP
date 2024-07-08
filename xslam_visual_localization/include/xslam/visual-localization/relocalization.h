/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:08:51
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include "xslam/common/arg.h"
#include "xslam/common/distances.h"
#include "xslam/visual-types/visual-map.h"

// #include

namespace xslam {

// TODO: how to be compact with bow
// TODO: decide scalar
class Relocalization {
 public:
  using Ptr = std::shared_ptr<Relocalization>;

  using ConstPtr = std::shared_ptr<const Relocalization>;

  struct Options {
    ADD_ARG(Distance::DistanceType, distance_type) = Distance::DISTANCE_L2_NORM;

    ADD_ARG(int, num_k_retrieval) = 10;
    ADD_ARG(int, weight_covisible) = 200;

    ADD_ARG(int, num_match_threshold) = 300;
    ADD_ARG(int, ransac_inlier_threshold) = 400;

    ADD_ARG(bool, pnp_use_initial_pose) = true;
    ADD_ARG(int, pnp_max_iteration) = 2000;
    ADD_ARG(float, pnp_conf_thresh) = 0.98;
    ADD_ARG(float, pnp_projection_error) = 8.0;
    ADD_ARG(bool, pnp_refine_with_inlier) = true;
    // int num_k_retrieval;
  };

  struct Result {
    bool success = false;
    int pnp_inliers = 0;
    VisualFrame::Ptr reference_frame = nullptr;
  };

 public:
  explicit Relocalization(const Options& options);

  virtual ~Relocalization() = default;

  void setVisualMap(const VisualMap::Ptr& map) {
    visual_map_ = map;
  }

  struct PnPResult {
    bool success = false;
    int num_inliers = 0;
  };
  PnPResult solvePnPCVRansac(VisualFrame::Ptr& frame);

  std::vector<VisualFrame::Ptr> retrieveKeyframeTopK(
      const VectorXb& descriptor, size_t k);

  Result relocalize(VisualFrame::Ptr frame);

  std::vector<std::vector<VisualFrame::Ptr>> clusterCandidatesByCovisibility(
      const std::vector<VisualFrame::Ptr>& candidates);

  // std::vector<>

 protected:
  VisualMap::Ptr visual_map_ = nullptr;

  Distance::DistanceFunc distance_func;

  Options options_;
};

}  // namespace xslam
