/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:21:25
 * @LastEditors: David-Willo
 * @ Brief: grid for storing 2d keypoints, adapted from maplab and orb-slam3
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
 
#pragma once

#include "./keypoint.h"
#include "xslam/common/common.h"

namespace xslam {

// TODO: deep copy
class KeyPointGrid {
 public:
  using Ptr = std::shared_ptr<KeyPointGrid>;

  using ConstPtr = std::shared_ptr<const KeyPointGrid>;

 public:
  KeyPointGrid(
      const KeyPointContainer& keypoints, int rows, int cols, int cell_size_row,
      int cell_size_col);
  ~KeyPointGrid() = default;

  void initializeGrid();

  void assignKeyPointsToGrid();

  bool checkGridPosition(const Vector2& uv, int& px, int& py);

  std::vector<KeyPointId> getKeyPointsInArea(
      const Vector2& uv, const float& radius,
      bool b_recheck_radius = false) const;

 protected:
  int rows_, cols_;
  int cell_size_row_, cell_size_col_;
  int num_grid_row_, num_grid_col_;
  float num_grid_row_inv_, num_grid_col_inv_;

  const KeyPointContainer& keypoints_;

  std::vector<std::vector<KeyPointId>> grid_;
};

}  // namespace xslam
