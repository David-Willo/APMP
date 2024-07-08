/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:19:51
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-types/keypoint-grid.h"

#include "xslam/common/logging.h"

namespace xslam {

using namespace std; 

KeyPointGrid::KeyPointGrid(
    const KeyPointContainer& keypoints, int rows, int cols, int cell_size_row,
    int cell_size_col)
    : keypoints_(keypoints),
      rows_(rows),
      cols_(cols),
      cell_size_row_(cell_size_row),
      cell_size_col_(cell_size_col) {
  num_grid_row_ =
      static_cast<int>(std::ceil(static_cast<float>(rows_) / cell_size_row_));
  num_grid_col_ =
      static_cast<int>(std::ceil(static_cast<float>(cols_) / cell_size_col_));
  num_grid_col_inv_ = static_cast<float>(cell_size_col_) / cols_;
  num_grid_row_inv_ = static_cast<float>(cell_size_row_) / rows_;
  initializeGrid();
}

void KeyPointGrid::initializeGrid() {
  grid_.resize(num_grid_row_ * num_grid_col_);
}

bool KeyPointGrid::checkGridPosition(const Vector2& uv, int& px, int& py) {
  px = round((uv.x() - 0.0f) * num_grid_col_inv_);
  py = round((uv.y() - 0.0f) * num_grid_row_inv_);

  if (px < 0 || px >= num_grid_col_ || py < 0 || py >= num_grid_row_) {
    return false;
  }

  return true;
}

void KeyPointGrid::assignKeyPointsToGrid() {
  // auto isPosInGrid = [&](const Vector2d& kp, int& posX, int& posY) {
  //   posX = round((kp.x() - 0.0f) * num_grid_col_inv_);
  //   posY = round((kp.y() - 0.0f) * num_grid_row_inv_);

  //   if (posX < 0 || posX >= grid_cols || posY < 0 || posY >= grid_rows) {
  //     return false;
  //   }

  //   return true;
  // };

  for (KeyPointId i = 0; i < keypoints_.size(); i++) {
    const auto& kp = keypoints_[i];

    int gx, gy;
    if (checkGridPosition(kp.uv, gx, gy)) {
      grid_[gy * num_grid_col_ + gx].push_back(i);
    }
  }
}

std::vector<KeyPointId> KeyPointGrid::getKeyPointsInArea(
    const Vector2& uv, const float& radius, bool b_recheck_radius) const {
  std::vector<KeyPointId> indices;
  auto&& r2 = radius * radius;

  if (uv.x() < 0 || uv.x() > cols_ || uv.y() < 0 || uv.y() > rows_) {
    return indices;
  }

  const int min_cell_x =
      std::max(0, (int)floor((uv.x() - radius) * num_grid_col_inv_));
  const int max_cell_x =
      min(num_grid_col_ - 1, (int)ceil((uv.x() + radius) * num_grid_col_inv_));
  const int min_cell_y =
      max(0, (int)floor((uv.y() - radius) * num_grid_row_inv_));
  const int max_cell_y = min(
      (int)num_grid_row_ - 1, (int)ceil((uv.y() + radius) * num_grid_row_inv_));

  // INFOLOG("{}, {}, {}, {}", min_cell_x, min_cell_y, max_cell_x, max_cell_y);
  // INFOLOG("{}, {}", num_grid_col_, num_grid_row_);

  for (int ix = min_cell_x; ix <= max_cell_x; ix++) {
    for (int iy = min_cell_y; iy <= max_cell_y; iy++) {
      auto&& cell = grid_[iy * num_grid_col_ + ix];
      if (cell.empty()) {
        continue;
      }

      for (size_t j = 0, jend = cell.size(); j < jend; j++) {
        const auto& kp = keypoints_[cell[j]];
        // if (bCheckLevels) {
        //   if (kpUn.octave < minLevel)
        //     continue;
        //   if (maxLevel >= 0)
        //     if (kpUn.octave > maxLevel)
        //       continue;
        // }

        if (b_recheck_radius) {
          auto&& dist2 = kp.distance2(uv);

          if (dist2 <= r2) {
            indices.push_back(cell[j]);
          }
        } else {
          indices.push_back(cell[j]);
        }
      }
    }
  }

  return indices;
}

}  // namespace xslam