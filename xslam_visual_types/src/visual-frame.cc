/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-09-26 14:40:52
 * @LastEditTime: 2024-07-02 07:22:51
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-types/visual-frame.h"

#include <algorithm>

namespace xslam {
using namespace std;

void VisualFrame::releaseImage() {
  image_.release();
  // image_ = cv::Mat();
}

void VisualFrame::releaseDescriptors() {
  descriptors_.resize(0,0);
  global_descriptor_.resize(0);
}

std::vector<KeyPointId> VisualFrame::getKeyPointIndicesInGrid(
    const Vector2& uv, const float radius, const float radius2, std::pair<float, float>* depth_info) const {
  // CHECK_NOTNULL(keypoint_grid_);
  // return keypoint_grid_->getKeyPointsInArea(uv, radius);

  std::vector<KeyPointId> output;
  float radius_for_mean = radius2>0? radius2: radius;
  std::vector<float> valid_depths;
  
  // just get all neighbor points 
  auto&& candidates = keypoint_grid_->getKeyPointsInArea(uv, radius_for_mean, false);

  output.reserve(candidates.size());

  for_each(candidates.begin(), candidates.end(), [&](const KeyPointId& ind) {
    auto pixel_dist = keypoint(ind).distance(uv);
    if (pixel_dist <= radius) {
      output.push_back(ind);
    }
    
    if (pixel_dist <= radius_for_mean && mappoint(ind)) {
      // has valid mappoint
      valid_depths.push_back((this->getTbw()*mappoint(ind)->position()).z());
    }
  });

  // std::cout << valid_depths.size() <<  " valid depths in " <<  radius_for_mean << "\n";
  if (depth_info != nullptr && valid_depths.size() > 5) {
    float mean = std::accumulate(valid_depths.begin(), valid_depths.end(), 0.0,
                                 [](float sum, float val) { return sum + val; }) / valid_depths.size();

    // Compute the standard variance using lambda expression and accumulate function
    float variance = std::accumulate(valid_depths.begin(), valid_depths.end(), 0.0,
                                     [mean](float sum, float val) { return sum + (val - mean) * (val - mean); }) / valid_depths.size();

    *depth_info = std::make_pair(mean,  std::sqrt(variance));
  }

  return output;
}


std::vector<KeyPointId> VisualFrame::getUntrackedKeyPointIndicesInGrid(
    const Vector2& uv, float radius) const {
  
  std::vector<KeyPointId> output;
  auto&& candidates = keypoint_grid_->getKeyPointsInArea(uv, radius, false);
  output.reserve(candidates.size());

  for_each(candidates.begin(), candidates.end(), [&](const KeyPointId& ind) {
    if (!mappoint(ind) && keypoint(ind).distance(uv) <= radius) {
      output.push_back(ind);
    }
  });

  return output;
}

void VisualFrame::assignKeyPointsToGrid(int cell_size_row, int cell_size_col) {
  keypoint_grid_ = make_shared<KeyPointGrid>(
      keypoints_, camera_->imageHeight(), camera_->imageWidth(), cell_size_row,
      cell_size_col);

  keypoint_grid_->assignKeyPointsToGrid();
}

bool VisualFrame::project3(const Vector3d& pt, Vector2d* uv) const {
  CHECK_NOTNULL(uv);
  Vector3d ptc;
  ptc = getTbw() * pt;

  if (ptc.z() < 0.0) {
    return false;
  }

  auto proj_res = camera_->project3(ptc, uv);
  // INFOLOG("proj stat: {}", proj_res.getDetailedStatus())
  if (!proj_res.isKeypointVisible()) {
    return false;
  }

  return true;
}
// bool VisualFrame::project3(const Vector3d& pt, Vector3d* uvr);

void VisualFrame::set_keypoints(const KeyPointContainer& keypoints) {
  keypoints_.clear();
  std::copy(keypoints.begin(), keypoints.end(), std::back_inserter(keypoints_));
}

void VisualFrame::updateConnections() {
  frame_weight_list_.clear();
  frame_weight_list_.reserve(frame_connections_.size());
  for_each(
      frame_connections_.begin(), frame_connections_.end(),
      [this](const FrameWeight& fw) { frame_weight_list_.push_back(fw); });
  sort(
      frame_weight_list_.begin(), frame_weight_list_.end(),
      [](const pair<FrameId, uint32_t>& lhs,
         const pair<FrameId, uint32_t>& rhs) {
        return lhs.second > rhs.second;
      });
}

MapPointIdSet VisualFrame::mappoint_indices_tracked() {
  MapPointIdSet set_mappoints;
  set_mappoints.reserve(num_keypoint_);
  for_each(
      mappoints_.begin(), mappoints_.end(),
      [&set_mappoints](const MapPoint::ConstPtr& mappoint) {
        if (mappoint && mappoint->is_valid()) {
          set_mappoints.insert(mappoint->id());
        }
      });

  return set_mappoints;
}

std::tuple<MatrixXb, std::vector<KeyPointId>> VisualFrame::descriptors_untracked() {
  MatrixXb output;
  std::vector<KeyPointId> inds;
  // MapPointList mappoints_tracked;
  inds.reserve(mappoints_.size());
  // mappoints_tracked.reserve(mappoints_.size());
  for (size_t i = 0; i < mappoints_.size(); i++) {
    auto&& mappoint = mappoints_[i];
    if (!mappoint) {
      inds.push_back(i);
    }
  }

  output.resize(descriptors_.rows(), inds.size());
  for (size_t i = 0; i < inds.size(); i++) {
    output.col(i) = descriptors_.col(inds[i]);
  }

  return {output, inds};
}


std::tuple<MatrixXb, MapPointList> VisualFrame::descriptors_tracked() {
  MatrixXb output;
  std::vector<KeyPointId> inds;
  MapPointList mappoints_tracked;
  inds.reserve(mappoints_.size());
  mappoints_tracked.reserve(mappoints_.size());
  for (size_t i = 0; i < mappoints_.size(); i++) {
    auto&& mappoint = mappoints_[i];
    if (mappoint) {
      inds.push_back(i);
      mappoints_tracked.push_back(mappoint);
    }
  }

  output.resize(descriptors_.rows(), inds.size());
  for (size_t i = 0; i < inds.size(); i++) {
    output.col(i) = descriptors_.col(inds[i]);
  }

  return {output, mappoints_tracked};
}

int VisualFrame::countTrackedMapPoints() {
  int num_tracked = 0;
  for (auto&& mappoint : mappoints_) {
    if (mappoint) {
      num_tracked++;
    }
  }

  return num_tracked;
}

// remove not triangulated keypoints
void VisualFrame::compress() {
  KeyPointContainer keypoints;
  MapPointList mappoints;
  keypoints.reserve(num_keypoint_);
  mappoints.reserve(num_keypoint_);

  CHECK_EQ(keypoints_.size(), num_keypoint_);
  CHECK_EQ(mappoints_.size(), num_keypoint_);

  std::vector<KeyPointId> indices;
  for (size_t i = 0; i < num_keypoint_; i++) {
    auto& mappt = mappoints_[i];
    if (!mappt) {
      continue;
    }

    indices.push_back(i);
  }

  num_keypoint_ = indices.size();
  MatrixXb _descriptors(descriptors_.rows(), num_keypoint_);
  for (size_t i = 0; i < num_keypoint_; i++) {
    auto&& ind = indices[i];

    _descriptors.col(i) = descriptors_.col(ind);
    keypoints.push_back(keypoints_.at(ind));
    mappoints.push_back(mappoints_.at(ind));

    mappoints.back()->tracks_mutable().addTrack(id(), i);
  }

  descriptors_ = _descriptors;
  keypoints_ = keypoints;
  mappoints_ = mappoints;
}

void VisualFrame::initializeSearchIndex() {
  int dim = 256;
  int nlist = num_keypoint_ > 8 ? 8 : 1;
  quantizer_ = make_unique<faiss::IndexFlatL2>(dim);
  index_ = make_unique<faiss::IndexIVFFlat>(quantizer_.get(), dim, nlist);
  // TODO(DW): try IndexIVFPQ for lower memory consumption
  index_->train(num_keypoint_, (float*)descriptors_.data());
  index_->add(num_keypoint_, (float*)descriptors_.data());

  // index.train(num_desc, (float*)desc_all.data());
  // index.add(num_desc, (float*)desc_all.data());
}

// MapPointIdList VisualFrame::mappoint_indices_tracked() {
//   MapPointIdList inds;
//   inds.reserve(mappoints_.size());
//   for_each(mappoints_.begin(), mappoints_.end(), [&inds](const MapPointId&
//   id) {
//     if (id.isValid()) {
//       inds.push_back(id);
//     }
//   });

//   return inds;
// }

}  // namespace xslam