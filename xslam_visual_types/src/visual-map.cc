/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:22:55
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-types/visual-map.h"

#include "xslam/common/logging.h"

namespace xslam {
 
using namespace std;

void VisualMap::addMapPoint(MapPoint::Ptr mappt) {
  mappoints_.emplace(mappt->id(), mappt);
}

void VisualMap::addKeyFrame(VisualFrame::Ptr frame) {
  keyframes_.emplace(frame->id(), frame);
}

std::vector<VisualFrame::Ptr> VisualMap::getAllFrames() {
  std::vector<VisualFrame::Ptr> frames;
  frames.reserve(keyframes_.size());
  for (auto&& frame : keyframes_) {
    frames.push_back(frame.second);
  }

  return frames;
}

std::vector<MapPoint::Ptr> VisualMap::getAllMapPoints() {
  std::vector<MapPoint::Ptr> mappoints;
  mappoints.reserve(mappoints_.size());
  for (auto&& frame : mappoints_) {
    mappoints.push_back(frame.second);
  }

  return mappoints;
}

std::vector<VisualFrame::Ptr> VisualMap::getConnectedFramesTopK(
    const VisualFrame::Ptr& frame, int k) {
  //
  std::vector<VisualFrame::Ptr> frames;
  frames.reserve(frame->frame_connections().size());
  int iter = 0;
  for (auto&& [frame_id, weight] : frame->frame_connections_weight()) {
    frames.push_back(keyframes_[frame_id]);

    if (frames.size() == k) {
      break;
    }
  }

  return frames;
}

std::vector<VisualFrame::Ptr> VisualMap::getConnectedFramesByWeight(
    const VisualFrame::Ptr& frame, int weight_thresh) {
  //
  std::vector<VisualFrame::Ptr> frames;
  frames.reserve(frame->frame_connections().size());
  int iter = 0;
  for (auto&& [frame_id, weight] : frame->frame_connections_weight()) {
    // INFOLOG("{} {}", iter++, weight);
    if (weight < weight_thresh) {
      break;
    }

    // TODO: should we count frame_id?
    frames.push_back(keyframes_[frame_id]);
  }

  return frames;
}

// TODO: mutex
void VisualMap::updateMapPointDescriptor(
    MapPoint::Ptr& mappoint, Distance::DistanceType dist_type) {
  auto dist_func = Distance::createDistanceFunc<float>(dist_type);
  auto num_tracks = mappoint->tracks().size();

  aligned_std_vector<VectorXb> descriptors;
  descriptors.reserve(num_tracks);
  for (auto&& [frame_id, point_id] : mappoint->tracks()) {
    auto&& keyframe = keyframes_[frame_id];
    descriptors.push_back(keyframe->descriptor(point_id));
  }

  std::vector<std::vector<float>> distance;
  distance.resize(num_tracks, vector<float>(num_tracks, 0.0f));

  for (size_t i = 0; i < num_tracks; i++) {
    distance[i][i] = 0.0f;
    for (size_t j = i + 1; j < num_tracks; j++) {
      float distij = dist_func(descriptors[i], descriptors[j]);
      distance[i][j] = distij;
      distance[j][i] = distij;
    }
  }

  float best_median = numeric_limits<float>::max();
  int best_idx = 0;
  for (size_t i = 0; i < num_tracks; i++) {
    auto& dist_vec = distance[i];
    auto mid_iter = dist_vec.begin() + num_tracks / 2;
    std::nth_element(dist_vec.begin(), mid_iter, dist_vec.end());
    auto&& dist_median = dist_vec[num_tracks / 2];

    if (dist_median < best_median) {
      best_median = dist_median;
      best_idx = i;
    }
  }

  mappoint->set_descriptor(descriptors[best_idx]);
}

 void VisualMap::cacheMapPointsPCL() {
    mappoint_pcl_.reset(new PointCloudType);
    for (const auto& [id, mpt] : mappoints_) {
      if (!mpt->is_valid()) {
        continue;
      }
      PointType point;
      point.x = mpt->position()[0];
      point.y = mpt->position()[1];
      point.z = mpt->position()[2];
      mappoint_pcl_->push_back(point);
    }
    kdtree_.setInputCloud(mappoint_pcl_);
 }

 void VisualMap::emptyCachePCL() {
    mappoint_pcl_ = nullptr;
 }

 std::vector<int> VisualMap::searchByRadiusPCL(const PointType& searchPoint, double radius) {
    std::vector<int> pointIndices;
    if (mappoint_pcl_ == nullptr) {
      return pointIndices;
    }
    
    std::vector<float> pointRadiusSquaredDistance;
    kdtree_.radiusSearch(searchPoint, radius, pointIndices, pointRadiusSquaredDistance);
    return pointIndices;
 }

// std::vector<MapPoint::Ptr> VisualMap::getTrackedMapPoints(
//     const VisualFrame::Ptr& frame) {
//   std::vector<MapPoint::Ptr> mappoints;
//   auto&& inds = frame->();
//   mappoints.reserve(inds.size());

//   for_each(inds.begin(), inds.end(), [&](const MapPointId& id) {
//     mappoints.push_back(mappoints_[id]);
//   });

//   return mappoints;
// }

// void VisualMap::getTrackedMapPointsAndKeyPoints(
//     const VisualFrame::Ptr& frame, std::vector<MapPoint::Ptr>& mappoints,
//     std::vector<KeyPoint*>& keypoints) {
//   mappoints.clear();
//   keypoints.clear();
//   mappoints.reserve(frame->num_keypoints());
//   keypoints.reserve(frame->num_keypoints());

//   for (size_t i = 0; i < frame->num_keypoint_; i++) {
//     auto&& id = frame->mappoints_[i];
//     if (!id.isValid()) {
//       continue;
//     }

//     mappoints.push_back(mappoints_[id]);
//     keypoints.push_back(&frame->keypoints_[i]);
//   }
// }

}  // namespace xslam