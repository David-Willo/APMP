/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-27 06:10:56
 * @LastEditTime: 2024-07-02 07:21:55
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include <optional>
#include <unordered_map>

#include "types.h"
// #include "visual-frame.h"
#include "xslam/common/frame.h"

namespace xslam {

/**
 * @brief
 *
 */
// struct KeyPointId {
//   KeyPointId keypoint_index;
// };

using TrackElementTable = std::unordered_map<FrameId, KeyPointId>;

/**
 * @brief
 *
 */
class KeyPointTrack {
 public:
  KeyPointTrack() = default;

  ~KeyPointTrack() = default;

  inline size_t size() const {
    return elements_.size();
  }

  inline std::optional<const KeyPointId> track(const FrameId& id) const {
    auto iter = elements_.find(id);
    if (iter != elements_.end()) {
      return iter->second;
    }

    // return;
  }

  inline bool checkTrack(const FrameId& id) const {
    return elements_.find(id) != elements_.end();
  }

  inline void addTrack(const FrameId& frame_id, const KeyPointId& kp_id) {
    elements_[frame_id] = kp_id;
  }

  // KeyPointId& getTrack(const FrameId& id) {
  //   return elements_[id];
  // }

  // inline void addTrack(const FrameId& id, const KeyPointId& element) {

  // }
  TrackElementTable::const_iterator begin() const {
    return elements_.begin();
  }
  TrackElementTable::const_iterator end() const {
    return elements_.end();
  }
  // TrackElementTable::iterator begin() {
  //   return elements_.begin();
  // }
  // TrackElementTable::iterator end() {
  //   return elements_.end();
  // }

  inline void deleteTrack(const FrameId& id) {
    auto iter = elements_.find(id);
    if (iter != elements_.end()) {
      elements_.erase(iter);
    }
  }

  inline void reserve(const size_t num_elements) {
    elements_.reserve(num_elements);
  }

  inline void clear() {
    elements_.clear();
  }

 protected:
  TrackElementTable elements_;
};

}  // namespace xslam
