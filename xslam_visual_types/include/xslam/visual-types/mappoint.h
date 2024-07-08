/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:21:46
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

// #include <atomic>
// #include <opencv2/core/core.hpp>
// #include <shared_mutex>

// #include "keyframe.h"
// #include "visual-frame.h"
// #include "map.h"
#include <aslam/common/unique-id.h>

#include "./track.h"
#include "./types.h"

#include "xslam/common/common.h"
#include "xslam/common/eigen_types.h"

namespace xslam {

// class KeyFrame;
// class Map;
// class Frame;

// struct ProjStat {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//   bool is_visible = false;

//   Vector3d uvr;

//   double dist;

//   double view_cos;

//   double scale_pred;
// };
// UNIQUE_ID_DEFINE_ID(MapPointId);

class MapPoint {
  //   friend class Map;
  //   friend class MapIO;
 public:
  using Ptr = std::shared_ptr<MapPoint>;
  using ConstPtr = std::shared_ptr<const MapPoint>;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MapPoint();

  // MapPoint(const Vector3d& pos, KeyFrame* ref_kf);

  // MapPoint(const Vector3d& pos, Frame* frame, const int& idx);

  const MapPointId id() const {
    return id_;
  }

  // inline const bool isValid() const {
  //     return is_valid_;
  // }

  inline bool is_valid() const {
    return is_valid_;
  }

  inline void set_valid() {
    is_valid_ = true;
  }
  inline void set_invalid() {
    is_valid_ = false;
  }

  inline const Vector3& position() const {
    return position_;
  }
  inline Vector3& position_mutable() {
    return position_;
  }
  inline void set_position(const Vector3& position) {
    position_ = position;
  }

  inline const Vector3& normal() const {
    return normal_;
  }
  inline Vector3& normal() {
    return normal_;
  }
  inline void setNormal(const Vector3& normal) {
    normal_ = normal;
  }

  inline const Vector3& view_dir() const {
    return view_dir_;
  }
  inline Vector3& view_dir() {
    return view_dir_;
  }
  inline void set_view_dir(const Vector3& view_dir) {
    view_dir_  = view_dir;
  }

  // TODO:
  // void computeDistinctiveDescriptors();

  const VectorXb& descriptor() const {
    return descriptor_;
  }
  // VectorXb& descriptor() {
  //   return descriptor_;
  // }

  // void updateNormalAndDepth();

  
  // bool checkScaleAndVisible(const Vector3d& t_w_c, ProjStat& stat);

  // void replace(MapPoint* mappt);

  // MapPoint* getReplaced();

  inline const KeyPointTrack& tracks() const {
    return tracks_;
  }
  inline KeyPointTrack& tracks_mutable() {
    return tracks_;
  }
  inline void set_tracks(const KeyPointTrack& tracks) {
    tracks_ = tracks;
  }

  template <typename Scalar = unsigned char>
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> descriptor() const {
    return Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>(
        (Scalar*)descriptor_data(),
        descriptor_.size() * sizeof(unsigned char) / sizeof(Scalar));
  }
  // inline const VectorXf global_descriptorf() const {
  //   return global_descriptor<float>();
  // }
  inline const unsigned char* descriptor_data() const {
    return descriptor_.data();
  }
  inline VectorXb& descriptor_mutable() {
    return descriptor_;
  }
  inline void set_descriptor(const VectorXb& descriptor) {
    descriptor_ = descriptor;
  }
  // inline const KeyPointTrack& track() const {
  //   return tracks_;
  // }

 protected:
  MapPointId id_;

  KeyPointTrack tracks_;
  Vector3 position_, normal_, view_dir_;

  VectorXb descriptor_;

  std::atomic_bool is_valid_;

  // MapPoint* ptr_replaced_ = nullptr;

  // float min_dist_ = 0.0f, max_dist_ = 0.0f;

  // std::shared_mutex mutex_pos_;
  // std::shared_mutex mutex_attr_;
};

using MapPointList = std::vector<MapPoint::Ptr>;

template <>
inline const VectorXb MapPoint::descriptor<unsigned char>() const {
  return descriptor_;
}

}  // namespace xslam
