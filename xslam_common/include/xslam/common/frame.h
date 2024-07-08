#pragma once

#include <mutex>
#include <shared_mutex>

#include "aslam/common/unique-id.h"
#include "common.h"

namespace xslam {

UNIQUE_ID_DEFINE_ID(FrameId);

// #include "orb_dbow2/dbow2/BowVector.h"
// #include "orb_dbow2/dbow2/FeatureVector.h"

// #include <aslam/cameras/camera.h>

// #include "feature.h"
// #include "keyframe.h"
// #include "mappoint.h"

// #include "../cv/pinhole_camera.h"

// #include "../config/config.h"

// #include <opencv2/core.hpp>

// class MapPoint;
// class KeyFrame;
// using FrameId = aslam::FrameId;

class Frame {
 public:
  using Ptr = std::shared_ptr<Frame>;
  using ConstPtr = std::shared_ptr<const Frame>;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame();
  Frame(const Frame& frame) = delete;
  virtual ~Frame() = default;

 public:
  inline const FrameId& id() const {
    return id_;
  }
  inline void set_id(const FrameId& id) {
    id_ = id;
  }

  // FrameId& id_mutable() {
  //   return id_;
  // }

  inline SE3 getTbw() const {
    return Twb_.inverse();
  }

  inline const SE3& getTwb() const {
    return Twb_;
  }

  inline SE3 getTwbPrior() const {
    return Twb_prior_;
  }

  inline bool hasPrior() const {
    return has_prior_;
  }

  void setTbw(const SE3& Tbw) {
    Twb_ = Tbw.inverse();
  }

  void setTwb(const SE3& Twb) {
    Twb_ = Twb;
  }

  void setTwbPrior(const SE3& Twb) {
    Twb_prior_ = Twb;
    has_prior_ = true;
  }

  const SE3& getTbwSync() const {
    std::shared_lock lock(mutex_);
    return Twb_.inverse();
  }

  const SE3& getTwbSync() const {
    std::shared_lock lock(mutex_);
    return Twb_;
  }

  void setTbwSync(const SE3& Tbw) {
    std::unique_lock lock(mutex_);
    Twb_ = Tbw.inverse();
  }

  void setTwbSync(const SE3& Twb) {
    std::unique_lock lock(mutex_);
    Twb_ = Twb;
  }

  SE3 relativeTo(const SE3& Twb) {
    return Twb_.inverse() * Twb;
  }

  //   // void getBearingsUnmatched(std::vector<size_t> &inds,
  //   //                           eigen_aligned_std_vector<Vector3>
  //   &bearings);

  //   // void getBearingsMatched(std::vector<size_t> &inds,
  //   //                         eigen_aligned_std_vector<Vector3> &bearings);

  //   void getBearingVectors(std::vector<size_t> &inds,
  //                          eigen_aligned_std_vector<Vector3> &bearings,
  //                          bool b_matched = false);

  //   bool project3(const Vector3d &pt, Vector2d *uv) const;

  //   bool project3(const Vector3d &pt, Vector3d *uvr);

  //   bool unproject3(size_t idx, Vector3d *pt3d);

  //   std::vector<size_t> getFeaturesInArea(const float &x, const float &y,
  //                                         const float &r, const int minLevel
  //                                         = -1, const int maxLevel = -1)
  //                                         const;

  //   void assignFeaturesToGrid(int grid_rows = 48, int grid_cols = 64);

  // void createFramePrior(const Matrix15d& info);
  // {
  //   CHECK(state_prior_ == nullptr);
  //   state_prior_ = FramePrior::Ptr(new FramePrior);

  //   state_prior_->ba_ = ba_;
  //   state_prior_->bg_ = bg_;
  //   state_prior_->t_b_w_ = T
  // }

  const double timestamp_second() const {
    return static_cast<double>(timestamp_) / 1e9;
  }

  void set_timestamp(const int64_t& stamp) {
    timestamp_ = stamp;
  }

 private:
  FrameId id_;

  int64_t timestamp_;
  SE3 Twb_;
  bool is_keyframe_ = false;
  SE3 Twb_prior_;
  bool has_prior_= false;

  mutable std::shared_mutex mutex_;

  //   SE3 delta_Tcw_;
};

}  // namespace xslam

UNIQUE_ID_DEFINE_ID_HASH(xslam::FrameId);
