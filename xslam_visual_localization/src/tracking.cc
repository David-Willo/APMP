/**
 * @file tracking.cc
 * @author hyhuang hhuangat@gmail.com
 * @brief
 * @version 0.1
 * @date 2023-03-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "xslam/visual-localization/tracking.h"
/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 06:13:45
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/common/logging.h"
#include "xslam/visual-localization/matching.h"
#include "xslam/visual-localization/optimization.h"
#include "xslam/visual-localization/debug_visualize.h"

namespace xslam {
using namespace std;

Tracking::Tracking(Options options) : options_(options) {}

void Tracking::initialize(VisualFrame::Ptr& frame_init,
                          VisualFrame::Ptr& frame_ref) {
  CHECK_NOTNULL(frame_ref);
  CHECK_NOTNULL(visual_map_);

  frame_curr_ = frame_init;
  frame_ref_ = frame_ref;

  // local_keyframes_.clear();
  // local_keyframes_.push_back(frame_ref_);

  if (options_.tracking_on_init()) {
    trackLocalMap();
  }

  frame_last_ = frame_init;

  // // //   clearTemporalPoints();

  // for (size_t i = 0; i < frame_curr_->num_feats_; i++) {
  //   if (frame_curr_->mappoints_[i] && frame_curr_->is_outlier_[i]) {
  //     frame_curr_->mappoints_[i] = nullptr;
  //   }
  // }

  // last_frame_ = init_frame;
}

bool Tracking::track(VisualFrame::Ptr& frame, bool use_prior) {
  CHECK_NOTNULL(frame_last_);
  frame_curr_ = frame;
  int keypoints_track = this->tracked_last_;
  bool track_success = true;

  trackLastFrame(use_prior);  // to estimate temp pose
  keypoints_track = frame_curr_->countTrackedMapPoints();
  debug_vis::vis_track =  (frame_curr_->visualizeFrameInfo("after track", 2)).clone();

  trackLocalMap();
  tracked_curr_ = frame_curr_->countTrackedMapPoints();

  // anyway, add to history for debug track count
  this->point_track_cnt_ += tracked_curr_;
  this->point_track_rec_.push_back(tracked_curr_);
  this->point_detect_cnt_ += frame->keypoints().size();
  this->point_detct_rec_.push_back(frame->keypoints().size());

  // break when get too worse after match against map, pose estimation not
  // stable
  if (tracked_curr_ < options_.inlier_threshold_map()) {
    // not easy break, if track 1k already has 100 remain, easy to drop to
    // around 90 something the second part should be <= because if
    // keypoints_track = 0, keypoints_track2 would also be 0
    if (tracked_curr_ < 10 ||
        keypoints_track < options_.inlier_threshold_motion() &&
            tracked_curr_ <= keypoints_track * 0.8) {
      track_success = false;
    }
  }

  this->tracked_last_ = tracked_curr_;
  this->frame_last_ = frame_curr_;
  return track_success;
}

bool Tracking::trackKeyFrame() {
  auto matching_result = Matching::matchAgainstFrameIVF(
      frame_curr_, frame_ref_,
      Matching::Options()
          .distance_type(Distance::DISTANCE_L2_NORM)
          .nn_ratio(0.75f)
          .matching_threshold(0.45f));

  WARNLOG("#matches against key frame: {}", matching_result.num_match);
  // if (matching_result.num_match < options_.matching_threshold_keyframe()) {
  //   // TODO:
  //   return false;
  // }

  Optimization::optimizeFramePosePnP(
      frame_curr_,
      Optimization::Options().use_robust_kernel(true).projection_error(8.0f));
  auto num_tracked = frame_curr_->countTrackedMapPoints();
  WARNLOG("#tracked after keyframe matching: {}", num_tracked);

  if (num_tracked < options_.inlier_threshold_keyframe()) {
    return false;
  }

  return true;
}

bool Tracking::trackLastFrame(bool use_prior) {
  auto matching_result = Matching::matchAgainstFrameLK(
      frame_curr_, frame_last_,
      Matching::Options()
          .distance_type(Distance::DISTANCE_L2_NORM)
          .matching_threshold(1.0f)
          .search_radius(30));
  // .search_radius(8));
  // maybe smaller radius for LK, search for best descriptor match may not be
  // good when lk track is not enough

  INFOLOG("# LK against last frame: {}", matching_result.num_match);

  // if (matching_result.num_match < options_.matching_threshold_motion()) {
  //   // TODO:
  //   matching_result = Matching::matchAgainstFrame(
  //       frame_curr_, frame_last_,
  //       Matching::Options()
  //           .distance_type(Distance::DISTANCE_L2_NORM)
  //           .matching_threshold(0.7f)
  //           .search_radius(80));
  //   if (matching_result.num_match < options_.matching_threshold_motion()) {
  //     return false;
  //   }
  // }
  SE3 prior_pose = frame_curr_->getTwb();
  if (use_prior) {
    frame_curr_->setTwbPrior(prior_pose);
  }

  // DW: even when use prior, keep this optimization to reject ouliers
  Optimization::optimizeFramePosePnP(
      frame_curr_,
      Optimization::Options().use_robust_kernel(true).projection_error(8.0f).add_prior(use_prior));
  auto num_tracked = frame_curr_->countTrackedMapPoints();
  INFOLOG("# Opted tracked against last frame: {}", num_tracked);

  auto se3_rel = (frame_curr_->getTbw() * prior_pose);
  auto angular_diff = std::acos((se3_rel.rotationMatrix().trace() - 1.0) / 2.0)* (180.0 / M_PI);
  auto translation_diff = se3_rel.translation().norm();
  bool is_drift = angular_diff > 10 || translation_diff > 0.5 || num_tracked < matching_result.num_match*0.6;

  // DW: the imu prior may contradict with lk?
  // if (num_tracked < options_.inlier_threshold_motion()) {
  // if (true) {
  if (is_drift && use_prior) {
  // if (use_prior) {
    // if majority of mappoints are outliers, reset frame status
    WARNLOG("Track last frame fail, reset to prior");
    frame_curr_->setTwb(prior_pose);
    // DW: now reset mappoints do nothing, dummy function here
    frame_curr_->reset_mappoints();
    // return false;
  }

  // overwrite inlier descriptors to make sure inlier lk mappoints won't be overwrite in localmap tracking
  // frame_curr_->overwrite_mappoint_descriptors();

  return true;
}

bool Tracking::trackLocalMap(bool use_prior) {
  int num_matches_map;
  // INFOLOG("#tracked: {}", frame_curr_->countTrackedMapPoints());
  // Update Local KeyFrames and Local Points
  // updateLocalMap();
  updateLocalMapFast();

  searchLocalMap(24, 1);
  // INFOLOG("#matches against map: {}", matching_result.num_match);

  Optimization::optimizeFramePosePnP(
      frame_curr_,
      Optimization::Options().use_robust_kernel(true).projection_error(8.0f).add_prior(use_prior));

  auto num_tracked = frame_curr_->countTrackedMapPoints();
  INFOLOG("# Opted tracked against map: {}", num_tracked);
  debug_vis::vis_match =  (frame_curr_->visualizeFrameInfo("before fill", 2)).clone();
  
  
  if (num_tracked < options_.inlier_threshold_map()) {
    WARNLOG("Low number of tracked points")
    // return false;
  } else {
    // for inliers, update descriptors for smooth online tracking
    frame_curr_->overwrite_mappoint_descriptors();
    // after match, enrich the matching results, but using strict threshold
    searchLocalMap(3, 1); // for ust

  }

  // searchLocalMap(8, 1); // for kitti [[deprecated]]
  // searchLocalPoints(2);
  debug_vis::vis_reload =  (frame_curr_->visualizeFrameInfo("after fill", 2)).clone();

  // num_matches_map = trackLocalMap();
  // stat_.num_match_inliers = num_matches_map;
  return true;
}

// TODO: covisible frames by weight
void Tracking::updateLocalMapFast() {
  // update local keyframes
  {
    unordered_set<VisualFrame::Ptr> set_local_keyframes;
    unordered_map<FrameId, int> keyframe_counter;

    // timing::Timer timer_ckf("track/count_kf");
    for (size_t i = 0; i < frame_curr_->num_keypoints(); i++) {
      auto&& mappoint = frame_curr_->mappoint(i);
      if (!mappoint) {
        continue;
      }
      // MapPoint* mappt = frame_curr_->mappoints_[i];
      // TODO: validation check when multi-threading
      for (const auto& [frame_obs, ignore] : mappoint->tracks()) {
        keyframe_counter[frame_obs]++;
      }
    }

    if (keyframe_counter.empty()) {
      return;
    }

    int count_max = 0;
    VisualFrame::Ptr keyframe_max = nullptr;

    int num_local = 0;
    for (auto&& [keyframe_ind, count] : keyframe_counter) {
      // if (kf_ptr->not_valid_)
      //   continue;

      auto&& keyframe = visual_map_->getKeyFrame(keyframe_ind);
      CHECK_NOTNULL(keyframe);

      if (count > count_max) {
        count_max = count;
        keyframe_max = keyframe;
      }
      // local_keyframes_.push_back(keyframe);
    }

    if (keyframe_max) {
      frame_ref_ = keyframe_max;
    }
    local_keyframes_.clear();
    local_keyframes_.push_back(keyframe_max);

    auto keyframes_ref = visual_map_->getConnectedFramesTopK(
        keyframe_max, options_.num_keyframes_local());
    std::copy(keyframes_ref.begin(), keyframes_ref.end(),
              std::back_inserter(local_keyframes_));

    // timer_ckf.Stop();
  }
  // INFOLOG("a");

  // update local mappoints
  {
    // timing::Timer timer("track/count_mp");
    unordered_set<MapPoint::Ptr> set_local_mappoints;
    for (auto&& keyframe : local_keyframes_) {
      for_each(keyframe->mappoints().begin(), keyframe->mappoints().end(),
               [&set_local_mappoints](const MapPoint::Ptr& mappoint) {
                 if (mappoint && mappoint->is_valid()) {
                   set_local_mappoints.insert(mappoint);
                 }
               });
    }

    local_mappoints_.clear();
    local_mappoints_.assign(set_local_mappoints.begin(),
                            set_local_mappoints.end());
    // timer.Stop();
  }

  // INFOLOG(
  //     "#local keyframes: {}, #local mappoints: {}", local_keyframes_.size(),
  //     local_mappoints_.size());
}
void Tracking::updateLocalMap() {
  // update local keyframes
  {
    unordered_set<VisualFrame::Ptr> set_local_keyframes;
    unordered_map<FrameId, int> keyframe_counter;

    // timing::Timer timer_ckf("track/count_kf");
    for (size_t i = 0; i < frame_curr_->num_keypoints(); i++) {
      auto&& mappoint = frame_curr_->mappoint(i);
      if (!mappoint) {
        continue;
      }
      // MapPoint* mappt = frame_curr_->mappoints_[i];
      // TODO: validation check when multi-threading
      for (const auto& [frame_obs, ignore] : mappoint->tracks()) {
        keyframe_counter[frame_obs]++;
      }
    }

    if (keyframe_counter.empty()) {
      return;
    }

    int count_max = 0;
    VisualFrame::Ptr keyframe_max = nullptr;

    local_keyframes_.clear();
    local_keyframes_.reserve(keyframe_counter.size());

    int num_local = 0;
    for (auto&& [keyframe_ind, count] : keyframe_counter) {
      // if (kf_ptr->not_valid_)
      //   continue;

      auto&& keyframe = visual_map_->getKeyFrame(keyframe_ind);
      CHECK_NOTNULL(keyframe);

      if (count > count_max) {
        count_max = count;
        keyframe_max = keyframe;
      }
      local_keyframes_.push_back(keyframe);
    }
    // WARNLOG("#local kfs: {}", num_local);



    if (keyframe_max) {
      frame_ref_ = keyframe_max;
    }

    // timer_ckf.Stop();
  }


  // update local mappoints
  {
    // timing::Timer timer("track/count_mp");
    unordered_set<MapPoint::Ptr> set_local_mappoints;
    for (auto&& keyframe : local_keyframes_) {
      for_each(keyframe->mappoints().begin(), keyframe->mappoints().end(),
               [&set_local_mappoints](const MapPoint::Ptr& mappoint) {
                 if (mappoint && mappoint->is_valid()) {
                   set_local_mappoints.insert(mappoint);
                 }
               });
    }

    local_mappoints_.clear();
    local_mappoints_.assign(set_local_mappoints.begin(),
                            set_local_mappoints.end());
    // timer.Stop();
  }

  // INFOLOG(
  //     "#local keyframes: {}, #local mappoints: {}", local_keyframes_.size(),
  //     local_mappoints_.size());
}

// TODO: check view angle
void Tracking::searchLocalMap(const float radius, const float threshold) {
  //
  int num_candidates = 0;
  auto&& mappoints_tracked =
      frame_curr_->mappoint_indices_tracked();  // from last frame

  local_mappoints_projection_status_.resize(local_mappoints_.size());
  for (size_t i = 0; i < local_mappoints_.size(); i++) {
    auto&& mappoint = local_mappoints_[i];
    auto& proj_status = local_mappoints_projection_status_[i];
    // skip process those already tracked
    if (!mappoint->is_valid() || mappoints_tracked.count(mappoint->id())) {
      continue;
    }

    Vector2 uv;
    proj_status.is_visible = false;

    if (!frame_curr_->project3(mappoint->position(), &uv)) {
      continue;
    }

    proj_status.is_visible = true;
    proj_status.uvd << uv, -1.0;
    num_candidates++;
  }

  INFOLOG("# candidates against map: {}/{}", num_candidates, local_mappoints_.size());

  if (num_candidates > 0) {
    //
    Matching::matchAgainstMapPoints(
        frame_curr_, local_mappoints_, local_mappoints_projection_status_,
        Matching::Options()
            .distance_type(Distance::DISTANCE_L2_NORM)
            .search_radius(radius)
            // .search_radius(12)
            // .nn_test(false) // for kitti only ? [[deprecated]]
            // maybe no need nn test, just match all, we believe the map point 
            // results
            .matching_threshold(threshold));
    // DW: maybe no need to set threshold, if visible, just matched them.
    // .matching_threshold(0.85));
  }
}

}  // namespace xslam