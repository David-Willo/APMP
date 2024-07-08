/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-09-26 14:40:52
 * @LastEditTime: 2024-07-02 07:08:56
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */


#pragma once

#include "xslam/common/arg.h"
#include "xslam/visual-types/visual-map.h"
#include <fstream>


namespace xslam {
class Tracking {
 public:
  using Ptr = std::shared_ptr<Tracking>;

  using ConstPtr = std::shared_ptr<const Tracking>;

  struct Options {
    ADD_ARG(bool, tracking_on_init) = false;

    ADD_ARG(int, inlier_threshold_motion) = 100;
    ADD_ARG(int, inlier_threshold_keyframe) = 100;
    ADD_ARG(int, inlier_threshold_map) = 100;
    ADD_ARG(int, matching_threshold_motion) = 100;
    // ADD_ARG(int, matching_threshold_keyframe) = 100;
    ADD_ARG(int, matching_threshold_map) = 100;

    ADD_ARG(int, num_keyframes_local) = 20;

    ADD_ARG(float, ratio_match_keyframe) = 0.1;
  };

 public:
  explicit Tracking(Options options);

  virtual ~Tracking() = default;

  void initialize(VisualFrame::Ptr& frame_init, VisualFrame::Ptr& frame_ref);

  bool track(VisualFrame::Ptr& frame, bool use_prior=false);

  bool trackLastFrame(bool use_prior=false);

  bool trackKeyFrame();

  bool trackLocalMap(bool use_prior=false);

  void updateLocalMap();

  void updateLocalMapFast();

  void searchLocalMap(const float radius=24, const  float threshold=1);

  inline const VisualMap::Ptr& visual_map() const {
    return visual_map_;
  }
  inline VisualMap::Ptr& visual_map_mutable() {
    return visual_map_;
  }
  inline void set_visual_map(const VisualMap::Ptr& visual_map) {
    visual_map_ = visual_map;
  }
  inline const size_t get_total_track_points() {
    return point_track_cnt_;
  }
  inline const size_t get_total_detect_points() {
    return point_detect_cnt_;
  }

  inline const void dump_track_cnt_history(std::string outfile) {
    std::ofstream fs(outfile);
    for(const auto cnt : point_track_rec_) {
      fs << cnt << ", ";
    }
    fs.close();
  }

  inline const void dump_detect_cnt_history(std::string outfile) {
    std::ofstream fs(outfile);
    for(const auto cnt : point_detct_rec_) {
      fs << cnt << ", ";
    }
    fs.close();
  }

 protected:
  VisualMap::Ptr visual_map_ = nullptr;

  Options options_;

  VisualFrame::Ptr frame_curr_ = nullptr, frame_last_ = nullptr;
  size_t tracked_curr_, tracked_last_ = 0;
  VisualFrame::Ptr frame_ref_ = nullptr;

  std::vector<VisualFrame::Ptr> local_keyframes_;
  // std::unordered_map<MapPoint::Ptr, ProjectionStatus> local_mappoints_;
  std::vector<MapPoint::Ptr> local_mappoints_;
  std::vector<ProjectionStatus> local_mappoints_projection_status_;

  // debug use, to measure map and loc corresnpondence quality
  size_t  point_track_cnt_ = 0;
  size_t  point_detect_cnt_ = 0;
  std::vector<size_t> point_track_rec_;  
  std::vector<size_t> point_detct_rec_;

};

}  // namespace xslam