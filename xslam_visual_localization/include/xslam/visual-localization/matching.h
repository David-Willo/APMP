/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-06-08 03:21:59
 * @LastEditTime: 2024-07-02 07:08:31
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include <functional>

#include "xslam/common/arg.h"
#include "xslam/common/distances.h"
#include "xslam/common/eigen_types.h"
#include "xslam/visual-types/visual-frame.h"
#include "xslam/visual-types/visual-map.h"

namespace xslam {

struct Match {
  //
};

class Matching {
 public:
  struct Options {
    // ADD_ARG(Distance::DistanceFunc, distance_func);
    ADD_ARG(Distance::DistanceType, distance_type);
    ADD_ARG(bool, nn_test) = true;
    ADD_ARG(float, nn_ratio) = 0.9;

    ADD_ARG(float, search_radius) = 24;
    ADD_ARG(float, matching_threshold) = 0.;

    ADD_ARG(bool, untracked) = false;
  };

  struct Result {
    int num_match = 0;
    // std::vector<MapPoint::Ptr> mappoints;
  };

 public:
  Matching(/* args */) = delete;
  virtual ~Matching() = default;

  // void set_distance();
  // TODO: how to organize the matching results
  static Result matchAgainstCluster(
      VisualFrame::Ptr frame, const std::vector<VisualFrame::Ptr>& cluster,
      VisualMap::Ptr& visual_map, const Options& options);

  static Result matchAgainstFrameIVF(
      VisualFrame::Ptr frame, const VisualFrame::ConstPtr& frame_ref,
      const Options& options);

  static Result matchAgainstFrame(
      VisualFrame::Ptr frame, const VisualFrame::ConstPtr& frame_ref,
      const Options& options);
  static Result matchAgainstFrameLK(
      VisualFrame::Ptr frame, const VisualFrame::ConstPtr& frame_ref,
      const Options& options);

  static void matchAgainstMapPoints(
      VisualFrame::Ptr frame, const std::vector<MapPoint::Ptr>& mappoints,
      const std::vector<ProjectionStatus>& projection_status,
      const Options& options);

  template <typename T>
  static void reduceVector(std::vector<T> &v, std::vector<uchar> status) {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
  }

  static void rejectWithFundamentalMat(std::vector<cv::Point2f>& keypoints_last, std::vector<cv::Point2f>& keypoints_cur, std::vector<int>& ids, const aslam::Camera::Ptr& camera) {
      if (keypoints_cur.size() >= 8)
      {
          // ROS_DEBUG("FM ransac begins");
          // TicToc t_f;
          std::vector<cv::Point2f> un_cur_pts(keypoints_last.size()), un_forw_pts(keypoints_cur.size());
          for (unsigned int i = 0; i < keypoints_last.size(); i++)
          {
              Eigen::Vector3d tmp_p;
              camera->backProject3(Eigen::Vector2d(keypoints_last[i].x, keypoints_last[i].y), &tmp_p);
              tmp_p.x() = camera->getParameters()[0] * tmp_p.x() / tmp_p.z() + camera->imageWidth() / 2.0;
              tmp_p.y() = camera->getParameters()[1] * tmp_p.y() / tmp_p.z() + camera->imageHeight() / 2.0;
              un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

              camera->backProject3(Eigen::Vector2d(keypoints_cur[i].x, keypoints_cur[i].y), &tmp_p);
              tmp_p.x() = camera->getParameters()[0] * tmp_p.x() / tmp_p.z() + camera->imageWidth() / 2.0;
              tmp_p.y() = camera->getParameters()[1] * tmp_p.y() / tmp_p.z() + camera->imageHeight() / 2.0;
              un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
          }

          std::vector<uchar> status;
          const double F_THRESHOLD = 1;
          cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
          int size_a = keypoints_last.size();
          reduceVector(keypoints_last, status);
          reduceVector(keypoints_cur, status);
          reduceVector(ids, status);
          // ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, keypoints_cur.size(), 1.0 * keypoints_cur.size() / size_a);
          // ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
      }
  }
 protected:
  Distance::DistanceFunc distance_;
};

// Matching::Matching(/* args */) {}

// Matching::~Matching() {}

}  // namespace xslam
