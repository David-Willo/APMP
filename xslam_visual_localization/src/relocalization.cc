/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:10:19
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-localization/relocalization.h"

#include <opencv2/core/eigen.hpp>

#include "aslam/cameras/camera-pinhole.h"
#include "aslam/cameras/distortion-radtan.h"
#include "xslam/common/cv_helper.h"
#include "xslam/common/logging.h"
#include "xslam/visual-localization/matching.h"

namespace xslam {

using namespace std;

Relocalization::Relocalization(const Options& options) : options_(options) {
  // for (auto&& kf : keyframes) {
  // 	/* code */
  // }
  distance_func = Distance::createDistanceFunc<float>(options_.distance_type());
}

// TODO: determine success or not
Relocalization::Result Relocalization::relocalize(VisualFrame::Ptr frame) {
  CHECK_NOTNULL(visual_map_);
  CHECK(distance_func);

  Relocalization::Result result;
  result.success = false;

  auto&& candidate_frames = retrieveKeyframeTopK(
      frame->global_descriptor(), options_.num_k_retrieval());

  auto&& covisible_clusters = clusterCandidatesByCovisibility(candidate_frames);

  for (size_t i = 0; i < covisible_clusters.size(); i++) {
    auto&& cluster = covisible_clusters[i];
    frame->reset_mappoints();
    auto matching_result = Matching::matchAgainstCluster(
        frame, cluster, visual_map_, Matching::Options());
    // maybe the frame it self not have too much features hack for kitti
    if (matching_result.num_match >= options_.num_match_threshold() || matching_result.num_match >= frame->num_keypoints()*0.8) {
      result.reference_frame = cluster.front();
      frame->setTbw(cluster.front()->getTbw());
      auto pnp_result = solvePnPCVRansac(frame);
      if (pnp_result.success) {
        result.success = true;
        result.pnp_inliers = pnp_result.num_inliers;
        break;
      }
    }
  }

  return result;
}

std::vector<std::vector<VisualFrame::Ptr>>
Relocalization::clusterCandidatesByCovisibility(
    const std::vector<VisualFrame::Ptr>& candidates) {
  //
  std::vector<std::vector<VisualFrame::Ptr>> clusters;

  unordered_set<VisualFrame::Ptr> frames_set(
      candidates.begin(), candidates.end());
  unordered_set<VisualFrame::Ptr> frames_visited;

  for (auto&& frame : candidates) {
    if (frames_visited.count(frame)) {
      continue;
    }

    std::vector<VisualFrame::Ptr> cluster;
    // auto& curr_cluster = clusters.back();
    queue<VisualFrame::Ptr> frames_queue({frame});

    // LOG(INFO) << "queue: " << kfs_to_check.size();

    while (!frames_queue.empty()) {
      // front
      auto&& frame_to_check = frames_queue.front();
      frames_queue.pop();

      if (frames_visited.count(frame_to_check)) {
        continue;
      }
      frames_visited.insert(frame_to_check);
      cluster.push_back(frame_to_check);

      // visual_map_->get
      const auto&& visible_frames = visual_map_->getConnectedFramesByWeight(
          frame_to_check, options_.weight_covisible());
      // frame_to_check->frame_connections();

      // vector<KeyFrame *> cluster_kf;
      // cout << vis
      // cluster_kf.insert(kf);
      int count = 0;
      for (auto&& frame_next : frames_set) {
        if (!frames_visited.count(frame_next) &&
            (find(visible_frames.begin(), visible_frames.end(), frame_next) !=
             visible_frames.end())) {
          frames_queue.push(frame_next);
          count++;
        }
      }

      // INFOLOG("try intersection: {}", count);

      // std::set_intersection(set_kfs.begin(), set_kfs.end(),
      // visible_kfs.begin(),
      //                       visible_kfs.end(), cluster_kf.begin());

      // INFOLOG("#visible: {}", visible_kfs.size());
      // LOG(INFO) << fmt::format("#itersection: {}", count);

      // for (auto &&kf_nxt : cluster_kf) {
      //   if (!frames_visited.count(kf_nxt)) {
      //     kfs_to_check.push(kf_nxt);
      //   }
      // }
    }
    clusters.push_back(cluster);
  }

  return clusters;
}

std::vector<VisualFrame::Ptr> Relocalization::retrieveKeyframeTopK(
    const VectorXb& descriptor, size_t k) {
  std::vector<VisualFrame::Ptr> candidates;

  auto keyframes = visual_map_->getAllFrames();
  std::vector<pair<float, VisualFrame::Ptr>> keyframes_weight;
  keyframes_weight.reserve(keyframes.size());
  for_each(
      keyframes.begin(), keyframes.end(), [&](const VisualFrame::Ptr& frame) {
        // keyframes_weight.push_back();
        float dist = distance_func(frame->global_descriptor(), descriptor);
        keyframes_weight.emplace_back(dist, frame);
      });

  sort(keyframes_weight.begin(), keyframes_weight.end());
  for (size_t i = 0; i < min(k, keyframes.size()); i++) {
    candidates.push_back(keyframes_weight[i].second);
  }

  // auto keyframes = visual_map_
  return candidates;
}

Relocalization::PnPResult Relocalization::solvePnPCVRansac(
    VisualFrame::Ptr& frame) {
  // OptimzationInfo res;
  Relocalization::PnPResult result;

  cv::Mat tvec, rvec;
  cv::Mat k, d;
  auto camera = frame->camera();
  CHECK_NOTNULL(camera);
  {
    Matrix3d k_eigen =
        static_pointer_cast<aslam::PinholeCamera>(camera)->getCameraMatrix();
    cv::eigen2cv(k_eigen, k);

    Eigen::VectorXd d_eigen = camera->getDistortion().getParameters();
    cv::eigen2cv(d_eigen, d);
  }
  // INFOLOG("?");

  vector<cv::Point3f> points3d;
  vector<cv::Point2f> uvs;
  vector<size_t> inds;
  {
    for (size_t i = 0; i < frame->num_keypoints(); i++) {
      auto&& mappoint = frame->mappoint(i);

      if (mappoint && mappoint->is_valid()) {
        Vector2 uv = frame->keypoint(i).uv;
        // features_[i].uv;
        Vector3 pos = mappoint->position();
        points3d.emplace_back(pos.x(), pos.y(), pos.z());
        uvs.emplace_back(uv.x(), uv.y());

        inds.push_back(i);
      }
    }
  }
  // INFOLOG("??");

  cv::Mat inliers;
  auto success = cv::solvePnPRansac(
      points3d, uvs, k, d, rvec, tvec, options_.pnp_use_initial_pose(),
      options_.pnp_max_iteration(), options_.pnp_projection_error(),
      options_.pnp_conf_thresh(), inliers, cv::SOLVEPNP_ITERATIVE);
  // INFOLOG("??");

  if (!success) {
    result.num_inliers = 0;
    result.success = false;

    return result;
  }

  auto num_inliers = inliers.rows;
  vector<cv::Point3f> mappoint_inliers;
  vector<cv::Point2f> uvs_inliers;
  std::vector<int> ind_inliers;
  mappoint_inliers.reserve(num_inliers);
  uvs_inliers.reserve(num_inliers);
  ind_inliers.reserve(num_inliers);

  for (int i = 0; i < num_inliers; i++) {
    auto&& idx = inliers.at<int>(i, 0);
    mappoint_inliers.push_back(points3d[idx]);
    uvs_inliers.push_back(uvs[idx]);
    ind_inliers.push_back(inds[idx]);
  }

  unordered_set<int> set_inds(ind_inliers.begin(), ind_inliers.end());
  for (int i = 0; i < frame->num_keypoints(); i++) {
    if (!set_inds.count(i)) {
      frame->set_mappoint(i, nullptr);
      // frame->is_outlier_[i] = true;
    } else {
      // frame->is_outlier_[i] = false;
    }
  }

  INFOLOG("#candidates: {}, #inliers: {}", points3d.size(), num_inliers);

  // mappts = mappoint_inliers;
  // uvs = uvs_inliers;
  result.num_inliers = num_inliers;
  auto success_refinement = cv::solvePnP(
      mappoint_inliers, uvs_inliers, k, d, rvec, tvec, true,
      cv::SOLVEPNP_ITERATIVE);
  // auto success = cv::solvePnP(mappts, uvs, k, d, rvec, tvec, true,
  //                             cv::SOLVEPNP_ITERATIVE);

  // accumulate(inli)
  if (!success_refinement) {
    result.success = false;

    return result;
  }

  result.success = true;
  SE3 pose = CVHelper::convertToSE3(tvec, rvec);
  frame->setTbw(pose);

  return result;
}

}  // namespace xslam
