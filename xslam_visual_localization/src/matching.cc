/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-16 17:03:14
 * @LastEditors: DavidWillo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-localization/matching.h"
#include "xslam/visual-localization/debug_visualize.h"

#include <faiss/Index.h>
#include <faiss/IndexFlat.h>
#include <faiss/IndexIVFFlat.h>
#include <faiss/IndexIVFPQ.h>
#include <faiss/IndexPQ.h>
#include <faiss/MetricType.h>

#include "xslam/common/logging.h"

#define DEBUG_LK 1

namespace xslam {
using namespace std;

class MinimalWeightBipartiteMatching {
 public:
  MinimalWeightBipartiteMatching(int n, int m) : n_(n), m_(m), graph_(n + 1) {}

  void addEdge(int u, int v, int weight) {
    if (edgeSet_.count({u, v}) == 0) {
      graph_[u].emplace_back(v, weight);
      edgeSet_.insert({u, v});
    }
  }

  int findMatching(std::unordered_map<int, int>& matching) {
    std::vector<int> match(n_ + 1, NIL);
    std::vector<int> dist(n_ + 1);
    int matchingCount = 0;

    while (bfs(match, dist)) {
      for (int u = 1; u <= n_; ++u) {
        if (match[u] == NIL && dfs(match, dist, u)) {
          matching[u] = match[u];
          matchingCount++;
        }
      }
    }

    return matchingCount;
  }

  int getEdgeWeight(int u, int v) {
    for (const auto& edge : graph_[u]) {
      if (edge.first == v) {
        return edge.second;
      }
    }
    return INF;
  }

 private:
  const int INF = std::numeric_limits<int>::max();
  const int NIL = 0;

  bool bfs(std::vector<int>& match, std::vector<int>& dist) {
    std::queue<int> q;

    for (int u = 1; u <= n_; ++u) {
      if (match[u] == NIL) {
        dist[u] = 0;
        q.push(u);
      } else {
        dist[u] = INF;
      }
    }
    dist[NIL] = INF;

    while (!q.empty()) {
      int u = q.front();
      q.pop();

      if (u != NIL) {
        for (const auto& edge : graph_[u]) {
          int v = edge.first;
          if (dist[match[v]] == INF) {
            dist[match[v]] = dist[u] + 1;
            q.push(match[v]);
          }
        }
      }
    }

    return (dist[NIL] != INF);
  }

  bool dfs(std::vector<int>& match, std::vector<int>& dist, int u) {
    if (u == NIL) return true;

    for (const auto& edge : graph_[u]) {
      int v = edge.first;
      int weight = edge.second;

      if (dist[match[v]] == dist[u] + 1 && dfs(match, dist, match[v])) {
        match[u] = v;
        match[v] = u;
        return true;
      }
    }

    dist[u] = INF;
    return false;
  }

  int n_;
  int m_;
  std::vector<std::vector<std::pair<int, int>>> graph_;
  std::set<std::pair<int, int>> edgeSet_;
};

// #define DEBUG_MATCH 1
// TODO: how to be genric in scalar
// TODO: scale prediction
void Matching::matchAgainstMapPoints(
    VisualFrame::Ptr frame, const std::vector<MapPoint::Ptr>& mappoints,
    const std::vector<ProjectionStatus>& projection_status,
    const Options& options) {
  CHECK_EQ(mappoints.size(), projection_status.size());

  int num_matches = 0;
  auto dist_func = Distance::createDistanceFunc<float>(options.distance_type());

  // size_t ori_pts = frame->countTrackedMapPoints();
#ifdef DEBUG_MATCH
  size_t block_visibility = 0, block_coor = 0, block_ratio = 0,
         block_threshold = 0, block_exist = 0;
#endif


  for (size_t i = 0; i < mappoints.size(); i++) {
    auto&& mappoint = mappoints[i];
    auto&& proj_stat = projection_status[i];

    // not visible or already tracked from last frame
    if (!proj_stat.is_visible || !mappoint->is_valid()) {
#ifdef DEBUG_MATCH
      block_visibility++;
#endif
      continue;
    }

    const Vector2& uv = proj_stat.uvd.topRows(2);

    // DW: only fetch unmatched candidates, lk flow stable in most case
    // auto&& keypoint_indices =
    //     frame->getUntrackedKeyPointIndicesInGrid(uv,
    //     options.search_radius());

    // DW: if can match from mappoint. try overwrite the mappoint result, may be unstable
    std::pair<float, float> depth_info = std::make_pair(-1.0,-1.0);
    // magic code 15
    auto&& keypoint_indices =
        frame->getKeyPointIndicesInGrid(uv, options.search_radius(), 24, &depth_info);

    // WARNLOG("check kp indices size {} ",  keypoint_indices.size());

    if (keypoint_indices.empty()) {
#ifdef DEBUG_MATCH
      block_coor++;
#endif
      continue;
    }

    // float
    std::vector<pair<float, KeyPointId>> dist_ind_pairs;
    dist_ind_pairs.reserve(keypoint_indices.size());
    for (auto&& idx : keypoint_indices) {
      auto desc_dist =
          dist_func(frame->descriptor(idx), mappoint->descriptor());
      // // what about use pixel coordinate only? don't do that, not work
      // auto desc_dist =
      //     dist_func(frame->keypoint(idx).uv.cast<unsigned char>(),
      //     uv.cast<unsigned char>());
      dist_ind_pairs.emplace_back(desc_dist, idx);
    }

    if (dist_ind_pairs.size() >= 2) {
      std::partial_sort(dist_ind_pairs.begin(), dist_ind_pairs.begin() + 2,
                        dist_ind_pairs.end());
      if (options.nn_test() &&
          dist_ind_pairs[0].first >
              dist_ind_pairs[1].first * options.nn_ratio()) {
#ifdef DEBUG_MATCH
        block_ratio++;
#endif
        continue;
      }
    }

    // DW: actually we set this threshold very loose
    if (dist_ind_pairs.front().first > options.matching_threshold()) {
#ifdef DEBUG_MATCH
      // INFOLOG("check descriptor dist {} {}", dist_ind_pairs.front().first, dist_ind_pairs.back().first );
      block_threshold++;
#endif
      continue;
    }

    if (frame->mappoint(dist_ind_pairs.front().second) != nullptr) {
      // block_exist++;
      // continue;
      
      // already exist candidate, if more similar try overwrite?
      // attention: since we overwrite the mappoint descriptor in lk, 
      // those mapoints are always with 0 desc_dist_ori, so here only replaced 
      // bad mapoints tracked in local map
      auto desc_dist_ori = dist_func(
          frame->descriptor(dist_ind_pairs.front().second),
          frame->mappoint(dist_ind_pairs.front().second)->descriptor());

      if (dist_ind_pairs.front().first > desc_dist_ori * options.nn_ratio()) {
#ifdef DEBUG_MATCH
        block_exist++;
#endif
        continue;
      }
     
      // // also compare the depth? not sure a good idea
      // auto depth_ori = (frame->getTbw()*frame->mappoint(dist_ind_pairs.front().second)->position()).z();
      // auto depth_new = (frame->getTbw()*mappoint->position()).z();
      // if (depth_ori * options.nn_ratio() < depth_new) {
      //   block_exist++;
      //   continue;
      // }
    }

    // approximate z buffer
    // WARNLOG("statistics {}+={} ", depth_info.first, depth_info.second);
    if (depth_info.first > 0) {
      // has depth statistics
      auto depth_new = (frame->getTbw()*mappoint->position()).z();
      if (std::fabs(depth_new - depth_info.first) > 2*depth_info.second) {
        // larger than 2*sigma, could be outliers
        // ERRLOG("BLOCK! {}+={} vs {}", depth_info.first, depth_info.second, depth_new);
        continue;
      }
    }

    frame->set_mappoint(dist_ind_pairs.front().second, mappoint);
    num_matches++;
  }
#ifdef USE_BipartiteMatching
  MinimalWeightBipartiteMatching solver(mappoints.size(),
                                        frame->num_keypoints());
  for (size_t i = 0; i < mappoints.size(); i++) {
    auto&& mappoint = mappoints[i];
    auto&& proj_stat = projection_status[i];

    if (!proj_stat.is_visible || !mappoint->is_valid()) {
      block_visibility++;
      continue;
    }

    const Vector2& uv = proj_stat.uvd.topRows(2);
    auto&& keypoint_indices =
        frame->getUntrackedKeyPointIndicesInGrid(uv, options.search_radius());

    std::vector<pair<float, KeyPointId>> dist_ind_pairs;
    dist_ind_pairs.reserve(keypoint_indices.size());
    for (auto&& idx : keypoint_indices) {
      auto desc_dist =
          dist_func(frame->descriptor(idx), mappoint->descriptor());
      dist_ind_pairs.emplace_back(desc_dist, idx);
    }
    size_t upper_bound = 2;
    if (dist_ind_pairs.size() >= 2) {
      std::partial_sort(dist_ind_pairs.begin(), dist_ind_pairs.begin() + 2,
                        dist_ind_pairs.end());
    } else {
      upper_bound = dist_ind_pairs.size();
    }

    for (size_t ii = 0; ii < upper_bound; ii++) {
      solver.addEdge(i, dist_ind_pairs[ii].second, dist_ind_pairs[ii].first);
    }

    // for (auto&& idx :keypoint_indices) {
    //   solver.addEdge(i, idx, dist_func(frame->descriptor(idx),
    //   mappoint->descriptor()));
    // }
  }
  std::unordered_map<int, int> matching;
  int matchingCount = solver.findMatching(matching);
  WARNLOG("km matching {}", matchingCount);
  for (const auto& entry : matching) {
    auto&& mappoint = mappoints[entry.first];
    if (frame->mappoint(entry.second) == nullptr) {
      frame->set_mappoint(entry.second, mappoint);
    }
  }
  INFOLOG("before {} after {}", ori_pts, frame->countTrackedMapPoints());
#endif

#ifdef DEBUG_MATCH
  INFOLOG(
      "block by visibility {} block corr {} block by nn {} block by threhold "
      "{} block by exist {}",
      block_visibility, block_coor, block_ratio, block_threshold, block_exist);
#endif
  // INFOLOG("before {} after {}", ori_pts, frame->countTrackedMapPoints());
  INFOLOG("#matched from map: {}", num_matches);
}

Matching::Result Matching::matchAgainstFrame(
    VisualFrame::Ptr frame, const VisualFrame::ConstPtr& frame_ref,
    const Options& options) {
  Matching::Result result;
  int ncan = 0, ncan2 = 0, ncan3 = 0;
  auto dist_func = Distance::createDistanceFunc<float>(options.distance_type());
  for (size_t i = 0; i < frame_ref->num_keypoints(); i++) {
    auto mappoint = frame_ref->mappoint(i);
    if (!mappoint || !mappoint->is_valid()) {
      continue;
    }
    ncan++;

    Vector2 uv;
    if (!frame->project3(mappoint->position(), &uv)) {
      continue;
    }
    ncan2++;

    // TODO: octave
    // int nLastOctave = LastFrame.mvKeys[i].octave;

    // Search in a window. Size depends on scale
    // float radius = th;
    // cout << uv.transpose() << endl;

    std::vector<KeyPointId> indices =
        frame->getUntrackedKeyPointIndicesInGrid(uv, options.search_radius());

    if (indices.empty()) {
      continue;
    }
    ncan3++;

    const VectorXb& desc_ref = frame_ref->descriptor(i);

    float best_dist = std::numeric_limits<float>::max();
    int best_idx = -1;

    for (auto&& idx : indices) {
      const VectorXb& desc = frame->descriptor(idx);
      const float dist = dist_func(desc_ref, desc);

      if (dist < best_dist) {
        best_dist = dist;
        best_idx = idx;
      }
    }

    // const float th_match_proj = 0.7f;

    if (best_dist <= options.matching_threshold()) {
      frame->mappoint_mutable(best_idx) = mappoint;
      result.num_match++;
    }
    // cout << ncan << ' ' << ncan2 << ' ' << ncan3 << endl;
  }

  return result;
}

Matching::Result Matching::matchAgainstFrameLK(
    VisualFrame::Ptr frame, const VisualFrame::ConstPtr& frame_ref,
    const Options& options) {
  Matching::Result result;

  auto dist_func = Distance::createDistanceFunc<float>(options.distance_type());

  std::vector<cv::Point2f> keypoints_last, keypoints_cur;
  std::vector<int> keypoints_id;
  const auto& camera = frame_ref->camera();
  for (size_t i = 0; i < frame_ref->num_keypoints(); i++) {
    auto mappoint = frame_ref->mappoint(i);
    if (!mappoint || !mappoint->is_valid()) {
      continue;
    }

    const Vector2& uv = frame_ref->keypoints().at(i).uv;
    keypoints_last.push_back(cv::Point2f(uv.x(), uv.y()));
    keypoints_id.push_back(i);
  }
  if (keypoints_id.empty()) {
    return result;
  }
  // for (size_t i = 0; i < frame->num_keypoints(); i++) {
  //   const Vector2& uv =  frame->keypoints().at(i).uv;
  //   // if (options.untracked()) {
  //   //    auto mappoint = frame->mappoint(i);
  //   //   if (mappoint) {
  //   //     continue;
  //   //   }
  //   // }
  //   keypoints_cur.push_back(cv::Point2f(uv.x(), uv.y()));
  // }
  std::vector<unsigned char> status;
  std::vector<float> error;
  cv::calcOpticalFlowPyrLK(frame_ref->image(), frame->image(), keypoints_last,
                           keypoints_cur, status, error, cv::Size(21, 21), 3);
  reduceVector(keypoints_last, status);
  reduceVector(keypoints_id, status);
  reduceVector(keypoints_cur, status);
  rejectWithFundamentalMat(keypoints_last, keypoints_cur, keypoints_id, camera);
  // INFOLOG("#done lk: {}", keypoints_cur.size());

#ifdef DEBUG_LK

  cv::Mat debuglk_ref = frame_ref->image().clone();
  cv::Mat debuglk = frame->image().clone();
  // std::vector<cv::DMatch> matches;
  for (size_t p_idx = 0; p_idx < keypoints_cur.size(); p_idx++) {
    // matches.push_back(cv::DMatch(p_idx, p_idx, 1));
    // cv::circle(debuglk_ref, keypoints_last[p_idx], 2, cv::Scalar(0, 255, 255),
    //            2);
    cv::line(debuglk, keypoints_last[p_idx], keypoints_cur[p_idx], cv::Scalar(0, 255, 0), 3);
    cv::circle(debuglk, keypoints_cur[p_idx], 2, cv::Scalar(0, 255, 255), 3.5);
  }
  cv::resize(debuglk_ref, debuglk_ref,
             cv::Size(debuglk.cols / 2, debuglk.rows / 2));
  
  cv::resize(debuglk, debuglk, cv::Size(debuglk.cols / 2, debuglk.rows / 2));
  // cv::drawMatches(frame_ref->image(), keypoints_last, frame->image(),
  // keypoints_cur, debuglk, matches);
  // cv::imshow("lk_ref", debuglk_ref);
  // cv::imshow("lk", debuglk);
  // cv::waitKey(2);
  debug_vis::vis_lk = debuglk.clone();

#endif

  for (size_t i = 0; i < keypoints_id.size(); i++) {
    auto mappoint = frame_ref->mappoint(keypoints_id[i]);

    Vector2 uv;
    uv << keypoints_cur[i].x, keypoints_cur[i].y;
    std::vector<KeyPointId> indices =
        frame->getUntrackedKeyPointIndicesInGrid(uv, options.search_radius());
    if (indices.empty()) {
      continue;
    }

    const VectorXb& desc_ref = frame_ref->descriptor(keypoints_id[i]);

    float best_dist = std::numeric_limits<float>::max();
    int best_idx = -1;

    for (auto&& idx : indices) {
      const VectorXb& desc = frame->descriptor(idx);
      const float dist = dist_func(desc_ref, desc);

      if (dist < best_dist) {
        best_dist = dist;
        best_idx = idx;
      }
    }

    // DW: maybe overwrite the mappoint desc? make sure smooth track between frames
    // here we assume optical flow is more precise than prior pose
    // mappoint->set_descriptor(frame->descriptor(best_idx));

    // const float th_match_proj = 0.7f;
    // maybe skip threshold check for optical flow?
    // if (best_dist <= options.matching_threshold()) {
      frame->mappoint_mutable(best_idx) = mappoint;
      result.num_match++;
    // }
  }

  return result;
}

Matching::Result Matching::matchAgainstFrameIVF(
    VisualFrame::Ptr frame, const VisualFrame::ConstPtr& frame_ref,
    const Options& options) {
  //
  Matching::Result result;
  int ncan = 0, ncan2 = 0, ncan3 = 0;
  auto dist_func = Distance::createDistanceFunc<float>(options.distance_type());
  auto index = frame_ref->index();

  int k = 2;
  auto&& [desc_query, keypoint_indices] = frame->descriptors_untracked();
  int num_query = desc_query.cols();
  CHECK_EQ(num_query, keypoint_indices.size());
  using idx_t = faiss::Index::idx_t;

  std::vector<faiss::Index::idx_t> ref_inds(k * num_query);
  std::vector<float> ref_dists(k * num_query);
  index->search(desc_query.cols(), (float*)desc_query.data(), 2,
                ref_dists.data(), ref_inds.data());

  int num_matched = 0;
  for (size_t iq = 0; iq < num_query; iq++) {
    auto&& ind0 = ref_inds[iq * k];
    auto&& ind1 = ref_inds[iq * k + 1];
    auto&& dist0 = ref_dists[iq * k];
    auto&& dist1 = ref_dists[iq * k + 1];

    if ((dist0 < dist1 * options.nn_ratio()) &&
        dist0 < options.matching_threshold()) {
      num_matched++;
      frame->set_mappoint(keypoint_indices[iq], frame_ref->mappoint(ind0));
    }
  }
  result.num_match = num_matched;

  return result;
}

// TODO: wrapper of faiss
Matching::Result Matching::matchAgainstCluster(
    VisualFrame::Ptr frame, const std::vector<VisualFrame::Ptr>& clusters,
    VisualMap::Ptr& visual_map, const Options& options) {
  Matching::Result result;

  // MapPointIdSet inds;
  MatrixXb desc_all;
  std::vector<MatrixXb> descs;
  descs.reserve(clusters.size());
  MapPointList mappoints, mappoints_matched;
  mappoints_matched.resize(frame->num_keypoints(), nullptr);

  int num_desc = 0;
  for (auto&& frame_ref : clusters) {
    // auto&& points = visual_map->getTrackedMapPoints(frame_ref);
    auto&& [desc_ref, mappts_ref] = frame_ref->descriptors_tracked();
    descs.push_back(desc_ref);
    num_desc += desc_ref.cols();

    std::copy(mappts_ref.begin(), mappts_ref.end(), back_inserter(mappoints));
  }

  desc_all.resize(descs.back().rows(), num_desc);
  int start_col = 0;
  for (auto&& desc : descs) {
    desc_all.middleCols(start_col, desc.cols()) = desc;
    start_col += desc.cols();
  }

  CHECK_EQ(num_desc, mappoints.size());

  // TODO: dim
  int dim = 256;
  int nlist = 8;
  faiss::IndexFlatL2 index(dim);
  // faiss::IndexIVFFlat index(&quantizer, dim, nlist);
  // faiss::IndexPQ index(dim, 8, 8);
  // index.train(num_desc, (float*)desc_all.data());
  index.add(num_desc, (float*)desc_all.data());

  int k = 2;
  MatrixXb desc_query = frame->descriptors();
  int num_query = desc_query.cols();
  using idx_t =  faiss::Index::idx_t;

  std::vector<faiss::Index::idx_t> ref_inds(k * num_query);
  std::vector<float> ref_dists(k * num_query);
  index.search(desc_query.cols(), (float*)desc_query.data(), 2,
               ref_dists.data(), ref_inds.data());

  int num_matched = 0;
  for (size_t iq = 0; iq < num_query; iq++) {
    auto&& ind0 = ref_inds[iq * k];
    auto&& ind1 = ref_inds[iq * k + 1];
    auto&& dist0 = ref_dists[iq * k];
    auto&& dist1 = ref_dists[iq * k + 1];

    auto&& mappt_ind0 = mappoints[ind0];
    auto&& mappt_ind1 = mappoints[ind1];

    if ((dist0 < dist1 * options.nn_ratio()) || (mappt_ind0 == mappt_ind1)) {
      num_matched++;
      mappoints_matched[iq] = mappt_ind0;
    } else {
      mappoints_matched[iq] = nullptr;
    }
  }

  frame->set_mappoints(mappoints_matched);
  // auto&& frame_ref = clusters[0];
  // frame->setTbw(frame_ref->getTbw());

  INFOLOG("#keypoitns {}, #matched: {}", frame->num_keypoints(), num_matched);
  result.num_match = num_matched;

  return result;
}

}  // namespace xslam