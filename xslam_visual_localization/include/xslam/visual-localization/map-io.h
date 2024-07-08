/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:08:43
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include <aslam/cameras/camera.h>
#include <string>

#include "xslam/common/arg.h"
#include "xslam/visual-types/visual-map.h"

namespace xslam {

struct MapLoadingOptions {
  std::string model_path;
  std::string local_feature_path;
  std::string global_feature_path;
};

// loading visual map from colmap and hloc
class MapIO {
 public:
  using colmap_camera_t = uint32_t;
  using colmap_image_t = uint32_t;
  using colmap_keypoint_t = uint32_t;
  using colmap_point3d_t = uint64_t;
  static const colmap_point3d_t kInvalidPointId =
      std::numeric_limits<colmap_point3d_t>::max();

  using FrameIdMap = std::unordered_map<colmap_image_t, FrameId>;
  using FrameMapPointsMap =
      std::unordered_map<FrameId, std::vector<colmap_point3d_t>>;
  using MapPointIdMap = std::unordered_map<colmap_point3d_t, MapPointId>;
  using MapPointTrackMap = std::unordered_map<
      MapPointId, std::vector<std::pair<colmap_image_t, colmap_keypoint_t>>>;

  struct Options {
    ADD_ARG(std::string, reconstruction_path);
    ADD_ARG(std::string, global_feature_path);
    ADD_ARG(std::string, local_feature_path);

    ADD_ARG(int, grid_size_row) = 16;
    ADD_ARG(int, grid_size_col) = 16;
  };

 public:
  static VisualMap::Ptr loadVisualMap(const Options& options);

 protected:
  static aslam::Camera::Ptr loadCamera(const std::string& path);

  static void loadLocalFeatures(
      const std::string& feature_path, std::vector<VisualFrame::Ptr>& frames);
  static void loadGlobalFeatures(
      const std::string& feature_path, std::vector<VisualFrame::Ptr>& frames);

  static void loadMapPoints(
      const std::string& path, std::vector<MapPoint::Ptr>& mappoints,
      MapPointIdMap& id_map, MapPointTrackMap& track_map);

  static void loadFrames(
      const std::string& path, aslam::Camera::Ptr& camera,
      std::vector<VisualFrame::Ptr>& frames, FrameIdMap& id_map,
      FrameMapPointsMap& frame_points_map);

  //   void

  //  private:
  /* data */
};

}  // namespace xslam
