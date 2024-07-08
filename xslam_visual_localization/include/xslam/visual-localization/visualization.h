/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-09-26 14:40:52
 * @LastEditTime: 2024-07-02 07:09:05
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "xslam/common/logging.h"
#include "xslam/common/arg.h"
#include "xslam/visual-types/visual-map.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include "xslam/visual-localization/debug_visualize.h"

namespace xslam {

class VisualizationVLoc {
 public:
  using Ptr = std::shared_ptr<VisualizationVLoc>;

  using ConstPtr = std::shared_ptr<const VisualizationVLoc>;

  struct Options {
    ADD_ARG(bool, image_publish) = true;
    ADD_ARG(float, image_scale) = 1.0f;
    ADD_ARG(float, image_point_size) = 1.0f;
  };

 public:
  // VisualizationVLoc(/* args */) = default;
  VisualizationVLoc(ros::NodeHandle& nh, const Options& options);
  virtual ~VisualizationVLoc() = default;

  void spin();

  void setVisualMap(const VisualMap::Ptr& map) {
    visual_map_ = map;
  }

  void setGridColorArray(const visualization_msgs::MarkerArray& gridColorArray) {
    gridColorArray_ = gridColorArray;
    // INFOLOG("After set marker array has {} cells", gridColorArray_.markers.size());
  }

  void publishKeyFrames();

  void publishMapPoints();

  void publishMapCells();

  void publishTransform(const VisualFrame::Ptr& frame, bool is_stable=false);

  void visualizeFrameInfo(const VisualFrame::Ptr& frame);

  bool stop() {
    stop_ = true;
    return stop_;
  }

 protected:
  Options options_;

  std::atomic_bool stop_, finished_;
  VisualMap::Ptr visual_map_ = nullptr;

  ros::NodeHandle nh_;

  std::unique_ptr<image_transport::ImageTransport> image_transport_ = nullptr;

  ros::Publisher pub_keyframe_pose_, pub_mappts_, pub_odom_lidar_, pub_pose_cam_, pub_odom_cam_, pub_path_cam_, pub_path_cam_forremap_;
  
  ros::Publisher pub_gridcell_;
  image_transport::Publisher publiser_image_;
  // dw:these four are for debug only
  image_transport::Publisher publiser_image_lk_, publiser_image_track_, publiser_image_match_, publiser_image_reload_;
  sensor_msgs::ImagePtr message_image_;
  nav_msgs::Path path_cam_msg_;
  
  visualization_msgs::MarkerArray gridColorArray_;
  /* data */
};

}  // namespace xslam
