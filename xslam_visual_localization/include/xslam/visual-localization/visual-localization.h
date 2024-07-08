/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-09-26 14:40:52
 * @LastEditTime: 2024-07-02 07:09:00
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "xslam/common/common.h"
#include "xslam/visual-features/global-feature.h"
#include "xslam/visual-features/local-feature.h"
#include "xslam/visual-localization/relocalization.h"
#include "xslam/visual-localization/visualization.h"
#include "xslam/visual-localization/tracking.h"
#include "xslam/visual-localization/geo_observer.h"
#include "xslam/visual-types/visual-map.h"
#include <boost/circular_buffer.hpp>
namespace xslam {

class VisualLocalization {
 public:
  using Ptr = std::shared_ptr<VisualLocalization>;

  using ConstPtr = std::shared_ptr<const VisualLocalization>;

  struct Options {
    std::string topic_name;
    bool use_imu;
  };

 public:
  VisualLocalization(ros::NodeHandle& nh, const Options& options);

  virtual ~VisualLocalization();

  void localize(const cv::Mat& image, const ros::Time& timestamp);

  // std::vector<VisualFrame::Ptr> getClusterCovisible(
  //     const VisualFrame::ConstPtr& visual_frame);

  void imageCallBack(const sensor_msgs::ImageConstPtr& msg);

  void imuCallBack(const sensor_msgs::ImuConstPtr& msg);

  void compressedImageCallBack(const sensor_msgs::CompressedImageConstPtr& msg);

  VisualFrame::Ptr createFrameFromImageInput(
      const cv::Mat& image, bool b_extract_global = false,
      bool b_store_image = false);

  void saveTrajectory(const std::string& traj_file);

  void finish();

 protected:
  Options options_;

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_image_;
  ros::Subscriber subscriber_imu_;
  // image_transport::Subscriber subscriber_image_;
  // std::unique_ptr<image_transport::ImageTransport> image_transport_ =
  // nullptr;

  std::vector<VisualFrame::Ptr> frames_;

  VisualizationVLoc::Ptr visualization_ = nullptr;
  std::unique_ptr<std::thread> thread_visualization_ = nullptr;

  Tracking::Ptr tracking_ = nullptr;
  GeoObserver::Ptr geo_observer_=nullptr;

  aslam::Camera::Ptr camera_ = nullptr;

  VisualFrame::Ptr frame_last_ = nullptr;
  SE3 delta_pose_;

  LocalFeature::Ptr local_feature_extraction_ = nullptr;
  GlobalFeature::Ptr global_feature_extraction_ = nullptr;

  VisualMap::Ptr map_ = nullptr;

  std::atomic_bool is_inited_;
  std::atomic_bool is_relocalized_;
  Relocalization::Ptr relocalization_ = nullptr;

  size_t track_counter = 0;
  // size_t current_track_counter = 0;
  size_t reloc_counter = 0;

  // for relocalization experiment;
  double average_recover_distance=0;

};



}  // namespace xslam
