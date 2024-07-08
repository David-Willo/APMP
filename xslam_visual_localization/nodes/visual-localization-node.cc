/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-27 06:10:56
 * @LastEditTime: 2023-08-01 08:46:59
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2023 by davidwillo@foxmail.com, All Rights Reserved. 
 */


#include <ros/ros.h>
#include "xslam/common/logging.h"
#include "xslam/visual-localization/visual-localization.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_odom");
  ros::NodeHandle nh, nh_private("~");

  google::InitGoogleLogging(argv[0]);
  // google::SetStderrLogging(google::INFO);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  // FLAGS_minloglevel = 0;
  
  ros::AsyncSpinner spinner(3);

	xslam::VisualLocalization::Options options;
  // seems useless
	options.topic_name = "/stereo/frame_left/image_raw/compressed";
  xslam::VisualLocalization pipeline(nh, options);

  // pipeline.spin();
  // single thread
	// ros::spin();
  // multithread async for both imu and visual
  spinner.start();

  ros::waitForShutdown();

  // user actions on exit
  pipeline.finish();

  return 0;
}
