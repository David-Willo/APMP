/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-16 19:11:05
 * @LastEditors: DavidWillo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
 
#include "xslam/visual-localization/visualization.h"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>



namespace xslam {
using namespace std;

inline cv::Vec3b makeJet3B(float id) {
  if (id <= 0)
    return cv::Vec3b(128, 0, 0);
  if (id >= 1)
    return cv::Vec3b(0, 0, 128);

  int icP = (id * 8);
  float ifP = (id * 8) - icP;

  if (icP == 0)
    return cv::Vec3b(255 * (0.5 + 0.5 * ifP), 0, 0);
  if (icP == 1)
    return cv::Vec3b(255, 255 * (0.5 * ifP), 0);
  if (icP == 2)
    return cv::Vec3b(255, 255 * (0.5 + 0.5 * ifP), 0);
  if (icP == 3)
    return cv::Vec3b(255 * (1 - 0.5 * ifP), 255, 255 * (0.5 * ifP));
  if (icP == 4)
    return cv::Vec3b(255 * (0.5 - 0.5 * ifP), 255, 255 * (0.5 + 0.5 * ifP));
  if (icP == 5)
    return cv::Vec3b(0, 255 * (1 - 0.5 * ifP), 255);
  if (icP == 6)
    return cv::Vec3b(0, 255 * (0.5 - 0.5 * ifP), 255);
  if (icP == 7)
    return cv::Vec3b(0, 0, 255 * (1 - 0.5 * ifP));
  return cv::Vec3b(255, 255, 255);
}

VisualizationVLoc::VisualizationVLoc(
    ros::NodeHandle& nh, const Options& options)
    : nh_(nh), options_(options) {
  stop_ = false;

  // setup publisher
  pub_keyframe_pose_ =
      nh_.advertise<geometry_msgs::PoseArray>("keyframe_poses", 10);
  pub_mappts_ = nh_.advertise<sensor_msgs::PointCloud2>("mappoints", 10);
  pub_odom_lidar_ = nh_.advertise<nav_msgs::Odometry>("/current_odom_lidar", 10);
  pub_odom_cam_ = nh_.advertise<nav_msgs::Odometry>("/current_odom_cam", 10);
  pub_pose_cam_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_pose_cam", 10);
  image_transport_ = make_unique<image_transport::ImageTransport>(nh_);
  publiser_image_ = image_transport_->advertise("frame_info", 1);
  publiser_image_lk_ = image_transport_->advertise("vis_lk", 1);
  publiser_image_track_ = image_transport_->advertise("vis_track", 1);
  publiser_image_match_ = image_transport_->advertise("vis_match", 1);
  publiser_image_reload_ = image_transport_->advertise("vis_reload", 1);
  pub_path_cam_ = nh_.advertise<nav_msgs::Path>("/current_path_cam", 1);
  pub_path_cam_forremap_ = nh_.advertise<nav_msgs::Path>("/current_path_cam1", 1);
  pub_gridcell_ = nh_.advertise<visualization_msgs::MarkerArray>("grid_color_array", 10);
}

void VisualizationVLoc::spin() {
  CHECK_NOTNULL(visual_map_);
  ros::Rate rate(15);
  while (!stop_) {
    publishKeyFrames();

    publishMapPoints();

    publishMapCells();

    rate.sleep();
  }
}

void VisualizationVLoc::publishKeyFrames() {
  auto keyframes = visual_map_->getAllFrames();

  geometry_msgs::PoseArray msg_poses;
  msg_poses.header.frame_id = "map";
  msg_poses.header.stamp = ros::Time::now();

  for (auto&& kf : keyframes) {
    Vector3 trans = kf->getTwb().translation();
    Quaternion rot = kf->getTwb().unit_quaternion();

    geometry_msgs::Pose pose;
    pose.orientation.w = rot.w();
    pose.orientation.x = rot.x();
    pose.orientation.y = rot.y();
    pose.orientation.z = rot.z();
    pose.position.x = trans.x();
    pose.position.y = trans.y();
    pose.position.z = trans.z();
    msg_poses.poses.push_back(pose);
  }
  pub_keyframe_pose_.publish(msg_poses);
}

void VisualizationVLoc::publishMapPoints() {
  auto mappts = visual_map_->getAllMapPoints();

  auto msg_mappoints =
      sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);
  auto pcm = sensor_msgs::PointCloud2Modifier(*msg_mappoints);
  pcm.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcm.resize(mappts.size());

  // iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(*msg_mappoints, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(*msg_mappoints, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(*msg_mappoints, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_r(*msg_mappoints, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_g(*msg_mappoints, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_b(*msg_mappoints, "b");

  for (auto&& mp : mappts) {
    // if (mp->not_valid_)
    //   continue;
    Vector3 pos = mp->position();
    *out_x = pos(0);
    *out_y = pos(1);
    *out_z = pos(2);

    // if (mp->asscociations_.empty()) {
    //   *out_r = 198, *out_g = 39, *out_b = 40;
    // } else {
    //   *out_r = 40, *out_g = 39, *out_b = 198;
    // }

    ++out_x, ++out_y, ++out_z;
    ++out_r, ++out_g, ++out_b;
  }

  msg_mappoints->header.frame_id = "map";
  msg_mappoints->header.stamp = ros::Time::now();
  pub_mappts_.publish(msg_mappoints);
}

void VisualizationVLoc::publishTransform(const VisualFrame::Ptr& frame, bool is_stable) {
  static tf2_ros::TransformBroadcaster br;

  // unique_lock<std::mutex> lock(mutex_pose_);
  // for (auto&& info : tf_records_)
  SE3 pose = frame->getTwb();
  Vector3 t_w_c = pose.translation();
  Quaternion rot_w_c = pose.unit_quaternion();
  {
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "camera";

    transformStamped.transform.translation.x = t_w_c.x();
    transformStamped.transform.translation.y = t_w_c.y();
    transformStamped.transform.translation.z = t_w_c.z();

    transformStamped.transform.rotation.x = rot_w_c.x();
    transformStamped.transform.rotation.y = rot_w_c.y();
    transformStamped.transform.rotation.z = rot_w_c.z();
    transformStamped.transform.rotation.w = rot_w_c.w();
    br.sendTransform(transformStamped);
  }

  if (is_stable)
  {
    path_cam_msg_.header.stamp = ros::Time::now();
    path_cam_msg_.header.frame_id = "map";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "world"; // Replace with your desired frame ID

    pose_stamped.pose.position.x = t_w_c.x();
    pose_stamped.pose.position.y = t_w_c.y();
    pose_stamped.pose.position.z = t_w_c.z();

    path_cam_msg_.poses.push_back(pose_stamped);
    pub_path_cam_.publish(path_cam_msg_);
    pub_path_cam_forremap_.publish(path_cam_msg_);
  }

  {
    nav_msgs::Odometry odom_cam;
    odom_cam.header.stamp = ros::Time::now();
    odom_cam.header.frame_id = "map";

    odom_cam.pose.pose.position.x = t_w_c.x();
    odom_cam.pose.pose.position.y = t_w_c.y();
    odom_cam.pose.pose.position.z = t_w_c.z();

    odom_cam.pose.pose.orientation.x = rot_w_c.x();
    odom_cam.pose.pose.orientation.y = rot_w_c.y();
    odom_cam.pose.pose.orientation.z = rot_w_c.z();
    odom_cam.pose.pose.orientation.w = rot_w_c.w();

    pub_odom_cam_.publish(odom_cam);
    
    // also publish the posestamped msg for fov control 
    geometry_msgs::PoseStamped pose_cam;
    pose_cam.header.frame_id = "map";
    pose_cam.header.stamp = odom_cam.header.stamp;
    pose_cam.pose = odom_cam.pose.pose;
    pub_pose_cam_.publish(pose_cam);
  }

  // {
  //   SE3 pose = frame->getTwb();
  //   geometry_msgs::TransformStamped transformStamped;

  //   transformStamped.header.stamp = ros::Time::now();
  //   transformStamped.header.frame_id = "map";
  //   transformStamped.child_frame_id = "imu";

  //   Vector3 t_w_c = pose.translation();
  //   Quaternion rot_w_c = pose.unit_quaternion();
  //   transformStamped.transform.translation.x = t_w_c.x();
  //   transformStamped.transform.translation.y = t_w_c.y();
  //   transformStamped.transform.translation.z = t_w_c.z();

  //   transformStamped.transform.rotation.x = rot_w_c.x();
  //   transformStamped.transform.rotation.y = rot_w_c.y();
  //   transformStamped.transform.rotation.z = rot_w_c.z();
  //   transformStamped.transform.rotation.w = rot_w_c.w();
  //   br.sendTransform(transformStamped);
  // }

  // {
  //   // publish lidar frame odometry
  //   geometry_msgs::TransformStamped transformStamped;

  //   transformStamped.header.stamp = ros::Time::now();
  //   transformStamped.header.frame_id = "map";
  //   transformStamped.child_frame_id = "current_lidar";
  //   // TODO(DW) to config
  //   Vector3 t_c_l(-0.07105, -0.344159, -0.0341766);
  //   Quaternion rot_c_l;
  //   Matrix3 R_c_l;
  //   R_c_l << 0.0171327,    -0.99982, -0.00809888,
  //          0.0685358, 0.00925536,  -0.997606,
  //          0.997502,  0.0165366,   0.068682;
  //   rot_c_l = R_c_l;

  //   Vector3 t_w_l = rot_w_c * t_c_l.transpose() + t_w_c;
  //   Quaternion rot_w_l = rot_w_c * rot_c_l;

  //   //  maybe to robot rare?
  //   transformStamped.transform.translation.x = t_w_l.x();
  //   transformStamped.transform.translation.y = t_w_l.y();
  //   transformStamped.transform.translation.z = t_w_l.z();

  //   transformStamped.transform.rotation.x = rot_w_l.x();
  //   transformStamped.transform.rotation.y = rot_w_l.y();
  //   transformStamped.transform.rotation.z = rot_w_l.z();
  //   transformStamped.transform.rotation.w = rot_w_l.w();
  //   br.sendTransform(transformStamped);

  //   nav_msgs::Odometry odom_lidar;
  //   odom_lidar.header.stamp = ros::Time::now();
  //   odom_lidar.header.frame_id = "map";

  //   odom_lidar.pose.pose.position.x = t_w_l.x();
  //   odom_lidar.pose.pose.position.y = t_w_l.y();
  //   odom_lidar.pose.pose.position.z = t_w_l.z();

  //   odom_lidar.pose.pose.orientation.x = rot_w_l.x();
  //   odom_lidar.pose.pose.orientation.y = rot_w_l.y();
  //   odom_lidar.pose.pose.orientation.z = rot_w_l.z();
  //   odom_lidar.pose.pose.orientation.w = rot_w_l.w();

  //   pub_odom_lidar_.publish(odom_lidar);
  // }
}


void VisualizationVLoc::publishMapCells() {
  // INFOLOG("Publish marker array has {} cells", gridColorArray_.markers.size());
  pub_gridcell_.publish(gridColorArray_);
}

void VisualizationVLoc::visualizeFrameInfo(const VisualFrame::Ptr& frame) {
  auto&& image = frame->image();
  CHECK(!image.empty());

  cv::Mat image_viz;
  if (image.channels() == 1) {
    cv::Mat img_tmp;
    cv::cvtColor(image, img_tmp, cv::COLOR_GRAY2BGR);
    cv::resize(
        img_tmp, image_viz,
        cv::Size(
            image.cols / options_.image_scale(),
            image.rows / options_.image_scale()));
  } else {
    cv::resize(
        image, image_viz,
        cv::Size(
            image.cols / options_.image_scale(),
            image.rows / options_.image_scale()));
  }

  vector<pair<double, size_t>> depth_indices;
  for (size_t i = 0; i < frame->num_keypoints(); i++) {
    auto&& mp = frame->mappoint(i);
    auto&& kp = frame->keypoint(i);
    if (!mp) {
      Vector2 uv = kp.uv / options_.image_scale();
      auto pt1 = cv::Point2f(
          uv.x() - options_.image_point_size(),
          uv.y() - options_.image_point_size());
      auto pt2 = cv::Point2f(
          uv.x() + options_.image_point_size(),
          uv.y() + options_.image_point_size());
      cv::rectangle(
          image_viz, cv::Rect(pt1, pt2), cv::Vec3b(255, 255, 255), -1);
    } else {
      Vector3 ptc = frame->getTbw() * mp->position();
      depth_indices.push_back(make_pair(ptc.z(), i));
    }
  }

  sort(depth_indices.begin(), depth_indices.end());

  for (size_t i = 0; i < depth_indices.size(); i++) {
    auto&& feat = frame->keypoint(depth_indices[i].second);
    Vector2 uv = feat.uv / options_.image_scale();
    // uvs_mp.emplace_back(feat.uv.x(), feat.uv.y());
    auto pt1 = cv::Point2f(
        uv.x() - options_.image_point_size(),
        uv.y() - options_.image_point_size());
    auto pt2 = cv::Point2f(
        uv.x() + options_.image_point_size(),
        uv.y() + options_.image_point_size());
    cv::rectangle(
        image_viz, cv::Rect(pt1, pt2),
        makeJet3B(i * 0.85f / depth_indices.size()),
        // cv::Vec3b(10, 10, 10),
        -1);
  }

  message_image_ =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_viz).toImageMsg();
  message_image_->header.stamp = ros::Time::now();
  publiser_image_.publish(message_image_);
  message_image_ =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_vis::vis_lk).toImageMsg();
  publiser_image_lk_.publish(message_image_);
  message_image_ =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_vis::vis_track).toImageMsg();
  publiser_image_track_.publish(message_image_);
  message_image_ =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_vis::vis_match).toImageMsg();
  publiser_image_match_.publish(message_image_);
  message_image_ =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_vis::vis_reload).toImageMsg();
  publiser_image_reload_.publish(message_image_);
}




}  // namespace xslam