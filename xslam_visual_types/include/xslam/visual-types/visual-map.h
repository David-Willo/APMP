/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:22:21
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */


#pragma once

#include "mappoint.h"
#include "types.h"
#include "visual-frame.h"
#include "xslam/common/distances.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

namespace xslam {

class VisualMap {
 public:
  using Ptr = std::shared_ptr<VisualMap>;

  using ConstPtr = std::shared_ptr<const VisualMap>;

  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> PointCloudType;

 public:
  VisualMap(/* args */) = default;

  ~VisualMap() = default;

  void addMapPoint(MapPoint::Ptr mappt);

  void addKeyFrame(VisualFrame::Ptr frame);

  void removeMapPoint(MapPoint::Ptr mappt);

  void removeKeyFrame(VisualFrame::Ptr frame);

  const MapPoint::ConstPtr getMapPoint(const MapPointId& id) const {
    auto iter = mappoints_.find(id);
    if (iter != mappoints_.end()) {
      return iter->second;
    }
    return nullptr;
  }
  MapPoint::Ptr getMapPoint(const MapPointId& id) {
    auto iter = mappoints_.find(id);
    if (iter != mappoints_.end()) {
      return iter->second;
    }
    return nullptr;
  }

  const VisualFrame::ConstPtr getKeyFrame(const FrameId& id) const {
    auto iter = keyframes_.find(id);
    if (iter != keyframes_.end()) {
      return iter->second;
    }
    return nullptr;
  }
  VisualFrame::Ptr getKeyFrame(const FrameId& id) {
    auto iter = keyframes_.find(id);
    if (iter != keyframes_.end()) {
      return iter->second;
    }
    return nullptr;
  }

  // void getTrackedMapPointsAndKeyPoints(
  //     const VisualFrame::Ptr& frame, std::vector<MapPoint::Ptr>& mappoints,
  //     std::vector<KeyPoint*>& keypoints);

  // std::vector<MapPoint::Ptr> getTrackedMapPoints(const VisualFrame::Ptr&
  // frame);

  // for visualization
  std::vector<VisualFrame::Ptr> getAllFrames();
  std::vector<MapPoint::Ptr> getAllMapPoints();
  const PointCloudType::Ptr getPCLCloud() {
    return mappoint_pcl_;
  }

  const PointCloudType::Ptr getPCLLidarCloud() {
    return lidarpoint_pcl_;
  }

  void loadLidarMap() {
    lidarpoint_pcl_.reset(new PointCloudType);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile<pcl::PointXYZI> ("/media/host/OneTouch/ust_mini/exp_eval/5/mapping_0712/0.3/dlo_map.pcd", *cloud_in);
    for (size_t i = 0; i < cloud_in->points.size(); i++) {
      PointType point;
      point.x = cloud_in->points[i].x;
      point.y = cloud_in->points[i].y;
      point.z =cloud_in->points[i].z;
      lidarpoint_pcl_->push_back(point);

      auto mappoint = std::make_shared<MapPoint>();
      mappoint->set_position(xslam::Vector3(point.x, point.y ,point.z));
      addMapPoint(mappoint);
    }
    kdtree_.setInputCloud(lidarpoint_pcl_);
 
  }

  void cacheMapPointsPCL();
  void emptyCachePCL();
  std::vector<int> searchByRadiusPCL(const PointType& searchPoint, double radius);

  std::vector<VisualFrame::Ptr> getConnectedFramesByWeight(
      const VisualFrame::Ptr& frame, int weight);

  std::vector<VisualFrame::Ptr> getConnectedFramesTopK(
      const VisualFrame::Ptr& frame, int k);

  void updateMapPointDescriptor(
      MapPoint::Ptr& mappoint, Distance::DistanceType dist_type);

  void clear();

 protected:
  // TODO: do we need to use unique_id?
  std::unordered_map<MapPointId, MapPoint::Ptr> mappoints_;
  std::unordered_map<FrameId, VisualFrame::Ptr> keyframes_;
  PointCloudType::Ptr mappoint_pcl_;
  PointCloudType::Ptr lidarpoint_pcl_;
  pcl::KdTreeFLANN<PointType> kdtree_;

};

}  // namespace xslam
