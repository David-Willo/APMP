/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-11-22 16:34:06
 * @LastEditTime: 2023-11-23 09:19:13
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2023 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#pragma once

#include <mutex>
#include <shared_mutex>

#include "aslam/common/unique-id.h"
#include "./mappoint.h"

#include "xslam/common/common.h"


namespace xslam {

UNIQUE_ID_DEFINE_ID(TileId);


struct TileGrid {
  int xIndex;
  int yIndex;

  bool operator==(const TileGrid& other) const {
      return xIndex == other.xIndex && yIndex == other.yIndex;
  }
};

// Custom hash function for GridCell
namespace hash {
struct TileGridHash {
    size_t operator()(const TileGrid& cell) const {
        return std::hash<int>()(cell.xIndex) ^ std::hash<int>()(cell.yIndex);
    }
};

}


TileGrid getTileGrid(double x, double y, double res)  {
      int xIndex = static_cast<int>(std::floor(x / res));
      int yIndex = static_cast<int>(std::floor(y / res));
      return {xIndex, yIndex};
}

class Tile {
 public:
  using Ptr = std::shared_ptr<Tile>;
  using ConstPtr = std::shared_ptr<const Tile>;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Tile();
  Tile(const Tile& tlie) = delete;
  virtual ~Tile() = default;

 public:
  inline const TileId& id() const {
    return id_;
  }
  inline void set_id(const TileId& id) {
    id_ = id;
  }


  inline void set_res(const double res) {
    res_ = res;
  }



  // TileId& id_mutable() {
  //   return id_;
  // }
  inline const MapPoint::Ptr& mappoint(size_t idx) const {
    return mappoints_.at(idx);
  }
  inline MapPoint::Ptr& mappoint_mutable(size_t idx) {
    return mappoints_.at(idx);
  }

  inline void reset_mappoints() {
    return mappoints_.resize(0, nullptr);
  }


  inline const MapPointList& mappoints() const {
    return mappoints_;
  }
  inline MapPointList& mappoints_mutable() {
    return mappoints_;
  }
  inline void set_mappoints(const MapPointList& mappoints) {
    mappoints_ = mappoints;
  }

  inline void set_mappoint(
      const KeyPointId& keypoint_id, const MapPoint::Ptr& mappoint) {
    mappoints_.at(keypoint_id) = mappoint;
  }

 

 private:
  TileId id_;
  TileGrid grid_cell_;
  mutable std::shared_mutex mutex_;
  MapPointList mappoints_;
  double res_ = 1;

};

}  // namespace xslam

UNIQUE_ID_DEFINE_ID_HASH(xslam::TileId);



