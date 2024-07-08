#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <deque>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
// #include <shared_mutex>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>
#include <thread>

#include "eigen_stl_types.h"
#include "eigen_types.h"

// #include <aslam/cameras/camera.h>

// #include <ros/ros.h>

// #include <Open3D/Open3D.h>

// struct PointXYZIRT {
//   PCL_ADD_POINT4D
//   PCL_ADD_INTENSITY;
//   uint16_t ring;
//   float time;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
//                                   (float, x, x)(float, y, y)(float, z, z)(
//                                       float, intensity,
//                                       intensity)(uint16_t, ring,
//                                                  ring)(float, time, time));

// PCL_INSTANTIATE_removeNanFromPointCloud(PointXYZIRT);
// PCL_INSTANTIATE(PCLBase, PointXYZIRT);

namespace xslam {

using number_t = double;

using SE3 = Sophus::SE3<number_t>;
using SO3 = Sophus::SO3<number_t>;
using Sim3 = Sophus::Sim3<number_t>;

using SE3f = Sophus::SE3<float>;
using SO3f = Sophus::SO3<float>;
using Sim3f = Sophus::Sim3<float>;

// using pcl::Normal;
// using pcl::PointXYZ;

// using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
// using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;

// using PointCloudXYZIRT = pcl::PointCloud<PointXYZIRT>;

// using CloudFPFH = pcl::PointCloud<pcl::FPFHSignature33>;
// using CloudNormal = pcl::PointCloud<pcl::Normal>;

// #define INFOLOG(args...) LOG(INFO) << fmt::format(args);

// #define WARNLOG(args...) LOG(WARNING) << fmt::format(args);

// #define ERRLOG(args...) LOG(ERROR) << fmt::format(args);

// #define FATALLOG(args...) LOG(FATAL) << fmt::format(args);

// using LocalDescriptor = Eigen::Matrix<float, 1, 8>;

}  // namespace xslam

#define DEFINE_ATTR(TypeName, AttrName)               \
 public:                                              \
  const TypeName##& AttrName##() const {              \
    return AttrName##_;                               \
  }                                                   \
  TypeName##& AttrName##_mutable() {                  \
    return AttrName##_;                               \
  }                                                   \
  void set_##AttrName##(const TypeName##& AttrName) { \
    AttrName##_ = AttrName;                           \
  }                                                   \
                                                      \
 protected:                                           \
  TypeName AttrName##_;