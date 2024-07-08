/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-09-26 14:40:52
 * @LastEditTime: 2024-07-02 07:22:09
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include <aslam/cameras/camera.h>
#include <faiss/IndexFlat.h>
#include <faiss/IndexIVFFlat.h>

#include "./keypoint-grid.h"
#include "./keypoint.h"
#include "./mappoint.h"
#include "xslam/common/common.h"
#include "xslam/common/frame.h"

namespace xslam {

using KeyPointId = uint32_t;
// using FrameConnectionWeight = uint32_t;
using FrameConnections = std::unordered_map<FrameId, uint32_t>;
using FrameWeight = std::pair<FrameId, uint32_t>;
using FrameWeightList = std::vector<std::pair<FrameId, uint32_t>>;

/**
 * @brief visual frame
 *
 */
class VisualFrame : public Frame {
 public:
  /// \brief The descriptor matrix stores descriptors in columns, i.e. the
  /// descriptor matrix
  ///        has num_bytes_per_descriptor rows and num_descriptors columns.
  // typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>
  //     DescriptorsT;
  typedef Eigen::VectorXd KeypointScoresT;

  using Ptr = std::shared_ptr<VisualFrame>;
  using ConstPtr = std::shared_ptr<const VisualFrame>;

  friend class VisualMap;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  VisualFrame(/* args */) = default;

  virtual ~VisualFrame() = default;

  // functions from aslam_cv2

  // project && unproject function

  // occupancy grid
  // DEFINE_ATTR(std::string, filename);

  bool project3(const Vector3d& pt, Vector2d* uv) const;

  // bool project3(const Vector3d& pt, Vector3d* uvr);

  inline const std::string& filename() const {
    return image_file_name_;
  }
  inline std::string& filename_mutable() {
    return image_file_name_;
  }
  inline void set_filename(const std::string& filename) {
    image_file_name_ = filename;
  }

  const cv::Mat& image() const {
    return image_;
  }
  cv::Mat& image_mutable() {
    return image_;
  }
  void set_image(const cv::Mat& image) {
    image.copyTo(image_);
  }
  void set_stamp(const double stamp_secs) {
    stamp_secs_ = stamp_secs;
  }

  double get_stamp() {
    return stamp_secs_;
  }

  void releaseImage();
  void releaseDescriptors();

  inline const aslam::Camera::Ptr& camera() const {
    return camera_;
  }
  inline aslam::Camera::Ptr& camera_mutable() {
    return camera_;
  }
  inline void set_camera(const aslam::Camera::Ptr& camera) {
    camera_ = camera;
  }

  inline const KeyPointContainer& keypoints() const {
    return keypoints_;
  }
  inline KeyPointContainer& keypoints_mutable() {
    return keypoints_;
  }
  void set_keypoints(const KeyPointContainer& keypoints);

  KeyPoint& keypoint_mutable(size_t idx) {
    return keypoints_.at(idx);
  }
  const KeyPoint& keypoint(size_t idx) const {
    return keypoints_.at(idx);
  }

  const size_t& num_keypoints() const {
    return num_keypoint_;
  }
  void set_num_keypoints(const size_t& num_keypoint) {
    num_keypoint_ = num_keypoint;
  }

  const MatrixXb& descriptors() const {
    return descriptors_;
  }
  MatrixXb& descriptors_mutable() {
    return descriptors_;
  }
  void set_descriptors(const MatrixXb& desc) {
    descriptors_ = desc;
  }

  // inline
  template <typename Scalar = unsigned char>
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> descriptor(
      size_t idx) const {
    return Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>(
        (Scalar*)descriptor_data(idx),
        descriptors_.rows() * sizeof(unsigned char) / sizeof(Scalar));
  }
  inline const VectorXf descriptorf(size_t idx) const {
    return descriptor<float>(idx);
  }

  // inline const unsigned char * descriptor_remapped(size_t idx) const {
  //   return descriptors_.row(idx).data();
  // }
  inline const unsigned char* descriptor_data(size_t idx) const {
    return descriptors_.col(idx).data();
  }
  // TODO: lvalue reference?
  // inline RowVectorXb descriptor_mutable(size_t idx) {
  //   return descriptors_.row(idx);
  // }

  // TODO: maybe a function for remapping the data?
  template <typename Scalar = unsigned char>
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> global_descriptor()
      const {
    return Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>(
        (Scalar*)global_descriptor_data(),
        global_descriptor_.size() * sizeof(unsigned char) / sizeof(Scalar));
  }
  inline const VectorXf global_descriptorf() const {
    return global_descriptor<float>();
  }
  inline const unsigned char* global_descriptor_data() const {
    return global_descriptor_.data();
  }
  inline VectorXb& global_descriptor_mutable() {
    return global_descriptor_;
  }
  inline void set_global_descriptor(const VectorXb& global_descriptor) {
    global_descriptor_ = global_descriptor;
  }

  inline const MapPoint::Ptr& mappoint(size_t idx) const {
    return mappoints_.at(idx);
  }
  inline MapPoint::Ptr& mappoint_mutable(size_t idx) {
    return mappoints_.at(idx);
  }

  inline void reset_mappoints() {
    return mappoints_.resize(num_keypoint_, nullptr);
  }

  inline void overwrite_mappoint_descriptors() {
    for (size_t i = 0; i < this->num_keypoints(); i++) {
      auto mappoint = mappoints_.at(i);
      if (!mappoint || !mappoint->is_valid()) {
        continue;
      }
      mappoint->set_descriptor(descriptors_.col(i));
    }
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

  inline const FrameConnections& frame_connections() const {
    return frame_connections_;
  }
  inline FrameConnections& frame_connections_mutable() {
    return frame_connections_;
  }
  inline void set_frame_connections(const FrameConnections& frame_connections) {
    frame_connections_ = frame_connections;
  }

  inline const FrameWeightList& frame_connections_weight() const {
    return frame_weight_list_;
  }
  inline FrameWeightList& frame_connections_weight_mutable() {
    return frame_weight_list_;
  }

  MapPointIdSet mappoint_indices_tracked();

  std::tuple<MatrixXb, MapPointList> descriptors_tracked();
  std::tuple<MatrixXb, std::vector<KeyPointId>> descriptors_untracked();
  // MapPointList mappoint_indices_tracked();
  // inline void set_frame_connections_weight(
  //     const FrameWeightList& frame_connections_weight) {
  //   frame_weight_list_ = frame_connections_weight;
  // }

  inline void increseConnectionWeight(const VisualFrame::Ptr& frame) {
    frame_connections_[frame->id()] += 1;
  }
  inline void updateConnectionWeight(
      const VisualFrame::Ptr& frame, uint32_t weight) {
    frame_connections_[frame->id()] = weight;
  }
  void updateConnections();

  void assignKeyPointsToGrid(int cell_size_row, int cell_size_col);

  inline const KeyPointGrid::Ptr& keypoint_grid() const {
    return keypoint_grid_;
  }

  std::vector<KeyPointId> getUntrackedKeyPointIndicesInGrid(
      const Vector2& uv, float radius) const;
  
  std::vector<KeyPointId> getKeyPointIndicesInGrid(
      const Vector2& uv, const float radius, const float radius2=-1, std::pair<float, float>* depth_info=nullptr) const;

  int countTrackedMapPoints();

  void initializeSearchIndex();

  const faiss::Index* index() const {
    return index_.get();
  }

  void compress();

 protected:
  std::string image_file_name_;
  cv::Mat image_;
  double stamp_secs_;

  aslam::Camera::Ptr camera_ = nullptr;

  bool tracked_only_ = false;

  size_t num_keypoint_;
  KeyPointContainer keypoints_;

  KeyPointGrid::Ptr keypoint_grid_ = nullptr;
  // std::vector<> mappoints_;
  MapPointList mappoints_;

  MatrixXb descriptors_;
  VectorXb global_descriptor_;

  std::unique_ptr<faiss::IndexFlatL2> quantizer_;
  std::unique_ptr<faiss::IndexIVFFlat> index_;

  FrameConnections frame_connections_;
  FrameWeightList frame_weight_list_;
public:
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

  cv::Mat visualizeFrameInfo(std::string name, float scale=1) {
    auto&& image = this->image();
    CHECK(!image.empty());

    cv::Mat image_viz;
    if (image.channels() == 1) {
      cv::Mat img_tmp;
      cv::cvtColor(image, img_tmp, cv::COLOR_GRAY2BGR);
      cv::resize(
          img_tmp, image_viz,
          cv::Size(
              image.cols/scale  ,
              image.rows/scale  ));
    } else {
      cv::resize(
          image, image_viz,
          cv::Size(
              image.cols/scale  ,
              image.rows/scale ));
    }

    std::vector<std::pair<double, size_t>> depth_indices;
    for (size_t i = 0; i < num_keypoints(); i++) {
      auto&& mp = mappoint(i);
      auto&& kp = keypoint(i);
      if (!mp) {
        Vector2 uv = kp.uv/scale  ;
        auto pt1 = cv::Point2f(
            uv.x() - 1.5,
            uv.y() - 1.5);
        auto pt2 = cv::Point2f(
            uv.x() + 1.5,
            uv.y() + 1.5);
        cv::rectangle(
            image_viz, cv::Rect(pt1, pt2), cv::Vec3b(255, 255, 255), -1);
      } else {
        Vector3 ptc = getTbw() * mp->position();
        depth_indices.push_back(std::make_pair(ptc.z(), i));
      }
    }

    sort(depth_indices.begin(), depth_indices.end());

    for (size_t i = 0; i < depth_indices.size(); i++) {
      auto&& feat = keypoint(depth_indices[i].second);
      Vector2 uv = feat.uv/scale  ;
      // uvs_mp.emplace_back(feat.uv.x(), feat.uv.y());
      auto pt1 = cv::Point2f(
          uv.x() - 2.5,
          uv.y() - 2.5);
      auto pt2 = cv::Point2f(
          uv.x() + 2.5,
          uv.y() + 2.5);
      cv::rectangle(
          image_viz, cv::Rect(pt1, pt2),
          makeJet3B(i * 0.85f / depth_indices.size()),
          // cv::Vec3b(10, 10, 10),
          -1);
    }
    // cv::imshow(name, image_viz);
    return image_viz;
  }
};

template <>
inline const VectorXb VisualFrame::descriptor<unsigned char>(size_t idx) const {
  return descriptors_.col(idx);
}

template <>
inline const VectorXb VisualFrame::global_descriptor<unsigned char>() const {
  return global_descriptor_;
}

}  // namespace xslam
