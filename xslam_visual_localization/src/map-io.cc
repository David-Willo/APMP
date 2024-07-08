/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:10:03
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-localization/map-io.h"

#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-radtan.h>
#include <filesystem>
#include <tuple>

#include "highfive/H5Easy.hpp"
#include "highfive/H5File.hpp"
#include "xslam/common/eigen_types.h"
#include "xslam/common/logging.h"
#include "xslam/visual-localization/endian.h"

namespace xslam {

// using colmap::ReadBinaryLittleEndian;
using namespace colmap;
using namespace std;
namespace fs = filesystem;

VisualMap::Ptr MapIO::loadVisualMap(const Options& options) {
  auto camera_path =
      (fs::path(options.reconstruction_path()) / "cameras.bin").string();
  auto image_path =
      (fs::path(options.reconstruction_path()) / "images.bin").string();
  auto points_path =
      (fs::path(options.reconstruction_path()) / "points3D.bin").string();

  // load camera info
  auto camera = loadCamera(camera_path);

  // load frame information
  std::vector<VisualFrame::Ptr> frames;
  FrameIdMap frame_id_map;
  FrameMapPointsMap frame_points_map;

  loadFrames(image_path, camera, frames, frame_id_map, frame_points_map);
  INFOLOG("Done load frames");
  

  // load global features
  loadGlobalFeatures(options.global_feature_path(), frames);
  INFOLOG("Done load gf");
  
  loadLocalFeatures(options.local_feature_path(), frames);
  INFOLOG("Done load lf");
  

  // load mappoint information
  std::vector<MapPoint::Ptr> mappoints;
  MapPointIdMap id_map;
  MapPointTrackMap track_map;
  loadMapPoints(points_path, mappoints, id_map, track_map);
  INFOLOG("Done load mps");
  

  // add elements to map
  auto map = make_shared<VisualMap>();
  for (auto&& frame : frames) {
    frame->assignKeyPointsToGrid(
        options.grid_size_row(), options.grid_size_col());
    map->addKeyFrame(frame);
  }
  for (auto&& mappt : mappoints) {
    map->addMapPoint(mappt);
  }

  // add frame-mappoint connections
  for (auto&& frame : frames) {
    auto&& mappts_indices_colmap = frame_points_map[frame->id()];
    MapPointList mappoints;
    mappoints.reserve(mappts_indices_colmap.size());
    for_each(
        mappts_indices_colmap.begin(), mappts_indices_colmap.end(),
        [&mappoints, &id_map, &map](const colmap_point3d_t& id) {
          if (id != kInvalidPointId) {
            mappoints.push_back(map->getMapPoint(id_map[id]));
            return;
          }

          mappoints.push_back(nullptr);
        });

    frame->set_mappoints(mappoints);
    CHECK_EQ(frame->num_keypoints(), frame->mappoints().size());
  }

  // add mappoint-frame connection
  for (auto&& mappt : mappoints) {
    auto&& tracks = track_map[mappt->id()];
    Vector3 avg_dir(0, 0, 0);
    for (auto&& [image_id, keypoint_id] : tracks) {
      auto&& frame_id = frame_id_map[image_id];
      mappt->tracks_mutable().addTrack(frame_id, keypoint_id);
      auto&& obs_frame = map->getKeyFrame(frame_id);
      SE3 obs_twb = obs_frame->getTwb();
      auto cur_obs = (obs_twb.translation() - mappt->position()).normalized();
      // INFOLOG("DEBUG obs dir frame: {} {} {} point: {} {} {} dir: {} {} {}",
      //         obs_twb.translation()[0], obs_twb.translation()[1],
      //         obs_twb.translation()[2], mappt->position()[0], mappt->position()[1], mappt->position()[2],
      //         cur_obs[0], cur_obs[1], cur_obs[2]);
      avg_dir += cur_obs;
    }
    // update view dir for mappoint
    mappt->set_view_dir(avg_dir.normalized());
    // INFOLOG("DEBUG obs avg: {} {} {} / {} = {} {} {}", avg_dir[0], avg_dir[1], avg_dir[2], tracks.size(), mappt->view_dir()[0], mappt->view_dir()[1], mappt->view_dir()[2])
  }


  // update frame-connections
  for (auto&& mappt : mappoints) {
    for (auto&& [lhs_id, ignore] : mappt->tracks()) {
      auto&& frame_lhs = map->getKeyFrame(lhs_id);
      for (auto&& [rhs_id, ignore] : mappt->tracks()) {
        if (rhs_id == lhs_id) {
          continue;
        }

        auto&& frame_rhs = map->getKeyFrame(rhs_id);
        frame_lhs->increseConnectionWeight(frame_rhs);
        frame_rhs->increseConnectionWeight(frame_lhs);
      }
    }
  }

  // TODO: update descriptor for mappoint
  for (auto&& mappt : mappoints) {
    map->updateMapPointDescriptor(mappt, Distance::DISTANCE_L2_NORM);
  }

  // update connections
  for (auto&& frame : frames) {
    frame->updateConnections();
  }

  // build index
  INFOLOG("Start build index");
  
  for (auto&& frame: frames) {
    frame->compress();
    frame->initializeSearchIndex();
  }
  INFOLOG("Load map done");
  

  return map;
}

aslam::Camera::Ptr MapIO::loadCamera(const std::string& path) {
  CHECK(fs::exists(path)) << path;

  std::ifstream file(path, std::ios::binary);
  CHECK(file.is_open()) << path;

  auto num_camera = ReadBinaryLittleEndian<uint64_t>(&file);
  CHECK_EQ(num_camera, 1);  // support opencv model only
  auto camera_id = ReadBinaryLittleEndian<colmap_camera_t>(&file);

  auto camera_model = ReadBinaryLittleEndian<int>(&file);
  CHECK_EQ(camera_model, 4);  // support opencv model only
  auto width = ReadBinaryLittleEndian<uint64_t>(&file);
  INFOLOG("width: {}", width);
  auto height = ReadBinaryLittleEndian<uint64_t>(&file);

  VectorXd camera_params(4), dist_params(4);
  ReadBinaryLittleEndian(&file, camera_params);
  ReadBinaryLittleEndian(&file, dist_params);
  return aslam::createCamera<aslam::PinholeCamera, aslam::RadTanDistortion>(
      camera_params, width, height, dist_params);
}

void MapIO::loadMapPoints(
    const std::string& path, std::vector<MapPoint::Ptr>& mappoints,
    MapPointIdMap& id_map, MapPointTrackMap& track_map) {
  CHECK(fs::exists(path)) << path;
  std::ifstream file(path, std::ios::binary);
  CHECK(file.is_open()) << path;

  const size_t num_points = ReadBinaryLittleEndian<uint64_t>(&file);
  mappoints.clear();
  id_map.clear();
  track_map.clear();
  mappoints.reserve(num_points);
  id_map.reserve(num_points);
  track_map.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    auto mappoint = make_shared<MapPoint>();

    Vector3d position;
    Vector3b color;

    const auto point_id = ReadBinaryLittleEndian<colmap_point3d_t>(&file);

    // Vector3& position = mappoint->position_mutable();

    ReadBinaryLittleEndian(&file, position);
    mappoint->set_position(position.cast<number_t>());

    ReadBinaryLittleEndian(&file, color);
    auto error = ReadBinaryLittleEndian<double>(&file);

    auto& tracks = track_map[mappoint->id()];
    const size_t track_length = ReadBinaryLittleEndian<uint64_t>(&file);

    for (size_t j = 0; j < track_length; ++j) {
      const colmap_image_t image_id =
          ReadBinaryLittleEndian<colmap_image_t>(&file);
      const colmap_keypoint_t keypoint_id =
          ReadBinaryLittleEndian<colmap_keypoint_t>(&file);
      tracks.emplace_back(image_id, keypoint_id);
    }

    // filter out bad points
    if (error < 3 && track_length >= 2) {
      mappoint->set_valid();
      id_map.emplace(point_id, mappoint->id());
      mappoints.push_back(mappoint);
    } else {
      // mappoint->set_invalid();
    }
  }
}

void MapIO::loadGlobalFeatures(
    const std::string& feature_path, std::vector<VisualFrame::Ptr>& frames) {
  CHECK(fs::exists(feature_path));
  // global feature
  HighFive::File h5file(feature_path, HighFive::File::ReadOnly);

  for (auto&& frame : frames) {
    auto filename = frame->filename();
    std::string desc_key = filename + "/global_descriptor";
    VectorXf desc = H5Easy::load<VectorXf>(h5file, desc_key);
    CHECK_EQ(desc.size(), 4096);

    VectorXb descb(4096 * sizeof(float) / sizeof(unsigned char));
    memcpy(descb.data(), desc.data(), sizeof(float) * 4096);
    frame->set_global_descriptor(descb);
  }
}

void MapIO::loadLocalFeatures(
    const std::string& feature_path, std::vector<VisualFrame::Ptr>& frames) {
  // local feature
  CHECK(fs::exists(feature_path));
  HighFive::File h5file(feature_path, HighFive::File::ReadOnly);
  for (auto&& frame : frames) {
    auto filename = frame->filename();
    std::string desc_key = filename + "/descriptors";
    MatrixXf desc = H5Easy::load<MatrixXf>(h5file, desc_key);
    // cout << filename << ' ' << desc.row(0) << endl;
    CHECK_EQ(desc.rows(), 256);
    CHECK_EQ(desc.cols(), frame->num_keypoints());
    // MatrixXb descb(
    //     desc.rows(), desc.cols() * sizeof(float) / sizeof(unsigned char));
    // memcpy(descb.data(), desc.data(), desc.() * sizeof(float));
    MatrixXb descb = Eigen::Map<MatrixXb>(
        (unsigned char*)desc.data(),
        desc.rows() * sizeof(float) / sizeof(unsigned char), desc.cols());
    frame->set_descriptors(descb);

    // RowVectorXb descb(4096 * sizeof(float) / sizeof(unsigned char));
    // memcpy(descb.data(), desc.data(), sizeof(float) * 4096);
    // frame->set_global_descriptor(descb);

    std::string keypoint_key = filename + "/keypoints";
    MatrixXf keypoints = H5Easy::load<MatrixXf>(h5file, keypoint_key);
    CHECK_EQ(keypoints.cols(), 2);
    CHECK_EQ(keypoints.rows(), frame->num_keypoints());
    for (size_t i = 0; i < frame->num_keypoints(); i++) {
      auto&& kp = frame->keypoint(i);
      double error = kp.distance2(keypoints.row(i).cast<number_t>());
      // INFOLOG("uv: {}, kp: {}", kp.uv.transpose(), keypoints.row(i));
      CHECK_LE(error, 2.);
      // CHECK_NEAR(error, 0.5, 1e-6);
    }

    std::string score_key = filename + "/scores";
    VectorXf scores = H5Easy::load<VectorXf>(h5file, score_key);
    CHECK_EQ(scores.size(), frame->num_keypoints());
    for (size_t i = 0; i < frame->num_keypoints(); i++) {
      auto& kp = frame->keypoint_mutable(i);
      kp.responce = scores(i);
    }
  }
}

void MapIO::loadFrames(
    const std::string& path, aslam::Camera::Ptr& camera,
    std::vector<VisualFrame::Ptr>& frames, FrameIdMap& id_map,
    FrameMapPointsMap& frame_points_map) {
  CHECK(fs::exists(path)) << path;

  std::ifstream file(path, std::ios::binary);
  CHECK(file.is_open()) << path;

  frames.clear();
  id_map.clear();
  frame_points_map.clear();

  const size_t num_reg_images = ReadBinaryLittleEndian<uint64_t>(&file);
  frames.reserve(num_reg_images);
  INFOLOG("#frames from reconstruction: {}", num_reg_images);
  for (size_t i = 0; i < num_reg_images; ++i) {
    auto frame = make_shared<VisualFrame>();
    auto frame_id = ReadBinaryLittleEndian<colmap_image_t>(&file);

    id_map[frame_id] = frame->id();

    // image.SetImageId();

    Vector4d qvec;  // w, x, y, z
    ReadBinaryLittleEndian(&file, qvec);
    Quaternion q(qvec(0), qvec(1), qvec(2), qvec(3));
    q.normalize();
    SO3 rot(q.cast<number_t>());

    Vector3d tvec;
    ReadBinaryLittleEndian(&file, tvec);
    frame->setTbw(SE3(rot, tvec.cast<number_t>()));

    frame->set_camera(camera);
    ReadBinaryLittleEndian<colmap_camera_t>(&file);

    string file_name;
    char name_char;
    do {
      file.read(&name_char, 1);
      if (name_char != '\0') {
        file_name += name_char;
      }
    } while (name_char != '\0');
    // frame->set
    frame->set_filename(file_name);

    // frame->key
    KeyPointContainer keypoints;

    const size_t num_points = ReadBinaryLittleEndian<colmap_point3d_t>(&file);
    frame->set_num_keypoints(num_points);

    // std::vector<Eigen::Vector2d> points2D;
    // std::vector<uint64_t> point3D_ids;
    // point3D_ids.reserve(num_points);
    // INFOLOG("#keypoints: {}", num_points);
    // break;
    // frame_points_map[frame->id()] = std::vector<size_t>();
    auto& mappoints = frame_points_map[frame->id()];
    mappoints.reserve(num_points);

    keypoints.reserve(num_points);
    for (size_t j = 0; j < num_points; ++j) {
      KeyPoint kp;
      kp.uv.x() = ReadBinaryLittleEndian<double>(&file);
      kp.uv.y() = ReadBinaryLittleEndian<double>(&file);
      keypoints.push_back(kp);
      mappoints.push_back(ReadBinaryLittleEndian<colmap_point3d_t>(&file));
    }

    frame->set_keypoints(keypoints);

    // image.SetUp(Camera(image.CameraId()));
    // image.SetPoints2D(points2D);

    // for (point2D_t point2D_idx = 0; point2D_idx < image.NumPoints2D();
    //      ++point2D_idx) {
    //   if (point3D_ids[point2D_idx] != kInvalidPoint3DId) {
    //     image.SetPoint3DForPoint2D(point2D_idx, point3D_ids[point2D_idx]);
    //   }
    // }

    // frame->num_feats_ = frame->features_.size();
    // frame->mappoints_.resize(frame->num_feats_, nullptr);
    // frame->file_name_ = file_name;
    //
    // frames.push_back(frame);
    // mappt_inds.push_back(mp_inds);
    //
    // filenames_.push_back(file_name);

    // image.SetRegistered(true);
    // reg_image_ids_.push_back(image.ImageId());

    // images_.emplace(image.ImageId(), image);

    frames.push_back(frame);
  }

  file.close();
}

}  // namespace xslam
