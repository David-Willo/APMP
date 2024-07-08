#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "xslam/common/logging.h"
#include "xslam/visual-localization/map-io.h"
#include "xslam/visual-localization/visualization.h"

namespace xslam {

using namespace std;

class TestMapIO : public MapIO, public testing::Test {
 protected:
  void SetUp() override {
    options.reconstruction_path("./test_map_sample/");
  }
  // aslam::Camera::Ptr camera_;
  Options options;
};

TEST_F(TestMapIO, CameraLoading) {
  auto camera_path = options.reconstruction_path() + "cameras.bin";
  auto camera = loadCamera(camera_path);
  CHECK_EQ(camera->imageWidth(), 1024);
  CHECK_EQ(camera->imageHeight(), 768);
  // 602.22559827398243 590.50399419145378 519.69403999999997 393.51303000000001

  EXPECT_NEAR_EIGEN(
      camera->getParameters(),
      Vector4d(
          602.22559827398243, 590.50399419145378, 519.69403999999997,
          393.51303000000001),
      1e-6);

  // -0.082783464123504583 0.080260007126736699 -0.00028703496585012913
  // 0.00070697263324392945
  EXPECT_NEAR_EIGEN(
      camera->getDistortion().getParameters(),
      Vector4d(
          -0.082783464123504583, 0.080260007126736699, -0.00028703496585012913,
          0.00070697263324392945),
      1e-6);
}

TEST_F(TestMapIO, FrameLoading) {
  const std::string& image_path = "./test_map_sample/images.bin";
  const std::string& camera_path = "./test_map_sample/cameras.bin";
  auto camera = loadCamera(camera_path);
  std::vector<VisualFrame::Ptr> frames;
  FrameIdMap frame_id_map;
  FrameMapPointsMap frame_points_map;
  loadFrames(image_path, camera, frames, frame_id_map, frame_points_map);

  // 46 0.99941478437878328 0.00099477609790315342 0.034189346744210529
  // 0.00043330685361395388 -2.1587527443877343 0.033589443469976547
  // -6.2368636465308214 0 000045.png
  {
    auto id = frame_id_map[46];
    auto iter = std::find_if(
        frames.begin(), frames.end(),
        [&id](const VisualFrame::Ptr& frame) { return frame->id() == id; });
    CHECK(iter != frames.end());
    auto frame = *iter;
    EXPECT_NEAR_EIGEN(
        frame->getTbw().translation(),
        Vector3d(
            -2.1587527443877343, 0.033589443469976547, -6.2368636465308214),
        1e-6);
  }
  // 29 0.99261977505584564 -0.0040262645395100979 -0.12116390116824402
  // -0.0030133728000807029 -1.7588451787366164 0.033677915118112155 -1.1227
  // 370159744576 0 000028.png

  {
    auto id = frame_id_map[29];
    auto iter = std::find_if(
        frames.begin(), frames.end(),
        [&id](const VisualFrame::Ptr& frame) { return frame->id() == id; });
    CHECK(iter != frames.end());
    auto frame = *iter;
    EXPECT_NEAR_EIGEN(
        frame->getTbw().translation(),
        Vector3d(
            -1.7588451787366164, 0.033677915118112155, -1.1227370159744576),
        1e-6);
    EXPECT_NEAR_EIGEN_QUATERNION(
        Quaterniond(
            0.99261977505584564, -0.0040262645395100979, -0.12116390116824402,
            -0.0030133728000807029),
        frame->getTbw().unit_quaternion(), 1e-6);

    EXPECT_NEAR_EIGEN(
        frame->keypoints().back().uv, Vector2d(570.5, 759.5), 1e-6);

    EXPECT_EQ(
        frame_points_map[frame->id()][4], std::numeric_limits<uint64_t>::max());
    EXPECT_EQ(frame_points_map[frame->id()][5], 7728);
    // EXPECT_EQ(frame_points_map[29][4], -1);
  }
}

TEST_F(TestMapIO, LocalFeatureLoading) {
  const std::string& image_path = "./test_map_sample/images.bin";
  const std::string& camera_path = "./test_map_sample/cameras.bin";
  const std::string& feature_path = "./test_map_sample/features.h5";
  auto camera = loadCamera(camera_path);
  std::vector<VisualFrame::Ptr> frames;
  FrameIdMap frame_id_map;
  FrameMapPointsMap frame_points_map;
  loadFrames(image_path, camera, frames, frame_id_map, frame_points_map);

  loadLocalFeatures(feature_path, frames);

  auto frame_id = frame_id_map[40];
  CHECK(frame_id.isValid());
  auto iter = find_if(
      frames.begin(), frames.end(), [&frame_id](const VisualFrame::Ptr& frame) {
        return (frame->id() == frame_id);
      });
  CHECK(iter != frames.end());
  auto frame = *iter;

  MatrixXf desc(frame->num_keypoints(), 256);
  ifstream ifs("./test_map_sample/local_000039.txt");
  for (size_t i = 0; i < frame->num_keypoints(); i++) {
    std::string line;
    std::getline(ifs, line);
    stringstream ss(line);
    for (size_t j = 0; j < 256; j++) {
      ss >> desc(i, j);
    }
  }
  ifs.close();

  for (size_t i = 0; i < frame->num_keypoints(); i++) {
    EXPECT_NEAR_EIGEN(frame->descriptor<float>(i), desc.row(i), 1e-6);
  }
}

TEST_F(TestMapIO, GlobalFeatureLoading) {
  const std::string& image_path = "./test_map_sample/images.bin";
  const std::string& camera_path = "./test_map_sample/cameras.bin";
  const std::string& feature_path = "./test_map_sample/global_features.h5";
  auto camera = loadCamera(camera_path);
  std::vector<VisualFrame::Ptr> frames;
  FrameIdMap frame_id_map;
  FrameMapPointsMap frame_points_map;
  loadFrames(image_path, camera, frames, frame_id_map, frame_points_map);

  loadGlobalFeatures(feature_path, frames);

  VectorXf desc(4096);
  ifstream ifs("./test_map_sample/descriptor_000039.txt");
  for (size_t i = 0; i < 4096; i++) {
    std::string line;
    std::getline(ifs, line);
    desc(i) = stof(line);
  }
  ifs.close();

  auto frame_id = frame_id_map[40];
  CHECK(frame_id.isValid());
  auto iter = find_if(
      frames.begin(), frames.end(), [&frame_id](const VisualFrame::Ptr& frame) {
        return (frame->id() == frame_id);
      });
  CHECK(iter != frames.end());
  // CHECK_NE(iter, frames.end());
  WARNLOG("#kps in 0039.png {}", (*iter)->num_keypoints());

  VectorXf descf = (*iter)->global_descriptor<float>();

  EXPECT_NEAR_EIGEN(desc, descf, 1e-6);
}

TEST_F(TestMapIO, MapPointLoading) {
  const std::string& mappoints_path = "./test_map_sample/points3D.bin";
  std::vector<MapPoint::Ptr> mappoints;
  MapPointIdMap id_map;
  MapPointTrackMap track_map;
  loadMapPoints(mappoints_path, mappoints, id_map, track_map);

  // 15224 27.606938745089757 -1.2134077211143495 18.810724428768076 43 37
  // 39 1.7154863106139839 12 289 29 563 11 296 7 352 13 307 6 363 4 393 3 397 2
  // 422 1 439

  // 9877 6.0874009510266625 1.4232541309565723 -0.58799388096264693 24 33
  // 42 2.3947346231312183 24 1061 23 1057 22 1074 21 1057 20 939 18 733 16 735
  // 12 703 13 724

  {
    auto id = id_map[13985];
    CHECK(id.isValid());
    auto iter = std::find_if(
        mappoints.begin(), mappoints.end(),
        [&id](const MapPoint::Ptr& mappt) { return mappt->id() == id; });
    EXPECT_NE(iter, mappoints.end());
    auto mappt = *iter;

    EXPECT_NEAR_EIGEN(
        mappt->position(),
        Vector3d(-2.16663281471953, 0.52936295063977745, -1.8483823779645776),
        1e-6);
    EXPECT_EQ(track_map[id].back().first, 7);
    EXPECT_EQ(track_map[id].back().second, 1382);
  }
  {
    auto id = id_map[9619];
    auto iter = std::find_if(
        mappoints.begin(), mappoints.end(),
        [&id](const MapPoint::Ptr& mappt) { return mappt->id() == id; });
    EXPECT_NE(iter, mappoints.end());
    auto mappt = *iter;

    EXPECT_NEAR_EIGEN(
        mappt->position(),
        Vector3d(2.4364745722031391, 0.47780685033719172, 0.29386796177697561),
        1e-6);
    EXPECT_EQ(track_map[id].back().first, 19);
    EXPECT_EQ(track_map[id].back().second, 1252);
  }
}

// TEST_F(TestMapIO, MapLoading) {
//   auto map = loadVisualMap(options);
//   ros::NodeHandle nh;
//   VisualizationVLoc visualization(nh);

//   visualization.setVisualMap(map);

//   auto thread_viz_ = std::unique_ptr<thread>(
//       new std::thread(&VisualizationVLoc::spin, &visualization));

//   ros::spin();
//   visualization.stop();

//   thread_viz_->join();
// }

}  // namespace xslam

// MAPLAB_UNITTEST_ENTRYPOINT

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_map_loading");
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  testing::InitGoogleTest(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_v = 1;
  // customInit();
  return RUN_ALL_TESTS();
}