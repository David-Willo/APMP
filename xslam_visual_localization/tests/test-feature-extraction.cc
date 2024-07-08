#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <torch/csrc/jit/runtime/graph_executor.h>

#include "xslam/common/logging.h"
#include "xslam/common/timing.h"
#include "xslam/visual-features/netvlad-jit.h"
#include "xslam/visual-features/superpoint-jit.h"
#include "xslam/visual-features/superpoint.h"
#include "xslam/visual-localization/map-io.h"
#include "xslam/visual-localization/visualization.h"

namespace xslam {

using namespace std;

class TestFeatureExtraction : public MapIO, public testing::Test {
 protected:
  void SetUp() override {
    // options.reconstruction_path = "./test_map_sample/";
  }
  // aslam::Camera::Ptr camera_;
  // Options options;
};

TEST_F(TestFeatureExtraction, NetVLADJIT) {
  NetVLADJIT::Options options;
#if (defined(__aarch64__) || defined(__arm__))
  std::string script_path = "./test_models/netvlad_orin_1024x768_fp32.ts";
#else
  std::string script_path = "./test_models/netvlad_x86_4080_1024x768_fp32.ts";
#endif
  script_path = "/test_models/netvlad.pt"
  options.model_path(script_path);
  // options.image_height(768);
  // options.image_width(1024);
  NetVLADJIT model(options);
  torch::NoGradGuard grad_guard;

  cv::Mat image = cv::imread("./test_map_sample/000039.png", cv::IMREAD_COLOR);
  cv::Mat image_viz = image.clone();
  EXPECT_FALSE(image.empty());
  cv::Mat image_rgb, image_float;
  cv::cvtColor(image, image_rgb, cv::COLOR_BGR2RGB);
  image_rgb.convertTo(image_float, CV_32FC3, 1.f / 255.f, 0);

  timing::Timer timer("global extraction");
  VectorXb desc_data;
  model.extractFeature(image_float, desc_data);
  timer.Stop();

  const std::string& image_path = "./test_map_sample/images.bin";
  const std::string& camera_path = "./test_map_sample/cameras.bin";
  const std::string& feature_path = "./test_map_sample/global_features.h5";
  auto camera = loadCamera(camera_path);
  std::vector<VisualFrame::Ptr> frames;
  FrameIdMap frame_id_map;
  FrameMapPointsMap frame_points_map;
  loadFrames(image_path, camera, frames, frame_id_map, frame_points_map);

  loadGlobalFeatures(feature_path, frames);

  auto frame_id = frame_id_map[40];
  CHECK(frame_id.isValid());
  auto iter = find_if(
      frames.begin(), frames.end(), [&frame_id](const VisualFrame::Ptr& frame) {
        return (frame->id() == frame_id);
      });
  CHECK(iter != frames.end());
  auto frame = *iter;

  auto&& desc_frame = frame->global_descriptorf();

  VectorXf desc = Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>>(
      (float*)desc_data.data(),
      desc_data.size() * sizeof(unsigned char) / sizeof(float));
  EXPECT_NEAR_EIGEN(desc_frame, desc, 0.01);

  timing::Timing::Print(std::cout);
}

TEST_F(TestFeatureExtraction, SuperPointJIT) {
#if (defined(__aarch64__) || defined(__arm__))
  std::string script_path = "./test_models/superpoint_orin_1024x768_fp32.ts";
#else
  std::string script_path = "./test_models/superpoint_x86_4080_1024x768_fp32.ts";
#endif
  script_path = './test_models/superpoint.pt'
  SuperPointJIT::Options options;
  options.model_path(script_path).image_height(768).image_width(1024);
  SuperPointJIT model(options);
  torch::NoGradGuard grad_guard;

  cv::Mat image = cv::imread("./test_map_sample/000039.png", cv::IMREAD_COLOR);
  cv::Mat image_viz = image.clone();
  EXPECT_FALSE(image.empty());
  cv::Mat image_gray, image_float;
  cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  image_gray.convertTo(image_float, CV_32FC1, 1.f / 255.f, 0);

  timing::Timer timer("local extraction");
  KeyPointContainer keypoints;
  MatrixXb descriptors;
  model.extractFeature(image_float, keypoints, descriptors);
  timer.Stop();

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
  auto keypoints_frame = frame->keypoints();
  for (auto&& kp : keypoints) {
    cv::circle(
        image_viz, cv::Point(kp.uv.x(), kp.uv.y()), 2, cv::Scalar(0, 0, 255),
        1);
  }
  // cv::imshow("win", image_viz);
  // cv::waitKey(-1);

  auto dx = [](const MatrixXb& desc, size_t idx) -> VectorXf {
    return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>>(
        (float*)desc.col(idx).data(),
        desc.rows() * sizeof(unsigned char) / sizeof(float));
  };

  // for (auto&& kp : keypoints)
  size_t num_count = 0;
  for (size_t i = 0; i < keypoints.size(); i++) {
    auto&& kp = keypoints[i];
    auto iter = find_if(
        keypoints_frame.begin(), keypoints_frame.end(),
        [&kp](const KeyPoint& kpf) {
          return ((kpf.uv - kp.uv).norm() < 1e-6);
        });
    if (iter == keypoints_frame.end()) {
      continue;
    }
    auto idx = distance(keypoints_frame.begin(), iter);
    VectorXf df = frame->descriptorf(idx);
    VectorXf dxx = dx(descriptors, i);

    num_count++;
    EXPECT_NEAR_EIGEN(df.transpose(), dxx.transpose(), 0.01);
  }
  EXPECT_EQ(frame->filename(), "000039.png");
  // cout << "#loaded keypoints: " << frame->num_keypoints() << endl;
  // cout << "#frame keypoints: " << keypoints.size() << endl;
  // cout << "#common keypoints: " << num_count << endl;
  EXPECT_GE(num_count, keypoints.size() * 0.95);

  timing::Timing::Print(std::cout);
}

TEST_F(TestFeatureExtraction, SuperPointJIT_FP16) {
  // SuperPointJIT::Options options;
  // options.model_path("./test_models/superpoint.pt")
  //     .image_height(768)
  //     .image_width(1024)
  //     .build_tensorrt(true)
  //     .use_fp16(true);
  // SuperPointJIT model(options);
  // torch::NoGradGuard grad_guard;

  // cv::Mat image = cv::imread("./test_map_sample/000039.png", cv::IMREAD_COLOR);
  // cv::Mat image_viz = image.clone();
  // EXPECT_FALSE(image.empty());
  // cv::Mat image_gray, image_float;
  // cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
  // image_gray.convertTo(image_float, CV_32FC1, 1.f / 255.f, 0);

  // timing::Timer timer("local extraction");
  // KeyPointContainer keypoints;
  // MatrixXb descriptors;
  // model.extractFeature(image_float, keypoints, descriptors);
  // timer.Stop();

  // const std::string& image_path = "./test_map_sample/images.bin";
  // const std::string& camera_path = "./test_map_sample/cameras.bin";
  // const std::string& feature_path = "./test_map_sample/features.h5";
  // auto camera = loadCamera(camera_path);
  // std::vector<VisualFrame::Ptr> frames;
  // FrameIdMap frame_id_map;
  // FrameMapPointsMap frame_points_map;
  // loadFrames(image_path, camera, frames, frame_id_map, frame_points_map);

  // loadLocalFeatures(feature_path, frames);

  // auto frame_id = frame_id_map[40];
  // CHECK(frame_id.isValid());
  // auto iter = find_if(
  //     frames.begin(), frames.end(), [&frame_id](const VisualFrame::Ptr& frame) {
  //       return (frame->id() == frame_id);
  //     });
  // CHECK(iter != frames.end());
  // auto frame = *iter;

  // MatrixXf desc(frame->num_keypoints(), 256);
  // auto keypoints_frame = frame->keypoints();
  // for (auto&& kp : keypoints) {
  //   cv::circle(
  //       image_viz, cv::Point(kp.uv.x(), kp.uv.y()), 2, cv::Scalar(0, 0, 255),
  //       1);
  // }
  // // cv::imshow("win", image_viz);
  // // cv::waitKey(-1);

  // auto dx = [](const MatrixXb& desc, size_t idx) -> VectorXf {
  //   return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 1>>(
  //       (float*)desc.col(idx).data(),
  //       desc.rows() * sizeof(unsigned char) / sizeof(float));
  // };

  // // for (auto&& kp : keypoints)
  // size_t num_count = 0;
  // for (size_t i = 0; i < keypoints.size(); i++) {
  //   auto&& kp = keypoints[i];
  //   auto iter = find_if(
  //       keypoints_frame.begin(), keypoints_frame.end(),
  //       [&kp](const KeyPoint& kpf) {
  //         return ((kpf.uv - kp.uv).norm() < 1e-6);
  //       });
  //   if (iter == keypoints_frame.end()) {
  //     continue;
  //   }
  //   auto idx = distance(keypoints_frame.begin(), iter);
  //   VectorXf df = frame->descriptorf(idx);
  //   VectorXf dxx = dx(descriptors, i);

  //   num_count++;
  //   EXPECT_NEAR_EIGEN(df.transpose(), dxx.transpose(), 0.01);
  // }
  // EXPECT_EQ(frame->filename(), "000039.png");
  // // cout << "#loaded keypoints: " << frame->num_keypoints() << endl;
  // // cout << "#frame keypoints: " << keypoints.size() << endl;
  // // cout << "#common keypoints: " << num_count << endl;
  // EXPECT_GE(num_count, keypoints.size() * 0.95);

  // timing::Timing::Print(std::cout);
}

}  // namespace xslam

int main(int argc, char** argv) {
  // ros::init(argc, argv, "test_map_loading");
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
