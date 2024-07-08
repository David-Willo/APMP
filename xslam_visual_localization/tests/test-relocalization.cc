
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include "xslam/common/logging.h"
#include "xslam/common/timing.h"
#include "xslam/visual-features/netvlad-jit.h"
#include "xslam/visual-features/superpoint-jit.h"
#include "xslam/visual-localization/map-io.h"
#include "xslam/visual-localization/matching.h"
#include "xslam/visual-localization/relocalization.h"
// #include "xslam/visual-localization/visualization.h"

namespace xslam {

using namespace std;
using namespace xslam;

class TestRelocalization : public testing::Test {
 protected:
  void SetUp() override {
    // options.reconstruction_path = "./test_map_sample/";
  }
  // aslam::Camera::Ptr camera_;
};

TEST_F(TestRelocalization, Relocalization) {
  MapIO::Options options =
      MapIO::Options()
          .reconstruction_path("./test_map_sample")
          .global_feature_path("./test_map_sample/global_features.h5")
          .local_feature_path("./test_map_sample/features.h5");
  // options.reconstruction_path = "./test_map_sample";
  // options.global_feature_path = "";
  // options.local_feature_path = "";
  auto map = MapIO::loadVisualMap(options);

  auto frame = make_shared<VisualFrame>();

  // VectorXb descriptor;
  {
#if (defined(__aarch64__) || defined(__arm__))
    std::string script_path = "./test_models/netvlad_orin_1024x768_fp32.ts";
#else
    std::string script_path = "./test_models/netvlad_x86_4080_1024x768_fp32.ts";
#endif
    // options.image_height(768);
    // options.image_width(1024);
    NetVLADJIT model(NetVLADJIT::Options().model_path(script_path));
    torch::NoGradGuard grad_guard;

    cv::Mat image =
        cv::imread("./test_map_sample/000039.png", cv::IMREAD_COLOR);
    cv::Mat image_viz = image.clone();
    EXPECT_FALSE(image.empty());
    cv::Mat image_rgb, image_float;
    cv::cvtColor(image, image_rgb, cv::COLOR_BGR2RGB);
    image_rgb.convertTo(image_float, CV_32FC3, 1.f / 255.f, 0);

    model.extractFeature(image_float, frame->global_descriptor_mutable());
  }
  {
#if (defined(__aarch64__) || defined(__arm__))
    std::string script_path = "./test_models/superpoint_orin_1024x768_fp32.ts";
#else
    std::string script_path =
        "./test_models/superpoint_x86_4080_1024x768_fp32.ts";
#endif
    SuperPointJIT::Options options;
    options.model_path(script_path).image_height(768).image_width(1024);
    SuperPointJIT model(options);
    torch::NoGradGuard grad_guard;

    cv::Mat image =
        cv::imread("./test_map_sample/000039.png", cv::IMREAD_COLOR);
    cv::Mat image_viz = image.clone();
    EXPECT_FALSE(image.empty());
    cv::Mat image_gray, image_float;
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
    image_gray.convertTo(image_float, CV_32FC1, 1.f / 255.f, 0);

    model.extractFeature(
        image_float, frame->keypoints_mutable(), frame->descriptors_mutable());
    frame->set_num_keypoints(frame->keypoints().size());

    frame->set_camera(map->getAllFrames()[0]->camera());
  }

  // Relocalization::Options reloc_options;
  Relocalization relocalization(Relocalization::Options().distance_type(
      xslam::Distance::DISTANCE_L2_NORM));
  relocalization.setVisualMap(map);
  auto&& candidates =
      relocalization.retrieveKeyframeTopK(frame->global_descriptor(), 10);
  WARNLOG("#candidates: {}", candidates.size());
  EXPECT_EQ(candidates.size(), 10);
  EXPECT_EQ(candidates.front()->filename(), "000039.png");
  auto frame_ref = candidates.front();
  for (auto&& frame : candidates) {
    WARNLOG("frame: {}", frame->filename());
  }

  auto&& clusters = relocalization.clusterCandidatesByCovisibility(candidates);
  WARNLOG("#clusters: {}", clusters.size());

  timing::Timer timer_match("matching");
  auto result = Matching::matchAgainstCluster(
      frame, clusters[0], map, Matching::Options());
  timer_match.Stop();

  timing::Timer timer_pnp("solve_pnp");
  relocalization.solvePnPCVRansac(frame);
  timer_pnp.Stop();
  EXPECT_NEAR_EIGEN(
      frame_ref->getTwb().translation(), frame->getTwb().translation(), 0.1);
  EXPECT_NEAR_EIGEN_QUATERNION(
      frame_ref->getTwb().unit_quaternion(), frame->getTwb().unit_quaternion(),
      0.05);
  // WARNLOG("ref position: {}",
  // WARNLOG("loc position: {}", frame->getTwb().translation().transpose());

  timing::Timing::Print(std::cout);

  // for (size_t i = 0; i < clusters.size(); i++) {
  //   WARNLOG("cluster_id: {}, #frames: {}", clusters.size(),
  //   clusters[i].size());
  // }

  // relocalization.retrieveKeyframeTopK()
  // relocalization.retrieveKeyframeTopK();
  // relocalization.retrieveKeyframeTopK();
}

}  // namespace xslam

// MAPLAB_UNITTEST_ENTRYPOINT

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