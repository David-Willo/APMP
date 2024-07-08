#include <glog/logging.h>
#include <gtest/gtest.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <torch/csrc/jit/runtime/graph_executor.h>

#include "xslam/common/logging.h"
#include "xslam/common/timing.h"
#include "xslam/common/distances.h"

namespace xslam {

using namespace std;

TEST(TestDistance, Distances) {
  auto dist_func =
      Distance::createDistanceFunc<float>(Distance::DISTANCE_L2_NORM);
  VectorXf a(4), b(4);
  a << 1, 2, 3, 4;
  b << 4, 3, 2, 1;

  VectorXb aa = remap<unsigned char>(a);
  VectorXb bb = remap<unsigned char>(b);

  EXPECT_EQ(dist_func(aa, aa), 0.0) << "a";
  EXPECT_EQ(dist_func(aa, bb), sqrt(20.0f));
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