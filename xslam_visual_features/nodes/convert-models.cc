/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-06-08 03:21:59
 * @LastEditTime: 2024-04-08 08:30:21
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2023 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include <glog/logging.h>

#include "xslam/common/timing.h"
#include "xslam/visual-features/netvlad-jit.h"
#include "xslam/visual-features/superpoint-jit.h"

using namespace xslam;
using namespace std;
using namespace std::chrono;

// DEFINE_string(local_feature_model, "", "model type.");
DEFINE_int32(image_width, 1280, "image width.");
DEFINE_int32(image_height, 720, "image height.");
DEFINE_string(local_feature_model_path, "", "model path.");
DEFINE_string(local_feature_output_path, "", "output model path.");
DEFINE_string(global_feature_model_path, "", "model path.");
DEFINE_string(global_feature_output_path, "", "output model path.");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_v = 1;

  {
    // SuperPointJIT::Options options;
    SuperPointJIT model(SuperPointJIT::Options()
                            .model_path(FLAGS_local_feature_model_path)
                            .build_tensorrt(true)
                            .image_height(FLAGS_image_height)
                            .image_width(FLAGS_image_width)
                            .use_fp16(false));
    model.saveModule(FLAGS_local_feature_output_path);
  }

  {
    NetVLADJIT model(NetVLADJIT::Options()
                         .model_path(FLAGS_global_feature_model_path)
                         .build_tensorrt(true)
                         .image_height(FLAGS_image_height)
                         .image_width(FLAGS_image_width));
    model.saveModule(FLAGS_global_feature_output_path);
  }

  return 0;
}
