/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-07 07:16:02
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-features/feature-factory.h"

#include "xslam/visual-features/superpoint-jit.h"
#include "xslam/visual-features/netvlad-jit.h"


namespace xslam {
using namespace std;

LocalFeature::Type FeatureFactory::getLocalFeatureType(
    const std::string& type) {
  LocalFeature::Type feat_type;
  if (type == "superpoint") {
    feat_type = LocalFeature::kSuerPointJIT;
  } else if (type == "superpoint_jit") {
    feat_type = LocalFeature::kSuerPointJIT;
  } else if (type == "orb") {
    feat_type = LocalFeature::kORB;
  }  else {
    throw std::runtime_error("unrecognized local feature type");
  }

  return feat_type;
}

GlobalFeature::Type FeatureFactory::getGlobalFeatureType(
    const std::string& type) {
  GlobalFeature::Type feat_type;
  if (type == "netvlad") {
    feat_type = GlobalFeature::kNetVLAD;
  }

  return feat_type;
}

LocalFeature::Ptr FeatureFactory::createLocalFeature(
    const LocalFeature::Options& options) {
  LocalFeature::Ptr extraction;
  switch (options.feature_type()) {
    case LocalFeature::kSuerPointJIT:
      extraction = make_shared<SuperPointJIT>(options);
      break;
    default:
      throw std::runtime_error("feature type not available");
      break;
  }
  std::cout << "Done load local extractor\n";
  return extraction;
}

GlobalFeature::Ptr FeatureFactory::createGlobalFeature(
    const GlobalFeature::Options& options) {
  GlobalFeature::Ptr extraction;
  switch (options.feature_type()) {
    case GlobalFeature::kNetVLAD:
      extraction = make_shared<NetVLADJIT>(options);
      break;
    default:
      throw std::runtime_error("feature type not available");
      break;
  }
  std::cout << "Done load global extractor\n";
  return extraction;
}

}  // namespace xslam