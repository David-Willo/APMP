/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 06:08:44
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */

#pragma once

#include "global-feature.h"
#include "local-feature.h"
#include "xslam/common/arg.h"

namespace xslam {

class FeatureFactory {
 public:
  static LocalFeature::Type getLocalFeatureType(const std::string& type);
  static GlobalFeature::Type getGlobalFeatureType(const std::string& type);
  // static LocalFeature::Type global_type(const std::string& type);

  static LocalFeature::Ptr createLocalFeature(
      const LocalFeature::Options& options);

  static GlobalFeature::Ptr createGlobalFeature(
      const GlobalFeature::Options& options);
};

}  // namespace xslam
