/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:22:43
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-types/mappoint.h"

#include "xslam/visual-types/track.h"

namespace xslam {

MapPoint::MapPoint() {
  aslam::generateId(&id_);

  // observations_.clear();
}

}  // namespace xslam
