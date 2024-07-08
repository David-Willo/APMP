/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:22:29
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-types/frame-graph.h"

namespace xslam {

using namespace std;

inline void FrameGraph::Vertex::removeEdge(Frame::Ptr frame) {
  edges_.erase(
      remove_if(
          edges_.begin(), edges_.end(),
          [frame](const Edge& edge) -> bool { return edge.neighbor == frame; }),
      edges_.end());
}

}  // namespace xslam
