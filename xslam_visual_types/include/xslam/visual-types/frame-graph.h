/*** 
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:20:03
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
 

// g2o hyper graph

#pragma once

#include "xslam/common/frame.h"

namespace xslam {

class FrameGraph {
 public:
  FrameGraph(/* args */);
  virtual ~FrameGraph();

 public:
  class Vertex;
  class Edge;

  class Vertex {
   public:
    inline void addEdge(Frame::Ptr frame, number_t weight) {
      edges_.emplace_back();
    }

    void removeEdge(Frame::Ptr frame);

   protected:
    std::vector<Edge> edges_;
  };

  // using VertexIdMap = std::map<FrameId, >
  // EdgeSet

  struct Edge {
    Frame::Ptr neighbor;
    number_t weight;
  };

  // using EdgeList = std::vector<Edge>;

 protected:
  std::unordered_map<FrameId, Vertex> vertices;
};

}  // namespace xslam
