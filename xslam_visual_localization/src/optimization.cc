/*
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-04-13 08:32:50
 * @LastEditTime: 2024-07-02 07:10:15
 * @LastEditors: David-Willo
 * Jinhao HE (David Willo), IADC HKUST(GZ)
 * Copyright (c) 2024 by davidwillo@foxmail.com, All Rights Reserved. 
 */
#include "xslam/visual-localization/optimization.h"
#include <bitset>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
// #include <g2o/solvers/structure_only/structure_only_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include "xslam/common/logging.h"
#include "xslam/g2o_types/g2o_types.h"
#include "xslam/g2o_types/g2o_visual_types.h"

namespace xslam {

void Optimization::optimizeFramePosePnP(
    VisualFrame::Ptr& frame, const Options& options) {
  INFOLOG("# mappoints for pose estimate: {}", frame->countTrackedMapPoints());
  //
  g2o::SparseOptimizer optimizer;
  g2o::OptimizationAlgorithm* solver;
  solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<g2o::BlockSolver_6_3>(
          std::make_unique<
              g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>()));
  optimizer.setAlgorithm(solver);

  // add pose vertex
  auto* vertex_se3 = new VertexSE3();
  vertex_se3->setEstimate(frame->getTbw());
  vertex_se3->setId(0);
  vertex_se3->setFixed(false);
  optimizer.addVertex(vertex_se3);

  // add edge
  auto&& robust_kernel_delta = sqrt(options.projection_error());
  std::vector<g2o::OptimizableGraph::Edge*> edges;
  std::vector<KeyPointId> indices;
  // std::bitset is_outlier;
  std::vector<bool> is_outlier(frame->num_keypoints(), false);
  edges.reserve(frame->num_keypoints());
  indices.reserve(frame->num_keypoints());
  for (size_t i = 0; i < frame->num_keypoints(); i++) {
    auto&& mappt = frame->mappoint(i);
    if (!mappt || !mappt->is_valid()) {
      continue;
    }

    auto&& keypoint = frame->keypoint(i);
    auto edge = new EdgeProjectionPoseOnly(mappt->position(), frame->camera());
    edge->setVertex(0, vertex_se3);
    edge->setMeasurement(keypoint.uv);
    // TODO: set information
    edge->setInformation(Matrix2::Identity());
    if (options.use_robust_kernel()) {
      g2o::RobustKernel* kernel = g2o::RobustKernelFactory::instance()
                                      ->creator(options.robust_kernel_type())
                                      ->construct();
      CHECK_NOTNULL(kernel);
      //  = new g2o::RobustKernelHuber;
      edge->setRobustKernel(kernel);
      kernel->setDelta(robust_kernel_delta);
    }
    optimizer.addEdge(edge);
    edges.push_back(edge);
    indices.push_back(i);
  }

  // add prior edge, would size be unconsistent? index mess up?
  if (options.add_prior() && frame->hasPrior()) {
    auto edge = new EdgeSE3Prior();
    g2o::RobustKernel* kernel = g2o::RobustKernelFactory::instance()
                                      ->creator(options.robust_kernel_type())
                                      ->construct();
    edge->setVertex(0, vertex_se3);
    edge->setMeasurement(frame->getTwbPrior().inverse());

    CHECK_NOTNULL(kernel);
    kernel->setDelta(1);
    edge->setRobustKernel(kernel);
    // maybe scale it? not sure.
    edge->setInformation(Matrix6d::Identity());

    edge->setLevel(0);
    optimizer.addEdge(edge);
  }


  INFOLOG("#edges for optimization: {}", edges.size());
  INFOLOG("#vertices in optimization: {}", optimizer.vertices().size());
  INFOLOG("#edges in optimization: {}", optimizer.edges().size());

  const int num_iterations[4] = {10, 10, 10, 10};

  int num_bad = 0;

  const bool do_more = true;
  for (size_t it = 0; it < 4; it++) {
    vertex_se3->setEstimate(frame->getTbw());
    optimizer.initializeOptimization(0);
    optimizer.optimize(num_iterations[it]);

    num_bad = 0;
    if (do_more) {
      for (size_t i = 0, iend = edges.size(); i < iend; i++) {
        auto&& edge = edges[i];
        auto&& idx = indices[i];

        if (is_outlier[idx]) {
          edge->computeError();
        }

        auto&& chi2 = edge->chi2();

        if (chi2 > options.projection_error()) {
          is_outlier[idx] = true;
          edge->setLevel(1);  // outlier
          num_bad++;
        } else {
          is_outlier[idx] = false;
          edge->setLevel(0);  // inlier
        }

        if (it == 2) {
          edge->setRobustKernel(0);
        }
      }
      // INFOLOG("#iter: {}, #bad: {}", it, num_bad);
    }

    if (optimizer.edges().size() < 10) {
      break;
    }
  }

  // auto* vertex = static_cast<VertexSophusSE3*>(optimizer.vertex(0));
  const SE3& pose = vertex_se3->estimate();
  // const Matrix3d rot_mat = pose.rotationMatrix();
  // cv::Mat pose = Converter::toCvMat(SE3quat_recov);
  frame->setTbw(pose);
  // frame->Twc_est = frame->getTwc();

  if (options.clear_outliers()) {
    for (size_t i = 0; i < is_outlier.size(); i++) {
      if (is_outlier[i]) {
        frame->set_mappoint(i, nullptr);
      }
    }
  }
}

}  // namespace xslam