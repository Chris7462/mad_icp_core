// Copyright 2024 R(obots) V(ision) and P(erception) group
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include "mad_icp_core/frame.hpp"
#include "mad_icp_core/lie_algebra.hpp"
#include "mad_icp_core/mad_icp.hpp"
#include "mad_icp_core/mad_tree.hpp"
#include "mad_icp_core/vel_estimator.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <deque>
#include <vector>

namespace mad_icp_core
{

class Pipeline
{
public:
  Pipeline(
    double sensor_hz,
    bool deskew,
    double b_max,
    double rho_ker,
    double p_th,
    double b_min,
    double b_ratio,
    int num_keyframes,
    int num_threads);

  ~Pipeline();

  const Eigen::Matrix4d currentPose() const { return frame_to_map_.matrix(); }
  const Eigen::Matrix4d keyframePose() const { return keyframe_to_map_.matrix(); }
  const std::vector<Eigen::Isometry3d> & trajectory() const { return trajectory_; }
  bool isInitialized() const { return is_initialized_; }
  bool isMapUpdated() const { return is_map_updated_; }
  size_t currentID() const { return seq_; }
  size_t keyframeID() const { return seq_keyframe_; }

  const ContainerType currentLeaves() const;
  const ContainerType modelLeaves() const;

  void compute(const double & curr_stamp, ContainerType curr_cloud_mem);

protected:
  void initialize(const double & curr_stamp, const ContainerTypePtr curr_cloud);
  void deskew(
    const ContainerTypePtr & curr_cloud,
    const Eigen::Isometry3d & T_prev,
    const Eigen::Isometry3d & T_now);

  // parameters — declaration order must match initializer list order in pipeline.cpp
  double sensor_hz_;
  bool deskew_;
  double b_max_;
  double p_th_;
  double b_min_;
  int num_keyframes_;
  int num_threads_;
  int max_parallel_levels_;

  // core objects
  MADicp icp_;
  VelEstimator vel_estimator_;

  // state
  Eigen::Isometry3d frame_to_map_;
  Eigen::Isometry3d keyframe_to_map_;
  Vector6d current_velocity_;

  std::deque<Frame *> keyframes_;
  std::deque<Frame *> frames_;
  std::vector<Eigen::Isometry3d> trajectory_;

  MADtree * current_tree_;
  LeafList model_leaves_;
  LeafList current_leaves_;

  size_t seq_;
  size_t seq_keyframe_;
  bool is_initialized_;
  bool is_map_updated_;
};

}  // namespace mad_icp_core
