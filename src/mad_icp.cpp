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

#include "mad_icp_core/mad_icp.hpp"
#include "mad_icp_core/constants.hpp"

#include <Eigen/Eigenvalues>

#include <omp.h>

namespace mad_icp_core
{

MADicp::MADicp(double min_ball, double rho_ker, double b_ratio, int num_threads)
: min_ball_(min_ball), rho_ker_(sqrt(rho_ker)), b_ratio_(b_ratio), num_threads_(num_threads)
{
  X_.setIdentity();
  H_adder_.setZero();
  b_adder_.setZero();

  H_adders_ = std::vector<Matrix6d>(num_threads);
  b_adders_ = std::vector<Vector6d>(num_threads);
}

void MADicp::resetAdders()
{
  H_adder_.setZero();
  b_adder_.setZero();

  for (size_t i = 0; i < static_cast<size_t>(num_threads_); ++i) {
    H_adders_[i].setZero();
    b_adders_[i].setZero();
  }
}

void MADicp::setMoving(const LeafList & moving_leaves)
{
  moving_leaves_ = moving_leaves;
}

void MADicp::init(const Eigen::Isometry3d & moving_in_fixed)
{
  X_ = moving_in_fixed;
}

void MADicp::errorAndJacobian(
  double & e,
  JacobianMatrixType & J,
  const MADtree & fixed,
  const MADtree & moving,
  const Eigen::Vector3d & moving_transformed) const
{
  const auto & fixed_point  = fixed.mean_;
  const auto & fixed_normal = fixed.eigenvectors_.col(0);
  const auto & moving_point = moving.mean_;
  const Eigen::Matrix3d & R = X_.linear();

  e = (moving_transformed - fixed_point).dot(fixed_normal);
  J.block<1, 3>(0, 0) = fixed_normal.transpose() * R;
  J.block<1, 3>(0, 3) = -J.block<1, 3>(0, 0) * skew(moving_point);
}

void MADicp::update(const MADtree * fixed_tree)
{
  const int thread_id = omp_get_thread_num();

  for (auto & moving : moving_leaves_) {
    const Eigen::Vector3d ml = X_ * moving->mean_;
    const auto f = fixed_tree->bestMatchingLeafFast(ml);

    const double src_ball = min_ball_ + b_ratio_ * moving->mean_.norm();
    if ((ml - f->mean_).norm() > src_ball) {
      continue;
    }

    // NOTE: multiple threads (each handling a different keyframe) can
    // reach this line for the SAME `moving` leaf concurrently, since
    // moving_leaves_ is shared across all keyframe iterations of the
    // parallel loop in Pipeline::compute(). matched_ is std::atomic<bool>
    // specifically so this concurrent write is well-defined; relaxed
    // ordering is sufficient since every writer stores the same value
    // (true) and no other memory operation needs to be ordered against it.
    moving->matched_.store(true, std::memory_order_relaxed);

    JacobianMatrixType J;
    double e;

    errorAndJacobian(e, J, *f, *moving, ml);

    double scale = 1.0;
    const double chi = abs(e);
    if (chi > rho_ker_) {
      scale = rho_ker_ / chi;
    }
    const double w = 1.0 - f->bbox_(0) / min_ball_;
    scale *= w * w;

    H_adders_[thread_id] += scale * J.transpose() * J;
    b_adders_[thread_id] += scale * J.transpose() * e;
  }
}

void MADicp::updateState()
{
  for (size_t i = 0; i < static_cast<size_t>(num_threads_); ++i) {
    H_adder_ += H_adders_[i];
    b_adder_ += b_adders_[i];
  }

  // Damp the diagonal (Levenberg-Marquardt style) so a near-singular
  // H_adder_ (e.g. long, feature-poor corridors with few constraints)
  // doesn't produce an ill-conditioned solve.
  Matrix6d H_damped = H_adder_ + ICP_DAMPING * Matrix6d::Identity();

  Vector6d dx = H_damped.ldlt().solve(-b_adder_);

  // Second line of defense: if the solve still produced a non-finite
  // result (e.g. H_damped remained singular despite damping), skip this
  // update rather than propagating NaN/Inf into X_.
  if (!dx.allFinite()) {
    return;
  }

  Eigen::Isometry3d dX = Eigen::Isometry3d::Identity();
  dX.linear() = expMapSO3(dx.tail(3));
  dX.translation() = dx.head(3);
  X_ = X_ * dX;
}

}  // namespace mad_icp_core
