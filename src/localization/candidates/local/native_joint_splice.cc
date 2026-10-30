// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/local/native_joint_error.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "Eigen/Eigenvalues"

namespace openloc::localization::unified {
namespace {

Result ConditionalRoot(const Eigen::MatrixXd& covariance,
                       Eigen::MatrixXd* root) {
  const Eigen::MatrixXd symmetric =
      0.5 * (covariance + covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
    return {Reason::CORRELATION_UNQUALIFIED,
            "Delayed native joint covariance decomposition failed"};
  const double tolerance = 64 * std::numeric_limits<double>::epsilon() *
      std::max(1.0, solver.eigenvalues().cwiseAbs().maxCoeff());
  if (solver.eigenvalues().minCoeff() < -tolerance)
    return {Reason::CORRELATION_UNQUALIFIED,
            "Delayed native branches have inconsistent shared driving covariance"};
  Eigen::Index rank = 0;
  for (const double value : solver.eigenvalues())
    if (value > tolerance) ++rank;
  root->resize(covariance.rows(), rank);
  Eigen::Index column = 0;
  for (Eigen::Index i = 0; i < solver.eigenvalues().size(); ++i)
    if (solver.eigenvalues()[i] > tolerance)
      root->col(column++) = solver.eigenvectors().col(i) *
                            std::sqrt(solver.eigenvalues()[i]);
  return {};
}

}  // namespace

Result NativeJointError::SpliceCandidate(
    const NativeJointError& candidate,
    const std::shared_ptr<const int>& lease, uint64_t scan_sequence) {
  if (!active_) return status_;
  if (!candidate.active_) return candidate.status_;
  std::string prefix;
  for (const auto& fork : forks_)
    if (lease && fork.second.lock() == lease) prefix = fork.first;
  const auto shared_fork = candidate.forks_.find(prefix);
  if (prefix.empty() || shared_fork == candidate.forks_.end() ||
      shared_fork->second.lock() != lease)
    return {Reason::CORRELATION_UNQUALIFIED,
            "Delayed native task lost its original stochastic fork lease"};
  if (structured()) {
    if (!candidate.structured() || primitives_ != candidate.primitives_)
      return {Reason::CORRELATION_UNQUALIFIED,
              "Delayed structured task lost its primitive source registry"};
    NativeJointError merged = candidate;
    const auto output = Error("output");
    if (!output.valid())
      return {Reason::CORRELATION_UNQUALIFIED,
              "Delayed structured task lost the actually published output"};
    merged.structured_memory_["output"] = output;
    merged.history_ = history_;
    merged.frontier_finalized_ = frontier_finalized_;
    merged.pending_sources_ = pending_sources_;
    for (const auto& source : candidate.pending_sources_)
      merged.pending_sources_.push_back(source);
    merged.AddSource({"lidar_raw", scan_sequence});
    merged.inserted_primitives_.insert(inserted_primitives_.begin(),
                                       inserted_primitives_.end());
    *this = std::move(merged);
    return {};
  }
  // A fork pins its original orthonormal basis and every later primitive
  // driver. These are independent unit variables spanning both branches,
  // even when ordinary history or old geometry dependencies have retired.
  Eigen::MatrixXd cross = Eigen::MatrixXd::Zero(dimension_, candidate.dimension_);
  for (const auto& driver : memory_) {
    if (driver.first.compare(0, prefix.size(), prefix) != 0) continue;
    const auto shared = candidate.memory_.find(driver.first);
    if (shared == candidate.memory_.end() ||
        shared->second.rows() != driver.second.rows())
      return {Reason::CORRELATION_UNQUALIFIED,
              "Delayed native task lost a pinned shared primitive driver"};
    cross += driver.second.transpose() * shared->second;
  }
  Eigen::MatrixXd live_residual;
  auto result = ConditionalRoot(
      Eigen::MatrixXd::Identity(dimension_, dimension_) -
          cross * cross.transpose(), &live_residual);
  if (!result.ok()) return result;
  const Eigen::Index merged_dimension =
      candidate.dimension_ + live_residual.cols();
  if (merged_dimension > config_.native_joint_max_memory_rows)
    return {Reason::CORRELATION_UNQUALIFIED,
            "Delayed native joint bridge exceeds its driving-variable capacity"};
  NativeJointError merged = candidate;
  merged.dimension_ = merged_dimension;
  for (auto& item : merged.memory_) {
    item.second.conservativeResize(Eigen::NoChange, merged_dimension);
    item.second.rightCols(live_residual.cols()).setZero();
  }
  Eigen::MatrixXd live_loading(dimension_, merged_dimension);
  live_loading.leftCols(candidate.dimension_) = cross;
  live_loading.rightCols(live_residual.cols()) = live_residual;
  merged.Set("output", Get("output") * live_loading);
  if (!merged.active_) return merged.status_;
  // Keep every ACTUALLY published snapshot. Candidate smoother outputs were
  // never public and must not replace the live graph history.
  merged.history_ = history_;
  merged.frontier_finalized_ = frontier_finalized_;
  merged.previous_cross_ = live_loading;
  merged.pending_sources_ = pending_sources_;
  for (const auto& source : candidate.pending_sources_)
    merged.pending_sources_.push_back(source);
  merged.AddSource({"lidar_raw", scan_sequence});
  Eigen::MatrixXd commit_root;
  result = ConditionalRoot(
      Eigen::MatrixXd::Identity(merged_dimension, merged_dimension) -
          live_loading.transpose() * live_loading, &commit_root);
  if (!result.ok()) return result;
  merged.driving_cross_.clear();
  merged.driving_cross_["commit:" + std::to_string(scan_sequence)] =
      commit_root.transpose();
  merged.Compact();
  if (!merged.active_) return merged.status_;
  *this = std::move(merged);
  return {};
}

}  // namespace openloc::localization::unified
