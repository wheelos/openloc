// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/local/bounded_joint_error.h"

#include <algorithm>
#include <set>
#include <utility>

#include "Eigen/QR"

namespace openloc::localization::unified {

Eigen::Index BoundedJointError::rows() const {
  Eigen::Index result = 0;
  for (const auto& entry : memory_) result += entry.second.rows();
  return result;
}

Result BoundedJointError::Independent(const std::string& key,
                                      const Eigen::MatrixXd& root) {
  if (key.empty() || memory_.count(key) || root.rows() <= 0 ||
      root.cols() <= 0 || !root.allFinite())
    return {Reason::INVALID_INPUT, "Invalid or reused frontier noise key"};
  if (root.rows() > maximum_rows_ - rows() ||
      root.cols() > maximum_rows_ - dimension_)
    return {Reason::CORRELATION_UNQUALIFIED,
            "Closed frontier capacity exceeded; no dependency dropped"};
  for (auto& entry : memory_) {
    entry.second.conservativeResize(Eigen::NoChange, dimension_ + root.cols());
    entry.second.rightCols(root.cols()).setZero();
  }
  Eigen::MatrixXd loading = Eigen::MatrixXd::Zero(
      root.rows(), dimension_ + root.cols());
  loading.rightCols(root.cols()) = root;
  dimension_ += root.cols();
  memory_[key] = std::move(loading);
  return {};
}

Result BoundedJointError::Linear(const std::string& key,
                                 const std::vector<Term>& terms) {
  if (key.empty() || terms.empty() || terms.front().coefficient.rows() <= 0)
    return {Reason::INVALID_INPUT, "Invalid frontier linear query"};
  const auto height = terms.front().coefficient.rows();
  const auto replaced = memory_.find(key);
  const auto old_rows = replaced == memory_.end() ? 0 : replaced->second.rows();
  if (height > maximum_rows_ - (rows() - old_rows))
    return {Reason::CORRELATION_UNQUALIFIED,
            "Closed frontier capacity exceeded; no dependency dropped"};
  Eigen::MatrixXd next = Eigen::MatrixXd::Zero(height, dimension_);
  for (const auto& term : terms) {
    const auto source = memory_.find(term.key);
    if (source == memory_.end() || term.coefficient.rows() != height ||
        term.coefficient.cols() != source->second.rows() ||
        !term.coefficient.allFinite())
      return {Reason::HISTORY_UNAVAILABLE, "Missing or invalid frontier dependency"};
    next.noalias() += term.coefficient * source->second;
  }
  if (!next.allFinite())
    return {Reason::CORRELATION_UNQUALIFIED, "Frontier loading overflow"};
  memory_[key] = std::move(next);
  return {};
}

Result BoundedJointError::Retain(const std::vector<std::string>& keys) {
  const std::set<std::string> keep(keys.begin(), keys.end());
  for (const auto& key : keep)
    if (!memory_.count(key))
      return {Reason::HISTORY_UNAVAILABLE, "Cannot retain absent frontier query"};
  for (auto it = memory_.begin(); it != memory_.end();)
    if (keep.count(it->first)) ++it;
    else it = memory_.erase(it);
  return {};
}

Result BoundedJointError::Compact() {
  const auto height = rows();
  if (height == 0) {
    dimension_ = 0;
    return {};
  }
  if (height >= dimension_) return {};
  Eigen::MatrixXd joint(height, dimension_);
  Eigen::Index offset = 0;
  for (const auto& entry : memory_) {
    joint.middleRows(offset, entry.second.rows()) = entry.second;
    offset += entry.second.rows();
  }
  // L^T = Q R -> L L^T = R^T R. Keep all rows of R, including singular
  // directions; no statistical/rank threshold discards weak information.
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(joint.transpose());
  const Eigen::MatrixXd upper =
      qr.matrixQR().topRows(height).template triangularView<Eigen::Upper>();
  const Eigen::MatrixXd root = upper.transpose();
  if (!root.allFinite())
    return {Reason::CORRELATION_UNQUALIFIED, "Frontier QR is nonfinite"};
  offset = 0;
  for (auto& entry : memory_) {
    entry.second = root.middleRows(offset, entry.second.rows());
    offset += entry.second.rows();
  }
  dimension_ = height;
  return {};
}

Result BoundedJointError::CrossCovariance(
    const std::string& first, const std::string& second,
    Eigen::MatrixXd* covariance) const {
  const auto a = memory_.find(first), b = memory_.find(second);
  if (!covariance || a == memory_.end() || b == memory_.end())
    return {Reason::HISTORY_UNAVAILABLE, "Query is outside the retained frontier"};
  Eigen::MatrixXd next = a->second * b->second.transpose();
  if (!next.allFinite())
    return {Reason::CORRELATION_UNQUALIFIED, "Frontier covariance overflow"};
  *covariance = std::move(next);
  return {};
}

}  // namespace openloc::localization::unified
