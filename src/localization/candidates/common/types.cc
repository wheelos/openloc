// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include <algorithm>
#include <cmath>
#include <set>

#include "Eigen/Cholesky"
#include "openloc/localization/local/native_types.h"

namespace openloc {
namespace localization {
namespace unified {

struct JointErrorExpression::Node {
  Eigen::Index rows;
  std::shared_ptr<const int> primitive;
  Eigen::MatrixXd root;
  std::vector<Term> terms;
};

JointErrorExpression JointErrorExpression::Primitive(
    const std::shared_ptr<const int>& identity, const Eigen::MatrixXd& root) {
  if (!identity || root.rows() == 0 || !root.allFinite()) return {};
  auto node = std::make_shared<Node>();
  node->rows = root.rows();
  node->primitive = identity;
  node->root = root;
  return JointErrorExpression(std::move(node));
}

JointErrorExpression JointErrorExpression::Combine(
    Eigen::Index rows, const std::vector<Term>& terms) {
  if (rows <= 0 || terms.empty()) return {};
  auto node = std::make_shared<Node>();
  node->rows = rows;
  for (const auto& term : terms) {
    if (!term.error.valid() || term.coefficient.rows() != rows ||
        term.coefficient.cols() != term.error.rows() ||
        !term.coefficient.allFinite())
      return {};
    if (term.coefficient.isZero(0)) continue;
    node->terms.push_back(term);
  }
  return JointErrorExpression(std::move(node));
}

Eigen::Index JointErrorExpression::rows() const {
  return node_ ? node_->rows : 0;
}

Result JointErrorExpression::CrossCovariance(
    const JointErrorExpression& other, Eigen::MatrixXd* covariance) const {
  if (!covariance || !valid() || !other.valid())
    return {Reason::INVALID_INPUT, "Invalid structured error covariance query"};
  // Iterative postorder handles shared DAG nodes once and avoids recursive
  // traversal of long sampled-IMU histories.
  std::vector<std::pair<std::shared_ptr<const Node>, bool>> pending{
      {node_, false}, {other.node_, false}};
  std::set<const Node*> visited;
  std::vector<std::shared_ptr<const Node>> order;
  while (!pending.empty()) {
    auto item = std::move(pending.back());
    pending.pop_back();
    if (item.second) {
      order.push_back(std::move(item.first));
      continue;
    }
    if (!visited.insert(item.first.get()).second) continue;
    pending.emplace_back(item.first, true);
    for (const auto& term : item.first->terms)
      pending.emplace_back(term.error.node_, false);
  }
  const bool self = node_ == other.node_;
  const Eigen::Index total = self ? rows() : rows() + other.rows();
  std::map<const Node*, Eigen::MatrixXd> weights;
  weights[node_.get()] = Eigen::MatrixXd::Zero(total, rows());
  weights[node_.get()].topRows(rows()).setIdentity();
  if (!self) {
    auto& second = weights[other.node_.get()];
    if (second.size() == 0)
      second = Eigen::MatrixXd::Zero(total, other.rows());
    second.bottomRows(other.rows()).setIdentity();
  }
  std::map<std::shared_ptr<const int>, Eigen::MatrixXd,
           std::owner_less<std::shared_ptr<const int>>> sources;
  for (auto it = order.rbegin(); it != order.rend(); ++it) {
    const auto& node = **it;
    const auto found = weights.find(it->get());
    if (found == weights.end()) continue;
    Eigen::MatrixXd weight = std::move(found->second);
    weights.erase(found);
    if (node.primitive) {
      Eigen::MatrixXd loading = weight * node.root;
      auto& shared = sources[node.primitive];
      if (shared.size() == 0) shared = std::move(loading);
      else if (shared.cols() == loading.cols()) shared += loading;
      else
        return {Reason::CORRELATION_UNQUALIFIED,
                "Structured primitive reused with inconsistent dimension"};
    } else {
      for (const auto& term : node.terms) {
        Eigen::MatrixXd loading = weight * term.coefficient;
        auto& next = weights[term.error.node_.get()];
        if (next.size() == 0) next = std::move(loading);
        else next += loading;
      }
    }
  }
  Eigen::MatrixXd next = Eigen::MatrixXd::Zero(rows(), other.rows());
  for (const auto& source : sources)
    next.noalias() += source.second.topRows(rows()) *
                      source.second.bottomRows(other.rows()).transpose();
  if (!next.allFinite())
    return {Reason::CORRELATION_UNQUALIFIED,
            "Structured covariance query overflowed"};
  *covariance = std::move(next);
  return {};
}

Eigen::Isometry3d Pose(const LocalState& state) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = state.orientation.toRotationMatrix();
  pose.translation() = state.position;
  return pose;
}

Matrix6d PoseCovariance(const LocalState& state) {
  Matrix6d covariance;
  covariance.topLeftCorner<3, 3>() = state.covariance.block<3, 3>(0, 0);
  covariance.topRightCorner<3, 3>() = state.covariance.block<3, 3>(0, 6);
  covariance.bottomLeftCorner<3, 3>() = state.covariance.block<3, 3>(6, 0);
  covariance.bottomRightCorner<3, 3>() = state.covariance.block<3, 3>(6, 6);
  return covariance;
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& value) {
  Eigen::Matrix3d result;
  result << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(), -value.y(),
      value.x(), 0.0;
  return result;
}

Eigen::Quaterniond ExpRotation(const Eigen::Vector3d& value) {
  const double angle = value.norm();
  if (angle < 1e-12) {
    return Eigen::Quaterniond(1.0, value.x() / 2.0, value.y() / 2.0,
                              value.z() / 2.0)
        .normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, value / angle));
}

Eigen::Vector3d LogRotation(const Eigen::Quaterniond& value) {
  Eigen::Quaterniond q = value.normalized();
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  const Eigen::AngleAxisd angle(q);
  return angle.angle() * angle.axis();
}

bool ValidCovariance(const Matrix6d& covariance) {
  if (!covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-9)) {
    return false;
  }
  Eigen::LLT<Matrix6d> decomposition(covariance);
  return decomposition.info() == Eigen::Success;
}

const char* ReasonName(Reason reason) {
  switch (reason) {
    case Reason::NONE:
      return "NONE";
    case Reason::CONFIG_INVALID:
      return "CONFIG_INVALID";
    case Reason::WAITING_FOR_STATIONARY:
      return "WAITING_FOR_STATIONARY";
    case Reason::WAITING_FOR_IMU:
      return "WAITING_FOR_IMU";
    case Reason::IMU_STALE:
      return "IMU_STALE";
    case Reason::WHEEL_STALE:
      return "WHEEL_STALE";
    case Reason::INVALID_INPUT:
      return "INVALID_INPUT";
    case Reason::DUPLICATE:
      return "DUPLICATE";
    case Reason::TIMESTAMP_REGRESSION:
      return "TIMESTAMP_REGRESSION";
    case Reason::CLOCK_INVALID:
      return "CLOCK_INVALID";
    case Reason::IMU_GAP:
      return "IMU_GAP";
    case Reason::COVARIANCE_EXCEEDED:
      return "COVARIANCE_EXCEEDED";
    case Reason::INNOVATION_REJECTED:
      return "INNOVATION_REJECTED";
    case Reason::HISTORY_UNAVAILABLE:
      return "HISTORY_UNAVAILABLE";
    case Reason::EPOCH_MISMATCH:
      return "EPOCH_MISMATCH";
    case Reason::RESET_WHILE_MOVING:
      return "RESET_WHILE_MOVING";
    case Reason::GLOBAL_UNAVAILABLE:
      return "GLOBAL_UNAVAILABLE";
    case Reason::GLOBAL_STALE:
      return "GLOBAL_STALE";
    case Reason::MAP_MISMATCH:
      return "MAP_MISMATCH";
    case Reason::MAP_NOT_READY:
      return "MAP_NOT_READY";
    case Reason::MATCH_FAILED:
      return "MATCH_FAILED";
    case Reason::DEGENERATE:
      return "DEGENERATE";
    case Reason::RELOCALIZATION_AMBIGUOUS:
      return "RELOCALIZATION_AMBIGUOUS";
    case Reason::RELOCALIZATION_VERIFYING:
      return "RELOCALIZATION_VERIFYING";
    case Reason::LIDAR_STALE:
      return "LIDAR_STALE";
    case Reason::POINT_TIME_MISSING:
      return "POINT_TIME_MISSING";
    case Reason::GEOMETRY_UNAVAILABLE:
      return "GEOMETRY_UNAVAILABLE";
    case Reason::CORRECTION_BUDGET_EXCEEDED:
      return "CORRECTION_BUDGET_EXCEEDED";
    case Reason::CORRELATION_UNQUALIFIED:
      return "CORRELATION_UNQUALIFIED";
    case Reason::UNCERTAINTY_UNQUALIFIED:
      return "UNCERTAINTY_UNQUALIFIED";
  }
  return "UNKNOWN";
}

}  // namespace unified
}  // namespace localization
}  // namespace openloc
