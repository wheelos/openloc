// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/map/map_registration_error.h"

#include <array>
#include <cmath>
#include <limits>
#include <map>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"
#include "candidates/local/local_motion.h"
#include "pcl/kdtree/kdtree_flann.h"

namespace openloc::localization::unified {
namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Sources = std::vector<std::vector<std::size_t>>;
using Loading = Eigen::Matrix<double, 6, 3>;

Eigen::Vector3d Point(const pcl::PointXYZ& point) {
  return {point.x, point.y, point.z};
}

Result VoxelSources(const Cloud& raw, const Cloud& filtered, double voxel_size,
                    uint32_t maximum_points, Sources* output) {
  std::map<std::array<int64_t, 3>, std::vector<std::size_t>> groups;
  const float inverse = 1.0f / static_cast<float>(voxel_size);
  for (std::size_t i = 0; i < raw.size(); ++i) {
    const auto point = Point(raw[i]);
    std::array<int64_t, 3> key;
    for (int axis = 0; axis < 3; ++axis) {
      const float coordinate = static_cast<float>(point[axis]) * inverse;
      if (!std::isfinite(coordinate) ||
          std::abs(coordinate) >= std::numeric_limits<int32_t>::max() ||
          std::abs(coordinate - std::round(coordinate)) <= 1e-6f)
        return {Reason::CORRELATION_UNQUALIFIED,
                "Registration voxel assignment has no unique local derivative"};
      key[2 - axis] = static_cast<int64_t>(std::floor(coordinate));
    }
    groups[key].push_back(i);
  }
  if (groups.empty())
    return {Reason::INVALID_INPUT, "Registration voxel support is empty"};
  std::vector<const std::vector<std::size_t>*> ordered;
  for (const auto& group : groups) ordered.push_back(&group.second);
  const std::size_t count = std::min<std::size_t>(ordered.size(), maximum_points);
  if (count != filtered.size())
    return {Reason::CORRELATION_UNQUALIFIED,
            "Registration voxel support does not match actual filtered cloud"};
  Sources sources;
  sources.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t index = ordered.size() <= maximum_points
        ? i : i * (ordered.size() - 1) / (count - 1);
    const auto& members = *ordered[index];
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto raw_index : members) centroid += Point(raw[raw_index]);
    centroid /= members.size();
    if ((centroid - Point(filtered[i])).norm() > 1e-5 * (1 + centroid.norm()))
      return {Reason::CORRELATION_UNQUALIFIED,
              "Registration voxel order/centroid does not match actual filter"};
    sources.push_back(members);
  }
  *output = std::move(sources);
  return {};
}

struct NormalLoading {
  std::vector<std::pair<std::size_t, Eigen::Matrix3d>> points;
};

Result NormalSources(const Cloud& target, uint32_t count, std::size_t index,
                     const Eigen::Vector3d& frozen_normal,
                     const pcl::KdTreeFLANN<pcl::PointXYZ>& tree,
                     NormalLoading* output) {
  std::vector<int> indices(count);
  std::vector<float> distances(count);
  if (tree.nearestKSearch(target[index], count, indices, distances) !=
      static_cast<int>(count))
    return {Reason::CORRELATION_UNQUALIFIED, "Registration normal support missing"};
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const auto i : indices) center += Point(target[i]);
  center /= indices.size();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto i : indices) {
    const Eigen::Vector3d delta = Point(target[i]) - center;
    covariance.noalias() += delta * delta.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
  if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite() ||
      eigen.eigenvalues()[1] - eigen.eigenvalues()[0] <=
          64 * std::numeric_limits<double>::epsilon() *
              std::max(1.0, eigen.eigenvalues().cwiseAbs().maxCoeff()))
    return {Reason::CORRELATION_UNQUALIFIED,
            "Registration normal has no unique eigenvector derivative"};
  Eigen::Vector3d normal = eigen.eigenvectors().col(0);
  if (normal.dot(frozen_normal) < 0) normal = -normal;
  if (!normal.isApprox(frozen_normal, 1e-6))
    return {Reason::CORRELATION_UNQUALIFIED,
            "Registration normal support differs from frozen iteration"};
  for (const auto i : indices) {
    const Eigen::Vector3d centered = Point(target[i]) - center;
    Eigen::Matrix3d derivative = Eigen::Matrix3d::Zero();
    for (int axis = 1; axis < 3; ++axis) {
      const Eigen::Vector3d v = eigen.eigenvectors().col(axis);
      derivative.noalias() += v *
          ((centered.dot(normal) * v.transpose() +
            centered.dot(v) * normal.transpose()) /
           (eigen.eigenvalues()[0] - eigen.eigenvalues()[axis]));
    }
    output->points.emplace_back(i, derivative);
  }
  return {};
}

}  // namespace

Result LinearizeRawIcp(
    const Cloud& raw_source, const Cloud& source, const Cloud& raw_target,
    const Cloud& target, double source_voxel_size, double target_voxel_size,
    uint32_t normal_neighbors, uint32_t maximum_registration_points,
    uint32_t maximum_error_points, const std::vector<IcpErrorStep>& steps,
    RegistrationErrorMap* output, bool deterministic_target) {
  if (!output || steps.empty() || normal_neighbors < 3 ||
      maximum_registration_points < 2 ||
      raw_source.size() > maximum_error_points ||
      (!deterministic_target && raw_target.size() > maximum_error_points))
    return {Reason::CORRELATION_UNQUALIFIED,
            "Registration raw-error support exceeds its declared capacity"};
  Sources source_members, target_members;
  auto valid = VoxelSources(raw_source, source, source_voxel_size,
                            maximum_registration_points, &source_members);
  if (!valid.ok()) {
    valid.message = "Source: " + valid.message;
    return valid;
  }
  if (!deterministic_target) {
    valid = VoxelSources(raw_target, target, target_voxel_size,
                         maximum_registration_points, &target_members);
    if (!valid.ok()) {
      valid.message = "Target: " + valid.message;
      return valid;
    }
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target.makeShared());
  std::map<std::size_t, NormalLoading> normals;
  RegistrationErrorMap modeled;
  modeled.initial_jacobian.setIdentity();
  modeled.source_point_jacobians.assign(raw_source.size(), Loading::Zero());
  if (!deterministic_target)
    modeled.target_point_jacobians.assign(raw_target.size(), Loading::Zero());
  for (const auto& step : steps) {
    Eigen::LLT<Matrix6d> solve(step.information);
    if (solve.info() != Eigen::Success)
      return {Reason::CORRELATION_UNQUALIFIED,
              "Frozen registration information is not full rank"};
    Matrix6d compose = Matrix6d::Identity();
    compose.bottomRightCorner<3, 3>() =
        ExpRotation(step.delta.tail<3>()).toRotationMatrix();
    Matrix6d increment = Matrix6d::Identity();
    increment.bottomRightCorner<3, 3>() =
        RightRotationJacobian(-step.delta.tail<3>());
    const Matrix6d gain = increment * solve.solve(Matrix6d::Identity());
    const Matrix6d transition = compose - gain * step.information;
    modeled.initial_jacobian = (transition * modeled.initial_jacobian).eval();
    for (auto& loading : modeled.source_point_jacobians)
      loading = (transition * loading).eval();
    for (auto& loading : modeled.target_point_jacobians)
      loading = (transition * loading).eval();
    for (const auto& correspondence : step.correspondences) {
      if (correspondence.source >= source.size() ||
          correspondence.target >= target.size())
        return {Reason::INVALID_INPUT, "Frozen registration point index invalid"};
      const auto& normal = correspondence.normal;
      const Eigen::Vector3d moved =
          step.before * Point(source[correspondence.source]);
      Eigen::Matrix<double, 1, 6> jacobian;
      jacobian.head<3>() = normal.transpose();
      jacobian.tail<3>() =
          (moved - step.before.translation()).cross(normal).transpose();
      const Eigen::Matrix<double, 6, 1> direction =
          -correspondence.weight * gain * jacobian.transpose();
      const auto& sources = source_members[correspondence.source];
      for (const auto i : sources)
        modeled.source_point_jacobians[i].noalias() += direction *
            (normal.transpose() * step.before.linear() / sources.size());
      // A conditional fixed prior has no stochastic target-point/normal
      // directions. Their voxel/eigenvector derivatives are neither needed
      // nor qualified by the source-only model.
      if (deterministic_target) continue;
      const auto& targets = target_members[correspondence.target];
      for (const auto i : targets)
        modeled.target_point_jacobians[i].noalias() -=
            direction * normal.transpose() / targets.size();
      auto found = normals.find(correspondence.target);
      if (found == normals.end()) {
        NormalLoading loading;
        valid = NormalSources(target, normal_neighbors, correspondence.target,
                               normal, tree, &loading);
        if (!valid.ok()) return valid;
        found = normals.emplace(correspondence.target, std::move(loading)).first;
      }
      const Eigen::RowVector3d normal_residual =
          (moved - Point(target[correspondence.target])).transpose();
      for (const auto& dependency : found->second.points) {
        const auto& members = target_members[dependency.first];
        for (const auto i : members)
          modeled.target_point_jacobians[i].noalias() +=
              direction * normal_residual * dependency.second / members.size();
      }
    }
  }
  if (!modeled.initial_jacobian.allFinite())
    return {Reason::CORRELATION_UNQUALIFIED, "Registration error model overflow"};
  for (const auto* loadings : {&modeled.source_point_jacobians,
                             &modeled.target_point_jacobians})
    for (const auto& loading : *loadings)
      if (!loading.allFinite())
        return {Reason::CORRELATION_UNQUALIFIED,
                "Registration raw-point loading overflow"};
  modeled.assessment = {};
  *output = std::move(modeled);
  return {};
}

}  // namespace openloc::localization::unified
