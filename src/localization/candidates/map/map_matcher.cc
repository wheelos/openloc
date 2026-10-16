// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "candidates/map/map_matcher.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"
#include "Eigen/LU"
#include "candidates/common/observed_subspace.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"

namespace openloc {
namespace localization {
namespace unified {

MapMatcher::MapMatcher(const MatchConfig& config)
    : config_(config),
      map_(new Cloud),
      normals_(new pcl::PointCloud<pcl::Normal>) {}

Result MapMatcher::ValidateConfig() const {
  if (config_.algorithm != RegistrationAlgorithm::kNdt &&
      config_.algorithm != RegistrationAlgorithm::kPointToPlaneIcp) {
    return {Reason::CONFIG_INVALID, "unknown map registration algorithm"};
  }
  for (double value : {config_.source_voxel_size,
                       config_.resolution,
                       config_.step_size,
                       config_.transformation_epsilon,
                       config_.max_fitness,
                       config_.correspondence_distance,
                       config_.minimum_overlap,
                       config_.minimum_source_overlap,
                       config_.minimum_target_overlap,
                       config_.max_translation,
                       config_.max_rotation,
                       config_.minimum_information,
                       config_.maximum_condition,
                       config_.position_variance_floor,
                       config_.rotation_variance_floor,
                       config_.ambiguity_margin,
                       config_.rotation_length,
                       config_.normal_planarity_ratio,
                       config_.icp_max_rmse,
                       config_.icp_condition_ratio,
                       config_.icp_translation_tolerance,
                       config_.icp_rotation_tolerance}) {
    if (!std::isfinite(value) || value <= 0.0) {
      return {Reason::CONFIG_INVALID, "map matcher budgets must be positive"};
    }
  }
  if (!std::isfinite(config_.target_voxel_size) ||
      config_.target_voxel_size < 0.0) {
    return {Reason::CONFIG_INVALID,
            "target voxel size must be zero (inherit source) or positive"};
  }
  if (config_.minimum_points < 10 || config_.maximum_iterations == 0 ||
      config_.maximum_seeds < 2 || config_.minimum_overlap > 1.0 ||
      config_.maximum_condition < 1.0 ||
      config_.maximum_registration_points < config_.minimum_points ||
      config_.normal_neighbors < 4 ||
      config_.normal_neighbors >
          static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      config_.normal_planarity_ratio <= 0.0 ||
      config_.normal_planarity_ratio > 1.0 || config_.icp_max_rmse <= 0.0 ||
      config_.icp_condition_ratio <= 0.0 || config_.icp_condition_ratio > 1.0 ||
      config_.icp_translation_tolerance <= 0.0 ||
      config_.icp_rotation_tolerance <= 0.0) {
    return {Reason::CONFIG_INVALID, "invalid matcher counts/overlap"};
  }
  return {};
}

Result MapMatcher::DownsampleAndLimit(const Cloud::ConstPtr& input,
                                      Cloud* output, double voxel_size) const {
  if (input == nullptr || output == nullptr || !std::isfinite(voxel_size) ||
      voxel_size <= 0.0) {
    return {Reason::INVALID_INPUT, "invalid voxel filter input"};
  }
  Cloud::Ptr filtered(new Cloud);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(input);
  const float leaf_size = static_cast<float>(voxel_size);
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.filter(*filtered);
  output->clear();
  if (config_.algorithm != RegistrationAlgorithm::kPointToPlaneIcp ||
      filtered->size() <= config_.maximum_registration_points) {
    *output = std::move(*filtered);
    return {};
  }
  output->reserve(config_.maximum_registration_points);
  const size_t last = filtered->size() - 1;
  const size_t count = config_.maximum_registration_points;
  for (size_t sample = 0; sample < count; ++sample) {
    const size_t index = sample * last / (count - 1);
    output->push_back((*filtered)[index]);
  }
  return {};
}

Result MapMatcher::Load(const std::string& pcd_path) {
  Cloud::Ptr cloud(new Cloud);
  if (pcd_path.empty() || pcl::io::loadPCDFile(pcd_path, *cloud) != 0) {
    ready_ = false;
    return {Reason::MAP_NOT_READY, "cannot load global PCD: " + pcd_path};
  }
  return SetMap(cloud);
}

Result MapMatcher::SetMap(const Cloud::ConstPtr& cloud) {
  ready_ = false;
  target_voxelization_ms_ = 0.0;
  target_kdtree_build_ms_ = 0.0;
  target_normal_estimation_ms_ = 0.0;
  target_normal_neighbor_search_ms_ = 0.0;
  target_normal_fit_ms_ = 0.0;
  const bool profile =
      config_.enable_icp_profiling &&
      config_.algorithm == RegistrationAlgorithm::kPointToPlaneIcp;
  const auto valid = ValidateConfig();
  if (!valid.ok()) {
    return valid;
  }
  if (!cloud || cloud->size() < config_.minimum_points) {
    return {Reason::MAP_NOT_READY, "global map has insufficient points"};
  }
  for (const auto& point : *cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      return {Reason::INVALID_INPUT, "nonfinite point in global map"};
    }
  }
  const double target_voxel_size = config_.target_voxel_size > 0.0
                                       ? config_.target_voxel_size
                                       : config_.source_voxel_size;
  const auto voxelization_started =
      profile ? std::chrono::steady_clock::now()
              : std::chrono::steady_clock::time_point{};
  const Result downsampled =
      DownsampleAndLimit(cloud, map_.get(), target_voxel_size);
  if (profile) {
    target_voxelization_ms_ =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - voxelization_started)
            .count();
  }
  if (!downsampled.ok()) {
    return downsampled;
  }
  if (map_->size() < config_.minimum_points) {
    return {Reason::MAP_NOT_READY, "filtered global map is too sparse"};
  }
  const auto tree_started = profile ? std::chrono::steady_clock::now()
                                    : std::chrono::steady_clock::time_point{};
  tree_.setInputCloud(map_);
  if (profile) {
    target_kdtree_build_ms_ =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - tree_started)
            .count();
  }
  if (config_.algorithm == RegistrationAlgorithm::kNdt) {
    registration_.setResolution(config_.resolution);
    registration_.setStepSize(config_.step_size);
    registration_.setTransformationEpsilon(config_.transformation_epsilon);
    registration_.setMaximumIterations(config_.maximum_iterations);
    registration_.setInputTarget(map_);
  }
  const int normal_neighbors =
      config_.algorithm == RegistrationAlgorithm::kPointToPlaneIcp
          ? static_cast<int>(config_.normal_neighbors)
          : 10;
  const double planarity_ratio =
      config_.algorithm == RegistrationAlgorithm::kPointToPlaneIcp
          ? config_.normal_planarity_ratio
          : 0.2;
  const double minimum_eigenvalue =
      config_.algorithm == RegistrationAlgorithm::kPointToPlaneIcp ? 1e-8
                                                                   : 1e-9;
  normals_->resize(map_->size());
  normal_failures_.assign(map_->size(), NormalFailure::kOther);
  const auto normal_estimation_started =
      profile ? std::chrono::steady_clock::now()
              : std::chrono::steady_clock::time_point{};
  double normal_neighbor_search_ms = 0.0;
  for (size_t index = 0; index < map_->size(); ++index) {
    auto& normal = (*normals_)[index];
    normal.normal_x = normal.normal_y = normal.normal_z =
        std::numeric_limits<float>::quiet_NaN();
    std::vector<int> neighbors(normal_neighbors);
    std::vector<float> distances(normal_neighbors);
    const auto neighbor_search_started =
        profile ? std::chrono::steady_clock::now()
                : std::chrono::steady_clock::time_point{};
    const int neighbor_count = tree_.nearestKSearch(
        (*map_)[index], normal_neighbors, neighbors, distances);
    if (profile) {
      normal_neighbor_search_ms +=
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - neighbor_search_started)
              .count();
    }
    if (neighbor_count != normal_neighbors) {
      continue;
    }
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (int neighbor : neighbors) {
      const auto& point = (*map_)[neighbor];
      center += Eigen::Vector3d(point.x, point.y, point.z);
    }
    center /= neighbors.size();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int neighbor : neighbors) {
      const auto& point = (*map_)[neighbor];
      const Eigen::Vector3d delta =
          Eigen::Vector3d(point.x, point.y, point.z) - center;
      covariance += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    if (eigen.info() != Eigen::Success) {
      continue;
    }
    if (eigen.eigenvalues()(1) <= minimum_eigenvalue) {
      normal_failures_[index] = NormalFailure::kRank;
      continue;
    }
    if (eigen.eigenvalues()(0) > eigen.eigenvalues()(1) * planarity_ratio) {
      normal_failures_[index] = NormalFailure::kNonplanar;
      continue;
    }
    const Eigen::Vector3d n = eigen.eigenvectors().col(0);
    normal.normal_x = n.x();
    normal.normal_y = n.y();
    normal.normal_z = n.z();
  }
  if (profile) {
    target_normal_estimation_ms_ =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - normal_estimation_started)
            .count();
    target_normal_neighbor_search_ms_ = normal_neighbor_search_ms;
    target_normal_fit_ms_ = std::max(
        0.0, target_normal_estimation_ms_ - target_normal_neighbor_search_ms_);
  }
  ready_ = true;
  return {};
}

Result MapMatcher::MatchPointToPlaneIcp(const Cloud::ConstPtr& base_cloud,
                                        const Eigen::Isometry3d& initial,
                                        MatchResult* result) const {
  Eigen::Isometry3d estimate = initial;
  bool converged = false;
  const double distance_squared =
      config_.correspondence_distance * config_.correspondence_distance;
  const bool profile = config_.enable_icp_profiling;
  double nearest_search_ms = 0.0;
  double linearization_ms = 0.0;
  double solver_ms = 0.0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  for (uint32_t iteration = 0; iteration < config_.maximum_iterations;
       ++iteration) {
    Eigen::Matrix<double, 6, 6> information =
        Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> observability_information =
        Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
    uint32_t correspondences = 0;
    const auto correspondence_started =
        profile ? std::chrono::steady_clock::now()
                : std::chrono::steady_clock::time_point{};
    double iteration_search_ms = 0.0;
    for (const auto& point : *base_cloud) {
      const Eigen::Vector3d moved =
          estimate * Eigen::Vector3d(point.x, point.y, point.z);
      const pcl::PointXYZ query(static_cast<float>(moved.x()),
                                static_cast<float>(moved.y()),
                                static_cast<float>(moved.z()));
      const auto search_started = profile
                                      ? std::chrono::steady_clock::now()
                                      : std::chrono::steady_clock::time_point{};
      const int nearest_count =
          tree_.nearestKSearch(query, 1, indices, distances);
      if (profile) {
        iteration_search_ms +=
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - search_started)
                .count();
      }
      if (nearest_count != 1 || distances[0] > distance_squared) {
        continue;
      }
      const auto& normal = (*normals_)[static_cast<size_t>(indices[0])];
      const Eigen::Vector3d n(normal.normal_x, normal.normal_y,
                              normal.normal_z);
      if (!n.allFinite() || n.norm() < 0.5) {
        continue;
      }
      const auto& target = (*map_)[static_cast<size_t>(indices[0])];
      const Eigen::Vector3d q(target.x, target.y, target.z);
      const double residual = n.dot(moved - q);
      Eigen::Matrix<double, 1, 6> jacobian;
      jacobian.head<3>() = n.transpose();
      jacobian.tail<3>() =
          (moved - estimate.translation()).cross(n).transpose();
      const double weight = std::min(
          1.0, config_.icp_max_rmse / std::max(std::abs(residual), 1e-12));
      information.noalias() += weight * jacobian.transpose() * jacobian;
      observability_information.noalias() += jacobian.transpose() * jacobian;
      gradient.noalias() += weight * jacobian.transpose() * residual;
      ++correspondences;
    }
    if (profile) {
      const double correspondence_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - correspondence_started)
              .count();
      nearest_search_ms += iteration_search_ms;
      linearization_ms +=
          std::max(0.0, correspondence_ms - iteration_search_ms);
      result->icp_nearest_search_ms = nearest_search_ms;
      result->icp_linearization_ms = linearization_ms;
    }
    if (correspondences < config_.minimum_points) {
      return {Reason::MATCH_FAILED, "insufficient planar ICP correspondences"};
    }
    const auto solver_started = profile
                                    ? std::chrono::steady_clock::now()
                                    : std::chrono::steady_clock::time_point{};
    const auto record_solver_time = [&]() {
      if (profile) {
        solver_ms += std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - solver_started)
                         .count();
        result->icp_solver_ms = solver_ms;
      }
    };
    Eigen::Matrix<double, 6, 1> column_scales;
    for (int column = 0; column < 6; ++column) {
      column_scales(column) =
          std::sqrt(std::max(0.0, observability_information(column, column)));
      if (!std::isfinite(column_scales(column)) ||
          column_scales(column) < 1e-8) {
        record_solver_time();
        return {Reason::DEGENERATE,
                "unobservable point-to-plane ICP direction"};
      }
    }
    const Eigen::Matrix<double, 6, 6> normalized_information =
        column_scales.cwiseInverse().asDiagonal() * observability_information *
        column_scales.cwiseInverse().asDiagonal();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(
        normalized_information);
    if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite() ||
        eigen.eigenvalues().minCoeff() <= 0.0 ||
        eigen.eigenvalues().minCoeff() / eigen.eigenvalues().maxCoeff() <
            config_.icp_condition_ratio) {
      record_solver_time();
      return {Reason::DEGENERATE, "unobservable point-to-plane ICP geometry"};
    }
    const Eigen::Matrix<double, 6, 1> delta =
        information.ldlt().solve(-gradient);
    if (!delta.allFinite()) {
      record_solver_time();
      return {Reason::MATCH_FAILED, "point-to-plane ICP update is nonfinite"};
    }
    const Eigen::Vector3d rotation_vector = delta.tail<3>();
    const double rotation_angle = rotation_vector.norm();
    const Eigen::Matrix3d rotation_delta =
        rotation_angle > 1e-12
            ? Eigen::AngleAxisd(rotation_angle,
                                rotation_vector / rotation_angle)
                  .toRotationMatrix()
            : Eigen::Matrix3d::Identity();
    estimate.translation() += delta.head<3>();
    estimate.linear() = rotation_delta * estimate.linear();
    record_solver_time();
    result->iterations = iteration + 1;
    if (delta.head<3>().norm() < config_.icp_translation_tolerance &&
        rotation_angle < config_.icp_rotation_tolerance) {
      converged = true;
      break;
    }
  }
  result->converged = converged;
  if (!converged) {
    return {Reason::MATCH_FAILED, "point-to-plane ICP iteration limit"};
  }
  if (!estimate.matrix().allFinite() ||
      !(estimate.linear().transpose() * estimate.linear())
           .isApprox(Eigen::Matrix3d::Identity(), 1e-4) ||
      std::abs(estimate.linear().determinant() - 1.0) > 1e-4) {
    return {Reason::INVALID_INPUT,
            "point-to-plane ICP returned invalid rigid transform"};
  }

  Cloud::Ptr aligned_ptr(new Cloud);
  aligned_ptr->reserve(base_cloud->size());
  Eigen::Matrix<double, 6, 6> final_information =
      Eigen::Matrix<double, 6, 6>::Zero();
  double residual_squared_sum = 0.0;
  uint32_t final_correspondences = 0;
  uint32_t source_matches = 0;
  for (const auto& point : *base_cloud) {
    const Eigen::Vector3d moved =
        estimate * Eigen::Vector3d(point.x, point.y, point.z);
    const pcl::PointXYZ transformed(static_cast<float>(moved.x()),
                                    static_cast<float>(moved.y()),
                                    static_cast<float>(moved.z()));
    aligned_ptr->push_back(transformed);
    if (tree_.nearestKSearch(transformed, 1, indices, distances) != 1 ||
        distances[0] > distance_squared) {
      continue;
    }
    ++source_matches;
    const auto& normal = (*normals_)[static_cast<size_t>(indices[0])];
    const Eigen::Vector3d n(normal.normal_x, normal.normal_y, normal.normal_z);
    if (!n.allFinite() || n.norm() < 0.5) {
      continue;
    }
    const auto& target = (*map_)[static_cast<size_t>(indices[0])];
    const Eigen::Vector3d q(target.x, target.y, target.z);
    const double residual = n.dot(moved - q);
    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian.head<3>() = n.transpose();
    jacobian.tail<3>() = (moved - estimate.translation()).cross(n).transpose();
    final_information.noalias() += jacobian.transpose() * jacobian;
    residual_squared_sum += residual * residual;
    ++final_correspondences;
  }
  if (aligned_ptr->empty()) {
    return {Reason::MATCH_FAILED, "point-to-plane ICP produced no scan points"};
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> aligned_tree;
  aligned_tree.setInputCloud(aligned_ptr);
  uint32_t target_matches = 0;
  for (const auto& target : *map_) {
    if (aligned_tree.nearestKSearch(target, 1, indices, distances) == 1 &&
        distances[0] <= distance_squared) {
      ++target_matches;
    }
  }
  result->source_overlap =
      static_cast<double>(source_matches) / base_cloud->size();
  result->target_overlap = static_cast<double>(target_matches) / map_->size();
  result->overlap =
      static_cast<double>(final_correspondences) / base_cloud->size();
  result->matched_points = final_correspondences;
  if (final_correspondences < config_.minimum_points ||
      result->source_overlap < config_.minimum_source_overlap ||
      result->target_overlap < config_.minimum_target_overlap) {
    return {Reason::MATCH_FAILED,
            "insufficient bidirectional point-to-plane ICP overlap"};
  }
  const double rmse = std::sqrt(residual_squared_sum / final_correspondences);
  result->score = rmse;
  if (!std::isfinite(rmse) || rmse > config_.icp_max_rmse) {
    return {Reason::MATCH_FAILED, "point-to-plane ICP residual gate"};
  }
  Eigen::Matrix<double, 6, 1> column_scales;
  for (int column = 0; column < 6; ++column) {
    column_scales(column) =
        std::sqrt(std::max(0.0, final_information(column, column)));
    if (!std::isfinite(column_scales(column)) || column_scales(column) < 1e-8) {
      return {Reason::DEGENERATE,
              "unobservable final point-to-plane ICP direction"};
    }
  }
  const Eigen::Matrix<double, 6, 6> normalized_information =
      column_scales.cwiseInverse().asDiagonal() * final_information *
      column_scales.cwiseInverse().asDiagonal();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> final_eigen(
      normalized_information);
  if (final_eigen.info() != Eigen::Success ||
      final_eigen.eigenvalues().minCoeff() <= 0.0 ||
      final_eigen.eigenvalues().minCoeff() /
              final_eigen.eigenvalues().maxCoeff() <
          config_.icp_condition_ratio) {
    return {Reason::DEGENERATE,
            "unobservable final point-to-plane ICP geometry"};
  }
  result->hessian_min_eigenvalue = final_eigen.eigenvalues().minCoeff();
  result->hessian_condition_number = final_eigen.eigenvalues().maxCoeff() /
                                     final_eigen.eigenvalues().minCoeff();
  const double hessian_min_eigenvalue = result->hessian_min_eigenvalue;
  const double hessian_condition_number = result->hessian_condition_number;
  const Eigen::Isometry3d correction = initial.inverse() * estimate;
  if (correction.translation().norm() > config_.max_translation ||
      LogRotation(Eigen::Quaterniond(correction.linear())).norm() >
          config_.max_rotation) {
    return {Reason::MATCH_FAILED,
            "point-to-plane ICP correction exceeds configured basin"};
  }
  result->pose = estimate;
  const Result evidence = ProjectMapInformation(
      final_information, residual_squared_sum / final_correspondences, config_,
      result);
  result->hessian_min_eigenvalue = hessian_min_eigenvalue;
  result->hessian_condition_number = hessian_condition_number;
  return evidence;
}

Result MapMatcher::Evidence(const Cloud& cloud, MatchResult* result) const {
  result->ndt_support_available = true;
  Matrix6d information = Matrix6d::Zero();
  uint32_t accepted = 0;
  double residual_sum = 0.0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  const double distance_squared =
      config_.correspondence_distance * config_.correspondence_distance;
  for (const auto& point : cloud) {
    if (tree_.nearestKSearch(point, 1, indices, distances) != 1 ||
        distances[0] > distance_squared) {
      continue;
    }
    const auto& normal = (*normals_)[indices[0]];
    ++result->ndt_distance_supported_points;
    Eigen::Vector3d n(normal.normal_x, normal.normal_y, normal.normal_z);
    if (!n.allFinite() || n.norm() < 0.5) {
      switch (normal_failures_[indices[0]]) {
        case NormalFailure::kRank:
          ++result->ndt_rank_rejected_points;
          break;
        case NormalFailure::kNonplanar:
          ++result->ndt_nonplanar_rejected_points;
          break;
        case NormalFailure::kOther:
          ++result->ndt_other_normal_rejected_points;
          break;
      }
      continue;
    }
    n.normalize();
    const auto& target = (*map_)[indices[0]];
    const Eigen::Vector3d x(point.x, point.y, point.z);
    const Eigen::Vector3d center(target.x, target.y, target.z);
    // Rotation error is left-multiplicative around base origin in map axes.
    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian.head<3>() = n.transpose();
    jacobian.tail<3>() = -n.transpose() * Skew(x - result->pose.translation());
    information += jacobian.transpose() * jacobian;
    const double residual = n.dot(x - center);
    residual_sum += residual * residual;
    ++accepted;
  }
  result->overlap = static_cast<double>(accepted) / cloud.size();
  result->matched_points = accepted;
  result->ndt_normal_supported_points = accepted;
  const double coverage =
      config_.separate_ndt_support
          ? static_cast<double>(result->ndt_distance_supported_points) /
                cloud.size()
          : result->overlap;
  if (accepted < config_.minimum_points || coverage < config_.minimum_overlap) {
    return {Reason::MATCH_FAILED, "insufficient map overlap"};
  }
  return ProjectMapInformation(information, residual_sum / accepted, config_,
                               result);
}

Result ProjectMapInformation(const Matrix6d& information,
                             double residual_variance,
                             const MatchConfig& config, MatchResult* result) {
  if (result == nullptr || !information.allFinite() ||
      !information.isApprox(information.transpose(), 1e-9) ||
      !std::isfinite(residual_variance) || residual_variance < 0.0 ||
      !std::isfinite(config.rotation_length) || config.rotation_length <= 0.0 ||
      !std::isfinite(config.minimum_information) ||
      config.minimum_information <= 0.0 ||
      !std::isfinite(config.maximum_condition) ||
      config.maximum_condition < 1.0 ||
      !std::isfinite(config.position_variance_floor) ||
      config.position_variance_floor <= 0.0 ||
      !std::isfinite(config.rotation_variance_floor) ||
      config.rotation_variance_floor <= 0.0) {
    return {Reason::INVALID_INPUT,
            "invalid geometric information or qualified floors"};
  }
  Matrix6d scale = Matrix6d::Identity();
  scale.bottomRightCorner<3, 3>() *= config.rotation_length;
  const Matrix6d inverse_scale = scale.inverse();
  Eigen::SelfAdjointEigenSolver<Matrix6d> eigen(inverse_scale * information *
                                                inverse_scale);
  if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite() ||
      eigen.eigenvalues().minCoeff() < -1e-9) {
    return {Reason::INVALID_INPUT,
            "scaled geometric information decomposition failed"};
  }
  const double threshold =
      std::max(config.minimum_information,
               eigen.eigenvalues().maxCoeff() / config.maximum_condition);
  result->hessian_min_eigenvalue = eigen.eigenvalues().minCoeff();
  result->hessian_condition_number =
      eigen.eigenvalues().maxCoeff() /
      std::max(eigen.eigenvalues().minCoeff(),
               std::numeric_limits<double>::min());
  const int rank = (eigen.eigenvalues().array() >= threshold).count();
  if (rank == 0) {
    return {Reason::DEGENERATE, "no observable registration direction"};
  }
  result->projection.resize(rank, 6);
  result->projected_covariance = Eigen::MatrixXd::Zero(rank, rank);
  const double variance =
      std::max(residual_variance, config.position_variance_floor);
  int row = 0;
  for (int index = 0; index < 6; ++index) {
    if (eigen.eigenvalues()(index) < threshold) {
      continue;
    }
    auto direction = eigen.eigenvectors().col(index).eval();
    Eigen::Index dominant;
    direction.cwiseAbs().maxCoeff(&dominant);
    if (direction(dominant) < 0.0) {
      direction = -direction;
    }
    result->projection.row(row) = direction.transpose() * scale;
    result->projected_covariance(row, row) =
        variance / eigen.eigenvalues()(index);
    ++row;
  }
  Matrix6d floor = Matrix6d::Zero();
  floor.topLeftCorner<3, 3>().diagonal().setConstant(
      config.position_variance_floor);
  floor.bottomRightCorner<3, 3>().diagonal().setConstant(
      config.rotation_variance_floor);
  result->projected_covariance +=
      result->projection * floor * result->projection.transpose();
  const Result canonical = CanonicalizeObservedSubspace(
      result->projection, result->projected_covariance, config.rotation_length,
      &result->projection, &result->projected_covariance);
  if (!canonical.ok()) {
    return canonical;
  }
  result->full_pose = rank == 6;
  result->covariance.setZero();
  if (result->full_pose) {
    const Matrix6d inverse_projection = result->projection.inverse();
    result->covariance = inverse_projection * result->projected_covariance *
                         inverse_projection.transpose();
    if (!ValidCovariance(result->covariance)) {
      return {Reason::INVALID_INPUT, "registration covariance invalid"};
    }
  }
  return {};
}

Result MapMatcher::Match(const Cloud::ConstPtr& base_cloud,
                         const Eigen::Isometry3d& initial,
                         MatchResult* result) const {
  const auto started = std::chrono::steady_clock::now();
  const auto finish = [&started, result](Result status) {
    if (result != nullptr) {
      result->processing_time_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - started)
              .count();
    }
    return status;
  };
  if (!ready_) {
    return finish({Reason::MAP_NOT_READY, "map not loaded"});
  }
  if (result == nullptr || !base_cloud ||
      base_cloud->size() < config_.minimum_points ||
      !initial.matrix().allFinite()) {
    return finish({Reason::INVALID_INPUT, "invalid scan/initial pose"});
  }
  *result = MatchResult();
  result->source_points = static_cast<uint32_t>(base_cloud->size());
  if (config_.enable_icp_profiling &&
      config_.algorithm == RegistrationAlgorithm::kPointToPlaneIcp) {
    result->target_voxelization_ms = target_voxelization_ms_;
    result->target_kdtree_build_ms = target_kdtree_build_ms_;
    result->target_normal_estimation_ms = target_normal_estimation_ms_;
    result->target_normal_neighbor_search_ms =
        target_normal_neighbor_search_ms_;
    result->target_normal_fit_ms = target_normal_fit_ms_;
  }
  for (const auto& point : *base_cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      return finish({Reason::INVALID_INPUT, "nonfinite scan point"});
    }
  }
  Cloud::Ptr filtered(new Cloud);
  const Result downsampled =
      DownsampleAndLimit(base_cloud, filtered.get(), config_.source_voxel_size);
  if (!downsampled.ok()) {
    return finish(downsampled);
  }
  if (filtered->size() < config_.minimum_points) {
    return finish({Reason::MATCH_FAILED, "filtered scan is too sparse"});
  }
  result->registration_source_points = static_cast<uint32_t>(filtered->size());
  result->registration_target_points = static_cast<uint32_t>(map_->size());
  if (config_.algorithm == RegistrationAlgorithm::kPointToPlaneIcp) {
    return finish(MatchPointToPlaneIcp(filtered, initial, result));
  }
  registration_.setInputSource(filtered);
  Cloud aligned;
  registration_.align(aligned, initial.matrix().cast<float>());
  result->iterations = registration_.getFinalNumIteration();
  result->converged = registration_.hasConverged();
  const double score = registration_.getFitnessScore(
      config_.correspondence_distance * config_.correspondence_distance);
  if (!result->converged || !std::isfinite(score) ||
      score > config_.max_fitness) {
    return finish(
        {Reason::MATCH_FAILED, "NDT did not produce an accepted match"});
  }
  const Eigen::Matrix4d transform =
      registration_.getFinalTransformation().cast<double>();
  if (!transform.allFinite() ||
      !(transform.topLeftCorner<3, 3>().transpose() *
        transform.topLeftCorner<3, 3>())
           .isApprox(Eigen::Matrix3d::Identity(), 1e-4) ||
      std::abs(transform.topLeftCorner<3, 3>().determinant() - 1.0) > 1e-4) {
    return finish(
        {Reason::INVALID_INPUT, "NDT returned invalid rigid transform"});
  }
  result->pose = Eigen::Isometry3d(transform);
  result->pose.linear() =
      Eigen::Quaterniond(result->pose.linear()).normalized().toRotationMatrix();
  result->score = score;
  result->ambiguous = false;
  if ((result->pose.translation() - initial.translation()).norm() >
          config_.max_translation ||
      LogRotation(Eigen::Quaterniond(result->pose.linear() *
                                     initial.linear().transpose()))
              .norm() > config_.max_rotation) {
    return finish(
        {Reason::MATCH_FAILED, "match outside bounded convergence basin"});
  }
  result->score = score;
  return finish(Evidence(aligned, result));
}

Result MapMatcher::Recover(const Cloud::ConstPtr& base_cloud,
                           const std::vector<Eigen::Isometry3d>& seeds,
                           MatchResult* result) const {
  if (result == nullptr || seeds.empty() ||
      seeds.size() > config_.maximum_seeds) {
    return {Reason::INVALID_INPUT, "missing or excessive recovery hypotheses"};
  }
  std::vector<MatchResult> hypotheses;
  for (const auto& seed : seeds) {
    MatchResult hypothesis;
    if (!Match(base_cloud, seed, &hypothesis).ok() || !hypothesis.full_pose) {
      continue;
    }
    auto same = std::find_if(
        hypotheses.begin(), hypotheses.end(),
        [&hypothesis](const MatchResult& other) {
          return (other.pose.translation() - hypothesis.pose.translation())
                         .norm() < 0.5 &&
                 LogRotation(
                     Eigen::Quaterniond(other.pose.linear() *
                                        hypothesis.pose.linear().transpose()))
                         .norm() < 0.1;
        });
    if (same == hypotheses.end()) {
      hypotheses.push_back(hypothesis);
    } else if (hypothesis.score < same->score) {
      *same = hypothesis;
    }
  }
  if (hypotheses.empty()) {
    return {Reason::MATCH_FAILED, "all recovery hypotheses rejected"};
  }
  std::sort(hypotheses.begin(), hypotheses.end(),
            [](const MatchResult& a, const MatchResult& b) {
              return a.score < b.score;
            });
  *result = hypotheses.front();
  if (hypotheses.size() > 1 &&
      (hypotheses[1].score - result->score) / std::max(result->score, 1e-6) <
          config_.ambiguity_margin) {
    result->ambiguous = true;
    return {Reason::RELOCALIZATION_AMBIGUOUS,
            "distinct map matches have similar evidence"};
  }
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace openloc
