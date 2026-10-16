// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "openloc/localization/map/scan_to_map.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "candidates/common/observed_subspace.h"
#include "candidates/common/types.h"
#include "candidates/map/map_matcher.h"

namespace openloc {
namespace localization {
namespace map {
namespace {

bool IsRigidTransform(const Eigen::Isometry3d& transform) {
  return transform.matrix().allFinite() &&
         (transform.linear().transpose() * transform.linear())
             .isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
         std::abs(transform.linear().determinant() - 1.0) <= 1e-6;
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d skew;
  skew << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(),
      -vector.y(), vector.x(), 0.0;
  return skew;
}

unified::MatchConfig ToMatchConfig(const ScanToMapConfig& config) {
  unified::MatchConfig result;
  result.algorithm = config.algorithm == ScanMatchingAlgorithm::kPointToPlaneIcp
                         ? unified::RegistrationAlgorithm::kPointToPlaneIcp
                         : unified::RegistrationAlgorithm::kNdt;
  result.source_voxel_size = config.scan_voxel_size_m;
  result.target_voxel_size = config.map_voxel_size_m > 0.0
                                 ? config.map_voxel_size_m
                                 : config.scan_voxel_size_m;
  result.resolution = config.ndt_resolution_m;
  result.step_size = config.ndt_step_size_m;
  result.transformation_epsilon = config.ndt_transformation_epsilon_m;
  result.maximum_iterations =
      config.algorithm == ScanMatchingAlgorithm::kPointToPlaneIcp
          ? config.icp_maximum_iterations
          : config.ndt_maximum_iterations;
  result.maximum_registration_points = config.maximum_registration_points;
  result.normal_neighbors = config.normal_neighbors;
  result.normal_planarity_ratio = config.normal_planarity_ratio;
  result.icp_max_rmse = config.icp_max_rmse_m;
  result.icp_condition_ratio = config.icp_condition_ratio;
  result.icp_translation_tolerance = config.icp_translation_tolerance_m;
  result.icp_rotation_tolerance = config.icp_rotation_tolerance_rad;
  result.enable_icp_profiling = config.enable_icp_profiling;
  result.max_fitness = config.maximum_fitness;
  result.correspondence_distance =
      config.algorithm == ScanMatchingAlgorithm::kPointToPlaneIcp
          ? config.icp_distance_m
          : config.correspondence_distance_m;
  result.minimum_overlap =
      config.algorithm == ScanMatchingAlgorithm::kPointToPlaneIcp
          ? std::min(config.icp_minimum_source_overlap,
                     config.icp_minimum_target_overlap)
          : config.minimum_overlap;
  result.minimum_source_overlap = config.icp_minimum_source_overlap;
  result.separate_ndt_support = config.separate_ndt_support;
  result.minimum_target_overlap = config.icp_minimum_target_overlap;
  result.minimum_points =
      config.algorithm == ScanMatchingAlgorithm::kPointToPlaneIcp
          ? config.icp_minimum_pairs
          : config.minimum_points;
  result.max_translation =
      config.algorithm == ScanMatchingAlgorithm::kPointToPlaneIcp
          ? config.icp_maximum_correction_translation_m
          : config.maximum_correction_translation_m;
  result.max_rotation =
      config.algorithm == ScanMatchingAlgorithm::kPointToPlaneIcp
          ? config.icp_maximum_correction_rotation_rad
          : config.maximum_correction_rotation_rad;
  result.minimum_information = config.minimum_information;
  result.maximum_condition = config.maximum_information_condition;
  result.position_variance_floor = config.position_variance_floor_m2;
  result.rotation_variance_floor = config.rotation_variance_floor_rad2;
  result.rotation_length = config.rotation_length_m;
  return result;
}

const char* MatchReason(unified::Reason reason) {
  switch (reason) {
    case unified::Reason::MAP_NOT_READY:
      return "map_not_ready";
    case unified::Reason::DEGENERATE:
      return "match_degenerate";
    case unified::Reason::INVALID_INPUT:
      return "invalid_match_input";
    case unified::Reason::MATCH_FAILED:
      return "match_failed";
    default:
      return "match_rejected";
  }
}

}  // namespace

struct ScanToMapLocalizer::Window {
  explicit Window(const unified::MatchConfig& config) : matcher(config) {}
  unified::MapMatcher matcher;
  uint64_t last_used = 0;
  uint32_t point_count = 0;
  double build_time_ms = 0.0;
};

ScanToMapLocalizer::ScanToMapLocalizer(const ScanToMapConfig& config)
    : config_(config) {}

ScanToMapLocalizer::~ScanToMapLocalizer() = default;

bool ScanToMapLocalizer::ValidateConfig(std::string* error) const {
  const auto invalid = [error](const char* message) {
    if (error != nullptr) {
      *error = message;
    }
    return false;
  };
  if (config_.algorithm != ScanMatchingAlgorithm::kPclNdt &&
      config_.algorithm != ScanMatchingAlgorithm::kPointToPlaneIcp) {
    return invalid("unknown scan matching algorithm");
  }
  if (config_.map_frame.empty() || config_.base_frame.empty() ||
      config_.cloud_frame.empty() ||
      !IsRigidTransform(config_.base_from_cloud)) {
    return invalid("frame names and rigid T_base_cloud are required");
  }
  const double positive_values[] = {
      config_.scan_voxel_size_m,
      config_.ndt_resolution_m,
      config_.ndt_step_size_m,
      config_.ndt_transformation_epsilon_m,
      config_.icp_distance_m,
      config_.icp_minimum_source_overlap,
      config_.icp_minimum_target_overlap,
      config_.normal_planarity_ratio,
      config_.icp_max_rmse_m,
      config_.icp_condition_ratio,
      config_.icp_translation_tolerance_m,
      config_.icp_rotation_tolerance_rad,
      config_.icp_maximum_correction_translation_m,
      config_.icp_maximum_correction_rotation_rad,
      config_.maximum_fitness,
      config_.correspondence_distance_m,
      config_.minimum_overlap,
      config_.maximum_correction_translation_m,
      config_.maximum_correction_rotation_rad,
      config_.minimum_information,
      config_.maximum_information_condition,
      config_.position_variance_floor_m2,
      config_.rotation_variance_floor_rad2,
      config_.rotation_length_m,
      config_.local_window_radius_m,
      config_.window_cell_size_m};
  for (double value : positive_values) {
    if (!std::isfinite(value) || value <= 0.0) {
      return invalid("matcher and local-window parameters must be positive");
    }
  }
  if (!std::isfinite(config_.map_voxel_size_m) ||
      config_.map_voxel_size_m < 0.0) {
    return invalid("map voxel size must be zero (inherit scan) or positive");
  }
  if (config_.minimum_overlap > 1.0 || config_.minimum_points < 10 ||
      config_.ndt_maximum_iterations == 0 ||
      config_.icp_maximum_iterations == 0 ||
      config_.maximum_registration_points < config_.icp_minimum_pairs ||
      config_.icp_minimum_pairs < 10 ||
      config_.icp_minimum_source_overlap > 1.0 ||
      config_.icp_minimum_target_overlap > 1.0 ||
      config_.normal_neighbors < 4 ||
      config_.normal_neighbors >
          static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
      config_.normal_planarity_ratio > 1.0 ||
      config_.icp_condition_ratio > 1.0 ||
      config_.maximum_cached_windows == 0 ||
      !config_.window_origin_xy_m.allFinite() ||
      !std::isfinite(config_.minimum_scan_range_m) ||
      !std::isfinite(config_.maximum_scan_range_m) ||
      config_.minimum_scan_range_m < 0.0 ||
      config_.maximum_scan_range_m < 0.0 ||
      (config_.maximum_scan_range_m > 0.0 &&
       config_.minimum_scan_range_m >= config_.maximum_scan_range_m)) {
    return invalid("invalid point, cache, overlap, or range budget");
  }
  return true;
}

bool ScanToMapLocalizer::Configure(const MapMetadata& metadata,
                                   PointCloud::ConstPtr cloud,
                                   std::string* error) {
  ready_ = false;
  windows_.clear();
  map_cloud_.reset();
  if (!ValidateConfig(error)) {
    return false;
  }
  if (metadata.map_id.empty() || metadata.version.empty() ||
      metadata.content_id.empty() || metadata.map_frame.empty() ||
      metadata.source_cloud_frame.empty() ||
      metadata.reference_state == MapReferenceState::kUnspecified ||
      !metadata.integrity_verified || metadata.map_frame != config_.map_frame ||
      !cloud || cloud->empty() ||
      cloud->size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
    if (error != nullptr) {
      *error =
          "map identity, verified integrity, frame, and points are required";
    }
    return false;
  }
  for (const auto& point : *cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      if (error != nullptr) {
        *error = "map contains a nonfinite point";
      }
      return false;
    }
  }
  metadata_ = metadata;
  map_cloud_.reset(new PointCloud(*cloud));
  PointCloud::Ptr xy_index(new PointCloud);
  xy_index->reserve(map_cloud_->size());
  for (const auto& point : *map_cloud_) {
    xy_index->emplace_back(point.x, point.y, 0.0f);
  }
  map_tree_.setInputCloud(xy_index);
  ready_ = true;
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

bool ScanToMapLocalizer::PrepareMapWindows(
    const std::vector<Eigen::Vector3d>& map_positions, std::string* error) {
  if (!ready_ || map_positions.empty()) {
    if (error != nullptr) {
      *error = "map must be ready and at least one window position is required";
    }
    return false;
  }
  std::vector<WindowKey> keys;
  keys.reserve(map_positions.size());
  const double cell_size = config_.window_cell_size_m;
  for (const auto& position : map_positions) {
    if (!position.allFinite()) {
      if (error != nullptr) {
        *error = "window positions must be finite";
      }
      return false;
    }
    const WindowKey key(
        static_cast<int64_t>(std::floor(
            (position.x() - config_.window_origin_xy_m.x()) / cell_size)),
        static_cast<int64_t>(std::floor(
            (position.y() - config_.window_origin_xy_m.y()) / cell_size)));
    if (std::find(keys.begin(), keys.end(), key) == keys.end()) {
      keys.push_back(key);
    }
  }
  if (keys.size() > config_.maximum_cached_windows) {
    if (error != nullptr) {
      *error = "window preparation exceeds the configured cache capacity";
    }
    return false;
  }
  for (const auto& key : keys) {
    const Eigen::Vector3d center(
        config_.window_origin_xy_m.x() +
            (static_cast<double>(key.first) + 0.5) * cell_size,
        config_.window_origin_xy_m.y() +
            (static_cast<double>(key.second) + 0.5) * cell_size,
        0.0);
    std::string window_error;
    if (GetWindow(center, &window_error) == nullptr) {
      if (error != nullptr) {
        *error = window_error;
      }
      return false;
    }
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

ScanToMapLocalizer::Window* ScanToMapLocalizer::GetWindow(
    const Eigen::Vector3d& map_position, std::string* error) {
  const double cell_size = config_.window_cell_size_m;
  const WindowKey key(
      static_cast<int64_t>(std::floor(
          (map_position.x() - config_.window_origin_xy_m.x()) / cell_size)),
      static_cast<int64_t>(std::floor(
          (map_position.y() - config_.window_origin_xy_m.y()) / cell_size)));
  auto found = windows_.find(key);
  if (found != windows_.end()) {
    found->second->last_used = ++use_sequence_;
    return found->second.get();
  }

  const bool profile =
      config_.enable_icp_profiling &&
      config_.algorithm == ScanMatchingAlgorithm::kPointToPlaneIcp;
  const auto window_build_started = std::chrono::steady_clock::now();
  const Eigen::Vector2d center(
      config_.window_origin_xy_m.x() +
          (static_cast<double>(key.first) + 0.5) * cell_size,
      config_.window_origin_xy_m.y() +
          (static_cast<double>(key.second) + 0.5) * cell_size);
  const double half_cell_diagonal = std::sqrt(2.0) * cell_size * 0.5;
  const double search_radius =
      config_.local_window_radius_m + half_cell_diagonal;
  PointCloud::Ptr xy_query(new PointCloud);
  xy_query->push_back(pcl::PointXYZ(static_cast<float>(center.x()),
                                    static_cast<float>(center.y()), 0.0f));
  std::vector<int> indices;
  std::vector<float> distances;
  map_tree_.radiusSearch(*xy_query->begin(), search_radius, indices, distances);

  PointCloud::Ptr local_map(new PointCloud);
  local_map->reserve(indices.size());
  const double radius_squared = search_radius * search_radius;
  for (int index : indices) {
    const auto& point = (*map_cloud_)[static_cast<size_t>(index)];
    const double dx = static_cast<double>(point.x) - center.x();
    const double dy = static_cast<double>(point.y) - center.y();
    if (dx * dx + dy * dy <= radius_squared) {
      local_map->push_back(point);
    }
  }
  if (local_map->size() < config_.minimum_points) {
    if (error != nullptr) {
      *error = "predicted map window has insufficient points";
    }
    return nullptr;
  }
  auto window = std::make_unique<Window>(ToMatchConfig(config_));
  const auto status = window->matcher.SetMap(local_map);
  if (!status.ok()) {
    if (error != nullptr) {
      *error = status.message;
    }
    return nullptr;
  }
  window->point_count = static_cast<uint32_t>(local_map->size());
  if (profile) {
    window->build_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - window_build_started)
            .count();
  }
  window->last_used = ++use_sequence_;
  Window* result = window.get();
  if (windows_.size() >= config_.maximum_cached_windows) {
    const auto oldest = std::min_element(
        windows_.begin(), windows_.end(), [](const auto& lhs, const auto& rhs) {
          return lhs.second->last_used < rhs.second->last_used;
        });
    windows_.erase(oldest);
  }
  windows_.emplace(key, std::move(window));
  return result;
}

ScanToMapResult ScanToMapLocalizer::Match(const ScanToMapRequest& request) {
  const auto started = std::chrono::steady_clock::now();
  ScanToMapResult output;
  const auto finish = [&started](ScanToMapResult result) {
    result.processing_time_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - started)
                                    .count();
    return result;
  };
  output.timestamp_ns = request.timestamp_ns;
  output.clock_domain = request.clock_domain;
  output.map_id = metadata_.map_id;
  output.map_version = metadata_.version;
  output.map_content_id = metadata_.content_id;
  output.map_frame = config_.map_frame;
  output.base_frame = config_.base_frame;
  output.score_type =
      config_.algorithm == ScanMatchingAlgorithm::kPointToPlaneIcp
          ? "point_to_plane_rmse_m"
          : "pcl_ndt_fitness";
  output.diagnostic_only =
      config_.separate_ndt_support ||
      metadata_.reference_state == MapReferenceState::kDiagnosticOnly ||
      !metadata_.physical_up_qualified || !metadata_.independent_reference;
  output.physical_up_qualified = metadata_.physical_up_qualified;
  output.independent_reference = metadata_.independent_reference;
  output.covariance_calibrated = false;
  output.calibration_state = CalibrationState::kUncalibrated;
  if (!ready_) {
    output.status = MatchStatus::kNotReady;
    output.reason = "map_not_ready";
    return finish(std::move(output));
  }
  if (request.timestamp_ns <= 0 || request.clock_domain.empty() ||
      request.cloud_frame != config_.cloud_frame || !request.deskewed ||
      !request.cloud || request.cloud->empty() ||
      request.cloud->size() >
          static_cast<size_t>(std::numeric_limits<uint32_t>::max()) ||
      !IsRigidTransform(request.map_from_base_initial)) {
    output.reason = "invalid_scan_timestamp_frame_or_initial_pose";
    return finish(std::move(output));
  }

  PointCloud::Ptr filtered(new PointCloud);
  filtered->reserve(request.cloud->size());
  const double minimum_squared =
      config_.minimum_scan_range_m * config_.minimum_scan_range_m;
  const double maximum_squared =
      config_.maximum_scan_range_m * config_.maximum_scan_range_m;
  for (const auto& point : *request.cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      output.reason = "scan_contains_nonfinite_point";
      return finish(std::move(output));
    }
    const double range_squared = static_cast<double>(point.x) * point.x +
                                 static_cast<double>(point.y) * point.y +
                                 static_cast<double>(point.z) * point.z;
    if ((config_.minimum_scan_range_m > 0.0 &&
         range_squared < minimum_squared) ||
        (config_.maximum_scan_range_m > 0.0 &&
         range_squared > maximum_squared)) {
      continue;
    }
    filtered->push_back(point);
  }
  output.source_points = static_cast<uint32_t>(filtered->size());
  const Eigen::Isometry3d map_from_cloud_initial =
      request.map_from_base_initial * config_.base_from_cloud;
  std::string window_error;
  Window* window =
      GetWindow(map_from_cloud_initial.translation(), &window_error);
  if (window == nullptr) {
    output.status = MatchStatus::kNotReady;
    output.reason = "map_window_not_ready";
    output.detail = window_error;
    return finish(std::move(output));
  }
  output.map_window_points = window->point_count;
  output.map_window_build_ms = window->build_time_ms;
  unified::MatchResult match;
  const unified::Result status =
      window->matcher.Match(filtered, map_from_cloud_initial, &match);
  output.converged = match.converged;
  output.ambiguous = match.ambiguous;
  output.score = match.score;
  output.overlap = match.overlap;
  output.source_overlap = match.source_overlap;
  output.target_overlap = match.target_overlap;
  output.processing_time_ms = match.processing_time_ms;
  output.target_voxelization_ms = match.target_voxelization_ms;
  output.target_kdtree_build_ms = match.target_kdtree_build_ms;
  output.target_normal_estimation_ms = match.target_normal_estimation_ms;
  output.target_normal_neighbor_search_ms =
      match.target_normal_neighbor_search_ms;
  output.target_normal_fit_ms = match.target_normal_fit_ms;
  output.icp_nearest_search_ms = match.icp_nearest_search_ms;
  output.icp_linearization_ms = match.icp_linearization_ms;
  output.icp_solver_ms = match.icp_solver_ms;
  output.hessian_min_eigenvalue = match.hessian_min_eigenvalue;
  output.hessian_condition_number = match.hessian_condition_number;
  output.iterations = match.iterations;
  output.source_points = match.source_points;
  output.matched_points = match.matched_points;
  output.registration_source_points = match.registration_source_points;
  output.registration_target_points = match.registration_target_points;
  output.ndt_support_available = match.ndt_support_available;
  output.ndt_distance_supported_points = match.ndt_distance_supported_points;
  output.ndt_normal_supported_points = match.ndt_normal_supported_points;
  output.ndt_rank_rejected_points = match.ndt_rank_rejected_points;
  output.ndt_nonplanar_rejected_points = match.ndt_nonplanar_rejected_points;
  output.ndt_other_normal_rejected_points =
      match.ndt_other_normal_rejected_points;
  if (!status.ok()) {
    output.reason = MatchReason(status.reason);
    output.detail = status.message;
    return finish(std::move(output));
  }
  const Eigen::Isometry3d map_from_base =
      match.pose * config_.base_from_cloud.inverse();
  const Eigen::Vector3d cloud_to_base_in_map =
      match.pose.linear() * config_.base_from_cloud.inverse().translation();
  Eigen::Matrix<double, 6, 6> cloud_to_base_jacobian =
      Eigen::Matrix<double, 6, 6>::Identity();
  cloud_to_base_jacobian.topRightCorner<3, 3>() = -Skew(cloud_to_base_in_map);
  const Eigen::Matrix<double, 6, 6> base_to_cloud_jacobian =
      cloud_to_base_jacobian.inverse();
  Eigen::MatrixXd base_projection = match.projection * base_to_cloud_jacobian;
  Eigen::MatrixXd base_projected_covariance;
  const unified::Result transformed_subspace =
      unified::CanonicalizeObservedSubspace(
          base_projection, match.projected_covariance,
          config_.rotation_length_m, &output.observed_projection,
          &base_projected_covariance);
  if (!transformed_subspace.ok()) {
    output.reason = "uncertainty_transform_failed";
    output.detail = transformed_subspace.message;
    return finish(std::move(output));
  }
  output.status = MatchStatus::kMatched;
  output.valid = true;
  output.map_from_base = map_from_base;
  output.correction_translation_m =
      (output.map_from_base.translation() -
       request.map_from_base_initial.translation())
          .norm();
  output.correction_rotation_rad =
      unified::LogRotation(
          Eigen::Quaterniond(
              output.map_from_base.linear() *
              request.map_from_base_initial.linear().transpose()))
          .norm();
  output.geometry_full_rank = match.full_pose;
  output.covariance = cloud_to_base_jacobian * match.covariance *
                      cloud_to_base_jacobian.transpose();
  output.observed_covariance = std::move(base_projected_covariance);
  output.observability_available = match.projection.rows() > 0;
  return finish(std::move(output));
}

bool ScanToMapLocalizer::ready() const { return ready_; }

size_t ScanToMapLocalizer::cached_window_count() const {
  return windows_.size();
}

size_t ScanToMapLocalizer::map_point_count() const {
  return map_cloud_ ? map_cloud_->size() : 0;
}

}  // namespace map
}  // namespace localization
}  // namespace openloc
