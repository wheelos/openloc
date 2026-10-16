// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace openloc {
namespace localization {
namespace map {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

enum class MapReferenceState {
  kUnspecified,
  kDiagnosticOnly,
  kQualified,
};

enum class MatchStatus {
  kMatched,
  kRejected,
  kNotReady,
};

enum class ScanMatchingAlgorithm {
  kPclNdt,
  kPointToPlaneIcp,
};

enum class CalibrationState {
  kUncalibrated,
  kCalibrated,
};

struct MapMetadata {
  std::string map_id;
  std::string version;
  std::string content_id;
  std::string map_frame;
  std::string source_cloud_frame;
  MapReferenceState reference_state = MapReferenceState::kUnspecified;
  // Set only after the host's map loader checks the release manifest/hashes.
  bool integrity_verified = false;
  bool physical_up_qualified = false;
  bool independent_reference = false;
};

struct ScanToMapConfig {
  std::string map_frame;
  std::string base_frame;
  std::string score_type = "pcl_ndt_fitness";
  ScanMatchingAlgorithm algorithm = ScanMatchingAlgorithm::kPclNdt;
  std::string cloud_frame;
  // Maps cloud-frame points into base_frame.
  Eigen::Isometry3d base_from_cloud = Eigen::Isometry3d::Identity();
  double scan_voxel_size_m = 0.5;
  // Zero keeps the legacy behavior by using scan_voxel_size_m.
  double map_voxel_size_m = 0.0;
  double ndt_resolution_m = 1.0;
  double ndt_step_size_m = 0.1;
  double ndt_transformation_epsilon_m = 0.001;
  uint32_t ndt_maximum_iterations = 40;
  uint32_t icp_maximum_iterations = 40;
  uint32_t maximum_registration_points = 12000;
  uint32_t normal_neighbors = 15;
  double normal_planarity_ratio = 0.1;
  double icp_distance_m = 1.5;
  uint32_t icp_minimum_pairs = 100;
  double icp_minimum_source_overlap = 0.4;
  double icp_minimum_target_overlap = 0.4;
  double icp_max_rmse_m = 0.2;
  double icp_condition_ratio = 1e-6;
  double icp_translation_tolerance_m = 0.001;
  double icp_rotation_tolerance_rad = 0.0001;
  double icp_maximum_correction_translation_m = 15.0;
  double icp_maximum_correction_rotation_rad = 1.0471975511965976;
  // Enables detailed ICP stage timers, which add measurement overhead.
  bool enable_icp_profiling = false;
  double maximum_fitness = 0.5;
  double correspondence_distance_m = 1.0;
  double minimum_overlap = 0.5;
  // Opt-in diagnostic NDT quality model; never changes ICP or planar minima.
  bool separate_ndt_support = false;
  uint32_t minimum_points = 100;
  double maximum_correction_translation_m = 2.0;
  double maximum_correction_rotation_rad = 0.3;
  double minimum_information = 0.0001;
  double maximum_information_condition = 1000000.0;
  double position_variance_floor_m2 = 0.01;
  double rotation_variance_floor_rad2 = 0.001;
  double rotation_length_m = 1.0;
  double minimum_scan_range_m = 0.0;
  double maximum_scan_range_m = 0.0;
  double local_window_radius_m = 40.0;
  double window_cell_size_m = 20.0;
  Eigen::Vector2d window_origin_xy_m = Eigen::Vector2d::Zero();
  uint32_t maximum_cached_windows = 8;
};

struct ScanToMapRequest {
  int64_t timestamp_ns = 0;
  std::string clock_domain;
  std::string cloud_frame;
  bool deskewed = false;
  PointCloud::ConstPtr cloud;
  Eigen::Isometry3d map_from_base_initial = Eigen::Isometry3d::Identity();
};

struct ScanToMapResult {
  MatchStatus status = MatchStatus::kRejected;
  // Stable machine-readable reason code; detail carries descriptive text.
  std::string reason;
  std::string detail;
  bool valid = false;
  CalibrationState calibration_state = CalibrationState::kUncalibrated;
  int64_t timestamp_ns = 0;
  std::string clock_domain;
  std::string map_id;
  std::string map_version;
  std::string map_content_id;
  std::string map_frame;
  std::string base_frame;
  Eigen::Isometry3d map_from_base = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  std::string score_type;
  bool covariance_calibrated = false;
  bool observability_available = false;
  Eigen::MatrixXd observed_projection;
  Eigen::MatrixXd observed_covariance;
  bool geometry_full_rank = false;
  bool diagnostic_only = true;
  bool physical_up_qualified = false;
  bool independent_reference = false;
  bool converged = false;
  bool ambiguous = false;
  double score = 0.0;
  double overlap = 0.0;
  double source_overlap = 0.0;
  double target_overlap = 0.0;
  double processing_time_ms = 0.0;
  double map_window_build_ms = 0.0;
  double target_voxelization_ms = 0.0;
  double target_kdtree_build_ms = 0.0;
  double target_normal_estimation_ms = 0.0;
  double target_normal_neighbor_search_ms = 0.0;
  double target_normal_fit_ms = 0.0;
  double icp_nearest_search_ms = 0.0;
  double icp_linearization_ms = 0.0;
  double icp_solver_ms = 0.0;
  double hessian_min_eigenvalue = 0.0;
  double hessian_condition_number = 0.0;
  double correction_translation_m = 0.0;
  double correction_rotation_rad = 0.0;
  uint32_t iterations = 0;
  uint32_t source_points = 0;
  uint32_t matched_points = 0;
  uint32_t registration_source_points = 0;
  uint32_t registration_target_points = 0;
  // NDT observed-support counts; unavailable before Evidence or on ICP.
  bool ndt_support_available = false;
  uint32_t ndt_distance_supported_points = 0;
  uint32_t ndt_normal_supported_points = 0;
  uint32_t ndt_rank_rejected_points = 0;
  uint32_t ndt_nonplanar_rejected_points = 0;
  uint32_t ndt_other_normal_rejected_points = 0;
  uint32_t map_window_points = 0;
};

// Standalone scan-to-map component. It emits observations only; it does not
// publish poses, own an estimator, or alter the caller's odometry. Calls must
// be serialized; each instance owns mutable registration and cache state.
class ScanToMapLocalizer {
 public:
  explicit ScanToMapLocalizer(const ScanToMapConfig& config);
  ~ScanToMapLocalizer();

  ScanToMapLocalizer(const ScanToMapLocalizer&) = delete;
  ScanToMapLocalizer& operator=(const ScanToMapLocalizer&) = delete;

  bool Configure(const MapMetadata& metadata, PointCloud::ConstPtr cloud,
                 std::string* error);
  bool PrepareMapWindows(const std::vector<Eigen::Vector3d>& map_positions,
                         std::string* error);
  ScanToMapResult Match(const ScanToMapRequest& request);
  bool ready() const;
  size_t cached_window_count() const;
  size_t map_point_count() const;

 private:
  struct Window;
  using WindowKey = std::pair<int64_t, int64_t>;

  bool ValidateConfig(std::string* error) const;
  Window* GetWindow(const Eigen::Vector3d& map_position, std::string* error);

  ScanToMapConfig config_;
  MapMetadata metadata_;
  PointCloud::ConstPtr map_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> map_tree_;
  std::map<WindowKey, std::unique_ptr<Window>> windows_;
  uint64_t use_sequence_ = 0;
  bool ready_ = false;
};

}  // namespace map
}  // namespace localization
}  // namespace openloc
