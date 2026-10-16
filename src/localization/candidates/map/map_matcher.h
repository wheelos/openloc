// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <string>
#include <vector>

#include "candidates/common/types.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/ndt.h"

namespace openloc {
namespace localization {
namespace unified {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

enum class RegistrationAlgorithm {
  kNdt,
  kPointToPlaneIcp,
};

struct MatchConfig {
  RegistrationAlgorithm algorithm = RegistrationAlgorithm::kNdt;
  double source_voxel_size = 0.5;
  // Zero inherits source_voxel_size.
  double target_voxel_size = 0.0;
  double resolution = 1.0;
  double step_size = 0.1;
  double transformation_epsilon = 0.001;
  uint32_t maximum_iterations = 40;
  double max_fitness = 0.5;
  double correspondence_distance = 1.0;
  double minimum_overlap = 0.5;
  // Diagnostic alternative: gate distance coverage separately from the
  // minimum planar count and projected-information rank. Default is unchanged.
  bool separate_ndt_support = false;
  double minimum_source_overlap = 0.4;
  double minimum_target_overlap = 0.25;
  uint32_t minimum_points = 100;
  double max_translation = 2.0;
  double max_rotation = 0.3;
  double minimum_information = 0.0001;
  double maximum_condition = 1000000.0;
  double position_variance_floor = 0.0;
  double rotation_variance_floor = 0.0;
  double ambiguity_margin = 0.2;
  uint32_t maximum_seeds = 16;
  double rotation_length = 1.0;
  uint32_t maximum_registration_points = 12000;
  uint32_t normal_neighbors = 15;
  double normal_planarity_ratio = 0.1;
  double icp_max_rmse = 0.2;
  double icp_condition_ratio = 1e-6;
  double icp_translation_tolerance = 0.001;
  double icp_rotation_tolerance = 0.0001;
  bool enable_icp_profiling = false;
};

struct MatchResult {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Matrix6d covariance = Matrix6d::Zero();
  Eigen::MatrixXd projection;
  Eigen::MatrixXd projected_covariance;
  bool full_pose = false;
  bool converged = false;
  double score = 0.0;
  double overlap = 0.0;
  double source_overlap = 0.0;
  double target_overlap = 0.0;
  double processing_time_ms = 0.0;
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
  bool ambiguous = false;
};

Result ProjectMapInformation(const Matrix6d& information,
                             double residual_variance,
                             const MatchConfig& config, MatchResult* result);

// A fixed, versioned global map is observation evidence, never the ODOM map.
class MapMatcher {
 public:
  explicit MapMatcher(const MatchConfig& config);
  MapMatcher(const MapMatcher&) = delete;
  MapMatcher& operator=(const MapMatcher&) = delete;
  Result Load(const std::string& pcd_path);
  Result SetMap(const Cloud::ConstPtr& cloud);
  Result Match(const Cloud::ConstPtr& base_cloud,
               const Eigen::Isometry3d& initial, MatchResult* result) const;
  Result Recover(const Cloud::ConstPtr& base_cloud,
                 const std::vector<Eigen::Isometry3d>& seeds,
                 MatchResult* result) const;
  bool ready() const { return ready_; }

 private:
  Result ValidateConfig() const;
  Result Evidence(const Cloud& cloud, MatchResult* result) const;
  Result MatchPointToPlaneIcp(const Cloud::ConstPtr& base_cloud,
                              const Eigen::Isometry3d& initial,
                              MatchResult* result) const;
  Result DownsampleAndLimit(const Cloud::ConstPtr& input, Cloud* output,
                            double voxel_size) const;
  MatchConfig config_;
  Cloud::Ptr map_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  enum class NormalFailure { kOther, kRank, kNonplanar };
  std::vector<NormalFailure> normal_failures_;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
  mutable pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>
      registration_;
  double target_voxelization_ms_ = 0.0;
  double target_kdtree_build_ms_ = 0.0;
  double target_normal_estimation_ms_ = 0.0;
  double target_normal_neighbor_search_ms_ = 0.0;
  double target_normal_fit_ms_ = 0.0;
  bool ready_ = false;
};

}  // namespace unified
}  // namespace localization
}  // namespace openloc
