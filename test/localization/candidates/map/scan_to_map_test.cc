// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "openloc/localization/map/scan_to_map.h"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"

#include "Eigen/Cholesky"

namespace openloc {
namespace localization {
namespace map {
namespace {

ScanToMapConfig TestConfig() {
  ScanToMapConfig config;
  config.map_frame = "map";
  config.base_frame = "base_link";
  config.cloud_frame = "lidar";
  config.scan_voxel_size_m = 0.1;
  config.ndt_resolution_m = 0.5;
  config.maximum_fitness = 0.3;
  config.correspondence_distance_m = 0.8;
  config.minimum_overlap = 0.9;
  config.minimum_points = 20;
  config.maximum_correction_translation_m = 1.0;
  config.maximum_correction_rotation_rad = 0.3;
  config.position_variance_floor_m2 = 0.01;
  config.rotation_variance_floor_rad2 = 0.001;
  config.local_window_radius_m = 5.0;
  config.window_cell_size_m = 4.0;
  return config;
}

MapMetadata TestMetadata() {
  MapMetadata metadata;
  metadata.map_id = "synthetic-map";
  metadata.version = "v1";
  metadata.content_id = "content-hash";
  metadata.map_frame = "map";
  metadata.source_cloud_frame = "body";
  metadata.reference_state = MapReferenceState::kDiagnosticOnly;
  metadata.integrity_verified = true;
  return metadata;
}

PointCloud::Ptr AsymmetricMap() {
  PointCloud::Ptr cloud(new PointCloud);
  for (int x = 0; x <= 40; ++x) {
    for (int y = 0; y <= 30; ++y) {
      const double px = -4.0 + 0.2 * x;
      const double py = -3.0 + 0.2 * y;
      const double pz = 0.05 * px + 0.02 * py * py + 0.002 * px * py;
      cloud->emplace_back(static_cast<float>(px), static_cast<float>(py),
                          static_cast<float>(pz));
    }
  }
  for (int y = 0; y <= 20; ++y) {
    for (int z = 1; z <= 15; ++z) {
      const double py = -2.0 + 0.2 * y;
      const double pz = 0.2 * z;
      const double px = 3.5 + 0.03 * std::sin(py) + 0.01 * pz;
      cloud->emplace_back(static_cast<float>(px), static_cast<float>(py),
                          static_cast<float>(pz));
    }
  }
  for (int x = 0; x <= 25; ++x) {
    for (int z = 1; z <= 13; ++z) {
      const double px = -3.0 + 0.2 * x;
      const double pz = 0.2 * z;
      const double py = 3.1 + 0.025 * std::sin(px) + 0.015 * pz;
      cloud->emplace_back(static_cast<float>(px), static_cast<float>(py),
                          static_cast<float>(pz));
    }
  }
  return cloud;
}

PointCloud::Ptr ScanInCloudFrame(const PointCloud& map,
                                 const Eigen::Isometry3d& map_from_cloud) {
  PointCloud::Ptr scan(new PointCloud);
  scan->reserve(map.size());
  const Eigen::Isometry3d cloud_from_map = map_from_cloud.inverse();
  for (const auto& point : map) {
    const Eigen::Vector3d transformed =
        cloud_from_map * Eigen::Vector3d(point.x, point.y, point.z);
    scan->emplace_back(static_cast<float>(transformed.x()),
                       static_cast<float>(transformed.y()),
                       static_cast<float>(transformed.z()));
  }
  return scan;
}

Eigen::Matrix<double, 6, 6> SubspaceInformation(
    const Eigen::MatrixXd& projection, const Eigen::MatrixXd& covariance) {
  return projection.transpose() * covariance.ldlt().solve(projection);
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d skew;
  skew << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(),
      -vector.y(), vector.x(), 0.0;
  return skew;
}

TEST(ScanToMapTest, SeparateSupportCannotBecomeQualifiedFromMapMetadata) {
  auto config = TestConfig();
  config.separate_ndt_support = true;
  ScanToMapLocalizer localizer(config);
  auto metadata = TestMetadata();
  metadata.reference_state = MapReferenceState::kQualified;
  metadata.physical_up_qualified = true;
  metadata.independent_reference = true;
  std::string error;
  ASSERT_TRUE(localizer.Configure(metadata, AsymmetricMap(), &error)) << error;
  // The diagnostic label also survives an early invalid-input return.
  EXPECT_TRUE(localizer.Match(ScanToMapRequest()).diagnostic_only);
}

TEST(ScanToMapTest, RequiresVerifiedMapIdentityAndMatchingFrames) {
  ScanToMapLocalizer localizer(TestConfig());
  auto map = AsymmetricMap();
  auto metadata = TestMetadata();
  std::string error;

  metadata.integrity_verified = false;
  EXPECT_FALSE(localizer.Configure(metadata, map, &error));
  EXPECT_FALSE(error.empty());

  metadata = TestMetadata();
  metadata.map_frame = "other_map";
  EXPECT_FALSE(localizer.Configure(metadata, map, &error));
  EXPECT_FALSE(localizer.ready());
}

TEST(ScanToMapTest, RejectsInvalidMatcherResolutionConfiguration) {
  const PointCloud::Ptr map = AsymmetricMap();
  std::string error;

  ScanToMapConfig config = TestConfig();
  config.map_voxel_size_m = -0.1;
  ScanToMapLocalizer invalid_map_voxel(config);
  EXPECT_FALSE(invalid_map_voxel.Configure(TestMetadata(), map, &error));

  config = TestConfig();
  config.ndt_maximum_iterations = 0;
  ScanToMapLocalizer invalid_iterations(config);
  EXPECT_FALSE(invalid_iterations.Configure(TestMetadata(), map, &error));
}

TEST(ScanToMapTest, EmitsTimestampedDiagnosticMatchFromBoundedMapWindow) {
  const PointCloud::Ptr source = AsymmetricMap();
  auto map = PointCloud::Ptr(new PointCloud(*source));
  const size_t local_points = source->size();
  for (const auto& point : *source) {
    map->emplace_back(point.x + 40.0f, point.y + 40.0f, point.z);
  }
  ASSERT_GT(map->size(), local_points);

  ScanToMapConfig config = TestConfig();
  const Eigen::Isometry3d base_from_cloud =
      Eigen::Translation3d(0.2, -0.1, 0.05) *
      Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ());
  config.base_from_cloud = base_from_cloud;
  ScanToMapLocalizer localizer(config);
  std::string error;
  ASSERT_TRUE(localizer.Configure(TestMetadata(), map, &error)) << error;
  EXPECT_EQ(map->size(), localizer.map_point_count());
  EXPECT_EQ(0U, localizer.cached_window_count());

  const Eigen::Isometry3d map_from_base =
      Eigen::Translation3d(0.35, -0.22, 0.18) *
      Eigen::AngleAxisd(0.09, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX());
  const Eigen::Isometry3d map_from_cloud = map_from_base * base_from_cloud;
  const PointCloud::Ptr scan = ScanInCloudFrame(*source, map_from_cloud);

  ScanToMapRequest request;
  request.timestamp_ns = 1791440875499901772LL;
  request.clock_domain = "sensor_unix";
  request.cloud_frame = "lidar";
  request.deskewed = true;
  request.cloud = scan;
  request.map_from_base_initial = map_from_base;

  const ScanToMapResult result = localizer.Match(request);
  ASSERT_EQ(MatchStatus::kMatched, result.status) << result.reason;
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.diagnostic_only);
  EXPECT_EQ(CalibrationState::kUncalibrated, result.calibration_state);
  EXPECT_FALSE(result.covariance_calibrated);
  EXPECT_EQ(request.timestamp_ns, result.timestamp_ns);
  EXPECT_EQ(request.clock_domain, result.clock_domain);
  EXPECT_EQ("synthetic-map", result.map_id);
  EXPECT_EQ("v1", result.map_version);
  EXPECT_EQ("map", result.map_frame);
  EXPECT_EQ("base_link", result.base_frame);
  EXPECT_EQ("pcl_ndt_fitness", result.score_type);
  EXPECT_LT(result.map_window_points, map->size());
  EXPECT_EQ(scan->size(), result.source_points);
  EXPECT_GE(result.matched_points, config.minimum_points);
  ASSERT_TRUE(result.ndt_support_available);
  EXPECT_GE(result.ndt_distance_supported_points,
            result.ndt_normal_supported_points);
  EXPECT_EQ(result.ndt_normal_supported_points, result.matched_points);
  EXPECT_EQ(result.ndt_distance_supported_points,
            result.ndt_normal_supported_points +
                result.ndt_rank_rejected_points +
                result.ndt_nonplanar_rejected_points +
                result.ndt_other_normal_rejected_points);
  EXPECT_GT(result.registration_target_points, 0U);
  EXPECT_DOUBLE_EQ(result.overlap,
                   static_cast<double>(result.ndt_normal_supported_points) /
                       result.registration_source_points);
  EXPECT_TRUE(std::isfinite(result.processing_time_ms));
  EXPECT_TRUE(result.converged);
  EXPECT_TRUE(result.observability_available);
  EXPECT_LT(
      (result.map_from_base.translation() - map_from_base.translation()).norm(),
      0.12);
  EXPECT_LT((result.map_from_base.translation() -
             request.map_from_base_initial.translation())
                .norm(),
            config.maximum_correction_translation_m);
  EXPECT_EQ(local_points * 2, map->size());
  EXPECT_EQ(1U, localizer.cached_window_count());
  EXPECT_EQ(MatchStatus::kMatched, localizer.Match(request).status);
  EXPECT_EQ(1U, localizer.cached_window_count());
}

TEST(ScanToMapTest, SelectsPointToPlaneIcpBackendAndReportsMetricType) {
  const PointCloud::Ptr map = AsymmetricMap();
  ScanToMapConfig config = TestConfig();
  config.algorithm = ScanMatchingAlgorithm::kPointToPlaneIcp;
  config.enable_icp_profiling = true;
  config.icp_distance_m = 1.5;
  config.icp_minimum_source_overlap = 0.4;
  config.icp_minimum_target_overlap = 0.25;
  ScanToMapLocalizer localizer(config);
  std::string error;
  ASSERT_TRUE(localizer.Configure(TestMetadata(), map, &error)) << error;

  const Eigen::Isometry3d map_from_cloud =
      Eigen::Translation3d(0.35, -0.22, 0.18) *
      Eigen::AngleAxisd(0.09, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX());
  ScanToMapRequest request;
  request.timestamp_ns = 1791440875499901772LL;
  request.clock_domain = "sensor_unix";
  request.cloud_frame = "lidar";
  request.deskewed = true;
  request.cloud = ScanInCloudFrame(*map, map_from_cloud);
  request.map_from_base_initial = map_from_cloud;

  const ScanToMapResult result = localizer.Match(request);

  ASSERT_EQ(MatchStatus::kMatched, result.status) << result.reason;
  EXPECT_TRUE(result.converged);
  EXPECT_EQ("point_to_plane_rmse_m", result.score_type);
  EXPECT_GE(result.source_overlap, config.icp_minimum_source_overlap);
  EXPECT_GE(result.target_overlap, config.icp_minimum_target_overlap);
  EXPECT_FALSE(result.covariance_calibrated);
  EXPECT_EQ(CalibrationState::kUncalibrated, result.calibration_state);
  EXPECT_GT(result.map_window_build_ms, 0.0);
  EXPECT_GT(result.target_voxelization_ms, 0.0);
  EXPECT_GT(result.target_kdtree_build_ms, 0.0);
  EXPECT_GT(result.target_normal_estimation_ms, 0.0);
  EXPECT_GT(result.target_normal_neighbor_search_ms, 0.0);
  EXPECT_GT(result.target_normal_fit_ms, 0.0);
  EXPECT_GT(result.icp_nearest_search_ms, 0.0);
  EXPECT_GT(result.icp_linearization_ms, 0.0);
  EXPECT_GT(result.icp_solver_ms, 0.0);
}

TEST(ScanToMapTest, TransformsUncertaintyThroughNonzeroCloudExtrinsic) {
  const PointCloud::Ptr map = AsymmetricMap();
  const Eigen::Isometry3d map_from_cloud =
      Eigen::Translation3d(0.35, -0.22, 0.18) *
      Eigen::AngleAxisd(0.09, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX());
  const PointCloud::Ptr scan = ScanInCloudFrame(*map, map_from_cloud);

  ScanToMapLocalizer cloud_localizer(TestConfig());
  std::string error;
  ASSERT_TRUE(cloud_localizer.Configure(TestMetadata(), map, &error)) << error;
  ScanToMapRequest cloud_request;
  cloud_request.timestamp_ns = 1791440875499901772LL;
  cloud_request.clock_domain = "sensor_unix";
  cloud_request.cloud_frame = "lidar";
  cloud_request.deskewed = true;
  cloud_request.cloud = scan;
  cloud_request.map_from_base_initial = map_from_cloud;
  const ScanToMapResult cloud_result = cloud_localizer.Match(cloud_request);
  ASSERT_EQ(MatchStatus::kMatched, cloud_result.status) << cloud_result.detail;

  ScanToMapConfig base_config = TestConfig();
  base_config.base_from_cloud =
      Eigen::Translation3d(0.7, -0.4, 0.2) *
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitZ());
  ScanToMapLocalizer base_localizer(base_config);
  ASSERT_TRUE(base_localizer.Configure(TestMetadata(), map, &error)) << error;
  ScanToMapRequest base_request = cloud_request;
  base_request.map_from_base_initial =
      map_from_cloud * base_config.base_from_cloud.inverse();
  const ScanToMapResult base_result = base_localizer.Match(base_request);
  ASSERT_EQ(MatchStatus::kMatched, base_result.status) << base_result.detail;

  const Eigen::Isometry3d expected_map_from_base =
      cloud_result.map_from_base * base_config.base_from_cloud.inverse();
  EXPECT_TRUE(base_result.map_from_base.matrix().isApprox(
      expected_map_from_base.matrix(), 1e-5));

  const Eigen::Vector3d cloud_to_base_in_map =
      cloud_result.map_from_base.linear() *
      base_config.base_from_cloud.inverse().translation();
  Eigen::Matrix<double, 6, 6> cloud_to_base_jacobian =
      Eigen::Matrix<double, 6, 6>::Identity();
  cloud_to_base_jacobian.topRightCorner<3, 3>() = -Skew(cloud_to_base_in_map);
  const Eigen::Matrix<double, 6, 6> expected_covariance =
      cloud_to_base_jacobian * cloud_result.covariance *
      cloud_to_base_jacobian.transpose();
  EXPECT_TRUE(base_result.covariance.isApprox(expected_covariance, 1e-5));

  const Eigen::Matrix<double, 6, 6> expected_information =
      cloud_to_base_jacobian.inverse().transpose() *
      SubspaceInformation(cloud_result.observed_projection,
                          cloud_result.observed_covariance) *
      cloud_to_base_jacobian.inverse();
  EXPECT_TRUE(SubspaceInformation(base_result.observed_projection,
                                  base_result.observed_covariance)
                  .isApprox(expected_information, 1e-5));

  EXPECT_GT(base_result.covariance(1, 1), cloud_result.covariance(1, 1));
  EXPECT_NEAR(base_result.covariance(1, 5), expected_covariance(1, 5), 1e-5);
}

TEST(ScanToMapTest, PreparesEachBoundedLocalWindowOnlyOnce) {
  ScanToMapConfig config = TestConfig();
  config.window_cell_size_m = 4.0;
  config.maximum_cached_windows = 2;
  ScanToMapLocalizer localizer(config);
  std::string error;
  ASSERT_TRUE(localizer.Configure(TestMetadata(), AsymmetricMap(), &error))
      << error;

  const std::vector<Eigen::Vector3d> positions = {
      Eigen::Vector3d(0.1, 0.1, 0.0), Eigen::Vector3d(1.0, 1.0, 0.0),
      Eigen::Vector3d(4.1, 0.1, 0.0)};
  ASSERT_TRUE(localizer.PrepareMapWindows(positions, &error)) << error;
  EXPECT_EQ(2U, localizer.cached_window_count());

  const std::vector<Eigen::Vector3d> too_many_positions = {
      Eigen::Vector3d(-8.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(8.0, 0.0, 0.0)};
  EXPECT_FALSE(localizer.PrepareMapWindows(too_many_positions, &error));
  EXPECT_EQ("window preparation exceeds the configured cache capacity", error);
  EXPECT_EQ(2U, localizer.cached_window_count());
}

TEST(ScanToMapTest, RejectsWrongScanFrameAndReportsMapOutsideWindow) {
  ScanToMapLocalizer localizer(TestConfig());
  const PointCloud::Ptr map = AsymmetricMap();
  std::string error;
  ASSERT_TRUE(localizer.Configure(TestMetadata(), map, &error)) << error;

  ScanToMapRequest request;
  request.timestamp_ns = 1;
  request.clock_domain = "sensor";
  request.cloud_frame = "unexpected_lidar";
  request.deskewed = true;
  request.cloud = map;
  EXPECT_EQ(MatchStatus::kRejected, localizer.Match(request).status);

  request.cloud_frame = "lidar";
  request.deskewed = false;
  EXPECT_EQ(MatchStatus::kRejected, localizer.Match(request).status);
  request.deskewed = true;
  request.timestamp_ns = 0;
  EXPECT_EQ(MatchStatus::kRejected, localizer.Match(request).status);

  request.timestamp_ns = 1;
  request.map_from_base_initial.translation() = Eigen::Vector3d(1000, 1000, 0);
  const ScanToMapResult outside = localizer.Match(request);
  EXPECT_EQ(MatchStatus::kNotReady, outside.status);
  EXPECT_EQ("map_window_not_ready", outside.reason);
  EXPECT_EQ("predicted map window has insufficient points", outside.detail);
}

TEST(ScanToMapTest, AppliesRangeFilterAndRejectsNonfiniteInput) {
  ScanToMapConfig config = TestConfig();
  config.minimum_scan_range_m = 0.5;
  config.maximum_scan_range_m = 10.0;
  ScanToMapLocalizer localizer(config);
  const PointCloud::Ptr map = AsymmetricMap();
  std::string error;
  ASSERT_TRUE(localizer.Configure(TestMetadata(), map, &error)) << error;

  PointCloud::Ptr scan(new PointCloud(*map));
  scan->push_back(pcl::PointXYZ(0.1f, 0.0f, 0.0f));
  size_t in_range_points = 0;
  for (const auto& point : *map) {
    const double range =
        std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (range >= config.minimum_scan_range_m &&
        range <= config.maximum_scan_range_m) {
      ++in_range_points;
    }
  }
  ScanToMapRequest request;
  request.timestamp_ns = 2;
  request.clock_domain = "sensor";
  request.cloud_frame = "lidar";
  request.deskewed = true;
  request.cloud = scan;
  const ScanToMapResult result = localizer.Match(request);
  EXPECT_EQ(MatchStatus::kMatched, result.status) << result.reason;
  EXPECT_EQ(in_range_points, result.source_points);

  PointCloud::Ptr invalid(new PointCloud(*map));
  (*invalid)[0].x = std::numeric_limits<float>::quiet_NaN();
  request.cloud = invalid;
  EXPECT_EQ("scan_contains_nonfinite_point", localizer.Match(request).reason);
}

}  // namespace
}  // namespace map
}  // namespace localization
}  // namespace openloc
