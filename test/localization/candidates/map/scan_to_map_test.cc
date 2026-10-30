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
#include "candidates/global/map_observation_adapter.h"

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

TEST(ScanToMapTest, StructuredRegistrationMatchesDenseReusedPointAndSeedErrors) {
  namespace u = unified;
  auto config = TestConfig();
  config.base_frame = config.cloud_frame = "body";
  config.algorithm = ScanMatchingAlgorithm::kPointToPlaneIcp;
  config.retain_joint_error_support = true;
  config.scan_voxel_size_m = 0.05;
  config.normal_neighbors = 6;
  config.minimum_points = config.icp_minimum_pairs = 20;
  config.local_window_radius_m = 10;
  config.maximum_scan_range_m = 10;
  PointCloud::Ptr reference(new PointCloud);
  for (int a = 0; a < 4; ++a)
    for (int b = 0; b < 4; ++b) {
      const double x = 0.2 * a + 0.03713, y = 0.2 * b + 0.04317;
      reference->emplace_back(x, y, 0.45119);
      reference->emplace_back(2.73713, x, y + 1.6);
      reference->emplace_back(x, 2.34317, y + 1.6);
    }
  ScanToMapLocalizer matcher(config);
  std::string error;
  ASSERT_TRUE(matcher.Configure(TestMetadata(), reference, &error)) << error;
  u::JointLidarScan dense;
  u::StructuredLidarScan structured;
  dense.reference = structured.reference = {1, 1, 7, "unix"};
  const Eigen::Index dimension = 15 + 3 * (reference->size() + 1);
  auto& state = dense.conditioning_state;
  state.valid = true;
  state.epoch = {"source-aware-map-test", 1};
  state.stamp = {1, 1, 9, "unix"};
  state.covariance = 0.01 * u::Matrix15d::Identity();
  Eigen::MatrixXd local_loading = Eigen::MatrixXd::Zero(15, dimension);
  local_loading.leftCols(15) = 0.1 * u::Matrix15d::Identity();
  state.joint_error = u::JointLocalError{
      Eigen::MatrixXd::Identity(dimension, dimension), local_loading};
  structured.conditioning_state.state = state;
  structured.conditioning_state.state.joint_error.reset();
  const auto local_error = u::JointErrorExpression::Primitive(
      std::make_shared<const int>(0), 0.1 * u::Matrix15d::Identity());
  structured.conditioning_state.output_error = local_error;
  Eigen::Matrix<double, 3, 15> shared =
      Eigen::Matrix<double, 3, 15>::Zero();
  shared.leftCols(3) = 0.2 * Eigen::Matrix3d::Identity();
  std::vector<u::JointErrorExpression> raw;
  for (std::size_t i = 0; i <= reference->size(); ++i) {
    const Eigen::Vector3d point = i == reference->size()
        ? Eigen::Vector3d(20, 20, 20)
        : Eigen::Vector3d((*reference)[i].x, (*reference)[i].y, (*reference)[i].z);
    dense.points_in_reference_base.push_back(point);
    structured.points_in_reference_base.push_back(point);
    Eigen::MatrixXd loading = shared * local_loading;
    loading.middleCols(15 + 3 * i, 3) = 0.01 * Eigen::Matrix3d::Identity();
    dense.point_jacobians.push_back(loading);
    raw.push_back(u::JointErrorExpression::Primitive(
        std::make_shared<const int>(0), 0.01 * Eigen::Matrix3d::Identity()));
    structured.point_errors.push_back(u::JointErrorExpression::Combine(
        3, {{shared, local_error}, {Eigen::Matrix3d::Identity(), raw.back()}}));
  }
  Eigen::Matrix<double, 6, 15> local_seed =
      Eigen::Matrix<double, 6, 15>::Zero();
  local_seed.leftCols(3).topRows(3).setIdentity();
  local_seed.block<3, 3>(3, 6).setIdentity();
  Eigen::Matrix<double, 6, 3> point_seed =
      Eigen::Matrix<double, 6, 3>::Constant(0.13);
  point_seed.topRows(3) += Eigen::Matrix3d::Identity();
  const Eigen::MatrixXd seed_loading =
      local_seed * local_loading +
      point_seed * dense.point_jacobians.front() -
      0.5 * point_seed * dense.point_jacobians[1];
  const auto seed = u::JointErrorExpression::Combine(
      6, {{local_seed, local_error},
          {point_seed, structured.point_errors.front()},
          {-0.5 * point_seed, structured.point_errors[1]}});
  Eigen::MatrixXd known_seed_cross;
  ASSERT_TRUE(seed.CrossCovariance(
      structured.point_errors.front(), &known_seed_cross).ok());
  EXPECT_TRUE(known_seed_cross.isApprox(
      seed_loading * dense.point_jacobians.front().transpose(), 1e-12));
  ScanToMapRequest request;
  request.timestamp_ns = 1'000'000'000;
  request.clock_domain = "unix";
  request.cloud_frame = "body";
  request.deskewed = true;
  request.joint_scan = &dense;
  request.initial_joint_jacobian = seed_loading;
  const auto dense_match = matcher.Match(request);
  ASSERT_TRUE(dense_match.valid) << dense_match.reason << ": " << dense_match.detail;
  ASSERT_TRUE(dense_match.joint_error);
  ASSERT_TRUE(dense_match.joint_error->assessment.ok())
      << dense_match.joint_error->assessment.message;
  request.joint_scan = nullptr;
  request.initial_joint_jacobian.resize(0, 0);
  request.structured_scan = &structured;
  request.initial_structured_error = seed;
  const auto match = matcher.Match(request);
  ASSERT_TRUE(match.valid) << match.reason << ": " << match.detail;
  ASSERT_TRUE(match.structured_error);
  ASSERT_TRUE(match.structured_error->assessment.ok())
      << match.structured_error->assessment.message;
  EXPECT_FALSE(match.joint_error);
  EXPECT_FALSE(match.covariance_calibrated);
  EXPECT_TRUE(match.diagnostic_only);
  EXPECT_EQ(match.source_points, reference->size());
  EXPECT_TRUE(match.map_from_base.matrix().isApprox(
      dense_match.map_from_base.matrix(), 0));
  EXPECT_TRUE(match.covariance.isApprox(dense_match.covariance, 0));
  const auto& model = *match.structured_error;
  const auto& oracle = *dense_match.joint_error;
  EXPECT_EQ(model.conditioning.sequence, state.stamp.sequence);
  EXPECT_EQ(model.source_sequence, dense.reference.sequence);
  EXPECT_LE((model.covariance - oracle.covariance).norm(), 1e-11);
  EXPECT_LE((model.local_cross_covariance -
             local_loading * oracle.local_cross_covariance).norm(), 1e-11);
  Eigen::MatrixXd cross;
  ASSERT_TRUE(model.pose_error.CrossCovariance(seed, &cross).ok());
  EXPECT_LE((cross - oracle.pose_jacobian * seed_loading.transpose()).norm(), 1e-11);
  for (std::size_t i = 0; i < structured.point_errors.size(); ++i) {
    ASSERT_TRUE(model.pose_error.CrossCovariance(
        structured.point_errors[i], &cross).ok());
    EXPECT_LE((cross - oracle.pose_jacobian *
                          dense.point_jacobians[i].transpose()).norm(), 1e-11);
  }
  const auto repeated = matcher.Match(request);
  ASSERT_TRUE(repeated.structured_error);
  ASSERT_TRUE(repeated.structured_error->assessment.ok());
  ASSERT_TRUE(model.pose_error.CrossCovariance(
      repeated.structured_error->pose_error, &cross).ok());
  EXPECT_TRUE(cross.isApprox(model.covariance, 1e-11));
  constexpr int64_t kUnixTime = 1791882562503163099LL;
  const double native_time = static_cast<double>(kUnixTime) / 1e9;
  ASSERT_NE(native_time, kUnixTime * 1e-9);
  request.timestamp_ns = kUnixTime;
  dense.reference.time = structured.reference.time = native_time;
  state.stamp.time = structured.conditioning_state.state.stamp.time = native_time;
  const auto unix_structured = matcher.Match(request);
  ASSERT_TRUE(unix_structured.valid) << unix_structured.reason;
  ASSERT_TRUE(unix_structured.structured_error->assessment.ok());
  request.timestamp_ns += 1000000;
  EXPECT_EQ(matcher.Match(request).reason,
            "invalid_structured_scan_or_seed_error_support");
  request.timestamp_ns = kUnixTime;
  request.structured_scan = nullptr;
  request.initial_structured_error = {};
  request.joint_scan = &dense;
  request.initial_joint_jacobian = seed_loading;
  ASSERT_TRUE(matcher.Match(request).valid);
  request.timestamp_ns = 1'000'000'000;
  dense.reference.time = structured.reference.time = 1;
  state.stamp.time = structured.conditioning_state.state.stamp.time = 1;
  request.joint_scan = nullptr;
  request.initial_joint_jacobian.resize(0, 0);
  request.structured_scan = &structured;
  request.initial_structured_error = seed;
  u::GlobalObservation sentinel;
  sentinel.source = "unchanged";
  EXPECT_EQ(u::AdaptScanToMapObservation(match, state, {}, {}, &sentinel).reason,
            u::Reason::CORRELATION_UNQUALIFIED);
  EXPECT_EQ(sentinel.source, "unchanged");
  request.initial_structured_error = {};
  EXPECT_EQ(matcher.Match(request).reason,
            "invalid_structured_scan_or_seed_error_support");
  request.initial_structured_error = seed;
  request.joint_scan = &dense;
  EXPECT_EQ(matcher.Match(request).reason,
            "invalid_structured_scan_or_seed_error_support");
  request.joint_scan = nullptr;
  structured.conditioning_state.state.covariance *= 2;
  EXPECT_EQ(matcher.Match(request).reason,
            "inconsistent_structured_local_error_support");
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
  config.calibration_id = "synthetic-calibration";
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
  EXPECT_EQ("synthetic-calibration", result.calibration_id);
  EXPECT_EQ("pcl_ndt_fitness", result.score_type);
  EXPECT_EQ("scan_matcher_residual_scaled_hessian", result.covariance_source);
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
  unified::LocalState local;
  local.stamp = {static_cast<double>(request.timestamp_ns) * 1e-9,
                 static_cast<double>(request.timestamp_ns) * 1e-9, 1,
                 request.clock_domain};
  local.epoch = {"scan-to-map-test", 1};
  local.valid = true;
  unified::MapObservationContract contract;
  contract.map_id = "synthetic-map";
  contract.map_version = "v1";
  contract.map_content_id = "content-hash";
  contract.map_frame = "map";
  contract.base_frame = "base_link";
  contract.calibration_id = "synthetic-calibration";
  unified::GlobalObservation observation;
  ASSERT_TRUE(unified::AdaptScanToMapObservation(
                  result, local, contract,
                  unified::MapObservationTiming{request.timestamp_ns, 1},
                  &observation)
                  .ok());
  EXPECT_EQ(result.geometry_full_rank
                ? unified::GlobalObservation::Kind::POSE
                : unified::GlobalObservation::Kind::PROJECTED_POSE,
            observation.kind);
  EXPECT_FALSE(observation.uncertainty_qualified);
  EXPECT_FALSE(observation.independent_of_local);
  EXPECT_EQ(result.observed_projection.rows(), observation.projection.rows());
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
  EXPECT_EQ("scan_matcher_residual_scaled_hessian", result.covariance_source);
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

TEST(ScanToMapTest, ReentryRequiresSupportedSeedAndDoesNotPoisonLaterMatches) {
  const PointCloud::Ptr map = AsymmetricMap();
  auto config = TestConfig();
  config.algorithm = ScanMatchingAlgorithm::kPointToPlaneIcp;
  config.icp_minimum_target_overlap = .25;
  ScanToMapLocalizer localizer(config);
  std::string error;
  ASSERT_TRUE(localizer.Configure(TestMetadata(), map, &error)) << error;
  const Eigen::Isometry3d truth =
      Eigen::Translation3d(.35, -.22, .18) *
      Eigen::AngleAxisd(.09, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-.04, Eigen::Vector3d::UnitY());
  ScanToMapRequest request;
  request.timestamp_ns = 10'000'000'000LL;
  request.clock_domain = "sensor_unix";
  request.cloud_frame = "lidar";
  request.deskewed = true;
  request.cloud = ScanInCloudFrame(*map, truth);
  for (const double offset : {0.0, .1, .5}) {
    SCOPED_TRACE(offset);
    request.map_from_base_initial = truth;
    request.map_from_base_initial.translation().x() += offset;
    const auto result = localizer.Match(request);
    ASSERT_TRUE(result.valid) << result.reason << " " << result.detail;
    EXPECT_LT((result.map_from_base.translation() - truth.translation()).norm(),
              10 * config.icp_translation_tolerance_m);
    EXPECT_LT(Eigen::AngleAxisd(truth.linear().transpose() *
                                result.map_from_base.linear())
                  .angle(),
              10 * config.icp_rotation_tolerance_rad);
    EXPECT_TRUE(result.diagnostic_only);
    EXPECT_FALSE(result.covariance_calibrated);
    request.timestamp_ns += 1'000'000'000LL;
  }
  request.map_from_base_initial.translation() = Eigen::Vector3d(1000, 1000, 0);
  request.timestamp_ns += 120'000'000'000LL;
  const auto outside = localizer.Match(request);
  EXPECT_FALSE(outside.valid);
  EXPECT_EQ(MatchStatus::kNotReady, outside.status);
  EXPECT_EQ("map_window_not_ready", outside.reason);
  request.map_from_base_initial = truth;
  request.timestamp_ns += 1'000'000'000LL;
  const auto returned = localizer.Match(request);
  ASSERT_TRUE(returned.valid) << returned.reason;
  EXPECT_LT((returned.map_from_base.translation() - truth.translation()).norm(),
            10 * config.icp_translation_tolerance_m);
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
