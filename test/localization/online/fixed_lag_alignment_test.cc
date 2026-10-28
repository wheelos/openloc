// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "openloc/localization/global/fixed_lag_alignment.h"
#include "openloc/localization/local/native_odometry.h"
#include "openloc/localization/map/scan_to_map.h"

#include <cmath>
#include <limits>

#include "Eigen/Cholesky"
#include "gtest/gtest.h"

namespace openloc {
namespace localization {
namespace {

constexpr int64_t kStart = 10'000'000'000LL;
constexpr int64_t kStep = 10'000'000LL;
using Matrix15d = Eigen::Matrix<double, 15, 15>;

FixedLagAlignmentConfig Config() {
  FixedLagAlignmentConfig config;
  config.reference = {"site", "v1", "calibration"};
  auto& geodetic = config.reference.geodetic;
  geodetic.state = GeodeticReferenceState::kVerified;
  geodetic.horizontal_crs_realization = "TEST:synthetic-crs";
  geodetic.coordinate_epoch_decimal_year = 2026.0;
  geodetic.vertical_datum = "TEST:synthetic-vertical";
  geodetic.enu_origin_latitude_deg = 0.0;
  geodetic.enu_origin_longitude_deg = 0.0;
  geodetic.enu_origin_height_m = 0.0;
  config.map_content_id = "synthetic-map";
  config.map_frame = {"map"};
  config.odom_frame = {"odom"};
  config.base_frame = {"body"};
  config.motion_error_model_id = "synthetic-markov-chain";
  config.motion_error_model_qualified = true;
  // These synthetic contracts do not establish deployment qualification.
  config.solve_budget_ms = 100000.0;
  return config;
}

GraphSourceQualification Qualification() {
  GraphSourceQualification qualification;
  qualification.uncertainty_model_id = "synthetic-independent-noise";
  qualification.independence_evidence_id = "synthetic-disjoint-sources";
  qualification.quality_valid = true;
  qualification.uncertainty_qualified = true;
  qualification.independent_of_local = true;
  qualification.independent_of_graph_sources = true;
  return qualification;
}

GraphOdometrySample Odom(int index) {
  GraphOdometrySample sample;
  sample.odometry.timestamp = {kStart + kStep * index, ClockDomain::kUnixEpoch};
  sample.odometry.epoch = {"local-producer", 1};
  sample.odometry.parent_frame = {"odom"};
  sample.odometry.child_frame = {"body"};
  sample.odometry.pose.translation.x() = 0.1 * index;
  sample.receive_time = sample.odometry.timestamp;
  sample.sequence = index + 1;
  sample.covariance = Matrix15d::Identity() * (0.01 + 0.001 * index);
  sample.covariance_model_valid = true;
  sample.uncertainty_qualified = true;
  return sample;
}

GraphMotionConnection Connection(int end_index) {
  GraphMotionConnection connection;
  connection.epoch = Odom(end_index).odometry.epoch;
  connection.start_time = Odom(end_index - 1).odometry.timestamp;
  connection.end_time = Odom(end_index).odometry.timestamp;
  connection.start_sequence = Odom(end_index - 1).sequence;
  connection.end_sequence = Odom(end_index).sequence;
  connection.cross_covariance = Odom(end_index - 1).covariance;
  connection.sources = {{"imu", Odom(end_index - 1).sequence},
                        {"imu", Odom(end_index).sequence}};
  return connection;
}

GlobalObservationContext Context(int index, const std::string& source) {
  const auto config = Config();
  GlobalObservationContext context;
  context.measurement_time = Odom(index).odometry.timestamp;
  context.receive_time = context.measurement_time;
  context.sequence = index + 1;
  context.source = source;
  context.epoch = Odom(index).odometry.epoch;
  context.reference = config.reference;
  context.map_content_id = config.map_content_id;
  context.map_frame = config.map_frame;
  context.base_frame = config.base_frame;
  return context;
}

GlobalMapObservation Map(int index) {
  GlobalMapObservation observation;
  observation.context = Context(index, "survey-map");
  observation.map_from_base.translation =
      Eigen::Vector3d(100.0 + 0.1 * index, 20.0, 0.0);
  observation.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 0.001;
  observation.covariance_source = "synthetic-map-noise";
  return observation;
}

void Bootstrap(FixedLagGlobalAlignment* owner) {
  ASSERT_TRUE(owner->AddOdometry(Odom(0)).ok());
  for (int index = 0; index < 3; ++index) {
    if (index != 0) {
      const auto connection = Connection(index);
      ASSERT_TRUE(owner->AddOdometry(Odom(index), &connection).ok());
    }
    const auto status =
        owner->SubmitMapObservation(Map(index), Qualification());
    if (index < 2) {
      EXPECT_EQ(StatusCode::kRejectedMeasurement, status.code());
    } else {
      ASSERT_TRUE(status.ok()) << status.message();
    }
  }
}

GlobalHeadingObservation Heading(int index) {
  GlobalHeadingObservation heading;
  heading.context = Context(index, "dual-antenna-heading");
  heading.variance_rad2 = 0.0001;
  heading.covariance_source = "synthetic-baseline-azimuth";
  return heading;
}

TEST(FixedLagAlignmentTest, PositionHeadingAcquiresOnlyDiagnosticHorizontalAxes) {
  auto config = Config();
  config.enable_heading_bootstrap = true;
  FixedLagGlobalAlignment invalid;
  EXPECT_EQ(StatusCode::kInvalidArgument, invalid.Configure(config).code());
  config.assume_gravity_aligned_odom = true;
  for (const bool heading_first : {false, true}) {
    FixedLagGlobalAlignment owner;
    ASSERT_TRUE(owner.Configure(config).ok());
    const auto odom = Odom(0);
    ASSERT_TRUE(owner.AddOdometry(odom).ok());
    auto heading = Heading(0);
    heading.direction_in_base = Eigen::Vector3d::UnitY();
    heading.heading_rad = 0.4 + std::acos(-1.0) / 2.0;
    GlobalHorizontalObservation position;
    position.context = Context(0, "receiver-position");
    position.point_in_base = Eigen::Vector3d(1.0, -0.5, 0.3);
    position.position = Eigen::Vector2d(100.0, 20.0) +
        Eigen::Rotation2Dd(0.4) * position.point_in_base.head<2>();
    position.covariance = Eigen::Matrix2d::Identity() * 0.01;
    position.covariance_source = "synthetic-receiver-position";
    auto qualification = Qualification();
    // The diagnostic bound does not pretend these receiver messages are
    // independent of one another or of the local state.
    qualification.independent_of_local = false;
    qualification.independent_of_graph_sources = false;
    qualification.independence_evidence_id.clear();
    ASSERT_TRUE((heading_first
        ? owner.SubmitHeadingObservation(heading, qualification)
        : owner.SubmitHorizontalObservation(position, qualification)).ok());
    FixedLagAlignmentResult result;
    ASSERT_TRUE(owner.GetResult(odom.odometry.timestamp, &result).ok());
    EXPECT_FALSE(result.estimate);
    EXPECT_FALSE(result.horizontal_estimate);
    ASSERT_TRUE((heading_first
        ? owner.SubmitHorizontalObservation(position, qualification)
        : owner.SubmitHeadingObservation(heading, qualification)).ok());
    ASSERT_TRUE(owner.GetResult(odom.odometry.timestamp, &result).ok());
    ASSERT_TRUE(result.horizontal_estimate);
    EXPECT_FALSE(result.estimate);
    EXPECT_FALSE(result.full_pose_evidence_fresh);
    EXPECT_TRUE(result.horizontal_evidence_fresh);
    EXPECT_TRUE(result.diagnostic_only);
    EXPECT_FALSE(result.assessment.ok());
    EXPECT_EQ(LocalizationTrustLevel::kDiagnostic, result.trust_level);
    EXPECT_TRUE(result.horizontal_estimate->map_base_xy.isApprox(
        Eigen::Vector2d(100.0, 20.0), 1e-9));
    EXPECT_NEAR(0.4, result.horizontal_estimate->map_from_odom_xy_yaw.z(), 1e-9);
    EXPECT_EQ(1u, result.horizontal_estimate->correction_id);
    EXPECT_TRUE(result.horizontal_estimate->covariance.llt().info() ==
                Eigen::Success);
    EXPECT_TRUE(odom.odometry.pose.translation.isZero());
    std::vector<GraphObservationDecision> decisions;
    ASSERT_TRUE(owner.GetDecisions(&decisions).ok());
    ASSERT_EQ(2u, decisions.size());
    EXPECT_FALSE(decisions.front().observation.alignment_updated);
    EXPECT_TRUE(decisions.back().observation.alignment_updated);
    EXPECT_EQ(1u, decisions.back().observation.correction_id);
    const Timestamp late{odom.odometry.timestamp.nanoseconds +
        config.maximum_full_pose_age_ns + 1, config.clock_domain};
    ASSERT_TRUE(owner.GetResult(late, &result).ok());
    EXPECT_FALSE(result.horizontal_evidence_fresh);
    EXPECT_FALSE(result.full_pose_evidence_fresh);
    EXPECT_EQ(LocalizationTrustLevel::kUntrusted, result.trust_level);
  }
}

TEST(FixedLagAlignmentTest, HeadingUpdatesSelectedGraphWithoutFullPoseRenewal) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  auto heading = Heading(2);
  heading.heading_rad = 0.02;
  ASSERT_TRUE(owner.SubmitHeadingObservation(heading, Qualification()).ok());
  FixedLagAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(Odom(2).odometry.timestamp, &result).ok());
  ASSERT_TRUE(result.estimate);
  EXPECT_EQ(LocalizationTrustLevel::kDiagnostic, result.trust_level);
  EXPECT_EQ(StatusCode::kRejectedMeasurement, result.assessment.code());
  EXPECT_EQ(2u, result.estimate->correction_id);
  EXPECT_GT(result.estimate->map_from_base.rotation.toRotationMatrix()(1, 0), 0.0);
  EXPECT_DOUBLE_EQ(kStart * 1e-9 + 2 * kStep * 1e-9,
                   result.last_full_pose_time_seconds);
  EXPECT_FALSE(result.horizontal_estimate);
  EXPECT_EQ(StatusCode::kDuplicateTimestamp,
            owner.SubmitHeadingObservation(heading, Qualification()).code());
  heading.context.source = "bad-heading";
  heading.heading_rad = std::acos(-1.0);
  EXPECT_EQ(StatusCode::kRejectedMeasurement,
            owner.SubmitHeadingObservation(heading, Qualification()).code());
  ASSERT_TRUE(owner.GetResult(Odom(2).odometry.timestamp, &result).ok());
  EXPECT_EQ(2u, result.estimate->correction_id);
}

TEST(FixedLagAlignmentTest, HeadingRejectsBadGeometryQualityAndExactTime) {
  auto config = Config();
  config.enable_heading_bootstrap = true;
  config.assume_gravity_aligned_odom = true;
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(config).ok());
  ASSERT_TRUE(owner.AddOdometry(Odom(0)).ok());
  auto heading = Heading(0);
  heading.variance_rad2 = 0.0;
  EXPECT_EQ(StatusCode::kInvalidArgument,
            owner.SubmitHeadingObservation(heading, Qualification()).code());
  heading.variance_rad2 = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(StatusCode::kInvalidArgument,
            owner.SubmitHeadingObservation(heading, Qualification()).code());
  heading = Heading(0);
  heading.direction_in_base = Eigen::Vector3d::UnitZ();
  EXPECT_EQ(StatusCode::kRejectedMeasurement,
            owner.SubmitHeadingObservation(heading, Qualification()).code());
  heading = Heading(0);
  heading.context.source = "unqualified-heading";
  auto qualification = Qualification();
  qualification.quality_valid = false;
  EXPECT_EQ(StatusCode::kRejectedMeasurement,
            owner.SubmitHeadingObservation(heading, qualification).code());
  heading.context.measurement_time.nanoseconds += 1;
  EXPECT_EQ(StatusCode::kOutsideHistory,
            owner.SubmitHeadingObservation(heading, Qualification()).code());
  FixedLagAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(Odom(0).odometry.timestamp, &result).ok());
  EXPECT_FALSE(result.estimate);
  EXPECT_FALSE(result.horizontal_estimate);
  EXPECT_EQ(LocalizationTrustLevel::kUntrusted, result.trust_level);
  EXPECT_EQ(5u, result.rejected_observations);
}

TEST(FixedLagAlignmentTest, RejectsDiagnosticAndUnqualifiedDeclarations) {
  FixedLagGlobalAlignment owner;
  auto config = Config();
  config.motion_error_model_qualified = false;
  EXPECT_EQ(StatusCode::kInvalidArgument, owner.Configure(config).code());
  config = Config();
  config.reference.geodetic.state = GeodeticReferenceState::kDiagnosticOnly;
  EXPECT_EQ(StatusCode::kInvalidArgument, owner.Configure(config).code());
  ASSERT_TRUE(owner.Configure(Config()).ok());
  ASSERT_TRUE(owner.AddOdometry(Odom(0)).ok());
  auto qualification = Qualification();
  qualification.independent_of_graph_sources = false;
  EXPECT_EQ(StatusCode::kRejectedMeasurement,
            owner.SubmitMapObservation(Map(0), qualification).code());
  auto correlated = Map(0);
  correlated.context.provenance.shared_local_scans =
      ObservationDependency::kPresent;
  EXPECT_EQ(StatusCode::kRejectedMeasurement,
            owner.SubmitMapObservation(correlated, Qualification()).code());
  EXPECT_EQ(StatusCode::kRejectedMeasurement,
            owner.SubmitMapObservation(Map(0), Qualification()).code());
}

TEST(FixedLagAlignmentTest, ExplicitUncalibratedModeUsesTheSameConditionalGraph) {
  auto config = Config();
  config.allow_uncalibrated_inputs = true;
  config.reference.geodetic = {};
  config.reference.geodetic.state = GeodeticReferenceState::kDiagnosticOnly;
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(config).ok());
  auto qualification = Qualification();
  qualification.uncertainty_qualified = false;
  qualification.independent_of_local = false;
  qualification.independent_of_graph_sources = false;
  qualification.local_correlation.emplace();
  auto& correlation = *qualification.local_correlation;
  correlation.motion_error_model_id = config.motion_error_model_id;
  correlation.conditional_independence_evidence_id = "declared-sufficient-model";
  correlation.conditional_model_qualified = true;
  correlation.residual_independent_of_history_and_sources = true;
  correlation.cross_covariance = Eigen::MatrixXd::Zero(15, 6);
  correlation.cross_covariance(0, 0) = 0.0005;
  for (int index = 0; index < 5; ++index) {
    auto sample = Odom(index);
    sample.uncertainty_qualified = false;
    auto connection = Connection(index);
    ASSERT_TRUE(owner.AddOdometry(sample, index ? &connection : nullptr).ok());
    auto map = Map(index);
    map.context.reference = config.reference;
    map.context.provenance.shared_local_scans = ObservationDependency::kPresent;
    correlation.odometry_sequence = sample.sequence;
    const auto result = owner.SubmitMapObservation(map, qualification);
    if (index >= 2) {
      ASSERT_TRUE(result.ok()) << result.message();
    }
  }
  FixedLagAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(Odom(4).receive_time, &result).ok());
  ASSERT_TRUE(result.estimate);
  EXPECT_TRUE(result.diagnostic_only);
  EXPECT_FALSE(result.assessment.ok());
  EXPECT_TRUE(result.odometry_fresh);
  EXPECT_TRUE(result.full_pose_evidence_fresh);
  EXPECT_GT(result.estimate->correction_id, 0);
  EXPECT_NEAR(result.estimate->map_from_base.translation.x(), 100.4, 0.01);
  std::vector<GraphObservationDecision> decisions;
  ASSERT_TRUE(owner.GetDecisions(&decisions).ok());
  ASSERT_EQ(decisions.size(), 5);
  EXPECT_FALSE(decisions.back().qualification.uncertainty_qualified);
  EXPECT_TRUE(decisions.back().qualification.local_correlation);
  EXPECT_TRUE(decisions.back().observation.alignment_updated);
}

TEST(FixedLagAlignmentTest, UncalibratedModeNeverWaivesSufficiencyOrQuality) {
  auto config = Config();
  config.allow_uncalibrated_inputs = true;
  FixedLagGlobalAlignment owner;
  config.motion_error_model_qualified = false;
  EXPECT_FALSE(owner.Configure(config).ok());
  config.motion_error_model_qualified = true;
  ASSERT_TRUE(owner.Configure(config).ok());
  auto sample = Odom(0);
  sample.uncertainty_qualified = false;
  sample.covariance_model_valid = false;
  EXPECT_FALSE(owner.AddOdometry(sample).ok());
  sample.covariance_model_valid = true;
  ASSERT_TRUE(owner.AddOdometry(sample).ok());
  auto qualification = Qualification();
  qualification.uncertainty_qualified = false;
  qualification.quality_valid = false;
  EXPECT_FALSE(owner.SubmitMapObservation(Map(0), qualification).ok());
  qualification.quality_valid = true;
  qualification.independent_of_local = false;
  EXPECT_FALSE(owner.SubmitMapObservation(Map(0), qualification).ok());
  qualification.local_correlation.emplace();
  auto& correlation = *qualification.local_correlation;
  correlation.motion_error_model_id = config.motion_error_model_id;
  correlation.conditional_independence_evidence_id = "declared-model";
  correlation.odometry_sequence = sample.sequence;
  correlation.cross_covariance = Eigen::MatrixXd::Zero(15, 6);
  qualification.independent_of_graph_sources = false;
  EXPECT_FALSE(owner.SubmitMapObservation(Map(0), qualification).ok());
}

unified::LocalConfig RawNativeConfig() {
  namespace u = unified;
  u::LocalConfig local_config;
  local_config.session = "actual-native-graph";
  local_config.allow_imu_only_initialization = true;
  local_config.stationary_samples = 3;
  local_config.stationary_duration = 0.015;
  local_config.enable_native_joint_error = true;
  local_config.native_imu_sample_period = 0.01;
  local_config.history_duration = 0.035;
  local_config.enable_lidar = true;
  local_config.lidar_frame = "lidar";
  local_config.lidar_calibration_id = "test-lidar";
  local_config.lidar_point_noise_std = 0.01;
  local_config.lidar_plane_noise_std = 0.01;
  local_config.lidar_voxel_size = 1;
  local_config.lidar_max_voxels = 100;
  local_config.lidar_max_scans = 2;
  local_config.lidar_min_points_per_plane = 3;
  local_config.lidar_min_correspondences = 2;
  local_config.lidar_max_points = 1000;
  local_config.lidar_max_correspondence_distance = 0.3;
  local_config.lidar_observed_singular_value = 1;
  local_config.lidar_innovation_gate = 100;
  local_config.lidar_max_scan_age = 0.1;
  local_config.lidar_future_tolerance = 0.001;
  local_config.lidar_max_translation_correction_rate = 0.2;
  local_config.lidar_max_rotation_correction_rate = 0.1;
  local_config.lidar_max_velocity_correction_acceleration = 0.3;
  local_config.lidar_anchor_variance_scale = 1;
  local_config.lidar_correction_variance_scale = 2;
  local_config.lidar_max_translation_correction = 2;
  local_config.lidar_max_rotation_correction = 0.5;
  local_config.lidar_max_velocity_correction = 2;
  local_config.lidar_max_update_iterations = 2;
  local_config.lidar_process_noise_psd_tolerance = 1e-12;
  local_config.lidar_rotation_length = 1;
  return local_config;
}

TEST(FixedLagAlignmentTest, ActualRawNativeLioFeedsTheSelectedGraphUncalibrated) {
  namespace u = unified;
  const auto local_config = RawNativeConfig();
  NativeOdometry local(local_config);
  ASSERT_TRUE(local.ValidateConfig().ok());
  auto config = Config();
  config.allow_uncalibrated_inputs = true;
  config.motion_error_model_id = "native-frozen-linearization";
  config.history_duration_ns = 60'000'000LL;
  config.maximum_observation_age_ns = config.history_duration_ns;
  FixedLagGlobalAlignment graph;
  ASSERT_TRUE(graph.Configure(config).ok());
  double previous_time = 0;
  uint64_t previous_sequence = 0;
  uint64_t map_commits = 0, gnss_commits = 0;
  for (uint64_t index = 1; index <= 19; ++index) {
    SCOPED_TRACE(index);
    u::ImuSample imu;
    imu.stamp = {1 + 0.01 * (index - 1), 1 + 0.01 * (index - 1), index, "unix"};
    imu.acceleration.z() = local_config.gravity;
    local.AddImu(imu);
    if (index < 3) continue;
    u::LocalState modeled;
    const auto lookup = local.JointErrorLookup(imu.stamp.time, &modeled);
    ASSERT_TRUE(lookup.ok()) << lookup.message;
    GraphOdometrySample sample;
    sample.sequence = modeled.stamp.sequence;
    sample.odometry.timestamp = {
        1'000'000'000LL + 10'000'000LL * static_cast<int64_t>(index - 1),
        ClockDomain::kUnixEpoch};
    sample.receive_time = sample.odometry.timestamp;
    sample.odometry.epoch = {modeled.epoch.session, modeled.epoch.generation};
    sample.odometry.parent_frame = config.odom_frame;
    sample.odometry.child_frame = config.base_frame;
    sample.odometry.pose = {modeled.position, modeled.orientation};
    sample.covariance = modeled.covariance;
    sample.joint_error = modeled.joint_error;
    sample.covariance_model_valid = modeled.covariance_model_valid;
    sample.uncertainty_qualified = modeled.uncertainty_qualified;
    GraphMotionConnection connection;
    if (previous_sequence) {
      u::MotionIncrement motion;
      const auto connected =
          local.JointErrorMotionBetween(previous_time, imu.stamp.time, &motion);
      ASSERT_TRUE(connected.ok()) << connected.message;
      connection.epoch = sample.odometry.epoch;
      connection.start_time = {
          sample.odometry.timestamp.nanoseconds - 10'000'000LL, config.clock_domain};
      connection.end_time = sample.odometry.timestamp;
      connection.start_sequence = previous_sequence;
      connection.end_sequence = index;
      connection.cross_covariance = motion.cross_covariance;
      connection.joint_cross_covariance = motion.joint_cross_covariance;
      for (const auto& source : motion.sources)
        connection.sources.push_back({source.source, source.sequence});
    }
    const auto added = graph.AddOdometry(sample, previous_sequence ? &connection : nullptr);
    ASSERT_TRUE(added.ok()) << added.message();
    previous_time = imu.stamp.time;
    previous_sequence = index;
    auto context = Context(0, "independent-survey");
    context.epoch = sample.odometry.epoch;
    context.measurement_time = context.receive_time = sample.odometry.timestamp;
    context.sequence = index;
    GlobalMapObservation map;
    map.context = context;
    map.map_from_base = {
        Eigen::Vector3d(100, 20, 0) + modeled.position, modeled.orientation};
    map.covariance.setIdentity();
    map.covariance *= 0.01;
    map.covariance_source = "declared-independent-survey-noise";
    auto qualification = Qualification();
    qualification.uncertainty_qualified = false;
    const auto mapped = graph.SubmitMapObservation(map, qualification);
    if (index >= 5) {
      ASSERT_TRUE(mapped.ok()) << mapped.message();
      ++map_commits;
      GlobalHorizontalObservation gnss;
      gnss.context = context;
      gnss.context.source = "independent-gnss";
      gnss.position = map.map_from_base.translation.head<2>();
      gnss.covariance.setIdentity();
      gnss.covariance *= 0.04;
      gnss.covariance_source = "declared-independent-gnss-noise";
      const auto corrected = graph.SubmitHorizontalObservation(gnss, qualification);
      ASSERT_TRUE(corrected.ok()) << corrected.message();
      ++gnss_commits;
    }
    const auto frozen_local = local.state();
    if (index == 3 || index == 7 || index == 11 || index == 15) {
      u::LidarScan scan;
      scan.stamp = local.internal_state().stamp;
      scan.stamp.sequence = (index + 1) / 4;
      scan.epoch = local.internal_state().epoch;
      scan.frame_id = "lidar";
      scan.calibration_id = "test-lidar";
      for (double x : {0.1, 0.3, 0.5})
        for (double z : {0.1, 0.3, 0.5})
          for (double side : {-1.0, 1.0})
            scan.points.push_back({
                Eigen::Vector3d(x, side + 0.25 + (index == 3 ? 0 : 0.02), z),
                scan.stamp.time});
      const auto updated = local.AddLidar(scan);
      ASSERT_TRUE(updated.ok() || updated.reason == u::Reason::GEOMETRY_UNAVAILABLE)
          << updated.message;
      ASSERT_TRUE(local.joint_error_status().ok()) << local.joint_error_status().message;
    }
    EXPECT_TRUE(local.state().position.isApprox(frozen_local.position, 0));
    EXPECT_TRUE(local.state().orientation.isApprox(frozen_local.orientation, 0));
    EXPECT_FALSE(modeled.uncertainty_qualified);
  }
  EXPECT_EQ(local.lidar_health().accepted, 4);
  EXPECT_GT(map_commits, 0);
  EXPECT_GT(gnss_commits, 0);
  FixedLagAlignmentResult result;
  ASSERT_TRUE(graph.GetResult({1'180'000'000LL, config.clock_domain}, &result).ok());
  ASSERT_TRUE(result.estimate);
  EXPECT_TRUE(result.diagnostic_only);
  EXPECT_FALSE(result.assessment.ok());
  EXPECT_GT(result.estimate->correction_id, 0);
}

class SharedRawScanGraphTest : public ::testing::TestWithParam<bool> {};

TEST_P(SharedRawScanGraphTest, ActualSharedRawScanRegistrationFeedsTheSameGraph) {
  namespace u = unified;
  const bool delayed_bootstrap = GetParam();
  auto local_config = RawNativeConfig();
  local_config.native_joint_max_dimension = 1024;
  NativeOdometry local(local_config);
  map::PointCloud::Ptr reference(new map::PointCloud);
  for (int a = 0; a < 4; ++a)
    for (int b = 0; b < 4; ++b) {
      const double x = 0.2 * a + 0.03713, y = 0.2 * b + 0.04317;
      reference->emplace_back(x, y, 0.45119);
      reference->emplace_back(2.73713, x, y + 1.6);
      reference->emplace_back(x, 2.34317, y + 1.6);
    }
  map::ScanToMapConfig matcher_config;
  matcher_config.map_frame = "map";
  matcher_config.base_frame = matcher_config.cloud_frame = "body";
  matcher_config.calibration_id = "calibration";
  matcher_config.algorithm = map::ScanMatchingAlgorithm::kPointToPlaneIcp;
  matcher_config.retain_joint_error_support = true;
  matcher_config.scan_voxel_size_m = 0.05;
  matcher_config.normal_neighbors = 6;
  matcher_config.minimum_points = matcher_config.icp_minimum_pairs = 20;
  matcher_config.local_window_radius_m = 10;
  map::ScanToMapLocalizer matcher(matcher_config);
  map::MapMetadata metadata;
  metadata.map_id = "site";
  metadata.version = "v1";
  metadata.content_id = "synthetic-map";
  metadata.map_frame = "map";
  metadata.source_cloud_frame = "map";
  metadata.reference_state = map::MapReferenceState::kDiagnosticOnly;
  metadata.integrity_verified = true;
  std::string error;
  ASSERT_TRUE(matcher.Configure(metadata, reference, &error)) << error;
  auto config = Config();
  config.allow_uncalibrated_inputs = true;
  config.motion_error_model_id = "native-frozen-linearization";
  config.maximum_error_dimension = 1024;
  config.verification_frames = 2;
  config.history_duration_ns = config.maximum_observation_age_ns = 60'000'000LL;
  FixedLagGlobalAlignment graph;
  ASSERT_TRUE(graph.Configure(config).ok());
  double previous_time = 0;
  uint64_t previous_sequence = 0, updates = 0;
  std::unique_ptr<NativeOdometry::LidarTask> delayed;
  for (uint64_t index = 1; index <= 19; ++index) {
    SCOPED_TRACE(index);
    u::ImuSample imu;
    imu.stamp = {1 + 0.01 * (index - 1), 1 + 0.01 * (index - 1), index, "unix"};
    imu.acceleration.z() = local_config.gravity;
    local.AddImu(imu);
    if (delayed) {
      const auto replayed = delayed->ReplayImu(imu);
      ASSERT_TRUE(replayed.ok() || replayed.reason == u::Reason::WHEEL_STALE ||
                  replayed.reason == u::Reason::CORRELATION_UNQUALIFIED ||
                  replayed.reason == u::Reason::CORRECTION_BUDGET_EXCEEDED)
          << replayed.message;
      ASSERT_TRUE(delayed->internal_state().valid);
    }
    if (index < 3) continue;
    const auto nominal = local.state();
    const bool has_scan = delayed_bootstrap ?
        (index == 5 || index == 9 || index == 13 || index == 17) :
        (index == 3 || index == 7 || index == 11 || index == 17);
    u::JointLidarScan shared;
    u::LocalState modeled;
    if (index == 3 || index == 7 || index == 11 || index == 15) {
      u::LidarScan scan;
      scan.stamp = local.internal_state().stamp;
      scan.stamp.sequence = (index + 1) / 4;
      scan.epoch = local.internal_state().epoch;
      scan.frame_id = "lidar";
      scan.calibration_id = "test-lidar";
      for (const auto& point : *reference)
        scan.points.push_back({Eigen::Vector3d(point.x, point.y, point.z),
                               scan.stamp.time - (index == 3 ? 0 : 0.005)});
      if (delayed_bootstrap || index == 15) {
        ASSERT_TRUE(local.PrepareLidarTask(scan, &delayed).ok());
        const auto computed = delayed->Compute();
        ASSERT_TRUE(computed.ok() || computed.reason == u::Reason::GEOMETRY_UNAVAILABLE)
            << computed.message;
      } else {
        const auto updated = local.AddLidar(scan);
        ASSERT_TRUE(updated.ok() || updated.reason == u::Reason::GEOMETRY_UNAVAILABLE)
            << updated.message;
      }
    }
    if (delayed && has_scan) {
      const auto committed = local.CommitLidarTask(delayed.get());
      ASSERT_TRUE(committed.ok() || committed.reason == u::Reason::GEOMETRY_UNAVAILABLE)
          << committed.message;
      delayed.reset();
    }
    if (has_scan) {
      const auto exported = local.PrepareJointLidarScan(
          delayed_bootstrap ? (index - 1) / 4 : (index == 17 ? 4 : (index + 1) / 4),
                                                       &shared);
      ASSERT_TRUE(exported.ok()) << exported.message;
      modeled = shared.conditioning_state;
    } else {
      ASSERT_TRUE(local.JointErrorLookup(imu.stamp.time, &modeled).ok());
    }
    GraphOdometrySample sample;
    sample.sequence = index;
    sample.odometry.timestamp = {
        1'000'000'000LL + 10'000'000LL * static_cast<int64_t>(index - 1),
        config.clock_domain};
    sample.receive_time = sample.odometry.timestamp;
    sample.odometry.epoch = {modeled.epoch.session, modeled.epoch.generation};
    sample.odometry.parent_frame = config.odom_frame;
    sample.odometry.child_frame = config.base_frame;
    sample.odometry.pose = {modeled.position, modeled.orientation};
    sample.covariance = modeled.covariance;
    sample.joint_error = modeled.joint_error;
    sample.covariance_model_valid = true;
    GraphMotionConnection connection;
    if (previous_sequence) {
      u::MotionIncrement motion;
      const auto connected = local.JointErrorMotionBetween(
          previous_time, imu.stamp.time, &motion);
      ASSERT_TRUE(connected.ok()) << connected.message;
      connection.epoch = sample.odometry.epoch;
      connection.start_time = {sample.odometry.timestamp.nanoseconds - 10'000'000LL,
                               config.clock_domain};
      connection.end_time = sample.odometry.timestamp;
      connection.start_sequence = previous_sequence;
      connection.end_sequence = index;
      connection.cross_covariance = motion.cross_covariance;
      connection.joint_cross_covariance = motion.joint_cross_covariance;
      for (const auto& source : motion.sources)
        connection.sources.push_back({source.source, source.sequence});
    }
    const auto added = graph.AddOdometry(sample, previous_sequence ? &connection : nullptr);
    ASSERT_TRUE(added.ok()) << added.message();
    previous_time = imu.stamp.time;
    previous_sequence = index;
    if (has_scan) {
      map::ScanToMapRequest request;
      request.timestamp_ns = static_cast<int64_t>(std::llround(shared.reference.time * 1e9));
      request.clock_domain = "unix";
      request.cloud_frame = "body";
      request.deskewed = true;
      request.joint_scan = &shared;
      request.initial_joint_jacobian =
          Eigen::MatrixXd::Zero(6, shared.conditioning_state.joint_error->covariance.rows());
      const auto matched = matcher.Match(request);
      ASSERT_TRUE(matched.valid) << matched.reason << ": " << matched.detail;
      ASSERT_TRUE(matched.joint_error);
      const auto& model = *matched.joint_error;
      ASSERT_TRUE(model.assessment.ok()) << model.assessment.message;
      if (model.source_sequence == 1) {
        request.initial_joint_jacobian.resize(0, 0);
        const auto rejected = matcher.Match(request);
        EXPECT_FALSE(rejected.valid);
        EXPECT_EQ(rejected.reason, "invalid_shared_scan_or_seed_error_support");
        request.joint_scan = nullptr;
        request.cloud = reference;
        const auto nominal_match = matcher.Match(request);
        ASSERT_TRUE(nominal_match.valid);
        EXPECT_TRUE(nominal_match.map_from_base.matrix().isApprox(
            matched.map_from_base.matrix(), 0));
        EXPECT_TRUE(nominal_match.covariance.isApprox(matched.covariance, 0));
      }
      EXPECT_TRUE((model.covariance - model.pose_jacobian *
          model.local_cross_covariance).isZero(1e-12));
      auto map = Map(0);
      map.context.epoch = sample.odometry.epoch;
      map.context.source = "actual-shared-scan";
      map.context.sequence = model.source_sequence;
      map.context.measurement_time = {
          static_cast<int64_t>(std::llround(shared.reference.time * 1e9)), config.clock_domain};
      map.context.receive_time = sample.odometry.timestamp;
      map.context.provenance.shared_local_scans = ObservationDependency::kPresent;
      map.context.provenance.odometry_deskew = ObservationDependency::kPresent;
      map.map_from_base = {matched.map_from_base.translation(),
                          Eigen::Quaterniond(matched.map_from_base.linear())};
      map.covariance = model.covariance;
      map.covariance_source = "uncalibrated-frozen-ICP-conditional-on-fixed-map";
      GraphSourceQualification qualification;
      qualification.quality_valid = true;
      qualification.uncertainty_model_id = map.covariance_source;
      qualification.local_correlation.emplace();
      auto& correlation = *qualification.local_correlation;
      correlation.motion_error_model_id = config.motion_error_model_id;
      correlation.odometry_sequence = index;
      correlation.conditioning_time = sample.odometry.timestamp;
      correlation.cross_covariance = model.local_cross_covariance;
      correlation.conditional_independence_evidence_id = "all-points-and-seed-in-native-basis";
      correlation.conditional_model_qualified = true;
      correlation.residual_independent_of_history_and_sources = true;
      const auto submitted = graph.SubmitMapObservation(map, qualification);
      if (model.source_sequence == 1) {
        EXPECT_FALSE(submitted.ok());
      } else {
        ASSERT_TRUE(submitted.ok()) << submitted.message();
        ++updates;
      }
    }
    EXPECT_TRUE(local.state().position.isApprox(nominal.position, 0));
    EXPECT_TRUE(local.state().orientation.isApprox(nominal.orientation, 0));
    EXPECT_TRUE(local.state().covariance.isApprox(nominal.covariance, 0));
  }
  EXPECT_EQ(updates, 3);
  FixedLagAlignmentResult result;
  ASSERT_TRUE(graph.GetResult({1'180'000'000LL, config.clock_domain}, &result).ok());
  ASSERT_TRUE(result.estimate);
  EXPECT_TRUE(result.diagnostic_only);
  EXPECT_FALSE(result.assessment.ok());
}

INSTANTIATE_TEST_SUITE_P(NativeSharedRegistration, SharedRawScanGraphTest,
                        ::testing::Bool());

TEST(FixedLagAlignmentTest, JointMemoryIsRetainedThroughFacadeBootstrap) {
  FixedLagGlobalAlignment owner;
  auto config = Config();
  config.maximum_error_dimension = 16;
  ASSERT_TRUE(owner.Configure(config).ok());
  auto qualification = Qualification();
  qualification.independent_of_local = false;
  qualification.independent_of_graph_sources = false;
  qualification.local_correlation.emplace();
  auto& correlation = *qualification.local_correlation;
  correlation.motion_error_model_id = config.motion_error_model_id;
  correlation.conditional_independence_evidence_id = "synthetic-hidden-memory";
  correlation.conditional_model_qualified = true;
  correlation.residual_independent_of_history_and_sources = true;
  correlation.cross_covariance = Eigen::MatrixXd::Zero(16, 6);
  correlation.cross_covariance(15, 0) = 0.0025;
  for (int index = 0; index < 4; ++index) {
    auto sample = Odom(index);
    unified::JointLocalError joint;
    joint.covariance = Eigen::MatrixXd::Zero(16, 16);
    joint.covariance.topLeftCorner<15, 15>() = sample.covariance;
    joint.covariance(15, 15) = 0.025;
    joint.output_jacobian = Eigen::MatrixXd::Zero(15, 16);
    joint.output_jacobian.leftCols<15>().setIdentity();
    sample.joint_error = joint;
    auto connection = Connection(index);
    if (index != 0) {
      connection.joint_cross_covariance = Eigen::MatrixXd::Zero(16, 16);
      connection.joint_cross_covariance.topLeftCorner<15, 15>() =
          connection.cross_covariance;
      connection.joint_cross_covariance(15, 15) = 0.025;
    }
    const auto added = owner.AddOdometry(sample, index ? &connection : nullptr);
    ASSERT_TRUE(added.ok()) << added.message();
    if (index < 3) {
      correlation.odometry_sequence = sample.sequence;
      // Map covariance is .001; conditional variance remains positive.
      const auto status = owner.SubmitMapObservation(Map(index), qualification);
      if (index < 2) {
        EXPECT_EQ(status.code(), StatusCode::kRejectedMeasurement);
      } else {
        ASSERT_TRUE(status.ok()) << status.message();
      }
    }
  }
  FixedLagAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(Odom(3).odometry.timestamp, &result).ok());
  ASSERT_TRUE(result.estimate);
  EXPECT_NEAR(result.estimate->map_from_base.translation.x(), 100.3, 1e-8);
  correlation.odometry_sequence = Odom(3).sequence;
  correlation.cross_covariance = Eigen::MatrixXd::Zero(15, 6);
  EXPECT_EQ(owner.SubmitMapObservation(Map(3), qualification).code(),
            StatusCode::kRejectedMeasurement);
  correlation.cross_covariance = Eigen::MatrixXd::Zero(16, 6);
  correlation.cross_covariance(15, 0) = 0.0025;
  EXPECT_TRUE(owner.SubmitMapObservation(Map(3), qualification).ok());
}

TEST(FixedLagAlignmentTest, DelayedAntennaXYAndProjectedMapShareOneOwner) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  const auto connection = Connection(3);
  ASSERT_TRUE(owner.AddOdometry(Odom(3), &connection).ok());
  FixedLagAlignmentResult before;
  ASSERT_TRUE(owner.GetResult(Odom(3).odometry.timestamp, &before).ok());
  ASSERT_TRUE(before.estimate.has_value());
  GlobalHorizontalObservation fix;
  fix.context = Context(2, "independent-gnss");
  fix.context.receive_time = Odom(3).odometry.timestamp;
  fix.point_in_base = Eigen::Vector3d(0.5, 0.0, 0.2);
  fix.position = Eigen::Vector2d(100.7, 20.01);
  fix.covariance << 0.01, 0.002, 0.002, 0.02;
  fix.covariance_source = "synthetic-xy-noise";
  ASSERT_TRUE(owner.SubmitHorizontalObservation(fix, Qualification()).ok());
  auto map = Map(3);
  map.context.source = "independent-projected-map";
  map.kind = GlobalMapObservation::Kind::kProjectedPose;
  map.projection = Eigen::MatrixXd::Zero(1, 6);
  map.projection(0, 0) = 1.0;
  map.projected_covariance = Eigen::MatrixXd::Identity(1, 1) * 0.01;
  ASSERT_TRUE(owner.SubmitMapObservation(map, Qualification()).ok());
  FixedLagAlignmentResult after;
  ASSERT_TRUE(owner.GetResult(Odom(3).odometry.timestamp, &after).ok());
  ASSERT_TRUE(after.estimate.has_value());
  EXPECT_EQ(before.estimate->correction_id + 2, after.estimate->correction_id);
  EXPECT_DOUBLE_EQ(before.last_full_pose_time_seconds,
                   after.last_full_pose_time_seconds);
  const Eigen::Vector3d relative = after.estimate->map_from_base.translation -
                                   after.estimate->map_from_odom.translation;
  EXPECT_TRUE(relative.isApprox(after.estimate->map_from_odom.rotation *
                                    Odom(3).odometry.pose.translation,
                                1e-9));
  EXPECT_EQ(StatusCode::kDuplicateTimestamp,
            owner.SubmitHorizontalObservation(fix, Qualification()).code());
}

TEST(FixedLagAlignmentTest, DeterministicConditionalDoesNotRevokeQualifiedOdom) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  auto previous = Odom(2);
  for (int index = 3; index <= 6; ++index) {
    auto sample = Odom(index);
    sample.covariance = previous.covariance;
    auto connection = Connection(index);
    connection.cross_covariance = previous.covariance;
    const auto result = owner.AddOdometry(sample, &connection);
    ASSERT_TRUE(result.ok()) << result.message();
    previous = sample;
  }
  FixedLagAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(previous.odometry.timestamp, &result).ok());
  ASSERT_TRUE(result.estimate.has_value());
  EXPECT_TRUE(result.odometry_fresh);
  EXPECT_NEAR(result.estimate->map_from_odom.translation.x(), 100, 1e-8);
}

TEST(FixedLagAlignmentTest, SharedSourceRequiresQualifiedSufficientErrorModel) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  auto map = Map(2);
  map.context.source = "conditionally-shared-map";
  map.context.provenance.shared_local_scans = ObservationDependency::kPresent;
  map.context.provenance.odometry_deskew = ObservationDependency::kPresent;
  auto qualification = Qualification();
  qualification.independent_of_local = false;
  qualification.independent_of_graph_sources = false;
  qualification.independence_evidence_id.clear();
  GraphLocalObservationCorrelation model;
  model.motion_error_model_id = Config().motion_error_model_id;
  model.conditional_independence_evidence_id = "synthetic-sufficient-state";
  model.odometry_sequence = Odom(2).sequence;
  model.cross_covariance = Eigen::MatrixXd::Zero(15, 6);
  model.cross_covariance(0, 0) = 0.001;
  qualification.local_correlation = model;
  EXPECT_EQ(owner.SubmitMapObservation(map, qualification).code(),
            StatusCode::kRejectedMeasurement);
  qualification.local_correlation->conditional_model_qualified = true;
  qualification.local_correlation->residual_independent_of_history_and_sources = true;
  qualification.local_correlation->motion_error_model_id = "wrong-model";
  EXPECT_EQ(owner.SubmitMapObservation(map, qualification).code(),
            StatusCode::kRejectedMeasurement);
  qualification.local_correlation->motion_error_model_id = model.motion_error_model_id;
  qualification.local_correlation->odometry_sequence++;
  EXPECT_FALSE(owner.SubmitMapObservation(map, qualification).ok());
  qualification.local_correlation->odometry_sequence = model.odometry_sequence;
  ASSERT_TRUE(owner.SubmitMapObservation(map, qualification).ok());
  std::vector<GraphObservationDecision> decisions;
  ASSERT_TRUE(owner.GetDecisions(&decisions).ok());
  ASSERT_TRUE(decisions.back().observation.alignment_updated);
  ASSERT_TRUE(decisions.back().qualification.local_correlation.has_value());
  EXPECT_TRUE(decisions.back().qualification.local_correlation->cross_covariance
                  .isApprox(model.cross_covariance, 0));
}

TEST(FixedLagAlignmentTest, QualifiedConditionalFullPosesCanBootstrapSameOwner) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  auto qualification = Qualification();
  qualification.independent_of_local = false;
  qualification.independent_of_graph_sources = false;
  qualification.independence_evidence_id.clear();
  GraphLocalObservationCorrelation model;
  model.motion_error_model_id = Config().motion_error_model_id;
  model.conditional_independence_evidence_id = "synthetic-full-state-proof";
  model.conditional_model_qualified = true;
  model.residual_independent_of_history_and_sources = true;
  model.cross_covariance = Eigen::MatrixXd::Zero(15, 6);
  model.cross_covariance(0, 0) = 0.001;
  for (int index = 0; index < 3; ++index) {
    const auto sample = Odom(index);
    const auto connection = Connection(index);
    ASSERT_TRUE(owner.AddOdometry(sample, index ? &connection : nullptr).ok());
    model.odometry_sequence = sample.sequence;
    qualification.local_correlation = model;
    auto map = Map(index);
    map.context.provenance.shared_local_scans = ObservationDependency::kPresent;
    if (index == 0) {
      qualification.local_correlation->cross_covariance(0, 0) = 1;
      EXPECT_FALSE(owner.SubmitMapObservation(map, qualification).ok());
      qualification.local_correlation->cross_covariance = model.cross_covariance;
    }
    const auto status = owner.SubmitMapObservation(map, qualification);
    EXPECT_EQ(status.ok(), index == 2) << status.message();
    EXPECT_DOUBLE_EQ(sample.odometry.pose.translation.x(), 0.1 * index);
  }
  FixedLagAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(Odom(2).odometry.timestamp, &result).ok());
  ASSERT_TRUE(result.estimate.has_value());
  EXPECT_NEAR(result.estimate->map_from_odom.translation.x(), 100, 1e-8);
}

TEST(FixedLagAlignmentTest, MissingMotionRevokesGlobalButRetainsNewOdom) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  EXPECT_EQ(StatusCode::kOutsideHistory, owner.AddOdometry(Odom(3)).code());
  FixedLagAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(Odom(3).odometry.timestamp, &result).ok());
  EXPECT_FALSE(result.estimate.has_value());
  EXPECT_FALSE(result.assessment.ok());
  EXPECT_EQ(Odom(3).odometry.timestamp.nanoseconds,
            result.timestamp.nanoseconds);
  const auto connection = Connection(4);
  ASSERT_TRUE(owner.AddOdometry(Odom(4), &connection).ok());
}

TEST(FixedLagAlignmentTest, NoInterpolatedNodeOrPartialPoseBootstrap) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  ASSERT_TRUE(owner.AddOdometry(Odom(0)).ok());
  const auto connection = Connection(1);
  ASSERT_TRUE(owner.AddOdometry(Odom(1), &connection).ok());
  auto between = Map(0);
  between.context.measurement_time.nanoseconds += kStep / 2;
  between.context.receive_time = Odom(1).odometry.timestamp;
  EXPECT_EQ(StatusCode::kOutsideHistory,
            owner.SubmitMapObservation(between, Qualification()).code());
  GlobalPositionObservation fix;
  fix.context = Context(1, "independent-xyz");
  fix.position = Map(1).map_from_base.translation;
  fix.covariance = Eigen::Matrix3d::Identity();
  fix.covariance_source = "synthetic-xyz-noise";
  ASSERT_TRUE(owner.SubmitPositionObservation(fix, Qualification()).ok());
  FixedLagAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(Odom(1).odometry.timestamp, &result).ok());
  EXPECT_FALSE(result.estimate.has_value());
}

TEST(FixedLagAlignmentTest, BoundedDecisionsRetainAdapterAndOwnerRejections) {
  FixedLagGlobalAlignment owner;
  auto config = Config();
  config.maximum_decisions = 2;
  ASSERT_TRUE(owner.Configure(config).ok());
  auto invalid = Map(0);
  invalid.map_from_base.rotation.coeffs().setZero();
  EXPECT_EQ(StatusCode::kInvalidArgument,
            owner.SubmitMapObservation(invalid, Qualification()).code());
  auto unqualified = Qualification();
  unqualified.independent_of_local = false;
  EXPECT_EQ(StatusCode::kRejectedMeasurement,
            owner.SubmitMapObservation(Map(0), unqualified).code());
  std::vector<GraphObservationDecision> decisions;
  ASSERT_TRUE(owner.GetDecisions(&decisions).ok());
  ASSERT_EQ(2u, decisions.size());
  EXPECT_EQ("synthetic-map-noise",
            decisions.front().observation.covariance_source);
  EXPECT_FALSE(decisions.back().qualification.independent_of_local);
  ASSERT_TRUE(owner.AddOdometry(Odom(0)).ok());
  EXPECT_EQ(StatusCode::kRejectedMeasurement,
            owner.SubmitMapObservation(Map(0), Qualification()).code());
  ASSERT_TRUE(owner.GetDecisions(&decisions).ok());
  ASSERT_EQ(2u, decisions.size());
  EXPECT_EQ(2u, decisions.front().observation.decision_id);
  EXPECT_EQ(3u, decisions.back().observation.decision_id);
  EXPECT_FALSE(decisions.back().observation.alignment_updated);
  FixedLagAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(Odom(0).odometry.timestamp, &result).ok());
  EXPECT_EQ(1u, result.decisions_dropped);
  EXPECT_EQ(3u, result.submitted_observations);
  EXPECT_EQ(3u, result.rejected_observations);
  ASSERT_EQ(1u, result.sources.size());
  EXPECT_EQ(3u, result.sources.front().rejected);
}

TEST(FixedLagAlignmentTest,
     MotionEndpointMismatchRetainsCurrentOdomForRecovery) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  auto connection = Connection(3);
  ++connection.start_time.nanoseconds;
  EXPECT_EQ(StatusCode::kOutsideHistory,
            owner.AddOdometry(Odom(3), &connection).code());
  FixedLagAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(Odom(3).odometry.timestamp, &result).ok());
  EXPECT_FALSE(result.estimate.has_value());
  EXPECT_FALSE(result.assessment.ok());
  EXPECT_EQ(Odom(3).odometry.timestamp.nanoseconds,
            result.timestamp.nanoseconds);
  connection = Connection(4);
  ASSERT_TRUE(owner.AddOdometry(Odom(4), &connection).ok());
}

TEST(FixedLagAlignmentTest,
     InvalidJointMotionRetainsOdomWithoutReusingTheGraph) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  auto connection = Connection(3);
  connection.cross_covariance = Matrix15d::Identity() * 100.0;
  EXPECT_EQ(StatusCode::kInvalidArgument,
            owner.AddOdometry(Odom(3), &connection).code());
  FixedLagAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(Odom(3).odometry.timestamp, &result).ok());
  EXPECT_FALSE(result.estimate.has_value());
  EXPECT_EQ(Odom(3).odometry.timestamp.nanoseconds,
            result.timestamp.nanoseconds);
  connection = Connection(4);
  ASSERT_TRUE(owner.AddOdometry(Odom(4), &connection).ok());
}

TEST(FixedLagAlignmentTest, FullPoseFreshnessDoesNotFollowPartialUpdates) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  GlobalHorizontalObservation fix;
  fix.context = Context(2, "independent-xy");
  fix.position = Eigen::Vector2d(100.2, 20.0);
  fix.covariance = Eigen::Matrix2d::Identity();
  fix.covariance_source = "synthetic-xy-noise";
  ASSERT_TRUE(owner.SubmitHorizontalObservation(fix, Qualification()).ok());
  FixedLagAlignmentResult result;
  auto evaluation = Odom(2).odometry.timestamp;
  evaluation.nanoseconds += Config().maximum_full_pose_age_ns + 1;
  ASSERT_TRUE(owner.GetResult(evaluation, &result).ok());
  EXPECT_TRUE(result.estimate.has_value());
  EXPECT_FALSE(result.odometry_fresh);
  EXPECT_FALSE(result.full_pose_evidence_fresh);
  EXPECT_FALSE(result.assessment.ok());
}

TEST(FixedLagAlignmentTest, RetainedPartialGraphNeverRenewsFullPoseTrust) {
  for (const bool retain : {false, true}) {
    auto config = Config();
    config.maximum_full_pose_age_ns = 3 * kStep;
    config.retain_graph_on_full_pose_staleness = retain;
    FixedLagGlobalAlignment owner;
    ASSERT_TRUE(owner.Configure(config).ok());
    Bootstrap(&owner);
    for (int index = 3; index <= 9; ++index) {
      auto motion = Connection(index);
      ASSERT_TRUE(owner.AddOdometry(Odom(index), &motion).ok());
      auto partial = Map(index);
      partial.kind = GlobalMapObservation::Kind::kProjectedPose;
      partial.projection = Eigen::Matrix<double, 5, 6>::Zero();
      partial.projection.rightCols<5>().setIdentity();
      partial.projected_covariance = Eigen::Matrix<double, 5, 5>::Identity() * .001;
      const auto status = owner.SubmitMapObservation(partial, Qualification());
      if (retain) {
        ASSERT_TRUE(status.ok()) << status.message();
      } else if (index >= 6) {
        EXPECT_FALSE(status.ok());
      }
      FixedLagAlignmentResult result;
      ASSERT_TRUE(owner.GetResult(Odom(index).odometry.timestamp, &result).ok());
      if (index >= 6) {
        EXPECT_FALSE(result.full_pose_evidence_fresh);
        EXPECT_EQ(LocalizationTrustLevel::kUntrusted, result.trust_level);
      }
      if (retain) {
        ASSERT_TRUE(result.estimate);
        EXPECT_NEAR(100.0 + .1 * index, result.estimate->map_from_base.translation.x(), 1e-8);
        EXPECT_DOUBLE_EQ(kStart * 1e-9 + 2 * kStep * 1e-9,
                         result.last_full_pose_time_seconds);
      }
    }
    if (retain) {
      const auto motion = Connection(10);
      ASSERT_TRUE(owner.AddOdometry(Odom(10), &motion).ok());
      ASSERT_TRUE(owner.SubmitMapObservation(Map(10), Qualification()).ok());
      FixedLagAlignmentResult result;
      ASSERT_TRUE(owner.GetResult(Odom(10).odometry.timestamp, &result).ok());
      EXPECT_TRUE(result.full_pose_evidence_fresh);
      const auto next_motion = Connection(11);
      ASSERT_TRUE(owner.AddOdometry(Odom(11), &next_motion).ok());
      auto recovery = Map(11);
      recovery.recovery = true;
      EXPECT_FALSE(owner.SubmitMapObservation(recovery, Qualification()).ok());
      ASSERT_TRUE(owner.GetResult(Odom(11).odometry.timestamp, &result).ok());
      EXPECT_TRUE(result.verification_pending);
      EXPECT_FALSE(result.estimate);
      EXPECT_FALSE(owner.AddOdometry(Odom(12)).ok());
      ASSERT_TRUE(owner.GetResult(Odom(12).odometry.timestamp, &result).ok());
      EXPECT_FALSE(result.estimate);
    }
  }
}

TEST(FixedLagAlignmentTest, DoubleTimeAliasesCannotSelectAnotherIntegerEpoch) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  auto odom = Odom(0);
  odom.odometry.timestamp.nanoseconds = 1'792'454'400'000'000'000LL;
  odom.receive_time = odom.odometry.timestamp;
  ASSERT_TRUE(owner.AddOdometry(odom).ok());
  auto map = Map(0);
  map.context.measurement_time = odom.odometry.timestamp;
  map.context.receive_time = odom.receive_time;
  ++map.context.measurement_time.nanoseconds;
  EXPECT_EQ(StatusCode::kOutsideHistory,
            owner.SubmitMapObservation(map, Qualification()).code());
  map.context.measurement_time = odom.odometry.timestamp;
  EXPECT_EQ(StatusCode::kRejectedMeasurement,
            owner.SubmitMapObservation(map, Qualification()).code());
}

TEST(FixedLagAlignmentTest, EpochChangeRetiresAlignmentAndKeepsDecisionIds) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  std::vector<GraphObservationDecision> before;
  ASSERT_TRUE(owner.GetDecisions(&before).ok());
  auto next = Odom(3);
  ++next.odometry.epoch.generation;
  ASSERT_TRUE(owner.AddOdometry(next).ok());
  FixedLagAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(next.odometry.timestamp, &result).ok());
  EXPECT_FALSE(result.estimate.has_value());
  EXPECT_EQ(next.odometry.epoch, result.epoch);
  auto retired = Map(3);
  EXPECT_EQ(StatusCode::kEpochMismatch,
            owner.SubmitMapObservation(retired, Qualification()).code());
  std::vector<GraphObservationDecision> after;
  ASSERT_TRUE(owner.GetDecisions(&after).ok());
  EXPECT_EQ(before.back().observation.decision_id + 1,
            after.back().observation.decision_id);
}

TEST(FixedLagAlignmentTest, OutlierDoesNotCommitAndRecoveryRequiresNewFrames) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  const auto connection = Connection(3);
  ASSERT_TRUE(owner.AddOdometry(Odom(3), &connection).ok());
  FixedLagAlignmentResult before;
  ASSERT_TRUE(owner.GetResult(Odom(3).odometry.timestamp, &before).ok());
  ASSERT_TRUE(before.estimate.has_value());
  auto outlier = Map(3);
  outlier.map_from_base.translation.x() += 1000.0;
  EXPECT_EQ(StatusCode::kRejectedMeasurement,
            owner.SubmitMapObservation(outlier, Qualification()).code());
  std::vector<GraphObservationDecision> rejected_decisions;
  ASSERT_TRUE(owner.GetDecisions(&rejected_decisions).ok());
  ASSERT_FALSE(rejected_decisions.empty());
  const auto& innovation = rejected_decisions.back().observation.innovation;
  ASSERT_TRUE(innovation.available);
  ASSERT_EQ(innovation.observed_dimension, innovation.residual.size());
  ASSERT_EQ(innovation.residual.size(), innovation.covariance.rows());
  EXPECT_NEAR(
      innovation.nis,
      innovation.residual.dot(
          innovation.covariance.ldlt().solve(innovation.residual)),
      1e-8);
  FixedLagAlignmentResult rejected;
  ASSERT_TRUE(owner.GetResult(Odom(3).odometry.timestamp, &rejected).ok());
  ASSERT_TRUE(rejected.estimate.has_value());
  EXPECT_EQ(before.estimate->correction_id, rejected.estimate->correction_id);
  EXPECT_TRUE(before.estimate->map_from_odom.translation.isApprox(
      rejected.estimate->map_from_odom.translation, 0.0));
  EXPECT_TRUE(before.estimate->alignment_covariance.isApprox(
      rejected.estimate->alignment_covariance, 0.0));

  for (int index = 4; index <= 6; ++index) {
    const auto motion = Connection(index);
    ASSERT_TRUE(owner.AddOdometry(Odom(index), &motion).ok());
    auto recovery = Map(index);
    recovery.recovery = true;
    recovery.map_from_base.translation.x() += 10.0;
    std::vector<GraphObservationDecision> decisions;
    // The first recovery follows a legitimate ODOM graph update.
    if (index == 4) {
      FixedLagAlignmentResult current;
      ASSERT_TRUE(
          owner.GetResult(Odom(index).odometry.timestamp, &current).ok());
      ASSERT_TRUE(current.estimate.has_value());
      before = current;
    }
    const auto status = owner.SubmitMapObservation(recovery, Qualification());
    ASSERT_TRUE(owner.GetDecisions(&decisions).ok());
    FixedLagAlignmentResult result;
    ASSERT_TRUE(owner.GetResult(Odom(index).odometry.timestamp, &result).ok());
    if (index < 6) {
      EXPECT_EQ(StatusCode::kRejectedMeasurement, status.code());
      EXPECT_TRUE(result.verification_pending);
      EXPECT_FALSE(result.estimate.has_value());
      EXPECT_EQ(before.estimate->correction_id,
                decisions.back().observation.correction_id);
      EXPECT_FALSE(decisions.back().observation.alignment_updated);
      EXPECT_EQ(StatusCode::kDuplicateTimestamp,
                owner.SubmitMapObservation(recovery, Qualification()).code());
    } else {
      ASSERT_TRUE(status.ok()) << status.message();
      ASSERT_TRUE(result.estimate.has_value());
      EXPECT_FALSE(result.verification_pending);
      EXPECT_EQ(before.estimate->correction_id + 1,
                result.estimate->correction_id);
      EXPECT_NEAR(110.0, result.estimate->map_from_odom.translation.x(), 1e-6);
    }
  }
}

TEST(FixedLagAlignmentTest, AmbiguityInterruptsVerificationWithoutAnOdomReset) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  ASSERT_TRUE(owner.AddOdometry(Odom(0)).ok());
  for (int index = 0; index <= 4; ++index) {
    if (index > 0) {
      const auto connection = Connection(index);
      ASSERT_TRUE(owner.AddOdometry(Odom(index), &connection).ok());
    }
    auto observation = Map(index);
    observation.ambiguous = index == 1;
    const auto status =
        owner.SubmitMapObservation(observation, Qualification());
    FixedLagAlignmentResult result;
    ASSERT_TRUE(owner.GetResult(Odom(index).odometry.timestamp, &result).ok());
    if (index < 4) {
      EXPECT_EQ(StatusCode::kRejectedMeasurement, status.code());
      EXPECT_FALSE(result.estimate.has_value());
      EXPECT_EQ(index != 1, result.verification_pending);
    } else {
      ASSERT_TRUE(status.ok()) << status.message();
      ASSERT_TRUE(result.estimate.has_value());
      EXPECT_EQ(1u, result.estimate->correction_id);
      EXPECT_EQ(Odom(0).odometry.epoch, result.epoch);
    }
  }
}

TEST(FixedLagAlignmentTest, DelayedFactorsKeepTheirTimesAndWindowLimits) {
  FixedLagGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  for (int index = 3; index <= 5; ++index) {
    const auto connection = Connection(index);
    ASSERT_TRUE(owner.AddOdometry(Odom(index), &connection).ok());
  }
  auto latest = Map(5);
  ASSERT_TRUE(owner.SubmitMapObservation(latest, Qualification()).ok());
  auto delayed = Map(3);
  delayed.context.receive_time = Odom(5).odometry.timestamp;
  ASSERT_TRUE(owner.SubmitMapObservation(delayed, Qualification()).ok());
  FixedLagAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(Odom(5).odometry.timestamp, &result).ok());
  ASSERT_TRUE(result.estimate.has_value());
  EXPECT_DOUBLE_EQ(
      static_cast<double>(latest.context.measurement_time.nanoseconds) * 1e-9,
      result.last_full_pose_time_seconds);
  EXPECT_EQ(StatusCode::kDuplicateTimestamp,
            owner.SubmitMapObservation(delayed, Qualification()).code());
  auto too_old = Map(4);
  too_old.context.receive_time.nanoseconds +=
      Config().maximum_observation_age_ns + 1'000'000;
  EXPECT_EQ(StatusCode::kInvalidTimestamp,
            owner.SubmitMapObservation(too_old, Qualification()).code());
  auto before_anchor = Map(1);
  before_anchor.context.receive_time = Odom(5).odometry.timestamp;
  EXPECT_EQ(StatusCode::kOutsideHistory,
            owner.SubmitMapObservation(before_anchor, Qualification()).code());
}

TEST(FixedLagAlignmentTest, FourArmsShareImmutableOdomWithoutSyntheticAnchors) {
  for (const bool use_map : {false, true}) {
    for (const bool use_gnss : {false, true}) {
      SCOPED_TRACE(::testing::Message()
                   << "map=" << use_map << ",gnss=" << use_gnss);
      FixedLagGlobalAlignment owner;
      ASSERT_TRUE(owner.Configure(Config()).ok());
      for (int index = 0; index <= 5; ++index) {
        const auto input = Odom(index);
        const auto saved_pose = input.odometry.pose;
        const Matrix15d saved_covariance = input.covariance;
        const auto connection =
            index == 0 ? GraphMotionConnection{} : Connection(index);
        ASSERT_TRUE(
            owner.AddOdometry(input, index == 0 ? nullptr : &connection).ok());
        if (use_map) {
          const auto status =
              owner.SubmitMapObservation(Map(index), Qualification());
          if (index < 2) {
            EXPECT_EQ(StatusCode::kRejectedMeasurement, status.code());
          } else {
            ASSERT_TRUE(status.ok()) << status.message();
          }
        }
        if (use_gnss) {
          GlobalHorizontalObservation fix;
          fix.context = Context(index, "independent-gnss");
          fix.position = Eigen::Vector2d(100.0 + 0.1 * index, 20.02);
          fix.covariance = Eigen::Matrix2d::Identity() * 0.01;
          fix.covariance_source = "synthetic-xy";
          ASSERT_TRUE(
              owner.SubmitHorizontalObservation(fix, Qualification()).ok());
        }
        EXPECT_TRUE(input.odometry.pose.translation.isApprox(
            saved_pose.translation, 0.0));
        EXPECT_TRUE(input.odometry.pose.rotation.coeffs().isApprox(
            saved_pose.rotation.coeffs(), 0.0));
        EXPECT_TRUE(input.covariance.isApprox(saved_covariance, 0.0));
      }
      FixedLagAlignmentResult result;
      ASSERT_TRUE(owner.GetResult(Odom(5).odometry.timestamp, &result).ok());
      EXPECT_EQ(use_map, result.estimate.has_value());
      if (use_map) {
        EXPECT_TRUE(result.odometry_fresh);
        EXPECT_TRUE(result.full_pose_evidence_fresh);
        EXPECT_NEAR(100.0, result.estimate->map_from_odom.translation.x(),
                    0.01);
        EXPECT_NEAR(20.0, result.estimate->map_from_odom.translation.y(), 0.02);
      } else {
        EXPECT_FALSE(result.full_pose_evidence_fresh);
        EXPECT_FALSE(result.assessment.ok());
      }
      uint64_t submitted = 0;
      for (const auto& source : result.sources) {
        EXPECT_EQ(source.submitted,
                  source.alignment_updates + source.retained + source.rejected);
        submitted += source.submitted;
        if (!use_map) {
          EXPECT_EQ(0u, source.alignment_updates);
        }
      }
      EXPECT_EQ(result.submitted_observations, submitted);
      EXPECT_EQ(static_cast<uint64_t>(6 * (use_map + use_gnss)), submitted);
    }
  }
}

}  // namespace
}  // namespace localization
}  // namespace openloc
