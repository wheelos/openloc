// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "candidates/global/map_observation_adapter.h"

#include "gtest/gtest.h"

#include "candidates/global/global_alignment.h"

namespace openloc {
namespace localization {
namespace unified {
namespace {

constexpr int64_t kTimestampNs = 1791440875499901000LL;

LocalState Local(int64_t timestamp_ns, uint64_t sequence) {
  LocalState local;
  local.stamp = {static_cast<double>(timestamp_ns) * 1e-9,
                 static_cast<double>(timestamp_ns) * 1e-9, sequence, "unix"};
  local.epoch = {"map-adapter-test", 1};
  local.position = Eigen::Vector3d(1.0, 2.0, 0.5);
  local.covariance = Matrix15d::Identity() * 0.01;
  local.valid = true;
  return local;
}

MapObservationContract Contract() {
  MapObservationContract contract;
  contract.map_id = "map";
  contract.map_version = "v1";
  contract.map_content_id = "sha256:map";
  contract.map_frame = "map";
  contract.base_frame = "base_link";
  contract.calibration_id = "calibration";
  return contract;
}

map::ScanToMapResult Matched(int64_t timestamp_ns) {
  map::ScanToMapResult match;
  match.status = map::MatchStatus::kMatched;
  match.valid = true;
  match.converged = true;
  match.timestamp_ns = timestamp_ns;
  match.clock_domain = "unix";
  match.map_id = "map";
  match.map_version = "v1";
  match.map_content_id = "sha256:map";
  match.map_frame = "map";
  match.base_frame = "base_link";
  match.calibration_id = "calibration";
  match.score_type = "point_to_plane_rmse_m";
  match.covariance_source = "scan_matcher_residual_scaled_hessian";
  match.covariance = Matrix6d::Identity() * 0.01;
  match.covariance_calibrated = false;
  match.map_from_base = Eigen::Isometry3d::Identity();
  match.geometry_full_rank = true;
  return match;
}

GlobalConfig OwnerPolicy(bool diagnostic_only = true) {
  GlobalConfig config;
  config.map_id = "map";
  config.map_version = "v1";
  config.map_content_id = "sha256:map";
  config.map_frame = "map";
  config.base_frame = "base_link";
  config.calibration_id = "calibration";
  config.diagnostic_only = diagnostic_only;
  return config;
}

TEST(MapObservationAdapterTest, CarriesIdentityAndUnqualifiedCovariance) {
  const auto local = Local(kTimestampNs, 1);
  auto match = Matched(kTimestampNs);
  GlobalObservation observation;
  ASSERT_TRUE(AdaptScanToMapObservation(match, local, Contract(),
                                        MapObservationTiming{kTimestampNs, 17},
                                        &observation)
                  .ok());
  EXPECT_EQ(GlobalObservation::Kind::POSE, observation.kind);
  EXPECT_EQ(17u, observation.stamp.sequence);
  ASSERT_TRUE(observation.measurement_timestamp_ns.has_value());
  EXPECT_EQ(kTimestampNs, *observation.measurement_timestamp_ns);
  ASSERT_TRUE(observation.receive_timestamp_ns.has_value());
  EXPECT_EQ(kTimestampNs, *observation.receive_timestamp_ns);
  EXPECT_EQ(local.epoch, observation.epoch);
  EXPECT_EQ("sha256:map", observation.map_content_id);
  EXPECT_EQ("map", observation.map_frame);
  EXPECT_EQ("base_link", observation.base_frame);
  EXPECT_TRUE(observation.quality_valid);
  EXPECT_TRUE(observation.diagnostic_only);
  EXPECT_EQ("scan_matcher_residual_scaled_hessian",
            observation.covariance_source);
  EXPECT_FALSE(observation.uncertainty_qualified);
  EXPECT_FALSE(observation.independent_of_local);
}

TEST(MapObservationAdapterTest, PreservesMeasurementArrivalAndSourceIdentity) {
  const auto local = Local(kTimestampNs, 1);
  // A real offset within the legacy association budget must not be retimed.
  const auto match = Matched(kTimestampNs + 500);
  const MapObservationTiming timing{kTimestampNs + 20000000, 71};
  GlobalObservation observation;
  ASSERT_TRUE(
      AdaptScanToMapObservation(match, local, Contract(), timing, &observation)
          .ok());
  EXPECT_EQ(kTimestampNs + 500, *observation.measurement_timestamp_ns);
  EXPECT_EQ(timing.receive_timestamp_ns, *observation.receive_timestamp_ns);
  EXPECT_EQ(timing.source_sequence, observation.stamp.sequence);
  EXPECT_EQ(static_cast<double>(match.timestamp_ns) * 1e-9,
            observation.stamp.time);
  EXPECT_NE(local.stamp.time, observation.stamp.time);
  EXPECT_EQ(static_cast<double>(timing.receive_timestamp_ns) * 1e-9,
            observation.stamp.receive_time);
}

TEST(MapObservationAdapterTest, InvalidGeometryAndTimingLeaveOutputUntouched) {
  const auto local = Local(kTimestampNs, 1);
  GlobalObservation observation;
  observation.source = "previous";
  auto match = Matched(kTimestampNs);
  match.map_from_base.linear()(0, 0) = 2.0;
  EXPECT_EQ(Reason::INVALID_INPUT,
            AdaptScanToMapObservation(match, local, Contract(),
                                      MapObservationTiming{kTimestampNs, 17},
                                      &observation)
                .reason);
  match = Matched(kTimestampNs);
  match.covariance.setZero();
  EXPECT_EQ(Reason::INVALID_INPUT,
            AdaptScanToMapObservation(match, local, Contract(),
                                      MapObservationTiming{kTimestampNs, 17},
                                      &observation)
                .reason);
  match = Matched(kTimestampNs);
  EXPECT_EQ(Reason::CLOCK_INVALID,
            AdaptScanToMapObservation(match, local, Contract(),
                                      MapObservationTiming{0, 1}, &observation)
                .reason);
  EXPECT_EQ("previous", observation.source);
}

TEST(MapObservationAdapterTest, ValidatesAndRetainsOnlyObservedSubspace) {
  auto local = Local(kTimestampNs, 1);
  auto match = Matched(kTimestampNs);
  match.geometry_full_rank = false;
  match.observability_available = true;
  match.observed_projection = Eigen::MatrixXd::Zero(1, 6);
  match.observed_projection(0, 0) = 1.0;
  match.observed_covariance = Eigen::MatrixXd::Identity(1, 1) * 0.01;
  match.covariance.setZero();
  GlobalObservation observation;
  ASSERT_TRUE(AdaptScanToMapObservation(match, local, Contract(),
                                        MapObservationTiming{kTimestampNs, 17},
                                        &observation)
                  .ok());
  EXPECT_EQ(GlobalObservation::Kind::PROJECTED_POSE, observation.kind);
  EXPECT_TRUE(observation.projection.isApprox(match.observed_projection));
  EXPECT_TRUE(
      observation.projected_covariance.isApprox(match.observed_covariance));
  match.observed_projection.setZero();
  EXPECT_EQ(Reason::INVALID_INPUT,
            AdaptScanToMapObservation(match, local, Contract(),
                                      MapObservationTiming{kTimestampNs, 17},
                                      &observation)
                .reason);
  match.observed_projection(0, 0) = 1.0;
  match.observed_covariance(0, 0) = -0.01;
  EXPECT_EQ(Reason::INVALID_INPUT,
            AdaptScanToMapObservation(match, local, Contract(),
                                      MapObservationTiming{kTimestampNs, 17},
                                      &observation)
                .reason);
}

TEST(MapObservationAdapterTest, DelayedMatchUsesHistoryAndActualArrival) {
  GlobalAlignment owner(OwnerPolicy());
  const auto first = Local(kTimestampNs, 1);
  ASSERT_TRUE(owner.AddLocal(first).ok());
  ASSERT_TRUE(owner
                  .SeedDiagnostic(Eigen::Isometry3d::Identity(),
                                  Matrix6d::Identity(), first.stamp.time)
                  .ok());
  const auto latest = Local(kTimestampNs + 20000000, 2);
  ASSERT_TRUE(owner.AddLocal(latest).ok());
  const auto match = Matched(kTimestampNs + 10000000);
  LocalState associated;
  ASSERT_TRUE(
      owner.Lookup(static_cast<double>(match.timestamp_ns) * 1e-9, &associated)
          .ok());
  GlobalObservation observation;
  auto measured = match;
  measured.map_from_base = Pose(associated);
  ASSERT_TRUE(AdaptScanToMapObservation(
                  measured, associated, Contract(),
                  MapObservationTiming{kTimestampNs + 20000000, 71},
                  &observation)
                  .ok());
  ASSERT_TRUE(owner.Observe(observation).ok());
  const auto revision = owner.state().correction_id;
  EXPECT_EQ(Reason::DUPLICATE, owner.Observe(observation).reason);
  EXPECT_EQ(revision, owner.state().correction_id);
  EXPECT_EQ(1u, owner.sources().at("map").accepted);
  LocalState retained;
  ASSERT_TRUE(owner.Lookup(latest.stamp.time, &retained).ok());
  EXPECT_TRUE(retained.position.isApprox(latest.position, 0.0));
  EXPECT_TRUE(retained.covariance.isApprox(latest.covariance, 0.0));
  observation.stamp.sequence = 72;
  observation.stamp.time += 0.001;
  EXPECT_EQ(Reason::INVALID_INPUT, owner.Observe(observation).reason);
  EXPECT_EQ(revision, owner.state().correction_id);
}

TEST(MapObservationAdapterTest, ProjectedMapAndGnssUseOneRevisionStream) {
  GlobalAlignment owner(OwnerPolicy());
  auto first = Local(kTimestampNs, 1);
  first.position.setZero();
  ASSERT_TRUE(owner.AddLocal(first).ok());
  ASSERT_TRUE(owner
                  .SeedDiagnostic(Eigen::Isometry3d::Identity(),
                                  Matrix6d::Identity(), first.stamp.time)
                  .ok());
  auto next = first;
  next.stamp.time = static_cast<double>(kTimestampNs + 10000000) * 1e-9;
  next.stamp.receive_time = next.stamp.time;
  next.stamp.sequence = 2;
  ASSERT_TRUE(owner.AddLocal(next).ok());
  auto match = Matched(kTimestampNs + 10000000);
  match.map_from_base.translation() = Eigen::Vector3d(0.2, 1000.0, 0.0);
  match.geometry_full_rank = false;
  match.observability_available = true;
  match.observed_projection = Eigen::MatrixXd::Zero(1, 6);
  match.observed_projection(0, 0) = 1.0;
  match.observed_covariance = Eigen::MatrixXd::Identity(1, 1) * 0.01;
  GlobalObservation observation;
  ASSERT_TRUE(AdaptScanToMapObservation(
                  match, next, Contract(),
                  MapObservationTiming{kTimestampNs + 10000000, 10},
                  &observation)
                  .ok());
  const auto before = owner.Predict(next.stamp.time);
  ASSERT_TRUE(owner.Observe(observation).ok());
  EXPECT_EQ(before.correction_id + 1, owner.state().correction_id);
  EXPECT_GT(owner.state().map_to_odom.translation().x(), 0.0);
  EXPECT_DOUBLE_EQ(0.0, owner.state().map_to_odom.translation().y());
  EXPECT_NEAR(before.covariance(1, 1), owner.state().covariance(1, 1), 1e-12);
  EXPECT_EQ(first.stamp.time, owner.state().last_full_observation);

  // GNSS arrives without another map update; it uses the same owner.
  observation = GlobalObservation{};
  observation.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
  observation.stamp = next.stamp;
  observation.source = "gnss";
  observation.epoch = next.epoch;
  observation.map_id = "map";
  observation.map_version = "v1";
  observation.map_content_id = "sha256:map";
  observation.map_frame = "map";
  observation.base_frame = "base_link";
  observation.calibration_id = "calibration";
  observation.quality_valid = true;
  observation.diagnostic_only = true;
  observation.pose.translation() = Eigen::Vector3d(0.2, 0.1, 0.0);
  observation.covariance.topLeftCorner<2, 2>() =
      Eigen::Matrix2d::Identity() * 0.01;
  const auto revision = owner.state().correction_id;
  ASSERT_TRUE(owner.Observe(observation).ok());
  EXPECT_EQ(revision + 1, owner.state().correction_id);
  EXPECT_EQ(1u, owner.sources().at("map").accepted);
  EXPECT_EQ(1u, owner.sources().at("gnss").accepted);
  EXPECT_EQ(first.stamp.time, owner.state().last_full_observation);
  LocalState retained;
  ASSERT_TRUE(owner.Lookup(first.stamp.time, &retained).ok());
  EXPECT_TRUE(retained.covariance.isApprox(first.covariance, 0.0));
  EXPECT_TRUE(retained.position.isApprox(first.position, 0.0));
  EXPECT_FALSE(owner.Evaluate(next.stamp.time).ok());
}

TEST(MapObservationAdapterTest, RejectsFailedMismatchedAndUntimedMatches) {
  const auto local = Local(kTimestampNs, 1);
  GlobalObservation observation;
  auto match = Matched(kTimestampNs);
  match.status = map::MatchStatus::kRejected;
  EXPECT_EQ(Reason::MATCH_FAILED,
            AdaptScanToMapObservation(match, local, Contract(),
                                      MapObservationTiming{kTimestampNs, 17},
                                      &observation)
                .reason);

  match = Matched(kTimestampNs);
  match.map_content_id = "different-map";
  EXPECT_EQ(Reason::MAP_MISMATCH,
            AdaptScanToMapObservation(match, local, Contract(),
                                      MapObservationTiming{kTimestampNs, 17},
                                      &observation)
                .reason);

  match = Matched(kTimestampNs);
  auto bad_calibration = Contract();
  bad_calibration.calibration_id = "stale-calibration";
  EXPECT_EQ(Reason::MAP_MISMATCH,
            AdaptScanToMapObservation(match, local, bad_calibration,
                                      MapObservationTiming{kTimestampNs, 17},
                                      &observation)
                .reason);

  match = Matched(kTimestampNs + 10000);
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            AdaptScanToMapObservation(match, local, Contract(),
                                      MapObservationTiming{kTimestampNs, 17},
                                      &observation)
                .reason);
}

TEST(MapObservationAdapterTest, FeedsSingleOwnerWithoutChangingOdomHistory) {
  GlobalAlignment owner(OwnerPolicy());

  const auto first = Local(kTimestampNs, 1);
  ASSERT_TRUE(owner.AddLocal(first).ok());
  ASSERT_TRUE(owner
                  .SeedDiagnostic(Eigen::Isometry3d::Identity(),
                                  Matrix6d::Identity(), first.stamp.time)
                  .ok());

  constexpr int64_t kNextTimestampNs = kTimestampNs + 10000000;
  const auto next = Local(kNextTimestampNs, 2);
  ASSERT_TRUE(owner.AddLocal(next).ok());
  auto match = Matched(kNextTimestampNs);
  match.map_from_base = Pose(next);
  auto contract = Contract();
  GlobalObservation observation;
  ASSERT_TRUE(AdaptScanToMapObservation(
                  match, next, contract,
                  MapObservationTiming{kNextTimestampNs, 18}, &observation)
                  .ok());
  ASSERT_TRUE(owner.Observe(observation).ok());

  const uint64_t correction_id = owner.state().correction_id;
  constexpr int64_t kRejectedTimestampNs = kNextTimestampNs + 10000000;
  const auto rejected_local = Local(kRejectedTimestampNs, 3);
  ASSERT_TRUE(owner.AddLocal(rejected_local).ok());
  match = Matched(kRejectedTimestampNs);
  match.map_from_base = Pose(rejected_local);
  ASSERT_TRUE(AdaptScanToMapObservation(
                  match, rejected_local, contract,
                  MapObservationTiming{kRejectedTimestampNs, 19}, &observation)
                  .ok());
  observation.map_content_id = "stale-map-content";
  EXPECT_EQ(Reason::MAP_MISMATCH, owner.Observe(observation).reason);
  EXPECT_EQ(correction_id, owner.state().correction_id);

  LocalState retained;
  ASSERT_TRUE(owner.Lookup(next.stamp.time, &retained).ok());
  EXPECT_TRUE(retained.position.isApprox(next.position, 0.0));
  EXPECT_TRUE(
      retained.orientation.coeffs().isApprox(next.orientation.coeffs(), 0.0));
  EXPECT_TRUE(retained.covariance.isApprox(next.covariance, 0.0));
  EXPECT_TRUE(owner.state().diagnostic_only);
  EXPECT_FALSE(owner.Evaluate(next.stamp.time).ok());
}

TEST(MapObservationAdapterTest, DiagnosticMapEvidenceCannotEnterTrustedOwner) {
  GlobalAlignment owner(OwnerPolicy(false));
  auto local = Local(kTimestampNs, 1);
  local.uncertainty_qualified = true;
  ASSERT_TRUE(owner.AddLocal(local).ok());

  auto match = Matched(kTimestampNs);
  match.calibration_state = map::CalibrationState::kCalibrated;
  match.covariance_calibrated = true;
  match.diagnostic_only = true;
  GlobalObservation observation;
  ASSERT_TRUE(AdaptScanToMapObservation(match, local, Contract(),
                                        MapObservationTiming{kTimestampNs, 17},
                                        &observation)
                  .ok());
  EXPECT_TRUE(observation.diagnostic_only);
  EXPECT_FALSE(observation.uncertainty_qualified);
  EXPECT_EQ(Reason::UNCERTAINTY_UNQUALIFIED, owner.Observe(observation).reason);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace openloc
