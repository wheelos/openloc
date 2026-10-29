// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "openloc/localization/global/diagnostic_alignment.h"

#include <cmath>

#include "Eigen/Cholesky"
#include "gtest/gtest.h"

namespace openloc {
namespace localization {
namespace {

constexpr int64_t kStart = 10'000'000'000LL;
constexpr int64_t kStep = 10'000'000LL;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

DiagnosticAlignmentConfig Config() {
  DiagnosticAlignmentConfig config;
  config.reference = {"site", "v1", "calibration"};
  config.reference.geodetic.state = GeodeticReferenceState::kDiagnosticOnly;
  config.map_content_id = "sha256:map";
  config.map_frame = {"map"};
  config.odom_frame = {"odom"};
  config.base_frame = {"body"};
  return config;
}

OdometrySample Odom(int index, uint64_t generation = 1) {
  OdometrySample sample;
  sample.timestamp = {kStart + index * kStep, ClockDomain::kUnixEpoch};
  sample.epoch = {"local-producer", generation};
  sample.parent_frame = {"odom"};
  sample.child_frame = {"body"};
  sample.pose.translation.x() = index;
  return sample;
}

GlobalObservationContext Context(int index, const std::string& source = "map") {
  const auto config = Config();
  GlobalObservationContext context;
  context.measurement_time = Odom(index).timestamp;
  context.receive_time = context.measurement_time;
  context.sequence = index + 1;
  context.source = source;
  context.epoch = Odom(index).epoch;
  context.reference = config.reference;
  context.map_content_id = config.map_content_id;
  context.map_frame = config.map_frame;
  context.base_frame = config.base_frame;
  return context;
}

GlobalMapObservation Map(int index) {
  GlobalMapObservation observation;
  observation.context = Context(index);
  observation.map_from_base.translation =
      Eigen::Vector3d(100.0 + index, 20.0, 0.0);
  observation.covariance = Matrix6d::Identity() * 0.01;
  observation.covariance_source = "synthetic-diagnostic";
  return observation;
}

void Bootstrap(DiagnosticGlobalAlignment* owner) {
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(owner->AddOdometry(Odom(i), Matrix6d::Identity() * 0.01).ok());
    const Status status = owner->SubmitMapObservation(Map(i));
    if (i < 2) {
      EXPECT_EQ(StatusCode::kRejectedMeasurement, status.code());
    } else {
      EXPECT_TRUE(status.ok()) << status.message();
    }
  }
}

TEST(DiagnosticAlignmentTest, QueuedHeadingAndPositionAcquireOnlyHorizontalState) {
  auto config = Config();
  config.enable_heading_bootstrap = true;
  DiagnosticGlobalAlignment owner;
  EXPECT_EQ(StatusCode::kInvalidArgument, owner.Configure(config).code());
  config.assume_gravity_aligned_odom = true;
  ASSERT_TRUE(owner.Configure(config).ok());
  GlobalHeadingObservation heading;
  heading.context = Context(0, "heading");
  heading.heading_rad = 0.35;
  heading.variance_rad2 = 0.0001;
  heading.covariance_source = "receiver-heading";
  EXPECT_EQ(StatusCode::kPending, owner.SubmitHeadingObservation(heading).code());
  ASSERT_TRUE(owner.AddOdometry(Odom(0), Matrix6d::Identity() * 0.01).ok());
  GlobalHorizontalObservation position;
  position.context = Context(0, "position");
  position.position = Eigen::Vector2d(100.0, 20.0);
  position.covariance = Eigen::Matrix2d::Identity() * 0.01;
  position.covariance_source = "receiver-position";
  ASSERT_TRUE(owner.SubmitHorizontalObservation(position).ok());
  DiagnosticAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(&result).ok());
  ASSERT_TRUE(result.horizontal_estimate);
  EXPECT_TRUE(result.horizontal_estimate->map_base_xy.isApprox(position.position));
  EXPECT_NEAR(0.35, result.horizontal_estimate->map_from_odom_xy_yaw.z(), 1e-10);
  EXPECT_FALSE(result.estimate);
  EXPECT_FALSE(result.full_pose_evidence_fresh);
  EXPECT_TRUE(result.horizontal_evidence_fresh);
  EXPECT_EQ(1u, result.decisions.back().correction_id);
  ASSERT_TRUE(owner.AddOdometry(Odom(1, 2), Matrix6d::Identity() * 0.01).ok());
  ASSERT_TRUE(owner.GetResult(&result).ok());
  EXPECT_FALSE(result.horizontal_estimate);
}

TEST(DiagnosticAlignmentTest, HeadingCorrectsBasinWithoutRenewingFullPose) {
  DiagnosticGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  GlobalHeadingObservation heading;
  heading.context = Context(2, "heading");
  heading.heading_rad = 0.03;
  heading.variance_rad2 = 0.0001;
  heading.covariance_source = "receiver-heading";
  ASSERT_TRUE(owner.SubmitHeadingObservation(heading).ok());
  DiagnosticAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(&result).ok());
  ASSERT_TRUE(result.estimate);
  EXPECT_EQ(2u, result.estimate->correction_id);
  EXPECT_GT(result.estimate->map_from_base.rotation.toRotationMatrix()(1, 0), 0.0);
  ASSERT_TRUE(result.last_full_pose_time);
  EXPECT_EQ(Odom(2).timestamp.nanoseconds, result.last_full_pose_time->nanoseconds);
  EXPECT_TRUE(result.decisions.back().alignment_updated);
}

TEST(DiagnosticAlignmentTest, InitializesFromMapEvidenceNotAnOraclePrior) {
  DiagnosticGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  DiagnosticAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(&result).ok());
  ASSERT_TRUE(result.estimate.has_value());
  EXPECT_NEAR(100.0, result.estimate->map_from_odom.translation.x(), 1e-9);
  EXPECT_NEAR(102.0, result.estimate->map_from_base.translation.x(), 1e-9);
  EXPECT_EQ(1u, result.estimate->correction_id);
  EXPECT_EQ(Config().reference, result.reference);
  EXPECT_EQ("sha256:map", result.map_content_id);
  ASSERT_EQ(3u, result.decisions.size());
  EXPECT_TRUE(result.decisions.back().status.ok());
  EXPECT_TRUE(result.decisions.back().alignment_updated);
  EXPECT_EQ(1u, result.decisions.back().correction_id);
  EXPECT_EQ(3u, result.decisions.back().decision_id);
}

TEST(DiagnosticAlignmentTest, PositionEvidenceDoesNotInventAnAlignment) {
  for (const bool fixed_gain : {false, true}) {
    SCOPED_TRACE(fixed_gain);
    DiagnosticGlobalAlignment owner;
    auto config = Config();
    config.use_fixed_gain_update = fixed_gain;
    ASSERT_TRUE(owner.Configure(config).ok());
    ASSERT_TRUE(owner.AddOdometry(Odom(0), Matrix6d::Identity()).ok());
    GlobalHorizontalObservation fix;
    fix.context = Context(0, "gnss");
    fix.position = Eigen::Vector2d(100.0, 20.0);
    fix.covariance = Eigen::Matrix2d::Identity();
    fix.covariance_source = "assumed-position-noise";
    ASSERT_TRUE(owner.SubmitHorizontalObservation(fix).ok());
    DiagnosticAlignmentResult result;
    ASSERT_TRUE(owner.GetResult(&result).ok());
    EXPECT_FALSE(result.estimate.has_value());
    ASSERT_EQ(1u, result.decisions.size());
    EXPECT_TRUE(result.decisions.back().status.ok());
    EXPECT_FALSE(result.decisions.back().alignment_updated);
    EXPECT_EQ(0u, result.decisions.back().correction_id);
  }
}

GlobalHorizontalObservation HorizontalFix(int index, const OdometrySample& odom) {
  GlobalHorizontalObservation fix;
  fix.context = Context(index, "gnss");
  fix.context.epoch = odom.epoch;
  fix.point_in_base = Eigen::Vector3d(1.0, -0.5, 0.3);
  const Eigen::Vector2d antenna =
      (odom.pose.translation + odom.pose.rotation * fix.point_in_base).head<2>();
  fix.position =
      Eigen::Rotation2Dd(0.35) * antenna + Eigen::Vector2d(100.0, 20.0);
  fix.covariance = Eigen::Matrix2d::Identity() * 0.01;
  fix.covariance_source = "uncalibrated-synthetic-xy";
  return fix;
}

TEST(DiagnosticAlignmentTest, HorizontalAcquisitionRequiresExplicitLevelAssumption) {
  auto config = Config();
  config.enable_horizontal_bootstrap = true;
  DiagnosticGlobalAlignment owner;
  EXPECT_EQ(StatusCode::kInvalidArgument, owner.Configure(config).code());
  config.assume_gravity_aligned_odom = true;
  ASSERT_TRUE(owner.Configure(config).ok());
}

TEST(DiagnosticAlignmentTest, HorizontalAcquisitionTracksWithoutInventingFullPose) {
  auto config = Config();
  config.enable_horizontal_bootstrap = true;
  config.assume_gravity_aligned_odom = true;
  DiagnosticGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(config).ok());
  DiagnosticAlignmentResult result;
  for (int i = 0; i <= 11; ++i) {
    auto odom = Odom(i);
    odom.pose.translation.y() = 0.03 * i * i;
    odom.pose.translation.z() = 17.0 + i * 0.2;
    odom.pose.rotation = Eigen::Quaterniond(
        Eigen::AngleAxisd(0.02 * i, Eigen::Vector3d::UnitY()));
    const auto original = odom.pose;
    ASSERT_TRUE(owner.AddOdometry(odom, Matrix6d::Identity() * 0.01).ok());
    const auto fix = HorizontalFix(i, odom);
    const auto status = owner.SubmitHorizontalObservation(fix);
    ASSERT_TRUE(status.ok()) << status.message();
    ASSERT_TRUE(owner.GetResult(&result).ok());
    EXPECT_FALSE(result.estimate.has_value());
    EXPECT_FALSE(result.last_full_pose_time.has_value());
    EXPECT_FALSE(result.full_pose_evidence_fresh);
    EXPECT_TRUE(odom.pose.translation.isApprox(original.translation, 0.0));
    EXPECT_EQ(i >= 10, result.horizontal_estimate.has_value());
    EXPECT_EQ(i >= 10, result.decisions.back().alignment_updated);
    if (i < 10) continue;
    const auto& horizontal = *result.horizontal_estimate;
    EXPECT_NEAR(100.0, horizontal.map_from_odom_xy_yaw.x(), 1e-8);
    EXPECT_NEAR(20.0, horizontal.map_from_odom_xy_yaw.y(), 1e-8);
    EXPECT_NEAR(0.35, horizontal.map_from_odom_xy_yaw.z(), 1e-8);
    EXPECT_TRUE(horizontal.map_base_xy.isApprox(
        Eigen::Rotation2Dd(0.35) * odom.pose.translation.head<2>() +
            Eigen::Vector2d(100.0, 20.0),
        1e-8));
    EXPECT_TRUE(result.horizontal_evidence_fresh);
    EXPECT_EQ(static_cast<uint64_t>(i - 9), horizontal.correction_id);
    EXPECT_EQ(odom.timestamp.nanoseconds,
              horizontal.last_observation_time.nanoseconds);
  }
  EXPECT_EQ("diagnostic_horizontal_updated",
            result.decisions.back().status.message());
  auto now = Odom(11).timestamp;
  now.nanoseconds += config.maximum_full_pose_age_ns + 1;
  ASSERT_TRUE(owner.GetResult(now, &result).ok());
  EXPECT_TRUE(result.horizontal_estimate.has_value());
  EXPECT_FALSE(result.horizontal_evidence_fresh);
  EXPECT_FALSE(result.odometry_fresh);
  EXPECT_FALSE(result.estimate.has_value());

  ASSERT_TRUE(owner.AddOdometry(Odom(12), Matrix6d::Identity() * 0.01).ok());
  auto outlier = HorizontalFix(12, Odom(12));
  outlier.position.array() += 1000.0;
  EXPECT_EQ(StatusCode::kRejectedMeasurement,
            owner.SubmitHorizontalObservation(outlier).code());
  ASSERT_TRUE(owner.GetResult(&result).ok());
  EXPECT_EQ(2u, result.horizontal_estimate->correction_id);
  EXPECT_FALSE(result.decisions.back().alignment_updated);

  for (int i = 13; i <= 15; ++i) {
    ASSERT_TRUE(owner.AddOdometry(Odom(i), Matrix6d::Identity() * 0.01).ok());
    const auto status = owner.SubmitMapObservation(Map(i));
    EXPECT_EQ(i == 15, status.ok());
  }
  ASSERT_TRUE(owner.GetResult(&result).ok());
  ASSERT_TRUE(result.estimate.has_value());
  EXPECT_FALSE(result.horizontal_estimate.has_value());
  EXPECT_TRUE(result.full_pose_evidence_fresh);
  EXPECT_EQ(3u, result.estimate->correction_id);
}

TEST(DiagnosticAlignmentTest, HorizontalStationarityAndFailedHoldoutCannotAcquire) {
  auto config = Config();
  config.enable_horizontal_bootstrap = true;
  config.assume_gravity_aligned_odom = true;
  DiagnosticGlobalAlignment stationary;
  ASSERT_TRUE(stationary.Configure(config).ok());
  DiagnosticAlignmentResult result;
  for (int i = 0; i < 12; ++i) {
    auto odom = Odom(i);
    odom.pose.translation.setZero();
    ASSERT_TRUE(stationary.AddOdometry(odom, Matrix6d::Identity()).ok());
    ASSERT_TRUE(stationary.SubmitHorizontalObservation(HorizontalFix(i, odom)).ok());
  }
  ASSERT_TRUE(stationary.GetResult(&result).ok());
  EXPECT_FALSE(result.horizontal_estimate.has_value());
  EXPECT_EQ("recovery_yaw_unobservable", result.decisions.back().status.message());

  DiagnosticGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(config).ok());
  for (int i = 0; i <= 10; ++i) {
    ASSERT_TRUE(owner.AddOdometry(Odom(i), Matrix6d::Identity()).ok());
    auto fix = HorizontalFix(i, Odom(i));
    if (i == 8) fix.position.x() += 100.0;
    ASSERT_TRUE(owner.SubmitHorizontalObservation(fix).ok());
    if (i == 8) {
      ASSERT_TRUE(owner.GetResult(&result).ok());
      EXPECT_EQ("recovery_holdout_inconsistent",
                result.decisions.back().status.message());
      EXPECT_EQ(StatusCode::kDuplicateTimestamp,
                owner.SubmitHorizontalObservation(fix).code());
    }
  }
  ASSERT_TRUE(owner.GetResult(&result).ok());
  EXPECT_FALSE(result.horizontal_estimate.has_value());
  EXPECT_FALSE(result.estimate.has_value());
}

TEST(DiagnosticAlignmentTest, HorizontalEpochChangeRetiresAcquisitionAndEstimate) {
  auto config = Config();
  config.enable_horizontal_bootstrap = true;
  config.assume_gravity_aligned_odom = true;
  DiagnosticGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(config).ok());
  for (int i = 0; i <= 22; ++i) {
    const auto odom = Odom(i, i < 10 ? 1 : 2);
    ASSERT_TRUE(owner.AddOdometry(odom, Matrix6d::Identity()).ok());
    ASSERT_TRUE(owner.SubmitHorizontalObservation(HorizontalFix(i, odom)).ok());
    DiagnosticAlignmentResult result;
    ASSERT_TRUE(owner.GetResult(&result).ok());
    EXPECT_EQ(i >= 20, result.horizontal_estimate.has_value());
    EXPECT_FALSE(result.estimate.has_value());
  }
  ASSERT_TRUE(owner.AddOdometry(Odom(23, 3), Matrix6d::Identity()).ok());
  DiagnosticAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(&result).ok());
  EXPECT_FALSE(result.horizontal_estimate.has_value());
  EXPECT_FALSE(result.horizontal_evidence_fresh);
  auto retired = HorizontalFix(23, Odom(23, 2));
  EXPECT_EQ(StatusCode::kEpochMismatch,
            owner.SubmitHorizontalObservation(retired).code());
}

TEST(DiagnosticAlignmentTest, HorizontalLossRequiresNewTrainingAndLaterHoldouts) {
  auto config = Config();
  config.enable_horizontal_bootstrap = true;
  config.assume_gravity_aligned_odom = true;
  DiagnosticGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(config).ok());
  for (int i = 0; i <= 10; ++i) {
    ASSERT_TRUE(owner.AddOdometry(Odom(i), Matrix6d::Identity()).ok());
    ASSERT_TRUE(owner.SubmitHorizontalObservation(HorizontalFix(i, Odom(i))).ok());
  }
  for (int i = 311; i <= 321; ++i) {
    ASSERT_TRUE(owner.AddOdometry(Odom(i), Matrix6d::Identity()).ok());
    auto fix = HorizontalFix(i, Odom(i));
    fix.position += Eigen::Vector2d(40.0, -30.0);
    ASSERT_TRUE(owner.SubmitHorizontalObservation(fix).ok());
    DiagnosticAlignmentResult result;
    ASSERT_TRUE(owner.GetResult(&result).ok());
    EXPECT_EQ(i == 321, result.horizontal_estimate.has_value());
    EXPECT_FALSE(result.estimate.has_value());
    if (i == 321) {
      EXPECT_NEAR(140.0, result.horizontal_estimate->map_from_odom_xy_yaw.x(), 1e-8);
      EXPECT_NEAR(-10.0, result.horizontal_estimate->map_from_odom_xy_yaw.y(), 1e-8);
      EXPECT_EQ(2u, result.horizontal_estimate->correction_id);
      EXPECT_TRUE(result.horizontal_evidence_fresh);
    }
  }
}

TEST(DiagnosticAlignmentTest, QueuedProjectedMapAndGnssShareOneOwner) {
  DiagnosticGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  auto map = Map(3);
  map.kind = GlobalMapObservation::Kind::kProjectedPose;
  map.map_from_base.translation.x() += 0.1;
  map.map_from_base.translation.y() = 5000.0;
  map.projection = Eigen::MatrixXd::Zero(1, 6);
  map.projection(0, 0) = 1.0;
  map.projected_covariance = Eigen::MatrixXd::Identity(1, 1) * 0.01;
  map.context.receive_time = Odom(4).timestamp;
  EXPECT_EQ(StatusCode::kPending, owner.SubmitMapObservation(map).code());
  DiagnosticAlignmentResult queued;
  ASSERT_TRUE(owner.GetResult(&queued).ok());
  EXPECT_EQ(1u, queued.pending_observations);
  EXPECT_EQ(StatusCode::kDuplicateTimestamp,
            owner.SubmitMapObservation(map).code());
  const auto input = Odom(4);
  ASSERT_TRUE(owner.AddOdometry(input, Matrix6d::Identity() * 0.01).ok());
  DiagnosticAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(&result).ok());
  ASSERT_TRUE(result.estimate.has_value());
  const auto revision = result.estimate->correction_id;
  EXPECT_EQ(2u, revision);
  EXPECT_NEAR(20.0, result.estimate->map_from_odom.translation.y(), 1e-9);
  EXPECT_NEAR(4.0,
              result.estimate->map_from_base.translation.x() -
                  result.estimate->map_from_odom.translation.x(),
              1e-9);
  ASSERT_TRUE(result.decisions.back().status.ok());
  EXPECT_EQ(map.context.measurement_time.nanoseconds,
            result.decisions.back().context.measurement_time.nanoseconds);
  EXPECT_EQ(map.context.receive_time.nanoseconds,
            result.decisions.back().context.receive_time.nanoseconds);
  EXPECT_DOUBLE_EQ(4.0, input.pose.translation.x());

  GlobalHorizontalObservation fix;
  fix.context = Context(4, "gnss");
  fix.position = Eigen::Vector2d(104.0, 20.0);
  fix.covariance = Eigen::Matrix2d::Identity() * 0.1;
  fix.covariance_source = "assumed-position-noise";
  ASSERT_TRUE(owner.SubmitHorizontalObservation(fix).ok());
  ASSERT_TRUE(owner.GetResult(&result).ok());
  EXPECT_EQ(revision + 1, result.estimate->correction_id);
}

TEST(DiagnosticAlignmentTest, EpochChangeRetiresQueuedObservations) {
  DiagnosticGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  EXPECT_EQ(StatusCode::kPending, owner.SubmitMapObservation(Map(5)).code());
  ASSERT_TRUE(owner.AddOdometry(Odom(3, 2), Matrix6d::Identity()).ok());
  DiagnosticAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(&result).ok());
  EXPECT_EQ(2u, result.epoch.generation);
  EXPECT_FALSE(result.estimate.has_value());
  EXPECT_FALSE(result.last_full_pose_time.has_value());
  EXPECT_FALSE(result.full_pose_evidence_fresh);
  ASSERT_FALSE(result.decisions.empty());
  EXPECT_EQ(StatusCode::kEpochMismatch, result.decisions.back().status.code());
  EXPECT_EQ(StatusCode::kEpochMismatch,
            owner.SubmitMapObservation(Map(3)).code());
}

TEST(DiagnosticAlignmentTest, RejectsInvalidStaleAndUnboundedInputs) {
  DiagnosticGlobalAlignment owner;
  auto config = Config();
  config.maximum_pending_observations = 1;
  config.maximum_decisions = 2;
  ASSERT_TRUE(owner.Configure(config).ok());
  EXPECT_EQ(StatusCode::kPending, owner.SubmitMapObservation(Map(0)).code());
  EXPECT_EQ(StatusCode::kRejectedMeasurement,
            owner.SubmitMapObservation(Map(1)).code());
  auto wrong = Map(2);
  wrong.context.map_content_id = "stale-map";
  EXPECT_EQ(StatusCode::kMapReferenceMismatch,
            owner.SubmitMapObservation(wrong).code());
  auto stale = Map(2);
  stale.context.receive_time.nanoseconds += 3'000'000'000LL;
  EXPECT_EQ(StatusCode::kInvalidTimestamp,
            owner.SubmitMapObservation(stale).code());
  auto invalid = Map(2);
  invalid.map_from_base.rotation.coeffs().setZero();
  EXPECT_EQ(StatusCode::kInvalidArgument,
            owner.SubmitMapObservation(invalid).code());
  ASSERT_TRUE(owner.AddOdometry(Odom(0), Matrix6d::Identity()).ok());
  DiagnosticAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(&result).ok());
  EXPECT_EQ(2u, result.decisions.size());
  EXPECT_EQ(3u, result.decisions_dropped);
  EXPECT_EQ(4u, result.decisions.front().decision_id);
  EXPECT_EQ(5u, result.decisions.back().decision_id);
  EXPECT_FALSE(result.estimate.has_value());
  EXPECT_EQ(StatusCode::kInvalidArgument, owner.Configure(config).code());
}

TEST(DiagnosticAlignmentTest, ContradictionAndAmbiguityDoNotBypassRecovery) {
  DiagnosticGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  ASSERT_TRUE(owner.AddOdometry(Odom(3), Matrix6d::Identity() * 0.01).ok());
  auto contradictory = Map(3);
  contradictory.map_from_base.translation.x() += 100.0;
  EXPECT_EQ(StatusCode::kRejectedMeasurement,
            owner.SubmitMapObservation(contradictory).code());
  DiagnosticAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(&result).ok());
  ASSERT_TRUE(result.estimate.has_value());
  EXPECT_EQ(1u, result.estimate->correction_id);
  EXPECT_FALSE(result.decisions.back().alignment_updated);

  for (int i = 4; i <= 8; ++i) {
    ASSERT_TRUE(owner.AddOdometry(Odom(i), Matrix6d::Identity() * 0.01).ok());
    auto candidate = Map(i);
    candidate.map_from_base.translation.x() += 50.0;
    candidate.recovery = true;
    candidate.ambiguous = i == 5;
    const Status status = owner.SubmitMapObservation(candidate);
    ASSERT_TRUE(owner.GetResult(&result).ok());
    if (i < 8) {
      EXPECT_EQ(StatusCode::kRejectedMeasurement, status.code());
      EXPECT_FALSE(result.estimate.has_value());
      EXPECT_FALSE(result.full_pose_evidence_fresh);
      EXPECT_EQ(i != 5, result.verification_pending);
    } else {
      ASSERT_TRUE(status.ok()) << status.message();
      ASSERT_TRUE(result.estimate.has_value());
      EXPECT_EQ(2u, result.estimate->correction_id);
      EXPECT_NEAR(150.0, result.estimate->map_from_odom.translation.x(), 1e-9);
      EXPECT_TRUE(result.full_pose_evidence_fresh);
    }
  }
}

TEST(DiagnosticAlignmentTest, PartialEvidenceAndOutagesDoNotRenewFullPoseAge) {
  for (const bool fixed_gain : {false, true}) {
    SCOPED_TRACE(fixed_gain);
    DiagnosticGlobalAlignment owner;
    auto config = Config();
    config.use_fixed_gain_update = fixed_gain;
    ASSERT_TRUE(owner.Configure(config).ok());
    Bootstrap(&owner);
    ASSERT_TRUE(owner.AddOdometry(Odom(3), Matrix6d::Identity() * 0.01).ok());
    GlobalHorizontalObservation fix;
    fix.context = Context(3, "gnss");
    fix.position = Eigen::Vector2d(103.0, 20.0);
    fix.covariance = Eigen::Matrix2d::Identity() * 0.1;
    fix.covariance_source = "synthetic-diagnostic";
    ASSERT_TRUE(owner.SubmitHorizontalObservation(fix).ok());

    DiagnosticAlignmentResult result;
    auto now = Odom(2).timestamp;
    now.nanoseconds += config.maximum_full_pose_age_ns;
    ASSERT_TRUE(owner.GetResult(now, &result).ok());
    ASSERT_TRUE(result.last_full_pose_time.has_value());
    EXPECT_EQ(Odom(2).timestamp.nanoseconds,
              result.last_full_pose_time->nanoseconds);
    EXPECT_TRUE(result.full_pose_evidence_fresh);
    EXPECT_FALSE(result.odometry_fresh);
    EXPECT_EQ(Odom(3).timestamp.nanoseconds, result.timestamp.nanoseconds);
    EXPECT_EQ(now.nanoseconds, result.evaluation_time.nanoseconds);
    ++now.nanoseconds;
    ASSERT_TRUE(owner.GetResult(now, &result).ok());
    EXPECT_FALSE(result.full_pose_evidence_fresh);
    ASSERT_TRUE(result.estimate.has_value());
    EXPECT_EQ(2u, result.estimate->correction_id);

    now = Odom(3).timestamp;
    now.nanoseconds += config.maximum_odometry_age_ns;
    ASSERT_TRUE(owner.GetResult(now, &result).ok());
    EXPECT_TRUE(result.odometry_fresh);
    ++now.nanoseconds;
    ASSERT_TRUE(owner.GetResult(now, &result).ok());
    EXPECT_FALSE(result.odometry_fresh);

    EXPECT_EQ(StatusCode::kInvalidTimestamp,
              owner.GetResult(Odom(2).timestamp, &result).code());
    now.clock_domain = ClockDomain::kSensorMonotonic;
    EXPECT_EQ(StatusCode::kClockDomainMismatch,
              owner.GetResult(now, &result).code());
    EXPECT_FALSE(result.odometry_fresh);
  }
}

TEST(DiagnosticAlignmentTest, LateSourceCannotRewriteCommittedFrontier) {
  for (const bool fixed_gain : {false, true}) {
    SCOPED_TRACE(fixed_gain);
    DiagnosticGlobalAlignment owner;
    auto config = Config();
    config.use_fixed_gain_update = fixed_gain;
    ASSERT_TRUE(owner.Configure(config).ok());
    Bootstrap(&owner);
    ASSERT_TRUE(owner.AddOdometry(Odom(5), Matrix6d::Identity() * 0.01).ok());
    auto current = Map(5);
    current.context.source = "current-map";
    current.map_from_base.translation.y() += 0.05;
    ASSERT_TRUE(owner.SubmitMapObservation(current).ok());
    std::vector<GlobalObservationDecision> decisions;
    ASSERT_TRUE(owner.GetDecisions(&decisions).ok());
    ASSERT_TRUE(decisions.back().innovation.available);
    EXPECT_EQ("map_from_odom", decisions.back().innovation.prior_state_frame);
    EXPECT_NEAR(decisions.back().innovation.nis,
                decisions.back().innovation.residual.dot(
                    decisions.back().innovation.covariance.ldlt().solve(
                        decisions.back().innovation.residual)),
                1e-8);
    const auto& innovation = decisions.back().innovation;
    ASSERT_TRUE(innovation.covariance_decomposition_available);
    EXPECT_TRUE(innovation.covariance.isApprox(
        innovation.prior_contribution + innovation.observation_contribution +
            innovation.local_contribution,
        1e-12));
    EXPECT_TRUE(innovation.prior_contribution.isApprox(
        2 * innovation.prior_covariance, 1e-12));
    auto late = Map(3);
    late.context.source = "delayed-map";
    late.context.receive_time = Odom(5).timestamp;
    EXPECT_EQ(StatusCode::kOutOfOrderTimestamp,
              owner.SubmitMapObservation(late).code());
    DiagnosticAlignmentResult result;
    ASSERT_TRUE(owner.GetResult(&result).ok());
    ASSERT_TRUE(result.estimate.has_value());
    EXPECT_EQ(2u, result.estimate->correction_id);
    EXPECT_FALSE(result.decisions.back().alignment_updated);
    EXPECT_EQ(late.context.measurement_time.nanoseconds,
              result.decisions.back().context.measurement_time.nanoseconds);
  }
}

TEST(DiagnosticAlignmentTest, CorrelatedBoundsCanAmplifySmallRotationResidual) {
  DiagnosticGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Matrix6d prior = Matrix6d::Identity();
  prior(0, 0) = 1e6;
  prior(0, 5) = prior(5, 0) = 990;
  ASSERT_TRUE(prior.llt().info() == Eigen::Success);
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(owner.AddOdometry(Odom(i), Matrix6d::Identity() * 0.01).ok());
    auto observation = Map(i);
    observation.covariance = prior;
    const auto status = owner.SubmitMapObservation(observation);
    if (i == 2) {
      ASSERT_TRUE(status.ok()) << status.message();
    } else {
      EXPECT_EQ(StatusCode::kRejectedMeasurement, status.code());
    }
  }
  const auto odom = Odom(3);
  ASSERT_TRUE(owner.AddOdometry(odom, Matrix6d::Identity() * 0.01).ok());
  auto observation = Map(3);
  observation.covariance = prior;
  observation.covariance(0, 5) = observation.covariance(5, 0) = -990;
  observation.map_from_base.rotation =
      Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ());
  observation.map_from_base.translation =
      Eigen::Vector3d(100, 20, 0) +
      observation.map_from_base.rotation * odom.pose.translation;
  ASSERT_TRUE(observation.covariance.llt().info() == Eigen::Success);
  ASSERT_TRUE(owner.SubmitMapObservation(observation).ok());
  DiagnosticAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(&result).ok());
  ASSERT_TRUE(result.estimate);
  const auto& decision = result.decisions.back();
  ASSERT_TRUE(decision.alignment_updated);
  ASSERT_TRUE(decision.innovation.available);
  const auto& innovation = decision.innovation;
  const Eigen::VectorXd expected =
      2 * innovation.prior_covariance *
      innovation.covariance.ldlt().solve(innovation.residual);
  const Eigen::Vector3d translation =
      result.estimate->map_from_odom.translation -
      innovation.prior_pose.translation;
  EXPECT_TRUE(translation.isApprox(expected.head<3>(), 1e-8));
  EXPECT_GT(translation.norm(), 5);
  EXPECT_LT(innovation.residual.head<3>().norm(), 1e-8);
  EXPECT_LT(innovation.nis, innovation.gate_threshold);
  EXPECT_TRUE(result.full_pose_evidence_fresh);
}

TEST(DiagnosticAlignmentTest, CorrelationOnlyControlSeparatesPoseFromGain) {
  for (const double cross : {0.0, -990.0, 990.0}) {
    SCOPED_TRACE(cross);
    DiagnosticGlobalAlignment owner;
    ASSERT_TRUE(owner.Configure(Config()).ok());
    Matrix6d covariance = Matrix6d::Identity();
    covariance(0, 0) = 1e6;
    covariance(0, 5) = covariance(5, 0) = cross;
    ASSERT_EQ(Eigen::Success, covariance.llt().info());
    for (int i = 0; i < 4; ++i) {
      auto odom = Odom(i);
      odom.pose.translation.setZero();
      ASSERT_TRUE(owner.AddOdometry(odom, Matrix6d::Identity() * 0.01).ok());
      auto observation = Map(i);
      observation.map_from_base.translation = Eigen::Vector3d(100, 20, 0);
      observation.covariance = covariance;
      if (i == 3) {
        observation.covariance(0, 5) = observation.covariance(5, 0) = -cross;
        observation.map_from_base.rotation =
            Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ());
      }
      const auto status = owner.SubmitMapObservation(observation);
      if (i < 2) {
        EXPECT_EQ(StatusCode::kRejectedMeasurement, status.code());
      } else {
        ASSERT_TRUE(status.ok()) << status.message();
      }
    }
    DiagnosticAlignmentResult result;
    ASSERT_TRUE(owner.GetResult(&result).ok());
    ASSERT_TRUE(result.estimate);
    const auto& innovation = result.decisions.back().innovation;
    ASSERT_TRUE(result.decisions.back().alignment_updated);
    ASSERT_TRUE(innovation.available);
    EXPECT_NEAR(0, innovation.residual.head<3>().norm(), 1e-10);
    EXPECT_NEAR(0.02, innovation.residual[5], 1e-10);
    EXPECT_NEAR(0, innovation.covariance(0, 5), 1e-8);
    const Eigen::Vector3d translation =
        result.estimate->map_from_odom.translation -
        innovation.prior_pose.translation;
    const double expected = 2 * innovation.prior_covariance(0, 5) * 0.02 /
                            innovation.covariance(5, 5);
    EXPECT_NEAR(expected, translation.x(), 1e-8);
    EXPECT_NEAR(0, translation.tail<2>().norm(), 1e-8);
    if (cross == 0) {
      EXPECT_NEAR(0, translation.norm(), 1e-8);
    } else {
      EXPECT_GT(cross * translation.x(), 0);
      EXPECT_GT(std::abs(translation.x()), 5);
    }
  }
}

TEST(DiagnosticAlignmentTest, FixedGainDoesNotConvertRotationIntoTranslation) {
  for (const double cross : {-990.0, 0.0, 990.0}) {
    SCOPED_TRACE(cross);
    auto config = Config();
    config.use_fixed_gain_update = true;
    DiagnosticGlobalAlignment owner;
    ASSERT_TRUE(owner.Configure(config).ok());
    Matrix6d covariance = Matrix6d::Identity();
    covariance(0, 0) = 1e6;
    covariance(0, 5) = covariance(5, 0) = cross;
    for (int i = 0; i < 4; ++i) {
      auto odom = Odom(i);
      odom.pose.translation.setZero();
      ASSERT_TRUE(owner.AddOdometry(odom, Matrix6d::Identity() * 0.01).ok());
      auto observation = Map(i);
      observation.map_from_base.translation = Eigen::Vector3d(100, 20, 0);
      observation.covariance = covariance;
      if (i == 3) {
        observation.covariance(0, 5) = observation.covariance(5, 0) = -cross;
        observation.map_from_base.rotation =
            Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ());
      }
      const auto status = owner.SubmitMapObservation(observation);
      if (i >= 2) ASSERT_TRUE(status.ok()) << status.message();
    }
    DiagnosticAlignmentResult result;
    ASSERT_TRUE(owner.GetResult(&result).ok());
    ASSERT_TRUE(result.estimate);
    const auto& innovation = result.decisions.back().innovation;
    EXPECT_EQ("diagnostic_fixed_gain_full_pose_bound",
              innovation.covariance_model);
    EXPECT_TRUE(result.estimate->map_from_odom.translation.isApprox(
        innovation.prior_pose.translation, 1e-12));
    EXPECT_NEAR(
        0.01,
        Eigen::AngleAxisd(result.estimate->map_from_odom.rotation).angle(),
        1e-12);
    EXPECT_TRUE(result.estimate->alignment_covariance.isApprox(
        0.25 * innovation.covariance, 1e-12));
  }
}

TEST(DiagnosticAlignmentTest, FixedGainXyDoesNotInventRotationOrRefreshPose) {
  auto config = Config();
  config.use_fixed_gain_update = true;
  DiagnosticGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(config).ok());
  Bootstrap(&owner);
  DiagnosticAlignmentResult before;
  ASSERT_TRUE(owner.GetResult(&before).ok());
  const auto odom = Odom(3);
  ASSERT_TRUE(owner.AddOdometry(odom, Matrix6d::Identity() * 0.01).ok());
  GlobalHorizontalObservation fix;
  fix.context = Context(3, "gnss");
  fix.position = Eigen::Vector2d(103.2, 19.6);
  fix.covariance = Eigen::Matrix2d::Identity() * 0.01;
  fix.covariance_source = "constructed-xy";
  ASSERT_TRUE(owner.SubmitHorizontalObservation(fix).ok());
  DiagnosticAlignmentResult after;
  ASSERT_TRUE(owner.GetResult(&after).ok());
  ASSERT_TRUE(after.estimate);
  const auto& innovation = after.decisions.back().innovation;
  EXPECT_EQ("diagnostic_fixed_gain_partial_bound", innovation.covariance_model);
  EXPECT_TRUE((after.estimate->map_from_odom.translation -
               before.estimate->map_from_odom.translation)
                  .head<2>()
                  .isApprox(0.5 * innovation.residual, 1e-12));
  EXPECT_EQ(before.estimate->map_from_odom.translation.z(),
            after.estimate->map_from_odom.translation.z());
  EXPECT_TRUE(after.estimate->map_from_odom.rotation.isApprox(
      before.estimate->map_from_odom.rotation, 1e-12));
  ASSERT_TRUE(after.last_full_pose_time);
  EXPECT_EQ(before.last_full_pose_time->nanoseconds,
            after.last_full_pose_time->nanoseconds);
}

TEST(DiagnosticAlignmentTest, RetainsProvenanceForQueuedAndRejectedEvidence) {
  DiagnosticGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  Bootstrap(&owner);
  auto observation = Map(3);
  auto& provenance = observation.context.provenance;
  provenance.acquisition_id = "capture-with-shared-lidar";
  provenance.preparation_id = "matcher-config-revision";
  provenance.odometry_initialization = ObservationDependency::kPresent;
  provenance.odometry_deskew = ObservationDependency::kPresent;
  provenance.shared_local_scans = ObservationDependency::kPresent;
  provenance.gnss_map_construction = ObservationDependency::kUnknown;
  observation.covariance_source = "residual-scaled-hessian-not-calibrated";
  EXPECT_EQ(StatusCode::kPending,
            owner.SubmitMapObservation(observation).code());
  ASSERT_TRUE(owner.AddOdometry(Odom(3), Matrix6d::Identity() * 0.01).ok());
  DiagnosticAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(&result).ok());
  ASSERT_FALSE(result.decisions.empty());
  const auto decision = result.decisions.back();
  EXPECT_TRUE(decision.status.ok());
  EXPECT_TRUE(decision.alignment_updated);
  EXPECT_EQ(observation.covariance_source, decision.covariance_source);
  EXPECT_EQ(provenance.acquisition_id,
            decision.context.provenance.acquisition_id);
  EXPECT_EQ(provenance.preparation_id,
            decision.context.provenance.preparation_id);
  EXPECT_EQ(ObservationDependency::kPresent,
            decision.context.provenance.odometry_initialization);
  EXPECT_EQ(ObservationDependency::kPresent,
            decision.context.provenance.odometry_deskew);
  EXPECT_EQ(ObservationDependency::kPresent,
            decision.context.provenance.shared_local_scans);
  EXPECT_EQ(ObservationDependency::kUnknown,
            decision.context.provenance.gnss_map_construction);
  observation.map_from_base.rotation.coeffs().setZero();
  EXPECT_EQ(StatusCode::kInvalidArgument,
            owner.SubmitMapObservation(observation).code());
  ASSERT_TRUE(owner.GetResult(&result).ok());
  EXPECT_FALSE(result.decisions.back().alignment_updated);
  EXPECT_EQ(observation.covariance_source,
            result.decisions.back().covariance_source);
  EXPECT_EQ(provenance.preparation_id,
            result.decisions.back().context.provenance.preparation_id);

  auto queued = Map(5);
  queued.context.provenance = provenance;
  EXPECT_EQ(StatusCode::kPending, owner.SubmitMapObservation(queued).code());
  ASSERT_TRUE(owner.AddOdometry(Odom(4, 2), Matrix6d::Identity()).ok());
  ASSERT_TRUE(owner.GetResult(&result).ok());
  EXPECT_EQ(StatusCode::kEpochMismatch, result.decisions.back().status.code());
  EXPECT_EQ(queued.covariance_source,
            result.decisions.back().covariance_source);
  EXPECT_EQ(provenance.acquisition_id,
            result.decisions.back().context.provenance.acquisition_id);
}

TEST(DiagnosticAlignmentTest, InvalidDependencyDoesNotAdvanceSourceFrontier) {
  DiagnosticGlobalAlignment owner;
  ASSERT_TRUE(owner.Configure(Config()).ok());
  ASSERT_TRUE(owner.AddOdometry(Odom(0), Matrix6d::Identity()).ok());
  auto observation = Map(0);
  observation.context.provenance.shared_local_scans =
      static_cast<ObservationDependency>(999);
  EXPECT_EQ(StatusCode::kInvalidArgument,
            owner.SubmitMapObservation(observation).code());
  observation.context.provenance.shared_local_scans =
      ObservationDependency::kAbsent;
  const auto status = owner.SubmitMapObservation(observation);
  EXPECT_EQ(StatusCode::kRejectedMeasurement, status.code());
  DiagnosticAlignmentResult result;
  ASSERT_TRUE(owner.GetResult(&result).ok());
  EXPECT_TRUE(result.verification_pending);
  EXPECT_FALSE(result.estimate.has_value());
}

TEST(DiagnosticAlignmentTest, DecisionsAreAvailableBeforeOdometry) {
  DiagnosticGlobalAlignment owner;
  std::vector<GlobalObservationDecision> decisions;
  EXPECT_EQ(StatusCode::kStateUnavailable,
            owner.GetDecisions(&decisions).code());
  ASSERT_TRUE(owner.Configure(Config()).ok());
  auto invalid = Map(0);
  invalid.context.map_content_id = "wrong-map";
  EXPECT_EQ(StatusCode::kMapReferenceMismatch,
            owner.SubmitMapObservation(invalid).code());
  ASSERT_TRUE(owner.GetDecisions(&decisions).ok());
  ASSERT_EQ(1u, decisions.size());
  EXPECT_EQ(StatusCode::kMapReferenceMismatch, decisions.front().status.code());
  EXPECT_EQ(0u, decisions.front().correction_id);
  EXPECT_EQ(StatusCode::kInvalidArgument, owner.GetDecisions(nullptr).code());
}

}  // namespace
}  // namespace localization
}  // namespace openloc
