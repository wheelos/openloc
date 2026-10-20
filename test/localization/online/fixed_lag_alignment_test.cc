// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "openloc/localization/global/fixed_lag_alignment.h"

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
