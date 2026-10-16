// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "alignment_recovery.h"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"

namespace openloc {
namespace localization {
namespace internal {
namespace {

GnssAlignmentRecoveryObservation Observation(int index,
                                             const Eigen::Vector2d& odom,
                                             const Eigen::Vector2d& translation,
                                             double yaw) {
  Eigen::Matrix2d rotation;
  rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  GnssAlignmentRecoveryObservation observation;
  observation.timestamp_ns = 1'000'000'000LL * (index + 1);
  observation.odom_antenna_xy = odom;
  observation.global_antenna_xy = rotation * odom + translation;
  observation.covariance_xy = Eigen::Matrix2d::Identity() * 0.01;
  return observation;
}

void AddAndExpectNoCandidate(
    GnssAlignmentRecovery* recovery,
    const GnssAlignmentRecoveryObservation& observation) {
  GnssAlignmentRecoveryDiagnostics diagnostics;
  GnssAlignmentRecoveryCandidate candidate;
  ASSERT_TRUE(
      recovery->AddRejectedObservation(observation, &diagnostics, &candidate)
          .ok());
  EXPECT_FALSE(diagnostics.candidate_validated);
}

TEST(GnssAlignmentRecoveryTest, RequiresLaterHoldoutBeforeCandidate) {
  GnssAlignmentRecoveryConfig config;
  config.training_observations = 4;
  config.validation_observations = 2;
  config.minimum_odom_baseline_m = 2.0;
  GnssAlignmentRecovery recovery(config);
  const Eigen::Vector2d translation(8.0, -3.0);
  constexpr double kYaw = 0.35;
  GnssAlignmentRecoveryCandidate candidate;
  GnssAlignmentRecoveryDiagnostics diagnostics;
  for (int index = 0; index < 4; ++index) {
    const Eigen::Vector2d odom(index * 2.0, 0.2 * index * index);
    auto observation = Observation(index, odom, translation, kYaw);
    ASSERT_TRUE(
        recovery.AddRejectedObservation(observation, &diagnostics, &candidate)
            .ok());
    EXPECT_FALSE(diagnostics.candidate_validated);
  }
  EXPECT_EQ(diagnostics.reason, "recovery_candidate_waiting_for_holdout");

  auto first_holdout =
      Observation(4, Eigen::Vector2d(8.0, 2.1), translation, kYaw);
  ASSERT_TRUE(
      recovery.AddRejectedObservation(first_holdout, &diagnostics, &candidate)
          .ok());
  EXPECT_FALSE(diagnostics.candidate_validated);
  auto second_holdout =
      Observation(5, Eigen::Vector2d(10.0, 5.0), translation, kYaw);
  ASSERT_TRUE(
      recovery.AddRejectedObservation(second_holdout, &diagnostics, &candidate)
          .ok());
  ASSERT_TRUE(diagnostics.candidate_validated);
  EXPECT_NEAR(candidate.global_from_odom_xy_yaw.x(), translation.x(), 1e-8);
  EXPECT_NEAR(candidate.global_from_odom_xy_yaw.y(), translation.y(), 1e-8);
  EXPECT_NEAR(candidate.global_from_odom_xy_yaw.z(), kYaw, 1e-8);
  EXPECT_EQ(candidate.training_count, 4U);
  EXPECT_EQ(candidate.validation_count, 2U);
}

TEST(GnssAlignmentRecoveryTest,
     CannotReanchorFromSingleOutlierOrStationaryYaw) {
  GnssAlignmentRecoveryConfig config;
  config.training_observations = 4;
  config.validation_observations = 2;
  config.minimum_odom_baseline_m = 2.0;
  GnssAlignmentRecovery recovery(config);
  GnssAlignmentRecoveryCandidate candidate;
  GnssAlignmentRecoveryDiagnostics diagnostics;
  auto outlier = Observation(0, Eigen::Vector2d::Zero(),
                             Eigen::Vector2d(1000.0, -1000.0), 0.0);
  ASSERT_TRUE(
      recovery.AddRejectedObservation(outlier, &diagnostics, &candidate).ok());
  EXPECT_FALSE(diagnostics.candidate_validated);
  EXPECT_EQ(diagnostics.reason, "recovery_training_window");

  recovery.Reset();
  for (int index = 0; index < 4; ++index) {
    auto stationary = Observation(index, Eigen::Vector2d(3.0, 4.0),
                                  Eigen::Vector2d(3.0, 4.0), 0.0);
    ASSERT_TRUE(
        recovery.AddRejectedObservation(stationary, &diagnostics, &candidate)
            .ok());
  }
  EXPECT_EQ(diagnostics.reason, "recovery_yaw_unobservable");
  EXPECT_FALSE(diagnostics.candidate_validated);
}

TEST(GnssAlignmentRecoveryTest, FailedHoldoutRestartsTrainingAndRequiresOrder) {
  GnssAlignmentRecoveryConfig config;
  config.training_observations = 3;
  config.validation_observations = 2;
  config.minimum_odom_baseline_m = 2.0;
  GnssAlignmentRecovery recovery(config);
  GnssAlignmentRecoveryCandidate candidate;
  GnssAlignmentRecoveryDiagnostics diagnostics;
  const Eigen::Vector2d translation(4.0, 2.0);
  constexpr double kYaw = -0.2;
  for (int index = 0; index < 3; ++index) {
    auto observation = Observation(index, Eigen::Vector2d(index * 3.0, 0.0),
                                   translation, kYaw);
    ASSERT_TRUE(
        recovery.AddRejectedObservation(observation, &diagnostics, &candidate)
            .ok());
  }
  auto bad_holdout = Observation(3, Eigen::Vector2d(9.0, 0.0),
                                 Eigen::Vector2d(-20.0, 70.0), 0.0);
  ASSERT_TRUE(
      recovery.AddRejectedObservation(bad_holdout, &diagnostics, &candidate)
          .ok());
  EXPECT_EQ(diagnostics.reason, "recovery_holdout_inconsistent");
  EXPECT_FALSE(diagnostics.candidate_validated);
  auto duplicate = bad_holdout;
  EXPECT_EQ(recovery.AddRejectedObservation(duplicate, &diagnostics, &candidate)
                .code(),
            StatusCode::kInvalidArgument);
}

TEST(GnssAlignmentRecoveryTest, BoundsTotalTrainingAndHoldoutAge) {
  GnssAlignmentRecoveryConfig config;
  config.training_observations = 3;
  config.validation_observations = 1;
  config.minimum_odom_baseline_m = 2.0;
  config.maximum_window_ns = 5'000'000'000LL;
  for (int holdout_index : {5, 6}) {
    GnssAlignmentRecovery recovery(config);
    GnssAlignmentRecoveryDiagnostics diagnostics;
    GnssAlignmentRecoveryCandidate candidate;
    for (int index : {0, 2, 4}) {
      ASSERT_TRUE(recovery
                      .AddRejectedObservation(
                          Observation(index, Eigen::Vector2d(index, 0.0),
                                      Eigen::Vector2d(8.0, 0.0), 0.0),
                          &diagnostics, &candidate)
                      .ok());
    }
    ASSERT_EQ(diagnostics.reason, "recovery_candidate_waiting_for_holdout");
    const auto holdout =
        Observation(holdout_index, Eigen::Vector2d(holdout_index, 0.0),
                    Eigen::Vector2d(8.0, 0.0), 0.0);
    ASSERT_TRUE(
        recovery.AddRejectedObservation(holdout, &diagnostics, &candidate)
            .ok());
    if (holdout_index == 5) {
      EXPECT_TRUE(diagnostics.candidate_validated);
    } else {
      EXPECT_FALSE(diagnostics.candidate_validated);
      EXPECT_EQ(diagnostics.reason, "recovery_window_timeout");
      EXPECT_EQ(diagnostics.training_count, 1U);
      EXPECT_EQ(diagnostics.validation_count, 0U);
      EXPECT_EQ(
          recovery.AddRejectedObservation(holdout, &diagnostics, &candidate)
              .code(),
          StatusCode::kInvalidArgument);
    }
  }

  config.maximum_window_ns = 3'000'000'000LL;
  GnssAlignmentRecovery recovery(config);
  GnssAlignmentRecoveryDiagnostics diagnostics;
  GnssAlignmentRecoveryCandidate candidate;
  for (int index : {0, 2, 4}) {
    ASSERT_TRUE(recovery
                    .AddRejectedObservation(
                        Observation(index, Eigen::Vector2d(index, 0.0),
                                    Eigen::Vector2d(8.0, 0.0), 0.0),
                        &diagnostics, &candidate)
                    .ok());
  }
  EXPECT_EQ(diagnostics.reason, "recovery_window_timeout");
  EXPECT_EQ(diagnostics.training_count, 1U);
  EXPECT_FALSE(diagnostics.candidate_validated);
}

TEST(GnssAlignmentRecoveryTest, CoherentBiasIsNotIndependentIntegrityEvidence) {
  GnssAlignmentRecovery recovery;
  GnssAlignmentRecoveryDiagnostics diagnostics;
  GnssAlignmentRecoveryCandidate candidate;
  const Eigen::Vector2d receiver_bias(20.0, -10.0);
  for (int index = 0; index < 11; ++index) {
    ASSERT_TRUE(recovery
                    .AddRejectedObservation(
                        Observation(index, Eigen::Vector2d(index, 0.0),
                                    receiver_bias, 0.0),
                        &diagnostics, &candidate)
                    .ok());
  }
  ASSERT_TRUE(diagnostics.candidate_validated);
  EXPECT_NEAR(
      (candidate.global_from_odom_xy_yaw.head<2>() - receiver_bias).norm(), 0.0,
      1e-8);
  EXPECT_GT(candidate.global_from_odom_xy_yaw.head<2>().norm(), 20.0);
}

}  // namespace
}  // namespace internal
}  // namespace localization
}  // namespace openloc
