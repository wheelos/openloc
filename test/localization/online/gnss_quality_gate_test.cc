// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "gnss_quality_gate.h"
#include "openloc/localization/online/gnss_position_gate.h"

#include <limits>

#include "gtest/gtest.h"

namespace openloc {
namespace localization {
namespace internal {
namespace {

Timestamp Time(int64_t ns) { return {ns, ClockDomain::kUnixEpoch}; }

GnssReceiverQuality FixedReceiver() {
  GnssReceiverQuality receiver;
  receiver.carrier_solution = GnssCarrierSolution::kFixed;
  receiver.satellite_count = 20;
  receiver.hdop = 0.8;
  receiver.differential_age_seconds = 0.2;
  receiver.horizontal_accuracy_m = 0.03;
  return receiver;
}

GnssQualityGatePolicy GatePolicy() {
  GnssQualityGatePolicy policy{};
  policy.minimum_satellites = 12;
  policy.maximum_hdop = 1.5;
  policy.maximum_differential_age_seconds = 3.0;
  policy.maximum_horizontal_accuracy_m = 0.3;
  policy.maximum_heading_accuracy_rad = 0.035;
  policy.maximum_baseline_length_error_m = 0.02;
  policy.fixed_solution_stability_ns = 100;
  policy.maximum_fixed_solution_gap_ns = 250;
  return policy;
}

TEST(GnssQualityGateTest, MissingHdopIsPositionOnlyExplicitNaNOptIn) {
  GnssQualityGate gate;
  auto policy = GatePolicy();
  auto receiver = FixedReceiver();
  receiver.hdop = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(gate.Configure(policy).ok());
  EXPECT_EQ(GnssQualityOutcome::kInvalid,
            gate.CheckPosition(Time(100), receiver).outcome);
  policy.allow_missing_hdop = true;
  ASSERT_TRUE(gate.Configure(policy).ok());
  EXPECT_EQ(GnssQualityOutcome::kWaitingForStability,
            gate.CheckPosition(Time(100), receiver).outcome);
  EXPECT_TRUE(gate.CheckPosition(Time(200), receiver).status.ok());
  EXPECT_EQ(GnssQualityOutcome::kInvalid,
            gate.CheckHeading(Time(200), receiver, 0.01, 1.0, 1.0).outcome);
  for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
    receiver.hdop = invalid;
    EXPECT_EQ(GnssQualityOutcome::kInvalid,
              gate.CheckPosition(Time(300), receiver).outcome);
  }
  receiver.hdop = std::numeric_limits<double>::quiet_NaN();
  receiver.horizontal_accuracy_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(GnssQualityOutcome::kInvalid,
            gate.CheckPosition(Time(300), receiver).outcome);
  receiver = FixedReceiver();
  receiver.hdop = policy.maximum_hdop;
  EXPECT_EQ(GnssQualityOutcome::kRejected,
            gate.CheckPosition(Time(300), receiver).outcome);
}

TEST(GnssQualityGateTest, PublicPositionGatePreservesMissingHdopPolicy) {
  GnssPositionGate gate;
  GnssPositionGatePolicy policy;
  policy.fixed_solution_stability_ns = 100;
  auto receiver = FixedReceiver();
  receiver.hdop = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(gate.Configure(policy).ok());
  EXPECT_EQ(StatusCode::kInvalidArgument, gate.Check(Time(100), receiver).code());
  policy.allow_missing_hdop = true;
  ASSERT_TRUE(gate.Configure(policy).ok());
  EXPECT_FALSE(gate.Check(Time(100), receiver).ok());
  EXPECT_TRUE(gate.Check(Time(200), receiver).ok());
}

TEST(GnssQualityGateTest, SharesReceiverChecksAcrossPositionAndHeading) {
  GnssQualityGate gate;
  ASSERT_TRUE(gate.Configure(GatePolicy()).ok());
  auto receiver = FixedReceiver();
  receiver.satellite_count = 5;

  const auto position = gate.CheckPosition(Time(100), receiver);
  EXPECT_EQ(GnssQualityOutcome::kRejected, position.outcome);
  EXPECT_EQ("insufficient_satellites", position.status.message());
  const auto heading = gate.CheckHeading(Time(100), receiver, 0.01, 1.0, 1.0);
  EXPECT_EQ(GnssQualityOutcome::kRejected, heading.outcome);
  EXPECT_EQ("insufficient_satellites", heading.status.message());
}

TEST(GnssQualityGateTest, TracksPositionAndHeadingStabilityIndependently) {
  GnssQualityGate gate;
  ASSERT_TRUE(gate.Configure(GatePolicy()).ok());
  const auto receiver = FixedReceiver();

  EXPECT_EQ(GnssQualityOutcome::kWaitingForStability,
            gate.CheckPosition(Time(100), receiver).outcome);
  EXPECT_EQ(GnssQualityOutcome::kWaitingForStability,
            gate.CheckHeading(Time(100), receiver, 0.01, 1.0, 1.0).outcome);
  EXPECT_EQ(GnssQualityOutcome::kAccepted,
            gate.CheckPosition(Time(200), receiver).outcome);
  EXPECT_EQ(GnssQualityOutcome::kAccepted,
            gate.CheckHeading(Time(200), receiver, 0.01, 1.0, 1.0).outcome);
}

TEST(GnssQualityGateTest, InvalidQualityRestartsOnlyItsMeasurementDwell) {
  GnssQualityGate gate;
  ASSERT_TRUE(gate.Configure(GatePolicy()).ok());
  const auto receiver = FixedReceiver();
  gate.CheckPosition(Time(100), receiver);
  gate.CheckHeading(Time(100), receiver, 0.01, 1.0, 1.0);

  auto poor_position = receiver;
  poor_position.horizontal_accuracy_m = 0.5;
  const auto rejected = gate.CheckPosition(Time(150), poor_position);
  EXPECT_EQ(GnssQualityOutcome::kRejected, rejected.outcome);
  EXPECT_EQ("horizontal_accuracy_exceeds_limit", rejected.status.message());
  EXPECT_EQ(GnssQualityOutcome::kWaitingForStability,
            gate.CheckPosition(Time(200), receiver).outcome);
  EXPECT_TRUE(
      gate.CheckHeading(Time(200), receiver, 0.01, 1.0, 1.0).status.ok());
}

TEST(GnssQualityGateTest, ReceiverGapRestartsTheStabilityWindow) {
  GnssQualityGate gate;
  auto policy = GatePolicy();
  policy.maximum_fixed_solution_gap_ns = 50;
  ASSERT_TRUE(gate.Configure(policy).ok());
  const auto receiver = FixedReceiver();
  gate.CheckPosition(Time(100), receiver);
  EXPECT_EQ(GnssQualityOutcome::kWaitingForStability,
            gate.CheckPosition(Time(200), receiver).outcome);
  EXPECT_EQ(GnssQualityOutcome::kWaitingForStability,
            gate.CheckPosition(Time(250), receiver).outcome);
  EXPECT_TRUE(gate.CheckPosition(Time(300), receiver).status.ok());
}

TEST(GnssQualityGateTest, AppliesHeadingSpecificAccuracyAndBaselineLimits) {
  GnssQualityGate gate;
  auto policy = GatePolicy();
  policy.fixed_solution_stability_ns = 0;
  ASSERT_TRUE(gate.Configure(policy).ok());
  auto receiver = FixedReceiver();
  // Position accuracy is structurally valid but is not a heading limit.
  receiver.horizontal_accuracy_m = 1.0;
  const auto inaccurate = gate.CheckHeading(Time(100), receiver, 0.1, 1.0, 1.0);
  EXPECT_EQ("heading_accuracy_exceeds_limit", inaccurate.status.message());
  const auto baseline = gate.CheckHeading(Time(200), receiver, 0.01, 1.1, 1.0);
  EXPECT_EQ("baseline_length_mismatch", baseline.status.message());
  EXPECT_TRUE(
      gate.CheckHeading(Time(300), receiver, 0.01, 1.0, 1.0).status.ok());
}

TEST(GnssQualityGateTest, InvalidConfigurationPreservesPolicyAndDwell) {
  GnssQualityGate gate;
  EXPECT_EQ(GnssQualityOutcome::kInvalid,
            gate.CheckPosition(Time(100), FixedReceiver()).outcome);
  ASSERT_TRUE(gate.Configure(GatePolicy()).ok());
  gate.CheckPosition(Time(100), FixedReceiver());
  auto policy = GatePolicy();
  policy.maximum_hdop = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(StatusCode::kInvalidArgument, gate.Configure(policy).code());
  policy = GatePolicy();
  policy.fixed_solution_stability_ns = -1;
  EXPECT_FALSE(gate.Configure(policy).ok());
  policy = GatePolicy();
  policy.maximum_fixed_solution_gap_ns = 0;
  EXPECT_FALSE(gate.Configure(policy).ok());
  EXPECT_TRUE(gate.CheckPosition(Time(200), FixedReceiver()).status.ok());
  gate.Reset();
  EXPECT_EQ(GnssQualityOutcome::kWaitingForStability,
            gate.CheckPosition(Time(300), FixedReceiver()).outcome);
}

TEST(GnssQualityGateTest, InvalidInputsCannotAdvanceStability) {
  GnssQualityGate gate;
  ASSERT_TRUE(gate.Configure(GatePolicy()).ok());
  const auto receiver = FixedReceiver();
  gate.CheckPosition(Time(100), receiver);
  auto invalid = receiver;
  invalid.hdop = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(GnssQualityOutcome::kInvalid,
            gate.CheckPosition(Time(200), invalid).outcome);
  EXPECT_EQ(StatusCode::kDuplicateTimestamp,
            gate.CheckPosition(Time(100), receiver).status.code());
  EXPECT_EQ(StatusCode::kOutOfOrderTimestamp,
            gate.CheckPosition(Time(99), receiver).status.code());
  EXPECT_EQ(StatusCode::kClockDomainMismatch,
            gate.CheckPosition({200, ClockDomain::kSensorMonotonic}, receiver)
                .status.code());
  EXPECT_TRUE(gate.CheckPosition(Time(200), receiver).status.ok());
  EXPECT_EQ(
      GnssQualityOutcome::kInvalid,
      gate.CheckHeading(Time(100), receiver,
                        std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0)
          .outcome);
  EXPECT_EQ(GnssQualityOutcome::kWaitingForStability,
            gate.CheckHeading(Time(100), receiver, 0.01, 1.0, 1.0).outcome);
}

TEST(GnssQualityGateTest, MissingAccuracyRequiresPositionDiagnosticOptIn) {
  GnssQualityGate gate;
  auto policy = GatePolicy();
  auto receiver = FixedReceiver();
  receiver.horizontal_accuracy_m = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(gate.Configure(policy).ok());
  EXPECT_EQ(GnssQualityOutcome::kInvalid,
            gate.CheckPosition(Time(100), receiver).outcome);
  policy.allow_missing_horizontal_accuracy_diagnostic = true;
  ASSERT_TRUE(gate.Configure(policy).ok());
  EXPECT_EQ(GnssQualityOutcome::kWaitingForStability,
            gate.CheckPosition(Time(100), receiver).outcome);
  EXPECT_TRUE(gate.CheckPosition(Time(200), receiver).status.ok());
  EXPECT_EQ(GnssQualityOutcome::kInvalid,
            gate.CheckHeading(Time(200), receiver, 0.01, 1.0, 1.0).outcome);
  receiver.carrier_solution = GnssCarrierSolution::kFloat;
  EXPECT_EQ("carrier_solution_not_fixed",
            gate.CheckPosition(Time(300), receiver).status.message());
  receiver = FixedReceiver();
  receiver.horizontal_accuracy_m = policy.maximum_horizontal_accuracy_m + 1;
  EXPECT_EQ("horizontal_accuracy_exceeds_limit",
            gate.CheckPosition(Time(400), receiver).status.message());
  for (double value : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
    receiver.horizontal_accuracy_m = value;
    EXPECT_EQ(GnssQualityOutcome::kInvalid,
              gate.CheckPosition(Time(500), receiver).outcome);
  }
}

TEST(GnssQualityGateTest, PreservesInclusiveAndExclusiveQualityBoundaries) {
  GnssQualityGate gate;
  auto policy = GatePolicy();
  policy.fixed_solution_stability_ns = 0;
  ASSERT_TRUE(gate.Configure(policy).ok());
  auto receiver = FixedReceiver();
  receiver.satellite_count = policy.minimum_satellites;
  receiver.horizontal_accuracy_m = policy.maximum_horizontal_accuracy_m;
  EXPECT_TRUE(gate.CheckPosition(Time(100), receiver).status.ok());
  receiver.hdop = policy.maximum_hdop;
  EXPECT_EQ("hdop_exceeds_limit",
            gate.CheckPosition(Time(200), receiver).status.message());
  receiver = FixedReceiver();
  receiver.differential_age_seconds = policy.maximum_differential_age_seconds;
  EXPECT_EQ("differential_age_exceeds_limit",
            gate.CheckPosition(Time(300), receiver).status.message());
  receiver = FixedReceiver();
  EXPECT_TRUE(gate.CheckHeading(Time(400), receiver,
                                policy.maximum_heading_accuracy_rad, 1.0, 1.0)
                  .status.ok());
}

}  // namespace
}  // namespace internal
}  // namespace localization
}  // namespace openloc
