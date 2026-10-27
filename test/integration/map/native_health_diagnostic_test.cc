// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "test/integration/map/native_health_diagnostic.h"

#include <limits>
#include <sstream>

#include "gtest/gtest.h"

#include "openloc/localization/online/types.h"

namespace openloc::replay {
namespace {

namespace loc = localization;
namespace u = localization::unified;

TEST(NativeHealthDiagnosticTest, HealthAndGlobalPoseContractsCoexist) {
  loc::Pose global_pose;
  global_pose.translation = Eigen::Vector3d(1.0, 2.0, 3.0);
  loc::LocalizationEstimate estimate;
  loc::HealthPose* health_pose = estimate.mutable_pose();
  health_pose->mutable_position()->set_x(global_pose.translation.x());
  health_pose->mutable_position()->set_y(global_pose.translation.y());
  health_pose->mutable_position()->set_z(global_pose.translation.z());
  health_pose->mutable_orientation()->set_qw(global_pose.rotation.w());
  std::string wire;
  ASSERT_TRUE(estimate.SerializeToString(&wire));
  loc::LocalizationEstimate decoded;
  ASSERT_TRUE(decoded.ParseFromString(wire));
  EXPECT_DOUBLE_EQ(decoded.pose().position().x(), 1.0);
  EXPECT_DOUBLE_EQ(decoded.pose().position().y(), 2.0);
  EXPECT_DOUBLE_EQ(decoded.pose().position().z(), 3.0);
  EXPECT_DOUBLE_EQ(decoded.pose().orientation().qw(), 1.0);
}

u::LocalState State(double time = 10.0) {
  u::LocalState state;
  state.valid = true;
  state.epoch = {"raw-native", 1};
  state.stamp.time = time;
  state.stamp.receive_time = time;
  state.stamp.sequence = 1;
  state.covariance = u::Matrix15d::Identity() * 1e-4;
  state.covariance_model_valid = false;
  return state;
}

TEST(NativeHealthDiagnosticTest, RelativeOutputDoesNotQualifyHealthOrGlobal) {
  std::ostringstream journal;
  NativeHealthDiagnostic health(journal);
  auto state = State();
  for (int i = 0; i < 30; ++i) {
    state.stamp.time = 10.0 + i * 0.01;
    state.stamp.sequence = i + 1;
    // Even the pre-LIO model flag is not deployment uncertainty evidence.
    state.covariance_model_valid = i % 2 == 0;
    const auto stamp = static_cast<int64_t>(10000000000LL + i * 10000000LL);
    auto result = health.Update(state, {}, stamp, stamp, true);
    ASSERT_TRUE(result.assessment_accepted);
    EXPECT_TRUE(result.local_relative_valid);
    EXPECT_TRUE(result.assessment.pose_valid());
    EXPECT_TRUE(result.diagnostic_global_initialized);
    EXPECT_FALSE(result.assessment.covariance_valid());
    EXPECT_FALSE(result.assessment.has_position_std_x());
    EXPECT_FALSE(result.assessment.has_yaw_std());
    EXPECT_FALSE(result.assessment.global_pose_valid());
    EXPECT_FALSE(result.assessment.map_alignment_valid());
    EXPECT_FALSE(result.health_trusted);
    EXPECT_EQ(result.status.capabilities(), 0);
    EXPECT_NE(result.status.availability_state(), loc::AVAILABILITY_NOMINAL);
    EXPECT_TRUE(loc::LocalizationHealth::HasReason(
        result.status.active_reasons(), loc::REASON_INVALID_COVARIANCE));
    EXPECT_FALSE(result.status.has_horizontal_uncertainty());
    EXPECT_FALSE(result.status.has_heading_uncertainty());
  }
  EXPECT_NE(journal.str().find("raw_native_uncertainty_unqualified"),
            std::string::npos);
  EXPECT_NE(journal.str().find("REASON_INVALID_COVARIANCE"), std::string::npos);
}

TEST(NativeHealthDiagnosticTest, StartupAndIntegrityFailureStayDistinct) {
  std::ostringstream journal;
  NativeHealthDiagnostic health(journal);
  u::LocalState startup;
  auto result =
      health.Update(startup, {u::Reason::WAITING_FOR_IMU, "no initialization"},
                    9000000000LL, 9000000000LL);
  EXPECT_TRUE(result.assessment_accepted);
  EXPECT_FALSE(result.local_relative_valid);
  EXPECT_FALSE(result.assessment.pose_valid());
  auto state = State();
  result = health.Update(state, {u::Reason::IMU_GAP, "IMU gap"}, 10000000000LL,
                         10000000000LL);
  EXPECT_TRUE(result.assessment_accepted);
  EXPECT_FALSE(result.local_relative_valid);
  EXPECT_FALSE(result.assessment.local_consistency_valid());
  EXPECT_FALSE(result.assessment.source_health(0).is_healthy());
  EXPECT_EQ(result.assessment.source_health(0).message(), "IMU gap");
  EXPECT_TRUE(loc::LocalizationHealth::HasReason(
      result.status.active_reasons(), loc::REASON_LOCAL_POSE_INVALID));
  EXPECT_NE(journal.str().find("IMU_GAP"), std::string::npos);
}

TEST(NativeHealthDiagnosticTest,
     SupervisorActuallyChecksNativeNumericsAndTime) {
  std::ostringstream journal;
  NativeHealthDiagnostic health(journal);
  auto state = State();
  state.position.x() = std::numeric_limits<double>::quiet_NaN();
  state.orientation.coeffs() *= 2.0;
  auto result = health.Update(state, {}, 10000000000LL, 11000000000LL);
  // Producer validity remains evidence, not an independent numerical verdict.
  EXPECT_TRUE(result.local_relative_valid);
  EXPECT_TRUE(loc::LocalizationHealth::HasReason(
      result.status.active_reasons(), loc::REASON_NON_FINITE_OUTPUT));
  EXPECT_TRUE(loc::LocalizationHealth::HasReason(
      result.status.active_reasons(), loc::REASON_INVALID_QUATERNION));
  EXPECT_TRUE(loc::LocalizationHealth::HasReason(result.status.active_reasons(),
                                                 loc::REASON_POSE_TIMEOUT));
  EXPECT_TRUE(loc::LocalizationHealth::HasReason(
      result.status.active_reasons(), loc::REASON_ASSESSMENT_TIMEOUT));
  EXPECT_EQ(result.status.availability_state(), loc::AVAILABILITY_INVALID);
}

TEST(NativeHealthDiagnosticTest,
     SupervisorOwnsIdentitySequenceAndEpochRollover) {
  std::ostringstream journal;
  NativeHealthDiagnostic health(journal);
  auto first = health.Update(State(), {}, 10000000000LL, 10000000000LL);
  auto state = State(10.01);
  state.epoch.generation = 2;
  state.stamp.sequence = 1;
  auto second = health.Update(state, {}, 10010000000LL, 10010000000LL);
  ASSERT_TRUE(second.assessment_accepted);
  EXPECT_EQ(first.status.session_id(), "raw-native:1");
  EXPECT_EQ(second.status.session_id(), "raw-native:2");
  EXPECT_FALSE(first.status.supervisor_session_id().empty());
  EXPECT_EQ(first.status.supervisor_session_id(),
            second.status.supervisor_session_id());
  EXPECT_NE(first.status.session_id(), first.status.supervisor_session_id());
  EXPECT_GT(second.status.sequence(), first.status.sequence());
  EXPECT_GT(second.status.transition_id(), first.status.transition_id());
  EXPECT_NE(journal.str().find(second.status.supervisor_session_id()),
            std::string::npos);
  state = State(10.02);
  auto retired = health.Update(state, {}, 10020000000LL, 10020000000LL);
  EXPECT_FALSE(retired.assessment_accepted);
  EXPECT_EQ(retired.status.session_id(), second.status.session_id());
  EXPECT_TRUE(loc::LocalizationHealth::HasReason(
      retired.status.active_reasons(), loc::REASON_SESSION_CHANGED));
}

}  // namespace
}  // namespace openloc::replay
