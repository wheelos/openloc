// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.
#include "candidates/local/planar_wheel_dr.h"

#include "gtest/gtest.h"

namespace openloc {
namespace localization {
namespace unified {
namespace {

PlanarWheelInput Input(double time) {
  PlanarWheelInput input;
  input.stamp.time = time;
  input.stamp.clock_id = "test";
  input.velocity.x() = 2.0;
  return input;
}

TEST(PlanarWheelDrTest, CorrectsKnownBiasAndSupportsSignedMotion) {
  PlanarWheelDr dr("test", 0.05, 0.01);
  auto input = Input(1.0);
  input.gyro_z = 0.01;
  ASSERT_TRUE(dr.Seed(input, Eigen::Vector2d::Zero(), 0.0).ok());
  for (int i = 1; i <= 100; ++i) {
    input.stamp.time = 1.0 + i * 0.01;
    ASSERT_TRUE(dr.Add(input).ok());
  }
  EXPECT_NEAR(dr.state().position.x(), 2.0, 1e-12);
  EXPECT_NEAR(dr.state().yaw, 0.0, 1e-12);
  input.velocity.x() = -2.0;
  ASSERT_TRUE(dr.Seed(input, Eigen::Vector2d::Zero(), 0.0).ok());
  input.stamp.time += 0.01;
  ASSERT_TRUE(dr.Add(input).ok());
  EXPECT_NEAR(dr.state().position.x(), -0.02, 1e-12);
  EXPECT_NEAR(dr.state().distance, 0.02, 1e-12);
}

TEST(PlanarWheelDrTest, IntegratesCounterSteeringRearAxleLateralVelocity) {
  PlanarWheelDr dr("test", 0.05, 0.0);
  auto input = Input(1.0);
  input.velocity.y() = -4.5 * 0.2;
  input.gyro_z = 0.2;
  ASSERT_TRUE(dr.Seed(input, Eigen::Vector2d::Zero(), 0.0).ok());
  for (int i = 1; i <= 100; ++i) {
    input.stamp.time = 1.0 + i * 0.01;
    ASSERT_TRUE(dr.Add(input).ok());
  }
  EXPECT_NEAR(dr.state().yaw, 0.2, 1e-12);
  const double a = std::sin(0.2) / 0.2;
  const double b = (1.0 - std::cos(0.2)) / 0.2;
  EXPECT_NEAR(dr.state().position.x(), 2.0 * a + 0.9 * b, 1e-12);
  EXPECT_NEAR(dr.state().position.y(), 2.0 * b - 0.9 * a, 1e-12);
}

TEST(PlanarWheelDrTest, RejectsClockAndTimeAndLatchesGapWithoutTeleport) {
  PlanarWheelDr dr("test", 0.05, 0.0);
  auto input = Input(1.0);
  EXPECT_FALSE(dr.Add(input).ok());
  ASSERT_TRUE(dr.Seed(input, Eigen::Vector2d(3.0, 4.0), 0.1).ok());
  EXPECT_EQ(dr.Add(input).reason, Reason::DUPLICATE);
  input.stamp.clock_id = "other";
  EXPECT_EQ(dr.Add(input).reason, Reason::CLOCK_INVALID);
  input.stamp.clock_id = "test";
  input.stamp.time = 0.99;
  EXPECT_EQ(dr.Add(input).reason, Reason::TIMESTAMP_REGRESSION);
  input.stamp.time = 1.2;
  EXPECT_EQ(dr.Add(input).reason, Reason::IMU_GAP);
  EXPECT_FALSE(dr.state().valid);
  EXPECT_EQ(dr.state().position.x(), 3.0);
  input.stamp.time = 1.01;
  EXPECT_EQ(dr.Add(input).reason, Reason::IMU_GAP);
  ASSERT_TRUE(dr.Seed(input, Eigen::Vector2d::Zero(), 0.0).ok());
  EXPECT_TRUE(dr.state().valid);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace openloc
