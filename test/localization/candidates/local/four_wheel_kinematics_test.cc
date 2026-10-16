// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.
#include "candidates/local/four_wheel_kinematics.h"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"

namespace openloc {
namespace localization {
namespace unified {
namespace {

FourWheelGeometry Geometry() {
  FourWheelGeometry geometry;
  geometry.wheelbase = 9.0;
  geometry.max_front_angle = 0.5061;
  geometry.max_rear_angle = 0.5061;
  return geometry;
}

FourWheelInput Input() {
  FourWheelInput input;
  input.center_forward_speed = 1.6;
  input.covariance.diagonal() << 0.01, 0.0001, 0.0001;
  return input;
}

TEST(FourWheelKinematicsTest, CounterSteeringCenterRearReferenceAndReverse) {
  auto geometry = Geometry();
  auto input = Input();
  input.front_angle = 0.2;
  input.rear_angle = -0.2;
  PlanarMotionObservation center;
  ASSERT_TRUE(FourWheelKinematics(geometry, input, &center).ok());
  EXPECT_NEAR(center.twist.y(), 0.0, 1e-12);
  EXPECT_NEAR(center.twist.z(), 2.0 * 1.6 * std::tan(0.2) / 9.0, 1e-12);
  geometry.reference_from_center.x() = -4.5;
  PlanarMotionObservation rear;
  ASSERT_TRUE(FourWheelKinematics(geometry, input, &rear).ok());
  EXPECT_NEAR(rear.twist.y(), -1.6 * std::tan(0.2), 1e-12);
  input.center_forward_speed = -1.6;
  PlanarMotionObservation reverse;
  ASSERT_TRUE(FourWheelKinematics(geometry, input, &reverse).ok());
  EXPECT_TRUE(reverse.twist.isApprox(-rear.twist, 1e-12));
}

TEST(FourWheelKinematicsTest, CrabAndStationaryDoNotInventYaw) {
  auto input = Input();
  input.front_angle = input.rear_angle = 0.2;
  PlanarMotionObservation result;
  ASSERT_TRUE(FourWheelKinematics(Geometry(), input, &result).ok());
  EXPECT_EQ(result.twist.z(), 0.0);
  EXPECT_NEAR(result.twist.y(), 1.6 * std::tan(0.2), 1e-12);
  input.center_forward_speed = 0.0;
  ASSERT_TRUE(FourWheelKinematics(Geometry(), input, &result).ok());
  EXPECT_EQ(result.twist.norm(), 0.0);
  EXPECT_TRUE(result.covariance.allFinite());
}

TEST(FourWheelKinematicsTest, CovarianceMatchesNumericalJacobianWithOffset) {
  auto geometry = Geometry();
  geometry.reference_from_center << -4.5, 0.3;
  auto input = Input();
  input.front_angle = 0.2;
  input.rear_angle = -0.1;
  input.covariance(0, 1) = input.covariance(1, 0) = 0.0001;
  PlanarMotionObservation result;
  ASSERT_TRUE(FourWheelKinematics(geometry, input, &result).ok());
  Eigen::Matrix3d numerical;
  constexpr double epsilon = 1e-6;
  for (int index = 0; index < 3; ++index) {
    auto plus = input;
    auto minus = input;
    if (index == 0) {
      plus.center_forward_speed += epsilon;
      minus.center_forward_speed -= epsilon;
    } else if (index == 1) {
      plus.front_angle += epsilon;
      minus.front_angle -= epsilon;
    } else {
      plus.rear_angle += epsilon;
      minus.rear_angle -= epsilon;
    }
    PlanarMotionObservation positive, negative;
    ASSERT_TRUE(FourWheelKinematics(geometry, plus, &positive).ok());
    ASSERT_TRUE(FourWheelKinematics(geometry, minus, &negative).ok());
    numerical.col(index) = (positive.twist - negative.twist) / (2 * epsilon);
  }
  EXPECT_TRUE(result.covariance.isApprox(
      numerical * input.covariance * numerical.transpose(), 1e-9));
  EXPECT_GT(std::abs(result.covariance(0, 2)), 1e-6);
}

TEST(FourWheelKinematicsTest, InvalidInputDoesNotOverwriteOutput) {
  auto geometry = Geometry();
  auto input = Input();
  PlanarMotionObservation output;
  output.twist.setConstant(123.0);
  input.front_angle = 0.6;
  EXPECT_EQ(FourWheelKinematics(geometry, input, &output).reason,
            Reason::INVALID_INPUT);
  EXPECT_EQ(output.twist.x(), 123.0);
  input = Input();
  input.covariance.setZero();
  EXPECT_EQ(FourWheelKinematics(geometry, input, &output).reason,
            Reason::INVALID_INPUT);
  input = Input();
  input.center_forward_speed = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(FourWheelKinematics(geometry, input, &output).reason,
            Reason::INVALID_INPUT);
  geometry.wheelbase = 0.0;
  EXPECT_EQ(FourWheelKinematics(geometry, Input(), &output).reason,
            Reason::CONFIG_INVALID);
}

TEST(FourWheelKinematicsTest, MeanRollingSpeedPercentAndCovariance) {
  auto geometry = Geometry();
  geometry.reference_from_center.x() = -4.5;
  constexpr double track = 2.0;
  MeanWheelSteeringInput input;
  input.mean_rolling_speed = 1.6;
  input.steering_percent = 40.0;
  input.covariance.diagonal() << 0.01, 0.25;
  PlanarMotionObservation result;
  ASSERT_TRUE(
      CounterSteeringMeanWheelKinematics(geometry, track, input, &result).ok());
  const double angle = 0.4 * geometry.max_front_angle;
  EXPECT_NEAR(result.twist.y(), -result.twist.x() * std::tan(angle), 1e-12);
  EXPECT_LT(result.twist.x(), input.mean_rolling_speed);
  double reconstructed = 0.0;
  for (double x : {-4.5, 4.5}) {
    for (double y : {-1.0, 1.0}) {
      reconstructed +=
          0.25 * std::hypot(result.twist.x() - result.twist.z() * y,
                            result.twist.z() * x);
    }
  }
  EXPECT_NEAR(reconstructed, input.mean_rolling_speed, 1e-12);
  Eigen::Matrix<double, 3, 2> numerical;
  constexpr double epsilon = 1e-5;
  for (int index = 0; index < 2; ++index) {
    auto plus = input;
    auto minus = input;
    if (index == 0) {
      plus.mean_rolling_speed += epsilon;
      minus.mean_rolling_speed -= epsilon;
    } else {
      plus.steering_percent += epsilon;
      minus.steering_percent -= epsilon;
    }
    PlanarMotionObservation positive, negative;
    ASSERT_TRUE(
        CounterSteeringMeanWheelKinematics(geometry, track, plus, &positive)
            .ok());
    ASSERT_TRUE(
        CounterSteeringMeanWheelKinematics(geometry, track, minus, &negative)
            .ok());
    numerical.col(index) = (positive.twist - negative.twist) / (2 * epsilon);
  }
  EXPECT_TRUE(result.covariance.isApprox(
      numerical * input.covariance * numerical.transpose(), 1e-9));
  input.mean_rolling_speed = -1.6;
  PlanarMotionObservation reverse;
  ASSERT_TRUE(
      CounterSteeringMeanWheelKinematics(geometry, track, input, &reverse)
          .ok());
  EXPECT_TRUE(reverse.twist.isApprox(-result.twist, 1e-12));
  input.steering_percent = 0.0;
  ASSERT_TRUE(
      CounterSteeringMeanWheelKinematics(geometry, track, input, &reverse)
          .ok());
  EXPECT_EQ(reverse.twist.x(), -1.6);
  EXPECT_EQ(reverse.twist.y(), 0.0);
  EXPECT_EQ(reverse.twist.z(), 0.0);
  input.steering_percent = 101.0;
  EXPECT_EQ(CounterSteeringMeanWheelKinematics(geometry, track, input, &reverse)
                .reason,
            Reason::INVALID_INPUT);
  EXPECT_EQ(
      CounterSteeringMeanWheelKinematics(geometry, 0.0, input, &reverse).reason,
      Reason::CONFIG_INVALID);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace openloc
