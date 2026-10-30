// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/local/continuous_output.h"

#include <algorithm>

#include "gtest/gtest.h"

#include "candidates/local/local_motion.h"

namespace openloc {
namespace localization {
namespace unified {
namespace {

LocalState State(double time, double x) {
  LocalState state;
  state.stamp = {time, time, static_cast<uint64_t>(time * 10.0), "clock"};
  state.epoch = {"session", 1};
  state.position.x() = x;
  state.covariance.setIdentity();
  state.valid = true;
  return state;
}

LocalConfig LinearizationConfig() {
  LocalConfig config;
  config.lidar_max_translation_correction_rate = 10;
  config.lidar_max_rotation_correction_rate = 10;
  config.lidar_max_velocity_correction_acceleration = 100;
  config.lidar_correction_variance_scale = 2;
  config.lidar_max_translation_correction = 2;
  config.lidar_max_rotation_correction = 1;
  config.lidar_max_velocity_correction = 3;
  config.lidar_process_noise_psd_tolerance = 1e-12;
  return config;
}

TEST(ContinuousOutputTest, AnalyticNoiseAvoidsCancellationOfLargeMarginals) {
  const auto config = LinearizationConfig();
  auto previous = State(1, 0);
  Matrix15d basis = Matrix15d::Identity();
  for (int row = 1; row < 15; ++row)
    for (int col = 0; col < row; ++col)
      basis(row, col) = 0.17 * (row + 1) / (col + 2);
  previous.covariance = 1e8 * basis * basis.transpose();
  Matrix15d transition = Matrix15d::Identity();
  transition.block<3, 3>(0, 3).diagonal().setConstant(0.005);
  transition.block<3, 3>(3, 6).setConstant(0.0003);
  const Matrix15d noise = Matrix15d::Identity() * 1e-10;
  auto current = State(1.005, 0);
  current.covariance =
      transition * previous.covariance * transition.transpose() + noise;
  current.covariance =
      (0.5 * (current.covariance + current.covariance.transpose())).eval();
  ContinuousOutput subtracting(config), analytic(config);
  subtracting.Reset(previous);
  analytic.Reset(previous);
  EXPECT_EQ(Reason::INVALID_INPUT,
            subtracting.Advance(previous, current, transition, 0.005).reason);
  ASSERT_TRUE(analytic.Advance(previous, current, transition, 0.005, &noise).ok());
  EXPECT_TRUE(analytic.state().covariance.isApprox(current.covariance, 1e-14));
  EXPECT_TRUE(analytic.state().position.isZero());
  auto invalid = noise;
  invalid(0, 0) = -0.01;
  EXPECT_EQ(Reason::INVALID_INPUT,
            analytic.Advance(current, State(1.01, 0), transition, 0.005,
                             &invalid).reason);
  EXPECT_DOUBLE_EQ(1.005, analytic.state().stamp.time);
}

TEST(ContinuousOutputTest, ZeroNoiseKeepsFactorThroughContinuousPublication) {
  auto previous = State(1, 0);
  previous.covariance *= 1e-4;
  previous.covariance.diagonal().head<3>().setConstant(1e-12);
  ContinuousOutput output(LinearizationConfig());
  output.Reset(previous);
  constexpr double dt = 0.005;
  Matrix15d dynamics = Matrix15d::Zero();
  dynamics.block<3, 3>(0, 3).setIdentity();
  dynamics(3, 7) = 10;
  dynamics(4, 6) = -10;
  dynamics.block<3, 3>(3, 12) = -Eigen::Matrix3d::Identity();
  dynamics.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity();
  const Matrix15d transition = Matrix15d::Identity() + dynamics * dt +
                               dynamics * dynamics * (dt * dt / 2);
  const Matrix15d zero = Matrix15d::Zero();
  for (int step = 1; step <= 12000; ++step) {
    auto current = previous;
    current.stamp = {1 + dt * step, 1 + dt * step,
                     static_cast<uint64_t>(step + 1), "clock"};
    ASSERT_TRUE(
        AdvanceCovarianceSquareRoot(previous, transition, zero, &current).ok());
    ASSERT_TRUE(output.Advance(previous, current, transition, dt, &zero).ok())
        << step;
    previous = current;
  }
  ASSERT_TRUE(output.state().covariance_square_root);
  EXPECT_TRUE(ValidLocalCovariance(output.state()));
  EXPECT_TRUE(output.state().covariance.isApprox(previous.covariance, 1e-12));
  EXPECT_TRUE(output.state().position.isZero());
  EXPECT_TRUE(output.state().velocity.isZero());
}

TEST(ContinuousOutputTest, JointErrorMapRetainsEndpointAndCorrectionMemory) {
  auto config = LinearizationConfig();
  ContinuousOutput output(config);
  const auto published = State(1, 0);
  output.Reset(published);
  const auto previous = State(1, 0.1);
  const auto current = State(1.1, 0.2);
  ASSERT_TRUE(output.Advance(previous, current, Matrix15d::Identity(), 0.1).ok());
  const auto nominal = output.state();
  ContinuousOutputErrorTransition map;
  ASSERT_TRUE(output.LinearizeLastStep(&map).ok());
  EXPECT_EQ(map.epoch, current.epoch);
  EXPECT_EQ(map.start.sequence, published.stamp.sequence);
  EXPECT_EQ(map.end.sequence, current.stamp.sequence);
  EXPECT_TRUE(map.previous_internal_state.position.isApprox(previous.position, 0));
  EXPECT_TRUE(map.current_internal_state.position.isApprox(current.position, 0));
  EXPECT_TRUE(map.previous_output_state.position.isApprox(published.position, 0));
  EXPECT_TRUE(map.previous_correction_velocity.isZero());
  EXPECT_NEAR(map.current_correction_velocity.x(), 1, 1e-12);
  EXPECT_NEAR(map.previous_internal(0, 0), -0.5, 1e-8);
  EXPECT_NEAR(map.current_internal(0, 0), 1, 1e-8);
  EXPECT_NEAR(map.previous_output(0, 0), 0.5, 1e-8);
  EXPECT_NEAR(map.previous_internal(3, 0), -5, 1e-8);
  EXPECT_NEAR(map.current_internal(3, 0), 10, 1e-8);
  EXPECT_NEAR(map.previous_output(3, 0), -5, 1e-8);
  EXPECT_NEAR(map.previous_internal(15, 0), 10, 1e-8);
  EXPECT_NEAR(map.current_internal(15, 0), 0, 1e-8);
  EXPECT_NEAR(map.previous_output(15, 0), -10, 1e-8);
  EXPECT_NEAR(map.previous_output(0, 15), 0.05, 1e-8);
  EXPECT_NEAR(map.previous_output(3, 15), 0.5, 1e-8);
  EXPECT_NEAR(map.previous_output(15, 15), 0, 1e-8);
  EXPECT_TRUE((map.current_internal.block<3, 3>(6, 6).isApprox(
      Eigen::Matrix3d::Identity(), 1e-8)));
  EXPECT_NEAR(map.previous_internal(9, 9), -0.9, 1e-8);
  EXPECT_NEAR(map.current_internal(9, 9), 1, 1e-8);
  EXPECT_NEAR(map.previous_output(9, 9), 0.9, 1e-8);
  // A common translation is one shared gauge error, not three noise sources.
  const auto common = (map.previous_internal.col(0) +
                       map.current_internal.col(0) +
                       map.previous_output.col(0)).eval();
  EXPECT_NEAR(common[0], 1, 1e-8);
  EXPECT_NEAR(common[3], 0, 1e-8);
  EXPECT_NEAR(common[15], 0, 1e-8);
  EXPECT_TRUE(output.state().position.isApprox(nominal.position, 0));
  EXPECT_TRUE(output.state().covariance.isApprox(nominal.covariance, 0));
}

TEST(ContinuousOutputTest, CovarianceSeparatesPropagationNoiseAndCorrection) {
  const auto config = LinearizationConfig();
  ContinuousOutput output(config);
  output.Reset(State(1, 0));
  auto previous = State(1, 0.1);
  auto current = State(1.1, 0.1);
  previous.orientation = current.orientation =
      ExpRotation(Eigen::Vector3d(0, 0, 0.02));
  Matrix15d transition = Matrix15d::Identity();
  transition.block<3, 3>(0, 3).diagonal().setConstant(0.1);
  const Matrix15d noise = Matrix15d::Identity() * 0.002;
  ASSERT_TRUE(output.Advance(previous, current, transition, 0.1, &noise).ok());
  const auto& covariance = output.state().covariance;
  // The actual correction is 0.05 m, 0.5 m/s and 0.02 rad.
  EXPECT_NEAR(covariance(0, 0), 1.01 + 0.002 + 2 * 0.05 * 0.05, 1e-12);
  EXPECT_NEAR(covariance(3, 3), 1 + 0.002 + 2 * 0.5 * 0.5, 1e-12);
  EXPECT_NEAR(covariance(8, 8), 1 + 0.002 + 2 * 0.02 * 0.02, 1e-12);
  EXPECT_NEAR(covariance(0, 3), 0.1, 1e-12);
  EXPECT_NEAR(covariance(1, 1), 1.01 + 0.002, 1e-12);
  EXPECT_NEAR(covariance(6, 6), 1 + 0.002, 1e-12);
}

TEST(ContinuousOutputTest, LimitedMemoryUsesTheActualLimiterDerivative) {
  auto config = LinearizationConfig();
  config.lidar_max_translation_correction_rate = 0.2;
  config.lidar_max_velocity_correction_acceleration = 0.3;
  ContinuousOutput output(config);
  output.Reset(State(1, 0));
  const auto previous = State(1, 1);
  const auto current = State(1.1, 1.1);
  ASSERT_TRUE(output.Advance(previous, current, Matrix15d::Identity(), 0.1).ok());
  ContinuousOutputErrorTransition map;
  ASSERT_TRUE(output.LinearizeLastStep(&map).ok());
  EXPECT_NEAR(map.previous_internal(0, 0), -1, 1e-8);
  EXPECT_NEAR(map.previous_output(0, 0), 1, 1e-8);
  // Saturated acceleration preserves longitudinal prior correction velocity.
  EXPECT_NEAR(map.previous_output(15, 15), 1, 1e-8);
  EXPECT_NEAR(map.previous_output(0, 15), 0.1, 1e-8);
  EXPECT_NEAR(map.previous_output(16, 16), 0.85, 1e-8);
  EXPECT_NEAR(map.previous_internal(16, 1), 0.03, 1e-8);
  const auto next = State(1.2, 1.2);
  ASSERT_TRUE(output.Advance(current, next, Matrix15d::Identity(), 0.1).ok());
  ASSERT_TRUE(output.LinearizeLastStep(&map).ok());
  EXPECT_EQ(map.start.sequence, current.stamp.sequence);
  EXPECT_EQ(map.end.sequence, next.stamp.sequence);
  EXPECT_NEAR(output.state().position.x(), 0.206, 1e-12);
}

TEST(ContinuousOutputTest, AnalyticMapMatchesActualRotatedLimiterBranches) {
  for (int branches = 0; branches < 8; ++branches) {
    for (double dt : {0.1, 0.0002}) {
      SCOPED_TRACE(branches);
      SCOPED_TRACE(dt);
      auto config = LinearizationConfig();
      config.lidar_max_translation_correction_rate = branches & 1 ? 0.2 : 1e6;
      config.lidar_max_velocity_correction_acceleration = branches & 2 ? 0.3 : 1e9;
      config.lidar_max_rotation_correction_rate = branches & 4 ? 0.1 : 1e6;
      auto previous = State(1, 0.1);
      previous.position = Eigen::Vector3d(0.1, -0.2, 0.3);
      previous.orientation = ExpRotation(Eigen::Vector3d(0.2, -0.1, 0.3));
      auto current = State(1 + dt, 0.2);
      current.position = Eigen::Vector3d(0.2, -0.1, 0.35);
      current.orientation = ExpRotation(Eigen::Vector3d(-0.1, 0.3, 0.4));
      auto published = State(1, -0.2);
      published.position = Eigen::Vector3d(-0.2, 0.1, 0.05);
      published.orientation = ExpRotation(Eigen::Vector3d(0.1, -0.3, -0.2));
      ContinuousOutput output(config);
      output.Reset(published);
      ASSERT_TRUE(output.Advance(previous, current, Matrix15d::Identity(), dt).ok());
      const auto nominal = output.state();
      ContinuousOutputErrorTransition map;
      ASSERT_TRUE(output.LinearizeLastStep(&map).ok());
      constexpr double step = 1e-6;
      for (int input = 0; input < 3; ++input) {
        for (int column = 0; column < 15; ++column) {
          const auto evaluate = [&](double delta) {
            auto left = previous;
            auto right = current;
            auto start = published;
            auto& state = input == 0 ? left : input == 1 ? right : start;
            if (column < 3) state.position[column] += delta;
            else if (column < 6) state.velocity[column - 3] += delta;
            else if (column < 9) {
              Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
              rotation[column - 6] = delta;
              state.orientation *= ExpRotation(rotation);
            } else if (column < 12) state.gyro_bias[column - 9] += delta;
            else state.accel_bias[column - 12] += delta;
            ContinuousOutput perturbed(config);
            perturbed.Reset(start);
            EXPECT_TRUE(perturbed.Advance(left, right, Matrix15d::Identity(), dt).ok());
            const auto& next = perturbed.state();
            Eigen::Matrix<double, 15, 1> difference;
            difference.segment<3>(0) = next.position - nominal.position;
            difference.segment<3>(3) = next.velocity - nominal.velocity;
            difference.segment<3>(6) =
                LogRotation(nominal.orientation.conjugate() * next.orientation);
            difference.segment<3>(9) = next.gyro_bias - nominal.gyro_bias;
            difference.segment<3>(12) = next.accel_bias - nominal.accel_bias;
            return difference;
          };
          const Eigen::Matrix<double, 15, 1> derivative =
              (evaluate(step) - evaluate(-step)) / (2 * step);
          Eigen::Matrix<double, 15, 1> actual;
          if (input == 0) actual = map.previous_internal.col(column).head<15>();
          else if (input == 1) actual = map.current_internal.col(column).head<15>();
          else actual = map.previous_output.col(column).head<15>();
          EXPECT_LE((actual - derivative).cwiseAbs().maxCoeff(),
                    1e-6 * std::max(1.0, actual.cwiseAbs().maxCoeff()))
              << "input=" << input << " column=" << column;
        }
      }
    }
  }
}

TEST(ContinuousOutputTest, ResolvedNearLimiterBranchesKeepAnalyticDerivatives) {
  for (int limiter = 0; limiter < 3; ++limiter) {
    for (double side : {-1.0, 0.0, 1.0}) {
      SCOPED_TRACE(limiter);
      SCOPED_TRACE(side);
      auto config = LinearizationConfig();
      constexpr double dt = 0.005;
      constexpr double margin = 1e-7;
      config.lidar_max_translation_correction_rate = limiter == 0 ? 0.2 : 10;
      config.lidar_max_velocity_correction_acceleration = limiter == 1 ? 0.3 : 100;
      config.lidar_max_rotation_correction_rate = limiter == 2 ? 0.1 : 10;
      const auto published = State(1, 0);
      auto previous = published;
      double error = limiter == 0 ? 0.2 * dt + side * margin * dt
                     : limiter == 1 ? 0.3 * dt * dt + side * margin * dt
                                    : 0.1 * dt + side * margin;
      if (limiter == 2) previous.orientation = ExpRotation({error, 0, 0});
      else previous.position.x() = error;
      auto current = previous;
      current.stamp.time += dt;
      const auto evaluate = [&](double delta, ContinuousOutputErrorTransition* map) {
        auto start = published;
        if (limiter == 2) start.orientation = ExpRotation({delta, 0, 0});
        else start.position.x() += delta;
        ContinuousOutput output(config);
        output.Reset(start);
        EXPECT_TRUE(output.Advance(previous, current, Matrix15d::Identity(), dt).ok());
        if (map) {
          const auto result = output.LinearizeLastStep(map);
          EXPECT_EQ(result.reason, side == 0 ? Reason::CORRELATION_UNQUALIFIED
                                            : Reason::NONE);
        }
        return output.state();
      };
      ContinuousOutputErrorTransition map;
      evaluate(0, &map);
      if (side == 0) continue;
      constexpr double step = 1e-11;
      const auto plus = evaluate(step, nullptr);
      const auto minus = evaluate(-step, nullptr);
      const double derivative =
          limiter == 2
              ? LogRotation(minus.orientation.conjugate() * plus.orientation).x() /
                    (2 * step)
              : (plus.position.x() - minus.position.x()) / (2 * step);
      const int index = limiter == 2 ? 6 : 0;
      EXPECT_NEAR(map.previous_output(index, index), derivative, 1e-6);
      const double expected =
          side < 0 ? (limiter == 0 ? 0.5 : limiter == 1 ? 0.5 : 0.0) : 1.0;
      EXPECT_NEAR(derivative, expected, 1e-6);
    }
  }
}

TEST(ContinuousOutputTest, LimiterKinksAreExplicitAndCannotReuseOldLinearization) {
  auto config = LinearizationConfig();
  config.lidar_max_translation_correction_rate = 0.2;
  ContinuousOutput output(config);
  ContinuousOutputErrorTransition map;
  map.previous_output.setConstant(42);
  EXPECT_EQ(output.LinearizeLastStep(&map).reason, Reason::HISTORY_UNAVAILABLE);
  output.Reset(State(1, 0));
  ASSERT_TRUE(output.Advance(State(1, 0.02), State(1.1, 0.12),
                             Matrix15d::Identity(), 0.1).ok());
  EXPECT_EQ(output.LinearizeLastStep(&map).reason, Reason::CORRELATION_UNQUALIFIED);
  EXPECT_TRUE((map.previous_output.array() == 42).all());
  output.Reset(State(2, 0));
  EXPECT_EQ(output.LinearizeLastStep(&map).reason, Reason::HISTORY_UNAVAILABLE);
  EXPECT_EQ(output.LinearizeLastStep(nullptr).reason, Reason::INVALID_INPUT);
}

TEST(ContinuousOutputTest, CorrectionStartsWithoutSameTimePoseJump) {
  LocalConfig config;
  config.lidar_max_translation_correction_rate = 0.2;
  config.lidar_max_rotation_correction_rate = 0.1;
  config.lidar_max_velocity_correction_acceleration = 0.3;
  config.lidar_correction_variance_scale = 2.0;
  config.lidar_max_translation_correction = 0.5;
  config.lidar_max_rotation_correction = 1.0;
  config.lidar_max_velocity_correction = 3.0;
  config.lidar_process_noise_psd_tolerance = 1e-12;
  ContinuousOutput output(config);
  LocalState published = State(1.0, 0.0);
  published.velocity.x() = 1.0;
  output.Reset(published);

  LocalState corrected = State(1.0, 1.0);
  corrected.velocity.x() = 1.0;
  corrected.covariance_model_valid = false;
  EXPECT_DOUBLE_EQ(output.state().position.x(), 0.0);
  EXPECT_DOUBLE_EQ(output.state().stamp.time, corrected.stamp.time);

  LocalState propagated = State(1.1, 1.1);
  propagated.velocity.x() = 1.0;
  Matrix15d transition = Matrix15d::Identity();
  ASSERT_TRUE(output.Advance(corrected, propagated, transition, 0.1).ok());
  EXPECT_FALSE(output.state().covariance_model_valid);
  EXPECT_NEAR(output.state().position.x(), 0.1015, 1e-12);
  EXPECT_NEAR(output.state().position.x() - published.position.x(), 0.1015,
              1e-12);
  EXPECT_NEAR(output.state().velocity.x(), 1.015, 1e-12);
  EXPECT_GT(output.state().covariance(0, 0), published.covariance(0, 0));
  EXPECT_TRUE(output.correction_budget_exceeded());
  EXPECT_EQ(output.correction_status().reason,
            Reason::CORRECTION_BUDGET_EXCEEDED);
  EXPECT_NE(output.correction_status().message.find("translation_m=1 "),
            std::string::npos);
  EXPECT_NE(output.correction_status().message.find("translation_limit_m=0.5 "),
            std::string::npos);

  const LocalState next_internal = State(1.2, 1.2);
  LocalState next = next_internal;
  next.velocity.x() = 1.0;
  ASSERT_TRUE(output.Advance(propagated, next, transition, 0.1).ok());
  EXPECT_NEAR(output.state().position.x(), 0.206, 1e-12);
  EXPECT_NEAR(output.state().velocity.x(), 1.045, 1e-12);
  output.Reset(next);
  EXPECT_FALSE(output.correction_budget_exceeded());
  EXPECT_TRUE(output.correction_status().ok());
}

TEST(ContinuousOutputTest, RotationVelocityAndCovarianceUseExplicitBudgets) {
  LocalConfig config;
  config.lidar_max_translation_correction_rate = 0.1;
  config.lidar_max_rotation_correction_rate = 0.2;
  config.lidar_max_velocity_correction_acceleration = 0.4;
  config.lidar_correction_variance_scale = 3.0;
  config.lidar_max_translation_correction = 2.0;
  config.lidar_max_rotation_correction = 1.0;
  config.lidar_max_velocity_correction = 3.0;
  config.lidar_process_noise_psd_tolerance = 1e-12;
  ContinuousOutput output(config);
  LocalState published = State(2.0, 0.0);
  published.covariance *= 0.5;
  output.Reset(published);

  LocalState corrected = published;
  corrected.orientation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  corrected.velocity =
      corrected.orientation * Eigen::Vector3d(2.0, 0.0, 0.0);
  LocalState propagated = corrected;
  propagated.stamp = {2.2, 2.2, 22, "clock"};
  propagated.position =
      corrected.orientation * Eigen::Vector3d(0.4, 0.0, 0.0);
  Matrix15d transition = Matrix15d::Identity();
  ASSERT_TRUE(output.Advance(corrected, propagated, transition, 0.2).ok());
  EXPECT_NEAR(LogRotation(output.state().orientation).norm(), 0.04, 1e-12);
  EXPECT_TRUE(output.state().velocity.isApprox(
      (output.state().position - published.position) / 0.2, 1e-12));
  EXPECT_GT(output.state().covariance(3, 3),
            published.covariance(3, 3));
  EXPECT_GT(output.state().covariance(8, 8), published.covariance(8, 8));
  EXPECT_TRUE(output.correction_status().ok());
}

TEST(ContinuousOutputTest, RejectsInconsistentProcessCovariance) {
  LocalConfig config;
  config.lidar_max_translation_correction_rate = 0.1;
  config.lidar_max_rotation_correction_rate = 0.1;
  config.lidar_max_velocity_correction_acceleration = 0.1;
  config.lidar_correction_variance_scale = 1.0;
  config.lidar_max_translation_correction = 1.0;
  config.lidar_max_rotation_correction = 1.0;
  config.lidar_max_velocity_correction = 1.0;
  config.lidar_process_noise_psd_tolerance = 1e-12;
  ContinuousOutput output(config);
  const LocalState start = State(3.0, 0.0);
  output.Reset(start);
  LocalState end = State(3.1, 0.0);
  end.covariance *= 0.5;
  EXPECT_EQ(output.Advance(start, end, Matrix15d::Identity(), 0.1).reason,
            Reason::INVALID_INPUT);
}

TEST(ContinuousOutputTest, SustainedCorrectionDemandRetainsBudgetDegradation) {
  LocalConfig config;
  config.lidar_max_translation_correction_rate = 0.2;
  config.lidar_max_rotation_correction_rate = 0.1;
  config.lidar_max_velocity_correction_acceleration = 0.3;
  config.lidar_correction_variance_scale = 2.0;
  config.lidar_max_translation_correction = 2.0;
  config.lidar_max_rotation_correction = 0.5;
  config.lidar_max_velocity_correction = 2.0;
  config.lidar_process_noise_psd_tolerance = 1e-12;
  ContinuousOutput output(config);
  LocalState internal = State(1.0, 0.0);
  output.Reset(internal);
  constexpr double dt = 0.1;
  for (int step = 1; step <= 100; ++step) {
    // Same-time LIO corrections cannot be instantaneously copied to output.
    internal.position.x() -= 0.06;
    const LocalState corrected = internal;
    internal.stamp.time += dt;
    internal.stamp.receive_time = internal.stamp.time;
    const double previous_output = output.state().position.x();
    ASSERT_TRUE(output.Advance(corrected, internal,
                               Matrix15d::Identity(), dt).ok());
    EXPECT_LE(std::abs(output.state().position.x() - previous_output),
              config.lidar_max_translation_correction_rate * dt + 1e-12);
  }
  EXPECT_GE(std::abs(internal.position.x() - output.state().position.x()), 4.0);
  EXPECT_EQ(output.correction_status().reason,
            Reason::CORRECTION_BUDGET_EXCEEDED);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace openloc
