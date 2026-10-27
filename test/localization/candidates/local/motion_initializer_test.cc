// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/local/motion_initializer.h"

#include <cmath>

#include "gtest/gtest.h"

#include "candidates/local/local_estimator.h"

namespace openloc {
namespace localization {
namespace unified {
namespace {

struct Evidence {
  std::vector<RelativeLidarPose> poses;
  std::vector<ImuSample> imu;
  Eigen::Vector3d accel_bias{0.1, -0.08, 0.06};
  Eigen::Vector3d gyro_bias{0.01, -0.015, 0.02};
  Eigen::Vector3d acceleration{0.2, -0.1, 0.05};
  Eigen::Vector3d initial_velocity{0.4, 0.2, -0.1};
};

Evidence Generate(bool rotate) {
  Evidence evidence;
  constexpr double interval = 0.25;
  constexpr int knots = 21;
  for (int i = 0; i < knots; ++i) {
    const double t = interval * i;
    RelativeLidarPose pose;
    pose.time = 1.0 + t;
    if (rotate) {
      pose.reference_from_base.linear() =
          (Eigen::AngleAxisd(0.5 * std::sin(t), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(0.4 * std::sin(1.7 * t),
                             Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(0.2 * t, Eigen::Vector3d::UnitZ()))
              .toRotationMatrix();
    }
    pose.reference_from_base.translation() =
        evidence.initial_velocity * t + 0.5 * evidence.acceleration * t * t;
    evidence.poses.push_back(pose);
  }
  for (int i = 0; i <= 1000; ++i) {
    const double t = i * 0.005;
    const int segment = std::min(i / 50, knots - 2);
    const Eigen::Quaterniond q0(
        evidence.poses[segment].reference_from_base.linear());
    const Eigen::Quaterniond q1(
        evidence.poses[segment + 1].reference_from_base.linear());
    const double alpha = (t - segment * interval) / interval;
    const Eigen::Quaterniond q = q0.slerp(alpha, q1);
    ImuSample imu;
    imu.stamp = {1.0 + t, 1.0 + t, static_cast<uint64_t>(i + 1), "unix"};
    imu.acceleration = q.conjugate() * (evidence.acceleration +
                                        Eigen::Vector3d(0.0, 0.0, 9.80665)) +
                       evidence.accel_bias;
    imu.angular_velocity =
        LogRotation(q0.conjugate() * q1) / interval + evidence.gyro_bias;
    evidence.imu.push_back(imu);
  }
  return evidence;
}

Evidence GenerateVariableMotion() {
  auto evidence = Generate(true);
  const auto orientation = [](double t) -> Eigen::Quaterniond {
    return Eigen::AngleAxisd(0.5 * std::sin(t), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(0.4 * std::sin(1.7 * t),
                             Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(0.2 * t, Eigen::Vector3d::UnitZ());
  };
  for (auto& pose : evidence.poses) {
    const double t = pose.time - 1.0;
    pose.reference_from_base.linear() = orientation(t).toRotationMatrix();
    pose.reference_from_base.translation() =
        evidence.initial_velocity * t +
        Eigen::Vector3d(0.5 * (std::sin(1.3 * t) - 1.3 * t),
                        0.4 * (std::cos(0.7 * t) - 1.0),
                        0.2 * (std::sin(1.1 * t) - 1.1 * t));
  }
  for (auto& sample : evidence.imu) {
    const double t = sample.stamp.time - 1.0;
    const Eigen::Vector3d acceleration(
        -0.845 * std::sin(1.3 * t), -0.196 * std::cos(0.7 * t),
        -0.242 * std::sin(1.1 * t));
    sample.acceleration =
        orientation(t).conjugate() *
            (acceleration + Eigen::Vector3d(0.0, 0.0, 9.80665)) +
        evidence.accel_bias;
    constexpr double h = 1e-5;
    sample.angular_velocity =
        LogRotation(orientation(t - h).conjugate() * orientation(t + h)) /
            (2.0 * h) +
        evidence.gyro_bias;
  }
  return evidence;
}

TEST(MotionInitializerTest, RecoversVariableTranslationAndAttitude) {
  const auto evidence = GenerateVariableMotion();
  MotionInitialization result;
  MotionInitializationDiagnostics diagnostics;
  const auto status = InitializeFromRelativeMotion(
      {}, evidence.poses, evidence.imu, &result, &diagnostics);
  ASSERT_TRUE(status.ok()) << status.message;
  const Eigen::Vector3d final_velocity =
      evidence.initial_velocity +
      Eigen::Vector3d(0.65 * (std::cos(6.5) - 1.0),
                      -0.28 * std::sin(3.5),
                      0.22 * (std::cos(5.5) - 1.0));
  EXPECT_LT((result.velocity - final_velocity).norm(), 0.003);
  EXPECT_LT((result.accel_bias - evidence.accel_bias).norm(), 0.003);
  EXPECT_LT((result.gyro_bias - evidence.gyro_bias).norm(), 0.003);
  EXPECT_TRUE(diagnostics.linear_system_evaluated);
  EXPECT_TRUE(diagnostics.solution_available);
  EXPECT_LT(diagnostics.gravity_error, 0.003);
  EXPECT_LT(diagnostics.angular_residual_rms, 0.003);
}

TEST(MotionInitializerTest, RejectsAccelerativePoseDriftDespiteSmallResidual) {
  auto evidence = GenerateVariableMotion();
  MotionInitialization result;
  MotionInitializationDiagnostics exact;
  ASSERT_TRUE(InitializeFromRelativeMotion(
                  {}, evidence.poses, evidence.imu, &result, &exact)
                  .ok());
  for (auto& pose : evidence.poses) {
    const double t = pose.time - evidence.poses.front().time;
    pose.reference_from_base.translation().z() -= 0.5 * 0.8 * t * t;
  }
  MotionInitializationDiagnostics drifted;
  result.time = 123.0;
  const auto status = InitializeFromRelativeMotion(
      {}, evidence.poses, evidence.imu, &result, &drifted);
  ASSERT_EQ(status.reason, Reason::INNOVATION_REJECTED) << status.message;
  EXPECT_DOUBLE_EQ(result.time, 123.0);
  ASSERT_TRUE(drifted.solution_available);
  EXPECT_NEAR(drifted.gravity_error, 0.8, 0.003);
  EXPECT_DOUBLE_EQ(drifted.minimum_scaled_singular_value,
                   exact.minimum_scaled_singular_value);
  EXPECT_NEAR(drifted.normalized_residual_rms,
              exact.normalized_residual_rms, 1e-9);
  EXPECT_DOUBLE_EQ(drifted.angular_residual_rms, exact.angular_residual_rms);
  EXPECT_NEAR(drifted.accel_bias_norm, exact.accel_bias_norm, 1e-9);
  evidence.poses.clear();
  EXPECT_EQ(InitializeFromRelativeMotion(
                {}, evidence.poses, evidence.imu, &result, &drifted)
                .reason,
            Reason::HISTORY_UNAVAILABLE);
  EXPECT_FALSE(drifted.linear_system_evaluated);
  EXPECT_FALSE(drifted.solution_available);
}

TEST(MotionInitializerTest,
     RecoversExcitedRelativeMotionWithoutZeroSpeedPrior) {
  const auto evidence = Generate(true);
  MotionInitialization result;
  const auto status =
      InitializeFromRelativeMotion({}, evidence.poses, evidence.imu, &result);
  ASSERT_TRUE(status.ok()) << status.message;
  EXPECT_LT((result.accel_bias - evidence.accel_bias).norm(), 0.002);
  EXPECT_LT((result.gyro_bias - evidence.gyro_bias).norm(), 0.003);
  EXPECT_LT((result.velocity -
             (evidence.initial_velocity + evidence.acceleration * 5.0))
                .norm(),
            0.003);
  EXPECT_LT(LogRotation(Eigen::Quaterniond(
                            evidence.poses.back().reference_from_base.linear())
                            .conjugate() *
                        result.orientation)
                .norm(),
            0.002);
  EXPECT_GT(result.minimum_scaled_singular_value, 1e-3);
}

TEST(MotionInitializerTest, IntegratesChangingAngularRateBetweenLidarPoses) {
  auto evidence = Generate(true);
  const auto orientation = [&](double t) {
    constexpr double interval = 0.25;
    const int segment =
        std::max(0, std::min(static_cast<int>(t / interval), 19));
    const double alpha = (t - segment * interval) / interval;
    const Eigen::Quaterniond first(
        evidence.poses[segment].reference_from_base.linear());
    const Eigen::Quaterniond second(
        evidence.poses[segment + 1].reference_from_base.linear());
    return (first.slerp(alpha, second) *
            ExpRotation(Eigen::Vector3d(
                0.0, 0.0, 0.03 * std::sin(2.0 * std::acos(-1.0) * alpha))))
        .normalized();
  };
  for (auto& sample : evidence.imu) {
    const double t = sample.stamp.time - 1.0;
    sample.acceleration =
        orientation(t).conjugate() *
            (evidence.acceleration + Eigen::Vector3d(0.0, 0.0, 9.80665)) +
        evidence.accel_bias;
    constexpr double h = 1e-5;
    sample.angular_velocity =
        LogRotation(orientation(t - h).conjugate() * orientation(t + h)) /
            (2.0 * h) +
        evidence.gyro_bias;
  }
  MotionInitialization result;
  const auto status =
      InitializeFromRelativeMotion({}, evidence.poses, evidence.imu, &result);
  ASSERT_TRUE(status.ok()) << status.message;
  EXPECT_LT((result.gyro_bias - evidence.gyro_bias).norm(), 0.003);
  EXPECT_LT((result.accel_bias - evidence.accel_bias).norm(), 0.01);
  EXPECT_LT(result.angular_residual_rms, 0.003);
}

TEST(MotionInitializerTest, RejectsConstantAttitudeAndPreservesPreviousResult) {
  const auto evidence = Generate(false);
  MotionInitialization output;
  output.time = 123.0;
  const auto result =
      InitializeFromRelativeMotion({}, evidence.poses, evidence.imu, &output);
  EXPECT_EQ(result.reason, Reason::DEGENERATE);
  EXPECT_DOUBLE_EQ(output.time, 123.0);
}

TEST(MotionInitializerTest, LidarRestIsExplicitAndRejectsConstantVelocity) {
  auto evidence = Generate(false);
  for (auto& pose : evidence.poses)
    pose.reference_from_base.translation().setZero();
  for (auto& sample : evidence.imu) {
    sample.acceleration = Eigen::Vector3d(0.0, 0.0, 9.80665);
    sample.angular_velocity = Eigen::Vector3d(0.03, -0.01, 0.02);
  }
  MotionInitialization result;
  MotionInitializationDiagnostics diagnostics;
  EXPECT_EQ(InitializeFromRelativeMotion(
                {}, evidence.poses, evidence.imu, &result, &diagnostics).reason,
            Reason::DEGENERATE);
  MotionInitializationConfig policy;
  policy.allow_lidar_stationary_initialization = true;
  ASSERT_TRUE(InitializeFromRelativeMotion(
                  policy, evidence.poses, evidence.imu, &result, &diagnostics)
                  .ok());
  EXPECT_TRUE(result.lidar_stationary_initialization);
  EXPECT_TRUE(diagnostics.lidar_stationary_initialization);
  EXPECT_FALSE(diagnostics.linear_system_evaluated);
  EXPECT_FALSE(diagnostics.solution_available);
  EXPECT_LT((result.gyro_bias - evidence.imu.front().angular_velocity).norm(),
            1e-8);
  EXPECT_TRUE(result.velocity.isZero());
  EXPECT_TRUE(result.accel_bias.isZero());
  EXPECT_LT(LogRotation(result.orientation).norm(), 1e-8);
  for (auto& pose : evidence.poses)
    pose.reference_from_base.translation().x() = 0.2 * (pose.time - 1.0);
  result.time = 123.0;
  EXPECT_EQ(InitializeFromRelativeMotion(
                policy, evidence.poses, evidence.imu, &result, &diagnostics).reason,
            Reason::DEGENERATE);
  EXPECT_DOUBLE_EQ(result.time, 123.0);
  EXPECT_FALSE(diagnostics.lidar_stationary_initialization);
  EXPECT_NEAR(diagnostics.maximum_lidar_speed, 0.2, 1e-12);
}

TEST(MotionInitializerTest, LidarRestRejectsInconsistentForceAndCommitsUnqualified) {
  auto evidence = Generate(false);
  for (auto& pose : evidence.poses)
    pose.reference_from_base.translation().setZero();
  for (auto& sample : evidence.imu) {
    sample.acceleration = Eigen::Vector3d(0.0, 0.0, 9.80665);
    sample.angular_velocity = Eigen::Vector3d(0.03, -0.01, 0.02);
  }
  MotionInitializationConfig policy;
  policy.allow_lidar_stationary_initialization = true;
  auto inconsistent = evidence.imu;
  inconsistent[500].acceleration.x() = 1.0;
  MotionInitialization result;
  MotionInitializationDiagnostics diagnostics;
  EXPECT_EQ(InitializeFromRelativeMotion(
                policy, evidence.poses, inconsistent, &result, &diagnostics).reason,
            Reason::DEGENERATE);
  EXPECT_FALSE(diagnostics.lidar_stationary_initialization);
  EXPECT_GT(diagnostics.maximum_stationary_force_residual, 0.9);
  LocalConfig config;
  config.session = "lidar-rest";
  config.enable_motion_initialization = true;
  config.enable_lidar_stationary_initialization = true;
  LocalEstimator estimator(config);
  for (const auto& sample : evidence.imu)
    ASSERT_TRUE(estimator.AddImu(sample).ok());
  EXPECT_FALSE(estimator.state().valid);
  ASSERT_TRUE(estimator.InitializeMotion(evidence.poses, &diagnostics).ok());
  EXPECT_TRUE(diagnostics.lidar_stationary_initialization);
  EXPECT_TRUE(estimator.state().valid);
  EXPECT_FALSE(estimator.state().covariance_model_valid);
  EXPECT_FALSE(estimator.state().uncertainty_qualified);
  EXPECT_EQ(estimator.last_motion(), nullptr);
  EXPECT_EQ(estimator.InitializeMotion(evidence.poses).reason,
            Reason::INVALID_INPUT);
}

TEST(MotionInitializerTest, RejectsGapAndUnsupportedMeasurementTime) {
  auto evidence = Generate(true);
  evidence.imu.erase(evidence.imu.begin() + 200, evidence.imu.begin() + 230);
  MotionInitialization output;
  EXPECT_EQ(
      InitializeFromRelativeMotion({}, evidence.poses, evidence.imu, &output)
          .reason,
      Reason::IMU_GAP);
  evidence = Generate(true);
  evidence.poses.front().time -= 0.01;
  EXPECT_EQ(
      InitializeFromRelativeMotion({}, evidence.poses, evidence.imu, &output)
          .reason,
      Reason::HISTORY_UNAVAILABLE);
}

TEST(MotionInitializerTest, RejectsExcessBiasWithoutCommittingResult) {
  auto evidence = Generate(true);
  for (auto& sample : evidence.imu) sample.angular_velocity.x() += 1.0;
  MotionInitialization output;
  output.time = 123.0;
  EXPECT_EQ(
      InitializeFromRelativeMotion({}, evidence.poses, evidence.imu, &output)
          .reason,
      Reason::INNOVATION_REJECTED);
  EXPECT_DOUBLE_EQ(output.time, 123.0);
}

TEST(MotionInitializerTest, CommitsOnlyOnceAtReceivedImuFrontierUnqualified) {
  const auto evidence = Generate(true);
  LocalConfig config;
  config.session = "motion-start-test";
  config.enable_motion_initialization = true;
  LocalEstimator estimator(config);
  for (const auto& sample : evidence.imu)
    ASSERT_TRUE(estimator.AddImu(sample).ok());
  auto poses = evidence.poses;
  poses.back().time -= 0.001;
  EXPECT_EQ(estimator.InitializeMotion(poses).reason,
            Reason::HISTORY_UNAVAILABLE);
  EXPECT_FALSE(estimator.state().valid);
  ASSERT_TRUE(estimator.InitializeMotion(evidence.poses).ok());
  EXPECT_TRUE(estimator.state().valid);
  EXPECT_FALSE(estimator.state().covariance_model_valid);
  EXPECT_FALSE(estimator.state().uncertainty_qualified);
  EXPECT_EQ(estimator.last_motion(), nullptr);
  EXPECT_TRUE(estimator.state().position.isZero());
  EXPECT_EQ(estimator.state().stamp.time, evidence.imu.back().stamp.time);
  EXPECT_EQ(estimator.InitializeMotion(evidence.poses).reason,
            Reason::INVALID_INPUT);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace openloc
