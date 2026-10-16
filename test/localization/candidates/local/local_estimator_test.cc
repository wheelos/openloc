// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "candidates/local/local_estimator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "Eigen/Cholesky"

namespace openloc {
namespace localization {
namespace unified {

class LocalEstimatorTestAccess {
 public:
  static void SetWheelBiasFeedback(LocalEstimator* estimator, bool gyro,
                                   bool accel, bool forward_only = false) {
    estimator->wheel_updates_gyro_bias_ = gyro;
    estimator->wheel_updates_accel_bias_ = accel;
    estimator->wheel_forward_velocity_only_ = forward_only;
  }
  static Result Propagate(const LocalEstimator& estimator,
                          const ImuSample& start, const ImuSample& end,
                          LocalState* state, Matrix15d* transition) {
    return estimator.Propagate(start, end, state, transition);
  }
  static Result UpdateWheel(LocalEstimator* estimator, const WheelSample& wheel,
                            const ImuSample& imu, LocalState* state,
                            Matrix15d* transition) {
    return estimator->UpdateWheel(wheel, imu, state, transition);
  }
};

namespace {

LocalConfig TestConfig() {
  LocalConfig config;
  config.session = "local-test-session";
  config.clock_id = "test-clock";
  config.gravity = 10.0;
  config.stationary_samples = 3;
  config.max_sample_age = 0.1;
  config.future_tolerance = 0.001;
  config.max_imu_gap = 0.025;
  config.wheel_timeout = 0.05;
  config.stationary_speed = 0.01;
  config.stationary_gyro = 0.03;
  config.stationary_accel_tolerance = 0.1;
  config.stationary_duration = 0.02;
  config.gyro_noise = 0.001;
  config.accel_noise = 0.01;
  config.gyro_bias_noise = 0.00001;
  config.accel_bias_noise = 0.0001;
  config.initial_velocity_std = 0.1;
  config.initial_attitude_std = 0.01;
  config.initial_gyro_bias_std = 0.001;
  config.initial_accel_bias_std = 0.01;
  config.zero_velocity_variance = 0.001;
  config.max_position_std = 10.0;
  config.max_velocity_std = 10.0;
  config.max_attitude_std = 1.0;
  config.innovation_gate = 100.0;
  return config;
}

void EnableTestLidar(LocalConfig* config) {
  config->enable_lidar = true;
  config->lidar_frame = "lidar";
  config->lidar_calibration_id = "test-lidar-calibration";
  config->lidar_point_noise_std = 0.01;
  config->lidar_plane_noise_std = 0.01;
  config->lidar_voxel_size = 1.0;
  config->lidar_max_voxels = 100;
  config->lidar_max_scans = 3;
  config->lidar_min_points_per_plane = 3;
  config->lidar_min_correspondences = 2;
  config->lidar_max_points = 1000;
  config->lidar_max_correspondence_distance = 0.3;
  config->lidar_observed_singular_value = 1.0;
  config->lidar_innovation_gate = 100.0;
  config->lidar_max_scan_age = 0.1;
  config->lidar_future_tolerance = 0.001;
  config->lidar_time_tolerance = 0.0;
  config->lidar_max_translation_correction_rate = 0.2;
  config->lidar_max_rotation_correction_rate = 0.1;
  config->lidar_max_velocity_correction_acceleration = 0.3;
  config->lidar_anchor_variance_scale = 1.0;
  config->lidar_correction_variance_scale = 2.0;
  config->lidar_max_translation_correction = 2.0;
  config->lidar_max_rotation_correction = 0.5;
  config->lidar_max_velocity_correction = 2.0;
  config->lidar_max_update_iterations = 2;
  config->lidar_process_noise_psd_tolerance = 1e-12;
  config->lidar_rotation_length = 1.0;
}

LidarScan CorridorScan(const LocalState& state, uint64_t sequence,
                       double lateral_offset = 0.0) {
  LidarScan scan;
  scan.stamp = state.stamp;
  scan.stamp.sequence = sequence;
  scan.epoch = state.epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "test-lidar-calibration";
  for (double x : {0.1, 0.3, 0.5}) {
    for (double z : {0.1, 0.3, 0.5}) {
      scan.points.push_back(
          {Eigen::Vector3d(x, 1.0 + lateral_offset, z), state.stamp.time});
      scan.points.push_back(
          {Eigen::Vector3d(x, -1.0 + lateral_offset, z), state.stamp.time});
    }
  }
  return scan;
}

Stamp MakeStamp(double time, uint64_t sequence) {
  Stamp stamp;
  stamp.time = time;
  stamp.receive_time = time;
  stamp.sequence = sequence;
  stamp.clock_id = "test-clock";
  return stamp;
}

ImuSample MakeImu(double time, uint64_t sequence) {
  ImuSample sample;
  sample.stamp = MakeStamp(time, sequence);
  sample.acceleration.z() = 10.0;
  return sample;
}

WheelSample MakeWheel(double time, uint64_t sequence, double speed = 0.0) {
  WheelSample sample;
  sample.stamp = MakeStamp(time, sequence);
  sample.speed = speed;
  sample.variance = 0.01;
  return sample;
}

void ExpectSameState(const LocalState& before, const LocalState& after) {
  EXPECT_EQ(before.stamp.time, after.stamp.time);
  EXPECT_EQ(before.stamp.receive_time, after.stamp.receive_time);
  EXPECT_EQ(before.stamp.sequence, after.stamp.sequence);
  EXPECT_EQ(before.stamp.clock_id, after.stamp.clock_id);
  EXPECT_TRUE(before.epoch == after.epoch);
  EXPECT_EQ((before.position - after.position).norm(), 0.0);
  EXPECT_EQ((before.velocity - after.velocity).norm(), 0.0);
  EXPECT_EQ((before.orientation.coeffs() - after.orientation.coeffs()).norm(),
            0.0);
  EXPECT_EQ((before.gyro_bias - after.gyro_bias).norm(), 0.0);
  EXPECT_EQ((before.accel_bias - after.accel_bias).norm(), 0.0);
  EXPECT_EQ((before.angular_velocity - after.angular_velocity).norm(), 0.0);
  EXPECT_EQ((before.covariance - after.covariance).norm(), 0.0);
  EXPECT_EQ(before.covariance_model_valid, after.covariance_model_valid);
  EXPECT_EQ(before.valid, after.valid);
}

void ExpectPositiveCovariance(const LocalState& state) {
  EXPECT_TRUE(state.covariance.allFinite());
  EXPECT_TRUE(state.covariance.isApprox(state.covariance.transpose(), 1e-12));
  Eigen::LLT<Matrix15d> factor(state.covariance);
  EXPECT_EQ(factor.info(), Eigen::Success);
  for (int i = 0; i < 15; ++i) {
    EXPECT_GT(state.covariance(i, i), 0.0);
  }
}

class LocalEstimatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_ = TestConfig();
    estimator_.reset(new LocalEstimator(config_));
  }

  void Initialize() {
    for (uint64_t sequence = 1; sequence <= config_.stationary_samples;
         ++sequence) {
      const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
      ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
      const Result result = estimator_->AddImu(MakeImu(time, sequence));
      ASSERT_TRUE(result.ok()) << result.message;
      if (sequence < config_.stationary_samples) {
        ASSERT_EQ(estimator_->Evaluate(time).reason,
                  Reason::WAITING_FOR_STATIONARY);
      }
    }
    ASSERT_TRUE(estimator_->state().valid);
  }

  LocalConfig config_;
  std::unique_ptr<LocalEstimator> estimator_;
};

TEST(LocalEstimatorConfigTest, RejectsMissingInvalidAndUnrepresentableConfig) {
  std::vector<LocalConfig> invalid;
  LocalConfig config = TestConfig();
  config.session.clear();
  invalid.push_back(config);
  config = TestConfig();
  config.clock_id.clear();
  invalid.push_back(config);
  config = TestConfig();
  config.stationary_samples = 0;
  invalid.push_back(config);
  config = TestConfig();
  config.stationary_duration = 0.0;
  invalid.push_back(config);
  config = TestConfig();
  config.gravity = 0.0;
  invalid.push_back(config);
  config = TestConfig();
  config.max_imu_gap = -1.0;
  invalid.push_back(config);
  config = TestConfig();
  config.future_tolerance = -1.0;
  invalid.push_back(config);
  config = TestConfig();
  config.gyro_noise = std::numeric_limits<double>::quiet_NaN();
  invalid.push_back(config);
  config = TestConfig();
  config.accel_noise = std::numeric_limits<double>::max();
  invalid.push_back(config);
  config = TestConfig();
  config.initial_attitude_std = 0.0;
  invalid.push_back(config);
  config = TestConfig();
  config.initial_velocity_std = config.max_velocity_std + 1.0;
  invalid.push_back(config);
  config = TestConfig();
  config.stationary_accel_tolerance = config.gravity;
  invalid.push_back(config);
  config = TestConfig();
  config.nonholonomic_variance = 0.0;
  invalid.push_back(config);
  config = TestConfig();
  config.enable_zero_velocity_update = true;
  config.zero_velocity_variance = 0.0;
  invalid.push_back(config);
  config = TestConfig();
  config.wheel_timeout = std::numeric_limits<double>::infinity();
  invalid.push_back(config);
  config = TestConfig();
  config.history_duration = 0.0;
  invalid.push_back(config);
  config = TestConfig();
  config.max_history_states = 1;
  invalid.push_back(config);
  config = TestConfig();
  config.enable_lidar = true;
  invalid.push_back(config);
  config = TestConfig();
  EnableTestLidar(&config);
  config.lidar_time_tolerance = 1e-6;
  invalid.push_back(config);
  for (const auto& item : invalid) {
    LocalEstimator estimator(item);
    EXPECT_EQ(estimator.ValidateConfig().reason, Reason::CONFIG_INVALID);
    EXPECT_EQ(estimator.AddImu(MakeImu(1.0, 1)).reason, Reason::CONFIG_INVALID);
    EXPECT_EQ(estimator.AddWheel(MakeWheel(1.0, 1)).reason,
              Reason::CONFIG_INVALID);
    EXPECT_EQ(estimator.EvaluateIntegrity(1.0).reason, Reason::CONFIG_INVALID);
    EXPECT_EQ(estimator.Evaluate(1.0).reason, Reason::CONFIG_INVALID);
    EXPECT_EQ(estimator.ResetStopped(1.0).reason, Reason::CONFIG_INVALID);
    EXPECT_FALSE(estimator.state().valid);
  }
  EXPECT_TRUE(LocalEstimator(TestConfig()).ValidateConfig().ok());
}

TEST_F(LocalEstimatorTest, StationaryStartupNeedsWheelAndConsecutiveSamples) {
  EXPECT_EQ(estimator_->Evaluate(1.0).reason, Reason::WAITING_FOR_IMU);
  EXPECT_EQ(estimator_->EvaluateIntegrity(1.0).reason, Reason::WAITING_FOR_IMU);
  EXPECT_TRUE(estimator_->AddImu(MakeImu(1.0, 1)).ok());
  EXPECT_EQ(estimator_->Evaluate(1.0).reason, Reason::WAITING_FOR_STATIONARY);
  EXPECT_EQ(estimator_->EvaluateIntegrity(1.0).reason,
            Reason::WAITING_FOR_STATIONARY);
  EXPECT_EQ(estimator_->state().stamp.time, 1.0);
  EXPECT_FALSE(estimator_->state().valid);
  for (uint64_t sequence = 2; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    EXPECT_TRUE(estimator_->AddImu(MakeImu(time, sequence)).ok());
    EXPECT_EQ(estimator_->Evaluate(time).reason,
              Reason::WAITING_FOR_STATIONARY);
  }
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4, 0.1)).ok());
  EXPECT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  EXPECT_EQ(estimator_->Evaluate(1.03).reason, Reason::WAITING_FOR_STATIONARY);
  for (uint64_t sequence = 5; sequence <= 7; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    const Result result = estimator_->AddImu(MakeImu(time, sequence));
    EXPECT_TRUE(result.ok());
    EXPECT_EQ(estimator_->Evaluate(time).reason,
              sequence < 7 ? Reason::WAITING_FOR_STATIONARY : Reason::NONE);
  }
  EXPECT_TRUE(estimator_->state().valid);
  EXPECT_EQ(estimator_->state().position.norm(), 0.0);
  EXPECT_EQ(estimator_->state().velocity.norm(), 0.0);
  EXPECT_EQ(estimator_->state().epoch.session, config_.session);
  EXPECT_EQ(estimator_->state().epoch.generation, 1U);
  ExpectPositiveCovariance(estimator_->state());
}

TEST(LocalEstimatorInitializationTest,
     ImuOnlyQuietWindowInitializesButDoesNotQualifyMotion) {
  LocalConfig config = TestConfig();
  config.allow_imu_only_initialization = true;
  LocalEstimator estimator(config);

  for (uint64_t sequence = 1; sequence <= config.stationary_samples;
       ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    const Result result = estimator.AddImu(MakeImu(time, sequence));
    if (sequence == config.stationary_samples) {
      EXPECT_EQ(result.reason, Reason::WHEEL_STALE);
    } else {
      EXPECT_TRUE(result.ok()) << result.message;
    }
    EXPECT_EQ(estimator.state().valid, sequence == config.stationary_samples);
  }

  ASSERT_TRUE(estimator.state().valid);
  EXPECT_TRUE(estimator.EvaluateIntegrity(1.02).ok());
  EXPECT_EQ(estimator.Evaluate(1.02).reason, Reason::WHEEL_STALE);
  EXPECT_EQ(estimator.wheel_health().accepted, 0U);
  ExpectPositiveCovariance(estimator.state());
}

TEST(LocalEstimatorInitializationTest,
     RequiresStationaryDwellAndRestartsAfterMotion) {
  LocalConfig config = TestConfig();
  config.stationary_duration = 0.04;
  LocalEstimator estimator(config);
  for (uint64_t sequence = 1; sequence <= 9; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    const double speed = sequence == 4 ? 0.1 : 0.0;
    ASSERT_TRUE(estimator.AddWheel(MakeWheel(time, sequence, speed)).ok());
    const Result result = estimator.AddImu(MakeImu(time, sequence));
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(estimator.state().valid, sequence == 9);
  }
  EXPECT_TRUE(estimator.state().valid);
}

TEST(LocalEstimatorInitializationTest,
     ZeroVelocityUpdateIsOptInAndPreservesPose) {
  LocalConfig enabled_config = TestConfig();
  enabled_config.enable_zero_velocity_update = true;
  LocalConfig disabled_config = enabled_config;
  disabled_config.enable_zero_velocity_update = false;
  LocalEstimator enabled(enabled_config);
  LocalEstimator disabled(disabled_config);
  for (uint64_t sequence = 1; sequence <= 8; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    const bool lateral_acceleration = sequence == 4 || sequence == 5;
    ImuSample imu = MakeImu(time, sequence);
    if (lateral_acceleration) {
      imu.acceleration.y() = 2.0;
    }
    const WheelSample wheel = MakeWheel(time, sequence);
    ASSERT_TRUE(enabled.AddWheel(wheel).ok());
    ASSERT_TRUE(disabled.AddWheel(wheel).ok());
    const Result enabled_result = enabled.AddImu(imu);
    const Result disabled_result = disabled.AddImu(imu);
    ASSERT_TRUE(enabled_result.ok()) << enabled_result.message;
    ASSERT_TRUE(disabled_result.ok()) << disabled_result.message;
  }

  EXPECT_EQ(enabled.zero_velocity_health().accepted, 1U);
  EXPECT_LT(enabled.state().velocity.norm(), disabled.state().velocity.norm());
  EXPECT_TRUE(
      enabled.state().position.isApprox(disabled.state().position, 1e-12));
  EXPECT_TRUE(enabled.state().orientation.coeffs().isApprox(
      disabled.state().orientation.coeffs(), 1e-12));
  ExpectPositiveCovariance(enabled.state());
}

TEST_F(LocalEstimatorTest,
       MotionContainsOnlyAppliedSamplesAndJointUncertainty) {
  Initialize();
  EXPECT_EQ(nullptr, estimator_->last_motion());
  const auto start = estimator_->state();
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.025, 4, 1000.0)).ok());
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.027, 5)).ok());
  EXPECT_EQ(Reason::INNOVATION_REJECTED,
            estimator_->AddImu(MakeImu(1.03, 4)).reason);
  ASSERT_TRUE(estimator_->state().valid);
  ASSERT_TRUE(estimator_->motion_status().ok())
      << estimator_->motion_status().message;
  const auto* motion = estimator_->last_motion();
  ASSERT_NE(nullptr, motion);
  EXPECT_EQ(start.stamp.time, motion->start.time);
  EXPECT_EQ(1.03, motion->end.time);
  EXPECT_TRUE(motion->start_covariance.isApprox(start.covariance));
  EXPECT_TRUE(motion->end_covariance.isApprox(estimator_->state().covariance));
  ASSERT_EQ(3U, motion->sources.size());
  EXPECT_EQ("imu", motion->sources[0].source);
  EXPECT_EQ(3U, motion->sources[0].sequence);
  EXPECT_EQ("imu", motion->sources[1].source);
  EXPECT_EQ(4U, motion->sources[1].sequence);
  EXPECT_EQ("wheel", motion->sources[2].source);
  EXPECT_EQ(5U, motion->sources[2].sequence);
  EXPECT_EQ(Reason::IMU_GAP, estimator_->AddImu(MakeImu(1.2, 5)).reason);
  EXPECT_EQ(nullptr, estimator_->last_motion());
}

TEST_F(LocalEstimatorTest,
       LocalLidarCorrectsInternalSubspaceWithoutSameTimeOdomJump) {
  EnableTestLidar(&config_);
  estimator_.reset(new LocalEstimator(config_));
  Initialize();
  EXPECT_EQ(estimator_->AddLidar(CorridorScan(estimator_->internal_state(), 1))
                .reason,
            Reason::GEOMETRY_UNAVAILABLE);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4)).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  const LocalState published_before = estimator_->state();
  ASSERT_TRUE(
      estimator_->AddLidar(CorridorScan(estimator_->internal_state(), 2, 0.1))
          .ok());
  EXPECT_FALSE(estimator_->internal_state().covariance_model_valid);
  EXPECT_FALSE(estimator_->state().covariance_model_valid);
  EXPECT_EQ(estimator_->Evaluate(estimator_->state().stamp.time).reason,
            Reason::CORRELATION_UNQUALIFIED);
  EXPECT_EQ(estimator_->last_motion(), nullptr);
  EXPECT_NE(estimator_->internal_state().position.y(),
            published_before.position.y());
  EXPECT_DOUBLE_EQ(estimator_->state().position.y(),
                   published_before.position.y());
  EXPECT_LT(estimator_->lidar_observed_pose_basis().rows(), 6);
  EXPECT_LT(estimator_->lidar_observed_pose_basis().col(0).norm(), 1e-10);

  std::vector<LocalEstimatorTrace> post_lidar_wheel_traces;
  estimator_->SetTraceCallback(
      [&post_lidar_wheel_traces](const LocalEstimatorTrace& trace) {
        post_lidar_wheel_traces.push_back(trace);
      });
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.04, 5)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.04, 5)).reason,
            Reason::CORRELATION_UNQUALIFIED);
  ASSERT_EQ(post_lidar_wheel_traces.size(), 1U);
  EXPECT_EQ(post_lidar_wheel_traces.front().kind,
            LocalEstimatorTraceKind::IMU_PROPAGATION);
  EXPECT_EQ(estimator_->wheel_health().rejected, 1U);
  EXPECT_EQ(estimator_->wheel_health().reason, Reason::CORRELATION_UNQUALIFIED);
  ASSERT_TRUE(estimator_->state().valid);
  EXPECT_FALSE(estimator_->state().covariance_model_valid);
  EXPECT_LE(std::abs(estimator_->state().position.y() -
                     published_before.position.y()),
            config_.lidar_max_translation_correction_rate * 0.01 + 1e-12);
  EXPECT_GE(estimator_->state().covariance(1, 1),
            published_before.covariance(1, 1));
  EXPECT_EQ(estimator_->last_motion(), nullptr);
  EXPECT_EQ(estimator_->motion_status().reason,
            Reason::CORRELATION_UNQUALIFIED);
  MotionIncrement unqualified;
  EXPECT_EQ(estimator_
                ->MotionBetween(published_before.stamp.time,
                                estimator_->state().stamp.time, &unqualified)
                .reason,
            Reason::CORRELATION_UNQUALIFIED);
}

TEST_F(LocalEstimatorTest, StaleLidarIsRejectedAndStoppedResetClearsGeometry) {
  EnableTestLidar(&config_);
  estimator_.reset(new LocalEstimator(config_));
  Initialize();
  LidarScan stale = CorridorScan(estimator_->internal_state(), 1);
  stale.stamp.receive_time =
      stale.stamp.time + config_.lidar_max_scan_age + 0.01;
  EXPECT_EQ(estimator_->AddLidar(stale).reason, Reason::LIDAR_STALE);
  EXPECT_EQ(estimator_->lidar_health().accepted, 0U);
  LidarScan delayed = CorridorScan(estimator_->internal_state(), 2);
  delayed.stamp.time -= 1e-9;
  for (auto& point : delayed.points) {
    point.time = delayed.stamp.time;
  }
  EXPECT_EQ(estimator_->AddLidar(delayed).reason, Reason::LIDAR_STALE);

  EXPECT_EQ(estimator_->AddLidar(CorridorScan(estimator_->internal_state(), 3))
                .reason,
            Reason::GEOMETRY_UNAVAILABLE);
  EXPECT_EQ(Reason::IMU_GAP, estimator_->AddImu(MakeImu(1.2, 4)).reason);
  for (uint64_t sequence = 5; sequence <= 7; ++sequence) {
    const double time = 1.21 + 0.01 * static_cast<double>(sequence - 5);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    EXPECT_EQ(estimator_->AddImu(MakeImu(time, sequence)).reason,
              Reason::IMU_GAP);
  }
  ASSERT_TRUE(estimator_->ResetStopped(1.24).ok());
  EXPECT_EQ(estimator_->state().epoch.generation, 2U);
  EXPECT_TRUE(estimator_->state().covariance_model_valid);
  for (uint64_t sequence = 8; sequence <= 10; ++sequence) {
    const double time = 1.25 + 0.01 * static_cast<double>(sequence - 8);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    ASSERT_TRUE(estimator_->AddImu(MakeImu(time, sequence)).ok());
  }
  ASSERT_TRUE(estimator_->state().valid);
  EXPECT_TRUE(estimator_->state().covariance_model_valid);
  EXPECT_EQ(estimator_->AddLidar(CorridorScan(estimator_->internal_state(), 4))
                .reason,
            Reason::GEOMETRY_UNAVAILABLE);
}

TEST_F(LocalEstimatorTest, BoundedHistoryRejectsInterpolationAndResetCrossing) {
  config_.max_history_states = 3;
  estimator_.reset(new LocalEstimator(config_));
  Initialize();
  for (uint64_t sequence = 4; sequence <= 7; ++sequence) {
    const double time = 1.02 + 0.01 * static_cast<double>(sequence - 3);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    ASSERT_TRUE(estimator_->AddImu(MakeImu(time, sequence)).ok());
  }
  LocalState state;
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            estimator_->Lookup(1.02, &state).reason);
  const double start_time = 1.02 + 0.01 * 2;
  const double end_time = 1.02 + 0.01 * 4;
  ASSERT_TRUE(estimator_->Lookup(start_time, &state).ok());
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            estimator_->Lookup(start_time + 0.005, &state).reason);
  MotionIncrement motion;
  ASSERT_TRUE(estimator_->MotionBetween(start_time, end_time, &motion).ok());
  EXPECT_EQ(5U, motion.start.sequence);
  EXPECT_EQ(7U, motion.end.sequence);
  EXPECT_EQ(5U,
            motion.sources.size());  // Three IMU samples, two wheel samples.
  const auto* last = estimator_->last_motion();
  ASSERT_NE(nullptr, last);
  MotionIncrement one_step;
  ASSERT_TRUE(
      estimator_->MotionBetween(last->start.time, last->end.time, &one_step)
          .ok());
  EXPECT_TRUE(one_step.cross_covariance.isApprox(last->cross_covariance));
  EXPECT_TRUE(one_step.covariance.isApprox(last->covariance));
  // Gather fresh stopped evidence after an explicit integrity fault.
  EXPECT_EQ(Reason::IMU_GAP, estimator_->AddImu(MakeImu(1.2, 8)).reason);
  for (uint64_t sequence = 9; sequence <= 11; ++sequence) {
    const double time = 1.21 + 0.01 * static_cast<double>(sequence - 9);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    EXPECT_EQ(Reason::IMU_GAP,
              estimator_->AddImu(MakeImu(time, sequence)).reason);
  }
  ASSERT_TRUE(estimator_->ResetStopped(1.24).ok());
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            estimator_->MotionBetween(start_time, end_time, &motion).reason);
}

TEST_F(LocalEstimatorTest, TiltAlignmentUsesConfiguredGravityAndGyroAverage) {
  const Eigen::Quaterniond tilt =
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitX());
  const Eigen::Vector3d body_gravity =
      tilt.conjugate() * Eigen::Vector3d(0.0, 0.0, config_.gravity);
  const Eigen::Vector3d bias(0.01, -0.01, 0.005);
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    ImuSample imu = MakeImu(time, sequence);
    imu.acceleration = body_gravity;
    imu.angular_velocity = bias;
    const Result result = estimator_->AddImu(imu);
    EXPECT_TRUE(result.ok());
    EXPECT_EQ(estimator_->Evaluate(time).reason,
              sequence < 3 ? Reason::WAITING_FOR_STATIONARY : Reason::NONE);
  }
  EXPECT_LT(
      LogRotation(tilt.conjugate() * estimator_->state().orientation).norm(),
      1e-12);
  EXPECT_LT((estimator_->state().gyro_bias - bias).norm(), 1e-12);
  EXPECT_EQ(estimator_->state().accel_bias.norm(), 0.0);
  // No GNSS is involved: translation and yaw are gauge choices, not observed.
  EXPECT_EQ(estimator_->state().position.norm(), 0.0);
  EXPECT_NEAR(estimator_->state().orientation.toRotationMatrix()(1, 0), 0.0,
              1e-12);
  for (uint64_t sequence = 4; sequence <= 13; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    ImuSample imu = MakeImu(time, sequence);
    imu.acceleration = body_gravity;
    imu.angular_velocity = bias;
    ASSERT_TRUE(estimator_->AddImu(imu).ok());
  }
  EXPECT_LT(estimator_->state().velocity.norm(), 1e-12);
  EXPECT_LT(estimator_->state().position.norm(), 1e-12);
  ExpectPositiveCovariance(estimator_->state());
}

TEST(LocalEstimatorDiagnosticsTest,
     TraceReportsPropagationAndWheelCorrections) {
  LocalConfig config = TestConfig();
  LocalEstimator traced(config);
  LocalEstimator untraced(config);
  std::vector<LocalEstimatorTrace> trace;
  traced.SetTraceCallback(
      [&trace](const LocalEstimatorTrace& event) { trace.push_back(event); });
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(traced.AddWheel(MakeWheel(time, sequence)).ok());
    ASSERT_TRUE(untraced.AddWheel(MakeWheel(time, sequence)).ok());
    ASSERT_TRUE(traced.AddImu(MakeImu(time, sequence)).ok());
    ASSERT_TRUE(untraced.AddImu(MakeImu(time, sequence)).ok());
  }
  ASSERT_TRUE(traced.state().valid);
  ASSERT_TRUE(untraced.state().valid);

  ASSERT_TRUE(traced.AddWheel(MakeWheel(1.03, 4, 0.2)).ok());
  ASSERT_TRUE(untraced.AddWheel(MakeWheel(1.03, 4, 0.2)).ok());
  ImuSample moving_imu = MakeImu(1.04, 5);
  moving_imu.acceleration.y() = 1.0;
  ASSERT_TRUE(traced.AddImu(moving_imu).ok());
  ASSERT_TRUE(untraced.AddImu(moving_imu).ok());
  ExpectSameState(untraced.state(), traced.state());

  ASSERT_EQ(trace.size(), 3U);
  EXPECT_EQ(trace[0].kind, LocalEstimatorTraceKind::IMU_PROPAGATION);
  EXPECT_EQ(trace[0].stamp.time, 1.03);
  EXPECT_NEAR(trace[0].dt, 0.01, 1e-12);
  EXPECT_LT((trace[0].velocity_correction -
             trace[0].linear_acceleration_local * trace[0].dt)
                .norm(),
            1e-12);
  EXPECT_EQ(trace[1].kind, LocalEstimatorTraceKind::WHEEL_UPDATE);
  EXPECT_EQ(trace[1].stamp.time, 1.03);
  EXPECT_DOUBLE_EQ(trace[1].wheel_speed, 0.2);
  EXPECT_NEAR(trace[1].wheel_innovation.x(), 0.2, 1e-12);
  EXPECT_NEAR(trace[1].wheel_innovation_distance,
              trace[1].wheel_innovation.x() * trace[1].wheel_innovation.x() /
                  trace[1].wheel_innovation_covariance(0, 0),
              1e-12);
  EXPECT_GT(trace[1].velocity_correction.norm(), 0.0);
  EXPECT_NEAR(trace[1].velocity_correction.y(),
              trace[1].wheel_gain(4, 0) * trace[1].wheel_innovation.x(), 1e-12);
  const auto& observation = trace[1].wheel_observation;
  EXPECT_NEAR(trace[1].wheel_lateral_correction_by_state_block.sum(),
              trace[1].velocity_correction.y(), 1e-12);
  EXPECT_NEAR(
      trace[1].wheel_lateral_cross_covariance_by_state_block.row(0).sum(),
      trace[1].velocity_y_covariance_row_before.dot(
          observation.row(0).transpose()),
      1e-12);
  EXPECT_EQ(trace[1].attitude_error_correction.norm(), 0.0);
  EXPECT_EQ((trace[1].gyro_bias_after - trace[1].gyro_bias_before -
             trace[1].gyro_bias_correction)
                .norm(),
            0.0);
  for (int state = 0; state < 15; ++state) {
    const double expected_covariance =
        trace[1].velocity_y_covariance_row_before(state) -
        trace[1].wheel_gain(4, 0) * trace[1].wheel_innovation_covariance(0, 0) *
            trace[1].unconstrained_wheel_gain(state, 0);
    EXPECT_NEAR(trace[1].velocity_y_covariance_row_after(state),
                expected_covariance, 1e-10)
        << "state=" << state;
  }

  EXPECT_DOUBLE_EQ(observation(0, 3), 1.0);
  EXPECT_NEAR(observation(0, 4), 0.0, 1e-12);
  EXPECT_NEAR(observation(0, 5), 0.0, 1e-12);
  EXPECT_NEAR(observation(0, 8), trace[1].wheel_body_velocity_before.y(),
              1e-12);
  for (int state = 0; state < 15; ++state) {
    if (state != 3 && state != 8) {
      EXPECT_NEAR(observation(0, state), 0.0, 1e-12) << "state=" << state;
    }
  }
  const double epsilon = 1e-7;
  const Eigen::Vector3d local_velocity = trace[1].velocity_before;
  const Eigen::Matrix3d local_to_body =
      trace[1].orientation_before.toRotationMatrix().transpose();
  const double nominal_forward_speed = (local_to_body * local_velocity).x();
  for (int axis = 0; axis < 3; ++axis) {
    const double perturbed_velocity_speed =
        (local_to_body *
         (local_velocity + epsilon * Eigen::Vector3d::Unit(axis)))
            .x();
    EXPECT_NEAR((perturbed_velocity_speed - nominal_forward_speed) / epsilon,
                observation(0, 3 + axis), 1e-8)
        << "velocity_axis=" << axis;
    const Eigen::Quaterniond perturbed_orientation =
        trace[1].orientation_before *
        Eigen::Quaterniond(
            Eigen::AngleAxisd(epsilon, Eigen::Vector3d::Unit(axis)));
    const double perturbed_forward_speed =
        (perturbed_orientation.toRotationMatrix().transpose() * local_velocity)
            .x();
    EXPECT_NEAR((perturbed_forward_speed - nominal_forward_speed) / epsilon,
                observation(0, 6 + axis), 1e-7)
        << "attitude_axis=" << axis;
  }
  EXPECT_EQ(trace[2].kind, LocalEstimatorTraceKind::IMU_PROPAGATION);
  EXPECT_EQ(trace[2].stamp.time, 1.04);
  EXPECT_NEAR(trace[2].dt, 0.01, 1e-12);
}

TEST(LocalEstimatorDiagnosticsTest, BiasFeedbackMasksUseActualJosephGain) {
  for (bool gyro : {false, true}) {
    for (bool accel : {false, true}) {
      LocalEstimator estimator(TestConfig());
      LocalEstimatorTestAccess::SetWheelBiasFeedback(&estimator, gyro, accel);
      LocalState state;
      state.velocity = Eigen::Vector3d(1.0, 0.2, 0.0);
      Matrix15d root = Matrix15d::Identity();
      root(9, 3) = 0.2;
      root(12, 3) = 0.3;
      state.covariance = root * root.transpose();
      const LocalState before = state;
      LocalEstimatorTrace trace;
      estimator.SetTraceCallback(
          [&trace](const LocalEstimatorTrace& value) { trace = value; });
      Matrix15d transition = Matrix15d::Identity();
      const WheelSample wheel = MakeWheel(1.0, 1, 1.1);
      ASSERT_TRUE(LocalEstimatorTestAccess::UpdateWheel(
                      &estimator, wheel, MakeImu(1.0, 1), &state, &transition)
                      .ok());
      EXPECT_EQ(state.gyro_bias.norm() > 0.0, gyro);
      EXPECT_EQ(state.accel_bias.norm() > 0.0, accel);
      const Matrix15d residual =
          Matrix15d::Identity() - trace.wheel_gain * trace.wheel_observation;
      const Matrix15d expected =
          residual * before.covariance * residual.transpose() +
          wheel.variance * trace.wheel_gain.col(0) *
              trace.wheel_gain.col(0).transpose();
      EXPECT_TRUE(state.covariance.isApprox(expected, 1e-12));
      EXPECT_TRUE(transition.isApprox(residual, 1e-12));
      ExpectPositiveCovariance(state);
    }
  }
}

TEST(LocalEstimatorDiagnosticsTest,
     ForwardOnlyGainPreservesBodyTransverseVelocity) {
  LocalEstimator estimator(TestConfig());
  LocalEstimatorTestAccess::SetWheelBiasFeedback(&estimator, false, false,
                                                 true);
  LocalState state;
  state.orientation = ExpRotation(Eigen::Vector3d(0.1, -0.2, 0.5));
  state.velocity = Eigen::Vector3d(1.0, 0.2, 0.1);
  state.covariance = Matrix15d::Identity();
  const LocalState before = state;
  LocalEstimatorTrace trace;
  estimator.SetTraceCallback(
      [&trace](const LocalEstimatorTrace& value) { trace = value; });
  Matrix15d transition = Matrix15d::Identity();
  const WheelSample wheel = MakeWheel(1.0, 1, 1.5);
  ASSERT_TRUE(LocalEstimatorTestAccess::UpdateWheel(
                  &estimator, wheel, MakeImu(1.0, 1), &state, &transition)
                  .ok());
  const Eigen::Vector3d body_correction =
      state.orientation.conjugate() * (state.velocity - before.velocity);
  EXPECT_GT(std::abs(body_correction.x()), 0.01);
  EXPECT_LT(body_correction.tail<2>().norm(), 1e-12);
  const Matrix15d residual =
      Matrix15d::Identity() - trace.wheel_gain * trace.wheel_observation;
  const Matrix15d expected =
      residual * before.covariance * residual.transpose() +
      wheel.variance * trace.wheel_gain.col(0) *
          trace.wheel_gain.col(0).transpose();
  EXPECT_TRUE(state.covariance.isApprox(expected, 1e-12));
  ExpectPositiveCovariance(state);
}

Eigen::Matrix3d RightInjectionReset(const Eigen::Vector3d& angle) {
  const double norm = angle.norm();
  const Eigen::Matrix3d skew = Skew(angle);
  if (norm < 1e-5) {
    return Eigen::Matrix3d::Identity() - 0.5 * skew + (1.0 / 6.0) * skew * skew;
  }
  return Eigen::Matrix3d::Identity() -
         ((1.0 - std::cos(norm)) / (norm * norm)) * skew +
         ((norm - std::sin(norm)) / (norm * norm * norm)) * skew * skew;
}

TEST(LocalEstimatorFeedbackDiagnosticTest,
     RightInjectionResetMatchesFiniteDifference) {
  for (const Eigen::Vector3d& angle :
       {Eigen::Vector3d::Zero().eval(), Eigen::Vector3d(1e-7, -2e-7, 3e-7),
        Eigen::Vector3d(0.3, -0.2, 0.1)}) {
    Eigen::Matrix3d numerical;
    constexpr double epsilon = 1e-7;
    for (int axis = 0; axis < 3; ++axis) {
      const Eigen::Vector3d delta = epsilon * Eigen::Vector3d::Unit(axis);
      numerical.col(axis) = (LogRotation(ExpRotation(angle).conjugate() *
                                         ExpRotation(angle + delta)) -
                             LogRotation(ExpRotation(angle).conjugate() *
                                         ExpRotation(angle - delta))) /
                            (2.0 * epsilon);
    }
    EXPECT_LT((RightInjectionReset(angle) - numerical).cwiseAbs().maxCoeff(),
              1e-8);
  }
}

TEST(LocalEstimatorFeedbackDiagnosticTest,
     SixtySecondKnownTruthComparesConstrainedFullAndDisabledWheelFeedback) {
  constexpr double dt = 0.01;
  constexpr int intervals = 6000;
  const char* profiles[] = {"perfect",        "combined",
                            "initial_only",   "periodic_wheel_only",
                            "iid_wheel_only", "imu_only"};
  for (int profile = 0; profile < 6; ++profile) {
    for (double yaw_rate : {0.0, 0.02}) {
      if (profile >= 2 && yaw_rate != 0.0) {
        continue;
      }
      const double initial_error = profile == 1 || profile == 2 ? 1.0 : 0.0;
      const double imu_error = profile == 1 || profile == 5 ? 1.0 : 0.0;
      const double wheel_error = profile == 1 || profile == 3 ? 1.0 : 0.0;
      std::mt19937 generator(20261014);
      std::normal_distribution<double> wheel_noise(0.0, 0.04 / std::sqrt(2.0));
      LocalConfig config;
      config.gravity = 9.80665;
      config.use_nonholonomic_constraint = false;
      LocalState truth;
      truth.orientation = Eigen::Quaterniond::Identity();
      truth.velocity = Eigen::Vector3d(1.6, 0.0, 0.0);
      LocalState initial = truth;
      initial.orientation =
          ExpRotation(initial_error * Eigen::Vector3d(0.002, -0.001, 0.0));
      initial.gyro_bias = initial_error * Eigen::Vector3d(0.0001, -0.0001, 0.0);
      initial.covariance.diagonal().segment<3>(0).setConstant(1e-12);
      initial.covariance.diagonal().segment<3>(3).setConstant(
          config.initial_velocity_std * config.initial_velocity_std);
      initial.covariance.diagonal().segment<3>(6).setConstant(
          config.initial_attitude_std * config.initial_attitude_std);
      initial.covariance.diagonal().segment<3>(9).setConstant(
          config.initial_gyro_bias_std * config.initial_gyro_bias_std);
      initial.covariance.diagonal().segment<3>(12).setConstant(
          config.initial_accel_bias_std * config.initial_accel_bias_std);
      std::array<LocalState, 3> states{initial, initial, initial};
      std::array<LocalEstimator, 3> estimators{LocalEstimator(config),
                                               LocalEstimator(config),
                                               LocalEstimator(config)};
      std::array<LocalEstimatorTrace, 2> traces;
      for (int strategy = 0; strategy < 2; ++strategy) {
        estimators[strategy].SetTraceCallback(
            [&, strategy](const LocalEstimatorTrace& trace) {
              traces[strategy] = trace;
            });
      }
      std::array<double, 3> maximum_position_error{};
      std::array<int, 3> rejected{};
      std::array<double, 2> nis_sum{};
      double maximum_attitude_injection = 0.0;
      const auto make_input = [&](double time, uint64_t sequence) {
        ImuSample imu = MakeImu(time, sequence);
        imu.acceleration = Eigen::Vector3d(0.0, 1.6 * yaw_rate, config.gravity);
        imu.angular_velocity = Eigen::Vector3d(0.0, 0.0, yaw_rate);
        // Deterministic correlated excitation, not calibrated white sensor
        // noise.
        imu.acceleration +=
            imu_error * Eigen::Vector3d(0.01 * std::sin(2.3 * time),
                                        0.01 * std::cos(1.7 * time),
                                        0.01 * std::sin(0.8 * time));
        return imu;
      };
      ImuSample previous = make_input(1.0, 1);
      for (int interval = 0; interval < intervals; ++interval) {
        const double time = 1.0 + (interval + 1) * dt;
        const ImuSample imu = make_input(time, interval + 2);
        const double elapsed = (interval + 1) * dt;
        const double yaw = yaw_rate * elapsed;
        truth.orientation = ExpRotation(Eigen::Vector3d(0.0, 0.0, yaw));
        truth.velocity =
            Eigen::Vector3d(1.6 * std::cos(yaw), 1.6 * std::sin(yaw), 0.0);
        truth.position =
            yaw_rate == 0.0
                ? Eigen::Vector3d(1.6 * elapsed, 0.0, 0.0)
                : Eigen::Vector3d(1.6 * std::sin(yaw) / yaw_rate,
                                  1.6 * (1.0 - std::cos(yaw)) / yaw_rate, 0.0);
        WheelSample wheel = MakeWheel(time, interval + 2, 1.6);
        wheel.speed += profile == 4 ? wheel_noise(generator)
                                    : wheel_error * 0.04 * std::sin(1.3 * time);
        wheel.variance = 0.01;
        for (int strategy = 0; strategy < 3; ++strategy) {
          Matrix15d transition = Matrix15d::Identity();
          ASSERT_TRUE(LocalEstimatorTestAccess::Propagate(
                          estimators[strategy], previous, imu,
                          &states[strategy], &transition)
                          .ok());
          if (strategy < 2) {
            const LocalState before = states[strategy];
            const Result update = LocalEstimatorTestAccess::UpdateWheel(
                &estimators[strategy], wheel, imu, &states[strategy],
                &transition);
            if (!update.ok()) {
              ASSERT_EQ(update.reason, Reason::INNOVATION_REJECTED)
                  << update.message;
              ++rejected[strategy];
            } else {
              ASSERT_EQ(traces[strategy].kind,
                        LocalEstimatorTraceKind::WHEEL_UPDATE);
              nis_sum[strategy] += traces[strategy].wheel_innovation_distance;
            }
            if (update.ok() && strategy == 0) {
              EXPECT_TRUE(states[strategy].position.isApprox(before.position));
              EXPECT_TRUE(
                  states[strategy].orientation.isApprox(before.orientation));
            } else if (update.ok()) {
              // Use the production observation and raw gain; change only
              // feedback.
              const Eigen::Matrix<double, 15, 1> gain =
                  traces[strategy].unconstrained_wheel_gain.col(0);
              const Eigen::Matrix<double, 15, 1> correction =
                  gain * traces[strategy].wheel_innovation.x();
              const Eigen::Vector3d angle = correction.segment<3>(6);
              maximum_attitude_injection =
                  std::max(maximum_attitude_injection, angle.norm());
              LocalState full = before;
              full.position += correction.segment<3>(0);
              full.velocity += correction.segment<3>(3);
              full.orientation =
                  (full.orientation * ExpRotation(angle)).normalized();
              full.gyro_bias += correction.segment<3>(9);
              full.accel_bias += correction.segment<3>(12);
              full.angular_velocity = imu.angular_velocity - full.gyro_bias;
              const Matrix15d residual =
                  Matrix15d::Identity() -
                  gain * traces[strategy].wheel_observation.row(0);
              const Matrix15d posterior =
                  residual * before.covariance * residual.transpose() +
                  wheel.variance * gain * gain.transpose();
              Matrix15d reset = Matrix15d::Identity();
              // Right Jacobian: reset the right-multiplicative attitude error.
              reset.block<3, 3>(6, 6) = RightInjectionReset(angle);
              full.covariance = reset * posterior * reset.transpose();
              full.covariance =
                  (0.5 * (full.covariance + full.covariance.transpose()))
                      .eval();
              ExpectPositiveCovariance(full);
              states[strategy] = full;
            }
          }
          maximum_position_error[strategy] =
              std::max(maximum_position_error[strategy],
                       (states[strategy].position - truth.position).norm());
        }
        previous = imu;
      }
      const char* names[] = {"constrained", "full_eskf", "wheel_disabled"};
      for (int strategy = 0; strategy < 3; ++strategy) {
        std::cout << "FEEDBACK_DIAGNOSTIC yaw_rate=" << yaw_rate
                  << " profile=" << profiles[profile]
                  << " strategy=" << names[strategy] << " max_position_error_m="
                  << maximum_position_error[strategy]
                  << " final_velocity_error_mps="
                  << (states[strategy].velocity - truth.velocity).norm()
                  << " final_attitude_error_rad="
                  << LogRotation(truth.orientation.conjugate() *
                                 states[strategy].orientation)
                         .norm()
                  << " final_gyro_bias_norm_radps="
                  << states[strategy].gyro_bias.norm()
                  << " final_position_y_m=" << states[strategy].position.y()
                  << " final_velocity_y_mps=" << states[strategy].velocity.y()
                  << " final_velocity_y_std_mps="
                  << std::sqrt(states[strategy].covariance(4, 4))
                  << " accepted_nis_mean="
                  << (strategy < 2
                          ? nis_sum[strategy] / (intervals - rejected[strategy])
                          : 0.0)
                  << " max_attitude_injection_rad="
                  << (strategy == 1 ? maximum_attitude_injection : 0.0)
                  << " rejected=" << rejected[strategy] << '\n';
        ExpectPositiveCovariance(states[strategy]);
        if (profile == 0) {
          EXPECT_LT(maximum_position_error[strategy], 1e-4);
          EXPECT_LT((states[strategy].velocity - truth.velocity).norm(), 1e-5);
        }
      }
    }
  }
}

TEST(LocalEstimatorPropagationTest,
     ErrorTransitionMatchesFiniteDifferenceOfMechanization) {
  const LocalConfig config = TestConfig();
  const LocalEstimator estimator(config);
  LocalState nominal;
  nominal.position = Eigen::Vector3d(0.4, -0.2, 0.1);
  nominal.velocity = Eigen::Vector3d(2.0, -0.4, 0.3);
  nominal.orientation = ExpRotation(Eigen::Vector3d(0.1, -0.2, 0.3));
  nominal.gyro_bias = Eigen::Vector3d(0.003, -0.002, 0.001);
  nominal.accel_bias = Eigen::Vector3d(0.1, -0.05, 0.02);
  nominal.covariance.setIdentity();
  ImuSample start = MakeImu(1.0, 1);
  ImuSample end = MakeImu(1.02, 2);
  start.acceleration = Eigen::Vector3d(0.2, -0.4, 9.5);
  end.acceleration = Eigen::Vector3d(0.3, -0.2, 9.6);
  start.angular_velocity = Eigen::Vector3d(0.1, -0.2, 0.3);
  end.angular_velocity = Eigen::Vector3d(0.11, -0.18, 0.32);

  LocalState nominal_end = nominal;
  Matrix15d transition = Matrix15d::Identity();
  ASSERT_TRUE(LocalEstimatorTestAccess::Propagate(estimator, start, end,
                                                  &nominal_end, &transition)
                  .ok());

  const auto perturb = [](LocalState* state, int index, double amount) {
    if (index < 3) {
      state->position(index) += amount;
    } else if (index < 6) {
      state->velocity(index - 3) += amount;
    } else if (index < 9) {
      state->orientation =
          (state->orientation *
           ExpRotation(amount * Eigen::Vector3d::Unit(index - 6)))
              .normalized();
    } else if (index < 12) {
      state->gyro_bias(index - 9) += amount;
    } else {
      state->accel_bias(index - 12) += amount;
    }
  };
  const auto error = [](const LocalState& reference, const LocalState& value) {
    Eigen::Matrix<double, 15, 1> delta;
    delta.segment<3>(0) = value.position - reference.position;
    delta.segment<3>(3) = value.velocity - reference.velocity;
    delta.segment<3>(6) =
        LogRotation(reference.orientation.conjugate() * value.orientation);
    delta.segment<3>(9) = value.gyro_bias - reference.gyro_bias;
    delta.segment<3>(12) = value.accel_bias - reference.accel_bias;
    return delta;
  };
  constexpr double epsilon = 1e-6;
  for (int column = 0; column < 15; ++column) {
    LocalState positive = nominal;
    LocalState negative = nominal;
    perturb(&positive, column, epsilon);
    perturb(&negative, column, -epsilon);
    Matrix15d ignored = Matrix15d::Identity();
    ASSERT_TRUE(LocalEstimatorTestAccess::Propagate(estimator, start, end,
                                                    &positive, &ignored)
                    .ok());
    ignored.setIdentity();
    ASSERT_TRUE(LocalEstimatorTestAccess::Propagate(estimator, start, end,
                                                    &negative, &ignored)
                    .ok());
    const Eigen::Matrix<double, 15, 1> numerical =
        (error(nominal_end, positive) - error(nominal_end, negative)) /
        (2.0 * epsilon);
    EXPECT_LT((numerical - transition.col(column)).cwiseAbs().maxCoeff(), 2e-5)
        << "error_state_column=" << column;
  }
}

TEST(LocalEstimatorPropagationTest,
     CovarianceTraceSeparatesPredictionAndDiscretizedNoise) {
  LocalConfig config = TestConfig();
  LocalEstimator estimator(config);
  LocalState state;
  state.covariance.setIdentity();
  state.covariance(3, 9) = state.covariance(9, 3) = 0.1;
  ImuSample start = MakeImu(1.0, 1);
  ImuSample end = MakeImu(1.02, 2);
  std::vector<LocalEstimatorTrace> trace;
  estimator.SetTraceCallback(
      [&trace](const LocalEstimatorTrace& event) { trace.push_back(event); });
  const Matrix15d covariance_before = state.covariance;
  Matrix15d transition = Matrix15d::Identity();
  ASSERT_TRUE(LocalEstimatorTestAccess::Propagate(estimator, start, end, &state,
                                                  &transition)
                  .ok());
  ASSERT_EQ(trace.size(), 1U);
  EXPECT_TRUE(trace.front().error_transition.isApprox(transition, 1e-14));
  const Matrix15d expected_prediction =
      transition * covariance_before * transition.transpose();
  EXPECT_TRUE(state.covariance.isApprox(
      expected_prediction + trace.front().process_noise_covariance, 1e-12));
  EXPECT_TRUE(
      (trace.front()
           .process_noise_velocity_y_by_source.colwise()
           .sum()
           .transpose())
          .isApprox(trace.front().process_noise_covariance.row(4).transpose(),
                    1e-12));
  EXPECT_TRUE(trace.front().propagated_covariance_y_row.isApprox(
      expected_prediction.row(4).transpose(), 1e-12));
  EXPECT_TRUE(trace.front().velocity_y_covariance_row_after.isApprox(
      state.covariance.row(4).transpose(), 1e-12));

  const Matrix15d& process_noise = trace.front().process_noise_covariance;
  EXPECT_TRUE(process_noise.isApprox(process_noise.transpose(), 1e-14));
  const auto discrete_noise = [](const LocalConfig& noise_config, double dt) {
    LocalEstimator isolated_estimator(noise_config);
    LocalState isolated_state;
    isolated_state.covariance.setIdentity();
    ImuSample isolated_start = MakeImu(1.0, 1);
    ImuSample isolated_end = MakeImu(1.0 + dt, 2);
    std::vector<LocalEstimatorTrace> isolated_trace;
    isolated_estimator.SetTraceCallback(
        [&isolated_trace](const LocalEstimatorTrace& event) {
          isolated_trace.push_back(event);
        });
    Matrix15d ignored_transition = Matrix15d::Identity();
    EXPECT_TRUE(LocalEstimatorTestAccess::Propagate(
                    isolated_estimator, isolated_start, isolated_end,
                    &isolated_state, &ignored_transition)
                    .ok());
    EXPECT_EQ(isolated_trace.size(), 1U);
    return isolated_trace.front().process_noise_covariance;
  };
  const std::array<std::pair<double, int>, 4> noise_checks{{
      {config.accel_noise, 3},
      {config.gyro_noise, 6},
      {config.gyro_bias_noise, 9},
      {config.accel_bias_noise, 12},
  }};
  for (std::size_t source = 0; source < noise_checks.size(); ++source) {
    LocalConfig isolated = TestConfig();
    isolated.accel_noise = 0.0;
    isolated.gyro_noise = 0.0;
    isolated.gyro_bias_noise = 0.0;
    isolated.accel_bias_noise = 0.0;
    if (source == 0) {
      isolated.accel_noise = noise_checks[source].first;
    } else if (source == 1) {
      isolated.gyro_noise = noise_checks[source].first;
    } else if (source == 2) {
      isolated.gyro_bias_noise = noise_checks[source].first;
    } else {
      isolated.accel_bias_noise = noise_checks[source].first;
    }
    const Matrix15d full_interval = discrete_noise(isolated, 0.02);
    const Matrix15d half_interval = discrete_noise(isolated, 0.01);
    const int state_index = noise_checks[source].second;
    const double expected =
        noise_checks[source].first * noise_checks[source].first * 0.02;
    EXPECT_NEAR(full_interval(state_index, state_index), expected, 1e-14);
    EXPECT_NEAR(full_interval(state_index, state_index),
                2.0 * half_interval(state_index, state_index), 1e-14);
  }
}

TEST(LocalEstimatorPropagationTest,
     ContinuousNoiseMonteCarloMatchesPredictedErrorCovariance) {
  constexpr int kTrials = 2048;
  constexpr int kIntervals = 100;
  constexpr double kDt = 0.01;
  constexpr double kInitialTime = 1.0;
  constexpr double kDuration = kIntervals * kDt;
  constexpr double kFamilywiseNormalBound = 4.0;
  constexpr double kDiscretizationAllowance = 0.02;

  LocalConfig config = TestConfig();
  config.gravity = 9.80665;
  config.max_imu_gap = 0.02;
  config.accel_noise = 0.02;
  config.gyro_noise = 0.003;
  config.gyro_bias_noise = 0.0005;
  config.accel_bias_noise = 0.001;
  const LocalEstimator estimator(config);

  LocalState nominal;
  nominal.position = Eigen::Vector3d(2.0, -1.0, 0.3);
  nominal.velocity = Eigen::Vector3d(0.7, -0.2, 0.1);
  nominal.orientation = ExpRotation(Eigen::Vector3d(0.03, -0.02, 0.01));
  Matrix15d initial_covariance = Matrix15d::Zero();
  initial_covariance.diagonal().segment<3>(0).setConstant(0.01 * 0.01);
  initial_covariance.diagonal().segment<3>(3).setConstant(0.02 * 0.02);
  initial_covariance.diagonal().segment<3>(6).setConstant(0.002 * 0.002);
  initial_covariance.diagonal().segment<3>(9).setConstant(0.001 * 0.001);
  initial_covariance.diagonal().segment<3>(12).setConstant(0.005 * 0.005);

  const Eigen::Vector3d ideal_specific_force(0.15, -0.08, config.gravity);
  const Eigen::Vector3d ideal_angular_rate = Eigen::Vector3d::Zero();
  const Eigen::Vector3d gravity_local(0.0, 0.0, config.gravity);
  std::mt19937_64 random(20261014);
  std::normal_distribution<double> standard_normal(0.0, 1.0);
  Matrix15d predicted_covariance_sum = Matrix15d::Zero();
  std::vector<Eigen::Matrix<double, 15, 1>> state_errors;
  state_errors.reserve(kTrials);

  const auto noise = [&random, &standard_normal](
                         double standard_deviation) -> Eigen::Vector3d {
    Eigen::Vector3d sample;
    for (int axis = 0; axis < 3; ++axis) {
      sample(axis) = standard_normal(random);
    }
    return (standard_deviation * sample).eval();
  };
  for (int trial = 0; trial < kTrials; ++trial) {
    Eigen::Matrix<double, 15, 1> initial_error;
    for (int index = 0; index < 15; ++index) {
      initial_error(index) =
          std::sqrt(initial_covariance(index, index)) * standard_normal(random);
    }

    LocalState estimate = nominal;
    estimate.covariance = initial_covariance;
    LocalState truth;
    truth.position = nominal.position + initial_error.segment<3>(0);
    truth.velocity = nominal.velocity + initial_error.segment<3>(3);
    truth.orientation =
        (nominal.orientation * ExpRotation(initial_error.segment<3>(6)))
            .normalized();
    const Eigen::Vector3d initial_gyro_bias = initial_error.segment<3>(9);
    const Eigen::Vector3d initial_accel_bias = initial_error.segment<3>(12);
    Eigen::Vector3d true_gyro_bias = initial_gyro_bias;
    Eigen::Vector3d true_accel_bias = initial_accel_bias;

    ImuSample start = MakeImu(kInitialTime, 1);
    start.acceleration = ideal_specific_force + true_accel_bias +
                         noise(config.accel_noise / std::sqrt(kDt));
    start.angular_velocity = ideal_angular_rate + true_gyro_bias +
                             noise(config.gyro_noise / std::sqrt(kDt));
    for (int interval = 0; interval < kIntervals; ++interval) {
      true_gyro_bias += noise(config.gyro_bias_noise * std::sqrt(kDt));
      true_accel_bias += noise(config.accel_bias_noise * std::sqrt(kDt));
      ImuSample end = MakeImu(kInitialTime + (interval + 1) * kDt,
                              static_cast<uint64_t>(interval + 2));
      end.acceleration = ideal_specific_force + true_accel_bias +
                         noise(config.accel_noise / std::sqrt(kDt));
      end.angular_velocity = ideal_angular_rate + true_gyro_bias +
                             noise(config.gyro_noise / std::sqrt(kDt));

      Matrix15d transition = Matrix15d::Identity();
      ASSERT_TRUE(LocalEstimatorTestAccess::Propagate(estimator, start, end,
                                                      &estimate, &transition)
                      .ok());
      start = end;
    }

    const Eigen::Vector3d truth_acceleration =
        truth.orientation * ideal_specific_force - gravity_local;
    truth.position += truth.velocity * kDuration +
                      0.5 * truth_acceleration * kDuration * kDuration;
    truth.velocity += truth_acceleration * kDuration;
    const Eigen::Vector3d final_gyro_bias = true_gyro_bias;
    const Eigen::Vector3d final_accel_bias = true_accel_bias;

    Eigen::Matrix<double, 15, 1> error;
    error.segment<3>(0) = estimate.position - truth.position;
    error.segment<3>(3) = estimate.velocity - truth.velocity;
    error.segment<3>(6) =
        LogRotation(truth.orientation.conjugate() * estimate.orientation);
    error.segment<3>(9) = estimate.gyro_bias - final_gyro_bias;
    error.segment<3>(12) = estimate.accel_bias - final_accel_bias;
    state_errors.push_back(error);
    predicted_covariance_sum += estimate.covariance;
  }

  const Matrix15d predicted_covariance =
      predicted_covariance_sum / static_cast<double>(kTrials);
  Eigen::Matrix<double, 15, 1> empirical_mean =
      Eigen::Matrix<double, 15, 1>::Zero();
  for (const auto& error : state_errors) {
    empirical_mean += error;
  }
  empirical_mean /= static_cast<double>(kTrials);
  Matrix15d empirical_covariance = Matrix15d::Zero();
  for (const auto& error : state_errors) {
    const Eigen::Matrix<double, 15, 1> centered = error - empirical_mean;
    empirical_covariance.noalias() += centered * centered.transpose();
  }
  empirical_covariance /= static_cast<double>(kTrials - 1);

  for (int row = 0; row < 15; ++row) {
    for (int column = 0; column < 15; ++column) {
      const double scale = std::sqrt(predicted_covariance(row, row) *
                                     predicted_covariance(column, column));
      const double sampling_error =
          kFamilywiseNormalBound *
          std::sqrt((predicted_covariance(row, row) *
                         predicted_covariance(column, column) +
                     predicted_covariance(row, column) *
                         predicted_covariance(row, column)) /
                    static_cast<double>(kTrials - 1));
      const double allowed_error =
          sampling_error + kDiscretizationAllowance * scale;
      EXPECT_LE(std::abs(empirical_covariance(row, column) -
                         predicted_covariance(row, column)),
                allowed_error)
          << "covariance_entry=(" << row << ',' << column << ")"
          << " empirical=" << empirical_covariance(row, column)
          << " predicted=" << predicted_covariance(row, column)
          << " allowance=" << allowed_error;
    }
  }
}

TEST(LocalEstimatorDiagnosticsTest,
     StraightScalarWheelHasNoFirstOrderLateralInformation) {
  constexpr double duration = 30.0;
  constexpr double gravity = 9.80665;
  Matrix15d dynamics = Matrix15d::Zero();
  dynamics.block<3, 3>(0, 3).setIdentity();
  dynamics.block<3, 3>(3, 6) = -Skew(Eigen::Vector3d(0.0, 0.0, gravity));
  dynamics.block<3, 3>(3, 12) = -Eigen::Matrix3d::Identity();
  dynamics.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 1, 15> observation =
      Eigen::Matrix<double, 1, 15>::Zero();
  observation(3) = 1.0;
  LocalConfig config;
  LocalEstimator estimator(config);
  LocalState state;
  state.velocity = Eigen::Vector3d(1.6, 0.0, 0.0);
  state.covariance = Matrix15d::Identity();
  LocalEstimatorTrace trace;
  estimator.SetTraceCallback(
      [&trace](const LocalEstimatorTrace& value) { trace = value; });
  ImuSample start = MakeImu(1.0, 1);
  start.acceleration.z() = gravity;
  ImuSample end = start;
  end.stamp = MakeStamp(1.01, 2);
  Matrix15d transition = Matrix15d::Identity();
  ASSERT_TRUE(LocalEstimatorTestAccess::Propagate(estimator, start, end, &state,
                                                  &transition)
                  .ok());
  EXPECT_TRUE(trace.error_transition.isApprox(
      Matrix15d::Identity() + 0.01 * dynamics + 0.00005 * dynamics * dynamics,
      1e-12));
  ASSERT_TRUE(LocalEstimatorTestAccess::UpdateWheel(
                  &estimator, MakeWheel(1.01, 2, 1.6), end, &state, &transition)
                  .ok());
  EXPECT_TRUE(trace.wheel_observation.row(0).isApprox(observation, 1e-12));
  // These directions cannot be admitted as information from scalar wheel speed.
  for (int index : {0, 1, 2, 4, 5, 6, 8, 9, 11, 13, 14}) {
    Eigen::Matrix<double, 15, 1> direction =
        Eigen::Matrix<double, 15, 1>::Unit(index);
    for (int power = 0; power < 15; ++power) {
      EXPECT_NEAR((observation * direction)(0), 0.0, 1e-12);
      direction = (dynamics * direction).eval();
    }
  }
  Eigen::Matrix<double, 15, 1> tilt_bias = Eigen::Matrix<double, 15, 1>::Zero();
  tilt_bias(7) = 1.0;
  tilt_bias(12) = gravity;
  EXPECT_NEAR((observation * dynamics * tilt_bias)(0), 0.0, 1e-12);
  Eigen::Matrix<double, 15, 1> forward_bias =
      Eigen::Matrix<double, 15, 1>::Unit(10);
  EXPECT_NEAR((observation * dynamics * dynamics * forward_bias)(0), -gravity,
              1e-12);
  EXPECT_NEAR((observation * dynamics)(7), gravity, 1e-12);
  EXPECT_NEAR((observation * dynamics)(12), -1.0, 1e-12);
  EXPECT_EQ((observation * dynamics * dynamics * dynamics).norm(), 0.0);
  std::cout << "STRAIGHT_OBSERVABILITY duration_s=" << duration
            << " scalar_wheel_observable_combinations=3"
            << " lateral_velocity_roll_gyro_x_unobservable=true\n";
}

TEST(LocalEstimatorDiagnosticsTest,
     RearSteeringTruthConflictsWithZeroLateralNhcAtRearAxle) {
  LocalConfig config;
  config.use_nonholonomic_constraint = true;
  LocalEstimator estimator(config);
  constexpr double rear_angle = -5.0 * 3.14159265358979323846 / 180.0;
  LocalState state;
  state.velocity = Eigen::Vector3d(1.6, 1.6 * std::tan(rear_angle), 0.0);
  state.covariance = Matrix15d::Identity();
  const Eigen::Vector3d truth_velocity = state.velocity;
  LocalEstimatorTrace trace;
  estimator.SetTraceCallback(
      [&trace](const LocalEstimatorTrace& value) { trace = value; });
  Matrix15d transition = Matrix15d::Identity();
  ASSERT_TRUE(LocalEstimatorTestAccess::UpdateWheel(
                  &estimator, MakeWheel(1.0, 1, 1.6), MakeImu(1.0, 1), &state,
                  &transition)
                  .ok());
  EXPECT_NEAR(trace.wheel_innovation.y(), -truth_velocity.y(), 1e-12);
  EXPECT_GT((state.velocity - truth_velocity).norm(), 0.01);
  std::cout << "REAR_STEER_NHC rear_angle_rad=" << rear_angle
            << " true_body_vy_mps=" << truth_velocity.y()
            << " pseudo_innovation_y_mps=" << trace.wheel_innovation.y()
            << " induced_velocity_error_mps="
            << (state.velocity - truth_velocity).norm() << '\n';
}

TEST(LocalEstimatorDiagnosticsTest,
     ThirtySecondMatchedNoiseWheelFeedbackEnsemble) {
  constexpr int trials = 64;
  constexpr int intervals = 600;
  constexpr double dt = 0.05;
  using Error = Eigen::Matrix<double, 15, 1>;
  const char* names[] = {"original", "forward_frozen_bias", "wheel_disabled"};
  for (double scale : {0.0001, 0.01, 1.0}) {
    LocalConfig config;
    config.max_imu_gap = 0.06;
    config.gyro_noise *= scale;
    config.accel_noise *= scale;
    config.gyro_bias_noise *= scale;
    config.accel_bias_noise *= scale;
    Matrix15d initial_covariance = Matrix15d::Zero();
    initial_covariance.diagonal().segment<3>(0).setConstant(1e-6 * scale *
                                                            scale);
    initial_covariance.diagonal().segment<3>(3).setConstant(
        std::pow(config.initial_velocity_std * scale, 2));
    initial_covariance.diagonal().segment<3>(6).setConstant(
        std::pow(config.initial_attitude_std * scale, 2));
    initial_covariance.diagonal().segment<3>(9).setConstant(
        std::pow(config.initial_gyro_bias_std * scale, 2));
    initial_covariance.diagonal().segment<3>(12).setConstant(
        std::pow(config.initial_accel_bias_std * scale, 2));
    const double wheel_variance = 0.01 * scale * scale;
    std::mt19937_64 generator(202610142013);
    std::normal_distribution<double> normal(0.0, 1.0);
    const auto noise = [&](double sigma) -> Eigen::Vector3d {
      Eigen::Vector3d sample;
      for (int axis = 0; axis < 3; ++axis) {
        sample(axis) = sigma * normal(generator);
      }
      return sample;
    };
    std::array<Error, 3> squared_error_sum{Error::Zero(), Error::Zero(),
                                           Error::Zero()};
    std::array<Error, 3> predicted_diagonal_sum{Error::Zero(), Error::Zero(),
                                                Error::Zero()};
    std::array<Error, 3> normalized_error_sum{Error::Zero(), Error::Zero(),
                                              Error::Zero()};
    std::array<double, 3> nees_sum{};
    std::array<double, 3> velocity_nees_sum{};
    std::array<double, 3> nis_sum{};
    std::array<int, 3> rejected{};
    for (int trial = 0; trial < trials; ++trial) {
      LocalState initial;
      Error perturbation;
      for (int index = 0; index < 15; ++index) {
        perturbation(index) =
            std::sqrt(initial_covariance(index, index)) * normal(generator);
      }
      initial.position = perturbation.segment<3>(0);
      initial.velocity =
          Eigen::Vector3d(1.6, 0.0, 0.0) + perturbation.segment<3>(3);
      initial.orientation = ExpRotation(perturbation.segment<3>(6));
      initial.gyro_bias = perturbation.segment<3>(9);
      initial.accel_bias = perturbation.segment<3>(12);
      initial.covariance = initial_covariance;
      std::array<LocalState, 3> states{initial, initial, initial};
      std::array<LocalEstimator, 3> estimators{LocalEstimator(config),
                                               LocalEstimator(config),
                                               LocalEstimator(config)};
      LocalEstimatorTestAccess::SetWheelBiasFeedback(&estimators[1], false,
                                                     false, true);
      Eigen::Vector3d true_bg = Eigen::Vector3d::Zero();
      Eigen::Vector3d true_ba = Eigen::Vector3d::Zero();
      for (int interval = 0; interval < intervals; ++interval) {
        const Eigen::Vector3d bg_step =
            noise(config.gyro_bias_noise * std::sqrt(dt));
        const Eigen::Vector3d ba_step =
            noise(config.accel_bias_noise * std::sqrt(dt));
        ImuSample start = MakeImu(1.0 + interval * dt, interval + 1);
        // One independent interval-average input shared by both endpoints.
        // This isolates Q from adjacent-sample averaging correlations.
        start.acceleration = Eigen::Vector3d(0.0, 0.0, config.gravity) +
                             true_ba + 0.5 * ba_step +
                             noise(config.accel_noise / std::sqrt(dt));
        start.angular_velocity =
            true_bg + 0.5 * bg_step + noise(config.gyro_noise / std::sqrt(dt));
        ImuSample end = start;
        end.stamp = MakeStamp(1.0 + (interval + 1) * dt, interval + 2);
        true_bg += bg_step;
        true_ba += ba_step;
        WheelSample wheel =
            MakeWheel(end.stamp.time, interval + 2,
                      1.6 + std::sqrt(wheel_variance) * normal(generator));
        wheel.variance = wheel_variance;
        for (int strategy = 0; strategy < 3; ++strategy) {
          Matrix15d transition = Matrix15d::Identity();
          ASSERT_TRUE(LocalEstimatorTestAccess::Propagate(
                          estimators[strategy], start, end, &states[strategy],
                          &transition)
                          .ok());
          if (strategy < 2) {
            const Eigen::Matrix3d rotation =
                states[strategy].orientation.toRotationMatrix().transpose();
            const Eigen::Vector3d body_velocity =
                rotation * states[strategy].velocity;
            Eigen::Matrix<double, 1, 15> observation =
                Eigen::Matrix<double, 1, 15>::Zero();
            observation.segment<3>(3) = rotation.row(0);
            observation.segment<3>(6) = Skew(body_velocity).row(0);
            const double innovation = wheel.speed - body_velocity.x();
            const double variance = (observation * states[strategy].covariance *
                                     observation.transpose())(0, 0) +
                                    wheel_variance;
            nis_sum[strategy] += innovation * innovation / variance;
            const Result result = LocalEstimatorTestAccess::UpdateWheel(
                &estimators[strategy], wheel, end, &states[strategy],
                &transition);
            if (!result.ok()) {
              ASSERT_EQ(result.reason, Reason::INNOVATION_REJECTED)
                  << result.message;
              ++rejected[strategy];
            }
          }
        }
      }
      for (int strategy = 0; strategy < 3; ++strategy) {
        const LocalState& state = states[strategy];
        Error error;
        error.segment<3>(0) =
            Eigen::Vector3d(1.6 * intervals * dt, 0.0, 0.0) - state.position;
        error.segment<3>(3) = Eigen::Vector3d(1.6, 0.0, 0.0) - state.velocity;
        error.segment<3>(6) = LogRotation(state.orientation.conjugate());
        error.segment<3>(9) = true_bg - state.gyro_bias;
        error.segment<3>(12) = true_ba - state.accel_bias;
        squared_error_sum[strategy] += error.cwiseProduct(error);
        predicted_diagonal_sum[strategy] += state.covariance.diagonal();
        normalized_error_sum[strategy] +=
            error.cwiseProduct(error).cwiseQuotient(
                state.covariance.diagonal());
        Eigen::LLT<Matrix15d> factor(state.covariance);
        ASSERT_EQ(factor.info(), Eigen::Success);
        nees_sum[strategy] += error.dot(factor.solve(error));
        const Eigen::Matrix3d velocity_covariance =
            state.covariance.block<3, 3>(3, 3);
        Eigen::LLT<Eigen::Matrix3d> velocity_factor(velocity_covariance);
        ASSERT_EQ(velocity_factor.info(), Eigen::Success);
        velocity_nees_sum[strategy] +=
            error.segment<3>(3).dot(velocity_factor.solve(error.segment<3>(3)));
      }
    }
    for (int strategy = 0; strategy < 3; ++strategy) {
      std::cout << "MATCHED_ENSEMBLE scale=" << scale
                << " strategy=" << names[strategy] << " trials=" << trials
                << " duration_s=" << intervals * dt
                << " mean_nees_15d=" << nees_sum[strategy] / trials
                << " mean_nees_velocity_3d="
                << velocity_nees_sum[strategy] / trials
                << " raw_nis_mean=" << nis_sum[strategy] / (trials * intervals)
                << " rejected=" << rejected[strategy];
      for (int axis : {0, 1, 3, 4, 6, 9}) {
        std::cout << " rmse_" << axis << '='
                  << std::sqrt(squared_error_sum[strategy](axis) / trials)
                  << " predicted_rms_sigma_" << axis << '='
                  << std::sqrt(predicted_diagonal_sum[strategy](axis) / trials)
                  << " marginal_nees_" << axis << '='
                  << normalized_error_sum[strategy](axis) / trials;
      }
      std::cout << '\n';
      if (scale == 0.0001) {
        // Four ensemble standard errors plus a discretization allowance.
        const double allowance = 4.0 * std::sqrt(30.0 / trials) + 1.0;
        EXPECT_NEAR(nees_sum[strategy] / trials, 15.0, allowance)
            << names[strategy];
      }
    }
  }
}

TEST(LocalEstimatorDiagnosticsTest,
     DisablingWheelUpdatesPreservesInitializationAndImuPropagation) {
  LocalConfig enabled_config = TestConfig();
  LocalConfig disabled_config = enabled_config;
  disabled_config.use_wheel_velocity_update = false;
  LocalEstimator enabled(enabled_config);
  LocalEstimator disabled(disabled_config);
  std::vector<LocalEstimatorTrace> enabled_trace;
  std::vector<LocalEstimatorTrace> disabled_trace;
  enabled.SetTraceCallback([&enabled_trace](const LocalEstimatorTrace& event) {
    enabled_trace.push_back(event);
  });
  disabled.SetTraceCallback(
      [&disabled_trace](const LocalEstimatorTrace& event) {
        disabled_trace.push_back(event);
      });

  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    const WheelSample wheel = MakeWheel(time, sequence);
    const ImuSample imu = MakeImu(time, sequence);
    ASSERT_TRUE(enabled.AddWheel(wheel).ok());
    ASSERT_TRUE(disabled.AddWheel(wheel).ok());
    ASSERT_TRUE(enabled.AddImu(imu).ok());
    ASSERT_TRUE(disabled.AddImu(imu).ok());
  }
  ASSERT_TRUE(enabled.state().valid);
  ASSERT_TRUE(disabled.state().valid);
  ExpectSameState(enabled.state(), disabled.state());
  const LocalState initialized = enabled.state();

  for (uint64_t sequence = 4; sequence <= 8; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    const WheelSample wheel = MakeWheel(time, sequence, 0.4);
    ImuSample imu = MakeImu(time, sequence);
    imu.acceleration.y() = 1.0;
    ASSERT_TRUE(enabled.AddWheel(wheel).ok());
    ASSERT_TRUE(disabled.AddWheel(wheel).ok());
    ASSERT_TRUE(enabled.AddImu(imu).ok());
    const Result disabled_result = disabled.AddImu(imu);
    EXPECT_TRUE(disabled_result.ok() ||
                disabled_result.reason == Reason::WHEEL_STALE)
        << disabled_result.message;
    EXPECT_TRUE(disabled.EvaluateIntegrity(time).ok());
    EXPECT_DOUBLE_EQ(disabled.state().stamp.time, time);
  }

  EXPECT_EQ(enabled.wheel_health().accepted, disabled.wheel_health().accepted);
  EXPECT_GT((enabled.state().velocity - disabled.state().velocity).norm(), 0.1);
  EXPECT_EQ(
      (initialized.position - disabled_trace.front().position_before).norm(),
      0.0);
  EXPECT_TRUE(std::none_of(
      disabled_trace.begin(), disabled_trace.end(), [](const auto& event) {
        return event.kind == LocalEstimatorTraceKind::WHEEL_UPDATE;
      }));
  EXPECT_TRUE(std::any_of(
      enabled_trace.begin(), enabled_trace.end(), [](const auto& event) {
        return event.kind == LocalEstimatorTraceKind::WHEEL_UPDATE;
      }));
}

TEST_F(LocalEstimatorTest, StartupGyroAndAccelerationMotionBreakStationaryRun) {
  for (uint64_t sequence = 1; sequence <= 9; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    ImuSample imu = MakeImu(time, sequence);
    if (sequence == 3) {
      imu.angular_velocity.z() = 0.5;
    }
    if (sequence == 6) {
      imu.acceleration.z() = 11.0;
    }
    const Result result = estimator_->AddImu(imu);
    EXPECT_TRUE(result.ok());
    EXPECT_EQ(estimator_->Evaluate(time).reason,
              sequence < 9 ? Reason::WAITING_FOR_STATIONARY : Reason::NONE);
    EXPECT_EQ(estimator_->state().valid, sequence == 9);
  }
  EXPECT_EQ(estimator_->state().gyro_bias.norm(), 0.0);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, ExactDuplicateImuDoesNotPropagateOrAlterState) {
  Initialize();
  const LocalState before = estimator_->state();
  const uint64_t accepted = estimator_->imu_health().accepted;
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.02, 3)).reason, Reason::DUPLICATE);
  ExpectSameState(before, estimator_->state());
  EXPECT_EQ(estimator_->imu_health().accepted, accepted);
  EXPECT_EQ(estimator_->imu_health().rejected, 1U);
  EXPECT_EQ(estimator_->imu_health().duplicates, 1U);
  EXPECT_TRUE(estimator_->Evaluate(1.02).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  EXPECT_EQ(estimator_->state().stamp.sequence, 4U);
  EXPECT_TRUE(estimator_->Evaluate(1.03).ok());
}

TEST_F(LocalEstimatorTest, InvalidImuDoesNotAdvanceWatermarkOrPose) {
  Initialize();
  const LocalState before = estimator_->state();
  std::vector<ImuSample> samples;
  std::vector<Reason> reasons;
  ImuSample sample = MakeImu(1.03, 4);
  sample.stamp.clock_id.clear();
  samples.push_back(sample);
  reasons.push_back(Reason::CLOCK_INVALID);
  sample = MakeImu(1.03, 4);
  sample.stamp.time = 0.0;
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 4);
  sample.stamp.receive_time = 0.0;
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 0);
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 4);
  sample.stamp.receive_time = std::numeric_limits<double>::quiet_NaN();
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 4);
  sample.stamp.time = std::numeric_limits<double>::infinity();
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 4);
  sample.stamp.receive_time = 1.0;
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 4);
  sample.stamp.receive_time = 1.2;
  samples.push_back(sample);
  reasons.push_back(Reason::IMU_STALE);
  sample = MakeImu(1.01, 4);
  samples.push_back(sample);
  reasons.push_back(Reason::TIMESTAMP_REGRESSION);
  sample = MakeImu(1.03, 2);
  samples.push_back(sample);
  reasons.push_back(Reason::TIMESTAMP_REGRESSION);
  sample = MakeImu(1.03, 4);
  sample.stamp.receive_time = 1.0195;
  sample.stamp.time = 1.02;
  samples.push_back(sample);
  reasons.push_back(Reason::TIMESTAMP_REGRESSION);
  sample = MakeImu(1.02, 4);
  samples.push_back(sample);
  reasons.push_back(Reason::DUPLICATE);
  sample = MakeImu(1.03, 4);
  sample.acceleration.x() = std::numeric_limits<double>::quiet_NaN();
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  sample = MakeImu(1.03, 4);
  sample.angular_velocity.z() = std::numeric_limits<double>::infinity();
  samples.push_back(sample);
  reasons.push_back(Reason::INVALID_INPUT);
  for (size_t i = 0; i < samples.size(); ++i) {
    EXPECT_EQ(estimator_->AddImu(samples[i]).reason, reasons[i]) << i;
    ExpectSameState(before, estimator_->state());
    EXPECT_EQ(estimator_->imu_health().accepted, 3U);
    EXPECT_TRUE(estimator_->Evaluate(1.03).ok());
  }
  EXPECT_EQ(estimator_->imu_health().rejected, samples.size());
  EXPECT_EQ(estimator_->imu_health().regressions, 3U);
  EXPECT_EQ(estimator_->imu_health().last_measurement, 1.02);
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  EXPECT_EQ(estimator_->state().stamp.time, 1.03);
}

TEST_F(LocalEstimatorTest,
       WheelRejectsDuplicateRegressionClockAndInvalidFields) {
  Initialize();
  const LocalState before = estimator_->state();
  EXPECT_EQ(estimator_->AddWheel(MakeWheel(1.02, 3)).reason, Reason::DUPLICATE);
  EXPECT_EQ(estimator_->AddWheel(MakeWheel(1.01, 4)).reason,
            Reason::TIMESTAMP_REGRESSION);
  EXPECT_EQ(estimator_->AddWheel(MakeWheel(1.03, 2)).reason,
            Reason::TIMESTAMP_REGRESSION);
  WheelSample wheel = MakeWheel(1.03, 4);
  wheel.stamp.clock_id = "another-clock";
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::CLOCK_INVALID);
  wheel = MakeWheel(1.03, 0);
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  wheel = MakeWheel(1.03, 4);
  wheel.stamp.receive_time = 0.0;
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  wheel = MakeWheel(1.03, 4);
  wheel.stamp.receive_time = 1.0;
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  wheel = MakeWheel(1.03, 4);
  wheel.stamp.receive_time = 1.2;
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::WHEEL_STALE);
  wheel = MakeWheel(1.03, 4);
  wheel.stamp.receive_time = 1.09;
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::WHEEL_STALE);
  wheel = MakeWheel(1.03, 4);
  wheel.speed = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  wheel = MakeWheel(1.03, 4);
  wheel.variance = -0.01;
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  wheel.variance = 0.0;
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  wheel.variance = std::numeric_limits<double>::infinity();
  EXPECT_EQ(estimator_->AddWheel(wheel).reason, Reason::INVALID_INPUT);
  ExpectSameState(before, estimator_->state());
  EXPECT_EQ(estimator_->wheel_health().accepted, 3U);
  EXPECT_EQ(estimator_->wheel_health().duplicates, 1U);
  EXPECT_EQ(estimator_->wheel_health().regressions, 2U);
  EXPECT_TRUE(estimator_->Evaluate(1.03).ok());
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4)).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
}

TEST_F(LocalEstimatorTest,
       StraightAccelerationAndWheelVelocityRemainContinuous) {
  Initialize();
  // Midpoint from the last stationary sample to the first accelerating sample.
  for (uint64_t sequence = 4; sequence <= 103; ++sequence) {
    const double elapsed = 0.01 * static_cast<double>(sequence - 3);
    const double time = 1.02 + elapsed;
    ASSERT_TRUE(
        estimator_->AddWheel(MakeWheel(time, sequence, elapsed - 0.005)).ok());
    ImuSample imu = MakeImu(time, sequence);
    imu.acceleration.x() = 1.0;
    ASSERT_TRUE(estimator_->AddImu(imu).ok());
  }
  EXPECT_NEAR(estimator_->state().velocity.x(), 0.995, 1e-9);
  EXPECT_NEAR(estimator_->state().position.x(), 0.495025, 1e-9);
  EXPECT_NEAR(estimator_->state().position.y(), 0.0, 1e-12);
  EXPECT_NEAR(estimator_->state().position.z(), 0.0, 1e-12);
  EXPECT_NEAR(LogRotation(estimator_->state().orientation).norm(), 0.0, 1e-12);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, WheelUpdateUsesInterpolatedMeasurementTime) {
  Initialize();
  LocalEstimator bracketed(config_);
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(bracketed.AddWheel(MakeWheel(time, sequence)).ok());
    ASSERT_TRUE(bracketed.AddImu(MakeImu(time, sequence)).ok());
  }

  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.025, 4, 0.5)).ok());
  ImuSample interval_end = MakeImu(1.03, 4);
  interval_end.acceleration.x() = 1.0;
  ASSERT_TRUE(estimator_->AddImu(interval_end).ok());

  ImuSample midpoint = MakeImu(1.025, 4);
  midpoint.acceleration.x() = 0.5;
  ASSERT_TRUE(bracketed.AddImu(midpoint).ok());
  ASSERT_TRUE(bracketed.AddWheel(MakeWheel(1.025, 4, 0.5)).ok());
  interval_end.stamp.sequence = 5;
  ASSERT_TRUE(bracketed.AddImu(interval_end).ok());

  EXPECT_NEAR(
      (estimator_->state().position - bracketed.state().position).norm(), 0.0,
      1e-14);
  EXPECT_NEAR(
      (estimator_->state().velocity - bracketed.state().velocity).norm(), 0.0,
      1e-14);
  EXPECT_NEAR((estimator_->state().orientation.coeffs() -
               bracketed.state().orientation.coeffs())
                  .norm(),
              0.0, 1e-14);
  EXPECT_NEAR(
      (estimator_->state().covariance - bracketed.state().covariance).norm(),
      0.0, 1e-13);
}

TEST_F(LocalEstimatorTest, MultipleQueuedWheelsFuseInMeasurementTimeOrder) {
  Initialize();
  LocalEstimator bracketed(config_);
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(bracketed.AddWheel(MakeWheel(time, sequence)).ok());
    ASSERT_TRUE(bracketed.AddImu(MakeImu(time, sequence)).ok());
  }

  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.024, 4, 0.2)).ok());
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.028, 5, 0.4)).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());

  ASSERT_TRUE(bracketed.AddImu(MakeImu(1.024, 4)).ok());
  ASSERT_TRUE(bracketed.AddWheel(MakeWheel(1.024, 4, 0.2)).ok());
  ASSERT_TRUE(bracketed.AddImu(MakeImu(1.028, 5)).ok());
  ASSERT_TRUE(bracketed.AddWheel(MakeWheel(1.028, 5, 0.4)).ok());
  ASSERT_TRUE(bracketed.AddImu(MakeImu(1.03, 6)).ok());

  EXPECT_NEAR(
      (estimator_->state().position - bracketed.state().position).norm(), 0.0,
      1e-14);
  EXPECT_NEAR(
      (estimator_->state().velocity - bracketed.state().velocity).norm(), 0.0,
      1e-14);
  EXPECT_NEAR(
      (estimator_->state().covariance - bracketed.state().covariance).norm(),
      0.0, 1e-13);
  EXPECT_EQ(estimator_->wheel_health().accepted, 5U);
}

TEST_F(LocalEstimatorTest, GyroIntegratesLocalYawWithoutAbsoluteHeading) {
  Initialize();
  for (uint64_t sequence = 4; sequence <= 13; ++sequence) {
    const double time = 1.02 + 0.01 * static_cast<double>(sequence - 3);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    ImuSample imu = MakeImu(time, sequence);
    imu.angular_velocity.z() = 1.0;
    ASSERT_TRUE(estimator_->AddImu(imu).ok());
  }
  EXPECT_NEAR(LogRotation(estimator_->state().orientation).z(), 0.095, 1e-12);
  EXPECT_EQ(estimator_->state().position.norm(), 0.0);
  EXPECT_EQ(estimator_->state().velocity.norm(), 0.0);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, WheelUsesConstrainedGainAndNeverSnapsPose) {
  Initialize();
  LocalEstimator inertial_only(config_);
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(inertial_only.AddWheel(MakeWheel(time, sequence)).ok());
    inertial_only.AddImu(MakeImu(time, sequence));
  }
  ImuSample imu = MakeImu(1.03, 4);
  imu.acceleration.x() = 1.0;
  imu.angular_velocity.z() = 0.2;
  ASSERT_TRUE(inertial_only.AddImu(imu).ok());
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4, 0.5)).ok());
  ASSERT_TRUE(estimator_->AddImu(imu).ok());
  EXPECT_EQ(
      (estimator_->state().position - inertial_only.state().position).norm(),
      0.0);
  EXPECT_EQ((estimator_->state().orientation.coeffs() -
             inertial_only.state().orientation.coeffs())
                .norm(),
            0.0);
  EXPECT_GT(estimator_->state().velocity.x(),
            inertial_only.state().velocity.x());
  EXPECT_GT((estimator_->state().accel_bias - inertial_only.state().accel_bias)
                .norm(),
            0.0);
  EXPECT_NEAR(estimator_->state().covariance(0, 0),
              inertial_only.state().covariance(0, 0), 1e-15);
  EXPECT_NEAR(estimator_->state().covariance(6, 6),
              inertial_only.state().covariance(6, 6), 1e-15);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest,
       BufferedWheelIsFusedOnceAndNeverHeldAsRepeatedEvidence) {
  Initialize();
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.025, 4, 0.5)).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  const double corrected_speed = estimator_->state().velocity.x();
  EXPECT_GT(corrected_speed, 0.2);
  EXPECT_LT(corrected_speed, 0.3);
  const LocalState corrected = estimator_->state();
  EXPECT_EQ(estimator_->AddWheel(MakeWheel(1.025, 4, 0.5)).reason,
            Reason::DUPLICATE);
  ExpectSameState(corrected, estimator_->state());
  EXPECT_TRUE(estimator_->AddImu(MakeImu(1.04, 5)).ok());
  // Propagation consumes the actual IMU once without reusing rejected wheel
  // input.
  EXPECT_EQ(estimator_->state().stamp.sequence, 5U);
  EXPECT_LT(std::abs(estimator_->state().velocity.x() - corrected_speed),
            0.001);
  EXPECT_EQ(estimator_->wheel_health().accepted, 4U);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest,
       WheelInnovationRejectedOnceWithoutPoseOrVelocitySnap) {
  Initialize();
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4, 100.0)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.03, 4)).reason,
            Reason::INNOVATION_REJECTED);
  EXPECT_EQ(estimator_->state().velocity.norm(), 0.0);
  EXPECT_EQ(estimator_->state().position.norm(), 0.0);
  EXPECT_EQ(estimator_->wheel_health().rejected, 1U);
  EXPECT_TRUE(estimator_->state().valid);
  EXPECT_TRUE(estimator_->Evaluate(1.03).ok());
  EXPECT_TRUE(estimator_->AddImu(MakeImu(1.04, 5)).ok());
  EXPECT_EQ(estimator_->wheel_health().rejected, 1U);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.05, 5)).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.05, 6)).ok());
  EXPECT_TRUE(estimator_->state().valid);
}

TEST_F(LocalEstimatorTest, WheelsBeforeCommittedFrontierAreNotReplayed) {
  Initialize();
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  EXPECT_EQ(estimator_->AddWheel(MakeWheel(1.025, 4)).reason,
            Reason::HISTORY_UNAVAILABLE);
  EXPECT_EQ(estimator_->AddWheel(MakeWheel(1.06, 4)).reason,
            Reason::INVALID_INPUT);
  EXPECT_TRUE(estimator_->Evaluate(1.03).ok());
}

TEST_F(LocalEstimatorTest, PendingWheelOverflowIsExplicitWithoutOverwrite) {
  Initialize();
  for (uint64_t offset = 0; offset < 64; ++offset) {
    const double time = 1.0201 + 0.0003 * static_cast<double>(offset);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, 4 + offset, 0.1)).ok());
  }
  const WheelSample overflow = MakeWheel(1.0393, 68, 0.2);
  EXPECT_EQ(estimator_->AddWheel(overflow).reason, Reason::INVALID_INPUT);
  EXPECT_EQ(estimator_->wheel_health().accepted, 67U);
  EXPECT_EQ(estimator_->wheel_health().rejected, 1U);
  EXPECT_EQ(estimator_->wheel_health().evictions, 1U);
}

TEST_F(LocalEstimatorTest, BufferedWheelCannotBeFusedOutsideReceiveAgeWindow) {
  Initialize();
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4, 0.5)).ok());
  ImuSample imu = MakeImu(1.04, 4);
  imu.stamp.receive_time = 1.09;
  EXPECT_EQ(estimator_->AddImu(imu).reason, Reason::WHEEL_STALE);
  EXPECT_EQ(estimator_->state().velocity.norm(), 0.0);
  EXPECT_EQ(estimator_->wheel_health().rejected, 1U);
  EXPECT_TRUE(estimator_->EvaluateIntegrity(1.09).ok());
  EXPECT_TRUE(estimator_->state().valid);
}

TEST_F(LocalEstimatorTest, WheelFreshnessDegradesPrecisionNotIntegrity) {
  Initialize();
  EXPECT_EQ(estimator_->Evaluate(1.075).reason, Reason::WHEEL_STALE);
  EXPECT_TRUE(estimator_->EvaluateIntegrity(1.075).ok());
  EXPECT_EQ(estimator_->Evaluate(1.13).reason, Reason::IMU_STALE);
  EXPECT_EQ(estimator_->EvaluateIntegrity(1.13).reason, Reason::IMU_STALE);
  EXPECT_EQ(estimator_->Evaluate(1.0).reason, Reason::INVALID_INPUT);
  EXPECT_EQ(estimator_->EvaluateIntegrity(1.0).reason, Reason::INVALID_INPUT);
  EXPECT_EQ(
      estimator_->Evaluate(std::numeric_limits<double>::quiet_NaN()).reason,
      Reason::INVALID_INPUT);
  EXPECT_EQ(
      estimator_->EvaluateIntegrity(std::numeric_limits<double>::quiet_NaN())
          .reason,
      Reason::INVALID_INPUT);
  for (uint64_t sequence = 4; sequence <= 9; ++sequence) {
    const double time = 1.02 + 0.01 * static_cast<double>(sequence - 3);
    const Result result = estimator_->AddImu(MakeImu(time, sequence));
    if (sequence <= 7) {
      EXPECT_TRUE(result.ok()) << result.message;
    } else {
      EXPECT_EQ(result.reason, Reason::WHEEL_STALE);
      EXPECT_TRUE(estimator_->state().valid);
      EXPECT_TRUE(estimator_->EvaluateIntegrity(time).ok());
    }
  }
  EXPECT_EQ(estimator_->wheel_health().accepted, 3U);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.09, 4)).ok());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.09, 10)).ok());
  EXPECT_TRUE(estimator_->state().valid);
}

TEST_F(LocalEstimatorTest,
       NonholonomicConstraintsReduceTransverseVelocityOnly) {
  config_.use_nonholonomic_constraint = true;
  estimator_.reset(new LocalEstimator(config_));
  Initialize();
  LocalConfig free_config = config_;
  free_config.use_nonholonomic_constraint = false;
  LocalEstimator free_estimator(free_config);
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const double time = 1.0 + 0.01 * static_cast<double>(sequence - 1);
    ASSERT_TRUE(free_estimator.AddWheel(MakeWheel(time, sequence)).ok());
    free_estimator.AddImu(MakeImu(time, sequence));
  }
  ImuSample imu = MakeImu(1.03, 4);
  imu.acceleration.y() = 2.0;
  imu.acceleration.z() = 11.0;
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4)).ok());
  ASSERT_TRUE(free_estimator.AddWheel(MakeWheel(1.03, 4)).ok());
  ASSERT_TRUE(estimator_->AddImu(imu).ok());
  ASSERT_TRUE(free_estimator.AddImu(imu).ok());
  EXPECT_LT(std::abs(estimator_->state().velocity.y()),
            std::abs(free_estimator.state().velocity.y()));
  EXPECT_LT(std::abs(estimator_->state().velocity.z()),
            std::abs(free_estimator.state().velocity.z()));
  EXPECT_EQ(
      (estimator_->state().position - free_estimator.state().position).norm(),
      0.0);
  EXPECT_EQ((estimator_->state().orientation.coeffs() -
             free_estimator.state().orientation.coeffs())
                .norm(),
            0.0);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest,
       ImuGapLatchesFaultWithoutTeleportOrAutomaticRecovery) {
  Initialize();
  const LocalState before = estimator_->state();
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.1, 4)).reason, Reason::IMU_GAP);
  EXPECT_EQ(estimator_->state().stamp.time, before.stamp.time);
  EXPECT_EQ((estimator_->state().position - before.position).norm(), 0.0);
  EXPECT_EQ(
      (estimator_->state().orientation.coeffs() - before.orientation.coeffs())
          .norm(),
      0.0);
  EXPECT_EQ((estimator_->state().covariance - before.covariance).norm(), 0.0);
  EXPECT_FALSE(estimator_->state().valid);
  EXPECT_EQ(estimator_->imu_health().accepted, 4U);
  EXPECT_EQ(estimator_->imu_health().rejected, 0U);
  EXPECT_EQ(estimator_->ResetStopped(1.1).reason, Reason::RESET_WHILE_MOVING);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.11, 4)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.11, 5)).reason, Reason::IMU_GAP);
  EXPECT_EQ(estimator_->Evaluate(1.11).reason, Reason::IMU_GAP);
  EXPECT_EQ(estimator_->EvaluateIntegrity(1.11).reason, Reason::IMU_GAP);
  EXPECT_EQ(estimator_->state().stamp.time, before.stamp.time);
  EXPECT_EQ(estimator_->state().epoch.generation, 1U);
}

TEST_F(LocalEstimatorTest,
       StoppedResetChangesEpochAndRequiresNewAlignmentEvidence) {
  Initialize();
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.1, 4)).reason, Reason::IMU_GAP);
  for (uint64_t sequence = 5; sequence <= 7; ++sequence) {
    const double time = 1.11 + 0.01 * static_cast<double>(sequence - 5);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    EXPECT_EQ(estimator_->AddImu(MakeImu(time, sequence)).reason,
              Reason::IMU_GAP);
  }
  ASSERT_TRUE(estimator_->ResetStopped(1.14).ok());
  EXPECT_EQ(estimator_->state().epoch.session, config_.session);
  EXPECT_EQ(estimator_->state().epoch.generation, 2U);
  EXPECT_FALSE(estimator_->state().valid);
  EXPECT_NEAR(estimator_->state().stamp.time, 1.13, 1e-12);
  EXPECT_EQ(estimator_->state().position.norm(), 0.0);
  EXPECT_EQ(estimator_->Evaluate(1.14).reason, Reason::WAITING_FOR_IMU);
  EXPECT_EQ(estimator_->ResetStopped(1.14).reason, Reason::RESET_WHILE_MOVING);
  // Epoch reset clears evidence, not the source duplicate watermarks.
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.13, 7)).reason, Reason::DUPLICATE);
  for (uint64_t sequence = 8; sequence <= 10; ++sequence) {
    const double time = 1.14 + 0.01 * static_cast<double>(sequence - 8);
    ASSERT_TRUE(estimator_->AddWheel(MakeWheel(time, sequence)).ok());
    EXPECT_TRUE(estimator_->AddImu(MakeImu(time, sequence)).ok());
    EXPECT_EQ(estimator_->Evaluate(time).reason,
              sequence < 10 ? Reason::WAITING_FOR_STATIONARY : Reason::NONE);
  }
  EXPECT_TRUE(estimator_->state().valid);
  EXPECT_EQ(estimator_->state().epoch.generation, 2U);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, MovingOrStaleEvidenceCannotResetEpoch) {
  Initialize();
  const LocalState before = estimator_->state();
  EXPECT_EQ(estimator_->ResetStopped(1.2).reason, Reason::RESET_WHILE_MOVING);
  ExpectSameState(before, estimator_->state());
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4, 0.5)).ok());
  EXPECT_EQ(estimator_->ResetStopped(1.03).reason, Reason::RESET_WHILE_MOVING);
  ExpectSameState(before, estimator_->state());
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.04, 5)).ok());
  ImuSample imu = MakeImu(1.04, 5);
  imu.angular_velocity.z() = 0.5;
  ASSERT_TRUE(estimator_->AddImu(imu).ok());
  const LocalState moving = estimator_->state();
  EXPECT_EQ(estimator_->ResetStopped(1.04).reason, Reason::RESET_WHILE_MOVING);
  ExpectSameState(moving, estimator_->state());
  EXPECT_EQ(estimator_->ResetStopped(0.0).reason, Reason::INVALID_INPUT);
  ExpectSameState(moving, estimator_->state());
}

TEST_F(LocalEstimatorTest, RecoveryNeedsConsecutiveBoundedStationaryEvidence) {
  Initialize();
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.1, 4)).reason, Reason::IMU_GAP);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.11, 4)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.11, 5)).reason, Reason::IMU_GAP);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.2, 5)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.2, 6)).reason, Reason::IMU_GAP);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.21, 6)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.21, 7)).reason, Reason::IMU_GAP);
  // A second recovery gap discarded the first stopped sample.
  EXPECT_EQ(estimator_->ResetStopped(1.21).reason, Reason::RESET_WHILE_MOVING);
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.22, 7)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.22, 8)).reason, Reason::IMU_GAP);
  ASSERT_TRUE(estimator_->ResetStopped(1.22).ok());
}

TEST_F(LocalEstimatorTest, PositionGaugeUncertaintyGrowsAndTripsPerAxisBudget) {
  config_.max_position_std = 0.0001;
  estimator_.reset(new LocalEstimator(config_));
  Initialize();
  ASSERT_TRUE(estimator_->AddWheel(MakeWheel(1.03, 4)).ok());
  EXPECT_EQ(estimator_->AddImu(MakeImu(1.03, 4)).reason,
            Reason::COVARIANCE_EXCEEDED);
  EXPECT_EQ(estimator_->Evaluate(1.03).reason, Reason::COVARIANCE_EXCEEDED);
  EXPECT_TRUE(estimator_->EvaluateIntegrity(1.03).ok());
  EXPECT_TRUE(estimator_->state().valid);
  EXPECT_GT(std::sqrt(estimator_->state().covariance(0, 0)),
            config_.max_position_std);
  const Eigen::Vector3d first_position = estimator_->state().position;
  ImuSample next = MakeImu(1.04, 5);
  next.acceleration.x() = 1.0;
  EXPECT_EQ(estimator_->AddImu(next).reason, Reason::COVARIANCE_EXCEEDED);
  EXPECT_EQ(estimator_->state().stamp.time, 1.04);
  EXPECT_TRUE(estimator_->state().valid);
  EXPECT_TRUE(estimator_->EvaluateIntegrity(1.04).ok());
  EXPECT_NE((estimator_->state().position - first_position).norm(), 0.0);
  ExpectPositiveCovariance(estimator_->state());
}

TEST_F(LocalEstimatorTest, VelocityAndAttitudeEachHaveIndependentSigmaBudgets) {
  for (bool attitude_budget : {false, true}) {
    config_ = TestConfig();
    if (attitude_budget) {
      config_.max_attitude_std = config_.initial_attitude_std;
    } else {
      config_.max_velocity_std = config_.initial_velocity_std;
    }
    estimator_.reset(new LocalEstimator(config_));
    Initialize();
    EXPECT_EQ(estimator_->AddImu(MakeImu(1.03, 4)).reason,
              Reason::COVARIANCE_EXCEEDED);
    EXPECT_TRUE(estimator_->EvaluateIntegrity(1.03).ok());
    EXPECT_TRUE(estimator_->state().valid);
    const int index = attitude_budget ? 6 : 3;
    const double limit =
        attitude_budget ? config_.max_attitude_std : config_.max_velocity_std;
    EXPECT_GT(std::sqrt(estimator_->state().covariance(index, index)), limit);
    ExpectPositiveCovariance(estimator_->state());
  }
}

TEST_F(LocalEstimatorTest, BiasRandomWalkAndFullCovarianceRemainPositive) {
  Initialize();
  const Matrix15d initial = estimator_->state().covariance;
  // No new wheel is fused, so the measured bias growth is only process noise.
  ASSERT_TRUE(estimator_->AddImu(MakeImu(1.03, 4)).ok());
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_GT(estimator_->state().covariance(9 + axis, 9 + axis),
              initial(9 + axis, 9 + axis));
    EXPECT_GT(estimator_->state().covariance(12 + axis, 12 + axis),
              initial(12 + axis, 12 + axis));
  }
  EXPECT_NE(estimator_->state().covariance(0, 3), 0.0);
  EXPECT_NE(estimator_->state().covariance(3, 12), 0.0);
  EXPECT_NE(estimator_->state().covariance(6, 9), 0.0);
  ExpectPositiveCovariance(estimator_->state());
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace openloc
