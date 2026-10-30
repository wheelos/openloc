// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "candidates/local/local_estimator.h"
#include "candidates/local/motion_bootstrap_error.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <utility>

#include "Eigen/Cholesky"

namespace openloc {
namespace localization {
namespace unified {
namespace {

bool Positive(double value) { return std::isfinite(value) && value > 0.0; }

bool Nonnegative(double value) { return std::isfinite(value) && value >= 0.0; }

Result CheckStamp(const Stamp& stamp, const LocalConfig& config,
                  const SourceCounters& health, uint64_t last_sequence,
                  Reason stale_reason) {
  if (stamp.clock_id != config.clock_id) {
    return {Reason::CLOCK_INVALID, "Measurement clock does not match"};
  }
  if (!Positive(stamp.time) || !Positive(stamp.receive_time) ||
      stamp.sequence == 0) {
    return {Reason::INVALID_INPUT, "Missing or nonfinite measurement stamp"};
  }
  if (stamp.time - stamp.receive_time > config.future_tolerance) {
    return {Reason::INVALID_INPUT, "Measurement is in the future"};
  }
  if (stamp.receive_time - stamp.time > config.max_sample_age) {
    return {stale_reason, "Measurement exceeded the receive-age limit"};
  }
  if (health.accepted != 0) {
    if (stamp.sequence == last_sequence) {
      return {Reason::DUPLICATE, "Sequence already consumed"};
    }
    if (stamp.sequence < last_sequence ||
        stamp.time < health.last_measurement ||
        stamp.receive_time < health.last_receive) {
      return {Reason::TIMESTAMP_REGRESSION,
              "Source time or sequence regressed"};
    }
    if (stamp.time == health.last_measurement) {
      return {Reason::DUPLICATE, "Measurement timestamp already consumed"};
    }
  }
  return {};
}

void Accept(const Stamp& stamp, SourceCounters* health) {
  ++health->accepted;
  health->last_measurement = stamp.time;
  health->last_receive = stamp.receive_time;
  health->last_sequence = stamp.sequence;
  health->reason = Reason::NONE;
}

ImuSample InterpolateImu(const ImuSample& start, const ImuSample& end,
                         double time) {
  if (time <= start.stamp.time) {
    return start;
  }
  if (time >= end.stamp.time) {
    return end;
  }
  const double alpha =
      (time - start.stamp.time) / (end.stamp.time - start.stamp.time);
  ImuSample sample;
  sample.stamp.time = time;
  sample.stamp.receive_time =
      start.stamp.receive_time +
      alpha * (end.stamp.receive_time - start.stamp.receive_time);
  sample.stamp.sequence = end.stamp.sequence;
  sample.stamp.clock_id = end.stamp.clock_id;
  sample.acceleration =
      ((1.0 - alpha) * start.acceleration + alpha * end.acceleration).eval();
  sample.angular_velocity =
      ((1.0 - alpha) * start.angular_velocity + alpha * end.angular_velocity)
          .eval();
  return sample;
}

bool FiniteState(const LocalState& state) {
  if (!state.position.allFinite() || !state.velocity.allFinite() ||
      !state.orientation.coeffs().allFinite() ||
      std::abs(state.orientation.norm() - 1.0) > 1e-9 ||
      !state.gyro_bias.allFinite() || !state.accel_bias.allFinite() ||
      !state.angular_velocity.allFinite() || !state.covariance.allFinite() ||
      !state.covariance.isApprox(state.covariance.transpose(), 1e-9)) {
    return false;
  }
  return ValidLocalCovariance(state);
}

Matrix15d DiscretizeProcessNoise(const Matrix15d& continuous_noise, double dt,
                                 const Matrix15d& transition,
                                 const Matrix15d& half_transition) {
  return (dt / 6.0) * (continuous_noise +
                       4.0 * half_transition * continuous_noise *
                           half_transition.transpose() +
                       transition * continuous_noise * transition.transpose());
}

}  // namespace

LocalEstimator::LocalEstimator(const LocalConfig& config)
    : config_(config),
      stationarity_(config),
      lio_(config),
      continuous_output_(config),
      joint_error_(config) {
  state_.epoch.session = config.session;
  state_.epoch.generation = 1;
  output_state_.epoch = state_.epoch;
}

Result LocalEstimator::ValidateConfig() const {
  if (config_.enable_native_structured_error &&
      !config_.enable_native_joint_error)
    return {Reason::CONFIG_INVALID,
            "Structured native errors require native joint propagation"};
  if (config_.enable_native_structured_error &&
      config_.native_joint_max_dimension < 18)
    return {Reason::CONFIG_INVALID,
            "Structured native queries require the 18D output memory"};
  if (config_.enable_native_joint_error &&
      (config_.native_joint_max_dimension < 15 ||
       config_.native_joint_max_memory_rows < 48 ||
       config_.native_joint_max_snapshots < 2 ||
       config_.native_joint_max_dimension > config_.native_joint_max_memory_rows))
    return {Reason::CONFIG_INVALID, "Invalid native joint-error resource bounds"};
  if (config_.session.empty() || config_.clock_id.empty() ||
      config_.stationary_samples == 0 || !Positive(config_.gravity) ||
      !Positive(config_.max_sample_age) ||
      !Nonnegative(config_.future_tolerance) ||
      !Positive(config_.max_imu_gap) || !Positive(config_.wheel_timeout) ||
      !Nonnegative(config_.stationary_speed) ||
      !Positive(config_.stationary_gyro) ||
      !Positive(config_.stationary_accel_tolerance) ||
      !Positive(config_.stationary_duration) ||
      !Nonnegative(config_.gyro_noise) || !Nonnegative(config_.accel_noise) ||
      !Nonnegative(config_.gyro_bias_noise) ||
      !Nonnegative(config_.accel_bias_noise) ||
      !Positive(config_.initial_velocity_std) ||
      !Positive(config_.initial_attitude_std) ||
      !Positive(config_.initial_gyro_bias_std) ||
      !Positive(config_.initial_accel_bias_std) ||
      !Positive(config_.nonholonomic_variance) ||
      !Nonnegative(config_.zero_velocity_variance) ||
      !Positive(config_.innovation_gate) ||
      !Positive(config_.max_position_std) ||
      !Positive(config_.max_velocity_std) ||
      !Positive(config_.max_attitude_std) ||
      !Positive(config_.history_duration) || config_.max_history_states < 2 ||
      config_.stationary_accel_tolerance >= config_.gravity ||
      config_.initial_velocity_std > config_.max_velocity_std ||
      config_.initial_attitude_std > config_.max_attitude_std) {
    return {Reason::CONFIG_INVALID, "Invalid local estimator configuration"};
  }
  if (config_.enable_zero_velocity_update &&
      !Positive(config_.zero_velocity_variance)) {
    return {Reason::CONFIG_INVALID,
            "Zero-velocity update needs a positive calibrated variance"};
  }
  if (config_.diagnostic_lidar_tracking_gain && !config_.enable_lidar)
    return {Reason::CONFIG_INVALID, "Diagnostic LiDAR gain requires LiDAR"};
  if (config_.enable_lidar &&
      (config_.lidar_frame.empty() || config_.lidar_calibration_id.empty() ||
       !Positive(config_.lidar_point_noise_std) ||
       !Positive(config_.lidar_plane_noise_std) ||
       !Positive(config_.lidar_voxel_size) || config_.lidar_max_voxels == 0 ||
       config_.lidar_max_scans == 0 || config_.lidar_min_points_per_plane < 3 ||
       config_.lidar_min_correspondences == 0 ||
       config_.lidar_max_points < config_.lidar_min_correspondences ||
       !Positive(config_.lidar_max_correspondence_distance) ||
       !Positive(config_.lidar_observed_singular_value) ||
       !Positive(config_.lidar_innovation_gate) ||
       !Positive(config_.lidar_max_scan_age) ||
       !Nonnegative(config_.lidar_future_tolerance) ||
       config_.lidar_time_tolerance != 0.0 ||
       !Positive(config_.lidar_max_translation_correction_rate) ||
       !Positive(config_.lidar_max_rotation_correction_rate) ||
       !Positive(config_.lidar_max_velocity_correction_acceleration) ||
       !Positive(config_.lidar_anchor_variance_scale) ||
       !Positive(config_.lidar_correction_variance_scale) ||
       !Positive(config_.lidar_max_translation_correction) ||
       !Positive(config_.lidar_max_rotation_correction) ||
       !Positive(config_.lidar_max_velocity_correction) ||
       config_.lidar_max_update_iterations == 0 ||
       !Nonnegative(config_.lidar_process_noise_psd_tolerance) ||
       !Positive(config_.lidar_rotation_length))) {
    return {Reason::CONFIG_INVALID,
            "LiDAR needs explicit calibration, noise, storage and dynamics "
            "budgets"};
  }
  if (config_.enable_lidar &&
      (!std::isfinite(config_.lidar_point_noise_std *
                      config_.lidar_point_noise_std) ||
       !std::isfinite(config_.lidar_plane_noise_std *
                      config_.lidar_plane_noise_std) ||
       !std::isfinite(config_.lidar_voxel_size * config_.lidar_voxel_size))) {
    return {Reason::CONFIG_INVALID,
            "LiDAR noise or geometry scale is unrepresentable"};
  }
  const double standard_deviations[] = {
      config_.gyro_noise,
      config_.accel_noise,
      config_.gyro_bias_noise,
      config_.accel_bias_noise,
      config_.initial_velocity_std,
      config_.initial_attitude_std,
      config_.initial_gyro_bias_std,
      config_.initial_accel_bias_std,
      config_.max_position_std,
      config_.max_velocity_std,
      config_.max_attitude_std,
      std::min(1e-6, config_.max_position_std * 0.5)};
  for (double value : standard_deviations) {
    if (!std::isfinite(value * value) ||
        (value > 0.0 && value * value == 0.0)) {
      return {Reason::CONFIG_INVALID, "Unrepresentable covariance scale"};
    }
  }
  return {};
}

Result LocalEstimator::Reject(Reason reason, const std::string& message,
                              SourceCounters* counters) {
  ++counters->rejected;
  counters->reason = reason;
  if (reason == Reason::DUPLICATE) {
    ++counters->duplicates;
  } else if (reason == Reason::TIMESTAMP_REGRESSION) {
    ++counters->regressions;
  }
  return {reason, message};
}

Result LocalEstimator::AddWheel(const WheelSample& sample) {
  Result result = ValidateConfig();
  if (result.ok()) {
    result = CheckStamp(sample.stamp, config_, wheel_health_,
                        last_wheel_sequence_, Reason::WHEEL_STALE);
  }
  if (!result.ok()) {
    return Reject(result.reason, result.message, &wheel_health_);
  }
  if (!std::isfinite(sample.speed) || !Positive(sample.variance)) {
    return Reject(Reason::INVALID_INPUT, "Invalid wheel speed or variance",
                  &wheel_health_);
  }
  if (sample.stamp.receive_time - sample.stamp.time > config_.wheel_timeout) {
    return Reject(Reason::WHEEL_STALE, "Wheel sample exceeded wheel freshness",
                  &wheel_health_);
  }
  // Only the next forward IMU interval is supported, not delayed replay.
  if (has_imu_ && sample.stamp.time < previous_imu_.stamp.time) {
    return Reject(Reason::HISTORY_UNAVAILABLE,
                  "Wheel sample predates the committed IMU frontier",
                  &wheel_health_);
  }
  if (has_imu_ && !faulted_ &&
      sample.stamp.time - previous_imu_.stamp.time > config_.max_imu_gap) {
    return Reject(Reason::INVALID_INPUT,
                  "Wheel sample exceeds the next bounded IMU interval",
                  &wheel_health_);
  }
  if (pending_wheels_.size() >= kMaxPendingWheelSamples) {
    ++wheel_health_.evictions;
    return Reject(Reason::INVALID_INPUT,
                  "Pending wheel queue exceeded its fixed capacity",
                  &wheel_health_);
  }
  pending_wheels_.push_back(sample);
  last_received_wheel_ = sample;
  has_received_wheel_ = true;
  last_wheel_sequence_ = sample.stamp.sequence;
  Accept(sample.stamp, &wheel_health_);
  return {};
}

Result LocalEstimator::AddImu(const ImuSample& sample) {
  Result result = ValidateConfig();
  if (result.ok()) {
    result = CheckStamp(sample.stamp, config_, imu_health_, last_imu_sequence_,
                        Reason::IMU_STALE);
  }
  if (!result.ok()) {
    return Reject(result.reason, result.message, &imu_health_);
  }
  if (!sample.acceleration.allFinite() ||
      !sample.angular_velocity.allFinite() ||
      !std::isfinite(sample.acceleration.norm()) ||
      !std::isfinite(sample.angular_velocity.norm())) {
    return Reject(Reason::INVALID_INPUT, "Nonfinite IMU measurement",
                  &imu_health_);
  }
  const bool gap = has_imu_ && sample.stamp.time - previous_imu_.stamp.time >
                                   config_.max_imu_gap;
  const ImuSample interval_start = previous_imu_;
  const bool had_imu = has_imu_;
  if (gap) {
    faulted_ = true;
    joint_error_.Fail({Reason::IMU_GAP,
                       "Native joint stochastic propagation cannot cross an IMU gap"});
    has_motion_ = false;
    motion_status_ = {Reason::IMU_GAP, "Motion cannot span an IMU gap"};
  }
  ConsumeStationaryWheels(sample.stamp.time);
  if (!initialized_ || faulted_) {
    const WheelSample* stationary_wheel =
        has_stationary_wheel_ ? &stationary_wheel_ : nullptr;
    stationarity_.Observe(
        sample, stationary_wheel,
        !initialized_ && config_.allow_imu_only_initialization, gap);
  }
  previous_imu_ = sample;
  has_imu_ = true;
  last_imu_sequence_ = sample.stamp.sequence;
  Accept(sample.stamp, &imu_health_);
  if (!initialized_ && config_.enable_motion_initialization) {
    if (gap) initialization_imu_.clear();
    initialization_imu_.push_back(sample);
    while (initialization_imu_.size() > config_.max_history_states ||
           (initialization_imu_.size() > 2 &&
            sample.stamp.time - initialization_imu_[1].stamp.time > 10.0)) {
      initialization_imu_.pop_front();
    }
  }

  if (gap) {
    state_.valid = false;
    output_state_.valid = false;
    imu_health_.reason = Reason::IMU_GAP;
    return {Reason::IMU_GAP, "IMU gap requires explicit stopped reset"};
  }
  if (faulted_) {
    imu_health_.reason = Reason::IMU_GAP;
    return {Reason::IMU_GAP, "Only stopped reset can recover the IMU fault"};
  }
  if (!initialized_) {
    if (!stationarity_.Qualified(sample.stamp.time)) {
      state_.stamp = sample.stamp;
      state_.valid = false;
      output_state_.stamp = sample.stamp;
      output_state_.valid = false;
      RecordHistory();
      return {};
    }
    result = Initialize(sample);
    if (!result.ok()) {
      stationarity_.Reset();
      return Reject(result.reason, result.message, &imu_health_);
    }
  } else if (had_imu) {
    const LocalState start_state = state_;
    const LocalState start_output = output_state_;
    LocalState working = state_;
    if (joint_error_.active())
      joint_error_.Copy("previous_internal", "internal");
    joint_error_.BeginImu(interval_start, sample);
    Matrix15d error_transition = Matrix15d::Identity();
    Matrix15d process_covariance = Matrix15d::Zero();
    std::vector<SourceSampleId> sources{{"imu", interval_start.stamp.sequence},
                                        {"imu", sample.stamp.sequence}};
    sources.insert(sources.end(), pending_lidar_sources_.begin(),
                   pending_lidar_sources_.end());
    ImuSample cursor = interval_start;
    Result wheel_result;
    while (!pending_wheels_.empty() &&
           pending_wheels_.front().stamp.time <= sample.stamp.time) {
      const WheelSample wheel = pending_wheels_.front();
      pending_wheels_.pop_front();
      const ImuSample at_wheel =
          InterpolateImu(interval_start, sample, wheel.stamp.time);
      result = Propagate(cursor, at_wheel, &working, &error_transition,
                         &joint_error_, &process_covariance);
      if (!result.ok()) {
        faulted_ = true;
        state_.valid = false;
        has_motion_ = false;
        motion_status_ = result;
        imu_health_.reason = result.reason;
        return result;
      }
      cursor = at_wheel;
      if (sample.stamp.receive_time - wheel.stamp.time >
          config_.wheel_timeout) {
        result = Reject(Reason::WHEEL_STALE,
                        "Buffered wheel exceeded the causal freshness window",
                        &wheel_health_);
      } else if (!config_.use_wheel_velocity_update) {
        result = {};
      } else if (!working.covariance_model_valid) {
        result = Reject(Reason::CORRELATION_UNQUALIFIED,
                        "Wheel update rejected because the local covariance "
                        "model is invalid",
                        &wheel_health_);
      } else {
        result = UpdateWheel(wheel, at_wheel, &working, &error_transition,
                             &process_covariance);
        if (result.ok()) {
          sources.push_back({"wheel", wheel.stamp.sequence});
        }
      }
      if (!result.ok() && wheel_result.ok()) {
        wheel_result = result;
      }
    }
    result = Propagate(cursor, sample, &working, &error_transition, &joint_error_,
                       &process_covariance);
    if (!result.ok()) {
      faulted_ = true;
      state_.valid = false;
      has_motion_ = false;
      motion_status_ = result;
      imu_health_.reason = result.reason;
      return result;
    }
    const WheelSample* stationary_wheel =
        has_trusted_wheel_ ? &last_trusted_wheel_ : nullptr;
    stationarity_.Observe(sample,
                          wheel_result.ok() ? stationary_wheel : nullptr, false,
                          !wheel_result.ok());
    Result stationary_result;
    if (config_.enable_zero_velocity_update &&
        stationarity_.Qualified(sample.stamp.time)) {
      stationary_result =
          UpdateZeroVelocity(sample, &working, &error_transition,
                             &process_covariance);
      if (stationary_result.ok()) {
        sources.push_back({"zero_velocity", sample.stamp.sequence});
      }
    }
    working.stamp = sample.stamp;
    working.angular_velocity = sample.angular_velocity - working.gyro_bias;
    if (config_.diagnostic_lidar_tracking_gain) {
      lidar_gain_covariance_ =
          (error_transition * lidar_gain_covariance_ * error_transition.transpose() +
           process_covariance).eval();
      lidar_gain_covariance_ =
          (0.5 * (lidar_gain_covariance_ + lidar_gain_covariance_.transpose())).eval();
    }
    state_ = working;
    state_.valid = true;
    if (lio_active_) {
      const Result output = continuous_output_.Advance(
          start_state, state_, error_transition,
          sample.stamp.time - start_state.stamp.time, &process_covariance);
      if (!output.ok()) {
        state_.valid = false;
        faulted_ = true;
        return output;
      }
      output_state_ = continuous_output_.state();
      if (joint_error_.active()) {
        ContinuousOutputErrorTransition transition;
        const auto linearized =
            continuous_output_.LinearizeLastStep(&transition);
        if (linearized.ok()) joint_error_.AdvanceOutput(transition);
        else joint_error_.Fail(linearized);
      }
    } else {
      output_state_ = state_;
      continuous_output_.Reset(output_state_);
      if (joint_error_.active()) {
        joint_error_.CopyInternalToOutput();
      }
    }
    state_.valid = EvaluateIntegrity(sample.stamp.receive_time).ok();
    output_state_.valid = state_.valid;
    if (!start_output.covariance_model_valid ||
        !output_state_.covariance_model_valid) {
      has_motion_ = false;
      motion_status_ = {
          Reason::CORRELATION_UNQUALIFIED,
          "Local covariance model is invalid for joint motion export"};
    } else {
      motion_status_ = ComputeMotionIncrement(
          start_output, output_state_,
          start_output.covariance * error_transition.transpose(), sources,
          &last_motion_);
      has_motion_ = motion_status_.ok();
    }
    RecordHistory(error_transition, sources,
                  start_output.covariance_model_valid &&
                      output_state_.covariance_model_valid);
    RecordInternalHistory();
    SyncJointMemory();
    joint_error_.Publish(output_state_, sources);
    pending_lidar_sources_.clear();
    if (!wheel_result.ok()) {
      return wheel_result;
    }
    if (!stationary_result.ok()) {
      return stationary_result;
    }
    return Evaluate(sample.stamp.receive_time);
  }

  state_.valid = initialized_;
  output_state_ = state_;
  continuous_output_.Reset(output_state_);
  state_.valid = EvaluateIntegrity(sample.stamp.receive_time).ok();
  output_state_.valid = state_.valid;
  RecordHistory();
  RecordInternalHistory();
  SyncJointMemory();
  joint_error_.Publish(output_state_, {});
  return Evaluate(sample.stamp.receive_time);
}

void LocalEstimator::ConsumeStationaryWheels(double time) {
  if (initialized_ && !faulted_) {
    return;
  }
  while (!pending_wheels_.empty() &&
         pending_wheels_.front().stamp.time <= time) {
    stationary_wheel_ = pending_wheels_.front();
    has_stationary_wheel_ = true;
    pending_wheels_.pop_front();
  }
}

Result LocalEstimator::Initialize(const ImuSample& sample) {
  LocalState initial;
  initial.epoch = state_.epoch;
  initial.stamp = sample.stamp;
  const Eigen::Vector3d acceleration = stationarity_.mean_acceleration();
  // Gravity constrains tilt only. Yaw is a local gauge, not an absolute
  // heading.
  const double roll = std::atan2(acceleration.y(), acceleration.z());
  const double pitch = std::atan2(
      -acceleration.x(), std::hypot(acceleration.y(), acceleration.z()));
  initial.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  initial.gyro_bias = stationarity_.mean_angular_velocity();
  initial.angular_velocity = sample.angular_velocity - initial.gyro_bias;
  // Position is an arbitrary origin. A positive gauge covariance keeps P SPD.
  const double gauge_std = std::min(1e-6, config_.max_position_std * 0.5);
  initial.covariance.diagonal().segment<3>(0).setConstant(gauge_std *
                                                          gauge_std);
  initial.covariance.diagonal().segment<3>(3).setConstant(
      config_.initial_velocity_std * config_.initial_velocity_std);
  initial.covariance.diagonal().segment<3>(6).setConstant(
      config_.initial_attitude_std * config_.initial_attitude_std);
  initial.covariance.diagonal().segment<3>(9).setConstant(
      config_.initial_gyro_bias_std * config_.initial_gyro_bias_std);
  initial.covariance.diagonal().segment<3>(12).setConstant(
      config_.initial_accel_bias_std * config_.initial_accel_bias_std);
  if (!FiniteState(initial)) {
    return {Reason::INVALID_INPUT,
            "Stationary initialization is not finite SPD"};
  }
  state_ = initial;
  lidar_gain_covariance_ = initial.covariance;
  output_state_ = initial;
  continuous_output_.Reset(output_state_);
  initialized_ = true;
  joint_error_.InitializeStationary(
      initial, acceleration, stationarity_.sample_count(),
      stationarity_.last_mean_sequence() == sample.stamp.sequence);
  if (has_stationary_wheel_) {
    last_trusted_wheel_ = stationary_wheel_;
    has_trusted_wheel_ = true;
  }
  return {};
}

Result LocalEstimator::InitializeMotion(
    const std::vector<RelativeLidarPose>& relative_poses,
    MotionInitializationDiagnostics* diagnostics,
    const RelativeBootstrapEvidence* evidence) {
  if (diagnostics) *diagnostics = {};
  if (!config_.enable_motion_initialization || !has_imu_ || initialized_ ||
      faulted_) {
    return {
        Reason::INVALID_INPUT,
        "Motion initialization requires an enabled, uninitialized IMU owner"};
  }
  if (relative_poses.empty() ||
      relative_poses.back().time != previous_imu_.stamp.time) {
    return {Reason::HISTORY_UNAVAILABLE,
            "Motion initialization must commit at the actual IMU frontier"};
  }
  const auto valid = ValidateConfig();
  if (!valid.ok()) return valid;
  MotionInitializationConfig policy;
  policy.gravity = config_.gravity;
  policy.maximum_imu_gap = config_.max_imu_gap;
  policy.allow_lidar_stationary_initialization =
      config_.enable_lidar_stationary_initialization;
  policy.maximum_stationary_speed = config_.stationary_speed;
  policy.maximum_stationary_angular_speed = config_.stationary_gyro;
  MotionInitialization aligned;
  const std::vector<ImuSample> imu(initialization_imu_.begin(),
                                   initialization_imu_.end());
  const auto result =
      InitializeFromRelativeMotion(policy, relative_poses, imu, &aligned,
                                   diagnostics);
  if (!result.ok()) return result;
  LocalState initial;
  initial.epoch = state_.epoch;
  initial.stamp = previous_imu_.stamp;
  initial.orientation = aligned.orientation;
  initial.velocity = aligned.velocity;
  initial.gyro_bias = aligned.gyro_bias;
  initial.accel_bias = aligned.accel_bias;
  initial.angular_velocity = previous_imu_.angular_velocity - initial.gyro_bias;
  const double gauge_std = std::min(1e-6, config_.max_position_std * 0.5);
  initial.covariance.diagonal().segment<3>(0).setConstant(gauge_std *
                                                          gauge_std);
  initial.covariance.diagonal().segment<3>(3).setConstant(
      config_.initial_velocity_std * config_.initial_velocity_std);
  initial.covariance.diagonal().segment<3>(6).setConstant(
      config_.initial_attitude_std * config_.initial_attitude_std);
  initial.covariance.diagonal().segment<3>(9).setConstant(
      config_.initial_gyro_bias_std * config_.initial_gyro_bias_std);
  initial.covariance.diagonal().segment<3>(12).setConstant(
      config_.initial_accel_bias_std * config_.initial_accel_bias_std);
  initial.covariance_model_valid = false;
  initial.uncertainty_qualified = false;
  initial.valid = true;
  if (!FiniteState(initial))
    return {Reason::INVALID_INPUT,
            "Motion initialization produced invalid state"};
  state_ = initial;
  lidar_gain_covariance_ = initial.covariance;
  output_state_ = initial;
  continuous_output_.Reset(initial);
  initialized_ = true;
  if (config_.enable_native_joint_error && evidence) {
    Eigen::Matrix<double, 21, 21> covariance;
    const auto modeled = RelativeBootstrapCovariance(
        policy, config_, relative_poses, imu, aligned, *evidence, &covariance);
    if (modeled.ok()) joint_error_.InitializeRelative(initial, covariance);
    else joint_error_.RejectInitialization(modeled);
  } else {
    joint_error_.Initialize(initial);
  }
  history_.clear();
  internal_history_.clear();
  initialization_imu_.clear();
  has_motion_ = false;
  motion_status_ = {
      Reason::CORRELATION_UNQUALIFIED,
      aligned.lidar_stationary_initialization
          ? "LiDAR-rest startup assumes zero accel bias; uncertainty unqualified"
          : "Relative-motion startup has no qualified joint uncertainty model"};
  RecordHistory(Matrix15d::Identity(), {}, false);
  RecordInternalHistory();
  if (joint_error_.active()) {
    std::vector<SourceSampleId> sources;
    for (const auto& sample : imu) sources.push_back({"imu", sample.stamp.sequence});
    for (const auto& frame : evidence->frames)
      sources.push_back({"lidar_raw", frame.scan.stamp.sequence});
    SyncJointMemory();
    joint_error_.Publish(output_state_, sources);
  }
  return {};
}

Result LocalEstimator::Propagate(const ImuSample& start, const ImuSample& end,
                                 LocalState* state,
                                 Matrix15d* error_transition,
                                 NativeJointError* observer,
                                 Matrix15d* process_covariance) const {
  // Inputs are already in body axes at the motion reference; the adapter owns
  // sensor rotation and lever-arm compensation.
  const double dt = end.stamp.time - start.stamp.time;
  if (dt < 0.0 || dt > config_.max_imu_gap) {
    return {Reason::INVALID_INPUT, "Invalid bounded IMU propagation interval"};
  }
  if (dt == 0.0) {
    state->angular_velocity = end.angular_velocity - state->gyro_bias;
    return {};
  }
  const Eigen::Vector3d acceleration =
      (0.5 * (start.acceleration + end.acceleration)).eval() -
      state->accel_bias;
  const Eigen::Vector3d rotation =
      (0.5 * (start.angular_velocity + end.angular_velocity)).eval() -
      state->gyro_bias;
  const Eigen::Quaterniond midpoint =
      (state->orientation * ExpRotation(rotation * (0.5 * dt))).normalized();
  const Eigen::Matrix3d body_to_local = midpoint.toRotationMatrix();
  const Eigen::Vector3d local_acceleration =
      (body_to_local * acceleration).eval() -
      Eigen::Vector3d(0.0, 0.0, config_.gravity);
  const LocalState state_before = *state;
  LocalState next = *state;
  next.stamp = end.stamp;
  next.position += state->velocity * dt + 0.5 * local_acceleration * dt * dt;
  next.velocity += local_acceleration * dt;
  next.orientation =
      (state->orientation * ExpRotation(rotation * dt)).normalized();
  next.angular_velocity = end.angular_velocity - state->gyro_bias;

  // Right-multiplicative body-frame attitude error: [p, v, theta, bg, ba].
  Matrix15d dynamics = Matrix15d::Zero();
  dynamics.block<3, 3>(0, 3).setIdentity();
  dynamics.block<3, 3>(3, 6) = -body_to_local * Skew(acceleration);
  dynamics.block<3, 3>(3, 12) = -body_to_local;
  dynamics.block<3, 3>(6, 6) = -Skew(rotation);
  dynamics.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity();
  const Matrix15d squared = dynamics * dynamics;
  const Matrix15d transition =
      Matrix15d::Identity() + dynamics * dt + 0.5 * squared * dt * dt;
  const Matrix15d half_transition = Matrix15d::Identity() +
                                    dynamics * (0.5 * dt) +
                                    squared * (0.125 * dt * dt);
  Matrix15d noise = Matrix15d::Zero();
  noise.block<3, 3>(3, 3) = body_to_local * body_to_local.transpose() *
                            (config_.accel_noise * config_.accel_noise);
  noise.block<3, 3>(6, 6).diagonal().setConstant(config_.gyro_noise *
                                                 config_.gyro_noise);
  noise.block<3, 3>(9, 9).diagonal().setConstant(config_.gyro_bias_noise *
                                                 config_.gyro_bias_noise);
  noise.block<3, 3>(12, 12).diagonal().setConstant(config_.accel_bias_noise *
                                                   config_.accel_bias_noise);
  // Simpson quadrature of Phi(t) Qc Phi(t)^T, with second-order Phi samples.
  const Matrix15d discrete_noise =
      (dt / 6.0) *
      (noise + 4.0 * half_transition * noise * half_transition.transpose() +
       transition * noise * transition.transpose());
  next.covariance =
      transition * state->covariance * transition.transpose() + discrete_noise;
  next.covariance =
      (0.5 * (next.covariance + next.covariance.transpose())).eval();
  if (state->covariance_square_root || discrete_noise.isZero(0)) {
    Matrix15d continuous_root = Matrix15d::Zero();
    continuous_root.block<3, 3>(3, 3) = body_to_local * config_.accel_noise;
    continuous_root.block<3, 3>(6, 6).diagonal().setConstant(
        config_.gyro_noise);
    continuous_root.block<3, 3>(9, 9).diagonal().setConstant(
        config_.gyro_bias_noise);
    continuous_root.block<3, 3>(12, 12).diagonal().setConstant(
        config_.accel_bias_noise);
    Eigen::Matrix<double, 15, 45> additive_root;
    additive_root << continuous_root, 2 * half_transition * continuous_root,
        transition * continuous_root;
    additive_root *= std::sqrt(dt / 6);
    const auto modeled =
        AdvanceCovarianceSquareRoot(*state, transition, additive_root, &next);
    if (!modeled.ok()) return modeled;
  }
  if (!FiniteState(next)) {
    return {Reason::INVALID_INPUT, "IMU propagation is not finite SPD"};
  }
  *state = next;
  if (process_covariance) {
    *process_covariance =
        (transition * *process_covariance * transition.transpose() +
         discrete_noise).eval();
  }
  if (observer && observer->active()) {
    const Eigen::Vector3d phi = rotation * dt;
    const Eigen::Matrix3d half = ExpRotation(0.5 * phi).toRotationMatrix();
    const Eigen::Matrix3d acceleration_attitude =
        -body_to_local * Skew(acceleration) * half.transpose();
    const Eigen::Matrix3d acceleration_gyro =
        -body_to_local * Skew(acceleration) *
        RightRotationJacobian(0.5 * phi) * (0.5 * dt);
    Matrix15d exact = Matrix15d::Identity();
    exact.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();
    exact.block<3, 3>(0, 6) = 0.5 * dt * dt * acceleration_attitude;
    exact.block<3, 3>(3, 6) = dt * acceleration_attitude;
    exact.block<3, 3>(0, 9) = -0.5 * dt * dt * acceleration_gyro;
    exact.block<3, 3>(3, 9) = -dt * acceleration_gyro;
    exact.block<3, 3>(0, 12) = -0.5 * dt * dt * body_to_local;
    exact.block<3, 3>(3, 12) = -dt * body_to_local;
    exact.block<3, 3>(6, 6) = ExpRotation(-phi).toRotationMatrix();
    exact.block<3, 3>(6, 9) = -dt * RightRotationJacobian(phi);
    Eigen::Matrix<double, 15, 6> input =
        Eigen::Matrix<double, 15, 6>::Zero();
    input.block<3, 3>(0, 0) = 0.25 * dt * dt * body_to_local;
    input.block<3, 3>(3, 0) = 0.5 * dt * body_to_local;
    input.block<3, 3>(0, 3) = 0.25 * dt * dt * acceleration_gyro;
    input.block<3, 3>(3, 3) = 0.5 * dt * acceleration_gyro;
    input.block<3, 3>(6, 3) = 0.5 * dt * RightRotationJacobian(phi);
    observer->AdvanceImu(exact, input, input, start.stamp.time, end.stamp.time);
  }
  *error_transition = (transition * *error_transition).eval();
  if (trace_callback_) {
    LocalEstimatorTrace trace;
    trace.kind = LocalEstimatorTraceKind::IMU_PROPAGATION;
    trace.stamp = end.stamp;
    trace.dt = dt;
    trace.specific_force_body = acceleration;
    trace.angular_rate_body = rotation;
    trace.linear_acceleration_local = local_acceleration;
    trace.position_before = state_before.position;
    trace.position_after = state->position;
    trace.velocity_before = state_before.velocity;
    trace.velocity_after = state->velocity;
    trace.velocity_correction = state->velocity - state_before.velocity;
    trace.velocity_y_covariance_row_before =
        state_before.covariance.row(4).transpose();
    trace.velocity_y_covariance_row_after =
        state->covariance.row(4).transpose();
    trace.error_transition = transition;
    const Matrix15d propagated_covariance =
        transition * state_before.covariance * transition.transpose();
    trace.covariance_prediction_delta =
        propagated_covariance - state_before.covariance;
    trace.covariance_after = state->covariance;
    trace.propagated_covariance_y_row =
        propagated_covariance.row(4).transpose();
    trace.process_noise_covariance = discrete_noise;
    std::array<Matrix15d, 4> noise_sources{Matrix15d::Zero(), Matrix15d::Zero(),
                                           Matrix15d::Zero(),
                                           Matrix15d::Zero()};
    noise_sources[0].block<3, 3>(3, 3) =
        body_to_local * body_to_local.transpose() *
        (config_.accel_noise * config_.accel_noise);
    noise_sources[1].block<3, 3>(6, 6).diagonal().setConstant(
        config_.gyro_noise * config_.gyro_noise);
    noise_sources[2].block<3, 3>(9, 9).diagonal().setConstant(
        config_.gyro_bias_noise * config_.gyro_bias_noise);
    noise_sources[3].block<3, 3>(12, 12).diagonal().setConstant(
        config_.accel_bias_noise * config_.accel_bias_noise);
    for (std::size_t source = 0; source < noise_sources.size(); ++source) {
      const Matrix15d source_noise = DiscretizeProcessNoise(
          noise_sources[source], dt, transition, half_transition);
      trace.process_noise_velocity_y_by_source.row(
          static_cast<Eigen::Index>(source)) = source_noise.row(4);
    }
    trace.orientation_before = state_before.orientation;
    trace.orientation_midpoint = midpoint;
    trace.orientation_after = state->orientation;
    trace.gyro_bias_before = state_before.gyro_bias;
    trace.gyro_bias_after = state->gyro_bias;
    trace.accel_bias_before = state_before.accel_bias;
    trace.accel_bias_after = state->accel_bias;
    trace_callback_(trace);
  }
  return {};
}

Result LocalEstimator::UpdateWheel(const WheelSample& sample,
                                   const ImuSample& imu, LocalState* state,
                                   Matrix15d* error_transition,
                                   Matrix15d* process_covariance) {
  const int dimension = config_.use_nonholonomic_constraint ? 3 : 1;
  const Eigen::Matrix3d local_to_body =
      state->orientation.toRotationMatrix().transpose();
  const Eigen::Vector3d body_velocity =
      (local_to_body * state->velocity).eval();
  const LocalState state_before = *state;
  Eigen::MatrixXd observation = Eigen::MatrixXd::Zero(dimension, 15);
  observation.block(0, 3, dimension, 3) = local_to_body.topRows(dimension);
  observation.block(0, 6, dimension, 3) =
      Skew(body_velocity).topRows(dimension);
  Eigen::VectorXd innovation = -body_velocity.head(dimension);
  innovation(0) += sample.speed;
  Eigen::MatrixXd measurement_noise =
      Eigen::MatrixXd::Identity(dimension, dimension) *
      config_.nonholonomic_variance;
  measurement_noise(0, 0) = sample.variance;
  const Eigen::MatrixXd innovation_covariance =
      observation * state->covariance * observation.transpose() +
      measurement_noise;
  Eigen::LLT<Eigen::MatrixXd> factor(innovation_covariance);
  if (factor.info() != Eigen::Success) {
    return Reject(Reason::INVALID_INPUT,
                  "Wheel innovation covariance is invalid", &wheel_health_);
  }
  const double distance = innovation.dot(factor.solve(innovation));
  if (!std::isfinite(distance) || distance > config_.innovation_gate) {
    return Reject(Reason::INNOVATION_REJECTED, "Wheel innovation exceeded gate",
                  &wheel_health_);
  }
  const Eigen::MatrixXd unconstrained_gain =
      factor.solve(observation * state->covariance).transpose();
  Eigen::MatrixXd gain = unconstrained_gain;
  // A constrained gain preserves the local pose exactly, including orientation.
  gain.block(0, 0, 3, dimension).setZero();
  gain.block(6, 0, 3, dimension).setZero();
  if (!wheel_updates_gyro_bias_) {
    gain.block(9, 0, 3, dimension).setZero();
  }
  if (!wheel_updates_accel_bias_) {
    gain.block(12, 0, 3, dimension).setZero();
  }
  if (wheel_forward_velocity_only_) {
    const Eigen::Vector3d forward = local_to_body.row(0).transpose();
    gain.block(3, 0, 3, dimension) =
        (forward * forward.transpose() * gain.block(3, 0, 3, dimension)).eval();
  }
  const Eigen::Matrix<double, 15, 1> correction = gain * innovation;
  LocalState next = *state;
  next.velocity += correction.segment<3>(3);
  next.gyro_bias += correction.segment<3>(9);
  next.accel_bias += correction.segment<3>(12);
  next.angular_velocity = imu.angular_velocity - next.gyro_bias;
  const Matrix15d residual = Matrix15d::Identity() - gain * observation;
  next.covariance = residual * state->covariance * residual.transpose() +
                    gain * measurement_noise * gain.transpose();
  next.covariance =
      (0.5 * (next.covariance + next.covariance.transpose())).eval();
  if (state->covariance_square_root) {
    const auto modeled = AdvanceCovarianceSquareRoot(
        *state, residual,
        gain * measurement_noise.llt().matrixL().toDenseMatrix(), &next);
    if (!modeled.ok())
      return Reject(modeled.reason, modeled.message, &wheel_health_);
  }
  if (!FiniteState(next)) {
    return Reject(Reason::INVALID_INPUT, "Wheel update is not finite SPD",
                  &wheel_health_);
  }
  *state = next;
  if (process_covariance) {
    *process_covariance =
        (residual * *process_covariance * residual.transpose() +
         gain * measurement_noise * gain.transpose()).eval();
  }
  joint_error_.AdvanceInternal(
      residual, gain * measurement_noise * gain.transpose());
  *error_transition = (residual * *error_transition).eval();
  if (trace_callback_) {
    LocalEstimatorTrace trace;
    trace.kind = LocalEstimatorTraceKind::WHEEL_UPDATE;
    trace.stamp = sample.stamp;
    trace.dt = sample.stamp.time - imu.stamp.time;
    trace.position_before = state_before.position;
    trace.position_after = state->position;
    trace.velocity_before = state_before.velocity;
    trace.velocity_after = state->velocity;
    trace.gyro_bias_before = state_before.gyro_bias;
    trace.gyro_bias_after = state->gyro_bias;
    trace.accel_bias_before = state_before.accel_bias;
    trace.accel_bias_after = state->accel_bias;
    trace.orientation_before = state_before.orientation;
    trace.orientation_midpoint = state_before.orientation;
    trace.orientation_after = state->orientation;
    trace.wheel_body_velocity_before = body_velocity;
    trace.wheel_body_velocity_after =
        (state->orientation.toRotationMatrix().transpose() * state->velocity)
            .eval();
    trace.wheel_innovation.head(dimension) = innovation;
    trace.wheel_observation.topRows(dimension) = observation;
    trace.wheel_gain.leftCols(dimension) = gain;
    trace.unconstrained_wheel_gain.leftCols(dimension) = unconstrained_gain;
    trace.wheel_innovation_covariance.topLeftCorner(dimension, dimension) =
        innovation_covariance;
    trace.covariance_update_delta = state->covariance - state_before.covariance;
    trace.covariance_after = state->covariance;
    const Eigen::VectorXd weighted_innovation = factor.solve(innovation);
    for (int block = 0; block < 5; ++block) {
      const Eigen::RowVectorXd cross_covariance =
          state_before.covariance.row(4).segment(3 * block, 3) *
          observation.block(0, 3 * block, dimension, 3).transpose();
      trace.wheel_lateral_cross_covariance_by_state_block.col(block).head(
          dimension) = cross_covariance.transpose();
      trace.wheel_lateral_correction_by_state_block(block) =
          cross_covariance.dot(weighted_innovation);
    }
    trace.velocity_y_covariance_row_before =
        state_before.covariance.row(4).transpose();
    trace.velocity_y_covariance_row_after =
        state->covariance.row(4).transpose();
    trace.velocity_correction = state->velocity - state_before.velocity;
    trace.attitude_error_correction =
        LogRotation(state_before.orientation.conjugate() * state->orientation);
    trace.gyro_bias_correction = state->gyro_bias - state_before.gyro_bias;
    trace.accel_bias_correction = state->accel_bias - state_before.accel_bias;
    trace.wheel_speed = sample.speed;
    trace.wheel_innovation_distance = distance;
    trace_callback_(trace);
  }
  last_trusted_wheel_ = sample;
  has_trusted_wheel_ = true;
  wheel_health_.reason = Reason::NONE;
  return {};
}

Result LocalEstimator::UpdateZeroVelocity(const ImuSample& imu,
                                          LocalState* state,
                                          Matrix15d* error_transition,
                                          Matrix15d* process_covariance) {
  Eigen::Matrix<double, 3, 15> observation =
      Eigen::Matrix<double, 3, 15>::Zero();
  observation.block<3, 3>(0, 3).setIdentity();
  const Eigen::Vector3d innovation = -state->velocity;
  const Eigen::Matrix3d measurement_noise =
      Eigen::Matrix3d::Identity() * config_.zero_velocity_variance;
  const Eigen::Matrix3d innovation_covariance =
      observation * state->covariance * observation.transpose() +
      measurement_noise;
  Eigen::LLT<Eigen::Matrix3d> factor(innovation_covariance);
  if (factor.info() != Eigen::Success) {
    return Reject(Reason::INVALID_INPUT,
                  "Zero-velocity innovation covariance is invalid",
                  &zupt_health_);
  }
  const double distance = innovation.dot(factor.solve(innovation));
  if (!std::isfinite(distance) || distance > config_.innovation_gate) {
    return Reject(Reason::INNOVATION_REJECTED,
                  "Zero-velocity innovation exceeded gate", &zupt_health_);
  }
  Eigen::Matrix<double, 15, 3> gain =
      factor.solve(observation * state->covariance).transpose();
  gain.block<3, 3>(0, 0).setZero();
  gain.block<3, 3>(6, 0).setZero();
  const Eigen::Matrix<double, 15, 1> correction = gain * innovation;
  LocalState next = *state;
  next.velocity += correction.segment<3>(3);
  next.gyro_bias += correction.segment<3>(9);
  next.accel_bias += correction.segment<3>(12);
  next.angular_velocity = imu.angular_velocity - next.gyro_bias;
  const Matrix15d residual = Matrix15d::Identity() - gain * observation;
  next.covariance = residual * state->covariance * residual.transpose() +
                    gain * measurement_noise * gain.transpose();
  next.covariance =
      (0.5 * (next.covariance + next.covariance.transpose())).eval();
  if (state->covariance_square_root) {
    const auto modeled = AdvanceCovarianceSquareRoot(
        *state, residual,
        gain * measurement_noise.llt().matrixL().toDenseMatrix(), &next);
    if (!modeled.ok())
      return Reject(modeled.reason, modeled.message, &zupt_health_);
  }
  if (!FiniteState(next)) {
    return Reject(Reason::INVALID_INPUT,
                  "Zero-velocity update is not finite SPD", &zupt_health_);
  }
  *state = next;
  if (process_covariance) {
    *process_covariance =
        (residual * *process_covariance * residual.transpose() +
         gain * measurement_noise * gain.transpose()).eval();
  }
  joint_error_.AdvanceInternal(
      residual, gain * measurement_noise * gain.transpose());
  *error_transition = (residual * *error_transition).eval();
  Accept(imu.stamp, &zupt_health_);
  return {};
}

Result LocalEstimator::EvaluateIntegrity(double now) const {
  const Result config = ValidateConfig();
  if (!config.ok()) {
    return config;
  }
  if (!Positive(now)) {
    return {Reason::INVALID_INPUT, "Evaluation time is invalid"};
  }
  if (faulted_) {
    return {Reason::IMU_GAP, "Explicit stopped reset is required"};
  }
  if (!has_imu_) {
    return {Reason::WAITING_FOR_IMU, "No actual IMU evidence"};
  }
  if (!initialized_) {
    return {Reason::WAITING_FOR_STATIONARY,
            "Stationary alignment is incomplete"};
  }
  if (previous_imu_.stamp.clock_id != config_.clock_id ||
      state_.stamp.clock_id != config_.clock_id) {
    return {Reason::CLOCK_INVALID, "State evidence clock does not match"};
  }
  if (previous_imu_.stamp.time - now > config_.future_tolerance ||
      previous_imu_.stamp.receive_time - now > config_.future_tolerance) {
    return {Reason::INVALID_INPUT, "Evaluation precedes available evidence"};
  }
  if (now - previous_imu_.stamp.time > config_.max_sample_age) {
    return {Reason::IMU_STALE, "IMU evidence expired"};
  }
  if (state_.stamp.time != previous_imu_.stamp.time ||
      state_.stamp.sequence != previous_imu_.stamp.sequence ||
      output_state_.stamp.time != previous_imu_.stamp.time ||
      output_state_.stamp.sequence != previous_imu_.stamp.sequence) {
    return {Reason::INVALID_INPUT,
            "State is not committed at the IMU frontier"};
  }
  if (!FiniteState(state_) || !FiniteState(output_state_)) {
    return {Reason::INVALID_INPUT, "State or full error covariance is invalid"};
  }
  return {};
}

Result LocalEstimator::Evaluate(double now) const {
  const Result integrity = EvaluateIntegrity(now);
  if (!integrity.ok()) {
    return integrity;
  }
  if (lio_active_ && continuous_output_.correction_budget_exceeded()) {
    return continuous_output_.correction_status();
  }
  if (!output_state_.covariance_model_valid) {
    return {Reason::CORRELATION_UNQUALIFIED,
            "Local covariance does not follow a qualified declared model"};
  }
  // IMU and wheel dead reckoning preserves a continuous local gauge, but
  // cannot guarantee long-duration position or heading accuracy by itself.
  if (!has_trusted_wheel_ ||
      last_trusted_wheel_.stamp.time - now > config_.future_tolerance ||
      last_trusted_wheel_.stamp.receive_time - now > config_.future_tolerance) {
    return {Reason::WHEEL_STALE, "No trusted causal wheel evidence"};
  }
  if (now - last_trusted_wheel_.stamp.time > config_.wheel_timeout) {
    return {Reason::WHEEL_STALE, "No fresh causal wheel evidence"};
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (std::sqrt(output_state_.covariance(axis, axis)) >
            config_.max_position_std ||
        std::sqrt(output_state_.covariance(axis + 3, axis + 3)) >
            config_.max_velocity_std ||
        std::sqrt(output_state_.covariance(axis + 6, axis + 6)) >
            config_.max_attitude_std) {
      return {Reason::COVARIANCE_EXCEEDED,
              "Per-axis local sigma budget exceeded"};
    }
  }
  return {};
}

Result LocalEstimator::ResetStopped(double now) {
  const Result config = ValidateConfig();
  if (!config.ok()) {
    return config;
  }
  if (!Positive(now)) {
    return {Reason::INVALID_INPUT, "Reset time is invalid"};
  }
  WheelSample reset_wheel = stationary_wheel_;
  bool has_reset_wheel = has_stationary_wheel_;
  if (has_received_wheel_ &&
      (!has_reset_wheel ||
       last_received_wheel_.stamp.time >= reset_wheel.stamp.time)) {
    reset_wheel = last_received_wheel_;
    has_reset_wheel = true;
  }
  if (!has_imu_ || !has_stationary_wheel_ ||
      !stationarity_.Qualified(previous_imu_.stamp.time) ||
      now - previous_imu_.stamp.time > config_.max_sample_age ||
      !has_reset_wheel ||
      now - reset_wheel.stamp.time > config_.wheel_timeout ||
      previous_imu_.stamp.time > now || reset_wheel.stamp.time > now ||
      previous_imu_.stamp.receive_time > now ||
      reset_wheel.stamp.receive_time > now ||
      std::abs(reset_wheel.speed) > config_.stationary_speed ||
      previous_imu_.angular_velocity.norm() > config_.stationary_gyro) {
    return {Reason::RESET_WHILE_MOVING,
            "Reset needs fresh independent stopped wheel and stationary IMU"};
  }
  if (state_.epoch.generation == std::numeric_limits<uint64_t>::max()) {
    return {Reason::EPOCH_MISMATCH, "Epoch generation is exhausted"};
  }
  const Epoch epoch{state_.epoch.session, state_.epoch.generation + 1};
  const Stamp reset_stamp = previous_imu_.stamp;
  state_ = LocalState();
  state_.epoch = epoch;
  state_.stamp = reset_stamp;
  output_state_ = state_;
  initialized_ = false;
  faulted_ = false;
  has_imu_ = false;
  has_received_wheel_ = false;
  has_stationary_wheel_ = false;
  has_trusted_wheel_ = false;
  previous_imu_ = ImuSample();
  last_received_wheel_ = WheelSample();
  stationary_wheel_ = WheelSample();
  last_trusted_wheel_ = WheelSample();
  stationarity_.Reset();
  pending_wheels_.clear();
  history_.clear();
  internal_history_.clear();
  initialization_imu_.clear();
  lio_.Reset(epoch);
  continuous_output_.Reset(output_state_);
  lio_active_ = false;
  joint_error_.Reset();
  joint_scans_.clear();
  pending_lidar_sources_.clear();
  has_motion_ = false;
  last_motion_ = MotionIncrement();
  motion_status_ = {Reason::HISTORY_UNAVAILABLE, "ODOM reset clears motion"};
  imu_health_.reason = Reason::NONE;
  wheel_health_.reason = Reason::NONE;
  return {};
}

void LocalEstimator::RecordHistory(const Matrix15d& transition,
                                   const std::vector<SourceSampleId>& sources,
                                   bool motion_qualified) {
  if (!state_.valid) {
    return;
  }
  history_.push_back({output_state_, transition, sources, motion_qualified});
  while (!history_.empty() &&
         (output_state_.stamp.time - history_.front().state.stamp.time >
              config_.history_duration ||
          history_.size() > config_.max_history_states)) {
    history_.pop_front();
  }
}

void LocalEstimator::RecordInternalHistory(bool lidar_correction) {
  if (!state_.valid) {
    return;
  }
  if (!internal_history_.empty() &&
      internal_history_.back().corrected.stamp.time == state_.stamp.time) {
    if (joint_error_.active() && lidar_correction &&
        !internal_history_.back().has_lidar_correction)
      joint_error_.Copy(
          NativeJointError::History(state_.stamp.sequence, true),
          NativeJointError::History(state_.stamp.sequence));
    internal_history_.back().corrected = state_;
    internal_history_.back().has_lidar_correction |= lidar_correction;
  } else {
    internal_history_.push_back({state_, state_});
  }
  if (joint_error_.active())
    joint_error_.Copy(NativeJointError::History(state_.stamp.sequence),
                     "internal");
  while (!internal_history_.empty() &&
         (state_.stamp.time - internal_history_.front().corrected.stamp.time >
              config_.history_duration ||
          internal_history_.size() > config_.max_history_states)) {
    internal_history_.pop_front();
  }
}

Result LocalEstimator::LookupInternal(
    double time, LocalState* state, InternalHistoryErrorMap* error_map) const {
  if (state == nullptr || !std::isfinite(time) || time <= 0.0 ||
      internal_history_.empty()) {
    return {Reason::HISTORY_UNAVAILABLE, "Invalid internal history request"};
  }
  const auto upper =
      std::lower_bound(internal_history_.begin(), internal_history_.end(), time,
                       [](const InternalHistoryEntry& item, double stamp) {
                         return item.corrected.stamp.time < stamp;
                       });
  if (upper != internal_history_.end() && upper->corrected.stamp.time == time) {
    if (error_map) {
      InternalHistoryErrorMap next;
      next.first = next.second = upper->corrected.stamp;
      next.first_error.setIdentity();
      *error_map = std::move(next);
    }
    *state = upper->corrected;
    return {};
  }
  if (upper == internal_history_.begin() || upper == internal_history_.end()) {
    return {Reason::HISTORY_UNAVAILABLE,
            "LiDAR point is outside bounded IMU history"};
  }
  // A same-time LiDAR correction is not motion over the preceding IMU
  // interval. Interpolate to its propagated endpoint, not across the jump.
  const auto& lower = std::prev(upper)->corrected;
  const auto& endpoint = upper->propagated;
  const double dt = endpoint.stamp.time - lower.stamp.time;
  if (dt <= 0.0 || dt > config_.max_imu_gap) {
    return {Reason::IMU_GAP, "LiDAR point brackets an IMU gap"};
  }
  const double alpha = (time - lower.stamp.time) / dt;
  LocalState interpolated = lower;
  interpolated.stamp.time = time;
  interpolated.position =
      (1.0 - alpha) * lower.position + alpha * endpoint.position;
  interpolated.velocity =
      (1.0 - alpha) * lower.velocity + alpha * endpoint.velocity;
  interpolated.orientation =
      lower.orientation.slerp(alpha, endpoint.orientation).normalized();
  interpolated.gyro_bias =
      (1.0 - alpha) * lower.gyro_bias + alpha * endpoint.gyro_bias;
  interpolated.accel_bias =
      (1.0 - alpha) * lower.accel_bias + alpha * endpoint.accel_bias;
  interpolated.angular_velocity =
      (1.0 - alpha) * lower.angular_velocity + alpha * endpoint.angular_velocity;
  interpolated.covariance =
      (1.0 - alpha) * lower.covariance + alpha * endpoint.covariance;
  interpolated.covariance_square_root.reset();
  interpolated.covariance_model_valid =
      lower.covariance_model_valid && endpoint.covariance_model_valid;
  if (error_map) {
    Eigen::Matrix3d first_rotation, second_rotation;
    const auto result = RotationInterpolationJacobians(
        lower.orientation, endpoint.orientation, alpha,
        &first_rotation, &second_rotation);
    if (!result.ok()) return result;
    InternalHistoryErrorMap map;
    map.first = lower.stamp;
    map.second = endpoint.stamp;
    map.second_propagated = upper->has_lidar_correction;
    map.first_error = (1.0 - alpha) * Matrix15d::Identity();
    map.second_error = alpha * Matrix15d::Identity();
    map.first_error.block<3, 3>(6, 6) = first_rotation;
    map.second_error.block<3, 3>(6, 6) = second_rotation;
    *error_map = std::move(map);
  }
  *state = interpolated;
  return {};
}

Result LocalEstimator::StoredGeometryErrorMap(
    LidarStoredGeometryErrorMap* error_map) const {
  if (!error_map)
    return {Reason::INVALID_INPUT, "Stored-geometry error destination is missing"};
  LidarStoredGeometryErrorMap next;
  const auto result = lio_.GeometryErrorMap(&next.planes, &next.anchors);
  if (!result.ok()) return result;
  next.epoch = state_.epoch;
  next.local_frontier = state_.stamp;
  next.lidar_revision = lidar_health_.accepted;
  *error_map = std::move(next);
  return {};
}

Result LocalEstimator::DeskewErrorMap(
    const LidarScan& scan, LidarDeskewErrorMap* error_map) const {
  if (!error_map || !config_.enable_lidar ||
      scan.points.size() > config_.lidar_max_points ||
      !Positive(scan.stamp.time) || !Positive(scan.stamp.receive_time) ||
      scan.stamp.sequence == 0)
    return {Reason::INVALID_INPUT, "Invalid bounded LiDAR error-map request"};
  if (!state_.valid || scan.epoch != state_.epoch)
    return {Reason::EPOCH_MISMATCH, "LiDAR error map requires the active local epoch"};
  if (scan.stamp.clock_id != config_.clock_id)
    return {Reason::CLOCK_INVALID, "LiDAR error map clock does not match"};
  if (scan.frame_id != config_.lidar_frame ||
      scan.calibration_id != config_.lidar_calibration_id)
    return {Reason::CONFIG_INVALID, "LiDAR error map calibration/frame does not match"};
  std::map<double, InternalHistoryErrorMap> interpolation;
  DeskewedScan deskewed;
  std::vector<DeskewPointError> direct;
  const auto result = DeskewLidarScan(
      scan,
      [&](double time, LocalState* state) {
        InternalHistoryErrorMap map;
        const auto status = LookupInternal(time, state, &map);
        if (status.ok()) interpolation[time] = std::move(map);
        return status;
      },
      &deskewed, &direct);
  if (!result.ok()) return result;
  LidarDeskewErrorMap next;
  next.epoch = scan.epoch;
  next.scan = scan.stamp;
  next.local_frontier = state_.stamp;
  next.lidar_revision = lidar_health_.accepted;
  next.points.reserve(scan.points.size());
  std::map<std::pair<double, bool>, std::size_t> indices;
  const auto add_loading =
      [&](const Stamp& stamp, bool propagated,
          const Eigen::Matrix<double, 3, 15>& jacobian,
          LidarDeskewPointError* point) -> Result {
    if (jacobian.isZero(0)) return {};
    const auto key = std::make_pair(stamp.time, propagated);
    auto found = indices.find(key);
    if (found == indices.end()) {
      const auto history = std::lower_bound(
          internal_history_.begin(), internal_history_.end(), stamp.time,
          [](const InternalHistoryEntry& entry, double time) {
            return entry.corrected.stamp.time < time;
          });
      if (history == internal_history_.end() ||
          history->corrected.stamp.time != stamp.time)
        return {Reason::HISTORY_UNAVAILABLE,
                "LiDAR error map lost its exact retained history clone"};
      found = indices.emplace(key, next.history.size()).first;
      next.history.push_back(
          {propagated ? history->propagated : history->corrected, propagated});
    }
    const auto loading = std::find_if(
        point->history.begin(), point->history.end(),
        [&](const LidarDeskewHistoryLoading& item) {
          return item.history_index == found->second;
        });
    if (loading == point->history.end())
      point->history.push_back({found->second, jacobian});
    else
      loading->jacobian += jacobian;
    return {};
  };
  for (std::size_t index = 0; index < direct.size(); ++index) {
    LidarDeskewPointError point;
    point.time = direct[index].point_time;
    point.point_in_end_base = deskewed.points_in_end_base[index];
    point.raw_point = direct[index].raw_point;
    const auto compose = [&](double time,
                             const Eigen::Matrix<double, 3, 15>& jacobian) {
      const auto& map = interpolation.at(time);
      auto status = add_loading(map.first, false,
                                jacobian * map.first_error, &point);
      if (!status.ok()) return status;
      return add_loading(map.second, map.second_propagated,
                         jacobian * map.second_error, &point);
    };
    auto status = compose(point.time, direct[index].point_state);
    if (!status.ok()) return status;
    status = compose(scan.stamp.time, direct[index].end_state);
    if (!status.ok()) return status;
    next.points.push_back(std::move(point));
  }
  *error_map = std::move(next);
  return {};
}

Result LocalEstimator::AddLidar(const LidarScan& scan) {
  if (!config_.enable_lidar) {
    return Reject(Reason::CONFIG_INVALID, "Local LiDAR is disabled",
                  &lidar_health_);
  }
  if (scan.stamp.clock_id != config_.clock_id || !Positive(scan.stamp.time) ||
      !Positive(scan.stamp.receive_time) || scan.stamp.sequence == 0) {
    return Reject(Reason::INVALID_INPUT, "Invalid LiDAR measurement stamp",
                  &lidar_health_);
  }
  if (scan.stamp.time - scan.stamp.receive_time >
      config_.lidar_future_tolerance) {
    return Reject(Reason::INVALID_INPUT, "LiDAR scan is in the future",
                  &lidar_health_);
  }
  if (scan.stamp.receive_time - scan.stamp.time > config_.lidar_max_scan_age) {
    return Reject(Reason::LIDAR_STALE,
                  "LiDAR scan exceeded the causal age budget", &lidar_health_);
  }
  if (lidar_health_.accepted != 0) {
    if (scan.stamp.sequence == lidar_health_.last_sequence ||
        scan.stamp.time == lidar_health_.last_measurement) {
      return Reject(Reason::DUPLICATE, "LiDAR scan frontier did not advance",
                    &lidar_health_);
    }
    if (scan.stamp.sequence < lidar_health_.last_sequence ||
        scan.stamp.time < lidar_health_.last_measurement ||
        scan.stamp.receive_time < lidar_health_.last_receive) {
      return Reject(Reason::TIMESTAMP_REGRESSION,
                    "LiDAR source time or sequence regressed", &lidar_health_);
    }
  }
  if (!state_.valid || scan.epoch != state_.epoch) {
    return Reject(Reason::EPOCH_MISMATCH,
                  "LiDAR scan does not match the active local epoch",
                  &lidar_health_);
  }
  if (scan.calibration_id != config_.lidar_calibration_id) {
    return Reject(Reason::CONFIG_INVALID,
                  "LiDAR calibration identity does not match", &lidar_health_);
  }
  if (scan.frame_id != config_.lidar_frame) {
    return Reject(Reason::CONFIG_INVALID, "LiDAR sensor frame does not match",
                  &lidar_health_);
  }
  if (scan.points.size() > config_.lidar_max_points) {
    return Reject(Reason::INVALID_INPUT,
                  "LiDAR scan exceeds the configured point bound",
                  &lidar_health_);
  }
  if (scan.stamp.time != state_.stamp.time) {
    return Reject(Reason::LIDAR_STALE,
                  "LiDAR correction requires the exact committed IMU frontier",
                  &lidar_health_);
  }
  std::optional<NativeJointError> joint_before;
  LidarDeskewErrorMap joint_deskew;
  std::vector<StoredPlaneError> joint_planes;
  if (joint_error_.active()) {
    joint_before = joint_error_;
    const auto prepared = PrepareJointLidar(scan, &joint_deskew, &joint_planes);
    if (!prepared.ok()) joint_error_.Fail(prepared);
  }
  Result result = lio_.Observe(
      scan,
      [this](double time, LocalState* item) {
        return LookupInternal(time, item);
      },
      &state_,
      [&](const LioUpdate& update, const LocalState& prior,
          const LocalState& before, const LocalState& after) {
        JointLidarIteration(update, prior, before, after,
                            joint_deskew, joint_planes);
      },
      config_.diagnostic_lidar_tracking_gain ? &lidar_gain_covariance_ : nullptr);
  if (!result.ok() && result.reason != Reason::GEOMETRY_UNAVAILABLE) {
    if (joint_before) joint_error_ = *joint_before;
    return Reject(result.reason, result.message, &lidar_health_);
  }
  const auto stored = lio_.geometry_source_sequences();
  if (std::find(stored.begin(), stored.end(), scan.stamp.sequence) != stored.end()) {
    StoreJointLidar(scan, joint_deskew);
    joint_error_.AddSource({"lidar_raw", scan.stamp.sequence});
  } else if (joint_before) {
    joint_error_ = *joint_before;
  }
  Accept(scan.stamp, &lidar_health_);
  if (result.ok()) {
    lio_active_ = true;
    state_.covariance_model_valid = false;
    output_state_.covariance_model_valid = false;
    has_motion_ = false;
    motion_status_ = {
        Reason::CORRELATION_UNQUALIFIED,
        "LIO correction invalidated the declared joint covariance model"};
    if (!history_.empty() &&
        history_.back().state.stamp.time == output_state_.stamp.time) {
      history_.back().state.covariance_model_valid = false;
      history_.back().motion_qualified = false;
    }
    pending_lidar_sources_.clear();
    for (uint64_t sequence : lio_.geometry_source_sequences()) {
      pending_lidar_sources_.push_back({"lidar_local_geometry", sequence});
    }
  } else {
    lidar_health_.reason = result.reason;
  }
  RecordInternalHistory(result.ok());
  SyncJointMemory();
  return result;
}

LocalEstimator::LidarTask::LidarTask(const LocalEstimator& owner,
                                   const LidarScan& scan)
    : scan_(scan),
      lidar_before_(owner.lidar_health_) {
  stochastic_fork_ = owner.joint_error_.PinFork();
  snapshot_ = std::make_unique<LocalEstimator>(owner);
  snapshot_->SetTraceCallback({});
}

Result LocalEstimator::LidarTask::Compute() {
  if (computed_ || committed_) {
    return {Reason::DUPLICATE, "LiDAR task can only be computed once"};
  }
  computed_ = true;
  decision_ = snapshot_->AddLidar(scan_);
  return decision_;
}

Result LocalEstimator::LidarTask::ReplayImu(const ImuSample& sample) {
  if (!computed_ || committed_ ||
      (!decision_.ok() && decision_.reason != Reason::GEOMETRY_UNAVAILABLE)) {
    return {Reason::INVALID_INPUT,
            "IMU catch-up requires an uncommitted accepted LiDAR task"};
  }
  return snapshot_->AddImu(sample);
}

Result LocalEstimator::PrepareStartupSnapshot(
    std::unique_ptr<LocalEstimator>* snapshot) const {
  if (!snapshot || *snapshot || !config_.enable_motion_initialization ||
      !has_imu_ || initialized_ || faulted_ || has_received_wheel_ ||
      !pending_wheels_.empty())
    return {Reason::INVALID_INPUT,
            "Startup snapshot requires an uninitialized IMU-only owner"};
  *snapshot = std::make_unique<LocalEstimator>(*this);
  (*snapshot)->SetTraceCallback({});
  return {};
}

Result LocalEstimator::CommitStartupSnapshot(const LocalEstimator& candidate) {
  if (candidate.owner_identity_ != owner_identity_ || initialized_ || faulted_ ||
      candidate.faulted_ || has_received_wheel_ || candidate.has_received_wheel_ ||
      !pending_wheels_.empty() || !candidate.pending_wheels_.empty())
    return {Reason::INVALID_INPUT, "Startup snapshot owner is retired or initialized"};
  if (candidate.state_.epoch != state_.epoch)
    return {Reason::EPOCH_MISMATCH, "Startup snapshot belongs to an old epoch"};
  const auto& live = previous_imu_;
  const auto& replayed = candidate.previous_imu_;
  if (imu_health_.accepted != candidate.imu_health_.accepted ||
      imu_health_.rejected != candidate.imu_health_.rejected ||
      live.stamp.time != replayed.stamp.time ||
      live.stamp.receive_time != replayed.stamp.receive_time ||
      live.stamp.sequence != replayed.stamp.sequence ||
      live.stamp.clock_id != replayed.stamp.clock_id ||
      (live.acceleration.array() != replayed.acceleration.array()).any() ||
      (live.angular_velocity.array() != replayed.angular_velocity.array()).any())
    return {Reason::HISTORY_UNAVAILABLE,
            "Startup snapshot has not replayed the exact accepted IMU frontier"};
  if (candidate.initialized_) {
    auto callback = trace_callback_;
    *this = candidate;
    trace_callback_ = std::move(callback);
  }
  return {};
}

Result LocalEstimator::PrepareLidarTask(
    const LidarScan& scan, std::unique_ptr<LidarTask>* task) const {
  if (task == nullptr || *task != nullptr) {
    return {Reason::INVALID_INPUT, "LiDAR task output must be empty"};
  }
  const Result integrity = EvaluateIntegrity(state_.stamp.receive_time);
  if (!integrity.ok()) {
    return integrity;
  }
  if (has_received_wheel_ || !pending_wheels_.empty()) {
    return {Reason::CORRELATION_UNQUALIFIED,
            "Asynchronous LiDAR has no wheel replay contract"};
  }
  if (!config_.enable_lidar || scan.epoch != state_.epoch ||
      scan.stamp.time != state_.stamp.time) {
    return {Reason::LIDAR_STALE,
            "LiDAR task needs the current enabled same-epoch IMU frontier"};
  }
  if (scan.points.empty() || scan.points.size() > config_.lidar_max_points) {
    return {Reason::INVALID_INPUT, "LiDAR task exceeds its point bounds"};
  }
  task->reset(new LidarTask(*this, scan));
  return {};
}

Result LocalEstimator::CommitLidarTask(LidarTask* task) {
  if (task == nullptr || !task->computed_ || task->committed_ ||
      task->snapshot_->owner_identity_ != owner_identity_) {
    return {Reason::INVALID_INPUT,
            "LiDAR task is incomplete, consumed, or belongs to another owner"};
  }
  if (!task->decision_.ok() &&
      task->decision_.reason != Reason::GEOMETRY_UNAVAILABLE) {
    return task->decision_;
  }
  const auto& candidate = *task->snapshot_;
  if (candidate.state_.epoch != state_.epoch) {
    return {Reason::EPOCH_MISMATCH, "LiDAR task belongs to an old local epoch"};
  }
  if (has_received_wheel_ || !pending_wheels_.empty() ||
      lidar_health_.accepted != task->lidar_before_.accepted ||
      lidar_health_.rejected != task->lidar_before_.rejected ||
      candidate.lidar_health_.accepted != task->lidar_before_.accepted + 1) {
    return {Reason::LIDAR_STALE,
            "LiDAR task crossed another local measurement update"};
  }
  const auto& live_imu = previous_imu_;
  const auto& replayed_imu = candidate.previous_imu_;
  if (candidate.state_.stamp.time != state_.stamp.time ||
      candidate.imu_health_.accepted != imu_health_.accepted ||
      replayed_imu.stamp.sequence != live_imu.stamp.sequence ||
      replayed_imu.stamp.time != live_imu.stamp.time ||
      replayed_imu.stamp.receive_time != live_imu.stamp.receive_time ||
      replayed_imu.stamp.clock_id != live_imu.stamp.clock_id ||
      (replayed_imu.acceleration.array() != live_imu.acceleration.array()).any() ||
      (replayed_imu.angular_velocity.array() !=
       live_imu.angular_velocity.array()).any()) {
    return {Reason::HISTORY_UNAVAILABLE,
            "LiDAR task has not replayed the exact live IMU frontier"};
  }
  if (state_.stamp.receive_time - task->scan_.stamp.time >
      config_.lidar_max_scan_age) {
    return {Reason::LIDAR_STALE, "Delayed LiDAR task exceeded its age budget"};
  }
  const Result live_integrity = EvaluateIntegrity(state_.stamp.receive_time);
  if (!live_integrity.ok()) {
    return live_integrity;
  }
  const Result candidate_integrity =
      candidate.EvaluateIntegrity(state_.stamp.receive_time);
  if (!candidate_integrity.ok()) {
    return candidate_integrity;
  }
  // Only the private internal trajectory/geometry is replaced. The output
  // smoother and all previously published pose/velocity samples remain live.
  state_ = candidate.state_;
  lidar_gain_covariance_ = candidate.lidar_gain_covariance_;
  lio_ = candidate.lio_;
  internal_history_ = candidate.internal_history_;
  joint_scans_ = candidate.joint_scans_;
  lidar_health_ = candidate.lidar_health_;
  lio_active_ = candidate.lio_active_;
  if (joint_error_.active()) {
    const auto spliced = joint_error_.SpliceCandidate(
        candidate.joint_error_, task->stochastic_fork_, task->scan_.stamp.sequence);
    if (!spliced.ok()) joint_error_.Fail(spliced);
  }
  if (lio_active_) {
    state_.covariance_model_valid = false;
    output_state_.covariance_model_valid = false;
    has_motion_ = false;
    motion_status_ = {
        Reason::CORRELATION_UNQUALIFIED,
        "Delayed LIO correction has no qualified joint output error model"};
    pending_lidar_sources_.clear();
    for (uint64_t sequence : lio_.geometry_source_sequences()) {
      pending_lidar_sources_.push_back({"lidar_local_geometry", sequence});
    }
    if (!history_.empty()) {
      history_.back().state.covariance_model_valid = false;
      history_.back().motion_qualified = false;
    }
  }
  task->committed_ = true;
  task->stochastic_fork_.reset();
  SyncJointMemory();
  return task->decision_;
}

Result LocalEstimator::Lookup(double time, LocalState* state) const {
  if (state == nullptr || !std::isfinite(time) || time <= 0.0) {
    return {Reason::INVALID_INPUT, "Invalid local history request"};
  }
  const auto found =
      std::lower_bound(history_.begin(), history_.end(), time,
                       [](const HistoryEntry& entry, double stamp) {
                         return entry.state.stamp.time < stamp;
                       });
  if (found == history_.end() || found->state.stamp.time != time) {
    return {Reason::HISTORY_UNAVAILABLE, "No exact committed local state"};
  }
  *state = found->state;
  return {};
}

Result LocalEstimator::MotionBetween(double start_time, double end_time,
                                     MotionIncrement* motion) const {
  if (motion == nullptr || !std::isfinite(start_time) ||
      !std::isfinite(end_time) || start_time <= 0.0 || end_time <= start_time) {
    return {Reason::INVALID_INPUT, "Invalid local motion interval"};
  }
  LocalState start;
  LocalState end;
  auto status = Lookup(start_time, &start);
  if (!status.ok()) {
    return status;
  }
  status = Lookup(end_time, &end);
  if (!status.ok()) {
    return status;
  }
  if (!start.covariance_model_valid || !end.covariance_model_valid) {
    return {Reason::CORRELATION_UNQUALIFIED,
            "Motion endpoint covariance model is invalid"};
  }
  Matrix15d transition = Matrix15d::Identity();
  std::set<std::pair<std::string, uint64_t>> identities;
  std::vector<SourceSampleId> sources;
  for (const auto& entry : history_) {
    if (entry.state.stamp.time <= start_time) {
      continue;
    }
    if (entry.state.stamp.time > end_time) {
      break;
    }
    if (!entry.motion_qualified) {
      return {Reason::CORRELATION_UNQUALIFIED,
              "Motion interval crosses an unqualified LiDAR output update"};
    }
    transition = (entry.transition * transition).eval();
    for (const auto& sample : entry.sources) {
      if (identities.emplace(sample.source, sample.sequence).second) {
        sources.push_back(sample);
      }
    }
  }
  return ComputeMotionIncrement(
      start, end, start.covariance * transition.transpose(), sources, motion);
}

}  // namespace unified
}  // namespace localization
}  // namespace openloc
