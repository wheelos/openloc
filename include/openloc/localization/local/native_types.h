// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace openloc {
namespace localization {
namespace unified {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix15d = Eigen::Matrix<double, 15, 15>;

enum class Reason {
  NONE,
  CONFIG_INVALID,
  WAITING_FOR_STATIONARY,
  WAITING_FOR_IMU,
  IMU_STALE,
  WHEEL_STALE,
  INVALID_INPUT,
  DUPLICATE,
  TIMESTAMP_REGRESSION,
  CLOCK_INVALID,
  IMU_GAP,
  COVARIANCE_EXCEEDED,
  INNOVATION_REJECTED,
  HISTORY_UNAVAILABLE,
  EPOCH_MISMATCH,
  RESET_WHILE_MOVING,
  GLOBAL_UNAVAILABLE,
  GLOBAL_STALE,
  MAP_MISMATCH,
  MAP_NOT_READY,
  MATCH_FAILED,
  DEGENERATE,
  RELOCALIZATION_AMBIGUOUS,
  RELOCALIZATION_VERIFYING,
  LIDAR_STALE,
  POINT_TIME_MISSING,
  GEOMETRY_UNAVAILABLE,
  CORRECTION_BUDGET_EXCEEDED,
  CORRELATION_UNQUALIFIED,
  UNCERTAINTY_UNQUALIFIED,
};

struct Result {
  Reason reason = Reason::NONE;
  std::string message;
  bool ok() const { return reason == Reason::NONE; }
};

// Immutable linear errors over shared primitive identities. Copies and affine
// combinations preserve identity; covariance queries never invent independence.
class JointErrorExpression {
 public:
  struct Term;
  JointErrorExpression() = default;
  static JointErrorExpression Primitive(
      const std::shared_ptr<const int>& identity, const Eigen::MatrixXd& root);
  static JointErrorExpression Combine(Eigen::Index rows,
                                      const std::vector<Term>& terms);
  Eigen::Index rows() const;
  bool valid() const { return static_cast<bool>(node_); }
  Result CrossCovariance(const JointErrorExpression& other,
                         Eigen::MatrixXd* covariance) const;
  Result Covariance(Eigen::MatrixXd* covariance) const {
    return CrossCovariance(*this, covariance);
  }

 private:
  struct Node;
  explicit JointErrorExpression(std::shared_ptr<const Node> node)
      : node_(std::move(node)) {}
  std::shared_ptr<const Node> node_;
};

struct JointErrorExpression::Term {
  Eigen::MatrixXd coefficient;
  JointErrorExpression error;
};

struct Stamp {
  double time = 0.0;
  double receive_time = 0.0;
  uint64_t sequence = 0;
  std::string clock_id;
};

struct Epoch {
  std::string session;
  uint64_t generation = 0;
  bool operator==(const Epoch& other) const {
    return session == other.session && generation == other.generation;
  }
  bool operator!=(const Epoch& other) const { return !(*this == other); }
};

struct ImuSample {
  Stamp stamp;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
};

struct WheelSample {
  Stamp stamp;
  // Signed base-forward velocity at the estimator motion reference, in m/s.
  // Adapters own encoder conversion, sign, frame, and lever-arm calibration.
  double speed = 0.0;
  // Variance of speed, in (m/s)^2.
  double variance = 0.0;
};

struct TimedLidarPoint {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double time = 0.0;
};

struct LidarScan {
  Stamp stamp;
  Epoch epoch;
  std::string frame_id;
  std::string calibration_id;
  Eigen::Isometry3d base_from_lidar = Eigen::Isometry3d::Identity();
  std::vector<TimedLidarPoint> points;
};

// A minimal full-rank basis for the producer's sufficient joint error state.
// The published 15D error is output_jacobian * joint_error. Extra memory must
// retain temporal/geometry reuse; positive covariance alone is not evidence.
struct JointLocalError {
  Eigen::MatrixXd covariance;
  Eigen::MatrixXd output_jacobian;
};

struct LocalState {
  Stamp stamp;
  Epoch epoch;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Matrix15d covariance = Matrix15d::Zero();
  std::optional<JointLocalError> joint_error;
  // True only while covariance/cross-time error claims follow the estimator's
  // declared error model. This is not deployment or accuracy qualification.
  bool covariance_model_valid = true;
  // True only when the local uncertainty model has been calibrated for use.
  bool uncertainty_qualified = false;
  bool valid = false;
  // Optional numerical representation of this SAME marginal, P = L L^T.
  // It is not a sufficient temporal basis or an uncertainty qualification.
  // covariance is its rounded dense view; consumers requiring a dense SPD
  // inverse must still check that their representation can resolve it.
  std::optional<Matrix15d> covariance_square_root;
};

struct SourceSampleId {
  std::string source;
  uint64_t sequence = 0;
};

// Actual pre-correction deskewed scan consumed by LIO. Point errors share the
// conditioning state's joint basis; no independent point noise may be added.
struct JointLidarScan {
  LocalState conditioning_state;
  Stamp reference;
  uint64_t lidar_revision = 0;
  std::vector<Eigen::Vector3d> points_in_reference_base;
  std::vector<Eigen::MatrixXd> point_jacobians;
};

// Source-aware query snapshot, deliberately not a finite Markov/sufficient
// dense joint state. Dense graph/transport APIs must not accept this type.
struct StructuredLocalState {
  LocalState state;
  JointErrorExpression output_error;
};

// Owner-local observations, not allocated-byte estimates or model qualification.
struct NativeStructuredResourceUsage {
  uint64_t primitive_sources = 0;
  uint64_t primitive_rows = 0;
  uint64_t memory_keys = 0;
  uint64_t snapshots = 0;
  uint64_t publish_queries = 0;
  double publish_query_ms = 0;
  uint64_t retired_registry_sources = 0;
  uint64_t retired_registry_rows = 0;
};

struct StructuredLidarScan {
  StructuredLocalState conditioning_state;
  Stamp reference;
  uint64_t lidar_revision = 0;
  std::vector<Eigen::Vector3d> points_in_reference_base;
  std::vector<JointErrorExpression> point_errors;
};

struct MotionIncrement {
  Epoch epoch;
  Stamp start;
  Stamp end;
  Eigen::Isometry3d start_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
  // [translation in start body, right attitude error in end body].
  Matrix6d covariance = Matrix6d::Zero();
  Matrix15d start_covariance = Matrix15d::Zero();
  Matrix15d end_covariance = Matrix15d::Zero();
  // E[error_start * error_end^T], with frozen, unsmoothed start state.
  Matrix15d cross_covariance = Matrix15d::Zero();
  Eigen::MatrixXd joint_cross_covariance;
  std::vector<SourceSampleId> sources;
  // Shared filter bias/history: not an independent between-factor covariance.
  std::string correlation_group;
};

struct ContinuousOutputErrorTransition {
  Epoch epoch;
  Stamp start;
  Stamp end;
  // Frozen inputs of the published step, not the owner's possibly newer
  // same-time internal state after a delayed LiDAR commit.
  LocalState previous_internal_state;
  LocalState current_internal_state;
  LocalState previous_output_state;
  Eigen::Vector3d previous_correction_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_correction_velocity = Eigen::Vector3d::Zero();
  // Output error order: [published 15D error, correction-velocity memory].
  // Input endpoint errors use the same right-attitude convention as LocalState.
  Eigen::Matrix<double, 18, 15> previous_internal =
      Eigen::Matrix<double, 18, 15>::Zero();
  Eigen::Matrix<double, 18, 15> current_internal =
      Eigen::Matrix<double, 18, 15>::Zero();
  Eigen::Matrix<double, 18, 18> previous_output =
      Eigen::Matrix<double, 18, 18>::Zero();
};

struct LidarDeskewHistoryState {
  LocalState state;
  bool before_lidar_correction = false;
};

struct LidarDeskewHistoryLoading {
  std::size_t history_index = 0;
  Eigen::Matrix<double, 3, 15> jacobian =
      Eigen::Matrix<double, 3, 15>::Zero();
};

struct LidarDeskewPointError {
  double time = 0;
  Eigen::Vector3d point_in_end_base = Eigen::Vector3d::Zero();
  Eigen::Matrix3d raw_point = Eigen::Matrix3d::Zero();
  std::vector<LidarDeskewHistoryLoading> history;
};

// One immutable owner snapshot's deterministic map, NOT a joint covariance.
// All point/history/raw-source cross terms must be retained by its caller.
struct LidarDeskewErrorMap {
  Epoch epoch;
  Stamp scan;
  Stamp local_frontier;
  uint64_t lidar_revision = 0;
  std::vector<LidarDeskewHistoryState> history;
  std::vector<LidarDeskewPointError> points;
};

// Read-only PRE-update query support, not a graph motion model or a retained
// corrected scan. Shared loadings use one unit-covariance native memory basis.
// Each raw point adds its own independent 3D error with raw_point_variance;
// reuse across queries must preserve (epoch, scan sequence, point index).
// Basis coordinates are snapshot-local; do not combine separate exports.
// Finite query covariance does not establish a sufficient temporal state.
struct JointLidarInputErrorMap {
  LidarDeskewErrorMap deskew;
  Eigen::MatrixXd internal_loading;
  Eigen::MatrixXd output_loading;
  std::vector<Eigen::MatrixXd> history_loadings;
  double raw_point_variance = 0.0;
};

struct StoredPlanePointError {
  uint64_t source_sequence = 0;
  std::size_t point_index = 0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  // Derivatives of plane center/normal w.r.t. this stored ODOM point.
  Eigen::Matrix3d center = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 3, 15> anchor_state =
      Eigen::Matrix<double, 3, 15>::Zero();
  Eigen::Matrix3d deskewed_point = Eigen::Matrix3d::Zero();
};

struct StoredPlaneError {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  std::vector<StoredPlanePointError> points;
};

// Derivatives of retained geometry only. Anchor, deskewed-point and raw-source
// errors are shared, not independent covariance contributions.
struct LidarStoredGeometryErrorMap {
  Epoch epoch;
  Stamp local_frontier;
  uint64_t lidar_revision = 0;
  std::map<uint64_t, LocalState> anchors;
  std::vector<StoredPlaneError> planes;
};

enum class LocalEstimatorTraceKind { IMU_PROPAGATION, WHEEL_UPDATE };

struct LocalEstimatorTrace {
  LocalEstimatorTraceKind kind = LocalEstimatorTraceKind::IMU_PROPAGATION;
  Stamp stamp;
  double dt = 0.0;
  Eigen::Vector3d specific_force_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_rate_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration_local = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_before = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_after = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_before = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_after = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_before = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond orientation_midpoint = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond orientation_after = Eigen::Quaterniond::Identity();
  Eigen::Vector3d wheel_body_velocity_before = Eigen::Vector3d::Zero();
  Eigen::Vector3d wheel_body_velocity_after = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_before = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_after = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_before = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_after = Eigen::Vector3d::Zero();
  Eigen::Vector3d attitude_error_correction = Eigen::Vector3d::Zero();
  Eigen::Vector3d wheel_innovation = Eigen::Vector3d::Zero();
  // Error-state order is [position, velocity, right attitude, gyro bias,
  // accelerometer bias].
  Eigen::Matrix<double, 3, 15> wheel_observation =
      Eigen::Matrix<double, 3, 15>::Zero();
  Eigen::Matrix<double, 15, 3> wheel_gain =
      Eigen::Matrix<double, 15, 3>::Zero();
  Eigen::Matrix<double, 15, 3> unconstrained_wheel_gain =
      Eigen::Matrix<double, 15, 3>::Zero();
  Eigen::Matrix3d wheel_innovation_covariance = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 15, 1> velocity_y_covariance_row_before =
      Eigen::Matrix<double, 15, 1>::Zero();
  Eigen::Matrix<double, 15, 1> velocity_y_covariance_row_after =
      Eigen::Matrix<double, 15, 1>::Zero();
  Eigen::Matrix<double, 3, 5> wheel_lateral_cross_covariance_by_state_block =
      Eigen::Matrix<double, 3, 5>::Zero();
  Eigen::Matrix<double, 5, 1> wheel_lateral_correction_by_state_block =
      Eigen::Matrix<double, 5, 1>::Zero();
  // The local-Y covariance row of FPF^T and the discretized Q sources.
  Matrix15d error_transition = Matrix15d::Identity();
  Matrix15d covariance_prediction_delta = Matrix15d::Zero();
  Matrix15d covariance_update_delta = Matrix15d::Zero();
  Matrix15d covariance_after = Matrix15d::Zero();
  Eigen::Matrix<double, 15, 1> propagated_covariance_y_row =
      Eigen::Matrix<double, 15, 1>::Zero();
  Matrix15d process_noise_covariance = Matrix15d::Zero();
  // Rows: accel white noise, gyro white noise, gyro-bias random walk,
  // accel-bias random walk.
  Eigen::Matrix<double, 4, 15> process_noise_velocity_y_by_source =
      Eigen::Matrix<double, 4, 15>::Zero();
  Eigen::Vector3d velocity_correction = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias_correction = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_correction = Eigen::Vector3d::Zero();
  double wheel_speed = 0.0;
  double wheel_innovation_distance = 0.0;
};

struct LocalConfig {
  std::string session;
  std::string clock_id = "unix";
  double gravity = 9.80665;
  double max_sample_age = 0.2;
  double future_tolerance = 0.02;
  double max_imu_gap = 0.05;
  double wheel_timeout = 0.2;
  double stationary_speed = 0.02;
  double stationary_gyro = 0.02;
  double stationary_accel_tolerance = 0.3;
  uint32_t stationary_samples = 50;
  double stationary_duration = 2.0;
  bool enable_zero_velocity_update = false;
  double zero_velocity_variance = 0.0;
  // Enables gravity/gyro-bias initialization from an IMU-only quiet window.
  // It does not establish zero translational velocity or motion confidence.
  bool allow_imu_only_initialization = false;
  // Relative scan/IMU alignment is an explicit alternative startup path.
  // It never establishes qualified local uncertainty or graph eligibility.
  bool enable_motion_initialization = false;
  bool enable_lidar_stationary_initialization = false;
  // Continuous-time white-noise spectral-density square roots, not per-sample
  // standard deviations. Propagation discretizes their PSDs over each dt.
  double gyro_noise = 0.01;
  double accel_noise = 0.1;
  double gyro_bias_noise = 0.0001;
  double accel_bias_noise = 0.001;
  double initial_velocity_std = 0.05;
  double initial_attitude_std = 0.02;
  double initial_gyro_bias_std = 0.01;
  double initial_accel_bias_std = 0.1;
  double nonholonomic_variance = 0.01;
  bool use_nonholonomic_constraint = false;
  // Diagnostic ablation only; keep enabled for normal wheel-aided operation.
  bool use_wheel_velocity_update = true;
  double innovation_gate = 11.345;
  double max_position_std = 1.0;
  double max_velocity_std = 0.5;
  double max_attitude_std = 0.1;
  double history_duration = 3.0;
  uint32_t max_history_states = 4096;
  // Opt-in, uncalibrated frozen-linearization stochastic observer. It never
  // replaces the nominal owner's diagnostic covariance or source decisions.
  bool enable_native_joint_error = false;
  // Exact source-aware native queries; dense graph export remains unsupported.
  // Requires enable_native_joint_error. Limits on dense matrices stay intact.
  // Primitive storage is separately bounded by the configured scan capacities
  // (3 * lidar_max_points * lidar_max_scans) and IMU history (12 *
  // max_history_states). Exhaustion rejects without retiring needed sources.
  bool enable_native_structured_error = false;
  uint32_t native_joint_max_dimension = 256;
  uint32_t native_joint_max_memory_rows = 4096;
  uint32_t native_joint_max_snapshots = 64;
  // Explicit sampled white-noise model: variance = configured noise PSD /
  // sample period. Actual sample identities are reused across both adjacent
  // trapezoids and any causal wheel subintervals. Zero leaves it unspecified.
  double native_imu_sample_period = 0.0;
  bool enable_lidar = false;
  // Diagnostic nominal-gain preconditioner only; never exported as uncertainty.
  bool diagnostic_lidar_tracking_gain = false;
  std::string lidar_frame;
  std::string lidar_calibration_id;
  double lidar_point_noise_std = 0.0;
  double lidar_plane_noise_std = 0.0;
  double lidar_voxel_size = 0.0;
  uint32_t lidar_max_voxels = 0;
  uint32_t lidar_max_scans = 0;
  uint32_t lidar_min_points_per_plane = 0;
  uint32_t lidar_min_correspondences = 0;
  uint32_t lidar_max_points = 0;
  double lidar_max_correspondence_distance = 0.0;
  double lidar_observed_singular_value = 0.0;
  double lidar_innovation_gate = 0.0;
  double lidar_max_scan_age = 0.0;
  double lidar_future_tolerance = 0.0;
  double lidar_time_tolerance = 0.0;
  double lidar_max_translation_correction_rate = 0.0;
  double lidar_max_rotation_correction_rate = 0.0;
  double lidar_max_velocity_correction_acceleration = 0.0;
  double lidar_anchor_variance_scale = 0.0;
  double lidar_correction_variance_scale = 0.0;
  double lidar_max_translation_correction = 0.0;
  double lidar_max_rotation_correction = 0.0;
  double lidar_max_velocity_correction = 0.0;
  uint32_t lidar_max_update_iterations = 0;
  double lidar_process_noise_psd_tolerance = 0.0;
  double lidar_rotation_length = 0.0;
};

struct SourceCounters {
  uint64_t accepted = 0;
  uint64_t rejected = 0;
  uint64_t duplicates = 0;
  uint64_t regressions = 0;
  uint64_t evictions = 0;
  double last_measurement = 0.0;
  double last_receive = 0.0;
  uint64_t last_sequence = 0;
  Reason reason = Reason::NONE;
};

Eigen::Isometry3d Pose(const LocalState& state);
Matrix6d PoseCovariance(const LocalState& state);
Eigen::Matrix3d Skew(const Eigen::Vector3d& value);
Eigen::Quaterniond ExpRotation(const Eigen::Vector3d& value);
Eigen::Vector3d LogRotation(const Eigen::Quaterniond& value);
bool ValidCovariance(const Matrix6d& covariance);
const char* ReasonName(Reason reason);

}  // namespace unified
}  // namespace localization
}  // namespace openloc
