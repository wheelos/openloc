// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

#include "openloc/localization/global/observation_types.h"
#include "openloc/localization/local/native_types.h"

namespace openloc {
namespace localization {

struct FixedLagAlignmentConfig {
  MapReference reference;
  std::string map_content_id;
  FrameId map_frame;
  FrameId odom_frame;
  FrameId base_frame;
  ClockDomain clock_domain = ClockDomain::kUnixEpoch;
  int64_t history_duration_ns = 3'000'000'000LL;
  int64_t maximum_observation_age_ns = 2'000'000'000LL;
  int64_t maximum_odometry_age_ns = 200'000'000LL;
  int64_t maximum_full_pose_age_ns = 2'000'000'000LL;
  double solve_budget_ms = 50.0;
  uint32_t maximum_states = 1000;
  uint32_t maximum_observations = 2000;
  uint32_t maximum_error_dimension = 256;
  std::size_t maximum_decisions = 1000;
  uint32_t verification_frames = 3;
  // An external qualification claim, not inferred from positive covariances,
  // a single cross-covariance, or provenance dependency declarations.
  std::string motion_error_model_id;
  bool motion_error_model_qualified = false;
  // Explicit functional use of declared sufficient stochastic models before
  // empirical calibration. Does not waive model validity, source quality,
  // conditional independence or reference identity. Never trusted output.
  bool allow_uncalibrated_inputs = false;
  // Keep an established graph available to partial observations after the
  // full-pose lease expires. Does not renew that lease or qualified trust;
  // explicit recovery, invalid motion/model and epoch changes still revoke it.
  bool retain_graph_on_full_pose_staleness = false;
  // Separate diagnostic XY/yaw acquisition, never SE(3) graph initialization.
  bool enable_heading_bootstrap = false;
  bool assume_gravity_aligned_odom = false;
};

struct GraphOdometrySample {
  OdometrySample odometry;
  Timestamp receive_time;
  uint64_t sequence = 0;
  // [position_odom, velocity_odom, right attitude_body, gyro bias, accel bias].
  Eigen::Matrix<double, 15, 15> covariance =
      Eigen::Matrix<double, 15, 15>::Zero();
  bool covariance_model_valid = false;
  bool uncertainty_qualified = false;
  // Optional sufficient joint state; its projected marginal must equal
  // covariance. Absent means the existing 15D identity projection.
  std::optional<unified::JointLocalError> joint_error;
};

struct GraphMotionSource {
  std::string source;
  uint64_t sequence = 0;
};

struct GraphMotionConnection {
  OdometryEpoch epoch;
  Timestamp start_time;
  Timestamp end_time;
  uint64_t start_sequence = 0;
  uint64_t end_sequence = 0;
  // Cov(e_previous, e_current), not a relative-pose marginal or its transpose.
  Eigen::Matrix<double, 15, 15> cross_covariance =
      Eigen::Matrix<double, 15, 15>::Zero();
  std::vector<GraphMotionSource> sources;
  // Cov(joint_previous, joint_current). Required if either endpoint has a
  // joint state, including entry/exit and dimension changes. Its projected
  // covariance must agree with cross_covariance, never replace it silently.
  Eigen::MatrixXd joint_cross_covariance;
};

struct GraphLocalObservationCorrelation {
  std::string motion_error_model_id;
  std::string conditional_independence_evidence_id;
  uint64_t odometry_sequence = 0;
  // Absent means the observation time. Delayed shared scans may condition on
  // a later retained error node without changing their measurement/pose time.
  std::optional<Timestamp> conditioning_time;
  // Cov(e, n), where n = measured - true in the observation residual chart.
  // Rows follow the retained joint error (15D without augmentation);
  // columns follow the measured subspace.
  Eigen::MatrixXd cross_covariance;
  // The retained error state must mediate ALL local/temporal/source reuse:
  // n = C^T P^-1 e + eta, with eta independent of the entire error chain,
  // previous observations and other source residuals, not just the same node.
  bool conditional_model_qualified = false;
  bool residual_independent_of_history_and_sources = false;
};

struct GraphSourceQualification {
  std::string uncertainty_model_id;
  // Unconditional claims apply only without local_correlation. Covers local
  // motion, temporal reuse and reuse across absolute sources.
  std::string independence_evidence_id;
  bool quality_valid = false;
  bool uncertainty_qualified = false;
  bool independent_of_local = false;
  bool independent_of_graph_sources = false;
  // Exclusive with unconditional independence. Covariance on the observation
  // remains marginal; the backend derives its conditional residual covariance.
  std::optional<GraphLocalObservationCorrelation> local_correlation;
};

struct FixedLagAlignmentEstimate {
  Pose map_from_odom;
  Pose map_from_base;
  // Backend alignment marginal/bound, never a held/published-pose covariance.
  Eigen::Matrix<double, 6, 6> alignment_covariance =
      Eigen::Matrix<double, 6, 6>::Zero();
  uint64_t correction_id = 0;
};

struct GraphObservationDecision {
  GlobalObservationDecision observation;
  GraphSourceQualification qualification;
};

struct GraphSourceStatistics {
  std::string source;
  uint64_t submitted = 0;
  uint64_t alignment_updates = 0;
  uint64_t retained = 0;
  uint64_t rejected = 0;
};

enum class LocalizationTrustLevel {
  kUntrusted,
  kDiagnostic,
  kQualified,
};

struct FixedLagAlignmentResult {
  Timestamp timestamp;
  Timestamp evaluation_time;
  OdometryEpoch epoch;
  MapReference reference;
  std::string map_content_id;
  FrameId map_frame;
  FrameId odom_frame;
  FrameId base_frame;
  std::string motion_error_model_id;
  LocalizationTrustLevel trust_level = LocalizationTrustLevel::kUntrusted;
  bool diagnostic_only = false;
  // Assessment is separate from retrieving a result. An estimate may be
  // retained while stale, degraded, or undergoing candidate verification.
  Status assessment = Status(StatusCode::kStateUnavailable,
                             "graph assessment has not been evaluated");
  bool verification_pending = false;
  bool odometry_fresh = false;
  bool full_pose_evidence_fresh = false;
  double last_full_pose_time_seconds = 0.0;
  uint64_t decisions_dropped = 0;
  uint64_t submitted_observations = 0;
  uint64_t rejected_observations = 0;
  // At most 32 named sources. Submissions beyond that bound are rejected and
  // remain in the aggregate counters and bounded decision journal.
  std::vector<GraphSourceStatistics> sources;
  std::optional<FixedLagAlignmentEstimate> estimate;
  std::optional<GlobalHorizontalEstimate> horizontal_estimate;
  bool horizontal_evidence_fresh = false;
};

/// Thin, synchronous facade over one existing SE(3) fixed-lag graph owner.
/// No diagnostic fallback or fusion with the separate 4DoF/CI backends.
/// The host serializes calls and schedules observations after their exact
/// ODOM node exists. No interpolation, implicit queue, or future-time seeding.
/// Input nanoseconds are preserved at the boundary; backend node keys still
/// use double seconds. Aliased node times are rejected; a measurement must
/// match the retained original integer timestamp, not just its rounded key.
/// Full poses alone can bootstrap/recover the SE(3) graph. XY/XYZ/heading factors neither
/// synthesize missing measurements nor renew the full-pose lease.
/// Source and motion qualifications must be established externally. Qualified
/// conditional source models can couple observations to its retained joint error;
/// this API does not prove sufficient-state claims, publish TF, or certify trust.
class FixedLagGlobalAlignment {
 public:
  FixedLagGlobalAlignment();
  ~FixedLagGlobalAlignment();
  FixedLagGlobalAlignment(const FixedLagGlobalAlignment&) = delete;
  FixedLagGlobalAlignment& operator=(const FixedLagGlobalAlignment&) = delete;

  Status Configure(const FixedLagAlignmentConfig& config);
  // No connection on the first sample of an epoch. Thereafter it must refer
  // to the previous accepted ODOM snapshot. Missing motion revokes alignment.
  Status AddOdometry(const GraphOdometrySample& sample,
                     const GraphMotionConnection* connection = nullptr);
  Status SubmitMapObservation(const GlobalMapObservation& observation,
                              const GraphSourceQualification& qualification);
  Status SubmitHorizontalObservation(
      const GlobalHorizontalObservation& observation,
      const GraphSourceQualification& qualification);
  Status SubmitPositionObservation(
      const GlobalPositionObservation& observation,
      const GraphSourceQualification& qualification);
  Status SubmitHeadingObservation(
      const GlobalHeadingObservation& observation,
      const GraphSourceQualification& qualification);
  Status GetResult(const Timestamp& evaluation_time,
                   FixedLagAlignmentResult* result) const;
  // Available before ODOM; includes adapter and owner rejections, qualification
  // declarations and actual correction revisions. Persists across ODOM epochs.
  Status GetDecisions(std::vector<GraphObservationDecision>* decisions) const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace localization
}  // namespace openloc
