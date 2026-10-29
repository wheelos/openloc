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

namespace openloc {
namespace localization {

struct DiagnosticAlignmentConfig {
  MapReference reference;
  std::string map_content_id;
  FrameId map_frame;
  FrameId odom_frame;
  FrameId base_frame;
  ClockDomain clock_domain = ClockDomain::kUnixEpoch;
  int64_t history_duration_ns = 3'000'000'000LL;
  int64_t maximum_interpolation_gap_ns = 50'000'000LL;
  int64_t maximum_observation_age_ns = 2'000'000'000LL;
  int64_t maximum_odometry_age_ns = 200'000'000LL;
  int64_t maximum_full_pose_age_ns = 2'000'000'000LL;
  std::size_t maximum_pending_observations = 128;
  std::size_t maximum_decisions = 1000;
  // Opt-in multi-position XY/yaw acquisition with later holdout validation.
  // Requires an explicit diagnostic assumption that ODOM and global XY are
  // gravity-level. Neither this declaration nor acquisition qualifies axes.
  bool enable_horizontal_bootstrap = false;
  bool assume_gravity_aligned_odom = false;
  // Allows a qualified-by-caller baseline azimuth plus XY/XYZ to acquire a
  // separate diagnostic horizontal result without motion or a full pose.
  bool enable_heading_bootstrap = false;
  // Explicit conservative comparison: fixed mean gain instead of CI gain.
  // Unqualified covariance bounds and the trust ceiling remain diagnostic.
  bool use_fixed_gain_update = false;
};

struct DiagnosticHorizontalEstimate {
  Eigen::Vector3d map_from_odom_xy_yaw = Eigen::Vector3d::Zero();
  Eigen::Vector2d map_base_xy = Eigen::Vector2d::Zero();
  // Parameter covariance at odom_reference_xy, not at the ODOM origin.
  // Explicitly uncalibrated; no height, roll/pitch or full-pose covariance.
  Eigen::Vector2d odom_reference_xy = Eigen::Vector2d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  Timestamp last_observation_time;
  uint64_t correction_id = 0;
};

struct DiagnosticAlignmentEstimate {
  Pose map_from_odom;
  Pose map_from_base;
  // Raw backend alignment covariance, not a qualified composed-pose bound.
  Eigen::Matrix<double, 6, 6> alignment_covariance =
      Eigen::Matrix<double, 6, 6>::Zero();
  uint64_t correction_id = 0;
};

struct DiagnosticAlignmentResult {
  Timestamp timestamp;
  Timestamp evaluation_time;
  OdometryEpoch epoch;
  MapReference reference;
  std::string map_content_id;
  FrameId map_frame;
  FrameId odom_frame;
  FrameId base_frame;
  std::size_t pending_observations = 0;
  bool verification_pending = false;
  bool odometry_fresh = false;
  bool full_pose_evidence_fresh = false;
  std::optional<Timestamp> last_full_pose_time;
  uint64_t decisions_dropped = 0;
  // Absent until distinct full-pose observations verify an alignment basin.
  // A position observation can be retained without creating an alignment.
  std::optional<DiagnosticAlignmentEstimate> estimate;
  // Separate partial result; never creates estimate or a full-pose lease.
  std::optional<DiagnosticHorizontalEstimate> horizontal_estimate;
  bool horizontal_evidence_fresh = false;
  std::vector<GlobalObservationDecision> decisions;
};

/// Public diagnostic facade over the existing SE(3) candidate owner.
/// No graph qualification, receiver gate, publication smoothing, TF publishing,
/// or trusted output. Do not combine with another global owner in one runtime.
/// Partial diagnostic updates are not a qualified joint covariance bound.
/// Callers serialize methods. Measurement/arrival remain exact in decisions;
/// the private history still uses double seconds and may reject close epochs.
class DiagnosticGlobalAlignment {
 public:
  DiagnosticGlobalAlignment();
  ~DiagnosticGlobalAlignment();
  DiagnosticGlobalAlignment(const DiagnosticGlobalAlignment&) = delete;
  DiagnosticGlobalAlignment& operator=(const DiagnosticGlobalAlignment&) =
      delete;

  Status Configure(const DiagnosticAlignmentConfig& config);
  /// Pose covariance is [translation_odom, right rotation_body]. It is an
  /// explicit uncalibrated diagnostic input, never a qualified motion model.
  Status AddOdometry(
      const OdometrySample& sample,
      const Eigen::Matrix<double, 6, 6>& diagnostic_pose_covariance);
  /// kPending means queued until ODOM brackets measurement time. Other
  /// rejects, including multi-frame verification, are recorded and not queued.
  Status SubmitMapObservation(const GlobalMapObservation& observation);
  Status SubmitHorizontalObservation(
      const GlobalHorizontalObservation& observation);
  Status SubmitHeadingObservation(const GlobalHeadingObservation& observation);
  Status GetResult(DiagnosticAlignmentResult* result) const;
  /// Evaluates freshness without extrapolating ODOM. Time must be in the
  /// configured clock and no earlier than the latest ODOM timestamp.
  /// A retained diagnostic estimate does not imply fresh evidence or trust.
  Status GetResult(const Timestamp& evaluation_time,
                   DiagnosticAlignmentResult* result) const;
  /// Available before the first ODOM sample so early rejects remain auditable.
  Status GetDecisions(std::vector<GlobalObservationDecision>* decisions) const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace localization
}  // namespace openloc
