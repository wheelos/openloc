// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "gnss_quality_gate.h"

#include <cmath>

namespace openloc {
namespace localization {
namespace internal {
namespace {

GnssQualityDecision Rejected(const std::string& reason) {
  return {GnssQualityOutcome::kRejected,
          Status(StatusCode::kRejectedMeasurement, reason)};
}

GnssQualityDecision Invalid(const std::string& reason,
                            StatusCode code = StatusCode::kInvalidArgument) {
  return {GnssQualityOutcome::kInvalid, Status(code, reason)};
}

GnssQualityDecision Accepted() {
  return {GnssQualityOutcome::kAccepted, Status::Ok()};
}

}  // namespace

Status ValidateGnssQualityGatePolicy(const GnssQualityGatePolicy& policy) {
  if (policy.minimum_satellites == 0 || !std::isfinite(policy.maximum_hdop) ||
      policy.maximum_hdop <= 0.0 ||
      !std::isfinite(policy.maximum_differential_age_seconds) ||
      policy.maximum_differential_age_seconds <= 0.0 ||
      !std::isfinite(policy.maximum_horizontal_accuracy_m) ||
      policy.maximum_horizontal_accuracy_m <= 0.0 ||
      !std::isfinite(policy.maximum_heading_accuracy_rad) ||
      policy.maximum_heading_accuracy_rad <= 0.0 ||
      !std::isfinite(policy.maximum_baseline_length_error_m) ||
      policy.maximum_baseline_length_error_m < 0.0 ||
      policy.fixed_solution_stability_ns < 0 ||
      policy.maximum_fixed_solution_gap_ns <= 0) {
    return Status(StatusCode::kInvalidArgument,
                  "GNSS quality policy contains invalid limits");
  }
  return Status::Ok();
}

Status GnssQualityGate::Configure(const GnssQualityGatePolicy& policy) {
  Status status = ValidateGnssQualityGatePolicy(policy);
  if (!status.ok()) {
    return status;
  }
  policy_ = policy;
  configured_ = true;
  Reset();
  return Status::Ok();
}

GnssQualityDecision GnssQualityGate::CheckInput(
    const Timestamp& timestamp, const GnssReceiverQuality& receiver,
    const FixedSolutionTrack& track, bool allow_missing_accuracy,
    bool allow_missing_hdop) const {
  if (!configured_) {
    return Invalid("GNSS quality gate is not configured",
                   StatusCode::kStateUnavailable);
  }
  if (timestamp.nanoseconds <= 0 ||
      (timestamp.clock_domain != ClockDomain::kUnixEpoch &&
       timestamp.clock_domain != ClockDomain::kSensorMonotonic) ||
      (!(allow_missing_hdop && std::isnan(receiver.hdop)) &&
       (!std::isfinite(receiver.hdop) || receiver.hdop <= 0.0)) ||
      !std::isfinite(receiver.differential_age_seconds) ||
      receiver.differential_age_seconds < 0.0 ||
      (!(allow_missing_accuracy &&
         std::isnan(receiver.horizontal_accuracy_m)) &&
       (!std::isfinite(receiver.horizontal_accuracy_m) ||
        receiver.horizontal_accuracy_m <= 0.0))) {
    return Invalid("GNSS timestamp or receiver quality is invalid");
  }
  if (track.active) {
    if (timestamp.clock_domain != track.clock_domain) {
      return Invalid("GNSS dwell clock domain changed",
                     StatusCode::kClockDomainMismatch);
    }
    if (timestamp.nanoseconds <= track.last_ns) {
      return Invalid("GNSS dwell timestamp did not advance",
                     timestamp.nanoseconds == track.last_ns
                         ? StatusCode::kDuplicateTimestamp
                         : StatusCode::kOutOfOrderTimestamp);
    }
  }
  return Accepted();
}

GnssQualityDecision GnssQualityGate::CheckPosition(
    const Timestamp& timestamp, const GnssReceiverQuality& receiver) {
  auto decision =
      CheckInput(timestamp, receiver, position_track_,
                 policy_.allow_missing_horizontal_accuracy_diagnostic,
                 policy_.allow_missing_hdop);
  if (!decision.status.ok()) {
    return decision;
  }
  decision = CheckReceiver(receiver);
  if (!decision.status.ok()) {
    position_track_.active = false;
    return decision;
  }
  if (receiver.horizontal_accuracy_m > policy_.maximum_horizontal_accuracy_m) {
    position_track_.active = false;
    return Rejected("horizontal_accuracy_exceeds_limit");
  }
  return CheckStability(timestamp, &position_track_);
}

GnssQualityDecision GnssQualityGate::CheckHeading(
    const Timestamp& timestamp, const GnssReceiverQuality& receiver,
    double heading_accuracy_rad, double measured_baseline_length_m,
    double configured_baseline_length_m) {
  auto decision = CheckInput(timestamp, receiver, heading_track_);
  if (!decision.status.ok()) {
    return decision;
  }
  if (!std::isfinite(heading_accuracy_rad) || heading_accuracy_rad <= 0.0 ||
      !std::isfinite(measured_baseline_length_m) ||
      measured_baseline_length_m <= 0.0 ||
      !std::isfinite(configured_baseline_length_m) ||
      configured_baseline_length_m <= 0.0) {
    return Invalid("GNSS heading accuracy or baseline is invalid");
  }
  decision = CheckReceiver(receiver);
  if (!decision.status.ok()) {
    heading_track_.active = false;
    return decision;
  }
  if (heading_accuracy_rad > policy_.maximum_heading_accuracy_rad) {
    heading_track_.active = false;
    return Rejected("heading_accuracy_exceeds_limit");
  }
  if (std::abs(measured_baseline_length_m - configured_baseline_length_m) >
      policy_.maximum_baseline_length_error_m) {
    heading_track_.active = false;
    return Rejected("baseline_length_mismatch");
  }
  return CheckStability(timestamp, &heading_track_);
}

void GnssQualityGate::Reset() {
  position_track_ = FixedSolutionTrack{};
  heading_track_ = FixedSolutionTrack{};
}

GnssQualityDecision GnssQualityGate::CheckReceiver(
    const GnssReceiverQuality& receiver) const {
  if (receiver.carrier_solution != GnssCarrierSolution::kFixed) {
    return Rejected("carrier_solution_not_fixed");
  }
  if (receiver.satellite_count < policy_.minimum_satellites) {
    return Rejected("insufficient_satellites");
  }
  if (receiver.hdop >= policy_.maximum_hdop) {
    return Rejected("hdop_exceeds_limit");
  }
  if (receiver.differential_age_seconds >=
      policy_.maximum_differential_age_seconds) {
    return Rejected("differential_age_exceeds_limit");
  }
  return Accepted();
}

GnssQualityDecision GnssQualityGate::CheckStability(const Timestamp& timestamp,
                                                    FixedSolutionTrack* track) {
  if (!track->active || timestamp.nanoseconds - track->last_ns >
                            policy_.maximum_fixed_solution_gap_ns) {
    track->start_ns = timestamp.nanoseconds;
  }
  track->active = true;
  track->last_ns = timestamp.nanoseconds;
  track->clock_domain = timestamp.clock_domain;
  if (timestamp.nanoseconds - track->start_ns <
      policy_.fixed_solution_stability_ns) {
    return {
        GnssQualityOutcome::kWaitingForStability,
        Status(StatusCode::kRejectedMeasurement, "fixed_solution_not_stable")};
  }
  return Accepted();
}

}  // namespace internal
}  // namespace localization
}  // namespace openloc
