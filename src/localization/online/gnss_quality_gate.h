// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#ifndef OPENLOC_LOCALIZATION_ONLINE_GNSS_QUALITY_GATE_H_
#define OPENLOC_LOCALIZATION_ONLINE_GNSS_QUALITY_GATE_H_

#include <string>

#include "openloc/localization/online/types.h"

namespace openloc {
namespace localization {
namespace internal {

struct GnssQualityGatePolicy {
  uint32_t minimum_satellites;
  double maximum_hdop;
  double maximum_differential_age_seconds;
  double maximum_horizontal_accuracy_m;
  double maximum_heading_accuracy_rad;
  double maximum_baseline_length_error_m;
  int64_t fixed_solution_stability_ns;
  int64_t maximum_fixed_solution_gap_ns;
  // Diagnostic position admission only; NaN denotes missing receiver accuracy,
  // never an estimated measurement covariance or qualified source.
  bool allow_missing_horizontal_accuracy_diagnostic = false;
  bool allow_missing_hdop = false;
};

Status ValidateGnssQualityGatePolicy(const GnssQualityGatePolicy& policy);

enum class GnssQualityOutcome {
  kAccepted,
  kWaitingForStability,
  kRejected,
  kInvalid,
};

struct GnssQualityDecision {
  GnssQualityOutcome outcome;
  Status status;
};

/// Applies shared receiver quality and per-measurement fixed-solution dwell.
/// Callers own frame checks, ODOM association, and measurement geometry.
class GnssQualityGate {
 public:
  /// Invalid configuration leaves the policy and dwell histories unchanged.
  Status Configure(const GnssQualityGatePolicy& policy);
  GnssQualityDecision CheckPosition(const Timestamp& timestamp,
                                    const GnssReceiverQuality& receiver);
  GnssQualityDecision CheckHeading(const Timestamp& timestamp,
                                   const GnssReceiverQuality& receiver,
                                   double heading_accuracy_rad,
                                   double measured_baseline_length_m,
                                   double configured_baseline_length_m);
  void Reset();

 private:
  struct FixedSolutionTrack {
    int64_t start_ns = 0;
    int64_t last_ns = 0;
    ClockDomain clock_domain = ClockDomain::kUnixEpoch;
    bool active = false;
  };

  GnssQualityDecision CheckInput(const Timestamp& timestamp,
                                 const GnssReceiverQuality& receiver,
                                 const FixedSolutionTrack& track,
                                 bool allow_missing_accuracy = false,
                                 bool allow_missing_hdop = false) const;
  GnssQualityDecision CheckReceiver(const GnssReceiverQuality& receiver) const;
  GnssQualityDecision CheckStability(const Timestamp& timestamp,
                                     FixedSolutionTrack* track);

  GnssQualityGatePolicy policy_;
  FixedSolutionTrack position_track_;
  FixedSolutionTrack heading_track_;
  bool configured_ = false;
};

}  // namespace internal
}  // namespace localization
}  // namespace openloc

#endif  // OPENLOC_LOCALIZATION_ONLINE_GNSS_QUALITY_GATE_H_
