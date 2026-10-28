// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <memory>

#include "openloc/localization/online/types.h"

namespace openloc::localization {

struct GnssPositionGatePolicy {
  uint32_t minimum_satellites = 6;
  double maximum_hdop = 3.0;
  double maximum_differential_age_seconds = 10.0;
  double maximum_horizontal_accuracy_m = 1.0;
  int64_t fixed_solution_stability_ns = 2'000'000'000LL;
  int64_t maximum_fixed_solution_gap_ns = 2'000'000'000LL;
  // NaN denotes absent receiver accuracy, never a covariance estimate.
  bool allow_missing_horizontal_accuracy_diagnostic = false;
  // NaN alone denotes an absent HDOP field (e.g. NovAtel BESTPOS).
  // Zero, negative and infinite HDOP remain invalid; other gates are unchanged.
  bool allow_missing_hdop = false;
};

// Receiver admission only, not another global estimator. Callers serialize.
// Reuses the same quality/dwell policy as the existing online engine.
class GnssPositionGate {
 public:
  GnssPositionGate();
  ~GnssPositionGate();
  GnssPositionGate(const GnssPositionGate&) = delete;
  GnssPositionGate& operator=(const GnssPositionGate&) = delete;
  Status Configure(const GnssPositionGatePolicy& policy);
  Status Check(const Timestamp& time, const GnssReceiverQuality& receiver);
  void Reset();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace openloc::localization
