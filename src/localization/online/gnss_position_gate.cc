// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "openloc/localization/online/gnss_position_gate.h"

#include "gnss_quality_gate.h"

namespace openloc::localization {

class GnssPositionGate::Impl {
 public:
  internal::GnssQualityGate gate;
};

GnssPositionGate::GnssPositionGate() : impl_(std::make_unique<Impl>()) {}
GnssPositionGate::~GnssPositionGate() = default;

Status GnssPositionGate::Configure(const GnssPositionGatePolicy& policy) {
  internal::GnssQualityGatePolicy shared{
      policy.minimum_satellites, policy.maximum_hdop,
      policy.maximum_differential_age_seconds,
      policy.maximum_horizontal_accuracy_m, 1.0, 0.0,
      policy.fixed_solution_stability_ns, policy.maximum_fixed_solution_gap_ns,
      policy.allow_missing_horizontal_accuracy_diagnostic,
      policy.allow_missing_hdop};
  return impl_->gate.Configure(shared);
}

Status GnssPositionGate::Check(const Timestamp& time,
                               const GnssReceiverQuality& receiver) {
  return impl_->gate.CheckPosition(time, receiver).status;
}

void GnssPositionGate::Reset() { impl_->gate.Reset(); }

}  // namespace openloc::localization
