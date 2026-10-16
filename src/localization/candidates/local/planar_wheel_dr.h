// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.
#pragma once

#include "candidates/common/types.h"

namespace openloc {
namespace localization {
namespace unified {

struct PlanarWheelInput {
  Stamp stamp;
  // Time-associated body velocity at the declared motion reference, m/s.
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  // Body-Z gyro rate, rad/s; caller qualifies tilt/frame and synchronization.
  double gyro_z = 0.0;
};

struct PlanarWheelState {
  Stamp stamp;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  // Continuous, unwrapped heading in the caller's local frame.
  double yaw = 0.0;
  double distance = 0.0;
  bool valid = false;
};

// Independent diagnostic candidate. No global input, LIO update, bias learning
// or qualified covariance; never substitutes for LocalEstimator implicitly.
class PlanarWheelDr {
 public:
  PlanarWheelDr(const std::string& clock, double max_gap, double gyro_bias);
  // Explicit diagnostic epoch seed, not a stopped-reset production contract.
  Result Seed(const PlanarWheelInput& input, const Eigen::Vector2d& position,
              double yaw);
  Result Add(const PlanarWheelInput& input);
  const PlanarWheelState& state() const { return state_; }

 private:
  Result Validate(const PlanarWheelInput& input) const;
  std::string clock_;
  double max_gap_;
  double gyro_bias_;
  bool faulted_ = false;
  PlanarWheelInput previous_;
  PlanarWheelState state_;
};

}  // namespace unified
}  // namespace localization
}  // namespace openloc
