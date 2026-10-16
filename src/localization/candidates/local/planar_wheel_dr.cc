// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.
#include "candidates/local/planar_wheel_dr.h"

#include <cmath>

namespace openloc {
namespace localization {
namespace unified {

PlanarWheelDr::PlanarWheelDr(const std::string& clock, double max_gap,
                             double gyro_bias)
    : clock_(clock), max_gap_(max_gap), gyro_bias_(gyro_bias) {}

Result PlanarWheelDr::Validate(const PlanarWheelInput& input) const {
  if (clock_.empty() || !std::isfinite(max_gap_) || max_gap_ <= 0.0 ||
      !std::isfinite(gyro_bias_)) {
    return {Reason::CONFIG_INVALID, "Invalid planar wheel DR configuration"};
  }
  if (input.stamp.clock_id != clock_) {
    return {Reason::CLOCK_INVALID, "Planar wheel DR clock mismatch"};
  }
  if (!std::isfinite(input.stamp.time) || !input.velocity.allFinite() ||
      !std::isfinite(input.gyro_z)) {
    return {Reason::INVALID_INPUT, "Non-finite planar wheel input"};
  }
  return {};
}

Result PlanarWheelDr::Seed(const PlanarWheelInput& input,
                           const Eigen::Vector2d& position, double yaw) {
  const Result result = Validate(input);
  if (!result.ok()) {
    return result;
  }
  if (!position.allFinite() || !std::isfinite(yaw)) {
    return {Reason::INVALID_INPUT, "Invalid planar wheel DR seed"};
  }
  state_.stamp = input.stamp;
  state_.position = position;
  state_.yaw = yaw;
  state_.distance = 0.0;
  state_.valid = true;
  previous_ = input;
  faulted_ = false;
  return {};
}

Result PlanarWheelDr::Add(const PlanarWheelInput& input) {
  const Result result = Validate(input);
  if (!result.ok()) {
    return result;
  }
  if (faulted_) {
    return {Reason::IMU_GAP, "Planar wheel DR gap requires explicit reseed"};
  }
  if (!state_.valid) {
    return {Reason::WAITING_FOR_IMU, "Planar wheel DR requires explicit seed"};
  }
  const double dt = input.stamp.time - previous_.stamp.time;
  if (dt <= 0.0) {
    return {dt == 0.0 ? Reason::DUPLICATE : Reason::TIMESTAMP_REGRESSION,
            "Planar wheel DR time must advance"};
  }
  if (dt > max_gap_) {
    faulted_ = true;
    state_.valid = false;
    return {Reason::IMU_GAP, "Planar wheel DR input gap"};
  }
  const Eigen::Vector2d velocity = 0.5 * (previous_.velocity + input.velocity);
  const double rate = 0.5 * (previous_.gyro_z + input.gyro_z) - gyro_bias_;
  const double half = 0.5 * rate * dt;
  const double sinc = std::abs(half) > 1e-12 ? std::sin(half) / half : 1.0;
  const double midpoint = state_.yaw + half;
  Eigen::Matrix2d rotation;
  rotation << std::cos(midpoint), -std::sin(midpoint), std::sin(midpoint),
      std::cos(midpoint);
  PlanarWheelState next = state_;
  next.position += dt * sinc * rotation * velocity;
  next.yaw += 2.0 * half;
  next.distance += dt * velocity.norm();
  next.stamp = input.stamp;
  if (!next.position.allFinite() || !std::isfinite(next.yaw) ||
      !std::isfinite(next.distance)) {
    return {Reason::INVALID_INPUT, "Non-finite planar wheel DR integration"};
  }
  state_ = next;
  previous_ = input;
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace openloc
