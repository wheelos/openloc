// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "candidates/local/stationarity_detector.h"

#include <cmath>

namespace openloc {
namespace localization {
namespace unified {

StationarityDetector::StationarityDetector(const LocalConfig& config)
    : gravity_(config.gravity),
      wheel_timeout_(config.wheel_timeout),
      stationary_speed_(config.stationary_speed),
      stationary_gyro_(config.stationary_gyro),
      stationary_accel_tolerance_(config.stationary_accel_tolerance),
      stationary_duration_(config.stationary_duration),
      required_samples_(config.stationary_samples) {}

void StationarityDetector::Reset() {
  sample_count_ = 0;
  start_time_ = 0.0;
  acceleration_sum_.setZero();
  angular_velocity_sum_.setZero();
  mean_acceleration_.setZero();
  mean_angular_velocity_.setZero();
}

bool StationarityDetector::MeetsInstantaneousThresholds(
    const ImuSample& imu, const WheelSample* wheel, bool allow_imu_only) const {
  if (imu.angular_velocity.norm() > stationary_gyro_ ||
      std::abs(imu.acceleration.norm() - gravity_) >
          stationary_accel_tolerance_) {
    return false;
  }
  if (wheel == nullptr) {
    return allow_imu_only;
  }
  return wheel->stamp.time <= imu.stamp.time &&
         imu.stamp.time - wheel->stamp.time <= wheel_timeout_ &&
         imu.stamp.receive_time - wheel->stamp.time <= wheel_timeout_ &&
         std::abs(wheel->speed) <= stationary_speed_;
}

void StationarityDetector::Observe(const ImuSample& imu,
                                   const WheelSample* wheel,
                                   bool allow_imu_only, bool reset_window) {
  const bool stationary =
      MeetsInstantaneousThresholds(imu, wheel, allow_imu_only);
  if (reset_window || !stationary) {
    Reset();
  }
  if (stationary && sample_count_ < required_samples_) {
    if (sample_count_ == 0) {
      start_time_ = imu.stamp.time;
    }
    ++sample_count_;
    acceleration_sum_ += imu.acceleration;
    angular_velocity_sum_ += imu.angular_velocity;
    mean_acceleration_ = acceleration_sum_ / sample_count_;
    mean_angular_velocity_ = angular_velocity_sum_ / sample_count_;
  }
}

bool StationarityDetector::Qualified(double time) const {
  return std::isfinite(time) && sample_count_ >= required_samples_ &&
         time >= start_time_ && time - start_time_ >= stationary_duration_;
}

}  // namespace unified
}  // namespace localization
}  // namespace openloc
