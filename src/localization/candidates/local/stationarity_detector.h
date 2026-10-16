// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <cstdint>

#include "candidates/common/types.h"

namespace openloc {
namespace localization {
namespace unified {

class StationarityDetector {
 public:
  explicit StationarityDetector(const LocalConfig& config);

  void Reset();
  void Observe(const ImuSample& imu, const WheelSample* wheel,
               bool allow_imu_only, bool reset_window = false);
  bool Qualified(double time) const;
  uint32_t sample_count() const { return sample_count_; }
  const Eigen::Vector3d& mean_acceleration() const {
    return mean_acceleration_;
  }
  const Eigen::Vector3d& mean_angular_velocity() const {
    return mean_angular_velocity_;
  }

 private:
  bool MeetsInstantaneousThresholds(const ImuSample& imu,
                                    const WheelSample* wheel,
                                    bool allow_imu_only) const;

  double gravity_;
  double wheel_timeout_;
  double stationary_speed_;
  double stationary_gyro_;
  double stationary_accel_tolerance_;
  double stationary_duration_;
  uint32_t required_samples_;
  uint32_t sample_count_ = 0;
  double start_time_ = 0.0;
  Eigen::Vector3d acceleration_sum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_sum_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_acceleration_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_angular_velocity_ = Eigen::Vector3d::Zero();
};

}  // namespace unified
}  // namespace localization
}  // namespace openloc
