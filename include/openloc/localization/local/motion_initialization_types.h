// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include "openloc/localization/local/native_types.h"

namespace openloc::localization::unified {

struct RelativeLidarPose {
  double time = 0.0;
  Eigen::Isometry3d reference_from_base = Eigen::Isometry3d::Identity();
};

struct MotionInitializationConfig {
  double gravity = 9.80665;
  double minimum_duration = 2.0;
  double maximum_duration = 10.0;
  double maximum_imu_gap = 0.05;
  double position_residual_scale = 0.05;
  double velocity_residual_scale = 0.1;
  double maximum_normalized_residual = 3.0;
  double minimum_scaled_singular_value = 1e-3;
  double maximum_gravity_error = 0.3;
  double maximum_accel_bias = 1.0;
  double maximum_gyro_bias = 0.2;
  double maximum_angular_residual = 0.1;
  bool allow_lidar_stationary_initialization = false;
  double maximum_stationary_speed = 0.02;
  double maximum_stationary_angular_speed = 0.02;
};

struct MotionInitialization {
  double time = 0.0;
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  double minimum_scaled_singular_value = 0.0;
  double normalized_residual_rms = 0.0;
  double angular_residual_rms = 0.0;
  bool lidar_stationary_initialization = false;
};

// Numerical evidence only; neither rank nor a small residual qualifies
// accuracy. A fresh record is returned on every call, including rejected
// windows.
struct MotionInitializationDiagnostics {
  bool lidar_stationary_initialization = false;
  double maximum_lidar_speed = 0.0;
  double maximum_lidar_angular_speed = 0.0;
  double maximum_stationary_force_residual = 0.0;
  bool linear_system_evaluated = false;
  bool solution_available = false;
  double minimum_scaled_singular_value = 0.0;
  double gravity_error = 0.0;
  double accel_bias_norm = 0.0;
  double gyro_bias_norm = 0.0;
  double normalized_residual_rms = 0.0;
  double angular_residual_rms = 0.0;
};

}  // namespace openloc::localization::unified
