// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <optional>

#include "candidates/common/types.h"

namespace openloc {
namespace localization {
namespace unified {

struct GlobalConfig {
  std::string clock_id = "unix";
  std::string map_id;
  std::string map_version;
  std::string map_content_id;
  std::string map_frame;
  std::string base_frame;
  std::string calibration_id;
  double history_duration = 3.0;
  double max_interpolation_gap = 0.05;
  double max_observation_age = 2.0;
  double local_timeout = 0.2;
  double global_timeout = 2.0;
  double future_tolerance = 0.02;
  double innovation_gate = 22.458;
  double max_position_std = 2.0;
  double max_attitude_std = 0.2;
  uint32_t verification_frames = 3;
  double verification_position = 0.5;
  double verification_angle = 0.1;
  // Drift variances are defined at the first ODOM reference pose per epoch.
  double position_drift_variance_rate = 0.01;
  double attitude_drift_variance_rate = 0.0001;
  bool enable_fixed_lag_graph = false;
  // Explicit alternative for bounded replays with declared, uncalibrated
  // local covariance. Cannot enable the qualified graph or trusted output.
  bool diagnostic_only = false;
  bool diagnostic_fixed_gain = false;
  bool enable_diagnostic_horizontal_bootstrap = false;
  bool enable_heading_bootstrap = false;
  bool motion_error_model_qualified = false;
  bool graph_allow_uncalibrated_inputs = false;
  bool graph_retain_on_full_pose_staleness = false;
  double graph_solve_budget_ms = 50.0;
  double graph_robust_threshold = 2.5;
  uint32_t graph_max_states = 1000;
  uint32_t graph_max_observations = 2000;
  uint32_t graph_max_error_dimension = 256;
  uint32_t graph_max_iterations = 5;
  double graph_max_linearization_angle = 0.2;
};

struct GlobalObservation {
  enum class Kind { POSE, POSITION, PROJECTED_POSE, HORIZONTAL_POSITION, HEADING };
  Kind kind = Kind::POSE;
  Stamp stamp;
  // Exact source times, when supplied. Stamp remains the legacy double-second
  // backend representation, not a nanosecond-precision history.
  std::optional<int64_t> measurement_timestamp_ns;
  std::optional<int64_t> receive_timestamp_ns;
  Epoch epoch;
  std::string source;
  std::string map_id;
  std::string map_version;
  std::string map_content_id;
  std::string map_frame;
  std::string base_frame;
  std::string calibration_id;
  std::string covariance_source;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Matrix6d covariance = Matrix6d::Zero();
  // POSITION measures this body-fixed point, not an invented base orientation.
  Eigen::Vector3d point_in_base = Eigen::Vector3d::Zero();
  double heading_rad = 0.0;
  double heading_variance = 0.0;
  Eigen::Vector3d direction_in_base = Eigen::Vector3d::UnitX();
  // Rows act on [translation_map (m), left rotation_map (rad)].
  Eigen::MatrixXd projection;
  Eigen::MatrixXd projected_covariance;
  // Caller source checks passed; this does not imply receiver quality or
  // covariance calibration.
  bool quality_valid = false;
  bool covariance_calibrated = false;
  bool diagnostic_only = false;
  bool physical_up_qualified = false;
  bool independent_reference = false;
  // True only when the source covariance model is calibrated for this use.
  // Diagnostic replay may consume unqualified covariance, but cannot publish
  // trusted output.
  bool uncertainty_qualified = false;
  bool georeferenced = false;
  bool recovery = false;
  bool ambiguous = false;
  bool independent_of_local = false;
  // Qualified conditional source noise n=C^T P^-1 e+eta; empty by default.
  Eigen::MatrixXd local_error_cross_covariance;
  uint64_t local_error_sequence = 0;
  std::optional<double> local_error_time;
  bool conditional_local_error_qualified = false;
};

struct HorizontalGlobalState {
  // [translation at reference XY, yaw], never a full SE(3) pose.
  Eigen::Vector3d alignment = Eigen::Vector3d::Zero();
  Eigen::Vector2d odom_reference_xy = Eigen::Vector2d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  double last_observation = 0.0;
  double covariance_time = 0.0;
  int64_t last_observation_ns = 0;
};

struct GlobalState {
  Epoch epoch;
  Eigen::Isometry3d map_to_odom = Eigen::Isometry3d::Identity();
  Matrix6d covariance = Matrix6d::Zero();
  double last_observation = 0.0;
  double last_full_observation = 0.0;
  double last_georeference = 0.0;
  double covariance_time = 0.0;
  uint64_t correction_id = 0;
  bool valid = false;
  bool georeferenced = false;
  bool verifying = false;
  bool diagnostic_only = false;
  std::optional<HorizontalGlobalState> horizontal;
};

struct GlobalEvidence {
  GlobalObservation observation;
  LocalState local;
};

bool ValidGlobalPose(const Eigen::Isometry3d& pose);
bool ValidGlobalObservationGeometry(const GlobalObservation& observation);

Matrix6d TransformCovarianceBound(const Eigen::Isometry3d& map_base,
                                  const LocalState& local,
                                  const Matrix6d& observation_covariance,
                                  Matrix6d* observation_bound = nullptr,
                                  Matrix6d* local_bound = nullptr);
Matrix6d GlobalPoseCovarianceBound(const GlobalState& global,
                                   const LocalState& local);
Matrix6d FixedGainCovarianceBound(const Matrix6d& prior,
                                  const Eigen::MatrixXd& observation,
                                  const Eigen::MatrixXd& projection,
                                  const Eigen::MatrixXd& gain);

}  // namespace unified
}  // namespace localization
}  // namespace openloc
