// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "openloc/localization/online/types.h"

namespace openloc {
namespace localization {

// Audit declarations do not qualify a joint statistical error model.
enum class ObservationDependency { kUnknown, kAbsent, kPresent };

struct GlobalObservationProvenance {
  std::string acquisition_id;
  std::string preparation_id;
  ObservationDependency odometry_initialization =
      ObservationDependency::kUnknown;
  ObservationDependency odometry_deskew = ObservationDependency::kUnknown;
  ObservationDependency shared_local_scans = ObservationDependency::kUnknown;
  ObservationDependency gnss_map_construction = ObservationDependency::kUnknown;
};

struct GlobalObservationContext {
  Timestamp measurement_time;
  Timestamp receive_time;
  uint64_t sequence = 0;
  std::string source;
  OdometryEpoch epoch;
  MapReference reference;
  std::string map_content_id;
  FrameId map_frame;
  FrameId base_frame;
  GlobalObservationProvenance provenance;
};

struct GlobalMapObservation {
  enum class Kind { kFullPose, kProjectedPose };
  GlobalObservationContext context;
  Kind kind = Kind::kFullPose;
  Pose map_from_base;
  // [translation_map (m), left rotation_map (rad)].
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::MatrixXd projection;
  Eigen::MatrixXd projected_covariance;
  std::string covariance_source;
  bool ambiguous = false;
  bool recovery = false;
};

// Prepared antenna coordinates; receiver admission and calibration are
// external.
struct GlobalHorizontalObservation {
  GlobalObservationContext context;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  Eigen::Vector3d point_in_base = Eigen::Vector3d::Zero();
  std::string covariance_source;
};

struct GlobalPositionObservation {
  GlobalObservationContext context;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  Eigen::Vector3d point_in_base = Eigen::Vector3d::Zero();
  std::string covariance_source;
};

// Azimuth atan2(map Y, map X) of a calibrated body-fixed antenna baseline.
// Not an INS attitude or a second antenna position. Receiver status, baseline
// length, uncertainty and dwell admission belong to the caller.
struct GlobalHeadingObservation {
  GlobalObservationContext context;
  double heading_rad = 0.0;
  double variance_rad2 = 0.0;
  Eigen::Vector3d direction_in_base = Eigen::Vector3d::UnitX();
  std::string covariance_source;
};

struct GlobalHorizontalEstimate {
  Eigen::Vector3d map_from_odom_xy_yaw = Eigen::Vector3d::Zero();
  Eigen::Vector2d map_base_xy = Eigen::Vector2d::Zero();
  // Diagnostic covariance at odom_reference_xy; not a qualified joint bound.
  Eigen::Vector2d odom_reference_xy = Eigen::Vector2d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  Timestamp last_observation_time;
  uint64_t correction_id = 0;
};

struct ObservationInnovation {
  bool available = false;
  std::string unavailable_reason;
  std::string prior_state_frame;
  uint32_t observed_dimension = 0;
  uint32_t prior_state_dimension = 0;
  Eigen::VectorXd residual;
  Eigen::MatrixXd covariance;
  double nis = 0.0;
  // The actual admission threshold, not a calibration or trust declaration.
  double gate_threshold = 0.0;
  std::string residual_basis;
  std::string covariance_model;
  bool covariance_decomposition_available = false;
  Eigen::MatrixXd prior_contribution;
  Eigen::MatrixXd observation_contribution;
  Eigen::MatrixXd local_contribution;
  Pose prior_pose;
  Eigen::Matrix<double, 6, 6> prior_covariance =
      Eigen::Matrix<double, 6, 6>::Zero();
};

struct GlobalObservationDecision {
  GlobalObservationContext context;
  Status status;
  uint64_t decision_id = 0;
  uint64_t correction_id = 0;
  // An OK status can mean retained, unanchored position evidence, not a change.
  bool alignment_updated = false;
  std::string covariance_source;
  ObservationInnovation innovation;
};

}  // namespace localization
}  // namespace openloc
