// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include "candidates/global/global_types.h"
#include "openloc/localization/global/observation_types.h"

namespace openloc {
namespace localization {
namespace alignment_bridge {

inline double Seconds(int64_t ns) { return static_cast<double>(ns) * 1e-9; }

inline std::string ClockId(ClockDomain domain) {
  return domain == ClockDomain::kUnixEpoch ? "unix" : "sensor_monotonic";
}

inline Status Convert(const unified::Result& result) {
  if (result.ok()) return Status::Ok();
  StatusCode code = StatusCode::kRejectedMeasurement;
  switch (result.reason) {
    case unified::Reason::CONFIG_INVALID:
    case unified::Reason::INVALID_INPUT:
      code = StatusCode::kInvalidArgument;
      break;
    case unified::Reason::HISTORY_UNAVAILABLE:
      code = StatusCode::kOutsideHistory;
      break;
    case unified::Reason::MAP_MISMATCH:
      code = StatusCode::kMapReferenceMismatch;
      break;
    case unified::Reason::EPOCH_MISMATCH:
      code = StatusCode::kEpochMismatch;
      break;
    case unified::Reason::CLOCK_INVALID:
      code = StatusCode::kInvalidTimestamp;
      break;
    case unified::Reason::DUPLICATE:
      code = StatusCode::kDuplicateTimestamp;
      break;
    case unified::Reason::TIMESTAMP_REGRESSION:
      code = StatusCode::kOutOfOrderTimestamp;
      break;
    default:
      break;
  }
  return Status(code, std::string(unified::ReasonName(result.reason)) + ": " +
                          result.message);
}

inline Eigen::Isometry3d Transform(const Pose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = pose.rotation.toRotationMatrix();
  result.translation() = pose.translation;
  return result;
}

inline Pose ValuePose(const Eigen::Isometry3d& transform) {
  return {transform.translation(), Eigen::Quaterniond(transform.linear())};
}

inline bool ValidProvenance(const GlobalObservationProvenance& provenance) {
  for (const auto dependency :
       {provenance.odometry_initialization, provenance.odometry_deskew,
        provenance.shared_local_scans, provenance.gnss_map_construction}) {
    switch (dependency) {
      case ObservationDependency::kUnknown:
      case ObservationDependency::kAbsent:
      case ObservationDependency::kPresent:
        break;
      default:
        return false;
    }
  }
  return true;
}

inline unified::GlobalObservation Observation(
    const GlobalObservationContext& context) {
  unified::GlobalObservation observation;
  observation.stamp = {Seconds(context.measurement_time.nanoseconds),
                       Seconds(context.receive_time.nanoseconds),
                       context.sequence,
                       ClockId(context.measurement_time.clock_domain)};
  observation.measurement_timestamp_ns = context.measurement_time.nanoseconds;
  observation.receive_timestamp_ns = context.receive_time.nanoseconds;
  observation.epoch = {context.epoch.producer_session,
                       context.epoch.generation};
  observation.source = context.source;
  observation.map_id = context.reference.id;
  observation.map_version = context.reference.version;
  observation.calibration_id = context.reference.calibration_id;
  observation.map_content_id = context.map_content_id;
  observation.map_frame = context.map_frame.value;
  observation.base_frame = context.base_frame.value;
  return observation;
}

}  // namespace alignment_bridge
}  // namespace localization
}  // namespace openloc
