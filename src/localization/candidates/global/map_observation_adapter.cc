// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "candidates/global/map_observation_adapter.h"

#include <cmath>
#include <limits>
#include <utility>

namespace openloc {
namespace localization {
namespace unified {
Result AdaptScanToMapObservation(const map::ScanToMapResult& match,
                                 const LocalState& local,
                                 const MapObservationContract& contract,
                                 const MapObservationTiming& timing,
                                 GlobalObservation* observation) {
  if (match.structured_error)
    return {Reason::CORRELATION_UNQUALIFIED,
            "Structured map sources require a source-aware graph adapter"};
  if (observation == nullptr || !local.valid || local.epoch.session.empty() ||
      local.epoch.generation == 0 || local.stamp.sequence == 0 ||
      !std::isfinite(local.stamp.time) || local.stamp.time <= 0.0 ||
      contract.source.empty() || contract.map_id.empty() ||
      contract.map_version.empty() || contract.map_content_id.empty() ||
      contract.map_frame.empty() || contract.base_frame.empty() ||
      contract.calibration_id.empty() || match.covariance_source.empty() ||
      contract.maximum_timestamp_error_ns < 0) {
    return {Reason::INVALID_INPUT, "invalid map observation contract"};
  }
  if (match.status != map::MatchStatus::kMatched || !match.valid ||
      !match.converged || match.ambiguous) {
    return {match.ambiguous ? Reason::RELOCALIZATION_AMBIGUOUS
                            : Reason::MATCH_FAILED,
            "scan-to-map result is not an accepted match"};
  }
  if (!match.geometry_full_rank && !match.observability_available) {
    return {Reason::INVALID_INPUT, "map observed subspace is unavailable"};
  }
  if (timing.receive_timestamp_ns <= 0 || timing.source_sequence == 0) {
    return {Reason::CLOCK_INVALID, "invalid map source timing"};
  }
  if (match.map_id != contract.map_id ||
      match.map_version != contract.map_version ||
      match.map_content_id != contract.map_content_id ||
      match.map_frame != contract.map_frame ||
      match.base_frame != contract.base_frame ||
      match.calibration_id != contract.calibration_id) {
    return {Reason::MAP_MISMATCH, "scan-to-map map identity or frame mismatch"};
  }
  const long double local_time_ns =
      static_cast<long double>(local.stamp.time) * 1e9L;
  if (match.clock_domain.empty() ||
      match.clock_domain != local.stamp.clock_id || match.timestamp_ns <= 0 ||
      local_time_ns >
          static_cast<long double>(std::numeric_limits<int64_t>::max()) ||
      local_time_ns <
          static_cast<long double>(std::numeric_limits<int64_t>::min())) {
    return {Reason::CLOCK_INVALID,
            "scan-to-map and local clocks are not comparable"};
  }
  const int64_t local_timestamp_ns =
      static_cast<int64_t>(std::round(local_time_ns));
  const long double timestamp_error =
      std::abs(static_cast<long double>(local_timestamp_ns) -
               static_cast<long double>(match.timestamp_ns));
  if (timestamp_error > contract.maximum_timestamp_error_ns) {
    return {Reason::HISTORY_UNAVAILABLE,
            "scan-to-map/local association exceeds timestamp error budget"};
  }

  GlobalObservation adapted;
  adapted.kind = match.geometry_full_rank
                     ? GlobalObservation::Kind::POSE
                     : GlobalObservation::Kind::PROJECTED_POSE;
  adapted.stamp = local.stamp;
  adapted.stamp.time = static_cast<double>(match.timestamp_ns) * 1e-9;
  adapted.measurement_timestamp_ns = match.timestamp_ns;
  adapted.stamp.receive_time =
      static_cast<double>(timing.receive_timestamp_ns) * 1e-9;
  adapted.stamp.sequence = timing.source_sequence;
  adapted.receive_timestamp_ns = timing.receive_timestamp_ns;
  adapted.epoch = local.epoch;
  adapted.source = contract.source;
  adapted.map_id = match.map_id;
  adapted.map_version = match.map_version;
  adapted.map_content_id = match.map_content_id;
  adapted.map_frame = match.map_frame;
  adapted.base_frame = match.base_frame;
  adapted.calibration_id = contract.calibration_id;
  adapted.covariance_source = match.covariance_source;
  adapted.pose = match.map_from_base;
  adapted.covariance = match.covariance;
  adapted.projection = match.observed_projection;
  adapted.projected_covariance = match.observed_covariance;
  if (!ValidGlobalObservationGeometry(adapted)) {
    return {Reason::INVALID_INPUT, "invalid map pose or observed covariance"};
  }
  adapted.quality_valid = true;
  adapted.covariance_calibrated = match.covariance_calibrated;
  adapted.diagnostic_only =
      match.diagnostic_only ||
      match.calibration_state != map::CalibrationState::kCalibrated ||
      !match.covariance_calibrated;
  adapted.uncertainty_qualified = !adapted.diagnostic_only;
  adapted.physical_up_qualified = match.physical_up_qualified;
  adapted.independent_reference = match.independent_reference;
  adapted.georeferenced = contract.georeferenced;
  // A map match consumes the ODOM pose that initialized its registration.
  adapted.independent_of_local = false;
  *observation = std::move(adapted);
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace openloc
