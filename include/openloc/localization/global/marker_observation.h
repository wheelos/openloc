// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include "openloc/localization/global/observation_types.h"

namespace openloc {
namespace localization {

struct KnownMarker {
  std::string id;
  MapReference reference;
  std::string map_content_id;
  FrameId map_frame;
  FrameId marker_frame;
  Pose map_from_marker;
};

struct MarkerDetection {
  GlobalObservationContext context;
  std::string marker_id;
  FrameId sensor_frame;
  FrameId marker_frame;
  Pose sensor_from_marker;
  bool ambiguous = false;
};

struct MarkerExtrinsics {
  std::string calibration_id;
  FrameId base_frame;
  FrameId sensor_frame;
  Pose base_from_sensor;
};

// Joint covariance of [map_from_marker, sensor_from_marker, base_from_sensor].
// Each block uses additive translation in its parent frame (m), followed by
// left rotation in that frame (rad). Cross-block correlations are preserved;
// zero blocks are explicit deterministic assumptions, not calibration claims.
using MarkerJointCovariance = Eigen::Matrix<double, 18, 18>;

// T_map_base = T_map_marker * T_sensor_marker^-1 * T_base_sensor^-1.
// Converts one already-associated detection; no detector, association, quality
// admission or uncertainty qualification is inferred. The host supplies those
// declarations to the existing global owner. Failure leaves output unchanged.
Status BuildMarkerObservation(const KnownMarker& marker,
                              const MarkerDetection& detection,
                              const MarkerExtrinsics& extrinsics,
                              const MarkerJointCovariance& covariance,
                              GlobalMapObservation* output);

}  // namespace localization
}  // namespace openloc
