// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <cstdint>
#include <string>

#include "candidates/global/global_types.h"
#include "openloc/localization/map/scan_to_map.h"

namespace openloc {
namespace localization {
namespace unified {

struct MapObservationContract {
  std::string source = "map";
  std::string map_id;
  std::string map_version;
  std::string map_content_id;
  std::string map_frame;
  std::string base_frame;
  std::string calibration_id;
  bool georeferenced = false;
  // Association allowance across the legacy double-second history boundary.
  // Never changes the original measurement timestamp.
  int64_t maximum_timestamp_error_ns = 1000;
};

struct MapObservationTiming {
  int64_t receive_timestamp_ns = 0;
  uint64_t source_sequence = 0;
};

Result AdaptScanToMapObservation(const map::ScanToMapResult& match,
                                 const LocalState& local,
                                 const MapObservationContract& contract,
                                 const MapObservationTiming& timing,
                                 GlobalObservation* observation);

}  // namespace unified
}  // namespace localization
}  // namespace openloc
