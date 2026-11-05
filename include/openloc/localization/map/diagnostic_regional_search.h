// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "openloc/localization/local/native_types.h"
#include "openloc/localization/map/scan_to_map.h"

namespace openloc::localization::map {

struct DiagnosticRegionConfig {
  std::string calibration_id;
  Eigen::Vector3d field_minimum = Eigen::Vector3d::Zero();
  Eigen::Vector3d field_maximum = Eigen::Vector3d::Zero();
  Eigen::Vector3d search_minimum = Eigen::Vector3d::Zero();
  Eigen::Vector3d search_maximum = Eigen::Vector3d::Zero();
  Eigen::Vector3d observed_minimum = Eigen::Vector3d::Zero();
  Eigen::Vector3d observed_maximum = Eigen::Vector3d::Zero();
  double yaw_minimum = 0;
  double yaw_maximum = 0;
  Eigen::Quaterniond tilt = Eigen::Quaterniond::Identity();
  double field_resolution_m = 0;
  double distance_sigma_m = 0;
  double inlier_distance_m = 0;
};

struct DiagnosticRegionResult {
  std::string status;
  std::string message;
  unified::Reason reason = unified::Reason::NONE;
  size_t modes = 0;
  size_t verified_modes = 0;
  bool truncated = false;
  size_t stable_tracks = 0;
  uint32_t attempt = 0;
  std::vector<ScanToMapResult> candidates;
  std::optional<Eigen::Isometry3d> map_from_base;
};

// Conditional regional/tilt/coverage search, never a calibrated posterior,
// global owner, trusted recovery lease or unconstrained whole-map search.
// Calls and the caller-provided matcher must be serialized.
class DiagnosticRegionalSearch {
 public:
  DiagnosticRegionalSearch(const DiagnosticRegionConfig& config,
                           const MapMetadata& metadata,
                           const PointCloud& cloud);
  ~DiagnosticRegionalSearch();
  DiagnosticRegionalSearch(const DiagnosticRegionalSearch&) = delete;
  DiagnosticRegionalSearch& operator=(const DiagnosticRegionalSearch&) = delete;
  DiagnosticRegionResult Observe(const unified::LocalState& local,
                                 uint64_t scan_sequence,
                                 const ScanToMapRequest& request,
                                 ScanToMapLocalizer* matcher);
  void Restart();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace openloc::localization::map
