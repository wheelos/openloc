// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "openloc/localization/map/scan_to_map.h"
#include "pcl/io/pcd_io.h"

namespace {

using openloc::localization::map::CalibrationState;
using openloc::localization::map::MapMetadata;
using openloc::localization::map::MapReferenceState;
using openloc::localization::map::MatchStatus;
using openloc::localization::map::PointCloud;
using openloc::localization::map::ScanMatchingAlgorithm;
using openloc::localization::map::ScanToMapConfig;
using openloc::localization::map::ScanToMapLocalizer;
using openloc::localization::map::ScanToMapRequest;
using openloc::localization::map::ScanToMapResult;

bool SplitCsv(const std::string& line, std::vector<std::string>* fields) {
  fields->clear();
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields->push_back(field);
  }
  return fields->size() == 9;
}

bool ParseDouble(const std::string& text, double* value) {
  try {
    std::size_t consumed = 0;
    *value = std::stod(text, &consumed);
    return consumed == text.size() && std::isfinite(*value);
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseTimestamp(const std::string& text, int64_t* timestamp_ns) {
  try {
    std::size_t consumed = 0;
    *timestamp_ns = std::stoll(text, &consumed);
    return consumed == text.size() && *timestamp_ns > 0;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseUnsigned(const std::string& text, uint32_t* value) {
  try {
    std::size_t consumed = 0;
    const auto parsed = std::stoul(text, &consumed);
    if (consumed != text.size() ||
        parsed > std::numeric_limits<uint32_t>::max()) {
      return false;
    }
    *value = static_cast<uint32_t>(parsed);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ReadWindowPositions(const std::string& request_path,
                         std::vector<Eigen::Vector3d>* positions) {
  std::ifstream requests(request_path);
  if (!requests || positions == nullptr) {
    return false;
  }
  std::string line;
  if (!std::getline(requests, line)) {
    return false;
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  if (line != "timestamp_ns,scan_path,x,y,z,qx,qy,qz,qw") {
    return false;
  }
  uint64_t row_count = 0;
  while (std::getline(requests, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    std::vector<std::string> fields;
    int64_t timestamp_ns = 0;
    double translation[3];
    if (!SplitCsv(line, &fields) || !ParseTimestamp(fields[0], &timestamp_ns)) {
      return false;
    }
    for (int index = 0; index < 3; ++index) {
      if (!ParseDouble(fields[index + 2], &translation[index])) {
        return false;
      }
    }
    positions->emplace_back(translation[0], translation[1], translation[2]);
    ++row_count;
  }
  return requests.eof() && row_count > 0;
}

bool ParseMatcherOptions(int argc, char** argv, ScanToMapConfig* config,
                         bool* prewarm_local_windows) {
  for (int index = 7; index < argc; index += 2) {
    if (index + 1 >= argc) {
      return false;
    }
    const std::string option(argv[index]);
    const std::string value(argv[index + 1]);
    if (option == "--matcher") {
      if (value == "ndt") {
        config->algorithm = ScanMatchingAlgorithm::kPclNdt;
      } else if (value == "point-to-plane-icp") {
        config->algorithm = ScanMatchingAlgorithm::kPointToPlaneIcp;
      } else {
        return false;
      }
    } else if (option == "--scan-voxel-m") {
      if (!ParseDouble(value, &config->scan_voxel_size_m)) {
        return false;
      }
    } else if (option == "--separate-ndt-support") {
      if (value != "true" && value != "false") {
        return false;
      }
      config->separate_ndt_support = value == "true";
    } else if (option == "--map-voxel-m") {
      if (!ParseDouble(value, &config->map_voxel_size_m)) {
        return false;
      }
    } else if (option == "--ndt-resolution-m") {
      if (!ParseDouble(value, &config->ndt_resolution_m)) {
        return false;
      }
    } else if (option == "--ndt-step-size-m") {
      if (!ParseDouble(value, &config->ndt_step_size_m)) {
        return false;
      }
    } else if (option == "--ndt-transformation-epsilon-m") {
      if (!ParseDouble(value, &config->ndt_transformation_epsilon_m)) {
        return false;
      }
    } else if (option == "--ndt-max-iterations") {
      if (!ParseUnsigned(value, &config->ndt_maximum_iterations)) {
        return false;
      }
    } else if (option == "--icp-max-points") {
      if (!ParseUnsigned(value, &config->maximum_registration_points)) {
        return false;
      }
    } else if (option == "--icp-iterations") {
      if (!ParseUnsigned(value, &config->icp_maximum_iterations)) {
        return false;
      }
    } else if (option == "--icp-normal-neighbors") {
      if (!ParseUnsigned(value, &config->normal_neighbors)) {
        return false;
      }
    } else if (option == "--icp-normal-planarity-ratio") {
      if (!ParseDouble(value, &config->normal_planarity_ratio)) {
        return false;
      }
    } else if (option == "--icp-distance-m") {
      if (!ParseDouble(value, &config->icp_distance_m)) {
        return false;
      }
    } else if (option == "--icp-min-pairs") {
      if (!ParseUnsigned(value, &config->icp_minimum_pairs)) {
        return false;
      }
    } else if (option == "--icp-min-source-overlap") {
      if (!ParseDouble(value, &config->icp_minimum_source_overlap)) {
        return false;
      }
    } else if (option == "--icp-min-target-overlap") {
      if (!ParseDouble(value, &config->icp_minimum_target_overlap)) {
        return false;
      }
    } else if (option == "--icp-max-rmse-m") {
      if (!ParseDouble(value, &config->icp_max_rmse_m)) {
        return false;
      }
    } else if (option == "--icp-condition-ratio") {
      if (!ParseDouble(value, &config->icp_condition_ratio)) {
        return false;
      }
    } else if (option == "--icp-translation-tolerance-m") {
      if (!ParseDouble(value, &config->icp_translation_tolerance_m)) {
        return false;
      }
    } else if (option == "--icp-rotation-tolerance-rad") {
      if (!ParseDouble(value, &config->icp_rotation_tolerance_rad)) {
        return false;
      }
    } else if (option == "--icp-max-correction-translation-m") {
      if (!ParseDouble(value, &config->icp_maximum_correction_translation_m)) {
        return false;
      }
    } else if (option == "--icp-max-correction-rotation-rad") {
      if (!ParseDouble(value, &config->icp_maximum_correction_rotation_rad)) {
        return false;
      }
    } else if (option == "--profile-icp-timing") {
      if (value == "true") {
        config->enable_icp_profiling = true;
      } else if (value == "false") {
        config->enable_icp_profiling = false;
      } else {
        return false;
      }
    } else if (option == "--prewarm-local-windows" &&
               prewarm_local_windows != nullptr) {
      if (value == "true") {
        *prewarm_local_windows = true;
      } else if (value == "false") {
        *prewarm_local_windows = false;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
  return true;
}

const char* StatusName(MatchStatus status) {
  switch (status) {
    case MatchStatus::kMatched:
      return "MATCHED";
    case MatchStatus::kRejected:
      return "REJECTED";
    case MatchStatus::kNotReady:
      return "NOT_READY";
  }
  return "UNKNOWN";
}

void WriteTransform(std::ostream& output, const Eigen::Isometry3d& transform) {
  const Eigen::Quaterniond quaternion(transform.linear());
  output << transform.translation().x() << ',' << transform.translation().y()
         << ',' << transform.translation().z() << ',' << quaternion.x() << ','
         << quaternion.y() << ',' << quaternion.z() << ',' << quaternion.w();
}

void WriteResult(std::ostream& output, const ScanToMapResult& result) {
  output << result.timestamp_ns << ',' << StatusName(result.status) << ','
         << result.reason << ',' << (result.valid ? 1 : 0) << ',';
  if (result.valid) {
    WriteTransform(output, result.map_from_base);
  } else {
    output << ",,,,,,";
  }
  output << ',' << result.score << ',' << result.overlap << ','
         << result.source_overlap << ',' << result.target_overlap << ','
         << (result.converged ? 1 : 0) << ',' << result.iterations << ','
         << result.processing_time_ms << ',' << result.hessian_min_eigenvalue
         << ',' << result.hessian_condition_number << ','
         << (result.geometry_full_rank ? 1 : 0) << ','
         << (result.observability_available ? 1 : 0) << ','
         << result.correction_translation_m << ','
         << result.correction_rotation_rad << ',' << result.source_points << ','
         << result.matched_points << ',' << result.map_window_points << ','
         << (result.covariance_calibrated ? 1 : 0) << ','
         << (result.calibration_state == CalibrationState::kUncalibrated
                 ? "uncalibrated"
                 : "calibrated")
         << ',' << result.map_window_build_ms << ','
         << result.target_voxelization_ms << ','
         << result.target_kdtree_build_ms << ','
         << result.target_normal_estimation_ms << ','
         << result.target_normal_neighbor_search_ms << ','
         << result.target_normal_fit_ms << ',' << result.icp_nearest_search_ms
         << ',' << result.icp_linearization_ms << ',' << result.icp_solver_ms
         << ',' << result.registration_source_points << ','
         << result.registration_target_points << ','
         << (result.ndt_support_available ? 1 : 0) << ','
         << result.ndt_distance_supported_points << ','
         << result.ndt_normal_supported_points << ','
         << result.ndt_rank_rejected_points << ','
         << result.ndt_nonplanar_rejected_points << ','
         << result.ndt_other_normal_rejected_points << ",\"";
  for (char character : result.detail) {
    if (character == '"') {
      output << '"';
    }
    output << character;
  }
  output << "\"\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 7 || argc % 2 == 0) {
    std::cerr << "usage: scan_to_map_dataset_replay <map.pcd> <requests.csv> "
                 "<results.csv> <map_id> <version> <content_id> "
                 "[--matcher ndt|point-to-plane-icp] "
                 "[--scan-voxel-m M] [--map-voxel-m M] "
                 "[--ndt-resolution-m M] [--ndt-step-size-m M] "
                 "[--ndt-transformation-epsilon-m M] "
                 "[--ndt-max-iterations N] [--icp-max-points N] "
                 "[--icp-iterations N] "
                 "[--icp-normal-neighbors N] "
                 "[--icp-normal-planarity-ratio R] "
                 "[--icp-distance-m M] [--icp-min-pairs N] "
                 "[--icp-min-source-overlap R] "
                 "[--icp-min-target-overlap R] "
                 "[--icp-max-rmse-m M] "
                 "[--icp-condition-ratio R] "
                 "[--icp-translation-tolerance-m M] "
                 "[--icp-rotation-tolerance-rad R] "
                 "[--icp-max-correction-translation-m M] "
                 "[--icp-max-correction-rotation-rad R] "
                 "[--prewarm-local-windows true|false] "
                 "[--profile-icp-timing true|false]\n";
    return 2;
  }

  PointCloud::Ptr map_cloud(new PointCloud);
  if (pcl::io::loadPCDFile(argv[1], *map_cloud) != 0) {
    std::cerr << "cannot load map PCD: " << argv[1] << '\n';
    return 2;
  }

  ScanToMapConfig config;
  config.map_frame = "map";
  config.base_frame = "body";
  config.cloud_frame = "body";
  bool prewarm_local_windows = false;
  if (!ParseMatcherOptions(argc, argv, &config, &prewarm_local_windows)) {
    std::cerr << "invalid scan matcher option\n";
    return 2;
  }

  ScanToMapLocalizer localizer(config);
  MapMetadata metadata;
  metadata.map_id = argv[4];
  metadata.version = argv[5];
  metadata.content_id = argv[6];
  metadata.map_frame = config.map_frame;
  metadata.source_cloud_frame = config.cloud_frame;
  metadata.reference_state = MapReferenceState::kDiagnosticOnly;
  metadata.integrity_verified = true;
  metadata.physical_up_qualified = false;
  metadata.independent_reference = false;
  std::string error;
  if (!localizer.Configure(metadata, map_cloud, &error)) {
    std::cerr << "cannot configure map matcher: " << error << '\n';
    return 2;
  }

  double prewarm_duration_ms = 0.0;
  size_t prewarmed_local_windows = 0;
  if (prewarm_local_windows) {
    std::vector<Eigen::Vector3d> positions;
    if (!ReadWindowPositions(argv[2], &positions)) {
      std::cerr << "cannot read valid positions for local-window preparation\n";
      return 2;
    }
    const auto started = std::chrono::steady_clock::now();
    if (!localizer.PrepareMapWindows(positions, &error)) {
      std::cerr << "cannot prepare local map windows: " << error << '\n';
      return 2;
    }
    prewarm_duration_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - started)
                              .count();
    prewarmed_local_windows = localizer.cached_window_count();
  }

  std::ifstream requests(argv[2]);
  if (!requests) {
    std::cerr << "cannot open request CSV: " << argv[2] << '\n';
    return 2;
  }
  std::ofstream results(argv[3], std::ios::trunc);
  if (!results) {
    std::cerr << "cannot create result CSV: " << argv[3] << '\n';
    return 2;
  }
  results << std::setprecision(17);
  results << "timestamp_ns,status,reason,valid,x,y,z,qx,qy,qz,qw,score,"
             "overlap,source_overlap,target_overlap,converged,iterations,"
             "processing_time_ms,"
             "hessian_min_eigenvalue,"
             "hessian_condition_number,geometry_full_rank,"
             "observability_available,correction_translation_m,"
             "correction_rotation_rad,source_points,matched_points,"
             "map_window_points,covariance_calibrated,calibration_state,"
             "map_window_build_ms,target_voxelization_ms,"
             "target_kdtree_build_ms,target_normal_estimation_ms,"
             "target_normal_neighbor_search_ms,target_normal_fit_ms,"
             "icp_nearest_search_ms,icp_linearization_ms,icp_solver_ms,"
             "registration_source_points,registration_target_points,"
             "ndt_support_available,ndt_distance_supported_points,"
             "ndt_normal_supported_points,ndt_rank_rejected_points,"
             "ndt_nonplanar_rejected_points,ndt_other_normal_rejected_points,"
             "detail\n";

  std::string line;
  if (!std::getline(requests, line)) {
    std::cerr << "request CSV has no header\n";
    return 2;
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  if (line != "timestamp_ns,scan_path,x,y,z,qx,qy,qz,qw") {
    std::cerr << "request CSV has an unexpected header\n";
    return 2;
  }
  uint64_t row_count = 0;
  while (std::getline(requests, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    std::vector<std::string> fields;
    ScanToMapRequest request;
    double translation[3];
    double quaternion[4];
    if (!SplitCsv(line, &fields) ||
        !ParseTimestamp(fields[0], &request.timestamp_ns)) {
      std::cerr << "malformed request row " << row_count + 2 << '\n';
      return 2;
    }
    for (int index = 0; index < 3; ++index) {
      if (!ParseDouble(fields[index + 2], &translation[index])) {
        std::cerr << "invalid translation at request row " << row_count + 2
                  << '\n';
        return 2;
      }
    }
    for (int index = 0; index < 4; ++index) {
      if (!ParseDouble(fields[index + 5], &quaternion[index])) {
        std::cerr << "invalid quaternion at request row " << row_count + 2
                  << '\n';
        return 2;
      }
    }
    Eigen::Quaterniond rotation(quaternion[3], quaternion[0], quaternion[1],
                                quaternion[2]);
    if (rotation.norm() < 1e-9) {
      std::cerr << "zero quaternion at request row " << row_count + 2 << '\n';
      return 2;
    }
    rotation.normalize();
    request.clock_domain = "sensor_unix";
    request.cloud_frame = config.cloud_frame;
    request.deskewed = true;
    request.map_from_base_initial = Eigen::Isometry3d::Identity();
    request.map_from_base_initial.linear() = rotation.toRotationMatrix();
    request.map_from_base_initial.translation() =
        Eigen::Vector3d(translation[0], translation[1], translation[2]);
    PointCloud::Ptr scan(new PointCloud);
    if (pcl::io::loadPCDFile(fields[1], *scan) != 0) {
      std::cerr << "cannot load scan PCD at request row " << row_count + 2
                << ": " << fields[1] << '\n';
      return 2;
    }
    request.cloud = scan;
    const ScanToMapResult result = localizer.Match(request);
    WriteResult(results, result);
    ++row_count;
  }
  if (!requests.eof() || row_count == 0 || !results) {
    std::cerr << "scan replay ended with an input/output error\n";
    return 2;
  }
  std::cout << "processed_scans=" << row_count
            << " cached_windows=" << localizer.cached_window_count()
            << " prewarmed_local_windows=" << prewarmed_local_windows
            << " window_preparation_ms=" << prewarm_duration_ms
            << " map_points=" << localizer.map_point_count() << '\n';
  return 0;
}
