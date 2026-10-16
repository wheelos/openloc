// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "alignment_recovery.h"
#include "alignment_smoother.h"

namespace {

using openloc::localization::StatusCode;
using openloc::localization::internal::AlignmentEstimate;
using openloc::localization::internal::AlignmentSmoother;
using openloc::localization::internal::AlignmentSmootherConfig;
using openloc::localization::internal::GnssAlignmentRecovery;
using openloc::localization::internal::GnssAlignmentRecoveryCandidate;
using openloc::localization::internal::GnssAlignmentRecoveryDiagnostics;
using openloc::localization::internal::GnssAlignmentRecoveryObservation;
using openloc::localization::internal::GnssFactorDiagnostics;

bool ReadLine(std::istream& source, std::string* line) {
  if (!std::getline(source, *line)) {
    return false;
  }
  if (!line->empty() && line->back() == '\r') {
    line->pop_back();
  }
  return true;
}

std::vector<std::string> Split(const std::string& line) {
  std::vector<std::string> fields;
  std::istringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

void WriteText(std::ostream& stream, const std::string& text) {
  stream << '"';
  for (char character : text) {
    if (character == '"') {
      stream << '"';
    }
    stream << character;
  }
  stream << '"';
}

int Replay(const char* input, const char* output, const std::string& mode,
           const char* recovery_output_path) {
  if (mode != "lio" && mode != "gnss" && mode != "map-rejected" &&
      mode != "gnss-map-rejected" && mode != "gnss-recovery") {
    std::cerr << "unknown replay mode\n";
    return 2;
  }
  const bool use_recovery = mode == "gnss-recovery";
  const bool use_gnss =
      mode == "gnss" || mode == "gnss-map-rejected" || use_recovery;
  std::ifstream source(input);
  std::ofstream result(output);
  std::ofstream recovery_result;
  if (use_recovery) {
    recovery_result.open(recovery_output_path);
  }
  std::string line;
  if (!source || !result || !ReadLine(source, &line) ||
      (use_recovery && !recovery_result) ||
      line !=
          "timestamp_ns,px,py,pz,ax,ay,az,gx,gy,sigma,distance_m,gnss_present,"
          "map_"
          "rejected") {
    std::cerr << "invalid replay streams/header\n";
    return 2;
  }
  AlignmentSmootherConfig config;
  config.initial_sigma =
      Eigen::Vector4d(1000.0, 1000.0, 100.0, 3.141592653589793);
  AlignmentSmoother smoother(config);
  GnssAlignmentRecovery recovery;
  int64_t previous_time = 0;
  bool initialized = false;
  uint64_t alignment_revision = 0;
  result << std::setprecision(17);
  recovery_result << std::setprecision(17);
  result
      << "timestamp_ns,px,py,pz,x,y,z,tx,ty,tz,yaw,var_x,var_y,var_z,var_yaw,"
         "gnss_attempted,gnss_accepted,nis,map_rejected,reason\n";
  if (use_recovery) {
    recovery_result
        << "timestamp_ns,gnss_attempted,innovation_rejected,nis,"
           "recovery_reason,training_count,validation_count,training_rms_m,"
           "validation_nis,candidate_validated,reanchored,alignment_revision,"
           "pre_x,pre_y,pre_z,pre_yaw,pre_var_z,post_x,post_y,post_z,"
           "post_yaw,post_var_z\n";
  }
  while (ReadLine(source, &line)) {
    const auto fields = Split(line);
    if (fields.size() != 13) {
      std::cerr << "invalid replay field count\n";
      return 2;
    }
    std::size_t consumed = 0;
    const int64_t time = std::stoll(fields[0], &consumed);
    if (consumed != fields[0].size() || time <= previous_time) {
      std::cerr << "invalid/nonmonotonic replay time\n";
      return 2;
    }
    double values[12];
    for (int index = 0; index < 12; ++index) {
      values[index] = std::stod(fields[index + 1], &consumed);
      if (consumed != fields[index + 1].size() ||
          !std::isfinite(values[index])) {
        std::cerr << "invalid replay scalar\n";
        return 2;
      }
    }
    if (values[8] <= 0.0 || values[9] < 0.0 ||
        (values[10] != 0.0 && values[10] != 1.0) ||
        (values[11] != 0.0 && values[11] != 1.0)) {
      std::cerr << "invalid sigma/distance/availability flag\n";
      return 2;
    }
    uint64_t state = 0;
    auto status = smoother.AddState(time, values[9], &state);
    if (!status.ok()) {
      std::cerr << status.message() << '\n';
      return 2;
    }
    AlignmentEstimate before_gnss;
    status = smoother.Estimate(&before_gnss);
    if (!status.ok()) {
      std::cerr << status.message() << '\n';
      return 2;
    }
    GnssFactorDiagnostics diagnostics;
    bool accepted = false;
    std::string reason = "not_requested";
    std::string recovery_reason = "not_requested";
    bool innovation_rejected = false;
    bool candidate_validated = false;
    bool reanchored = false;
    GnssAlignmentRecoveryDiagnostics recovery_diagnostics;
    GnssAlignmentRecoveryCandidate recovery_candidate;
    const bool attempt_gnss = use_gnss && values[10] == 1.0;
    if (attempt_gnss) {
      status = smoother.AddGnssHorizontalPosition(
          state, time, Eigen::Vector3d(values[3], values[4], values[5]),
          Eigen::Vector2d(values[6], values[7]),
          Eigen::Matrix2d::Identity() * values[8] * values[8], &diagnostics,
          !initialized);
      if (!status.ok() && status.code() != StatusCode::kRejectedMeasurement) {
        std::cerr << status.message() << '\n';
        return 2;
      }
      accepted = status.ok();
      initialized = initialized || accepted;
      reason = accepted ? "accepted_diagnostic_xy" : status.message();
      if (use_recovery) {
        if (accepted) {
          recovery.Reset();
          recovery_reason = "gnss_factor_accepted";
        } else if (status.code() == StatusCode::kRejectedMeasurement &&
                   diagnostics.horizontal_innovation_rejected) {
          innovation_rejected = true;
          GnssAlignmentRecoveryObservation observation;
          observation.timestamp_ns = time;
          observation.odom_antenna_xy = Eigen::Vector2d(values[3], values[4]);
          observation.global_antenna_xy = Eigen::Vector2d(values[6], values[7]);
          observation.covariance_xy =
              Eigen::Matrix2d::Identity() * values[8] * values[8];
          status = recovery.AddRejectedObservation(
              observation, &recovery_diagnostics, &recovery_candidate);
          if (!status.ok()) {
            std::cerr << status.message() << '\n';
            return 2;
          }
          recovery_reason = recovery_diagnostics.reason;
          candidate_validated = recovery_diagnostics.candidate_validated;
          if (candidate_validated) {
            Eigen::Vector4d alignment = before_gnss.map_from_odom;
            alignment.head<2>() =
                recovery_candidate.global_from_odom_xy_yaw.head<2>();
            alignment[3] = recovery_candidate.global_from_odom_xy_yaw[2];
            Eigen::Vector4d sigma = config.initial_sigma;
            sigma.head<2>() = recovery_candidate.sigma_xy_yaw.head<2>();
            sigma[3] = recovery_candidate.sigma_xy_yaw[2];
            sigma[2] = std::max(
                sigma[2],
                std::sqrt(std::max(0.0, before_gnss.covariance(2, 2))));
            uint64_t recovered_state = 0;
            status =
                smoother.Reanchor(time, alignment, sigma, &recovered_state);
            if (!status.ok()) {
              std::cerr << status.message() << '\n';
              return 2;
            }
            recovery.Reset();
            reanchored = true;
            ++alignment_revision;
            recovery_reason = "recovery_holdout_validated_global_reanchor";
          }
        } else {
          recovery.Reset();
          recovery_reason = "rejected_outside_innovation_recovery_path";
        }
      }
    }
    AlignmentEstimate estimate;
    status = smoother.Estimate(&estimate);
    if (!status.ok()) {
      std::cerr << status.message() << '\n';
      return 2;
    }
    const auto& a = estimate.map_from_odom;
    const double cosine = std::cos(a[3]);
    const double sine = std::sin(a[3]);
    const Eigen::Vector3d position(a[0] + cosine * values[0] - sine * values[1],
                                   a[1] + sine * values[0] + cosine * values[1],
                                   a[2] + values[2]);
    result << time;
    for (int index = 0; index < 3; ++index) {
      result << ',' << values[index];
    }
    for (int index = 0; index < 3; ++index) {
      result << ',' << position[index];
    }
    for (int index = 0; index < 4; ++index) {
      result << ',' << a[index];
    }
    for (int index = 0; index < 4; ++index) {
      result << ',' << estimate.covariance(index, index);
    }
    const bool map_requested =
        mode == "map-rejected" || mode == "gnss-map-rejected";
    result << ',' << attempt_gnss << ',' << accepted << ','
           << diagnostics.normalized_innovation_squared << ','
           << (map_requested && values[11] == 1.0) << ',';
    WriteText(result, reason);
    result << '\n';
    if (use_recovery) {
      recovery_result << time << ',' << attempt_gnss << ','
                      << innovation_rejected << ','
                      << diagnostics.normalized_innovation_squared << ',';
      WriteText(recovery_result, recovery_reason);
      recovery_result << ',' << recovery_diagnostics.training_count << ','
                      << recovery_diagnostics.validation_count << ','
                      << recovery_diagnostics.training_rms_m << ','
                      << recovery_diagnostics.validation_nis << ','
                      << candidate_validated << ',' << reanchored << ','
                      << alignment_revision << ','
                      << before_gnss.map_from_odom[0] << ','
                      << before_gnss.map_from_odom[1] << ','
                      << before_gnss.map_from_odom[2] << ','
                      << before_gnss.map_from_odom[3] << ','
                      << before_gnss.covariance(2, 2) << ','
                      << estimate.map_from_odom[0] << ','
                      << estimate.map_from_odom[1] << ','
                      << estimate.map_from_odom[2] << ','
                      << estimate.map_from_odom[3] << ','
                      << estimate.covariance(2, 2) << '\n';
    }
    previous_time = time;
  }
  if (!source.eof() || previous_time == 0 || !result ||
      (use_recovery && !recovery_result)) {
    std::cerr << "replay stream failed or empty\n";
    return 2;
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if ((argc != 4 && argc != 5) ||
      (argc == 5 && std::string(argv[3]) != "gnss-recovery") ||
      (argc == 4 && std::string(argv[3]) == "gnss-recovery")) {
    std::cerr << "usage: horizontal_alignment_replay input.csv output.csv "
                 "lio|gnss|map-rejected|gnss-map-rejected|gnss-recovery "
                 "[recovery-output.csv]\n";
    return 2;
  }
  try {
    return Replay(argv[1], argv[2], argv[3], argc == 5 ? argv[4] : nullptr);
  } catch (const std::invalid_argument& error) {
    std::cerr << "replay parse error: " << error.what() << '\n';
  } catch (const std::out_of_range& error) {
    std::cerr << "replay parse range error: " << error.what() << '\n';
  }
  return 2;
}
