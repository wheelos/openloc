// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "openloc/localization/online/localization_engine.h"

namespace {

namespace loc = openloc::localization;

std::vector<std::string> Fields(std::string line) {
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  std::vector<std::string> fields;
  std::istringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

int64_t Integer(const std::string& text) {
  std::size_t consumed = 0;
  const auto value = std::stoll(text, &consumed);
  if (consumed != text.size() || value <= 0) {
    throw std::runtime_error("invalid positive integer");
  }
  return value;
}

double Scalar(const std::string& text) {
  std::size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::runtime_error("invalid finite scalar");
  }
  return value;
}

std::vector<std::vector<std::string>> Read(const std::string& path,
                                           const std::string& header,
                                           std::size_t count) {
  std::ifstream stream(path);
  std::string line;
  if (!stream || !std::getline(stream, line) ||
      Fields(line) != Fields(header)) {
    throw std::runtime_error("invalid stream/header: " + path);
  }
  std::vector<std::vector<std::string>> rows;
  while (std::getline(stream, line)) {
    auto fields = Fields(line);
    if (fields.size() != count) {
      throw std::runtime_error("invalid field count: " + path);
    }
    rows.push_back(std::move(fields));
  }
  if (!stream.eof() || rows.empty()) {
    throw std::runtime_error("empty/unreadable input: " + path);
  }
  return rows;
}

void Text(std::ostream& output, const std::string& value) {
  output << '"';
  for (char character : value) {
    if (character == '"') {
      output << '"';
    }
    output << character;
  }
  output << '"';
}

loc::Timestamp Time(int64_t timestamp) {
  return {timestamp, loc::ClockDomain::kUnixEpoch};
}

Eigen::Vector3d Position(const Eigen::Vector4d& alignment,
                         const Eigen::Vector3d& point) {
  return alignment.head<3>() +
         Eigen::AngleAxisd(alignment[3], Eigen::Vector3d::UnitZ()) * point;
}

const char* Publication(loc::CorrectionPublicationState state) {
  switch (state) {
    case loc::CorrectionPublicationState::kUninitialized:
      return "uninitialized";
    case loc::CorrectionPublicationState::kAligned:
      return "aligned";
    case loc::CorrectionPublicationState::kRateLimited:
      return "rate_limited";
    case loc::CorrectionPublicationState::kHeldForReset:
      return "held_for_reset";
    case loc::CorrectionPublicationState::kAwaitingPolicyUpdate:
      return "awaiting_policy_update";
  }
  throw std::runtime_error("unknown publication state");
}

template <typename Derived>
void Values(std::ostream& output, const Eigen::MatrixBase<Derived>& values) {
  for (Eigen::Index index = 0; index < values.size(); ++index) {
    output << ',' << values.derived().coeff(index);
  }
}

struct FixEvent {
  int64_t arrival = 0;
  loc::GnssHorizontalFix fix;
};

void Check(const loc::Status& status, bool measurement = false) {
  if (status.ok() ||
      (measurement &&
       (status.code() == loc::StatusCode::kPending ||
        status.code() == loc::StatusCode::kRejectedMeasurement ||
        status.code() == loc::StatusCode::kOutsideHistory ||
        status.code() == loc::StatusCode::kOutsideFixedLagWindow ||
        status.code() == loc::StatusCode::kInterpolationGapTooLarge ||
        status.code() == loc::StatusCode::kOutOfOrderTimestamp))) {
    return;
  }
  throw std::runtime_error(status.message());
}

int Replay(char** argv) {
  const std::string mode = argv[4];
  if ((mode != "baseline" && mode != "recovery" && mode != "tracking") ||
      std::string(argv[5]) != "--diagnostic-synthetic-quality") {
    throw std::runtime_error("explicit diagnostic profile and arm required");
  }
  loc::LocalizationConfig config;
  config.map_reference = {"prepared-xy", "v1", "declared-diagnostic-lever"};
  config.map_reference.geodetic.state =
      loc::GeodeticReferenceState::kDiagnosticOnly;
  config.enable_gnss_reacquisition = mode != "baseline";
  if (mode == "tracking") {
    config.large_correction_policy =
        loc::LargeCorrectionPolicy::kRateLimitedTracking;
  }
  config.odometry_buffer.maximum_interpolation_gap_ns = 2'500'000'000LL;
  // Source-prepared 1 m sigma is not a receiver-reported accuracy field.
  config.maximum_horizontal_accuracy_m = 2.0;
  const auto lever_rows = Read(argv[6], "lx,ly,lz", 3);
  if (lever_rows.size() != 1) {
    throw std::runtime_error("exactly one declared lever required");
  }
  for (int index = 0; index < 3; ++index) {
    config.gnss_antenna_in_base[index] = Scalar(lever_rows[0][index]);
  }
  const auto odom_rows = Read(argv[1], "timestamp_ns,px,py,pz,qx,qy,qz,qw", 8);
  const auto fix_rows =
      Read(argv[2], "arrival_ns,timestamp_ns,gx,gy,sigma,quality", 6);
  std::vector<loc::OdometrySample> odometry;
  std::vector<FixEvent> fixes;
  int64_t previous = 0;
  for (const auto& row : odom_rows) {
    loc::OdometrySample sample;
    sample.timestamp = Time(Integer(row[0]));
    if (sample.timestamp.nanoseconds <= previous) {
      throw std::runtime_error("nonmonotonic ODOM");
    }
    previous = sample.timestamp.nanoseconds;
    sample.epoch = {"frozen-external-odom", 1};
    sample.parent_frame = {"odom"};
    sample.child_frame = {"base_link"};
    for (int index = 0; index < 3; ++index) {
      sample.pose.translation[index] = Scalar(row[index + 1]);
    }
    sample.pose.rotation = Eigen::Quaterniond(Scalar(row[7]), Scalar(row[4]),
                                              Scalar(row[5]), Scalar(row[6]));
    if (std::abs(sample.pose.rotation.norm() - 1.0) > 1e-8) {
      throw std::runtime_error("nonunit ODOM rotation");
    }
    odometry.push_back(sample);
  }
  previous = 0;
  for (const auto& row : fix_rows) {
    FixEvent event;
    event.arrival = Integer(row[0]);
    event.fix.timestamp = Time(Integer(row[1]));
    if (event.arrival < previous ||
        event.arrival < event.fix.timestamp.nanoseconds ||
        event.arrival < odometry.front().timestamp.nanoseconds) {
      throw std::runtime_error("invalid fix arrival order/time");
    }
    previous = event.arrival;
    event.fix.frame = {"map"};
    event.fix.map_reference = config.map_reference;
    event.fix.position = Eigen::Vector2d(Scalar(row[2]), Scalar(row[3]));
    const double sigma = Scalar(row[4]);
    if (sigma <= 0 || (row[5] != "0" && row[5] != "1")) {
      throw std::runtime_error("invalid sigma/quality flag");
    }
    event.fix.covariance = Eigen::Matrix2d::Identity() * sigma * sigma;
    event.fix.receiver.carrier_solution =
        row[5] == "1" ? loc::GnssCarrierSolution::kFixed
                      : loc::GnssCarrierSolution::kInvalid;
    event.fix.receiver.satellite_count = 20;
    event.fix.receiver.hdop = 0.8;
    event.fix.receiver.differential_age_seconds = 0.2;
    event.fix.receiver.horizontal_accuracy_m = 1.0;
    fixes.push_back(event);
  }
  const std::string output_path = argv[3];
  for (const auto& path : {output_path, output_path + ".factors.csv",
                           output_path + ".summary.csv"}) {
    if (std::filesystem::exists(path)) {
      throw std::runtime_error("refuse existing output: " + path);
    }
  }
  std::ofstream output(output_path);
  std::ofstream factors(output_path + ".factors.csv");
  std::ofstream summary(output_path + ".summary.csv");
  if (!output || !factors || !summary) {
    throw std::runtime_error("cannot open output streams");
  }
  output << std::setprecision(17)
         << "event,event_index,arrival_ns,measurement_ns,status,odom_ns,"
            "px,py,pz,qx,qy,qz,qw,epoch_generation,"
            "backend_tx,backend_ty,backend_tz,backend_yaw,"
            "published_tx,published_ty,published_tz,published_yaw,"
            "backend_x,backend_y,backend_z,published_x,published_y,published_z,"
            "backend_ax,backend_ay,backend_az,published_ax,published_ay,"
            "published_az,"
            "offset_x,offset_y,offset_z,offset_yaw,publication_state,pending,"
            "mean_matches,alignment_revision,reset_counter,reset_event,"
            "global_valid,vertical_initialized,correction_jump_m,"
            "correction_yaw_rad,speed_proxy_mps";
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      output << ",cov_" << row << col;
    }
  }
  output
      << ",factor_count,source_fresh,global_degraded,reference_x,reference_y";
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      output << ",body_cov_" << row << col;
    }
  }
  output << '\n';
  factors << std::setprecision(17)
          << "arrival_ns,record_index,timestamp_ns,source,type,accepted,reason,"
             "nis,threshold,state_id,residual_size\n";
  config.maximum_factor_records = fixes.size() * 2 + 1;
  loc::LocalizationEngine engine;
  Check(engine.Configure(config));
  std::vector<std::string> previous_records;
  loc::OdometrySample latest;
  loc::LocalizationResult before;
  bool has_before = false;
  std::size_t odom_index = 0;
  std::size_t fix_index = 0;
  std::size_t events = 0;
  double speed_proxy = 0.0;
  const auto started = std::chrono::steady_clock::now();
  while (odom_index < odometry.size() || fix_index < fixes.size()) {
    const bool is_odom = odom_index < odometry.size() &&
                         (fix_index == fixes.size() ||
                          odometry[odom_index].timestamp.nanoseconds <=
                              fixes[fix_index].arrival);
    loc::Status status;
    int64_t arrival;
    int64_t measurement;
    std::size_t event_index;
    if (is_odom) {
      const auto& sample = odometry[odom_index];
      if (odom_index > 0) {
        speed_proxy =
            (sample.pose.translation - latest.pose.translation).norm() /
            (static_cast<double>(sample.timestamp.nanoseconds -
                                 latest.timestamp.nanoseconds) /
             1e9);
      }
      latest = sample;
      arrival = measurement = sample.timestamp.nanoseconds;
      event_index = odom_index++;
      status = engine.AddOdometry(sample);
      Check(status);
      if (sample.pose.translation != latest.pose.translation ||
          sample.pose.rotation.coeffs() != latest.pose.rotation.coeffs()) {
        throw std::runtime_error("caller ODOM mutated");
      }
    } else {
      const auto& event = fixes[fix_index];
      arrival = event.arrival;
      measurement = event.fix.timestamp.nanoseconds;
      event_index = fix_index++;
      status = engine.SubmitGnssHorizontalFix(event.fix);
      Check(status, true);
    }
    loc::LocalizationResult result;
    Check(engine.GetResult(&result));
    if (result.odometry_epoch != latest.epoch ||
        result.timestamp.nanoseconds != latest.timestamp.nanoseconds) {
      throw std::runtime_error("result ODOM identity/time changed");
    }
    const Eigen::Vector3d antenna =
        latest.pose.translation +
        latest.pose.rotation * config.gnss_antenna_in_base;
    const double jump =
        has_before ? (Position(result.map_from_odom, latest.pose.translation) -
                      Position(before.map_from_odom, latest.pose.translation))
                         .norm()
                   : 0.0;
    const double yaw_jump =
        has_before
            ? std::remainder(result.map_from_odom[3] - before.map_from_odom[3],
                             2.0 * std::acos(-1.0))
            : 0.0;
    output << (is_odom ? "odom" : "fix") << ',' << event_index << ',' << arrival
           << ',' << measurement << ',' << static_cast<int>(status.code())
           << ',' << latest.timestamp.nanoseconds;
    Values(output, latest.pose.translation);
    Values(output, latest.pose.rotation.coeffs());
    output << ',' << latest.epoch.generation;
    Values(output, result.backend_map_from_odom);
    Values(output, result.map_from_odom);
    Values(output,
           Position(result.backend_map_from_odom, latest.pose.translation));
    Values(output, Position(result.map_from_odom, latest.pose.translation));
    Values(output, Position(result.backend_map_from_odom, antenna));
    Values(output, Position(result.map_from_odom, antenna));
    Values(output, result.publication_pose_offset);
    output << ',' << Publication(result.publication_state) << ','
           << result.correction_pending << ','
           << result.published_mean_matches_backend << ','
           << result.alignment_revision << ',' << result.reset_counter << ','
           << result.reset_event << ',' << result.global_alignment_valid << ','
           << result.vertical_position_initialized << ',' << jump << ','
           << yaw_jump << ',' << speed_proxy;
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        output << ',' << result.alignment_covariance(row, col);
      }
    }
    output << ',' << result.factor_records.size() << ','
           << result.correction_source_fresh << ','
           << result.global_output_degraded << ','
           << result.alignment_reference_in_odom.x() << ','
           << result.alignment_reference_in_odom.y();
    const Eigen::Vector3d rotated_position =
        Eigen::AngleAxisd(result.backend_map_from_odom[3],
                          Eigen::Vector3d::UnitZ()) *
        latest.pose.translation;
    Eigen::Matrix<double, 3, 4> body_jacobian =
        Eigen::Matrix<double, 3, 4>::Zero();
    body_jacobian.leftCols<3>().setIdentity();
    body_jacobian(0, 3) = -rotated_position.y();
    body_jacobian(1, 3) = rotated_position.x();
    const Eigen::Matrix3d body_covariance =
        body_jacobian * result.alignment_covariance * body_jacobian.transpose();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        output << ',' << body_covariance(row, col);
      }
    }
    output << '\n';
    previous_records.resize(result.factor_records.size());
    for (std::size_t index = 0; index < result.factor_records.size(); ++index) {
      const auto& record = result.factor_records[index];
      std::ostringstream serialized;
      serialized << std::setprecision(17) << index << ',' << record.timestamp_ns
                 << ',';
      Text(serialized, record.source);
      serialized << ',';
      Text(serialized, record.type);
      serialized << ',' << record.accepted << ',';
      Text(serialized, record.reason);
      serialized << ',' << record.normalized_innovation_squared << ','
                 << record.innovation_threshold << ',' << record.state_id << ','
                 << record.residual.size();
      if (previous_records[index] != serialized.str()) {
        factors << arrival << ',' << serialized.str() << '\n';
        previous_records[index] = serialized.str();
      }
    }
    before = result;
    has_before = true;
    ++events;
  }
  const auto pending =
      std::count_if(before.factor_records.begin(), before.factor_records.end(),
                    [](const loc::FactorRecord& record) {
                      return record.reason == "pending_odometry_bracket";
                    });
  rusage resources{};
  if (getrusage(RUSAGE_SELF, &resources) != 0) {
    throw std::runtime_error("cannot read peak resource usage");
  }
  summary << "odom_epochs,fix_submissions,output_events,pending_at_end,"
             "alignment_revision,reset_counter,wall_seconds,peak_rss_kb\n"
          << odometry.size() << ',' << fixes.size() << ',' << events << ','
          << pending << ',' << before.alignment_revision << ','
          << before.reset_counter << ','
          << std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                           started)
                 .count()
          << ',' << resources.ru_maxrss << '\n';
  output.close();
  factors.close();
  summary.close();
  if (!output || !factors || !summary) {
    throw std::runtime_error("output write failed");
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 7) {
    std::cerr
        << "usage: public_xy_dataset_replay odom.csv fixes.csv output.csv "
           "baseline|recovery|tracking --diagnostic-synthetic-quality "
           "lever.csv\n";
    return 2;
  }
  try {
    return Replay(argv);
  } catch (const std::exception& error) {
    std::cerr << "public XY diagnostic failed: " << error.what() << '\n';
    return 2;
  }
}
