// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "candidates/global/global_alignment.h"
#include "openloc/localization/map/scan_to_map.h"
#include "pcl/io/pcd_io.h"

namespace {
namespace u = openloc::localization::unified;
namespace m = openloc::localization::map;

bool ReadLine(std::istream& input, std::string* line) {
  if (!std::getline(input, *line)) return false;
  if (!line->empty() && line->back() == '\r') line->pop_back();
  return true;
}

std::vector<std::string> Fields(const std::string& line) {
  std::istringstream input(line);
  std::vector<std::string> result;
  std::string field;
  while (std::getline(input, field, ',')) {
    if (!field.empty() && field.back() == '\r') field.pop_back();
    result.push_back(field);
  }
  return result;
}

double Number(const std::string& field) {
  std::size_t used = 0;
  const double value = std::stod(field, &used);
  if (used != field.size() || !std::isfinite(value))
    throw std::runtime_error("invalid finite scalar");
  return value;
}

int64_t Time(const std::string& field) {
  std::size_t used = 0;
  const int64_t value = std::stoll(field, &used);
  if (used != field.size() || value <= 0)
    throw std::runtime_error("invalid integer timestamp");
  return value;
}

Eigen::Isometry3d Pose(const std::vector<std::string>& fields,
                       std::size_t offset) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 3; ++i)
    pose.translation()[i] = Number(fields[offset + i]);
  Eigen::Quaterniond q(Number(fields[offset + 6]), Number(fields[offset + 3]),
                       Number(fields[offset + 4]), Number(fields[offset + 5]));
  if (std::abs(q.norm() - 1.0) > 1e-6)
    throw std::runtime_error("non-unit quaternion");
  pose.linear() = q.toRotationMatrix();
  return pose;
}

void Require(const u::Result& result) {
  if (!result.ok()) throw std::runtime_error(result.message);
}

void Text(std::ostream& output, const std::string& value) {
  output << '"';
  for (char c : value) {
    if (c == '"') output << '"';
    output << c;
  }
  output << '"';
}

void Matrix(std::ostream& output, const Eigen::MatrixXd& matrix) {
  std::ostringstream values;
  values << std::setprecision(17);
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    if (row) values << '|';
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      if (col) values << ';';
      values << matrix(row, col);
    }
  }
  Text(output, values.str());
}

int Replay(char** argv) {
  const std::string mode = argv[5];
  if (mode != "odom" && mode != "gnss" && mode != "map" && mode != "all")
    throw std::runtime_error("invalid arm");
  if (std::string(argv[6]) != "--diagnostic-only")
    throw std::runtime_error("explicit diagnostic flag required");
  if (std::filesystem::exists(argv[4]))
    throw std::runtime_error("output already exists");
  std::ifstream input(argv[2]), prior(argv[3]);
  std::string line;
  if (!input || !prior || !ReadLine(input, &line) ||
      line !=
          "timestamp_ns,px,py,pz,qx,qy,qz,qw,gx,gy,sigma,gnss_present,"
          "scan_path,local_position_variance,local_rotation_variance")
    throw std::runtime_error("invalid input schema");
  if (!ReadLine(prior, &line))
    throw std::runtime_error("missing explicit diagnostic startup prior");
  const auto seed_fields = Fields(line);
  if (seed_fields.size() != 7)
    throw std::runtime_error("startup prior requires xyz xyzw");
  const auto seed = Pose(seed_fields, 0);
  u::GlobalConfig config;
  config.map_id = "frozen-map";
  config.map_version = "v1";
  config.calibration_id = "manual-unqualified";
  config.diagnostic_only = true;
  // Sparse sampling is not an online freshness qualification.
  config.global_timeout = 30.0;
  u::GlobalAlignment owner(config);
  Require(owner.ValidateConfig());
  m::ScanToMapConfig matcher_config;
  matcher_config.map_frame = "map";
  matcher_config.base_frame = matcher_config.cloud_frame = "body";
  matcher_config.separate_ndt_support = true;
  m::ScanToMapLocalizer matcher(matcher_config);
  if (mode == "map" || mode == "all") {
    m::PointCloud::Ptr cloud(new m::PointCloud);
    if (pcl::io::loadPCDFile(argv[1], *cloud) != 0)
      throw std::runtime_error("cannot load frozen map");
    m::MapMetadata metadata;
    metadata.map_id = config.map_id;
    metadata.version = config.map_version;
    metadata.content_id = "caller-hash-pinned";
    metadata.map_frame = "map";
    metadata.source_cloud_frame = "body";
    metadata.integrity_verified = true;
    metadata.reference_state = m::MapReferenceState::kDiagnosticOnly;
    std::string error;
    if (!matcher.Configure(metadata, cloud, &error))
      throw std::runtime_error(error);
  }
  std::ofstream output(argv[4]);
  if (!output) throw std::runtime_error("cannot open output");
  output << std::setprecision(17);
  output << "timestamp_ns,px,py,pz,x,y,z,qx,qy,qz,qw,gnss_present,"
            "gnss_accepted,gnss_reason,map_scheduled,map_valid,map_accepted,"
            "map_reason,score,normal_overlap,distance_overlap,rank,weight,"
            "var_x,var_y,var_z,var_roll,var_pitch,var_yaw,diagnostic_only,"
            "local_position_variance,local_rotation_variance,gnss_sigma_m,"
            "map_processing_time_ms,map_projection,map_observed_covariance,"
            "alignment_covariance,correction_translation_m,"
            "correction_rotation_rad,alignment_initialized,global_trusted,"
            "odom_qx,odom_qy,odom_qz,odom_qw\n";
  int64_t first = 0, previous = 0;
  uint64_t sequence = 0;
  while (ReadLine(input, &line)) {
    const auto f = Fields(line);
    if (f.size() != 15) throw std::runtime_error("invalid field count");
    const int64_t ns = Time(f[0]);
    if (ns <= previous) throw std::runtime_error("nonmonotonic event");
    if (first == 0) first = ns;
    const double time = 1.0 + static_cast<double>(ns - first) * 1e-9;
    const auto odom = Pose(f, 1);
    const double pos_variance = Number(f[13]);
    const double rot_variance = Number(f[14]);
    if (pos_variance <= 0.0 || rot_variance <= 0.0)
      throw std::runtime_error("invalid diagnostic ODOM variance");
    const double present = Number(f[11]);
    if (present != 0.0 && present != 1.0)
      throw std::runtime_error("invalid availability flag");
    u::LocalState local;
    local.stamp = {time, time, ++sequence, "unix"};
    local.epoch = {"frozen-gnss-off", 1};
    local.position = odom.translation();
    local.orientation = Eigen::Quaterniond(Number(f[7]), Number(f[4]),
                                           Number(f[5]), Number(f[6]));
    local.valid = true;
    local.covariance_model_valid = false;
    local.covariance = u::Matrix15d::Identity() * rot_variance;
    local.covariance.topLeftCorner<3, 3>() =
        Eigen::Matrix3d::Identity() * pos_variance;
    Require(owner.AddLocal(local));
    if (previous == 0) {
      u::Matrix6d covariance = u::Matrix6d::Identity();
      covariance.bottomRightCorner<3, 3>() *= 0.01;
      Require(owner.SeedDiagnostic(seed, covariance, time));
    }
    const auto before = owner.Predict(time).map_to_odom * odom;
    u::GlobalObservation observation;
    observation.stamp = local.stamp;
    observation.epoch = local.epoch;
    observation.map_id = config.map_id;
    observation.map_version = config.map_version;
    observation.calibration_id = config.calibration_id;
    observation.quality_valid = true;
    observation.uncertainty_qualified = false;
    bool gnss_accepted = false, map_accepted = false;
    std::string gnss_reason = "not_requested", map_reason = "not_requested";
    if (present == 1.0 && (mode == "gnss" || mode == "all")) {
      observation.source = "gnss";
      observation.kind = u::GlobalObservation::Kind::HORIZONTAL_POSITION;
      observation.point_in_base = Eigen::Vector3d(-0.131, -0.02329, 0.00412);
      observation.pose.translation().head<2>() =
          Eigen::Vector2d(Number(f[8]), Number(f[9]));
      const double sigma = Number(f[10]);
      if (sigma <= 0.0) throw std::runtime_error("invalid GNSS sigma");
      observation.covariance.topLeftCorner<2, 2>() =
          Eigen::Matrix2d::Identity() * sigma * sigma;
      const auto revision = owner.state().correction_id;
      const auto status = owner.Observe(observation);
      gnss_accepted = status.ok() && owner.state().correction_id > revision;
      gnss_reason = status.ok() && !gnss_accepted
                        ? "unanchored_position_evidence"
                        : u::ReasonName(status.reason);
      if (!status.ok() && status.reason != u::Reason::INNOVATION_REJECTED)
        Require(status);
    }
    m::ScanToMapResult match;
    const bool scheduled = f[12] != "-";
    if (scheduled && (mode == "map" || mode == "all")) {
      m::PointCloud::Ptr scan(new m::PointCloud);
      if (pcl::io::loadPCDFile(f[12], *scan) != 0)
        throw std::runtime_error("cannot load scheduled scan");
      m::ScanToMapRequest request;
      request.timestamp_ns = ns;
      request.clock_domain = "unix";
      request.cloud_frame = "body";
      request.deskewed = true;
      request.cloud = scan;
      // Current owner estimate only; no future/per-frame reference poses.
      request.map_from_base_initial = owner.Predict(time).map_to_odom * odom;
      match = matcher.Match(request);
      map_reason = match.reason;
      if (match.valid) {
        observation.source = "map";
        observation.point_in_base.setZero();
        observation.pose = match.map_from_base;
        observation.kind = match.geometry_full_rank
                               ? u::GlobalObservation::Kind::POSE
                               : u::GlobalObservation::Kind::PROJECTED_POSE;
        observation.covariance = match.covariance;
        observation.projection = match.observed_projection;
        observation.projected_covariance = match.observed_covariance;
        const auto status = owner.Observe(observation);
        map_accepted = status.ok();
        map_reason = u::ReasonName(status.reason);
        if (!status.ok() && status.reason != u::Reason::INNOVATION_REJECTED &&
            status.reason != u::Reason::RELOCALIZATION_VERIFYING)
          Require(status);
      }
    }
    u::LocalState retained;
    Require(owner.Lookup(time, &retained));
    if (!retained.position.isApprox(local.position, 0.0) ||
        !retained.orientation.coeffs().isApprox(local.orientation.coeffs(),
                                                0.0) ||
        !retained.covariance.isApprox(local.covariance, 0.0) ||
        retained.covariance_model_valid)
      throw std::runtime_error("ODOM identity/quality changed");
    const auto state = owner.Predict(time);
    const auto composed = state.map_to_odom * odom;
    if (!composed.matrix().allFinite() || !u::ValidCovariance(state.covariance))
      throw std::runtime_error("nonfinite pose or invalid output covariance");
    const Eigen::Quaterniond q(composed.linear());
    output << ns;
    for (double v : {local.position.x(), local.position.y(), local.position.z(),
                     composed.translation().x(), composed.translation().y(),
                     composed.translation().z(), q.x(), q.y(), q.z(), q.w()})
      output << ',' << v;
    output << ',' << present << ',' << gnss_accepted << ',';
    Text(output, gnss_reason);
    output << ',' << scheduled << ',' << match.valid << ',' << map_accepted
           << ',';
    Text(output, map_reason);
    output << ',' << match.score << ',' << match.overlap << ','
           << (match.ndt_support_available
                   ? static_cast<double>(match.ndt_distance_supported_points) /
                         match.registration_source_points
                   : 0.0)
           << ',' << match.observed_projection.rows() << ",0.5";
    for (int i = 0; i < 6; ++i) output << ',' << state.covariance(i, i);
    output << ',' << state.diagnostic_only << ',' << pos_variance << ','
           << rot_variance << ',' << Number(f[10]) << ','
           << match.processing_time_ms << ',';
    Matrix(output, match.observed_projection);
    output << ',';
    Matrix(output, match.observed_covariance);
    output << ',';
    Matrix(output, state.covariance);
    output << ',' << (composed.translation() - before.translation()).norm()
           << ','
           << u::LogRotation(Eigen::Quaterniond(composed.linear() *
                                                before.linear().transpose()))
                  .norm()
           << ',' << state.valid << ",0," << local.orientation.x() << ','
           << local.orientation.y() << ',' << local.orientation.z() << ','
           << local.orientation.w() << '\n';
    previous = ns;
  }
  if (!input.eof() || sequence == 0 || !output.flush())
    throw std::runtime_error("incomplete replay I/O");
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 7) {
    std::cerr << "usage: map.pcd events.csv prior.csv output.csv "
                 "odom|gnss|map|all --diagnostic-only\n";
    return 2;
  }
  try {
    return Replay(argv);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
