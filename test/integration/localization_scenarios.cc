// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "openloc/localization/global/fixed_lag_alignment.h"
#include "openloc/localization/global/marker_observation.h"
#include "test/integration/map/replay_csv.h"

namespace {
namespace loc = openloc::localization;
namespace csv = openloc::replay;
constexpr int64_t kStart = 10'000'000'000LL;
constexpr int64_t kStep = 500'000'000LL;
constexpr int kNodes = 24;
using Matrix15d = Eigen::Matrix<double, 15, 15>;

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void Require(const loc::Status& status) {
  Require(status.ok(), status.message());
}

void Matrix(std::ostream& out, const Eigen::MatrixXd& value) {
  out << value.rows() << 'x' << value.cols() << ':';
  for (Eigen::Index row = 0; row < value.rows(); ++row)
    for (Eigen::Index col = 0; col < value.cols(); ++col) {
      if (row || col) out << ';';
      out << value(row, col);
    }
}

loc::FixedLagAlignmentConfig Config() {
  loc::FixedLagAlignmentConfig config;
  config.reference = {"synthetic-site", "v1", "synthetic-extrinsics"};
  config.reference.geodetic.state = loc::GeodeticReferenceState::kDiagnosticOnly;
  config.map_content_id = "deterministic-scene";
  config.map_frame = {"map"};
  config.odom_frame = {"odom"};
  config.base_frame = {"base"};
  config.motion_error_model_id = "synthetic-15d-random-walk";
  config.motion_error_model_qualified = true;
  config.allow_uncalibrated_inputs = true;
  config.retain_graph_on_full_pose_staleness = true;
  config.solve_budget_ms = 100000;
  return config;
}

loc::GraphSourceQualification Qualification() {
  loc::GraphSourceQualification result;
  result.uncertainty_model_id = "synthetic-independent-detector-noise";
  result.independence_evidence_id = "disjoint-synthetic-errors-fixed-map-and-extrinsics";
  result.quality_valid = true;
  result.independent_of_local = true;
  result.independent_of_graph_sources = true;
  return result;
}

loc::GraphOdometrySample Odometry(int node) {
  loc::GraphOdometrySample sample;
  sample.odometry.timestamp = {kStart + node * kStep, loc::ClockDomain::kUnixEpoch};
  sample.receive_time = sample.odometry.timestamp;
  sample.odometry.epoch = {"synthetic-native-boundary", 1};
  sample.odometry.parent_frame = {"odom"};
  sample.odometry.child_frame = {"base"};
  sample.odometry.pose.translation =
      Eigen::Vector3d(.5 * node, .1 * std::sin(.2 * node), 0);
  sample.odometry.pose.rotation = Eigen::AngleAxisd(.03 * node, Eigen::Vector3d::UnitZ());
  sample.sequence = node + 1;
  sample.covariance = Matrix15d::Identity() * (.01 + .001 * node);
  sample.covariance_model_valid = true;
  return sample;
}

loc::Pose Truth(int node) {
  loc::Pose pose;
  const Eigen::Quaterniond map_from_odom(
      Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()));
  pose.translation =
      Eigen::Vector3d(100, 20, 1) +
      map_from_odom * Eigen::Vector3d(.5 * node, .1 * std::sin(.2 * node), 0);
  pose.rotation = map_from_odom * Eigen::Quaterniond(Eigen::AngleAxisd(
                                      .03 * node, Eigen::Vector3d::UnitZ()));
  return pose;
}

loc::GlobalObservationContext Context(int node, const std::string& source) {
  const auto config = Config();
  loc::GlobalObservationContext context;
  context.measurement_time = Odometry(node).odometry.timestamp;
  context.receive_time = context.measurement_time;
  context.sequence = node + 1;
  context.source = source;
  context.epoch = Odometry(node).odometry.epoch;
  context.reference = config.reference;
  context.map_content_id = config.map_content_id;
  context.map_frame = config.map_frame;
  context.base_frame = config.base_frame;
  context.provenance.acquisition_id = "synthetic-scene";
  context.provenance.preparation_id = "synthetic-functional-v1";
  context.provenance.odometry_initialization = loc::ObservationDependency::kAbsent;
  context.provenance.odometry_deskew = loc::ObservationDependency::kAbsent;
  context.provenance.shared_local_scans = loc::ObservationDependency::kAbsent;
  context.provenance.gnss_map_construction = loc::ObservationDependency::kAbsent;
  return context;
}

loc::GlobalMapObservation Map(int node, bool projected) {
  loc::GlobalMapObservation observation;
  observation.context = Context(node, projected ? "scan-to-map-projected" : "scan-to-map");
  observation.map_from_base = Truth(node);
  observation.covariance = Eigen::Matrix<double, 6, 6>::Identity() * .0004;
  observation.covariance_source = "synthetic-map-covariance";
  if (projected) {
    observation.kind = loc::GlobalMapObservation::Kind::kProjectedPose;
    observation.projection = Eigen::Matrix<double, 5, 6>::Zero();
    observation.projection.rightCols<5>().setIdentity();
    observation.projected_covariance = Eigen::Matrix<double, 5, 5>::Identity() * .0004;
  }
  return observation;
}

loc::GlobalMapObservation Marker(int node, bool ambiguous) {
  const auto config = Config();
  loc::KnownMarker marker;
  marker.id = "surveyed-door";
  marker.reference = config.reference;
  marker.map_content_id = config.map_content_id;
  marker.map_frame = config.map_frame;
  marker.marker_frame = {"door"};
  marker.map_from_marker.translation = {104, 22, 2};
  loc::MarkerExtrinsics extrinsics;
  extrinsics.calibration_id = config.reference.calibration_id;
  extrinsics.base_frame = config.base_frame;
  extrinsics.sensor_frame = {"camera"};
  extrinsics.base_from_sensor.translation = {.2, .1, .3};
  extrinsics.base_from_sensor.rotation =
      Eigen::AngleAxisd(.1, Eigen::Vector3d::UnitY());
  loc::MarkerDetection detection;
  detection.context = Context(node, "known-marker");
  detection.marker_id = marker.id;
  detection.marker_frame = marker.marker_frame;
  detection.sensor_frame = extrinsics.sensor_frame;
  detection.ambiguous = ambiguous;
  const auto transform = [](const loc::Pose& pose) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear() = pose.rotation.toRotationMatrix();
    t.translation() = pose.translation;
    return t;
  };
  const Eigen::Isometry3d measured =
      (transform(Truth(node)) * transform(extrinsics.base_from_sensor)).inverse() *
      transform(marker.map_from_marker);
  detection.sensor_from_marker.translation = measured.translation();
  detection.sensor_from_marker.rotation = Eigen::Quaterniond(measured.linear());
  // Survey and extrinsics are deterministic ONLY in this synthetic experiment.
  loc::MarkerJointCovariance covariance = loc::MarkerJointCovariance::Zero();
  covariance.block<6, 6>(6, 6) = Eigen::Matrix<double, 6, 6>::Identity() * .0004;
  loc::GlobalMapObservation result;
  Require(loc::BuildMarkerObservation(marker, detection, extrinsics, covariance, &result));
  return result;
}

int LossEnd(const std::string& scenario) {
  if (scenario == "joint_loss_short") return 9;
  if (scenario == "joint_loss_equal") return 12;
  if (scenario == "joint_loss_long") return 36;
  return 18;
}

bool WindowLoss(const std::string& scenario) {
  return scenario == "joint_loss_short" || scenario == "joint_loss_equal" ||
         scenario == "joint_loss_long";
}

int MapLossEnd(const std::string& scenario) {
  if (scenario == "map_loss_30s") return 66;
  if (scenario == "map_loss_60s") return 126;
  if (scenario == "map_loss_120s") return 246;
  return 0;
}

void Run(const std::string& scenario, const std::filesystem::path& output,
         const std::vector<std::vector<std::string>>& field_profile) {
  Require(std::filesystem::create_directory(output),
          "scenario output must be new");
  std::ofstream events(output / "decisions.csv"), states(output / "states.csv");
  events << std::setprecision(17)
         << "node,decision_id,source,measurement_ns,receive_ns,status,reason,expected,"
            "updated,correction_id,innovation_available,innovation_unavailable_reason,"
            "residual,covariance,nis,gate\n";
  states << std::setprecision(17)
         << "node,timestamp_ns,pose_available,full_pose_fresh,odom_fresh,trust,"
            "diagnostic_only,assessment,correction_id,x,y,z,truth_x,truth_y,"
            "truth_z,"
            "error_m,alignment_covariance_trace,submitted,rejected,updates,"
            "retained,"
            "input_unchanged,update_ms,qx,qy,qz,qw,truth_qx,truth_qy,truth_qz,"
            "truth_qw,rotation_error_rad,alignment_covariance,"
            "absolute_update_translation_m,absolute_update_rotation_rad\n";
  loc::FixedLagGlobalAlignment owner;
  Require(owner.Configure(Config()));
  loc::GraphOdometrySample previous;
  uint64_t written = 0;
  const bool burst = scenario == "gnss_outlier_burst" ||
                     scenario == "gnss_outlier_burst_omitted";
  const int map_loss_end = MapLossEnd(scenario);
  const int nodes = map_loss_end                             ? map_loss_end + 12
                    : scenario == "joint_loss_long" || burst ? 48
                                                             : kNodes;
  for (int node = 0; node < nodes; ++node) {
    const auto started = std::chrono::steady_clock::now();
    auto sample = Odometry(node);
    if (scenario == "joint_loss_local_drift_reentry" || WindowLoss(scenario)) {
      sample.odometry.pose.translation.x() +=
          .02 * std::max(0, std::min(node - 5, LossEnd(scenario) - 6));
    }
    const auto original = sample.odometry.pose;
    loc::GraphMotionConnection connection;
    if (node) {
      connection.epoch = sample.odometry.epoch;
      connection.start_time = previous.odometry.timestamp;
      connection.end_time = sample.odometry.timestamp;
      connection.start_sequence = previous.sequence;
      connection.end_sequence = sample.sequence;
      connection.cross_covariance = previous.covariance;
      connection.sources = {{"imu", previous.sequence},
                            {"imu", sample.sequence}};
    }
    Require(owner.AddOdometry(sample, node ? &connection : nullptr));
    loc::FixedLagAlignmentResult before_observations;
    Require(owner.GetResult(sample.receive_time, &before_observations));
    const auto record = [&](const loc::Status& status, const std::string& expected) {
      std::vector<loc::GraphObservationDecision> decisions;
      Require(owner.GetDecisions(&decisions));
      Require(!decisions.empty() && decisions.back().observation.decision_id == ++written,
              "missing terminal graph decision");
      const auto& d = decisions.back().observation;
      const bool verification = status.message().find("RELOCALIZATION_VERIFYING") != std::string::npos;
      const bool accepted = expected == "accepted_or_verifying" ? status.ok() || verification :
                            expected == "retained_unanchored" ? status.ok() && !d.alignment_updated :
                            expected == "rejected" ? !status.ok() && !d.alignment_updated :
                            status.ok();
      events << node << ',' << d.decision_id << ',' << d.context.source << ','
             << d.context.measurement_time.nanoseconds << ','
             << d.context.receive_time.nanoseconds << ',' << static_cast<int>(status.code()) << ',';
      csv::Text(events, status.message());
      events << ',' << expected << ',' << d.alignment_updated << ',' << d.correction_id
             << ',' << d.innovation.available << ',';
      csv::Text(events, d.innovation.unavailable_reason);
      events << ',';
      Matrix(events, d.innovation.residual);
      events << ',';
      Matrix(events, d.innovation.covariance);
      events << ',' << d.innovation.nis << ',' << d.innovation.gate_threshold << '\n';
      events.flush();
      Require(accepted, scenario + " node " + std::to_string(node) + ": " + status.message());
    };
    bool full_map = true, projected = false, gnss = true, marker = false;
    bool bad_gnss = false, bad_quality = false, ambiguous_marker = false;
    if (scenario == "indoor_map_no_gnss") gnss = false;
    if (scenario == "indoor_marker_no_map_gnss") {
      full_map = false; gnss = false; marker = true;
    }
    if (scenario == "map_degenerate_gnss_available" && node >= 6 && node < 18) {
      full_map = false; projected = true;
    }
    if (scenario == "gnss_degraded_map_available" && node >= 6 && node < 18)
      bad_quality = true;
    if (scenario == "indoor_corridor_marker") {
      gnss = false;
      if (node >= 6) {
        full_map = false;
        projected = true;
        marker = node % 3 == 0;
      }
    }
    if (scenario == "joint_degradation_marker" && node >= 6) {
      full_map = false;
      projected = true;
      bad_quality = true;
      marker = node % 3 == 0;
    }
    if (scenario == "gnss_only_after_anchor" && node >= 6) full_map = false;
    if (map_loss_end && node >= 6 && node < map_loss_end) full_map = false;
    if (scenario == "field_sparse_map_gnss") {
      full_map = field_profile[node][1] == "1";
      gnss = field_profile[node][2] == "1";
    }
    if (scenario == "outdoor_indoor_outdoor") {
      marker = node >= 6 && node < 12;
      if (node >= 6 && node < 12) gnss = false;
      if (node >= 12 && node < 18) { full_map = false; projected = true; }
    }
    if ((scenario == "joint_loss_reentry" ||
         scenario == "joint_loss_local_drift_reentry" ||
         WindowLoss(scenario)) &&
        node >= 6 && node < LossEnd(scenario)) {
      full_map = false; gnss = false;
    }
    if (burst && node >= 6 && node < 36) {
      bad_gnss = scenario == "gnss_outlier_burst";
      gnss = bad_gnss;
    }
    if (scenario == "fault_rejection") {
      bad_gnss = node == 10;
      marker = node == 12 || node == 13;
      ambiguous_marker = node == 12;
    }
    if (scenario == "gnss_only_cold_start") full_map = false;
    if (scenario == "delayed_map") {
      gnss = false;
      full_map = node >= 2;
    }
    if (full_map || projected) {
      const int observed = scenario == "delayed_map" ? node - 2 : node;
      auto observation = Map(observed, projected);
      observation.context.receive_time = sample.receive_time;
      record(owner.SubmitMapObservation(observation, Qualification()),
             projected ? "accepted" : "accepted_or_verifying");
    }
    if (marker) {
      record(owner.SubmitMapObservation(Marker(node, ambiguous_marker), Qualification()),
             ambiguous_marker ? "rejected" : "accepted_or_verifying");
    }
    if (gnss) {
      loc::GlobalHorizontalObservation observation;
      observation.context = Context(node, "gnss-xy");
      observation.point_in_base = {1, -.2, .3};
      observation.position = (Truth(node).translation +
          Truth(node).rotation * observation.point_in_base).head<2>();
      if (bad_gnss) observation.position += Eigen::Vector2d(50, -30);
      observation.covariance = Eigen::Matrix2d::Identity() * .01;
      observation.covariance_source = "synthetic-gnss-noise";
      auto qualification = Qualification();
      qualification.quality_valid = !bad_quality;
      record(owner.SubmitHorizontalObservation(observation, qualification),
             bad_gnss || bad_quality ? "rejected" :
             (node < 2 || scenario == "gnss_only_cold_start") ? "retained_unanchored" : "accepted");
    }
    loc::FixedLagAlignmentResult result;
    Require(owner.GetResult(sample.receive_time, &result));
    const bool unchanged = sample.odometry.pose.translation == original.translation &&
                           sample.odometry.pose.rotation.coeffs() == original.rotation.coeffs();
    Require(unchanged, "absolute graph mutated ODOM input");
    Require(result.trust_level != loc::LocalizationTrustLevel::kQualified,
            "uncalibrated scenario acquired qualified trust");
    uint64_t updates = 0, retained = 0;
    for (const auto& source : result.sources) {
      Require(source.submitted == source.alignment_updates + source.retained + source.rejected,
              "source decision ledger mismatch");
      updates += source.alignment_updates;
      retained += source.retained;
    }
    const auto true_pose = Truth(node);
    const auto& truth = true_pose.translation;
    const double error = result.estimate ?
        (result.estimate->map_from_base.translation - truth).norm() : 0;
    states << node << ',' << result.timestamp.nanoseconds << ',' << result.estimate.has_value()
           << ',' << result.full_pose_evidence_fresh << ',' << result.odometry_fresh
           << ',' << static_cast<int>(result.trust_level) << ',' << result.diagnostic_only
           << ',' << static_cast<int>(result.assessment.code()) << ','
           << (result.estimate ? result.estimate->correction_id : 0) << ',';
    if (result.estimate) {
      const auto& p = result.estimate->map_from_base.translation;
      states << p.x() << ',' << p.y() << ',' << p.z();
    } else states << ",,";
    states << ',' << truth.x() << ',' << truth.y() << ',' << truth.z() << ',';
    if (result.estimate) states << error;
    states << ',';
    if (result.estimate) states << result.estimate->alignment_covariance.trace();
    states << ',' << result.submitted_observations << ','
           << result.rejected_observations << ',' << updates << ',' << retained
           << ',' << unchanged << ','
           << std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - started)
                  .count()
           << ',';
    if (result.estimate) {
      const auto& rotation = result.estimate->map_from_base.rotation;
      states << rotation.x() << ',' << rotation.y() << ',' << rotation.z()
             << ',' << rotation.w();
    } else {
      states << ",,,";
    }
    const auto& true_rotation = true_pose.rotation;
    states << ',' << true_rotation.x() << ',' << true_rotation.y() << ','
           << true_rotation.z() << ',' << true_rotation.w() << ',';
    if (result.estimate)
      states << Eigen::AngleAxisd(true_rotation.conjugate() *
                                  result.estimate->map_from_base.rotation)
                    .angle();
    states << ',';
    if (result.estimate) Matrix(states, result.estimate->alignment_covariance);
    states << ',';
    if (result.estimate && before_observations.estimate) {
      const auto& before = before_observations.estimate->map_from_base;
      const auto& after = result.estimate->map_from_base;
      states << (after.translation - before.translation).norm() << ','
             << Eigen::AngleAxisd(before.rotation.conjugate() * after.rotation)
                    .angle();
    } else {
      states << ',';
    }
    states << '\n';
    states.flush();
    previous = sample;
  }
  Require(events.good() && states.good(), "incomplete scenario journals");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2 && argc != 3)
      throw std::runtime_error(
          "usage: localization_scenarios NEW_OUTPUT [FIELD_PROFILE]");
    const auto field_profile = csv::ReadRows(
        argc == 3 ? argv[2]
                  : "test/integration/map/testdata/field_sparse_map_gnss.csv",
        "node,map_available,gnss_available", 3);
    Require(field_profile.size() == kNodes, "field profile requires 24 nodes");
    for (int node = 0; node < kNodes; ++node) {
      const auto& row = field_profile[node];
      Require(row[0] == std::to_string(node) &&
                  (row[1] == "0" || row[1] == "1") &&
                  (row[2] == "0" || row[2] == "1"),
              "invalid field availability profile");
    }
    const std::filesystem::path output(argv[1]);
    Require(std::filesystem::create_directory(output), "output must be new");
    for (const auto* scenario : {"indoor_map_no_gnss",
                                 "indoor_marker_no_map_gnss",
                                 "outdoor_gnss_map",
                                 "map_degenerate_gnss_available",
                                 "gnss_degraded_map_available",
                                 "indoor_corridor_marker",
                                 "joint_degradation_marker",
                                 "gnss_only_after_anchor",
                                 "outdoor_indoor_outdoor",
                                 "joint_loss_reentry",
                                 "joint_loss_local_drift_reentry",
                                 "fault_rejection",
                                 "gnss_only_cold_start",
                                 "delayed_map",
                                 "field_sparse_map_gnss",
                                 "joint_loss_short",
                                 "joint_loss_equal",
                                 "joint_loss_long",
                                 "gnss_outlier_burst",
                                 "gnss_outlier_burst_omitted",
                                 "map_loss_30s",
                                 "map_loss_60s",
                                 "map_loss_120s"}) {
      Run(scenario, output / scenario, field_profile);
      std::cout << scenario << " completed\n";
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "localization_scenarios: " << e.what() << '\n';
    return 1;
  }
}
