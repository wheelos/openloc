// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <stdexcept>
#include <string>

#include "candidates/common/types.h"
#include "openloc/localization/health/localization_health.h"

namespace openloc::replay {

struct NativeHealthSnapshot {
  localization::LocalizationAssessment assessment;
  localization::LocalizationHealthStatus status;
  bool assessment_accepted = false;
  bool local_relative_valid = false;
  bool diagnostic_global_initialized = false;
  bool health_trusted = false;
};

// Independent supervisor execution, not an independent physical reference.
// Raw replay has no qualified uncertainty/continuity/convergence evidence.
// A nominal native covariance or diagnostic global correction cannot supply it.
class NativeHealthDiagnostic {
 public:
  explicit NativeHealthDiagnostic(const std::filesystem::path& output)
      : file_(output / "health.csv"), journal_(&file_) {
    Initialize();
  }

  explicit NativeHealthDiagnostic(std::ostream& journal) : journal_(&journal) {
    Initialize();
  }

  NativeHealthSnapshot Update(const localization::unified::LocalState& state,
                              const localization::unified::Result& integrity,
                              int64_t timestamp_ns, int64_t receive_ns,
                              bool diagnostic_global_initialized = false) {
    namespace loc = localization;
    if (timestamp_ns <= 0 || receive_ns <= 0)
      throw std::runtime_error("health requires positive replay timestamps");
    const double stamp = static_cast<double>(timestamp_ns) * 1e-9;
    const double receive = static_cast<double>(receive_ns) * 1e-9;
    NativeHealthSnapshot snapshot;
    snapshot.local_relative_valid = state.valid && integrity.ok();
    snapshot.diagnostic_global_initialized = diagnostic_global_initialized;
    auto& assessment = snapshot.assessment;
    assessment.mutable_header()->set_timestamp_sec(stamp);
    assessment.mutable_header()->set_module_name("native_health_diagnostic");
    assessment.mutable_header()->set_sequence_num(++sequence_);
    assessment.set_timestamp_sec(stamp);
    assessment.set_sequence(sequence_);
    assessment.set_session_id(state.epoch.session.empty()
                                  ? "native-replay-uninitialized"
                                  : state.epoch.session + ":" +
                                        std::to_string(state.epoch.generation));
    assessment.set_odom_generation(state.epoch.generation);
    assessment.set_local_measurement_time(state.stamp.time);
    assessment.set_estimator_running(true);
    assessment.set_estimator_converged(false);
    assessment.set_pose_valid(snapshot.local_relative_valid);
    assessment.set_velocity_valid(snapshot.local_relative_valid);
    assessment.set_heading_valid(snapshot.local_relative_valid);
    assessment.set_local_consistency_valid(integrity.ok());
    assessment.set_output_continuous(false);
    // No marginal is relabeled as qualified output uncertainty. Omit standard
    // deviations entirely instead of publishing zero or fabricated inflation.
    assessment.set_covariance_valid(false);
    assessment.set_global_pose_valid(false);
    assessment.set_georeference_valid(false);
    assessment.set_map_alignment_valid(false);
    assessment.set_lane_level_valid(false);
    assessment.set_recovery_available(false);
    auto* source = assessment.add_source_health();
    source->set_source_name("native_local_integrity");
    source->set_is_healthy(integrity.ok());
    source->set_last_seen_timestamp_sec(stamp);
    source->set_message(integrity.message);

    snapshot.assessment_accepted =
        supervisor_.UpdateAssessment(assessment, receive);
    // Retain real numeric output even on an integrity failure so the separate
    // checker can detect nonfinite values, bad quaternions and kinematic
    // faults.
    if (snapshot.assessment_accepted && std::isfinite(state.stamp.time) &&
        state.stamp.time > 0.0) {
      loc::LocalizationEstimate estimate;
      estimate.mutable_header()->set_timestamp_sec(state.stamp.time);
      estimate.mutable_header()->set_sequence_num(state.stamp.sequence);
      estimate.set_measurement_time(state.stamp.time);
      auto* pose = estimate.mutable_pose();
      SetVector(state.position, pose->mutable_position());
      SetVector(state.velocity, pose->mutable_linear_velocity());
      SetVector(state.angular_velocity, pose->mutable_angular_velocity());
      auto* rotation = pose->mutable_orientation();
      rotation->set_qx(state.orientation.x());
      rotation->set_qy(state.orientation.y());
      rotation->set_qz(state.orientation.z());
      rotation->set_qw(state.orientation.w());
      supervisor_.UpdatePose(estimate, receive);
    }
    snapshot.status = supervisor_.Evaluate(receive);
    Write(snapshot, state, integrity, timestamp_ns, receive_ns);
    return snapshot;
  }

 private:
  void Initialize() {
    if (!*journal_) throw std::runtime_error("cannot open health output");
    supervisor_.ResetSession("native-replay-uninitialized");
    *journal_
        << std::setprecision(17)
        << "timestamp_ns,receive_ns,local_relative_valid,integrity_reason,"
           "covariance_model_valid,uncertainty_qualified,"
           "assessment_covariance_valid,assessment_accepted,"
           "diagnostic_global_initialized,availability_state,"
           "capabilities,active_reasons,latched_reasons,primary_reason,"
           "sequence,transition_id,session_id,supervisor_session_id,"
           "health_trusted,qualification_reason\n";
    if (!*journal_) throw std::runtime_error("cannot write health header");
  }

  static void SetVector(const Eigen::Vector3d& value,
                        localization::Point3D* output) {
    output->set_x(value.x());
    output->set_y(value.y());
    output->set_z(value.z());
  }

  void Text(const std::string& value) {
    *journal_ << '"';
    for (char c : value) {
      if (c == '"') *journal_ << '"';
      *journal_ << c;
    }
    *journal_ << '"';
  }

  void Write(const NativeHealthSnapshot& snapshot,
             const localization::unified::LocalState& state,
             const localization::unified::Result& integrity,
             int64_t timestamp_ns, int64_t receive_ns) {
    const auto& status = snapshot.status;
    *journal_ << timestamp_ns << ',' << receive_ns << ','
              << snapshot.local_relative_valid << ','
              << localization::unified::ReasonName(integrity.reason) << ','
              << state.covariance_model_valid << ','
              << state.uncertainty_qualified << ','
              << snapshot.assessment.covariance_valid() << ','
              << snapshot.assessment_accepted << ','
              << snapshot.diagnostic_global_initialized << ','
              << localization::AvailabilityState_Name(
                     status.availability_state())
              << ',' << status.capabilities() << ',' << status.active_reasons()
              << ',' << status.latched_reasons() << ','
              << localization::HealthReason_Name(status.primary_reason()) << ','
              << status.sequence() << ',' << status.transition_id() << ',';
    Text(status.session_id());
    *journal_ << ',';
    Text(status.supervisor_session_id());
    *journal_ << ',' << snapshot.health_trusted
              << ",raw_native_uncertainty_unqualified\n";
    if (!*journal_) throw std::runtime_error("cannot write health output");
  }

  std::ofstream file_;
  std::ostream* journal_;
  localization::LocalizationHealth supervisor_;
  uint64_t sequence_ = 0;
};

}  // namespace openloc::replay
