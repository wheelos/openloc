// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <variant>

#include "candidates/local/local_lio.h"
#include "openloc/localization/global/diagnostic_alignment.h"
#include "openloc/localization/map/scan_to_map.h"
#include "pcl/io/pcd_io.h"
#include "test/integration/map/native_causal_map_search.h"
#include "test/integration/map/native_gnss_diagnostic.h"
#include "test/integration/map/replay_csv.h"

namespace openloc::replay {

template <typename Derived>
std::string Flatten(const Eigen::MatrixBase<Derived>& value) {
  std::ostringstream stream;
  stream << value.rows() << 'x' << value.cols() << ':'
         << std::setprecision(17);
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    for (Eigen::Index column = 0; column < value.cols(); ++column) {
      if (row != 0 || column != 0) stream << ';';
      stream << value(row, column);
    }
  }
  return stream.str();
}

struct NativeGlobalOdometryFault {
  enum class Kind { kNone, kDrift, kStep };
  Kind kind = Kind::kNone;
  int64_t start_ns = 0;
  Eigen::Vector2d value = Eigen::Vector2d::Zero();
};

// Replay-side scheduling/deskew only. Neither the seed nor global corrections
// are exposed to the local estimator.
class NativeMapDiagnostic {
 public:
  NativeMapDiagnostic(const std::filesystem::path& bundle,
                      const std::filesystem::path& output,
                      const std::string& expected_digests,
                      const std::filesystem::path& gnss_bundle = {},
                      const std::string& gnss_digests = {},
                      int64_t holdback_ns = 0, bool async_recovery = false,
                      bool level_odom_gnss_bootstrap = false,
                      NativeGlobalOdometryFault odometry_fault = {},
                      std::pair<int64_t, int64_t> map_holdout = {},
                      bool profile_map_timing = false,
                      bool fixed_gain_update = false,
                      bool structured_closure = false)
      : matches_(output / "map.csv"),
        global_(output / "global.csv"),
        decisions_(output / "global_decisions.csv"),
        measurements_(output / "graph_measurements.csv"),
        odometry_fault_(std::move(odometry_fault)),
        admission_(output / "global_admission.csv"),
        holdback_ns_(holdback_ns),
        async_recovery_(async_recovery),
        map_holdout_(map_holdout) {
    namespace loc = localization;
    measurements_ << std::setprecision(17)
                  << "kind,timestamp_ns,receive_ns,sequence,px,py,pz,qx,qy,qz,qw,"
                     "projection,covariance,lever_x,lever_y,lever_z,ambiguous\n";
    if (!measurements_) throw std::runtime_error("cannot open measurement journal");
    if (level_odom_gnss_bootstrap && gnss_bundle.empty())
      throw std::runtime_error("horizontal bootstrap requires GNSS input");
    if (odometry_fault_.kind != NativeGlobalOdometryFault::Kind::kNone &&
        (odometry_fault_.start_ns <= 0 || !odometry_fault_.value.allFinite()))
      throw std::runtime_error("invalid global ODOM fault injection");
    if (holdback_ns_ < 0 || holdback_ns_ > 500000000)
      throw std::runtime_error("global holdback must be in [0,500000000] ns");
    if (map_holdout_.first &&
        (bundle.empty() || map_holdout_.second <= map_holdout_.first))
      throw std::runtime_error("map holdout requires a map and increasing interval");
    const bool causal_search =
        !bundle.empty() && std::filesystem::exists(bundle / "recovery.csv");
    const bool seed_time_present =
        !bundle.empty() && std::filesystem::exists(bundle / "seed_time.csv");
    if (causal_search &&
        (seed_time_present || std::filesystem::exists(bundle / "seed.csv")))
      throw std::runtime_error("causal search cannot consume an offline seed");
    if (!bundle.empty() &&
        (expected_digests.size() != (seed_time_present ? 320 : 256) ||
         expected_digests.find_first_not_of("0123456789abcdef") !=
             std::string::npos))
      throw std::runtime_error("four explicit SHA256 map input pins required");
    size_t digest_index = 0;
    if (!bundle.empty())
      for (const auto* name :
           {"global.pcd", "identity.csv",
            causal_search ? "recovery.csv" : "seed.csv", "window.csv"}) {
        if (Sha256(bundle / name) != expected_digests.substr(digest_index, 64))
          throw std::runtime_error(std::string("map input hash mismatch: ") +
                                   name);
        digest_index += 64;
      }
    if (seed_time_present) {
      if (Sha256(bundle / "seed_time.csv") != expected_digests.substr(256, 64))
        throw std::runtime_error("map input hash mismatch: seed_time.csv");
      const auto policy = ReadRows((bundle / "seed_time.csv").string(),
                                   "reference_ns,maximum_offset_ns", 2);
      if (policy.size() != 1)
        throw std::runtime_error("one explicit seed time policy required");
      seed_reference_ns_ = replay::Time(policy[0][0]);
      seed_maximum_offset_ns_ = std::stoll(policy[0][1]);
      if (seed_maximum_offset_ns_ < 0)
        throw std::runtime_error("negative seed time tolerance");
    }
    if (!gnss_bundle.empty()) {
      if (gnss_digests.size() != 192 ||
          gnss_digests.find_first_not_of("0123456789abcdef") !=
              std::string::npos)
        throw std::runtime_error("three explicit GNSS SHA256 pins required");
      digest_index = 0;
      for (const auto* name : {"events.csv", "config.csv", "identity.csv"}) {
        if (Sha256(gnss_bundle / name) != gnss_digests.substr(digest_index, 64))
          throw std::runtime_error(std::string("GNSS input hash mismatch: ") +
                                   name);
        digest_index += 64;
      }
      gnss_ = std::make_unique<NativeGnssDiagnostic>(gnss_bundle, output);
    }
    const auto rows = ReadRows(
        ((bundle.empty() ? gnss_bundle : bundle) / "identity.csv").string(),
        "map_id,map_version,map_content_id,calibration_id,map_frame,"
        "odom_frame,base_frame",
        7);
    if (rows.size() != 1) throw std::runtime_error("one map identity required");
    if (!bundle.empty() && !gnss_bundle.empty() &&
        ReadRows((gnss_bundle / "identity.csv").string(),
                 "map_id,map_version,map_content_id,calibration_id,map_frame,"
                 "odom_frame,base_frame",
                 7) != rows)
      throw std::runtime_error("GNSS/map reference identity mismatch");
    const auto& row = rows.front();
    config_.reference = {row[0], row[1], row[3]};
    config_.reference.geodetic.state =
        loc::GeodeticReferenceState::kDiagnosticOnly;
    config_.map_content_id = row[2];
    config_.map_frame = {row[4]};
    config_.odom_frame = {row[5]};
    config_.base_frame = {row[6]};
    config_.enable_horizontal_bootstrap = level_odom_gnss_bootstrap;
    config_.assume_gravity_aligned_odom = level_odom_gnss_bootstrap;
    config_.use_fixed_gain_update = fixed_gain_update;
    Require(owner_.Configure(config_));
    if (level_odom_gnss_bootstrap) {
      horizontal_.open(output / "horizontal.csv");
      if (!horizontal_)
        throw std::runtime_error("cannot open horizontal output");
      horizontal_ << std::setprecision(17)
                  << "timestamp_ns,receive_ns,alignment_initialized,"
                     "correction_id,x,y,alignment_x,alignment_y,alignment_yaw,"
                     "last_observation_ns,odom_fresh,horizontal_fresh,"
                     "diagnostic_only,global_trusted,target_graph_available\n";
    }
    if (!bundle.empty()) {
      if (!causal_search) {
        const auto seed =
            ReadRows((bundle / "seed.csv").string(), "x,y,z,qx,qy,qz,qw", 7);
        if (seed.size() != 1)
          throw std::runtime_error("one map-side seed required");
        for (int i = 0; i < 3; ++i) seed_.translation()[i] = Number(seed[0][i]);
        Eigen::Quaterniond q(Number(seed[0][6]), Number(seed[0][3]),
                             Number(seed[0][4]), Number(seed[0][5]));
        if (std::abs(q.norm() - 1.0) > 1e-6)
          throw std::runtime_error("invalid map-side seed quaternion");
        seed_.linear() = q.toRotationMatrix();
      }
      loc::map::ScanToMapConfig matcher_config;
      matcher_config.map_frame = row[4];
      matcher_config.base_frame = row[6];
      matcher_config.cloud_frame = row[6];
      matcher_config.calibration_id = row[3];
      matcher_config.enable_icp_profiling = profile_map_timing;
      matcher_config.algorithm =
          loc::map::ScanMatchingAlgorithm::kPointToPlaneIcp;
      matcher_config.retain_joint_error_support = structured_closure;
      matcher_config.condition_on_fixed_target = structured_closure;
      const auto window =
          ReadRows((bundle / "window.csv").string(), "radius_m,cell_size_m", 2);
      if (window.size() != 1)
        throw std::runtime_error("one bounded map window policy required");
      matcher_config.local_window_radius_m = Number(window[0][0]);
      matcher_config.window_cell_size_m = Number(window[0][1]);
      if (matcher_config.local_window_radius_m < 5 ||
          matcher_config.local_window_radius_m > 30 ||
          matcher_config.window_cell_size_m < 0.5 ||
          matcher_config.window_cell_size_m > 2)
        throw std::runtime_error(
            "map window radius/cell outside diagnostic bounds");
      matcher_ = std::make_unique<loc::map::ScanToMapLocalizer>(matcher_config);
      if (profile_map_timing) {
        map_profile_.open(output / "map_profile.csv");
        if (!map_profile_)
          throw std::runtime_error("cannot open map timing profile");
        map_profile_ << std::setprecision(17)
                     << "sequence,iterations,total_ms,cached_window_build_ms,"
                        "cached_target_voxelization_ms,cached_target_kdtree_ms,"
                        "cached_target_normal_ms,cached_normal_search_ms,"
                        "cached_normal_fit_ms,icp_search_ms,"
                        "icp_linearization_ms,icp_solver_ms\n";
      }
      loc::map::PointCloud::Ptr cloud(new loc::map::PointCloud);
      if (pcl::io::loadPCDFile((bundle / "global.pcd").string(), *cloud) != 0)
        throw std::runtime_error("cannot load verified prior map");
      loc::map::MapMetadata metadata;
      metadata.map_id = row[0];
      metadata.version = row[1];
      metadata.content_id = row[2];
      metadata.map_frame = row[4];
      metadata.source_cloud_frame = row[6];
      metadata.reference_state = loc::map::MapReferenceState::kDiagnosticOnly;
      // PCD/identity/seed are checked against explicit caller pins above; the
      // orchestrator additionally validates selected original metadata/frames.
      metadata.integrity_verified = true;
      std::string error;
      if (!matcher_->Configure(metadata, cloud, &error))
        throw std::runtime_error(error);
      if (causal_search)
        causal_search_ = std::make_unique<NativeCausalMapSearch>(
            bundle / "recovery.csv", output,
            loc::unified::FieldIdentity{row[0], row[1], row[4], row[3]},
            *cloud);
      if (causal_search) {
        tracking_journal_.open(output / "map_tracking.csv");
        if (!tracking_journal_)
          throw std::runtime_error("cannot open map tracking journal");
        tracking_journal_ << "timestamp_ns,state,event,last_full_pose_ns,"
                             "generation,diagnostic_only\n";
      }
      if (causal_search && async_recovery_) {
        recovery_journal_.open(output / "causal_worker.csv");
        if (!recovery_journal_)
          throw std::runtime_error("cannot open causal worker journal");
        recovery_journal_ << "sequence,reference_ns,processing_ns,status\n";
      }
    }
    if (!matches_ || !global_ || !decisions_ || !admission_)
      throw std::runtime_error("cannot open map/global outputs");
    matches_ << std::setprecision(17)
             << "sequence,timestamp_ns,receive_ns,valid,reason,detail,"
                "observed_rank,source_overlap,target_overlap,score,"
                "processing_ms,global_status,global_message,map_window_points,"
                "registration_source_points,registration_target_points\n";
    global_ << std::setprecision(17)
            << "timestamp_ns,receive_ns,alignment_initialized,correction_id,"
               "x,y,z,qx,qy,qz,qw,odom_fresh,full_pose_fresh,"
               "diagnostic_only,global_trusted,target_graph_available\n";
    decisions_ << "decision_id,timestamp_ns,receive_ns,sequence,status,message,"
                  "alignment_updated,correction_id,covariance_source,"
                  "odometry_deskew,shared_local_scans,target_graph_available,"
                  "source,gnss_map_construction,innovation_available,"
                  "innovation_unavailable_reason,observed_dimension,"
                  "prior_state_frame,prior_state_dimension,"
                  "pre_update_residual,innovation_covariance,nis,"
                  "prior_pose_xyz_qxyzw,prior_covariance,gate_threshold,"
                  "residual_basis,covariance_model,covariance_decomposition_available,"
                  "innovation_prior_contribution,innovation_observation_contribution,"
                  "innovation_local_contribution\n";
    admission_ << "source,sequence,measurement_ns,source_receive_ns,"
                  "enqueue_processing_ns,submission_processing_ns,"
                  "holdback_ns,status,message\n";
    if (odometry_fault_.kind != NativeGlobalOdometryFault::Kind::kNone) {
      odometry_injection_.open(output / "global_odom_injection.csv");
      if (!odometry_injection_)
        throw std::runtime_error("cannot open global ODOM injection journal");
      odometry_injection_
          << "timestamp_ns,sequence,kind,offset_x_m,offset_y_m\n";
    }
    seed_evidence_.open(output / "seed_application.csv");
    if (!seed_evidence_) throw std::runtime_error("cannot open seed evidence");
    seed_evidence_
        << "sequence,reference_ns,processing_receive_ns,"
           "hypothesis_only,local_initialization_used,"
           "seed_reference_ns,seed_offset_ns,maximum_offset_ns,reason\n";
  }

  void AddOdometry(const localization::unified::LocalState& state,
                   int64_t stamp, int64_t receive) {
    if (!state.valid) return;
    history_.push_back(state);
    while (history_.size() > 2 &&
           state.stamp.time - history_.front().stamp.time > 1.0)
      history_.pop_front();
    localization::OdometrySample sample;
    sample.timestamp = Time(stamp);
    sample.epoch = {state.epoch.session, state.epoch.generation};
    sample.parent_frame = config_.odom_frame;
    sample.child_frame = config_.base_frame;
    sample.pose.translation = state.position;
    sample.pose.rotation = state.orientation;
    const Eigen::Vector2d injected_offset = GlobalOdometryOffset(stamp);
    sample.pose.translation.head<2>() += injected_offset;
    if (odometry_injection_.is_open() &&
        stamp >= odometry_fault_.start_ns) {
      odometry_injection_ << stamp << ',' << state.stamp.sequence << ','
                          << (odometry_fault_.kind ==
                                      NativeGlobalOdometryFault::Kind::kDrift
                                  ? "drift"
                                  : "step")
                          << ',' << injected_offset.x() << ','
                          << injected_offset.y() << '\n';
      if (!odometry_injection_)
        throw std::runtime_error("incomplete global ODOM injection journal");
    }
    // Local error order is [p,v,right rotation,bg,ba]. This is the native
    // marginal, explicitly uncalibrated; no joint/cross-time claim is made.
    Eigen::Matrix<double, 6, 6> covariance;
    covariance.topLeftCorner<3, 3>() = state.covariance.topLeftCorner<3, 3>();
    covariance.topRightCorner<3, 3>() = state.covariance.block<3, 3>(0, 6);
    covariance.bottomLeftCorner<3, 3>() = state.covariance.block<3, 3>(6, 0);
    covariance.bottomRightCorner<3, 3>() = state.covariance.block<3, 3>(6, 6);
    Require(owner_.AddOdometry(sample, covariance));
    last_odom_stamp_ = stamp;
    WriteGlobal(stamp, receive);
    WriteDecisions();
  }

  void ProcessGnss(int64_t receive) {
    if (!gnss_) return;
    gnss_->Drain(
        receive,
        [this, receive](localization::GlobalHorizontalObservation o) {
          o.context.epoch = localization::OdometryEpoch{
              history_.back().epoch.session, history_.back().epoch.generation};
          o.context.reference = config_.reference;
          o.context.map_content_id = config_.map_content_id;
          o.context.map_frame = config_.map_frame;
          o.context.base_frame = config_.base_frame;
          o.context.provenance.acquisition_id =
              o.context.epoch.producer_session;
          return SubmitGlobal(std::move(o), receive);
        },
        !history_.empty());
    if (!history_.empty()) WriteGlobal(last_odom_stamp_, receive);
    WriteDecisions();
  }

  void Observe(const localization::unified::LidarScan& scan, int64_t stamp,
               int64_t receive) {
    namespace loc = localization;
    namespace u = localization::unified;
    if (!matcher_) return;
    if (map_holdout_.first && stamp >= map_holdout_.first &&
        stamp < map_holdout_.second) {
      matches_ << scan.stamp.sequence << ',' << stamp << ',' << receive
               << ",0,EVALUATION_HOLDOUT,excluded_before_deskew_seed_and_match,"
                  "0,0,0,0,0,not_submitted,evaluation_holdout,0,0,0\n";
      return;
    }
    u::DeskewedScan deskewed;
    const auto status = u::DeskewLidarScan(
        scan, [this](double t, u::LocalState* s) { return Lookup(t, s); },
        &deskewed);
    if (!status.ok()) {
      matches_ << scan.stamp.sequence << ',' << stamp << ',' << receive << ",0,"
               << u::ReasonName(status.reason) << ',';
      Text(matches_, status.message);
      matches_ << ",0,0,0,0,0,not_submitted,deskew_unavailable,0,0,0\n";
      return;
    }
    const auto& state = history_.back();
    const Eigen::Isometry3d odom_from_base = u::Pose(state);
    if (!seeded_ && !causal_search_) {
      const int64_t offset =
          seed_reference_ns_ ? stamp - seed_reference_ns_ : 0;
      const bool stale =
          seed_reference_ns_ && std::abs(offset) > seed_maximum_offset_ns_;
      seed_evidence_ << scan.stamp.sequence << ',' << stamp << ',' << receive
                     << ",1,0," << seed_reference_ns_ << ',' << offset << ','
                     << seed_maximum_offset_ns_ << ','
                     << (stale ? "seed_reference_outside_tolerance" : "applied")
                     << '\n';
      seed_evidence_.flush();
      if (stale)
        throw std::runtime_error(
            "map seed reference outside explicit time tolerance");
      map_from_odom_ = seed_ * odom_from_base.inverse();
      seeded_ = true;
    }
    loc::map::PointCloud::Ptr cloud(new loc::map::PointCloud);
    cloud->reserve(deskewed.points_in_end_base.size());
    for (const auto& p : deskewed.points_in_end_base)
      cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    loc::map::ScanToMapRequest request;
    request.timestamp_ns = stamp;
    request.clock_domain = "unix";
    request.cloud_frame = config_.base_frame.value;
    request.deskewed = true;
    request.cloud = cloud;
    std::optional<Eigen::Isometry3d> proposal;
    if (!seeded_ && causal_search_) {
      proposal = RecoveryProposal(state, scan.stamp.sequence, stamp, request);
      if (!proposal) {
        matches_ << scan.stamp.sequence << ',' << stamp << ',' << receive
                 << ",0,causal_search_pending,"
                    "no_unique_verified_causal_proposal,0,0,0,0,0,"
                    "not_submitted,causal_search_pending,0,0,0\n";
        return;
      }
    }
    request.map_from_base_initial =
        (proposal ? *proposal : map_from_odom_) * odom_from_base;
    const auto match = matcher_->Match(request);
    if (map_profile_.is_open()) {
      map_profile_ << scan.stamp.sequence << ',' << match.iterations << ','
                   << match.processing_time_ms << ','
                   << match.map_window_build_ms << ','
                   << match.target_voxelization_ms << ','
                   << match.target_kdtree_build_ms << ','
                   << match.target_normal_estimation_ms << ','
                   << match.target_normal_neighbor_search_ms << ','
                   << match.target_normal_fit_ms << ','
                   << match.icp_nearest_search_ms << ','
                   << match.icp_linearization_ms << ','
                   << match.icp_solver_ms << '\n';
      if (!map_profile_)
        throw std::runtime_error("incomplete map timing profile");
    }
    const bool full_pose =
        match.valid && match.geometry_full_rank && !match.ambiguous;
    if (proposal) {
      if (full_pose) {
        map_from_odom_ = *proposal;
        seeded_ = true;
        hypothesis_stamp_ = stamp;
        RecordTracking(stamp, "verifying", "current_scan_verified", 0);
      } else {
        RecordTracking(stamp, "searching", "current_scan_rejected", 0);
      }
    }
    loc::Status admitted;
    bool submitted = false;
    if (match.valid &&
        (!causal_search_ || (seeded_ && (!recovering_ || full_pose)))) {
      ++matched_;
      loc::GlobalMapObservation observation;
      auto& context = observation.context;
      context.measurement_time = Time(stamp);
      context.receive_time = Time(receive);
      context.sequence = scan.stamp.sequence;
      context.source = "native-raw-scan-to-prior-map";
      context.epoch = {state.epoch.session, state.epoch.generation};
      context.reference = config_.reference;
      context.map_content_id = config_.map_content_id;
      context.map_frame = config_.map_frame;
      context.base_frame = config_.base_frame;
      context.provenance.acquisition_id = state.epoch.session;
      context.provenance.preparation_id = "native-map-diagnostic-v1";
      context.provenance.odometry_initialization =
          causal_search_ ? loc::ObservationDependency::kPresent
                         : loc::ObservationDependency::kAbsent;
      context.provenance.odometry_deskew = loc::ObservationDependency::kPresent;
      context.provenance.shared_local_scans =
          loc::ObservationDependency::kPresent;
      context.provenance.gnss_map_construction =
          gnss_ ? gnss_->dependency() : loc::ObservationDependency::kUnknown;
      observation.map_from_base.translation = match.map_from_base.translation();
      observation.map_from_base.rotation =
          Eigen::Quaterniond(match.map_from_base.linear());
      observation.covariance_source = match.covariance_source;
      observation.ambiguous = match.ambiguous;
      observation.recovery = recovering_;
      if (match.geometry_full_rank) {
        observation.covariance = match.covariance;
      } else {
        observation.kind = loc::GlobalMapObservation::Kind::kProjectedPose;
        observation.projection = match.observed_projection;
        observation.projected_covariance = match.observed_covariance;
      }
      admitted = SubmitGlobal(std::move(observation), receive);
      submitted = true;
    }
    matches_ << scan.stamp.sequence << ',' << stamp << ',' << receive << ','
             << match.valid << ',';
    Text(matches_, match.reason);
    matches_ << ',';
    Text(matches_, match.detail);
    matches_ << ',' << match.observed_projection.rows() << ','
             << match.source_overlap << ',' << match.target_overlap << ','
             << match.score << ',' << match.processing_time_ms << ',';
    if (submitted)
      matches_ << static_cast<int>(admitted.code());
    else
      matches_ << "not_submitted";
    matches_ << ',';
    Text(matches_, submitted     ? admitted.message()
                   : match.valid ? "recovery_requires_full_pose"
                                 : "map rejected");
    matches_ << ',' << match.map_window_points << ','
             << match.registration_source_points << ','
             << match.registration_target_points << '\n';
    WriteGlobal(stamp, receive);
    WriteDecisions();
  }

  void DrainGlobal(int64_t receive) {
    last_processing_receive_ = receive;
    bool submitted = false;
    while (!pending_.empty() &&
           pending_.front().context.measurement_time.nanoseconds <=
               receive - holdback_ns_) {
      const auto& pending = pending_.front();
      const auto status = std::visit(
          [this](const auto& observation) { return Apply(observation); },
          pending.observation);
      WriteAdmission(pending.context, pending.enqueue_receive, receive, status);
      pending_.pop_front();
      submitted = true;
    }
    if (submitted) {
      if (!history_.empty()) WriteGlobal(last_odom_stamp_, receive);
      WriteDecisions();
    }
  }

  void WriteDecisions() {
    namespace loc = localization;
    std::vector<loc::GlobalObservationDecision> journal;
    Require(owner_.GetDecisions(&journal));
    for (const auto& decision : journal) {
      if (decision.decision_id <= last_decision_) continue;
      if (decision.decision_id != last_decision_ + 1)
        throw std::runtime_error("global decision journal overrun");
      last_decision_ = decision.decision_id;
      updates_ += decision.alignment_updated;
      decisions_ << decision.decision_id << ','
                 << decision.context.measurement_time.nanoseconds << ','
                 << decision.context.receive_time.nanoseconds << ','
                 << decision.context.sequence << ','
                 << static_cast<int>(decision.status.code()) << ',';
      Text(decisions_, decision.status.message());
      decisions_ << ',' << decision.alignment_updated << ','
                 << decision.correction_id << ',';
      Text(decisions_, decision.covariance_source);
      decisions_ << ',';
      Text(decisions_, Dependency(decision.context.provenance.odometry_deskew));
      decisions_ << ',';
      Text(decisions_,
           Dependency(decision.context.provenance.shared_local_scans));
      decisions_ << ",0,";
      Text(decisions_, decision.context.source);
      decisions_ << ',';
      Text(decisions_,
           Dependency(decision.context.provenance.gnss_map_construction));
      decisions_ << ',' << decision.innovation.available << ',';
      Text(decisions_, decision.innovation.unavailable_reason);
      decisions_ << ',' << decision.innovation.observed_dimension << ',';
      Text(decisions_, decision.innovation.prior_state_frame);
      decisions_ << ',' << decision.innovation.prior_state_dimension << ',';
      Text(decisions_, Flatten(decision.innovation.residual));
      decisions_ << ',';
      Text(decisions_, Flatten(decision.innovation.covariance));
      decisions_ << ',' << std::setprecision(17) << decision.innovation.nis
                 << ',';
      Eigen::Matrix<double, 1, 7> prior_pose;
      prior_pose << decision.innovation.prior_pose.translation.x(),
          decision.innovation.prior_pose.translation.y(),
          decision.innovation.prior_pose.translation.z(),
          decision.innovation.prior_pose.rotation.x(),
          decision.innovation.prior_pose.rotation.y(),
          decision.innovation.prior_pose.rotation.z(),
          decision.innovation.prior_pose.rotation.w();
      Text(decisions_, Flatten(prior_pose));
      decisions_ << ',';
      Text(decisions_, Flatten(decision.innovation.prior_covariance));
      decisions_ << ',' << decision.innovation.gate_threshold << ',';
      Text(decisions_, decision.innovation.residual_basis);
      decisions_ << ',';
      Text(decisions_, decision.innovation.covariance_model);
      decisions_ << ',' << decision.innovation.covariance_decomposition_available << ',';
      Text(decisions_, Flatten(decision.innovation.prior_contribution));
      decisions_ << ',';
      Text(decisions_, Flatten(decision.innovation.observation_contribution));
      decisions_ << ',';
      Text(decisions_, Flatten(decision.innovation.local_contribution));
      decisions_ << '\n';
    }
  }

  void Finish() {
    if (gnss_) gnss_->Finish();
    if (recovery_future_.valid()) {
      recovery_future_.get();
      RecordRecovery(recovery_sequence_, recovery_stamp_, last_odom_stamp_,
                     "end_of_input_unconsumed");
    }
    if (causal_search_) causal_search_->Finish();
    if (tracking_journal_.is_open()) {
      tracking_journal_.flush();
      if (!tracking_journal_)
        throw std::runtime_error("incomplete map tracking journal");
    }
    if (recovery_journal_.is_open()) {
      recovery_journal_.flush();
      if (!recovery_journal_)
        throw std::runtime_error("incomplete causal worker journal");
    }
    // Explicit replay tail: advance only the processing clock, not any sensor
    // measurement/arrival or ODOM state. No unreceived observation is consumed.
    if (!pending_.empty()) DrainGlobal(last_processing_receive_ + holdback_ns_);
    matches_.flush();
    global_.flush();
    decisions_.flush();
    seed_evidence_.flush();
    admission_.flush();
    if (odometry_injection_.is_open()) {
      odometry_injection_.flush();
      if (!odometry_injection_)
        throw std::runtime_error("incomplete global ODOM injection journal");
    }
    if (horizontal_.is_open()) {
      horizontal_.flush();
      if (!horizontal_)
        throw std::runtime_error("incomplete horizontal output");
    }
    if (!matches_ || !global_ || !decisions_ || !seed_evidence_ ||
        !admission_ || !pending_.empty())
      throw std::runtime_error("incomplete map/global output");
    if (matcher_ && !causal_search_ &&
        (!matched_ || !updates_ || !global_outputs_))
      throw std::runtime_error(
          "diagnostic map/global alignment not established");
  }

  bool initialized() const { return alignment_initialized_; }

  const localization::DiagnosticAlignmentConfig& configuration() const {
    return config_;
  }

  localization::map::ScanToMapResult MatchStructured(
      const localization::unified::StructuredLidarScan& scan, int64_t stamp) {
    namespace u = localization::unified;
    if (!matcher_ || causal_search_ || !scan.conditioning_state.state.valid)
      throw std::runtime_error("structured closure requires a fixed prior-map seed");
    const auto& current = scan.conditioning_state;
    if (!structured_seed_) {
      if (seed_reference_ns_ == 0 ||
          std::abs(stamp - seed_reference_ns_) > seed_maximum_offset_ns_)
        throw std::runtime_error("structured map seed reference outside tolerance");
      structured_seed_ = current;
      map_from_odom_ = seed_ * u::Pose(current.state).inverse();
    }
    localization::map::ScanToMapRequest request;
    request.timestamp_ns = stamp;
    request.clock_domain = "unix";
    request.cloud_frame = config_.base_frame.value;
    request.deskewed = true;
    request.structured_scan = &scan;
    request.map_from_base_initial = map_from_odom_ * u::Pose(current.state);
    Eigen::Matrix<double, 6, 15> first =
        Eigen::Matrix<double, 6, 15>::Zero(), last = first;
    const auto& initial = structured_seed_->state;
    first.topLeftCorner<3, 3>() = -map_from_odom_.linear();
    first.block<3, 3>(0, 6) =
        seed_.linear() *
        u::Skew(initial.orientation.conjugate() *
                (current.state.position - initial.position));
    first.block<3, 3>(3, 6) = -seed_.linear();
    last.topLeftCorner<3, 3>() = map_from_odom_.linear();
    last.block<3, 3>(3, 6) = request.map_from_base_initial.linear();
    // The configuration seed is fixed at the first scan, not an independent
    // stochastic map/odom alignment. Later seeds reuse BOTH native endpoints.
    request.initial_structured_error = u::JointErrorExpression::Combine(
        6, {{first, structured_seed_->output_error}, {last, current.output_error}});
    return matcher_->Match(request);
  }

  template <class Submit>
  void DrainStructuredGnss(int64_t receive, Submit submit, bool available) {
    if (!gnss_) throw std::runtime_error("structured closure requires prepared GNSS");
    gnss_->Drain(receive, std::move(submit), available);
  }

  void FinishStructuredGnss() { gnss_->Finish(); }

 private:
  Eigen::Vector2d GlobalOdometryOffset(int64_t stamp) const {
    if (odometry_fault_.kind == NativeGlobalOdometryFault::Kind::kNone ||
        stamp < odometry_fault_.start_ns)
      return Eigen::Vector2d::Zero();
    if (odometry_fault_.kind == NativeGlobalOdometryFault::Kind::kStep)
      return odometry_fault_.value;
    return odometry_fault_.value *
           (static_cast<double>(stamp - odometry_fault_.start_ns) / 1e9);
  }

  std::optional<Eigen::Isometry3d> RecoveryProposal(
      const localization::unified::LocalState& state, uint64_t sequence,
      int64_t stamp, const localization::map::ScanToMapRequest& request) {
    namespace u = localization::unified;
    if (!async_recovery_) {
      const auto proposal = causal_search_->Observe(state, sequence, stamp,
                                                    request, matcher_.get());
      if (proposal) return *proposal * u::Pose(state).inverse();
      return std::nullopt;
    }
    if (recovery_future_.valid()) {
      if (recovery_future_.wait_for(std::chrono::seconds(0)) !=
          std::future_status::ready) {
        RecordRecovery(sequence, stamp, stamp, "busy");
        return std::nullopt;
      }
      const auto proposal = recovery_future_.get();
      const bool fresh = state.epoch == recovery_state_.epoch &&
                         stamp >= recovery_stamp_ &&
                         stamp - recovery_stamp_ <= 500000000;
      RecordRecovery(recovery_sequence_, recovery_stamp_, stamp,
                     fresh ? (proposal ? "proposal_consumed" : "no_proposal")
                           : "stale_result_rejected");
      if (proposal && fresh)
        return *proposal * u::Pose(recovery_state_).inverse();
    }
    recovery_state_ = state;
    recovery_sequence_ = sequence;
    recovery_stamp_ = stamp;
    // Only this snapshot task uses the matcher until its future is consumed.
    // Normal tracking starts after the result has joined on this thread.
    recovery_future_ =
        std::async(std::launch::async, [this, state, sequence, stamp, request] {
          return causal_search_->Observe(state, sequence, stamp, request,
                                         matcher_.get());
        });
    RecordRecovery(sequence, stamp, stamp, "queued");
    return std::nullopt;
  }

  void RecordRecovery(uint64_t sequence, int64_t reference, int64_t processing,
                      const char* status) {
    recovery_journal_ << sequence << ',' << reference << ',' << processing
                      << ',' << status << '\n';
    recovery_journal_.flush();
    if (!recovery_journal_)
      throw std::runtime_error("incomplete causal worker journal");
  }

  void RecordTracking(int64_t stamp, const char* state, const char* event,
                      int64_t last_full_pose) {
    tracking_journal_ << stamp << ',' << state << ',' << event << ','
                      << last_full_pose << ',' << causal_search_->generation()
                      << ",1\n";
    tracking_journal_.flush();
    if (!tracking_journal_)
      throw std::runtime_error("incomplete map tracking journal");
  }

  void CheckTrackingLease(const localization::DiagnosticAlignmentResult& result,
                          int64_t stamp) {
    if (!causal_search_ || !seeded_) return;
    const int64_t last_full = result.last_full_pose_time
                                  ? result.last_full_pose_time->nanoseconds
                                  : 0;
    if (hypothesis_stamp_ && result.estimate &&
        last_full >= *hypothesis_stamp_) {
      hypothesis_stamp_.reset();
      recovering_ = false;
      RecordTracking(stamp, "tracking", "alignment_committed", last_full);
    }
    const int64_t reference = hypothesis_stamp_.value_or(last_full);
    if (reference && stamp - reference > config_.maximum_full_pose_age_ns) {
      if (recovery_future_.valid())
        throw std::runtime_error("tracking reset with active recovery worker");
      seeded_ = false;
      recovering_ = true;
      hypothesis_stamp_.reset();
      causal_search_->Restart();
      RecordTracking(stamp, "searching", "full_pose_lease_expired", last_full);
    }
  }

  using Observation = std::variant<localization::GlobalMapObservation,
                                   localization::GlobalHorizontalObservation>;
  struct Pending {
    localization::GlobalObservationContext context;
    Observation observation;
    int64_t enqueue_receive;
  };

  localization::Status Apply(const localization::GlobalMapObservation& o) {
    return owner_.SubmitMapObservation(o);
  }
  localization::Status Apply(
      const localization::GlobalHorizontalObservation& o) {
    return owner_.SubmitHorizontalObservation(o);
  }

  template <class T>
  localization::Status SubmitGlobal(T observation, int64_t receive) {
    WriteMeasurement(observation);
    const auto& context = observation.context;
    // Keep the owner's validation for future-on-arrival inputs; the holdback
    // must not launder an invalid future measurement into a valid past one.
    if (!holdback_ns_ || context.measurement_time.nanoseconds >
                             context.receive_time.nanoseconds) {
      const auto status = Apply(observation);
      WriteAdmission(context, receive, receive, status);
      return status;
    }
    if (pending_.size() >= 256)
      throw std::runtime_error("global holdback capacity exhausted");
    const auto position = std::upper_bound(
        pending_.begin(), pending_.end(), context.measurement_time.nanoseconds,
        [](int64_t time, const Pending& pending) {
          return time < pending.context.measurement_time.nanoseconds;
        });
    pending_.insert(position, {context, std::move(observation), receive});
    return {localization::StatusCode::kPending,
            "retained in bounded diagnostic global holdback"};
  }

  void WriteMeasurement(const localization::GlobalMapObservation& observation) {
    const bool full =
        observation.kind == localization::GlobalMapObservation::Kind::kFullPose;
    WriteMeasurementContext(full ? "map" : "projected_map", observation.context);
    const auto& pose = observation.map_from_base;
    measurements_ << ',' << pose.translation.x() << ',' << pose.translation.y()
                  << ',' << pose.translation.z() << ',' << pose.rotation.x()
                  << ',' << pose.rotation.y() << ',' << pose.rotation.z()
                  << ',' << pose.rotation.w() << ',';
    measurements_ << Flatten(full ? Eigen::MatrixXd(Eigen::Matrix<double, 6, 6>::Identity())
                                  : observation.projection)
                  << ','
                  << Flatten(full ? Eigen::MatrixXd(observation.covariance)
                                  : observation.projected_covariance)
                  << ",0,0,0," << observation.ambiguous << '\n';
    FlushMeasurements();
  }

  void WriteMeasurement(
      const localization::GlobalHorizontalObservation& observation) {
    WriteMeasurementContext("gnss", observation.context);
    measurements_ << ',' << observation.position.x() << ','
                  << observation.position.y() << ",0,0,0,0,1,"
                  << Flatten(Eigen::Matrix2d::Identity()) << ','
                  << Flatten(observation.covariance);
    for (int axis = 0; axis < 3; ++axis)
      measurements_ << ',' << observation.point_in_base[axis];
    measurements_ << ",0\n";
    FlushMeasurements();
  }

  void WriteMeasurementContext(
      const char* kind, const localization::GlobalObservationContext& context) {
    measurements_ << kind << ',' << context.measurement_time.nanoseconds << ','
                  << context.receive_time.nanoseconds << ',' << context.sequence;
  }

  void FlushMeasurements() {
    measurements_.flush();
    if (!measurements_) throw std::runtime_error("incomplete measurement journal");
  }

  void WriteAdmission(const localization::GlobalObservationContext& context,
                      int64_t enqueue_receive, int64_t submission_receive,
                      const localization::Status& status) {
    Text(admission_, context.source);
    admission_ << ',' << context.sequence << ','
               << context.measurement_time.nanoseconds << ','
               << context.receive_time.nanoseconds << ',' << enqueue_receive
               << ',' << submission_receive << ',' << holdback_ns_ << ','
               << static_cast<int>(status.code()) << ',';
    Text(admission_, status.message());
    admission_ << '\n';
  }

  static const char* Dependency(localization::ObservationDependency value) {
    using D = localization::ObservationDependency;
    return value == D::kPresent  ? "present"
           : value == D::kAbsent ? "absent"
                                 : "unknown";
  }
  static std::string Sha256(const std::filesystem::path& path) {
    int descriptors[2];
    if (pipe(descriptors) != 0) throw std::runtime_error("hash pipe failed");
    const pid_t child = fork();
    if (child < 0) {
      close(descriptors[0]);
      close(descriptors[1]);
      throw std::runtime_error("hash process failed");
    }
    if (child == 0) {
      close(descriptors[0]);
      if (dup2(descriptors[1], STDOUT_FILENO) < 0) _exit(126);
      close(descriptors[1]);
      execlp("sha256sum", "sha256sum", "--", path.c_str(), nullptr);
      _exit(127);
    }
    close(descriptors[1]);
    std::string output;
    char buffer[512];
    ssize_t count;
    while ((count = read(descriptors[0], buffer, sizeof(buffer))) > 0)
      output.append(buffer, count);
    close(descriptors[0]);
    int status = 0;
    if (waitpid(child, &status, 0) != child || count < 0 ||
        !WIFEXITED(status) || WEXITSTATUS(status) != 0 || output.size() < 64)
      throw std::runtime_error("map SHA256 validation failed");
    return output.substr(0, 64);
  }
  static localization::Timestamp Time(int64_t ns) {
    return {ns, localization::ClockDomain::kUnixEpoch};
  }
  static void Require(const localization::Status& status) {
    if (!status.ok()) throw std::runtime_error(status.message());
  }
  localization::unified::Result Lookup(
      double time, localization::unified::LocalState* state) const {
    namespace u = localization::unified;
    const auto upper = std::lower_bound(
        history_.begin(), history_.end(), time,
        [](const u::LocalState& s, double t) { return s.stamp.time < t; });
    if (upper != history_.end() && upper->stamp.time == time) {
      *state = *upper;
      return {};
    }
    if (upper == history_.begin() || upper == history_.end())
      return {u::Reason::HISTORY_UNAVAILABLE,
              "native deskew history unavailable"};
    const auto lower = std::prev(upper);
    const double dt = upper->stamp.time - lower->stamp.time;
    if (lower->epoch != upper->epoch || dt <= 0 || dt > 0.02)
      return {u::Reason::HISTORY_UNAVAILABLE, "native deskew gap or epoch"};
    // Interpolation is for raw-point deskew, never a synthetic graph node.
    const double alpha = (time - lower->stamp.time) / dt;
    *state = *lower;
    state->position = (1 - alpha) * lower->position + alpha * upper->position;
    state->orientation = lower->orientation.slerp(alpha, upper->orientation);
    state->stamp.time = time;
    return {};
  }
  void WriteGlobal(int64_t stamp, int64_t receive) {
    localization::DiagnosticAlignmentResult result;
    Require(owner_.GetResult(Time(receive), &result));
    CheckTrackingLease(result, stamp);
    alignment_initialized_ = result.estimate.has_value();
    global_ << stamp << ',' << receive << ',' << result.estimate.has_value();
    if (result.estimate) {
      ++global_outputs_;
      const auto& estimate = *result.estimate;
      const auto& p = estimate.map_from_base;
      global_ << ',' << estimate.correction_id << ',' << p.translation.x()
              << ',' << p.translation.y() << ',' << p.translation.z() << ','
              << p.rotation.x() << ',' << p.rotation.y() << ','
              << p.rotation.z() << ',' << p.rotation.w();
      // A retained old owner estimate must not replace a recovery hypothesis
      // while its distinct full-pose observations await verification/holdback.
      if (!hypothesis_stamp_) {
        map_from_odom_ = Eigen::Isometry3d::Identity();
        map_from_odom_.translation() = estimate.map_from_odom.translation;
        map_from_odom_.linear() =
            estimate.map_from_odom.rotation.toRotationMatrix();
      }
    } else {
      global_ << ",,,,,,,,";
    }
    global_ << ',' << result.odometry_fresh << ','
            << result.full_pose_evidence_fresh << ",1,0,0\n";
    if (horizontal_.is_open()) {
      horizontal_ << stamp << ',' << receive << ','
                  << result.horizontal_estimate.has_value();
      if (result.horizontal_estimate) {
        const auto& estimate = *result.horizontal_estimate;
        horizontal_ << ',' << estimate.correction_id << ','
                    << estimate.map_base_xy.x() << ','
                    << estimate.map_base_xy.y() << ','
                    << estimate.map_from_odom_xy_yaw.x() << ','
                    << estimate.map_from_odom_xy_yaw.y() << ','
                    << estimate.map_from_odom_xy_yaw.z() << ','
                    << estimate.last_observation_time.nanoseconds;
      } else {
        horizontal_ << ",,,,,,,";
      }
      horizontal_ << ',' << result.odometry_fresh << ','
                  << result.horizontal_evidence_fresh << ",1,0,0\n";
      if (!horizontal_)
        throw std::runtime_error("incomplete horizontal output");
    }
  }
  localization::DiagnosticAlignmentConfig config_;
  localization::DiagnosticGlobalAlignment owner_;
  std::deque<Pending> pending_;
  std::unique_ptr<localization::map::ScanToMapLocalizer> matcher_;
  std::unique_ptr<NativeGnssDiagnostic> gnss_;
  std::optional<localization::unified::StructuredLocalState> structured_seed_;
  std::deque<localization::unified::LocalState> history_;
  Eigen::Isometry3d seed_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d map_from_odom_ = Eigen::Isometry3d::Identity();
  bool seeded_ = false;
  bool alignment_initialized_ = false;
  uint64_t matched_ = 0, updates_ = 0, global_outputs_ = 0, last_decision_ = 0;
  int64_t last_odom_stamp_ = 0;
  int64_t seed_reference_ns_ = 0, seed_maximum_offset_ns_ = 0;
  std::ofstream matches_, global_, decisions_, seed_evidence_, horizontal_;
  std::ofstream measurements_;
  std::ofstream odometry_injection_;
  std::ofstream map_profile_;
  NativeGlobalOdometryFault odometry_fault_;
  std::unique_ptr<NativeCausalMapSearch> causal_search_;
  std::ofstream tracking_journal_;
  std::optional<int64_t> hypothesis_stamp_;
  bool recovering_ = false;
  std::ofstream admission_;
  int64_t holdback_ns_ = 0;
  int64_t last_processing_receive_ = 0;
  bool async_recovery_ = false;
  std::pair<int64_t, int64_t> map_holdout_;
  localization::unified::LocalState recovery_state_;
  uint64_t recovery_sequence_ = 0;
  int64_t recovery_stamp_ = 0;
  std::ofstream recovery_journal_;
  // Joins before matcher/search/journals are destroyed on exceptional exit.
  std::future<std::optional<Eigen::Isometry3d>> recovery_future_;
};
}  // namespace openloc::replay
