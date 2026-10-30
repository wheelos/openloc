// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <set>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"
#include "Eigen/QR"
#include "candidates/local/local_motion.h"
#include "openloc/localization/global/fixed_lag_alignment.h"
#include "openloc/localization/local/native_odometry.h"
#include "test/integration/map/native_map_diagnostic.h"

namespace openloc::replay {
namespace u = localization::unified;
namespace structured_closure {

inline void Require(const u::Result& result) {
  if (!result.ok()) throw std::runtime_error(result.message);
}
inline void Require(const localization::Status& result) {
  if (!result.ok()) throw std::runtime_error(result.message());
}

inline u::StructuredLocalState Interpolate(
    const u::StructuredLocalState& first, int64_t first_ns,
    const u::StructuredLocalState& last, int64_t last_ns, int64_t query_ns) {
  if (!first.state.valid || !last.state.valid ||
      first.state.epoch != last.state.epoch || last_ns <= first_ns ||
      query_ns < first_ns || query_ns > last_ns)
    throw std::runtime_error("invalid structured interpolation endpoints");
  if (query_ns == first_ns) return first;
  if (query_ns == last_ns) return last;
  const double alpha = double(query_ns - first_ns) / double(last_ns - first_ns);
  u::StructuredLocalState result = first;
  auto& state = result.state;
  state.stamp.time = query_ns * 1e-9;
  state.stamp.receive_time = last.state.stamp.receive_time;
  // This is the actual left IMU contributor's identity, not a fabricated IMU.
  state.stamp.sequence = first.state.stamp.sequence;
  state.position = (1 - alpha) * first.state.position + alpha * last.state.position;
  state.velocity = (1 - alpha) * first.state.velocity + alpha * last.state.velocity;
  state.gyro_bias = (1 - alpha) * first.state.gyro_bias + alpha * last.state.gyro_bias;
  state.accel_bias = (1 - alpha) * first.state.accel_bias + alpha * last.state.accel_bias;
  state.orientation = first.state.orientation.slerp(alpha, last.state.orientation);
  u::Matrix15d a = (1 - alpha) * u::Matrix15d::Identity();
  u::Matrix15d b = alpha * u::Matrix15d::Identity();
  Eigen::Matrix3d rotation_a, rotation_b;
  Require(u::RotationInterpolationJacobians(
      first.state.orientation, last.state.orientation, alpha,
      &rotation_a, &rotation_b));
  a.block<3, 3>(6, 6) = rotation_a;
  b.block<3, 3>(6, 6) = rotation_b;
  result.output_error = u::JointErrorExpression::Combine(
      15, {{a, first.output_error}, {b, last.output_error}});
  Eigen::MatrixXd covariance;
  Require(result.output_error.Covariance(&covariance));
  state.covariance = covariance;
  state.covariance_square_root.reset();
  state.joint_error.reset();
  return result;
}

inline Eigen::MatrixXd Root(const Eigen::MatrixXd& covariance,
                           double* reconstruction_error) {
  if (covariance.rows() == 0 || covariance.rows() != covariance.cols() ||
      covariance.rows() > 2048 || !covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-10) ||
      (covariance.diagonal().array() <= 0).any())
    throw std::runtime_error("invalid finite-query covariance");
  const Eigen::VectorXd scale = covariance.diagonal().cwiseSqrt();
  const Eigen::MatrixXd normalization = scale.cwiseInverse().asDiagonal();
  const Eigen::MatrixXd normalized = normalization * covariance * normalization;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(normalized);
  if (spectrum.info() != Eigen::Success)
    throw std::runtime_error("finite-query eigensolver failed");
  const double threshold = 64 * std::numeric_limits<double>::epsilon() *
                           spectrum.eigenvalues().cwiseAbs().maxCoeff();
  if (spectrum.eigenvalues().minCoeff() < -threshold)
    throw std::runtime_error("finite-query covariance is not PSD");
  Eigen::Index rank = 0;
  for (double value : spectrum.eigenvalues()) {
    if (value > threshold / 8 && value <= threshold)
      throw std::runtime_error("unresolved finite-query covariance rank");
    if (value > threshold) ++rank;
  }
  if (rank == 0) throw std::runtime_error("empty finite-query covariance rank");
  Eigen::MatrixXd root(covariance.rows(), rank);
  if (rank == covariance.rows()) {
    const Eigen::LLT<Eigen::MatrixXd> triangular(normalized);
    if (triangular.info() != Eigen::Success)
      throw std::runtime_error("finite-query triangular root failed");
    root = scale.asDiagonal() * Eigen::MatrixXd(triangular.matrixL());
  } else {
    Eigen::MatrixXd eigen_root(covariance.rows(), rank);
    Eigen::Index column = 0;
    for (Eigen::Index i = 0; i < covariance.rows(); ++i)
      if (spectrum.eigenvalues()[i] > threshold)
        eigen_root.col(column++) =
            spectrum.eigenvectors().col(i) * std::sqrt(spectrum.eigenvalues()[i]);
    // Choose independent query coordinates, then root their principal minor.
    // This removes arbitrary eigenvector rotations from near-hard graph factors.
    const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> pivot(eigen_root.transpose());
    const auto indices = pivot.colsPermutation().indices().head(rank).eval();
    Eigen::MatrixXd columns(covariance.rows(), rank), principal(rank, rank);
    for (Eigen::Index j = 0; j < rank; ++j) {
      columns.col(j) = normalized.col(indices[j]);
      for (Eigen::Index i = 0; i < rank; ++i)
        principal(i, j) = normalized(indices[i], indices[j]);
    }
    const Eigen::LLT<Eigen::MatrixXd> triangular(principal);
    if (triangular.info() != Eigen::Success)
      throw std::runtime_error("finite-query independent-minor root failed");
    const Eigen::MatrixXd normalized_root =
        triangular.matrixL().solve(columns.transpose()).transpose();
    root = scale.asDiagonal() * normalized_root;
  }
  *reconstruction_error =
      (normalization * (root * root.transpose() - covariance) * normalization)
          .cwiseAbs().maxCoeff();
  if (*reconstruction_error > 1e-11)
    throw std::runtime_error("finite-query root reconstruction failed");
  return root;
}
}  // namespace structured_closure

// Test-only, offline, predeclared query family. The prior map/seed is fixed and
// GNSS independence is a declared diagnostic approximation, NOT field evidence.
class NativeStructuredClosure {
 public:
  // Reload only the selected measurement family, never rerun native LIO/ICP.
  static void ReplayFrozen(const std::filesystem::path& input,
                           const std::filesystem::path& output,
                           bool causal_prefixes = false);
  NativeStructuredClosure(const std::filesystem::path& selection,
                          const std::filesystem::path& gnss_bundle,
                          const std::filesystem::path& output,
                          NativeMapDiagnostic* inputs)
      : output_(output), inputs_(inputs), matches_(output / "structured_map.csv"),
        nodes_journal_(output / "structured_nodes.csv") {
    for (const auto& row : ReadRows(selection.string(), "scan_sequence", 1)) {
      const auto sequence = static_cast<uint64_t>(Time(row[0]));
      if ((!scans_.empty() && sequence <= *scans_.rbegin()) ||
          !scans_.insert(sequence).second)
        throw std::runtime_error("structured scan selection must strictly increase");
    }
    if (scans_.size() < 3 || scans_.size() > 16)
      throw std::runtime_error("structured closure requires 3..16 selected scans");
    for (const auto& row : ReadRows(
             (gnss_bundle / "events.csv").string(),
             "timestamp_ns,receive_ns,sequence,x,y,quality,satellites,hdop,age,adapter_reason",
             10)) {
      const auto stamp = Time(row[0]);
      if (!gnss_times_.insert(stamp).second)
        throw std::runtime_error("duplicate prepared GNSS measurement time");
    }
    if (gnss_times_.empty() || gnss_times_.size() > 64)
      throw std::runtime_error("structured closure requires 1..64 GNSS epochs");
    matches_ << std::setprecision(17)
             << "scan_sequence,raw_end_ns,reference_ns,receive_ns,valid,reason,"
                "observed_rank,source_points,registration_points,score,"
                "structured_error_available,model_reason,model_detail,"
                "px,py,pz,qx,qy,qz,qw\n";
    nodes_journal_ << std::setprecision(17)
                   << "timestamp_ns,left_imu_ns,right_imu_ns,left_sequence,"
                      "right_sequence,interpolated,px,py,pz,qx,qy,qz,qw,"
                      "native_receive_ns\n";
    std::ofstream scope(output / "structured_scope.csv");
    scope << "model,offline_finite_family,deterministic_prior_map_and_seed,"
             "gnss_cross_source_independence_assumed_not_proven,"
             "gnss_map_construction_present_or_unknown,"
             "native_uncertainty_calibrated,physical_accuracy_verified,trusted\n"
             "selected-native-ICP-plus-conditional-approximate-GNSS,1,1,1,1,0,0,0\n";
    if (!matches_ || !nodes_journal_ || !scope)
      throw std::runtime_error("cannot open structured closure journals");
  }

  void AddOdometry(localization::NativeOdometry* local, int64_t stamp,
                   int64_t receive) {
    if (local->state().valid) {
      u::StructuredLocalState current;
      structured_closure::Require(
          local->StructuredErrorLookup(local->state().stamp.time, &current));
      for (auto time : gnss_times_) {
        if (time == stamp) {
          gnss_states_[time] = {current, stamp, stamp,
                               current.state.stamp.sequence,
                               current.state.stamp.sequence};
          gnss_states_[time].receive_ns = receive;
        } else if (previous_ && previous_ns_ < time && time < stamp) {
          gnss_states_[time] = {
              structured_closure::Interpolate(*previous_, previous_ns_,
                                             current, stamp, time),
              previous_ns_, stamp, previous_->state.stamp.sequence,
              current.state.stamp.sequence};
          gnss_states_[time].receive_ns = receive;
        }
      }
      previous_ = std::move(current);
      previous_ns_ = stamp;
    }
    inputs_->DrainStructuredGnss(
        receive,
        [&](localization::GlobalHorizontalObservation observation) {
          if (gnss_states_.count(observation.context.measurement_time.nanoseconds))
            gnss_.push_back(std::move(observation));
          else
            return localization::Status(
                localization::StatusCode::kStateUnavailable,
                "no original-time structured interpolation bracket");
          return localization::Status::Ok();
        },
        local->state().valid);
  }

  void Observe(localization::NativeOdometry* local, uint64_t sequence,
               int64_t raw_end, int64_t stamp, int64_t receive) {
    if (!scans_.count(sequence)) return;
    u::StructuredLidarScan scan;
    structured_closure::Require(local->PrepareStructuredLidarScan(sequence, &scan));
    auto match = inputs_->MatchStructured(scan, stamp);
    matches_ << sequence << ',' << raw_end << ',' << stamp << ',' << receive << ','
             << match.valid << ',';
    Text(matches_, match.reason + ": " + match.detail);
    matches_ << ',' << match.observed_projection.rows() << ',' << match.source_points
             << ',' << match.registration_source_points << ',' << match.score
             << ',' << bool(match.structured_error) << ','
             << (match.structured_error
                     ? u::ReasonName(match.structured_error->assessment.reason)
                     : "CORRELATION_UNQUALIFIED") << ',';
    Text(matches_, match.structured_error
                       ? match.structured_error->assessment.message
                       : "structured error unavailable");
    const Eigen::Quaterniond rotation(match.map_from_base.linear());
    for (double value : match.map_from_base.translation()) matches_ << ',' << value;
    matches_ << ',' << rotation.x() << ',' << rotation.y() << ',' << rotation.z()
             << ',' << rotation.w();
    matches_ << '\n';
    matches_.flush();
    seen_scans_.insert(sequence);
    if (!match.valid)
      throw std::runtime_error("selected actual ICP rejected: " + match.reason +
                               ": " + match.detail);
    if (!match.structured_error)
      throw std::runtime_error("actual ICP lost structured registered-point errors");
    structured_closure::Require(match.structured_error->assessment);
    if (!match.geometry_full_rank || match.observed_projection.rows() != 6)
      throw std::runtime_error("selected bootstrap ICP is not full pose");
    Node node{scan.conditioning_state, stamp, stamp,
              scan.conditioning_state.state.stamp.sequence,
              scan.conditioning_state.state.stamp.sequence};
    node.receive_ns = receive;
    nodes_[stamp] = std::move(node);
    maps_.emplace(stamp, MapQuery{sequence, receive, std::move(match)});
  }

  void Finish() {
    namespace loc = localization;
    inputs_->FinishStructuredGnss();
    if (seen_scans_ != scans_ || maps_.size() < 3 || gnss_.empty())
      throw std::runtime_error("selected real native/map/GNSS family incomplete");
    const auto first_map = maps_.begin()->first;
    for (const auto& observation : gnss_) {
      const auto stamp = observation.context.measurement_time.nanoseconds;
      if (stamp <= std::next(maps_.begin())->first)
        throw std::runtime_error("GNSS query precedes the two common map anchors");
      nodes_.emplace(stamp, gnss_states_.at(stamp));
    }
    if (nodes_.begin()->first != first_map)
      throw std::runtime_error("common map anchor must be first query");
    const Eigen::Index rows = 15 * nodes_.size() + 6 * maps_.size();
    if (rows > 2048) throw std::runtime_error("selected finite family exceeds capacity");
    std::vector<u::JointErrorExpression::Term> terms;
    Eigen::Index offset = 0;
    for (auto& entry : nodes_) {
      auto& node = entry.second;
      node.offset = offset;
      Eigen::MatrixXd coefficient = Eigen::MatrixXd::Zero(rows, 15);
      coefficient.block<15, 15>(offset, 0).setIdentity();
      terms.push_back({std::move(coefficient), node.local.output_error});
      offset += 15;
      const auto& state = node.local.state;
      nodes_journal_ << entry.first << ',' << node.left_ns << ',' << node.right_ns
                     << ',' << node.left_sequence << ',' << node.right_sequence
                     << ',' << (node.left_ns != node.right_ns) << ','
                     << state.position.transpose().format(
                            Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","))
                     << ',' << state.orientation.x() << ',' << state.orientation.y()
                     << ',' << state.orientation.z() << ',' << state.orientation.w()
                     << ',' << node.receive_ns << '\n';
    }
    for (auto& entry : maps_) {
      entry.second.offset = offset;
      Eigen::MatrixXd coefficient = Eigen::MatrixXd::Zero(rows, 6);
      coefficient.block<6, 6>(offset, 0).setIdentity();
      terms.push_back({std::move(coefficient),
                      entry.second.match.structured_error->pose_error});
      offset += 6;
    }
    Eigen::MatrixXd covariance;
    structured_closure::Require(
        u::JointErrorExpression::Combine(rows, terms).Covariance(&covariance));
    std::ofstream family(output_ / "structured_family_covariance.csv");
    family << std::setprecision(17) << Flatten(covariance) << '\n';
    family.close();
    if (!family) throw std::runtime_error("incomplete finite-family covariance");
    double reconstruction = 0;
    const auto root = structured_closure::Root(covariance, &reconstruction);
    std::ofstream roots(output_ / "structured_family_root.csv");
    roots << std::setprecision(17) << Flatten(root) << '\n';
    std::ofstream summary(output_ / "structured_family.csv");
    summary << std::setprecision(17)
            << "nodes,map_queries,gnss_queries,rows,rank,normalized_reconstruction,"
               "finite_family_only,trusted\n"
            << nodes_.size() << ',' << maps_.size() << ',' << gnss_.size() << ','
            << rows << ',' << root.cols() << ',' << reconstruction << ",1,0\n";
    if (!summary || !roots) throw std::runtime_error("incomplete finite-family root");
    std::ofstream comparisons(output_ / "structured_comparison.csv");
    comparisons << std::setprecision(17)
                << "arm,lag_vs_no_eviction_pose_norm,lag_vs_no_eviction_covariance_norm\n";
    for (int arm = 0; arm < 4; ++arm) {
      const auto no_eviction = RunArm(root, arm, false);
      const auto lag = RunArm(root, arm, true);
      comparisons << Arm(arm) << ','
                  << (lag.map_from_base.translation -
                      no_eviction.map_from_base.translation).norm() << ','
                  << (lag.alignment_covariance -
                      no_eviction.alignment_covariance).norm() << '\n';
    }
    matches_.flush();
    nodes_journal_.flush();
    if (!matches_ || !nodes_journal_ || !comparisons)
      throw std::runtime_error("incomplete structured closure output");
  }

 private:
  struct Node {
    localization::unified::StructuredLocalState local;
    int64_t left_ns, right_ns;
    uint64_t left_sequence, right_sequence;
    Eigen::Index offset = 0;
    int64_t receive_ns = 0;
  };
  struct MapQuery {
    uint64_t sequence;
    int64_t receive;
    localization::map::ScanToMapResult match;
    Eigen::Index offset = 0;
  };
  static const char* Arm(int arm) {
    return arm == 0 ? "lio" : arm == 1 ? "gnss" : arm == 2 ? "map" : "all";
  }
  localization::GlobalObservationContext Context(
      const localization::FixedLagAlignmentConfig& config, int64_t stamp,
      int64_t receive, uint64_t sequence, const std::string& source) const {
    localization::GlobalObservationContext context;
    context.measurement_time = {stamp, config.clock_domain};
    context.receive_time = {receive, config.clock_domain};
    context.sequence = sequence;
    context.source = source;
    const auto& state = nodes_.at(stamp).local.state;
    context.epoch = {state.epoch.session, state.epoch.generation};
    context.reference = config.reference;
    context.map_content_id = config.map_content_id;
    context.map_frame = config.map_frame;
    context.base_frame = config.base_frame;
    context.provenance.acquisition_id = state.epoch.session;
    context.provenance.preparation_id =
        "offline-conditional-fixed-prior-native-query-family";
    context.provenance.odometry_initialization =
        localization::ObservationDependency::kAbsent;
    context.provenance.odometry_deskew =
        localization::ObservationDependency::kPresent;
    context.provenance.shared_local_scans =
        localization::ObservationDependency::kPresent;
    context.provenance.gnss_map_construction =
        localization::ObservationDependency::kPresent;
    return context;
  }
  localization::FixedLagAlignmentEstimate RunArm(
      const Eigen::MatrixXd& root, int arm, bool lag,
      const std::string& fault = "");
  NativeStructuredClosure(const std::filesystem::path& output)
      : output_(output), inputs_(nullptr) {}

  std::filesystem::path output_;
  NativeMapDiagnostic* inputs_;
  localization::DiagnosticAlignmentConfig frozen_config_;
  std::set<uint64_t> scans_, seen_scans_;
  std::set<int64_t> gnss_times_;
  std::optional<localization::unified::StructuredLocalState> previous_;
  int64_t previous_ns_ = 0;
  std::map<int64_t, Node> gnss_states_, nodes_;
  std::map<int64_t, MapQuery> maps_;
  std::vector<localization::GlobalHorizontalObservation> gnss_;
  std::ofstream matches_, nodes_journal_;
};

inline localization::FixedLagAlignmentEstimate NativeStructuredClosure::RunArm(
    const Eigen::MatrixXd& root, int arm, bool lag, const std::string& fault) {
  namespace loc = localization;
  const std::string name = (fault.empty() ? "" : fault + "_") +
                          std::string(Arm(arm)) +
                          (lag ? "_lag" : "_no_eviction");
  std::ofstream decisions(output_ / (name + "_decisions.csv"));
  std::ofstream states(output_ / (name + "_states.csv"));
  std::ofstream statistics(output_ / (name + "_sources.csv"));
  std::ofstream interventions(output_ / (name + "_interventions.csv"));
  interventions << "source,measurement_ns,action,dx_m,synthetic\n";
  decisions << std::setprecision(17)
            << "source,sequence,measurement_ns,receive_ns,status,reason,"
               "alignment_updated,innovation_available,innovation_residual,"
               "innovation_covariance,preupdate_nis,gate_threshold,"
               "innovation_unavailable_reason\n";
  states << std::setprecision(17)
         << "timestamp_ns,estimate_available,px,py,pz,diagnostic_only,trust_level,"
            "assessment,submitted,rejected,correction_id,verification_pending,"
            "full_pose_evidence_fresh,odometry_fresh\n";
  statistics << "source,submitted,alignment_updates,retained,rejected\n";
  loc::FixedLagAlignmentConfig config;
  const auto& input_config = inputs_ ? inputs_->configuration() : frozen_config_;
  config.reference = input_config.reference;
  config.map_content_id = input_config.map_content_id;
  config.map_frame = input_config.map_frame;
  config.odom_frame = input_config.odom_frame;
  config.base_frame = input_config.base_frame;
  config.allow_uncalibrated_inputs = true;
  config.motion_error_model_id =
      "offline-selected-native-ICP-family-conditional-fixed-prior-approx-GNSS";
  // Qualification of algebraic sufficiency for ONLY this frozen declared
  // approximate family. No empirical source/uncertainty qualification follows.
  config.motion_error_model_qualified = true;
  config.maximum_error_dimension = root.cols();
  config.verification_frames = 2;
  config.solve_budget_ms = 100000;
  config.history_duration_ns =
      lag ? 300'000'000LL : nodes_.rbegin()->first - nodes_.begin()->first + 1'000'000'000LL;
  config.maximum_observation_age_ns = config.history_duration_ns;
  loc::FixedLagGlobalAlignment graph;
  structured_closure::Require(graph.Configure(config));
  std::size_t journaled = 0;
  const auto journal = [&] {
    std::vector<loc::GraphObservationDecision> entries;
    structured_closure::Require(graph.GetDecisions(&entries));
    for (; journaled < entries.size(); ++journaled) {
      const auto& event = entries[journaled].observation;
      decisions << event.context.source << ',' << event.context.sequence << ','
                << event.context.measurement_time.nanoseconds << ','
                << event.context.receive_time.nanoseconds << ','
                << static_cast<int>(event.status.code()) << ',';
      Text(decisions, event.status.message());
      decisions << ',' << event.alignment_updated << ',' << event.innovation.available
                << ',' << Flatten(event.innovation.residual) << ','
                << Flatten(event.innovation.covariance) << ','
                << event.innovation.nis << ',' << event.innovation.gate_threshold << ',';
      Text(decisions, event.innovation.unavailable_reason);
      decisions << '\n';
    }
    decisions.flush();
  };
  const Eigen::MatrixXd unit = Eigen::MatrixXd::Identity(root.cols(), root.cols());
  const Node* previous = nullptr;
  int64_t previous_time = 0;
  std::size_t map_index = 0;
  std::size_t node_index = 0;
  for (const auto& entry : nodes_) {
    const auto& node = entry.second;
    const auto& native = node.local.state;
    loc::GraphOdometrySample sample;
    sample.sequence = native.stamp.sequence;
    sample.odometry.timestamp = {entry.first, config.clock_domain};
    sample.receive_time = {node.receive_ns, config.clock_domain};
    sample.odometry.epoch = {native.epoch.session, native.epoch.generation};
    sample.odometry.parent_frame = config.odom_frame;
    sample.odometry.child_frame = config.base_frame;
    sample.odometry.pose = {native.position, native.orientation};
    const Eigen::MatrixXd loading = root.middleRows(node.offset, 15);
    sample.covariance = loading * loading.transpose();
    sample.covariance_model_valid = true;
    sample.joint_error = u::JointLocalError{unit, loading};
    loc::GraphMotionConnection connection;
    if (previous) {
      connection.epoch = sample.odometry.epoch;
      connection.start_time = {previous_time, config.clock_domain};
      connection.end_time = sample.odometry.timestamp;
      connection.start_sequence = previous->local.state.stamp.sequence;
      connection.end_sequence = sample.sequence;
      connection.joint_cross_covariance = unit;
      connection.cross_covariance =
          root.middleRows(previous->offset, 15) * loading.transpose();
      // Interpolated nodes use the actual left contributor as their endpoint
      // token. Include both real interpolation brackets, never invent IMUs.
      for (uint64_t sequence = previous->left_sequence;
           sequence <= node.right_sequence; ++sequence)
        connection.sources.push_back({"imu", sequence});
    }
    structured_closure::Require(
        graph.AddOdometry(sample, previous ? &connection : nullptr));
    const auto found_map = maps_.find(entry.first);
    if (found_map != maps_.end()) {
      const auto& query = found_map->second;
      if (map_index < 2 || (arm & 2)) {
        const bool omitted = map_index == 2 &&
            (fault == "map_loss" || fault == "both_loss");
        const bool corrupted = map_index == 2 && fault == "map_outlier";
        interventions << "actual-structured-ICP," << entry.first << ','
                      << (omitted ? "omitted" : corrupted ? "injected" : "original")
                      << ',' << (corrupted ? 100 : 0) << ",0\n";
        if (!omitted) {
        loc::GlobalMapObservation observation;
        observation.context = Context(config, entry.first, query.receive,
                                      query.sequence, "actual-structured-ICP");
        observation.map_from_base = {
            query.match.map_from_base.translation(),
            Eigen::Quaterniond(query.match.map_from_base.linear())};
        if (corrupted) observation.map_from_base.translation.x() += 100;
        const Eigen::MatrixXd map_loading = root.middleRows(query.offset, 6);
        observation.covariance = map_loading * map_loading.transpose();
        observation.covariance_source = "selected-original-point-ICP-fixed-prior";
        loc::GraphSourceQualification qualification;
        qualification.quality_valid = true;
        qualification.uncertainty_model_id = observation.covariance_source;
        qualification.local_correlation.emplace();
        auto& correlation = *qualification.local_correlation;
        correlation.motion_error_model_id = config.motion_error_model_id;
        correlation.odometry_sequence = sample.sequence;
        correlation.conditioning_time = sample.odometry.timestamp;
        correlation.cross_covariance = map_loading.transpose();
        correlation.conditional_independence_evidence_id =
            "complete-selected-family-root-zero-conditional-map-residual";
        correlation.conditional_model_qualified = true;
        correlation.residual_independent_of_history_and_sources = true;
        graph.SubmitMapObservation(observation, qualification);
        }
      }
      ++map_index;
    }
    auto observations = gnss_;
    const bool synthetic = !fault.empty() && fault != "map_loss" &&
                           fault != "map_outlier" &&
                           fault != "recorded_gnss_omitted";
    if (synthetic) {
      observations.clear();
      if (node_index >= 2) {
        auto observation = gnss_.front();
        observation.context.measurement_time.nanoseconds = entry.first;
        observation.context.receive_time.nanoseconds = node.receive_ns;
        observation.context.sequence = 1'000'000 + node_index;
        observation.context.source = "SYNTHETIC-GNSS-native-node-fixed-anchor";
        // Same original local nodes, deterministic common-anchor trajectory;
        // not recorded repeated GNSS or an independent physical reference.
        const auto& first = nodes_.begin()->second.local.state;
        const auto& anchor = maps_.begin()->second.match.map_from_base;
        const Eigen::Matrix3d rotation =
            anchor.linear() * first.orientation.toRotationMatrix().transpose();
        const Eigen::Vector3d point = anchor.translation() + rotation *
            (native.position - first.position +
             native.orientation * observation.point_in_base);
        observation.position = point.head<2>();
        observations.push_back(std::move(observation));
      }
    }
    for (auto observation : observations) {
      if (observation.context.measurement_time.nanoseconds != entry.first ||
          !(arm & 1))
        continue;
      const bool omitted =
          fault == "recorded_gnss_omitted" ||
          (fault == "gnss_loss" && node_index == 3) ||
          (fault == "both_loss" && (node_index == 2 || node_index == 3)) ||
          (fault == "gnss_omitted" && node_index == 2);
      const bool corrupted = (fault == "gnss_outlier" && node_index == 2) ||
          (fault == "conflict" && (node_index == 2 || node_index == 3));
      interventions << observation.context.source << ',' << entry.first << ','
                    << (omitted ? "omitted" : corrupted ? "injected" :
                        synthetic ? "synthetic_nominal" : "original")
                    << ',' << (corrupted ? 100 : 0) << ',' << synthetic << '\n';
      if (omitted) continue;
      if (corrupted) observation.position.x() += 100;
      const auto original = observation.context;
      observation.context = Context(config, entry.first,
                                    original.receive_time.nanoseconds,
                                    original.sequence, original.source);
      observation.context.provenance = original.provenance;
      loc::GraphSourceQualification approximation;
      approximation.quality_valid = true;
      approximation.uncertainty_model_id = observation.covariance_source;
      approximation.local_correlation.emplace();
      auto& conditional = *approximation.local_correlation;
      conditional.motion_error_model_id = config.motion_error_model_id;
      conditional.odometry_sequence = sample.sequence;
      conditional.conditioning_time = sample.odometry.timestamp;
      conditional.cross_covariance = Eigen::MatrixXd::Zero(root.cols(), 2);
      conditional.conditional_independence_evidence_id =
          "DIAGNOSTIC-ASSUMPTION-not-field-evidence-conditioned-fixed-prior-GNSS";
      // Zero cross and unchanged GNSS residual covariance define ONLY the
      // explicitly approximate conditional model; keep dependency metadata.
      conditional.conditional_model_qualified = true;
      conditional.residual_independent_of_history_and_sources = true;
      graph.SubmitHorizontalObservation(observation, approximation);
    }
    journal();
    loc::FixedLagAlignmentResult result;
    structured_closure::Require(graph.GetResult(sample.odometry.timestamp, &result));
    states << entry.first << ',' << bool(result.estimate);
    if (result.estimate) {
      for (double value : result.estimate->map_from_base.translation)
        states << ',' << value;
    } else {
      states << ",,,";
    }
    states << ',' << result.diagnostic_only << ','
           << static_cast<int>(result.trust_level) << ',';
    Text(states, result.assessment.message());
    states << ',' << result.submitted_observations << ','
           << result.rejected_observations << ','
           << (result.estimate ? result.estimate->correction_id : 0) << ','
           << result.verification_pending << ',' << result.full_pose_evidence_fresh
           << ',' << result.odometry_fresh << '\n';
    previous = &node;
    previous_time = entry.first;
    ++node_index;
  }
  journal();
  loc::FixedLagAlignmentResult result;
  structured_closure::Require(graph.GetResult(
      {nodes_.rbegin()->first, config.clock_domain}, &result));
  for (const auto& source : result.sources)
    statistics << source.source << ',' << source.submitted << ','
               << source.alignment_updates << ',' << source.retained << ','
               << source.rejected << '\n';
  decisions.flush();
  states.flush();
  statistics.flush();
  interventions.flush();
  if (!decisions || !states || !statistics || !interventions)
    throw std::runtime_error("incomplete actual graph journals");
  if (!result.estimate || !result.diagnostic_only ||
      result.trust_level == loc::LocalizationTrustLevel::kQualified ||
      !result.estimate->map_from_base.translation.allFinite())
    throw std::runtime_error("real structured graph did not produce diagnostic estimate");
  return *result.estimate;
}

inline void NativeStructuredClosure::ReplayFrozen(
    const std::filesystem::path& input, const std::filesystem::path& output,
    bool causal_prefixes) {
  namespace loc = localization;
  if (!std::filesystem::create_directory(output))
    throw std::runtime_error("frozen output must be a new directory");
  NativeStructuredClosure replay(output);
  const auto identity = ReadRows((input / "identity.csv").string(),
      "map_id,map_version,map_content_id,calibration_id,map_frame,odom_frame,base_frame", 7);
  if (identity.size() != 1) throw std::runtime_error("one frozen identity required");
  const auto& id = identity.front();
  auto& config = replay.frozen_config_;
  config.reference = {id[0], id[1], id[3]};
  config.reference.geodetic.state = loc::GeodeticReferenceState::kDiagnosticOnly;
  config.map_content_id = id[2];
  config.map_frame = {id[4]};
  config.odom_frame = {id[5]};
  config.base_frame = {id[6]};
  Eigen::Index offset = 0;
  for (const auto& row : ReadRows((input / "nodes.csv").string(),
      "timestamp_ns,left_imu_ns,right_imu_ns,left_sequence,right_sequence,"
      "interpolated,px,py,pz,qx,qy,qz,qw,native_receive_ns", 14)) {
    const auto stamp = Time(row[0]);
    Node node;
    node.left_ns = Time(row[1]);
    node.right_ns = Time(row[2]);
    node.left_sequence = Time(row[3]);
    node.right_sequence = Time(row[4]);
    node.receive_ns = Time(row[13]);
    node.offset = offset;
    offset += 15;
    auto& state = node.local.state;
    state.valid = true;
    state.stamp.sequence = node.left_sequence;
    state.stamp.time = stamp * 1e-9;
    state.epoch = {"frozen-selected-native-family", 1};
    state.position = {Number(row[6]), Number(row[7]), Number(row[8])};
    state.orientation = Eigen::Quaterniond(
        Number(row[12]), Number(row[9]), Number(row[10]), Number(row[11]));
    if ((!replay.nodes_.empty() && stamp <= replay.nodes_.rbegin()->first) ||
        node.left_ns > stamp || stamp > node.right_ns ||
        !replay.nodes_.emplace(stamp, std::move(node)).second)
      throw std::runtime_error("invalid frozen original-time node order/bracket");
  }
  for (const auto& row : ReadRows((input / "maps.csv").string(),
      "timestamp_ns,receive_ns,sequence,px,py,pz,qx,qy,qz,qw", 10)) {
    MapQuery query;
    query.receive = Time(row[1]);
    query.sequence = Time(row[2]);
    query.offset = offset;
    offset += 6;
    query.match.map_from_base = Eigen::Isometry3d::Identity();
    query.match.map_from_base.translation() =
        Eigen::Vector3d(Number(row[3]), Number(row[4]), Number(row[5]));
    query.match.map_from_base.linear() = Eigen::Quaterniond(
        Number(row[9]), Number(row[6]), Number(row[7]), Number(row[8]))
        .toRotationMatrix();
    if (!replay.nodes_.count(Time(row[0])) ||
        !replay.maps_.emplace(Time(row[0]), std::move(query)).second)
      throw std::runtime_error("frozen map lacks unique native node");
  }
  for (const auto& row : ReadRows((input / "gnss.csv").string(),
      "timestamp_ns,receive_ns,sequence,x,y,sigma_m,lever_x,lever_y,lever_z", 9)) {
    loc::GlobalHorizontalObservation observation;
    observation.context.measurement_time = {Time(row[0]), config.clock_domain};
    observation.context.receive_time = {Time(row[1]), config.clock_domain};
    observation.context.sequence = Time(row[2]);
    observation.context.source = "raw-gga-horizontal-diagnostic";
    observation.context.provenance.gnss_map_construction =
        loc::ObservationDependency::kPresent;
    observation.context.provenance.odometry_initialization =
        loc::ObservationDependency::kAbsent;
    observation.context.provenance.odometry_deskew =
        loc::ObservationDependency::kAbsent;
    observation.context.provenance.shared_local_scans =
        loc::ObservationDependency::kAbsent;
    observation.position = {Number(row[3]), Number(row[4])};
    const double sigma = Number(row[5]);
    if (sigma <= 0 || !replay.nodes_.count(Time(row[0])))
      throw std::runtime_error("invalid frozen original GNSS");
    observation.covariance = Eigen::Matrix2d::Identity() * sigma * sigma;
    observation.point_in_base = {Number(row[6]), Number(row[7]), Number(row[8])};
    observation.covariance_source = "explicit-uncalibrated-diagnostic-sigma";
    replay.gnss_.push_back(std::move(observation));
  }
  const auto matrix = [&](const char* name) {
    std::ifstream stream(input / name);
    std::string line, extra;
    if (!ReadLine(stream, &line) || ReadLine(stream, &extra))
      throw std::runtime_error("one frozen matrix required");
    const auto parts = Fields(line, ':');
    if (parts.size() != 2) throw std::runtime_error("invalid frozen matrix");
    const auto shape = Fields(parts[0], 'x');
    if (shape.size() != 2) throw std::runtime_error("invalid frozen matrix shape");
    const auto rows = Time(shape[0]), columns = Time(shape[1]);
    if (rows != offset || columns > offset || rows > 2048)
      throw std::runtime_error("frozen family shape exceeds selected queries");
    const auto entries = Fields(parts[1], ';');
    if (entries.size() != static_cast<std::size_t>(rows * columns))
      throw std::runtime_error("invalid frozen matrix entries");
    Eigen::MatrixXd result(rows, columns);
    for (int64_t i = 0; i < rows; ++i)
      for (int64_t j = 0; j < columns; ++j)
        result(i, j) = Number(entries[i * columns + j]);
    return result;
  };
  if (replay.nodes_.size() != 5 || replay.maps_.size() != 4 ||
      replay.gnss_.size() != 1)
    throw std::runtime_error("fault schedule requires five nodes/four maps/one GNSS");
  const auto covariance = matrix("covariance.csv");
  const auto root = matrix("root.csv");
  if (covariance.cols() != offset ||
      (covariance.diagonal().array() <= 0).any())
    throw std::runtime_error("invalid frozen covariance");
  const Eigen::MatrixXd normalization =
      covariance.diagonal().cwiseSqrt().cwiseInverse().asDiagonal();
  if ((normalization * (root * root.transpose() - covariance) * normalization)
          .cwiseAbs().maxCoeff() > 1e-11)
    throw std::runtime_error("frozen root does not reconstruct covariance");
  if (causal_prefixes) {
    std::set<int64_t> watermarks;
    for (const auto& entry : replay.nodes_) watermarks.insert(entry.second.receive_ns);
    for (const auto& entry : replay.maps_) watermarks.insert(entry.second.receive);
    for (const auto& observation : replay.gnss_)
      watermarks.insert(observation.context.receive_time.nanoseconds);
    std::ofstream journal(output / "prefixes.csv");
    journal << "watermark_ns,nodes,maps,gnss,rows,rank,root_error,graphs\n";
    for (const auto watermark : watermarks) {
      NativeStructuredClosure prefix(output / std::to_string(watermark));
      prefix.frozen_config_ = config;
      std::vector<Eigen::Index> indices;
      const auto append = [&](Eigen::Index old, Eigen::Index rows) {
        const auto next = static_cast<Eigen::Index>(indices.size());
        for (Eigen::Index i = 0; i < rows; ++i) indices.push_back(old + i);
        return next;
      };
      for (const auto& entry : replay.nodes_)
        if (entry.second.receive_ns <= watermark) {
          auto node = entry.second;
          node.offset = append(node.offset, 15);
          prefix.nodes_.emplace(entry.first, std::move(node));
        }
      for (const auto& entry : replay.maps_)
        if (entry.second.receive <= watermark && prefix.nodes_.count(entry.first)) {
          auto query = entry.second;
          query.offset = append(query.offset, 6);
          prefix.maps_.emplace(entry.first, std::move(query));
        }
      if (prefix.maps_.size() < 2) continue;
      for (const auto& observation : replay.gnss_)
        if (observation.context.receive_time.nanoseconds <= watermark &&
            prefix.nodes_.count(observation.context.measurement_time.nanoseconds))
          prefix.gnss_.push_back(observation);
      std::filesystem::create_directory(prefix.output_);
      Eigen::MatrixXd current(indices.size(), indices.size());
      for (Eigen::Index i = 0; i < current.rows(); ++i)
        for (Eigen::Index j = 0; j < current.cols(); ++j)
          current(i, j) = covariance(indices[i], indices[j]);
      double error = 0;
      const auto current_root = structured_closure::Root(current, &error);
      for (int arm = 0; arm < 4; ++arm)
        for (const bool lag : {false, true})
          prefix.RunArm(current_root, arm, lag);
      journal << watermark << ',' << prefix.nodes_.size() << ','
              << prefix.maps_.size() << ',' << prefix.gnss_.size() << ','
              << current.rows() << ',' << current_root.cols() << ','
              << std::setprecision(17) << error << ",8\n";
      journal.flush();
    }
    if (!journal) throw std::runtime_error("incomplete causal prefix journal");
    return;
  }
  std::ofstream runs(output / "run_status.csv");
  runs << "run,status,error\n";
  for (const auto& fault : {"", "map_loss", "map_outlier",
                            "recorded_gnss_omitted", "synthetic_nominal",
                            "gnss_loss", "gnss_omitted", "gnss_outlier",
                            "conflict", "both_loss"})
    for (int arm = 0; arm < 4; ++arm)
      for (bool lag : {false, true}) {
        const auto name = (std::string(fault).empty() ? "" :
                           std::string(fault) + "_") + Arm(arm) +
                          (lag ? "_lag" : "_no_eviction");
        try {
          replay.RunArm(root, arm, lag, fault);
          runs << name << ",completed,\"\"\n";
        } catch (const std::exception& error) {
          // Each graph is isolated; retain its failure before independent runs.
          runs << name << ",failed,";
          Text(runs, error.what());
          runs << '\n';
        }
        runs.flush();
      }
  if (!runs) throw std::runtime_error("incomplete frozen run statuses");
}
}  // namespace openloc::replay
