// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "candidates/global/global_alignment.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "Eigen/Cholesky"
#include "Eigen/LU"
#include "Eigen/QR"
#include "alignment_recovery.h"
#include "candidates/global/fixed_lag_graph.h"

namespace openloc {
namespace localization {
namespace unified {
namespace {

Eigen::Matrix<double, 6, 1> Difference(const Eigen::Isometry3d& a,
                                       const Eigen::Isometry3d& b) {
  Eigen::Matrix<double, 6, 1> difference;
  difference.head<3>() = b.translation() - a.translation();
  difference.tail<3>() =
      LogRotation(Eigen::Quaterniond(b.linear() * a.linear().transpose()));
  return difference;
}

bool ValidExactTime(const std::optional<int64_t>& nanoseconds, double seconds) {
  return !nanoseconds.has_value() ||
         (*nanoseconds > 0 &&
          static_cast<double>(*nanoseconds) * 1e-9 == seconds);
}

}  // namespace

GlobalAlignment::GlobalAlignment(const GlobalConfig& config)
    : config_(config) {}
GlobalAlignment::~GlobalAlignment() = default;

Result GlobalAlignment::ValidateConfig() const {
  if (config_.diagnostic_fixed_gain &&
      (!config_.diagnostic_only || config_.enable_fixed_lag_graph))
    return {Reason::CONFIG_INVALID,
            "Fixed mean gain requires the explicit diagnostic owner"};
  if (config_.graph_allow_uncalibrated_inputs && !config_.enable_fixed_lag_graph)
    return {Reason::CONFIG_INVALID,
            "Uncalibrated modeled inputs require the fixed-lag graph owner"};
  if (config_.enable_diagnostic_horizontal_bootstrap &&
      (!config_.diagnostic_only || config_.enable_fixed_lag_graph)) {
    return {Reason::CONFIG_INVALID,
            "Horizontal bootstrap requires the explicit diagnostic owner"};
  }
  if (config_.enable_fixed_lag_graph &&
      (config_.diagnostic_only || !config_.motion_error_model_qualified ||
       !std::isfinite(config_.graph_solve_budget_ms) ||
       config_.graph_solve_budget_ms <= 0.0 ||
       !std::isfinite(config_.graph_robust_threshold) ||
       config_.graph_robust_threshold <= 0.0 || config_.graph_max_states < 2 ||
       config_.graph_max_observations < 1 ||
       config_.graph_max_error_dimension < 15 ||
       config_.graph_max_iterations == 0 ||
       config_.graph_max_iterations > 100 ||
       !std::isfinite(config_.graph_max_linearization_angle) ||
       config_.graph_max_linearization_angle <= 0.0 ||
       config_.graph_max_linearization_angle >= 1.0)) {
    return {
        Reason::CONFIG_INVALID,
        "Fixed-lag graph requires qualified motion model and finite budgets"};
  }
  if (config_.clock_id.empty() || config_.map_id.empty() ||
      config_.map_version.empty() || config_.calibration_id.empty() ||
      config_.verification_frames < 2) {
    return {Reason::CONFIG_INVALID, "missing global identity or verification"};
  }
  for (double value :
       {config_.history_duration, config_.max_interpolation_gap,
        config_.max_observation_age, config_.local_timeout,
        config_.global_timeout, config_.innovation_gate,
        config_.max_position_std, config_.max_attitude_std,
        config_.verification_position, config_.verification_angle,
        config_.position_drift_variance_rate,
        config_.attitude_drift_variance_rate}) {
    if (!std::isfinite(value) || value <= 0.0) {
      return {Reason::CONFIG_INVALID, "invalid global budget"};
    }
  }
  if (!std::isfinite(config_.future_tolerance) ||
      config_.future_tolerance < 0.0 ||
      config_.history_duration < config_.max_observation_age) {
    return {Reason::CONFIG_INVALID, "invalid global history/time policy"};
  }
  return {};
}

Result GlobalAlignment::Reject(Reason reason, const std::string& message,
                               SourceCounters* source) {
  if (source != nullptr) {
    ++source->rejected;
    source->reason = reason;
    source->duplicates += reason == Reason::DUPLICATE;
    source->regressions += reason == Reason::TIMESTAMP_REGRESSION;
  }
  reason_ = reason;
  return {reason, message};
}

void GlobalAlignment::ClearEpoch(const Epoch& epoch) {
  horizontal_bootstrap_.reset();
  horizontal_source_.clear();
  graph_.reset();
  motion_history_.clear();
  history_.clear();
  process_reference_in_odom_.setZero();
  sources_.clear();
  evidence_.clear();
  state_ = GlobalState();
  state_.epoch = epoch;
  state_.diagnostic_only =
      config_.diagnostic_only || config_.graph_allow_uncalibrated_inputs;
  candidate_frames_ = 0;
  candidate_time_ = 0.0;
  reason_ = Reason::GLOBAL_UNAVAILABLE;
}

Result GlobalAlignment::AddLocal(const LocalState& local,
                                 const MotionIncrement* motion) {
  if (!local.valid || local.epoch.session.empty() ||
      local.epoch.generation == 0 || local.stamp.sequence == 0 ||
      local.stamp.clock_id != config_.clock_id ||
      !std::isfinite(local.stamp.time) || local.stamp.time <= 0.0 ||
      !std::isfinite(local.stamp.receive_time) ||
      local.stamp.receive_time - local.stamp.time > config_.local_timeout ||
      local.stamp.time - local.stamp.receive_time > config_.future_tolerance ||
      !local.orientation.coeffs().allFinite() ||
      std::abs(local.orientation.norm() - 1.0) > 1e-6 ||
      !ValidGlobalPose(Pose(local)) ||
      !ValidCovariance(PoseCovariance(local))) {
    Invalidate(Reason::INVALID_INPUT);
    return {Reason::INVALID_INPUT, "invalid local snapshot"};
  }
  if (retired_sessions_.count(local.epoch.session) != 0) {
    return {Reason::EPOCH_MISMATCH, "retired producer session"};
  }
  if (config_.enable_fixed_lag_graph) {
    if (local.joint_error &&
        local.joint_error->covariance.rows() > config_.graph_max_error_dimension) {
      Invalidate(Reason::INVALID_INPUT);
      return {Reason::INVALID_INPUT, "Joint error exceeds configured graph capacity"};
    }
    const auto joint = ValidateJointLocalError(local);
    if (!joint.ok()) {
      Invalidate(joint.reason);
      return joint;
    }
  }
  if (!history_.empty()) {
    const auto& last = history_.back();
    if (local.epoch.session == last.epoch.session &&
        local.epoch.generation < last.epoch.generation) {
      return {Reason::EPOCH_MISMATCH, "old local epoch"};
    }
    if (local.stamp.time <= last.stamp.time) {
      return {Reason::TIMESTAMP_REGRESSION, "local time did not advance"};
    }
    if (local.epoch == last.epoch &&
        local.stamp.sequence <= last.stamp.sequence) {
      return {Reason::DUPLICATE, "local sequence did not advance"};
    }
  }
  if (history_.empty() || local.epoch != state_.epoch) {
    if (!history_.empty() && state_.epoch.session != local.epoch.session) {
      if (retired_sessions_.size() >= 128) {
        return {Reason::EPOCH_MISMATCH, "retired-session capacity exhausted"};
      }
      retired_sessions_.insert(state_.epoch.session);
    }
    ClearEpoch(local.epoch);
  }
  if (!local.covariance_model_valid && !config_.diagnostic_only) {
    Invalidate(Reason::INVALID_INPUT);
    history_.clear();
    motion_history_.clear();
    process_reference_in_odom_ = local.position;
    history_.push_back(local);
    return {Reason::INVALID_INPUT,
            "Local covariance error model is unavailable"};
  }
  if (!history_.empty() && !history_.back().covariance_model_valid &&
      !config_.diagnostic_only) {
    history_.clear();
    motion_history_.clear();
  }
  if (!config_.diagnostic_only && !config_.graph_allow_uncalibrated_inputs &&
      !local.uncertainty_qualified) {
    return {Reason::UNCERTAINTY_UNQUALIFIED,
            "local uncertainty model is unqualified"};
  }
  if (history_.empty()) {
    process_reference_in_odom_ = local.position;
  }
  if (config_.enable_fixed_lag_graph && !history_.empty()) {
    if (motion == nullptr || motion->epoch != local.epoch ||
        motion->start.time != history_.back().stamp.time ||
        motion->start.sequence != history_.back().stamp.sequence ||
        motion->end.time != local.stamp.time ||
        motion->end.sequence != local.stamp.sequence) {
      Invalidate(Reason::HISTORY_UNAVAILABLE);
      history_.clear();
      motion_history_.clear();
      process_reference_in_odom_ = local.position;
      history_.push_back(local);
      return {
          Reason::HISTORY_UNAVAILABLE,
          "Motion gap revoked global alignment; fresh local history retained"};
    }
    const auto joint = ValidateJointMotionWithValidatedLocalErrors(
        history_.back(), local, *motion);
    if (!joint.ok()) {
      Invalidate(joint.reason);
      history_.clear();
      motion_history_.clear();
      process_reference_in_odom_ = local.position;
      history_.push_back(local);
      Invalidate(Reason::HISTORY_UNAVAILABLE);
      return joint;
    }
    motion_history_[local.stamp.time] = *motion;
  }
  history_.push_back(local);
  while (history_.size() > 1 &&
         (local.stamp.time - history_.front().stamp.time >
              config_.history_duration ||
          history_.size() > 5000)) {
    history_.pop_front();
  }
  while (!motion_history_.empty() &&
         motion_history_.begin()->first <= history_.front().stamp.time) {
    motion_history_.erase(motion_history_.begin());
  }
  if (graph_ && graph_->initialized() && motion != nullptr) {
    const auto added = graph_->AddLocal(local, *motion);
    if (!added.ok()) {
      Invalidate(added.reason);
      return added;
    }
    AdoptGraphState();
  }
  return {};
}

Result GlobalAlignment::SeedDiagnostic(const Eigen::Isometry3d& map_to_odom,
                                       const Matrix6d& covariance,
                                       double time) {
  if (!config_.diagnostic_only || config_.enable_fixed_lag_graph ||
      !ValidateConfig().ok() || history_.empty() || state_.valid ||
      !sources_.empty() || time != history_.back().stamp.time ||
      !ValidGlobalPose(map_to_odom) || !ValidCovariance(covariance)) {
    return {Reason::INVALID_INPUT, "invalid diagnostic prior or owner mode"};
  }
  state_.map_to_odom = map_to_odom;
  state_.covariance = covariance;
  state_.covariance_time = time;
  state_.last_observation = time;
  state_.last_full_observation = time;
  state_.valid = true;
  state_.diagnostic_only = true;
  return {};
}

void GlobalAlignment::AdoptGraphState() {
  const uint64_t correction = state_.correction_id + 1;
  state_ = graph_->state();
  state_.diagnostic_only = config_.graph_allow_uncalibrated_inputs;
  state_.correction_id = correction;
}

Result GlobalAlignment::InitializeGraph(const GlobalObservation& observation,
                                        const LocalState& local) {
  FixedLagConfig policy;
  policy.lag = config_.history_duration;
  policy.solve_budget_ms = config_.graph_solve_budget_ms;
  policy.innovation_gate = config_.innovation_gate;
  policy.robust_threshold = config_.graph_robust_threshold;
  policy.max_states = config_.graph_max_states;
  policy.max_observations = config_.graph_max_observations;
  policy.max_error_dimension = config_.graph_max_error_dimension;
  policy.max_iterations = config_.graph_max_iterations;
  policy.max_linearization_angle = config_.graph_max_linearization_angle;
  std::unique_ptr<FixedLagGraph> candidate(new FixedLagGraph(policy));
  std::vector<LocalState> bootstrap;
  std::vector<MotionIncrement> bootstrap_motion;
  const double conditioning_time =
      observation.local_error_time.value_or(observation.stamp.time);
  if (observation.conditional_local_error_qualified) {
    for (const auto& snapshot : history_) {
      if (snapshot.stamp.time <= local.stamp.time ||
          snapshot.stamp.time > conditioning_time)
        continue;
      const auto motion = motion_history_.find(snapshot.stamp.time);
      if (motion == motion_history_.end())
        return {Reason::HISTORY_UNAVAILABLE, "Graph bootstrap motion is missing"};
      bootstrap.push_back(snapshot);
      bootstrap_motion.push_back(motion->second);
    }
  }
  auto result = candidate->Initialize(local, observation, bootstrap, bootstrap_motion);
  if (!result.ok()) {
    return result;
  }
  for (const auto& snapshot : history_) {
    if (snapshot.stamp.time <= (bootstrap.empty() ? local.stamp.time :
                                                  bootstrap.back().stamp.time)) {
      continue;
    }
    const auto motion = motion_history_.find(snapshot.stamp.time);
    if (motion == motion_history_.end()) {
      return {Reason::HISTORY_UNAVAILABLE, "Graph bootstrap motion is missing"};
    }
    result = candidate->AddLocal(snapshot, motion->second);
    if (!result.ok()) {
      return result;
    }
  }
  graph_.swap(candidate);
  AdoptGraphState();
  return {};
}

const LocalState* GlobalAlignment::latest_local() const {
  return history_.empty() ? nullptr : &history_.back();
}

Result GlobalAlignment::Lookup(double time, LocalState* local) const {
  if (local == nullptr || !std::isfinite(time) || history_.empty() ||
      time < history_.front().stamp.time || time > history_.back().stamp.time) {
    return {Reason::HISTORY_UNAVAILABLE, "no retained local bracket"};
  }
  if (!history_.back().covariance_model_valid && !config_.diagnostic_only) {
    return {Reason::INVALID_INPUT,
            "Local covariance error model is unavailable"};
  }
  auto upper = std::lower_bound(history_.begin(), history_.end(), time,
                                [](const LocalState& state, double stamp) {
                                  return state.stamp.time < stamp;
                                });
  if (upper->stamp.time == time) {
    *local = *upper;
    return {};
  }
  const auto& before = *std::prev(upper);
  const double duration = upper->stamp.time - before.stamp.time;
  if (duration > config_.max_interpolation_gap) {
    return {Reason::HISTORY_UNAVAILABLE, "local interpolation gap exceeded"};
  }
  const double weight = (time - before.stamp.time) / duration;
  *local = before;
  local->stamp.time = time;
  local->position = (1.0 - weight) * before.position + weight * upper->position;
  local->velocity = (1.0 - weight) * before.velocity + weight * upper->velocity;
  local->orientation = before.orientation.slerp(weight, upper->orientation);
  // Adjacent states are correlated. This is a conservative interpolation bound.
  local->covariance =
      2.0 * ((1.0 - weight) * before.covariance + weight * upper->covariance);
  local->covariance_square_root.reset();
  return {};
}

Result GlobalAlignment::LookupExact(double time, LocalState* local) const {
  if (!std::isfinite(time)) {
    return {Reason::INVALID_INPUT, "Invalid exact lookup time"};
  }
  const auto found =
      std::lower_bound(history_.begin(), history_.end(), time,
                       [](const LocalState& state, double stamp) {
                         return state.stamp.time < stamp;
                       });
  if (found == history_.end() || found->stamp.time != time) {
    return {Reason::HISTORY_UNAVAILABLE, "No exact retained local snapshot"};
  }
  return Lookup(time, local);
}

Result GlobalAlignment::Observe(const GlobalObservation& observation,
                                ObservationInnovation* diagnostics) {
  if (diagnostics != nullptr) {
    *diagnostics = ObservationInnovation();
    diagnostics->unavailable_reason = "prior_state_unavailable";
  }
  const auto existing = evidence_.find(observation.source);
  if (!config_.enable_fixed_lag_graph || existing == evidence_.end() ||
      observation.stamp.time >= existing->second.observation.stamp.time) {
    evidence_.erase(observation.source);
  }
  if (sources_.size() >= 32 && sources_.count(observation.source) == 0) {
    return {Reason::INVALID_INPUT, "too many global sources"};
  }
  auto& source = sources_[observation.source];
  const bool bounded_horizontal =
      config_.enable_heading_bootstrap && !state_.valid &&
      !observation.conditional_local_error_qualified &&
      (observation.kind == GlobalObservation::Kind::HEADING ||
       observation.kind == GlobalObservation::Kind::POSITION ||
       observation.kind == GlobalObservation::Kind::HORIZONTAL_POSITION);
  const auto valid_config = ValidateConfig();
  if (!valid_config.ok()) {
    return Reject(valid_config.reason, valid_config.message, &source);
  }
  if (observation.source.empty() || !observation.quality_valid ||
      (config_.enable_fixed_lag_graph && !observation.independent_of_local &&
       !observation.conditional_local_error_qualified && !bounded_horizontal) ||
      (!config_.enable_fixed_lag_graph &&
       observation.conditional_local_error_qualified) ||
      !ValidGlobalObservationGeometry(observation) ||
      observation.stamp.sequence == 0 ||
      !std::isfinite(observation.stamp.time) || observation.stamp.time <= 0.0 ||
      !std::isfinite(observation.stamp.receive_time) ||
      !ValidExactTime(observation.measurement_timestamp_ns,
                      observation.stamp.time) ||
      !ValidExactTime(observation.receive_timestamp_ns,
                      observation.stamp.receive_time)) {
    return Reject(Reason::INVALID_INPUT, "invalid global evidence", &source);
  }
  if (!config_.diagnostic_only && !config_.graph_allow_uncalibrated_inputs &&
      (!observation.uncertainty_qualified || observation.diagnostic_only)) {
    return Reject(Reason::UNCERTAINTY_UNQUALIFIED,
                  "global observation covariance is unqualified", &source);
  }
  if (observation.stamp.clock_id != config_.clock_id ||
      observation.stamp.time - observation.stamp.receive_time >
          config_.future_tolerance ||
      observation.stamp.receive_time - observation.stamp.time >
          config_.max_observation_age) {
    return Reject(Reason::CLOCK_INVALID, "global observation time invalid",
                  &source);
  }
  if (observation.map_id != config_.map_id ||
      observation.map_version != config_.map_version ||
      observation.calibration_id != config_.calibration_id ||
      (!config_.map_content_id.empty() &&
       observation.map_content_id != config_.map_content_id) ||
      (!config_.map_frame.empty() &&
       observation.map_frame != config_.map_frame) ||
      (!config_.base_frame.empty() &&
       observation.base_frame != config_.base_frame)) {
    return Reject(Reason::MAP_MISMATCH, "map/calibration identity mismatch",
                  &source);
  }
  if (observation.epoch != state_.epoch) {
    return Reject(Reason::EPOCH_MISMATCH, "global observation epoch mismatch",
                  &source);
  }
  if (observation.conditional_local_error_qualified) {
    LocalState local;
    const auto found = LookupExact(
        observation.local_error_time.value_or(observation.stamp.time), &local);
    if (!found.ok()) return Reject(found.reason, found.message, &source);
    const auto model = FixedLagGraph::ValidateConditionalObservation(local, observation);
    if (!model.ok()) return Reject(model.reason, model.message, &source);
  }
  if (config_.enable_fixed_lag_graph &&
      !config_.graph_retain_on_full_pose_staleness && graph_ && state_.valid &&
      observation.stamp.receive_time - state_.last_full_observation >
          config_.global_timeout) {
    Invalidate(Reason::GLOBAL_STALE);
  }
  const bool graph_update =
      graph_ && graph_->initialized() && state_.valid && !observation.recovery;
  if (!graph_update && (observation.stamp.time <= source.last_measurement ||
                        observation.stamp.sequence <= source.last_sequence)) {
    return Reject(observation.stamp.time == source.last_measurement
                      ? Reason::DUPLICATE
                      : Reason::TIMESTAMP_REGRESSION,
                  "global source frontier did not advance", &source);
  }
  source.last_measurement =
      std::max(source.last_measurement, observation.stamp.time);
  source.last_receive =
      std::max(source.last_receive, observation.stamp.receive_time);
  source.last_sequence =
      std::max(source.last_sequence, observation.stamp.sequence);
  if (observation.ambiguous) {
    evidence_.erase(observation.source);
    candidate_frames_ = 0;
    state_.verifying = false;
    return Reject(Reason::RELOCALIZATION_AMBIGUOUS, "ambiguous map hypothesis",
                  &source);
  }
  LocalState local;
  const auto lookup = config_.enable_fixed_lag_graph
                          ? LookupExact(observation.stamp.time, &local)
                          : Lookup(observation.stamp.time, &local);
  if (!lookup.ok()) {
    return Reject(lookup.reason, lookup.message, &source);
  }
  if (observation.kind == GlobalObservation::Kind::HEADING) {
    Eigen::Vector3d direction =
        local.orientation * observation.direction_in_base;
    if (state_.valid) direction = state_.map_to_odom.linear() * direction;
    if (direction.head<2>().squaredNorm() < 1e-8) {
      return Reject(Reason::DEGENERATE,
                    "baseline azimuth is undefined near vertical", &source);
    }
  }
  if (graph_update) {
    const auto observed = graph_->Observe(observation, diagnostics);
    if (!observed.ok()) {
      return Reject(observed.reason, observed.message, &source);
    }
    AdoptGraphState();
    const auto latest = evidence_.find(observation.source);
    if (latest == evidence_.end() ||
        observation.stamp.time >= latest->second.observation.stamp.time) {
      evidence_[observation.source] = {observation, local};
    }
    ++source.accepted;
    source.reason = Reason::NONE;
    reason_ = observation.kind == GlobalObservation::Kind::POSE
                  ? Reason::NONE
                  : Reason::DEGENERATE;
    return {};
  }
  if (state_.valid && observation.stamp.time < state_.last_observation) {
    return Reject(Reason::TIMESTAMP_REGRESSION,
                  "observation older than committed global frontier", &source);
  }
  if (state_.valid) {
    state_ = Predict(observation.stamp.time);
  }
  if (observation.kind != GlobalObservation::Kind::POSE) {
    return ObservePartial(observation, local, &source, diagnostics);
  }
  const Eigen::Isometry3d measured = observation.pose * Pose(local).inverse();
  Matrix6d observation_bound, local_bound;
  const Matrix6d covariance =
      TransformCovarianceBound(observation.pose, local, observation.covariance,
                               diagnostics ? &observation_bound : nullptr,
                               diagnostics ? &local_bound : nullptr);
  if (!ValidCovariance(covariance)) {
    return Reject(Reason::INVALID_INPUT, "invalid alignment covariance",
                  &source);
  }
  // Diagnostic validity is an estimate basin, never a trusted full-pose lease.
  if (!config_.diagnostic_only && state_.valid &&
      observation.stamp.time - state_.last_full_observation >
          config_.global_timeout) {
    Invalidate(Reason::GLOBAL_STALE);
  }
  if (!state_.valid || observation.recovery) {
    graph_.reset();
    if (candidate_frames_ > 0 && observation.stamp.time <= candidate_time_) {
      return Reject(Reason::TIMESTAMP_REGRESSION,
                    "recovery needs distinct advancing measurement frames",
                    &source);
    }
    if (candidate_frames_ == 0 ||
        observation.stamp.time - candidate_time_ > config_.global_timeout ||
        candidate_georeferenced_ != observation.georeferenced ||
        Difference(candidate_, measured).head<3>().norm() >
            config_.verification_position ||
        Difference(candidate_, measured).tail<3>().norm() >
            config_.verification_angle) {
      candidate_ = measured;
      candidate_covariance_ = covariance;
      candidate_frames_ = 1;
      candidate_georeferenced_ = observation.georeferenced;
    } else {
      ++candidate_frames_;
      candidate_ = measured;
      candidate_covariance_ = covariance;
    }
    candidate_time_ = observation.stamp.time;
    state_.verifying = true;
    if (observation.recovery) {
      state_.valid = false;
    }
    if (candidate_frames_ < config_.verification_frames) {
      return Reject(Reason::RELOCALIZATION_VERIFYING,
                    "waiting for distinct consistent frames", &source);
    }
    if (config_.enable_fixed_lag_graph) {
      const auto initialized = InitializeGraph(observation, local);
      if (!initialized.ok()) {
        state_.valid = false;
        return Reject(initialized.reason, initialized.message, &source);
      }
    } else {
      state_.map_to_odom = candidate_;
      state_.covariance = candidate_covariance_;
      state_.covariance_time = observation.stamp.time;
      state_.georeferenced = candidate_georeferenced_;
      state_.valid = true;
      ++state_.correction_id;
    }
    state_.verifying = false;
    candidate_frames_ = 0;
    state_.horizontal.reset();
    horizontal_bootstrap_.reset();
    horizontal_source_.clear();
  } else {
    if (observation.stamp.time < state_.last_observation) {
      return Reject(Reason::TIMESTAMP_REGRESSION,
                    "global fusion frontier did not advance", &source);
    }
    const auto innovation = Difference(state_.map_to_odom, measured);
    // The same local trajectory can occur in both estimates.
    const Matrix6d innovation_covariance =
        2.0 * (state_.covariance + covariance);
    const double distance =
        innovation.dot(innovation_covariance.ldlt().solve(innovation));
    if (diagnostics != nullptr) {
      diagnostics->available = true;
      diagnostics->unavailable_reason.clear();
      diagnostics->prior_state_frame = "map_from_odom";
      diagnostics->observed_dimension =
          static_cast<uint32_t>(innovation.size());
      diagnostics->prior_state_dimension = 6;
      diagnostics->residual = innovation;
      diagnostics->covariance = innovation_covariance;
      diagnostics->nis = distance;
      diagnostics->gate_threshold = config_.innovation_gate;
      diagnostics->residual_basis = "map.translation_m.left_rotation_rad.v1";
      diagnostics->covariance_model =
          config_.diagnostic_fixed_gain
              ? "diagnostic_fixed_gain_full_pose_bound"
              : "diagnostic_ci_full_pose_bound";
      diagnostics->covariance_decomposition_available = true;
      diagnostics->prior_contribution = 2.0 * state_.covariance;
      diagnostics->observation_contribution = 2.0 * observation_bound;
      diagnostics->local_contribution = 2.0 * local_bound;
      diagnostics->prior_pose.translation = state_.map_to_odom.translation();
      diagnostics->prior_pose.rotation =
          Eigen::Quaterniond(state_.map_to_odom.linear()).normalized();
      diagnostics->prior_covariance = state_.covariance;
    }
    if (!std::isfinite(distance) || distance > config_.innovation_gate) {
      return Reject(Reason::INNOVATION_REJECTED, "global innovation rejected",
                    &source);
    }
    Matrix6d fused;
    Eigen::Matrix<double, 6, 1> correction;
    if (config_.diagnostic_fixed_gain) {
      correction = 0.5 * innovation;
      fused = FixedGainCovarianceBound(state_.covariance, covariance,
                                       Matrix6d::Identity(),
                                       0.5 * Matrix6d::Identity());
    } else {
      const Matrix6d previous_information =
          state_.covariance.ldlt().solve(Matrix6d::Identity());
      const Matrix6d measured_information =
          covariance.ldlt().solve(Matrix6d::Identity());
      // Fixed-weight covariance intersection prevents correlated double
      // counting.
      fused = (0.5 * previous_information + 0.5 * measured_information)
                  .ldlt()
                  .solve(Matrix6d::Identity());
      correction = fused * (0.5 * measured_information) * innovation;
    }
    if (!ValidCovariance(fused) || !correction.allFinite())
      return Reject(Reason::INVALID_INPUT,
                    "full-pose update is not finite/positive", &source);
    state_.map_to_odom.translation() += correction.head<3>();
    state_.map_to_odom.linear() =
        ExpRotation(correction.tail<3>()).toRotationMatrix() *
        state_.map_to_odom.linear();
    state_.covariance = 0.5 * (fused + fused.transpose());
    state_.georeferenced = observation.georeferenced;
    ++state_.correction_id;
  }
  state_.last_observation = observation.stamp.time;
  state_.last_full_observation = observation.stamp.time;
  state_.last_georeference =
      observation.georeferenced ? observation.stamp.time : 0.0;
  evidence_[observation.source] = {observation, local};
  ++source.accepted;
  source.reason = Reason::NONE;
  reason_ = Reason::NONE;
  return {};
}

Result GlobalAlignment::ObservePartial(const GlobalObservation& observation,
                                       const LocalState& local,
                                       SourceCounters* source,
                                       ObservationInnovation* diagnostics) {
  if (observation.recovery) {
    return Reject(Reason::DEGENERATE,
                  "partial evidence cannot complete full-pose recovery",
                  source);
  }
  if (!state_.valid) {
    if (config_.enable_heading_bootstrap &&
        (observation.kind == GlobalObservation::Kind::HEADING ||
         observation.kind == GlobalObservation::Kind::POSITION ||
         observation.kind == GlobalObservation::Kind::HORIZONTAL_POSITION)) {
      return ObserveHeadingAcquisition(observation, local, source, diagnostics);
    }
    if (observation.kind != GlobalObservation::Kind::POSITION &&
        observation.kind != GlobalObservation::Kind::HORIZONTAL_POSITION) {
      return Reject(Reason::GLOBAL_UNAVAILABLE,
                    "projected pose requires a verified alignment basin",
                    source);
    }
    if (config_.enable_diagnostic_horizontal_bootstrap &&
        observation.kind == GlobalObservation::Kind::HORIZONTAL_POSITION) {
      return ObserveHorizontal(observation, local, source, diagnostics);
    }
    // A directly measured point is useful before global rotation is known.
    evidence_[observation.source] = {observation, local};
    ++source->accepted;
    source->reason = Reason::NONE;
    return {};
  }
  if (observation.stamp.time < state_.last_observation) {
    return Reject(Reason::TIMESTAMP_REGRESSION,
                  "partial observation older than global fusion frontier",
                  source);
  }
  const auto predicted = state_.map_to_odom * Pose(local);
  Eigen::MatrixXd h;
  Eigen::MatrixXd noise;
  Eigen::MatrixXd observation_noise;
  Eigen::VectorXd residual;
  if (observation.kind == GlobalObservation::Kind::POSITION ||
      observation.kind == GlobalObservation::Kind::HORIZONTAL_POSITION) {
    const Eigen::Vector3d local_point =
        local.position + local.orientation * observation.point_in_base;
    const Eigen::Vector3d rotated = state_.map_to_odom.linear() * local_point;
    residual = observation.pose.translation() -
               (rotated + state_.map_to_odom.translation());
    h = Eigen::Matrix<double, 3, 6>::Zero();
    h.leftCols(3).setIdentity();
    h.rightCols(3) = -Skew(rotated);
    Eigen::Matrix<double, 3, 6> local_jacobian;
    local_jacobian.leftCols<3>() = state_.map_to_odom.linear();
    local_jacobian.rightCols<3>() =
        -predicted.linear() * Skew(observation.point_in_base);
    noise = 2.0 * (observation.covariance.topLeftCorner<3, 3>() +
                   local_jacobian * PoseCovariance(local) *
                       local_jacobian.transpose());
    observation_noise = observation.covariance.topLeftCorner<3, 3>();
    if (observation.kind == GlobalObservation::Kind::HORIZONTAL_POSITION) {
      h = h.topRows(2).eval();
      residual = residual.head(2).eval();
      noise = noise.topLeftCorner(2, 2).eval();
      observation_noise = observation_noise.topLeftCorner(2, 2).eval();
    }
  } else if (observation.kind == GlobalObservation::Kind::HEADING) {
    const Eigen::Vector3d direction =
        predicted.linear() * observation.direction_in_base;
    const double squared = direction.head<2>().squaredNorm();
    if (squared < 1e-8)
      return Reject(Reason::DEGENERATE, "baseline azimuth is undefined", source);
    const Eigen::RowVector3d azimuth =
        Eigen::RowVector3d(-direction.y(), direction.x(), 0.0) / squared;
    h = Eigen::Matrix<double, 1, 6>::Zero();
    h.rightCols(3) = -azimuth * Skew(direction);
    const double delta = observation.heading_rad -
                         std::atan2(direction.y(), direction.x());
    residual = Eigen::VectorXd::Constant(
        1, std::atan2(std::sin(delta), std::cos(delta)));
    const Eigen::RowVector3d local_jacobian =
        -azimuth * predicted.linear() * Skew(observation.direction_in_base);
    noise = Eigen::Matrix<double, 1, 1>::Constant(
        2.0 * (observation.heading_variance +
               (local_jacobian * local.covariance.block<3, 3>(6, 6) *
                local_jacobian.transpose())(0, 0)));
    observation_noise = Eigen::Matrix<double, 1, 1>::Constant(
        observation.heading_variance);
  } else {
    Matrix6d alignment_jacobian = Matrix6d::Identity();
    alignment_jacobian.topRightCorner<3, 3>() =
        -Skew(state_.map_to_odom.linear() * local.position);
    Matrix6d local_jacobian = Matrix6d::Zero();
    local_jacobian.topLeftCorner<3, 3>() = state_.map_to_odom.linear();
    local_jacobian.bottomRightCorner<3, 3>() = predicted.linear();
    h = observation.projection * alignment_jacobian;
    residual = observation.projection * Difference(predicted, observation.pose);
    noise = 2.0 * (observation.projected_covariance +
                   observation.projection * local_jacobian *
                       PoseCovariance(local) * local_jacobian.transpose() *
                       observation.projection.transpose());
    observation_noise = observation.projected_covariance;
  }
  const Eigen::MatrixXd innovation =
      2.0 * (h * state_.covariance * h.transpose() + noise);
  const double distance = residual.dot(innovation.ldlt().solve(residual));
  if (diagnostics != nullptr) {
    diagnostics->available = true;
    diagnostics->unavailable_reason.clear();
    diagnostics->prior_state_frame = "map_from_odom";
    diagnostics->observed_dimension = static_cast<uint32_t>(residual.size());
    diagnostics->prior_state_dimension = 6;
    diagnostics->residual = residual;
    diagnostics->covariance = innovation;
    diagnostics->nis = distance;
    diagnostics->gate_threshold = config_.innovation_gate;
    diagnostics->covariance_model = config_.diagnostic_fixed_gain
                                        ? "diagnostic_fixed_gain_partial_bound"
                                        : "diagnostic_ci_partial_bound";
    if (observation.kind == GlobalObservation::Kind::HORIZONTAL_POSITION)
      diagnostics->residual_basis = "map.xy_m.v1";
    else if (observation.kind == GlobalObservation::Kind::POSITION)
      diagnostics->residual_basis = "map.xyz_m.v1";
    else if (observation.kind == GlobalObservation::Kind::HEADING)
      diagnostics->residual_basis = "map.baseline_azimuth_rad.v1";
    diagnostics->covariance_decomposition_available = true;
    diagnostics->prior_contribution =
        2.0 * h * state_.covariance * h.transpose();
    diagnostics->observation_contribution = 4.0 * observation_noise;
    diagnostics->local_contribution =
        2.0 * noise - diagnostics->observation_contribution;
    diagnostics->prior_pose.translation = state_.map_to_odom.translation();
    diagnostics->prior_pose.rotation =
        Eigen::Quaterniond(state_.map_to_odom.linear()).normalized();
    diagnostics->prior_covariance = state_.covariance;
  }
  if (!std::isfinite(distance) || distance > config_.innovation_gate) {
    evidence_.erase(observation.source);
    return Reject(Reason::INNOVATION_REJECTED,
                  "partial innovation rejected in observed subspace", source);
  }
  Matrix6d covariance;
  Eigen::Matrix<double, 6, 1> correction;
  if (config_.diagnostic_fixed_gain) {
    Eigen::MatrixXd gain;
    if (observation.kind == GlobalObservation::Kind::POSITION ||
        observation.kind == GlobalObservation::Kind::HORIZONTAL_POSITION) {
      gain = Eigen::MatrixXd::Zero(6, h.rows());
      gain.topRows(h.rows()) =
          0.5 * Eigen::MatrixXd::Identity(h.rows(), h.rows());
    } else if (observation.kind == GlobalObservation::Kind::HEADING) {
      gain = Eigen::MatrixXd::Zero(6, 1);
      gain.bottomRows(3) =
          0.5 * h.rightCols(3).transpose() / h.rightCols(3).squaredNorm();
    } else {
      gain = 0.5 * h.completeOrthogonalDecomposition().pseudoInverse();
    }
    correction = gain * residual;
    covariance = FixedGainCovarianceBound(state_.covariance, noise, h, gain);
  } else {
    // Generalized covariance intersection: unknown shared trajectory
    // correlation. Null directions receive no information and may become less
    // certain.
    Matrix6d prior_information =
        state_.covariance.ldlt().solve(Matrix6d::Identity());
    if (config_.diagnostic_only) {
      // Retain the conditional prior in the unmeasured subspace. This
      // diagnostic conditional model is not a qualified joint bound.
      const Eigen::MatrixXd measured_prior =
          h * state_.covariance * h.transpose();
      prior_information -= 0.5 * h.transpose() * measured_prior.ldlt().solve(h);
    } else {
      prior_information *= 0.5;
    }
    const Eigen::MatrixXd weighted = noise.ldlt().solve(h);
    const Matrix6d information =
        prior_information + 0.5 * h.transpose() * weighted;
    covariance = information.ldlt().solve(Matrix6d::Identity());
    correction = covariance * (0.5 * weighted.transpose()) * residual;
  }
  if (!ValidCovariance(covariance) || !correction.allFinite()) {
    return Reject(Reason::INVALID_INPUT,
                  "partial update is not finite/positive", source);
  }
  state_.map_to_odom.translation() += correction.head<3>();
  state_.map_to_odom.linear() =
      ExpRotation(correction.tail<3>()).toRotationMatrix() *
      state_.map_to_odom.linear();
  state_.covariance = 0.5 * (covariance + covariance.transpose());
  state_.last_observation = observation.stamp.time;
  // Do not refresh the full-pose lease or claim previously missing directions.
  if (observation.georeferenced) {
    state_.last_georeference = observation.stamp.time;
  }
  ++state_.correction_id;
  evidence_[observation.source] = {observation, local};
  ++source->accepted;
  source->reason = Reason::NONE;
  reason_ = Reason::DEGENERATE;
  return {};
}

Result GlobalAlignment::ObserveHeadingAcquisition(
    const GlobalObservation& observation, const LocalState& local,
    SourceCounters* source, ObservationInnovation* diagnostics) {
  if (diagnostics != nullptr) {
    diagnostics->unavailable_reason =
        "heading_acquisition_does_not_compute_a_scalar_innovation";
  }
  if (!observation.measurement_timestamp_ns ||
      observation.conditional_local_error_qualified) {
    return Reject(Reason::CORRELATION_UNQUALIFIED,
                  "partial heading acquisition requires exact time and an "
                  "unconditional source model", source);
  }
  // This is a separate, uncalibrated horizontal result, not a graph prior.
  // Retain original source/local pairs; never invent unmeasured Z or tilt.
  const bool heading_input =
      observation.kind == GlobalObservation::Kind::HEADING;
  const GlobalEvidence current{observation, local};
  const GlobalEvidence* heading = heading_input ? &current : nullptr;
  const GlobalEvidence* position = heading_input ? nullptr : &current;
  for (const auto& entry : evidence_) {
    const auto& candidate = entry.second;
    const auto& input = candidate.observation;
    if (input.conditional_local_error_qualified ||
        !input.measurement_timestamp_ns ||
        std::abs(input.stamp.time - observation.stamp.time) >
            config_.global_timeout) continue;
    if (!heading_input && input.kind == GlobalObservation::Kind::HEADING &&
        (!heading || input.stamp.time > heading->observation.stamp.time))
      heading = &candidate;
    if (heading_input &&
        (input.kind == GlobalObservation::Kind::POSITION ||
         input.kind == GlobalObservation::Kind::HORIZONTAL_POSITION) &&
        (!position || input.stamp.time > position->observation.stamp.time))
      position = &candidate;
  }
  if (heading && position) {
    const auto& h = heading->observation;
    const auto& p = position->observation;
    const Eigen::Vector3d direction =
        heading->local.orientation * h.direction_in_base;
    const double squared = direction.head<2>().squaredNorm();
    if (squared < 1e-8)
      return Reject(Reason::DEGENERATE, "baseline azimuth is undefined", source);
    const double delta = h.heading_rad -
                         std::atan2(direction.y(), direction.x());
    const double yaw = std::atan2(std::sin(delta), std::cos(delta));
    const Eigen::Matrix2d rotation =
        Eigen::Rotation2Dd(yaw).toRotationMatrix();
    HorizontalGlobalState horizontal;
    horizontal.odom_reference_xy =
        (position->local.position +
         position->local.orientation * p.point_in_base).head<2>();
    horizontal.alignment.head<2>() = p.pose.translation().head<2>();
    horizontal.alignment.z() = yaw;
    const Eigen::RowVector3d azimuth =
        Eigen::RowVector3d(-direction.y(), direction.x(), 0.0) / squared;
    const Eigen::RowVector3d heading_jacobian =
        -azimuth * heading->local.orientation.toRotationMatrix() *
        Skew(h.direction_in_base);
    Eigen::Matrix<double, 2, 6> point_jacobian;
    point_jacobian.leftCols<3>() =
        rotation * Eigen::Matrix<double, 2, 3>::Identity();
    point_jacobian.rightCols<3>() =
        -point_jacobian.leftCols<3>() *
        position->local.orientation.toRotationMatrix() * Skew(p.point_in_base);
    // Two correlation bounds: source/local, then position/heading. No claim
    // that two messages from one receiver are independent measurements.
    horizontal.covariance.topLeftCorner<2, 2>() = 4.0 *
        (p.covariance.topLeftCorner<2, 2>() +
         point_jacobian * PoseCovariance(position->local) *
             point_jacobian.transpose());
    horizontal.covariance(2, 2) = 4.0 *
        (h.heading_variance +
         (heading_jacobian * heading->local.covariance.block<3, 3>(6, 6) *
          heading_jacobian.transpose())(0, 0));
    horizontal.last_observation = std::min(h.stamp.time, p.stamp.time);
    horizontal.last_observation_ns =
        std::min(*h.measurement_timestamp_ns, *p.measurement_timestamp_ns);
    horizontal.covariance_time =
        std::max(h.stamp.time, p.stamp.time);
    if (state_.horizontal &&
        observation.stamp.time - state_.horizontal->last_observation <=
            config_.global_timeout) {
      const auto prior = Predict(horizontal.covariance_time).horizontal.value();
      const Eigen::Matrix2d prior_rotation =
          Eigen::Rotation2Dd(prior.alignment.z()).toRotationMatrix();
      const Eigen::Vector2d displacement =
          horizontal.odom_reference_xy - prior.odom_reference_xy;
      Eigen::Vector3d residual;
      residual.head<2>() = horizontal.alignment.head<2>() -
          (prior.alignment.head<2>() + prior_rotation * displacement);
      const double angle = yaw - prior.alignment.z();
      residual.z() = std::atan2(std::sin(angle), std::cos(angle));
      Eigen::Matrix3d reference_jacobian = Eigen::Matrix3d::Identity();
      const Eigen::Vector2d rotated = prior_rotation * displacement;
      reference_jacobian.block<2, 1>(0, 2) =
          Eigen::Vector2d(-rotated.y(), rotated.x());
      const Eigen::Matrix3d innovation = 2.0 *
          (horizontal.covariance + reference_jacobian * prior.covariance *
                                       reference_jacobian.transpose());
      const double distance = residual.dot(innovation.ldlt().solve(residual));
      if (!std::isfinite(distance) || distance > config_.innovation_gate)
        return Reject(Reason::INNOVATION_REJECTED,
                      "position/heading acquisition innovation rejected", source);
    }
    state_.horizontal = horizontal;
    ++state_.correction_id;
  }
  evidence_[observation.source] = current;
  ++source->accepted;
  source->reason = Reason::NONE;
  reason_ = Reason::DEGENERATE;
  return {Reason::NONE, state_.horizontal
                            ? "diagnostic_position_heading_updated"
                            : "retained_position_heading_waiting_for_pair"};
}

Result GlobalAlignment::ObserveHorizontal(const GlobalObservation& observation,
                                         const LocalState& local,
                                         SourceCounters* source,
                                         ObservationInnovation* diagnostics) {
  if (!observation.measurement_timestamp_ns) {
    return Reject(Reason::INVALID_INPUT,
                  "Horizontal acquisition requires original integer time",
                  source);
  }
  if (!horizontal_source_.empty() &&
      horizontal_source_ != observation.source) {
    return Reject(Reason::INVALID_INPUT,
                  "Horizontal acquisition uses one declared position source",
                  source);
  }
  if (state_.horizontal &&
      observation.stamp.time - state_.horizontal->last_observation >
          config_.global_timeout) {
    state_.horizontal.reset();
    horizontal_bootstrap_.reset();
  }
  const Eigen::Vector2d antenna =
      (local.position + local.orientation * observation.point_in_base).head<2>();
  std::string message;
  if (!state_.horizontal) {
    if (!horizontal_bootstrap_) {
      horizontal_bootstrap_ =
          std::make_unique<internal::GnssAlignmentRecovery>();
      horizontal_source_ = observation.source;
    }
    internal::GnssAlignmentRecoveryObservation input;
    input.timestamp_ns = *observation.measurement_timestamp_ns;
    input.odom_antenna_xy = antenna;
    input.global_antenna_xy = observation.pose.translation().head<2>();
    input.covariance_xy = observation.covariance.topLeftCorner<2, 2>();
    internal::GnssAlignmentRecoveryDiagnostics diagnostics;
    internal::GnssAlignmentRecoveryCandidate candidate;
    const auto status =
        horizontal_bootstrap_->AddObservation(input, &diagnostics, &candidate);
    if (!status.ok()) {
      return Reject(Reason::INVALID_INPUT, status.message(), source);
    }
    message = diagnostics.reason;
    if (diagnostics.candidate_validated) {
      const double yaw = candidate.global_from_odom_xy_yaw.z();
      HorizontalGlobalState horizontal;
      horizontal.odom_reference_xy = candidate.odom_reference_xy;
      horizontal.alignment.head<2>() =
          candidate.global_from_odom_xy_yaw.head<2>() +
          Eigen::Rotation2Dd(yaw) * candidate.odom_reference_xy;
      horizontal.alignment.z() = yaw;
      horizontal.covariance =
          candidate.sigma_xy_yaw.array().square().matrix().asDiagonal();
      state_.horizontal = horizontal;
      horizontal_bootstrap_.reset();
      ++state_.correction_id;
    }
  } else {
    if (observation.stamp.time < state_.horizontal->last_observation) {
      return Reject(Reason::TIMESTAMP_REGRESSION,
                    "horizontal observation older than committed frontier",
                    source);
    }
    auto predicted = Predict(observation.stamp.time).horizontal.value();
    const Eigen::Matrix2d rotation =
        Eigen::Rotation2Dd(predicted.alignment.z()).toRotationMatrix();
    const Eigen::Vector2d rotated =
        rotation * (antenna - predicted.odom_reference_xy);
    Eigen::Matrix<double, 2, 3> h;
    h.leftCols<2>().setIdentity();
    h.col(2) = Eigen::Vector2d(-rotated.y(), rotated.x());
    const Eigen::Vector2d residual =
        observation.pose.translation().head<2>() -
        (rotated + predicted.alignment.head<2>());
    Eigen::Matrix<double, 2, 6> local_jacobian;
    local_jacobian.leftCols<3>() = rotation *
        Eigen::Matrix<double, 2, 3>::Identity();
    local_jacobian.rightCols<3>() =
        -local_jacobian.leftCols<3>() *
        local.orientation.toRotationMatrix() * Skew(observation.point_in_base);
    const Eigen::Matrix2d noise =
        2.0 * (observation.covariance.topLeftCorner<2, 2>() +
               local_jacobian * PoseCovariance(local) *
                   local_jacobian.transpose());
    const Eigen::Matrix2d innovation =
        2.0 * (h * predicted.covariance * h.transpose() + noise);
    const double distance = residual.dot(innovation.ldlt().solve(residual));
    if (diagnostics != nullptr) {
      diagnostics->available = true;
      diagnostics->unavailable_reason.clear();
      diagnostics->prior_state_frame = "map_from_odom_xy_yaw";
      diagnostics->observed_dimension = static_cast<uint32_t>(residual.size());
      diagnostics->prior_state_dimension = 3;
      diagnostics->residual = residual;
      diagnostics->covariance = innovation;
      diagnostics->nis = distance;
      diagnostics->gate_threshold = config_.innovation_gate;
      diagnostics->covariance_model = "diagnostic_horizontal_ci_bound";
      diagnostics->residual_basis = "map.xy_m.v1";
      diagnostics->prior_pose.translation.head<2>() =
          predicted.alignment.head<2>();
      diagnostics->prior_pose.rotation =
          Eigen::AngleAxisd(predicted.alignment.z(), Eigen::Vector3d::UnitZ());
      diagnostics->prior_covariance.topLeftCorner<3, 3>() =
          predicted.covariance;
    }
    if (!std::isfinite(distance) || distance > config_.innovation_gate) {
      return Reject(Reason::INNOVATION_REJECTED,
                    "horizontal innovation rejected", source);
    }
    // Diagnostic CI only; preserve the unmeasured conditional prior.
    const Eigen::Matrix2d measured = h * predicted.covariance * h.transpose();
    const Eigen::Matrix3d information =
        predicted.covariance.ldlt().solve(Eigen::Matrix3d::Identity()) -
        0.5 * h.transpose() * measured.ldlt().solve(h) +
        0.5 * h.transpose() * noise.ldlt().solve(h);
    predicted.covariance =
        information.ldlt().solve(Eigen::Matrix3d::Identity());
    predicted.alignment +=
        predicted.covariance * (0.5 * h.transpose() * noise.ldlt().solve(residual));
    predicted.alignment.z() = std::atan2(std::sin(predicted.alignment.z()),
                                        std::cos(predicted.alignment.z()));
    predicted.covariance =
        (0.5 * (predicted.covariance + predicted.covariance.transpose())).eval();
    if (!predicted.alignment.allFinite() ||
        !predicted.covariance.allFinite() ||
        predicted.covariance.llt().info() != Eigen::Success) {
      return Reject(Reason::INVALID_INPUT,
                    "horizontal update is not finite positive definite",
                    source);
    }
    state_.horizontal = predicted;
    ++state_.correction_id;
    message = "diagnostic_horizontal_updated";
  }
  if (state_.horizontal) {
    state_.horizontal->last_observation = observation.stamp.time;
    state_.horizontal->last_observation_ns =
        *observation.measurement_timestamp_ns;
    state_.horizontal->covariance_time = observation.stamp.time;
  }
  evidence_[observation.source] = {observation, local};
  ++source->accepted;
  source->reason = Reason::NONE;
  reason_ = Reason::DEGENERATE;
  return {Reason::NONE, message};
}

void GlobalAlignment::Invalidate(Reason reason) {
  graph_.reset();
  state_.valid = false;
  state_.verifying = false;
  candidate_frames_ = 0;
  evidence_.clear();
  state_.horizontal.reset();
  horizontal_bootstrap_.reset();
  horizontal_source_.clear();
  reason_ = reason;
}

Result GlobalAlignment::Evaluate(double now) const {
  if (!std::isfinite(now) || now <= 0.0) {
    return {Reason::CLOCK_INVALID, "invalid evaluation clock"};
  }
  if (history_.empty()) {
    return {Reason::HISTORY_UNAVAILABLE, "local history empty"};
  }
  if (config_.diagnostic_only || config_.graph_allow_uncalibrated_inputs) {
    return {Reason::INVALID_INPUT,
            "diagnostic covariance/prior cannot qualify trusted output"};
  }
  if (now - history_.back().stamp.time > config_.local_timeout ||
      history_.back().stamp.time - now > config_.future_tolerance) {
    return {Reason::IMU_STALE, "local snapshot stale"};
  }
  if (!state_.valid) {
    return {reason_, "global alignment unavailable"};
  }
  if (now - state_.last_full_observation > config_.global_timeout ||
      state_.last_full_observation - now > config_.future_tolerance) {
    return {Reason::GLOBAL_STALE, "no fresh global evidence"};
  }
  if (!ValidCovariance(state_.covariance)) {
    return {Reason::INVALID_INPUT, "alignment covariance invalid"};
  }
  const Matrix6d combined = GlobalPoseCovarianceBound(
      Predict(history_.back().stamp.time), history_.back());
  if (combined.topLeftCorner<3, 3>().diagonal().maxCoeff() >
          config_.max_position_std * config_.max_position_std ||
      combined.bottomRightCorner<3, 3>().diagonal().maxCoeff() >
          config_.max_attitude_std * config_.max_attitude_std) {
    return {Reason::COVARIANCE_EXCEEDED, "global uncertainty budget exceeded"};
  }
  return {};
}

GlobalState GlobalAlignment::Predict(double time) const {
  GlobalState predicted = state_;
  if (predicted.horizontal && std::isfinite(time) &&
      time > predicted.horizontal->covariance_time) {
    auto& horizontal = *predicted.horizontal;
    const double elapsed = time - horizontal.covariance_time;
    horizontal.covariance.diagonal().head<2>().array() +=
        elapsed * config_.position_drift_variance_rate;
    horizontal.covariance(2, 2) +=
        elapsed * config_.attitude_drift_variance_rate;
    horizontal.covariance_time = time;
  }
  if (predicted.valid && std::isfinite(time) && time > state_.covariance_time) {
    const double elapsed = time - state_.covariance_time;
    Matrix6d reference_noise = Matrix6d::Zero();
    reference_noise.topLeftCorner<3, 3>().diagonal().setConstant(
        config_.position_drift_variance_rate);
    reference_noise.bottomRightCorner<3, 3>().diagonal().setConstant(
        config_.attitude_drift_variance_rate);
    Matrix6d reference_jacobian = Matrix6d::Identity();
    reference_jacobian.topRightCorner<3, 3>() =
        -Skew(state_.map_to_odom.linear() * process_reference_in_odom_);
    const Matrix6d inverse_reference_jacobian = reference_jacobian.inverse();
    predicted.covariance += elapsed * inverse_reference_jacobian *
                            reference_noise *
                            inverse_reference_jacobian.transpose();
    predicted.covariance_time = time;
  }
  return predicted;
}

}  // namespace unified
}  // namespace localization
}  // namespace openloc
