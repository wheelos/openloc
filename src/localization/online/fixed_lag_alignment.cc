// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "openloc/localization/global/fixed_lag_alignment.h"

#include <deque>
#include <map>
#include <utility>

#include "candidates/global/global_alignment.h"
#include "online/global_alignment_bridge.h"
#include "openloc/localization/online/validation.h"

namespace openloc {
namespace localization {
namespace {
namespace u = unified;
using alignment_bridge::ClockId;
using alignment_bridge::Convert;
using alignment_bridge::Seconds;

Status Unconfigured() {
  return Status(StatusCode::kStateUnavailable,
                "fixed-lag alignment not configured");
}
}  // namespace

class FixedLagGlobalAlignment::Impl {
 public:
  Status Configure(const FixedLagAlignmentConfig& config) {
    if (owner_) {
      return Status(StatusCode::kInvalidArgument,
                    "fixed-lag alignment configuration is immutable");
    }
    for (const auto& frame :
         {config.map_frame, config.odom_frame, config.base_frame}) {
      const auto status = ValidateFrameId(frame);
      if (!status.ok()) return status;
    }
    const auto reference = ValidateGeodeticReference(config.reference.geodetic);
    if (!reference.ok()) return reference;
    if ((config.reference.geodetic.state != GeodeticReferenceState::kVerified &&
         !(config.allow_uncalibrated_inputs &&
           config.reference.geodetic.state == GeodeticReferenceState::kDiagnosticOnly)) ||
        config.reference.id.empty() || config.reference.version.empty() ||
        config.reference.calibration_id.empty() ||
        config.map_content_id.empty() || config.motion_error_model_id.empty() ||
        !config.motion_error_model_qualified ||
        (config.enable_heading_bootstrap &&
         !config.assume_gravity_aligned_odom) ||
        config.map_frame.value == config.odom_frame.value ||
        config.map_frame.value == config.base_frame.value ||
        config.odom_frame.value == config.base_frame.value ||
        (config.clock_domain != ClockDomain::kUnixEpoch &&
         config.clock_domain != ClockDomain::kSensorMonotonic) ||
        config.history_duration_ns <= 0 ||
        config.maximum_observation_age_ns <= 0 ||
        config.maximum_odometry_age_ns <= 0 ||
        config.maximum_full_pose_age_ns <= 0 || config.maximum_decisions == 0) {
      return Status(StatusCode::kInvalidArgument,
                    "graph requires explicit identities, verified reference, "
                    "qualified motion error model and positive budgets");
    }
    u::GlobalConfig policy;
    policy.clock_id = ClockId(config.clock_domain);
    policy.map_id = config.reference.id;
    policy.map_version = config.reference.version;
    policy.calibration_id = config.reference.calibration_id;
    policy.map_content_id = config.map_content_id;
    policy.map_frame = config.map_frame.value;
    policy.base_frame = config.base_frame.value;
    policy.history_duration = Seconds(config.history_duration_ns);
    policy.max_observation_age = Seconds(config.maximum_observation_age_ns);
    policy.local_timeout = Seconds(config.maximum_odometry_age_ns);
    policy.global_timeout = Seconds(config.maximum_full_pose_age_ns);
    policy.enable_fixed_lag_graph = true;
    policy.motion_error_model_qualified = config.motion_error_model_qualified;
    policy.graph_allow_uncalibrated_inputs = config.allow_uncalibrated_inputs;
    policy.graph_retain_on_full_pose_staleness =
        config.retain_graph_on_full_pose_staleness;
    policy.graph_solve_budget_ms = config.solve_budget_ms;
    policy.graph_max_states = config.maximum_states;
    policy.graph_max_observations = config.maximum_observations;
    policy.graph_max_error_dimension = config.maximum_error_dimension;
    policy.verification_frames = config.verification_frames;
    policy.enable_heading_bootstrap = config.enable_heading_bootstrap;
    auto owner = std::make_unique<u::GlobalAlignment>(policy);
    const auto status = Convert(owner->ValidateConfig());
    if (!status.ok()) return status;
    config_ = config;
    owner_ = std::move(owner);
    return Status::Ok();
  }

  Status AddOdometry(const GraphOdometrySample& input,
                     const GraphMotionConnection* connection) {
    if (!owner_) return Unconfigured();
    auto status = ValidateOdometrySample(input.odometry);
    if (status.ok()) status = ValidateTimestamp(input.receive_time);
    if (!status.ok()) return status;
    const auto& odometry = input.odometry;
    if (odometry.timestamp.clock_domain != config_.clock_domain ||
        input.receive_time.clock_domain != config_.clock_domain) {
      return Status(StatusCode::kClockDomainMismatch,
                    "graph ODOM clock differs from configuration");
    }
    if (odometry.parent_frame.value != config_.odom_frame.value ||
        odometry.child_frame.value != config_.base_frame.value) {
      return Status(StatusCode::kFrameMismatch,
                    "graph ODOM frames differ from configuration");
    }
    status = ValidateCovariance(input.covariance, 15);
    if (!status.ok()) {
      owner_->Invalidate(u::Reason::INVALID_INPUT);
      return status;
    }
    if (!input.covariance_model_valid ||
        (!input.uncertainty_qualified && !config_.allow_uncalibrated_inputs)) {
      owner_->Invalidate(u::Reason::UNCERTAINTY_UNQUALIFIED);
      return Status(StatusCode::kRejectedMeasurement,
                    "local error model or uncertainty is unqualified");
    }
    u::LocalState local;
    local.stamp = {Seconds(odometry.timestamp.nanoseconds),
                   Seconds(input.receive_time.nanoseconds), input.sequence,
                   ClockId(config_.clock_domain)};
    local.epoch = {odometry.epoch.producer_session, odometry.epoch.generation};
    local.position = odometry.pose.translation;
    local.orientation = odometry.pose.rotation;
    local.covariance = input.covariance;
    local.covariance_model_valid = input.covariance_model_valid;
    local.uncertainty_qualified = input.uncertainty_qualified;
    local.joint_error = input.joint_error;
    local.valid = true;
    if (local.joint_error &&
        local.joint_error->covariance.rows() > config_.maximum_error_dimension) {
      owner_->Invalidate(u::Reason::INVALID_INPUT);
      return Status(StatusCode::kInvalidArgument,
                    "joint error dimension exceeds configured graph capacity");
    }
    const auto* previous = owner_->latest_local();
    if (previous != nullptr &&
        (local.stamp.time <= previous->stamp.time ||
         (local.epoch == previous->epoch &&
          local.stamp.sequence <= previous->stamp.sequence))) {
      return Convert(owner_->AddLocal(local));
    }
    u::MotionIncrement motion;
    if (connection != nullptr) {
      if (previous == nullptr || previous->epoch != local.epoch) {
        return Status(StatusCode::kEpochMismatch,
                      "motion requires a previous snapshot in the same epoch");
      }
      if (!latest_ || connection->epoch != odometry.epoch ||
          connection->start_time.clock_domain != config_.clock_domain ||
          connection->end_time.clock_domain != config_.clock_domain ||
          connection->start_time.nanoseconds !=
              latest_->timestamp.nanoseconds ||
          connection->end_time.nanoseconds != odometry.timestamp.nanoseconds ||
          connection->start_sequence != previous->stamp.sequence ||
          connection->end_sequence != local.stamp.sequence) {
        return RejectMotion(
            odometry, local,
            {u::Reason::HISTORY_UNAVAILABLE,
             "motion endpoints do not match accepted ODOM snapshots"});
      }
      std::vector<u::SourceSampleId> sources;
      for (const auto& source : connection->sources) {
        sources.push_back({source.source, source.sequence});
      }
      const auto computed = u::ComputeMotionIncrement(
          *previous, local, connection->cross_covariance, sources, &motion);
      if (!computed.ok()) {
        return RejectMotion(odometry, local, computed);
      }
      motion.joint_cross_covariance = connection->joint_cross_covariance;
    }
    status = Convert(owner_->AddLocal(local, connection ? &motion : nullptr));
    RetainOdometry(odometry, local);
    return status;
  }

  Status RejectMotion(const OdometrySample& odometry,
                      const u::LocalState& local, const u::Result& failure) {
    owner_->Invalidate(failure.reason);
    const auto retained = owner_->AddLocal(local);
    RetainOdometry(odometry, local);
    if (!retained.ok() && retained.reason != u::Reason::HISTORY_UNAVAILABLE) {
      return Convert(retained);
    }
    return Convert(failure);
  }

  void RetainOdometry(const OdometrySample& odometry,
                      const u::LocalState& local) {
    const auto* retained = owner_->latest_local();
    // Missing motion can reject fusion while retaining the new local epoch.
    if (retained != nullptr && retained->epoch == local.epoch &&
        retained->stamp.time == local.stamp.time &&
        retained->stamp.sequence == local.stamp.sequence) {
      if (!latest_ || latest_->epoch != odometry.epoch) {
        exact_times_.clear();
        last_full_pose_time_.reset();
      }
      latest_ = odometry;
      exact_times_[local.stamp.time] = odometry.timestamp.nanoseconds;
      while (!exact_times_.empty() &&
             (exact_times_.size() > 5000 ||
              local.stamp.time - exact_times_.begin()->first >
                  Seconds(config_.history_duration_ns))) {
        exact_times_.erase(exact_times_.begin());
      }
    }
  }

  Status Submit(const GlobalObservationContext& context,
                const GraphSourceQualification& qualification,
                u::GlobalObservation observation,
                const Status& adapter_status = Status::Ok()) {
    if (!owner_) return Unconfigured();
    const uint64_t before = owner_->state().correction_id;
    Status status = adapter_status;
    ObservationInnovation innovation;
    innovation.unavailable_reason = "rejected_before_innovation";
    if (status.ok()) {
      if (sources_.size() >= 32 && sources_.count(context.source) == 0) {
        status = Status(StatusCode::kRejectedMeasurement,
                        "graph source statistics capacity exceeded");
      } else {
        status = Apply(context, qualification, observation, &innovation);
      }
    }
    if (!innovation.available && innovation.unavailable_reason.empty()) {
      innovation.unavailable_reason = status.message();
    }
    const bool updated = status.ok() && owner_->state().correction_id != before;
    if (updated && observation.kind == u::GlobalObservation::Kind::POSE &&
        owner_->state().last_full_observation == observation.stamp.time) {
      last_full_pose_time_ = std::make_pair(
          observation.stamp.time, context.measurement_time.nanoseconds);
    }
    GlobalObservationDecision decision{
        context,      status,
        ++submitted_, owner_->state().correction_id,
        updated,      observation.covariance_source, std::move(innovation)};
    decisions_.push_back({std::move(decision), qualification});
    while (decisions_.size() > config_.maximum_decisions) {
      decisions_.pop_front();
    }
    if (!status.ok()) ++rejected_;
    if (sources_.count(context.source) != 0 || sources_.size() < 32) {
      auto& source = sources_[context.source];
      source.source = context.source;
      ++source.submitted;
      if (!status.ok()) {
        ++source.rejected;
      } else if (updated) {
        ++source.alignment_updates;
      } else {
        ++source.retained;
      }
    }
    return status;
  }

  Status Apply(const GlobalObservationContext& context,
               const GraphSourceQualification& qualification,
               u::GlobalObservation observation,
               ObservationInnovation* innovation) {
    if (!owner_) return Unconfigured();
    auto status = ValidateTimestamp(context.measurement_time);
    if (status.ok()) status = ValidateTimestamp(context.receive_time);
    if (!status.ok()) return status;
    if (context.measurement_time.clock_domain != config_.clock_domain ||
        context.receive_time.clock_domain != config_.clock_domain) {
      return Status(StatusCode::kClockDomainMismatch,
                    "graph observation clocks differ from configuration");
    }
    if (context.reference != config_.reference ||
        context.map_content_id != config_.map_content_id ||
        context.map_frame.value != config_.map_frame.value ||
        context.base_frame.value != config_.base_frame.value) {
      return Status(StatusCode::kMapReferenceMismatch,
                    "graph observation reference differs from configuration");
    }
    const bool bounded_horizontal =
        config_.enable_heading_bootstrap && !owner_->state().valid &&
        (observation.kind == u::GlobalObservation::Kind::HEADING ||
         observation.kind == u::GlobalObservation::Kind::POSITION ||
         observation.kind == u::GlobalObservation::Kind::HORIZONTAL_POSITION) &&
        !qualification.local_correlation;
    if (!alignment_bridge::ValidProvenance(context.provenance) ||
        observation.covariance_source.empty() ||
        qualification.uncertainty_model_id.empty() ||
        (!bounded_horizontal && !qualification.local_correlation &&
         qualification.independence_evidence_id.empty()) ||
        !qualification.quality_valid ||
        (!qualification.uncertainty_qualified &&
         !config_.allow_uncalibrated_inputs) ||
        (!bounded_horizontal && !qualification.local_correlation &&
         (!qualification.independent_of_local ||
          !qualification.independent_of_graph_sources))) {
      return Status(StatusCode::kRejectedMeasurement,
                    "graph source quality, uncertainty or independence "
                    "qualification is unavailable");
    }
    if (qualification.local_correlation) {
      const auto& model = *qualification.local_correlation;
      const auto dimension = observation.kind == u::GlobalObservation::Kind::POSE ? 6
          : observation.kind == u::GlobalObservation::Kind::POSITION ? 3
          : observation.kind == u::GlobalObservation::Kind::HORIZONTAL_POSITION ? 2
          : observation.kind == u::GlobalObservation::Kind::HEADING ? 1
          : observation.projection.rows();
      if (qualification.independent_of_local ||
          qualification.independent_of_graph_sources ||
          model.motion_error_model_id != config_.motion_error_model_id ||
          model.conditional_independence_evidence_id.empty() ||
          !model.conditional_model_qualified ||
          !model.residual_independent_of_history_and_sources ||
          model.odometry_sequence == 0 ||
          model.cross_covariance.rows() < 15 ||
          model.cross_covariance.rows() > config_.maximum_error_dimension ||
          model.cross_covariance.cols() != dimension ||
          !model.cross_covariance.allFinite()) {
        return Status(StatusCode::kRejectedMeasurement,
                      "conditional source model or sufficient-state qualification is unavailable");
      }
      observation.local_error_cross_covariance = model.cross_covariance;
      observation.local_error_sequence = model.odometry_sequence;
      if (model.conditioning_time) {
        const auto valid = ValidateTimestamp(*model.conditioning_time);
        if (!valid.ok()) return valid;
        if (model.conditioning_time->clock_domain != config_.clock_domain)
          return Status(StatusCode::kClockDomainMismatch,
                        "conditional error clock differs from configuration");
        const double time = Seconds(model.conditioning_time->nanoseconds);
        const auto exact = exact_times_.find(time);
        if (exact == exact_times_.end() ||
            exact->second != model.conditioning_time->nanoseconds)
          return Status(StatusCode::kOutsideHistory,
                        "conditional error has no matching integer-time ODOM node");
        observation.local_error_time = time;
      }
      observation.conditional_local_error_qualified = true;
    }
    for (const auto dependency : {context.provenance.odometry_initialization,
                                  context.provenance.odometry_deskew,
                                  context.provenance.shared_local_scans,
                                  context.provenance.gnss_map_construction}) {
      if (dependency == ObservationDependency::kPresent &&
          !qualification.local_correlation && !bounded_horizontal) {
        return Status(StatusCode::kRejectedMeasurement,
                      "known shared dependencies have no joint graph factor");
      }
    }
    const auto exact = exact_times_.find(observation.stamp.time);
    if (exact == exact_times_.end() ||
        exact->second != context.measurement_time.nanoseconds) {
      return Status(StatusCode::kOutsideHistory,
                    "measurement has no matching integer-time ODOM node");
    }
    observation.quality_valid = qualification.quality_valid;
    observation.uncertainty_qualified = qualification.uncertainty_qualified;
    observation.diagnostic_only = config_.allow_uncalibrated_inputs;
    observation.independent_of_local = qualification.independent_of_local;
    return Convert(owner_->Observe(observation, innovation));
  }

  Status GetResult(const Timestamp& evaluation_time,
                   FixedLagAlignmentResult* result) const {
    if (!owner_) return Unconfigured();
    if (result == nullptr) {
      return Status(StatusCode::kInvalidArgument, "null graph result");
    }
    auto status = ValidateTimestamp(evaluation_time);
    if (!status.ok()) return status;
    if (evaluation_time.clock_domain != config_.clock_domain) {
      return Status(StatusCode::kClockDomainMismatch,
                    "graph evaluation clock differs from configuration");
    }
    if (!latest_) {
      return Status(StatusCode::kStateUnavailable, "graph has no ODOM sample");
    }
    if (evaluation_time.nanoseconds < latest_->timestamp.nanoseconds) {
      return Status(StatusCode::kOutOfOrderTimestamp,
                    "graph evaluation precedes the latest ODOM sample");
    }
    FixedLagAlignmentResult output;
    output.timestamp = latest_->timestamp;
    output.evaluation_time = evaluation_time;
    output.epoch = latest_->epoch;
    output.reference = config_.reference;
    output.map_content_id = config_.map_content_id;
    output.map_frame = config_.map_frame;
    output.odom_frame = config_.odom_frame;
    output.base_frame = config_.base_frame;
    output.motion_error_model_id = config_.motion_error_model_id;
    output.diagnostic_only = config_.allow_uncalibrated_inputs;
    output.assessment =
        Convert(owner_->Evaluate(Seconds(evaluation_time.nanoseconds)));
    const auto& state = owner_->state();
    output.verification_pending = state.verifying;
    output.odometry_fresh =
        evaluation_time.nanoseconds - latest_->timestamp.nanoseconds <=
        config_.maximum_odometry_age_ns;
    output.full_pose_evidence_fresh =
        last_full_pose_time_ &&
        state.last_full_observation == last_full_pose_time_->first &&
        evaluation_time.nanoseconds >= last_full_pose_time_->second &&
        evaluation_time.nanoseconds - last_full_pose_time_->second <=
            config_.maximum_full_pose_age_ns;
    output.last_full_pose_time_seconds = state.last_full_observation;
    output.decisions_dropped = submitted_ - decisions_.size();
    output.submitted_observations = submitted_;
    output.rejected_observations = rejected_;
    for (const auto& entry : sources_) output.sources.push_back(entry.second);
    const auto predicted = owner_->Predict(Seconds(latest_->timestamp.nanoseconds));
    if (predicted.horizontal) {
      const auto& horizontal = *predicted.horizontal;
      const double yaw = horizontal.alignment.z();
      const Eigen::Matrix2d rotation =
          Eigen::Rotation2Dd(yaw).toRotationMatrix();
      GlobalHorizontalEstimate estimate;
      estimate.map_from_odom_xy_yaw.head<2>() =
          horizontal.alignment.head<2>() -
          rotation * horizontal.odom_reference_xy;
      estimate.map_from_odom_xy_yaw.z() = yaw;
      estimate.map_base_xy =
          rotation * (latest_->pose.translation.head<2>() -
                      horizontal.odom_reference_xy) +
          horizontal.alignment.head<2>();
      estimate.odom_reference_xy = horizontal.odom_reference_xy;
      estimate.covariance = horizontal.covariance;
      estimate.last_observation_time =
          {horizontal.last_observation_ns, config_.clock_domain};
      estimate.correction_id = state.correction_id;
      output.horizontal_estimate = estimate;
      if (!state.valid) output.diagnostic_only = true;
      output.horizontal_evidence_fresh =
          evaluation_time.nanoseconds >= horizontal.last_observation_ns &&
          evaluation_time.nanoseconds - horizontal.last_observation_ns <=
              config_.maximum_full_pose_age_ns;
    }
    if (state.valid) {
      FixedLagAlignmentEstimate estimate;
      estimate.map_from_odom = alignment_bridge::ValuePose(state.map_to_odom);
      estimate.map_from_base = alignment_bridge::ValuePose(
          state.map_to_odom * alignment_bridge::Transform(latest_->pose));
      estimate.alignment_covariance = state.covariance;
      estimate.correction_id = state.correction_id;
      output.estimate = std::move(estimate);
    }
    const bool fresh_estimate =
        output.odometry_fresh &&
        ((output.estimate && output.full_pose_evidence_fresh) ||
         (output.horizontal_estimate && output.horizontal_evidence_fresh));
    if (fresh_estimate && !output.verification_pending) {
      output.trust_level =
          output.diagnostic_only || !output.assessment.ok()
              ? LocalizationTrustLevel::kDiagnostic
              : LocalizationTrustLevel::kQualified;
    }
    *result = std::move(output);
    return Status::Ok();
  }

  Status GetDecisions(std::vector<GraphObservationDecision>* decisions) const {
    if (!owner_) return Unconfigured();
    if (decisions == nullptr) {
      return Status(StatusCode::kInvalidArgument, "null graph decisions");
    }
    decisions->assign(decisions_.begin(), decisions_.end());
    return Status::Ok();
  }

  FixedLagAlignmentConfig config_;
  std::unique_ptr<u::GlobalAlignment> owner_;
  std::optional<OdometrySample> latest_;
  std::optional<std::pair<double, int64_t>> last_full_pose_time_;
  std::map<double, int64_t> exact_times_;
  std::deque<GraphObservationDecision> decisions_;
  std::map<std::string, GraphSourceStatistics> sources_;
  uint64_t submitted_ = 0;
  uint64_t rejected_ = 0;
};

FixedLagGlobalAlignment::FixedLagGlobalAlignment()
    : impl_(std::make_unique<Impl>()) {}
FixedLagGlobalAlignment::~FixedLagGlobalAlignment() = default;

Status FixedLagGlobalAlignment::Configure(
    const FixedLagAlignmentConfig& config) {
  return impl_->Configure(config);
}

Status FixedLagGlobalAlignment::AddOdometry(
    const GraphOdometrySample& sample,
    const GraphMotionConnection* connection) {
  return impl_->AddOdometry(sample, connection);
}

Status FixedLagGlobalAlignment::SubmitMapObservation(
    const GlobalMapObservation& input,
    const GraphSourceQualification& qualification) {
  auto observation = alignment_bridge::Observation(input.context);
  observation.covariance_source = input.covariance_source;
  const auto status = ValidatePose(input.map_from_base);
  if (!status.ok()) {
    return impl_->Submit(input.context, qualification, std::move(observation),
                         status);
  }
  switch (input.kind) {
    case GlobalMapObservation::Kind::kFullPose:
      observation.kind = u::GlobalObservation::Kind::POSE;
      break;
    case GlobalMapObservation::Kind::kProjectedPose:
      observation.kind = u::GlobalObservation::Kind::PROJECTED_POSE;
      break;
    default:
      return impl_->Submit(input.context, qualification, std::move(observation),
                           Status(StatusCode::kInvalidArgument,
                                  "unknown graph map factor kind"));
  }
  observation.pose = alignment_bridge::Transform(input.map_from_base);
  observation.covariance = input.covariance;
  observation.projection = input.projection;
  observation.projected_covariance = input.projected_covariance;
  observation.ambiguous = input.ambiguous;
  observation.recovery = input.recovery;
  return impl_->Submit(input.context, qualification, std::move(observation));
}

Status FixedLagGlobalAlignment::SubmitHorizontalObservation(
    const GlobalHorizontalObservation& input,
    const GraphSourceQualification& qualification) {
  auto observation = alignment_bridge::Observation(input.context);
  observation.kind = u::GlobalObservation::Kind::HORIZONTAL_POSITION;
  observation.pose.translation().head<2>() = input.position;
  observation.covariance.topLeftCorner<2, 2>() = input.covariance;
  observation.point_in_base = input.point_in_base;
  observation.covariance_source = input.covariance_source;
  return impl_->Submit(input.context, qualification, std::move(observation));
}

Status FixedLagGlobalAlignment::SubmitPositionObservation(
    const GlobalPositionObservation& input,
    const GraphSourceQualification& qualification) {
  auto observation = alignment_bridge::Observation(input.context);
  observation.kind = u::GlobalObservation::Kind::POSITION;
  observation.pose.translation() = input.position;
  observation.covariance.topLeftCorner<3, 3>() = input.covariance;
  observation.point_in_base = input.point_in_base;
  observation.covariance_source = input.covariance_source;
  return impl_->Submit(input.context, qualification, std::move(observation));
}

Status FixedLagGlobalAlignment::SubmitHeadingObservation(
    const GlobalHeadingObservation& input,
    const GraphSourceQualification& qualification) {
  auto observation = alignment_bridge::Observation(input.context);
  observation.kind = u::GlobalObservation::Kind::HEADING;
  observation.heading_rad = input.heading_rad;
  observation.heading_variance = input.variance_rad2;
  observation.direction_in_base = input.direction_in_base;
  observation.covariance_source = input.covariance_source;
  return impl_->Submit(input.context, qualification, std::move(observation));
}

Status FixedLagGlobalAlignment::GetResult(
    const Timestamp& evaluation_time, FixedLagAlignmentResult* result) const {
  return impl_->GetResult(evaluation_time, result);
}

Status FixedLagGlobalAlignment::GetDecisions(
    std::vector<GraphObservationDecision>* decisions) const {
  return impl_->GetDecisions(decisions);
}

}  // namespace localization
}  // namespace openloc
