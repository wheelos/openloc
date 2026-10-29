// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "openloc/localization/global/diagnostic_alignment.h"

#include <algorithm>
#include <deque>
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
using alignment_bridge::Transform;
using alignment_bridge::ValidProvenance;
using alignment_bridge::ValuePose;

u::GlobalObservation Observation(const GlobalObservationContext& context) {
  auto observation = alignment_bridge::Observation(context);
  observation.quality_valid = true;
  observation.diagnostic_only = true;
  return observation;
}
}  // namespace

class DiagnosticGlobalAlignment::Impl {
 public:
  struct Pending {
    GlobalObservationContext context;
    u::GlobalObservation observation;
  };

  Status Configure(const DiagnosticAlignmentConfig& config) {
    if (owner_) {
      return Status(StatusCode::kInvalidArgument,
                    "diagnostic alignment configuration is immutable");
    }
    for (const auto& frame :
         {config.map_frame, config.odom_frame, config.base_frame}) {
      const Status status = ValidateFrameId(frame);
      if (!status.ok()) return status;
    }
    const Status reference_status =
        ValidateGeodeticReference(config.reference.geodetic);
    if (!reference_status.ok()) return reference_status;
    if (config.reference.id.empty() || config.reference.version.empty() ||
        ((config.enable_horizontal_bootstrap || config.enable_heading_bootstrap) &&
         !config.assume_gravity_aligned_odom) ||
        config.reference.calibration_id.empty() ||
        config.reference.geodetic.state !=
            GeodeticReferenceState::kDiagnosticOnly ||
        config.map_content_id.empty() ||
        config.map_frame.value == config.odom_frame.value ||
        config.map_frame.value == config.base_frame.value ||
        config.odom_frame.value == config.base_frame.value ||
        (config.clock_domain != ClockDomain::kUnixEpoch &&
         config.clock_domain != ClockDomain::kSensorMonotonic) ||
        config.history_duration_ns <= 0 ||
        config.maximum_interpolation_gap_ns <= 0 ||
        config.maximum_observation_age_ns <= 0 ||
        config.maximum_odometry_age_ns <= 0 ||
        config.maximum_full_pose_age_ns <= 0 ||
        config.maximum_pending_observations == 0 ||
        config.maximum_decisions == 0) {
      return Status(StatusCode::kInvalidArgument,
                    "diagnostic alignment requires explicit identities and "
                    "positive budgets");
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
    policy.max_interpolation_gap = Seconds(config.maximum_interpolation_gap_ns);
    policy.max_observation_age = Seconds(config.maximum_observation_age_ns);
    policy.local_timeout = Seconds(config.maximum_odometry_age_ns);
    policy.global_timeout = Seconds(config.maximum_full_pose_age_ns);
    policy.diagnostic_only = true;
    policy.diagnostic_fixed_gain = config.use_fixed_gain_update;
    policy.enable_diagnostic_horizontal_bootstrap =
        config.enable_horizontal_bootstrap;
    policy.enable_heading_bootstrap = config.enable_heading_bootstrap;
    auto owner = std::make_unique<u::GlobalAlignment>(policy);
    const Status status = Convert(owner->ValidateConfig());
    if (!status.ok()) return status;
    config_ = config;
    owner_ = std::move(owner);
    return Status::Ok();
  }

  Status SubmitMap(const GlobalMapObservation& input) {
    if (!owner_) return Unconfigured();
    const Status pose_status = ValidatePose(input.map_from_base);
    if (!pose_status.ok())
      return Reject(input.context, pose_status, input.covariance_source);
    auto observation = Observation(input.context);
    switch (input.kind) {
      case GlobalMapObservation::Kind::kFullPose:
        observation.kind = u::GlobalObservation::Kind::POSE;
        break;
      case GlobalMapObservation::Kind::kProjectedPose:
        observation.kind = u::GlobalObservation::Kind::PROJECTED_POSE;
        break;
      default:
        return Reject(input.context,
                      Status(StatusCode::kInvalidArgument,
                             "unknown map observation kind"),
                      input.covariance_source);
    }
    observation.pose = Transform(input.map_from_base);
    observation.covariance = input.covariance;
    observation.projection = input.projection;
    observation.projected_covariance = input.projected_covariance;
    observation.covariance_source = input.covariance_source;
    observation.ambiguous = input.ambiguous;
    observation.recovery = input.recovery;
    return Submit(input.context, std::move(observation));
  }

  Status AddOdometry(const OdometrySample& sample,
                     const u::Matrix6d& covariance) {
    if (!owner_) return Unconfigured();
    Status status = ValidateOdometrySample(sample);
    if (!status.ok()) return status;
    if (sample.timestamp.clock_domain != config_.clock_domain) {
      return Status(StatusCode::kClockDomainMismatch,
                    "ODOM clock differs from configuration");
    }
    if (sample.parent_frame.value != config_.odom_frame.value ||
        sample.child_frame.value != config_.base_frame.value) {
      return Status(StatusCode::kFrameMismatch,
                    "ODOM frame differs from configuration");
    }
    if (!u::ValidCovariance(covariance)) {
      return Status(StatusCode::kInvalidArgument,
                    "invalid ODOM pose or diagnostic covariance");
    }
    u::LocalState local;
    local.stamp = {Seconds(sample.timestamp.nanoseconds),
                   Seconds(sample.timestamp.nanoseconds), ++local_sequence_,
                   ClockId(config_.clock_domain)};
    local.epoch = {sample.epoch.producer_session, sample.epoch.generation};
    local.position = sample.pose.translation;
    local.orientation = sample.pose.rotation;
    local.covariance.topLeftCorner<3, 3>() = covariance.topLeftCorner<3, 3>();
    local.covariance.block<3, 3>(0, 6) = covariance.topRightCorner<3, 3>();
    local.covariance.block<3, 3>(6, 0) = covariance.bottomLeftCorner<3, 3>();
    local.covariance.block<3, 3>(6, 6) = covariance.bottomRightCorner<3, 3>();
    local.covariance_model_valid = false;
    local.valid = true;
    status = Convert(owner_->AddLocal(local));
    if (!status.ok()) return status;
    if (has_odometry_ && sample.epoch != latest_.epoch) {
      last_full_pose_time_.reset();
    }
    latest_ = sample;
    has_odometry_ = true;
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (it->context.epoch != sample.epoch) {
        Record(it->context,
               Status(StatusCode::kEpochMismatch,
                      "pending observation belongs to another ODOM epoch"),
               false, it->observation.covariance_source);
        it = pending_.erase(it);
      } else if (it->context.measurement_time.nanoseconds <=
                 sample.timestamp.nanoseconds) {
        Apply(it->context, it->observation);
        it = pending_.erase(it);
      } else {
        ++it;
      }
    }
    return Status::Ok();
  }

  Status Submit(const GlobalObservationContext& context,
                u::GlobalObservation observation) {
    if (!owner_) return Unconfigured();
    const auto reject = [&](const Status& status) {
      return Reject(context, status, observation.covariance_source);
    };
    Status status = ValidateTimestamp(context.measurement_time);
    if (status.ok()) status = ValidateTimestamp(context.receive_time);
    if (!status.ok()) return reject(status);
    if (context.measurement_time.clock_domain != config_.clock_domain ||
        context.receive_time.clock_domain != config_.clock_domain) {
      return reject(Status(StatusCode::kClockDomainMismatch,
                           "observation clocks differ from configuration"));
    }
    if (context.source.empty() || context.sequence == 0 ||
        context.epoch.producer_session.empty() ||
        context.epoch.generation == 0 ||
        observation.covariance_source.empty() ||
        !ValidProvenance(context.provenance) ||
        !u::ValidGlobalObservationGeometry(observation)) {
      return reject(Status(StatusCode::kInvalidArgument,
                           "invalid diagnostic observation"));
    }
    if (context.reference != config_.reference ||
        context.map_content_id != config_.map_content_id ||
        context.map_frame.value != config_.map_frame.value ||
        context.base_frame.value != config_.base_frame.value) {
      return reject(Status(StatusCode::kMapReferenceMismatch,
                           "observation identity differs from configuration"));
    }
    if (context.receive_time.nanoseconds >
            context.measurement_time.nanoseconds &&
        context.receive_time.nanoseconds -
                context.measurement_time.nanoseconds >
            config_.maximum_observation_age_ns) {
      return reject(Status(StatusCode::kInvalidTimestamp,
                           "observation is stale on arrival"));
    }
    if (Seconds(context.measurement_time.nanoseconds) -
            Seconds(context.receive_time.nanoseconds) >
        owner_->config().future_tolerance) {
      return reject(Status(StatusCode::kInvalidTimestamp,
                           "measurement is in the future of reception"));
    }
    if (has_odometry_ && context.epoch != latest_.epoch) {
      return reject(
          Status(StatusCode::kEpochMismatch, "observation ODOM epoch differs"));
    }
    if (has_odometry_ &&
        context.measurement_time.nanoseconds <= latest_.timestamp.nanoseconds) {
      return Apply(context, observation);
    }
    if (pending_.size() >= config_.maximum_pending_observations) {
      return reject(Status(StatusCode::kRejectedMeasurement,
                           "pending observation capacity exhausted"));
    }
    const auto duplicate = std::find_if(
        pending_.begin(), pending_.end(), [&](const Pending& pending) {
          return pending.context.source == context.source &&
                 (pending.context.sequence == context.sequence ||
                  pending.context.measurement_time.nanoseconds ==
                      context.measurement_time.nanoseconds);
        });
    if (duplicate != pending_.end()) {
      return reject(Status(StatusCode::kDuplicateTimestamp,
                           "duplicate pending source observation"));
    }
    const auto position = std::upper_bound(
        pending_.begin(), pending_.end(), context.measurement_time.nanoseconds,
        [](int64_t ns, const Pending& pending) {
          return ns < pending.context.measurement_time.nanoseconds;
        });
    pending_.insert(position, {context, std::move(observation)});
    return Status(StatusCode::kPending,
                  "waiting for ODOM measurement-time bracket");
  }

  Status GetResult(DiagnosticAlignmentResult* output) const {
    return GetResult(latest_.timestamp, output);
  }

  Status GetDecisions(std::vector<GlobalObservationDecision>* decisions) const {
    if (!owner_) return Unconfigured();
    if (decisions == nullptr)
      return Status(StatusCode::kInvalidArgument, "null decision history");
    decisions->assign(decisions_.begin(), decisions_.end());
    return Status::Ok();
  }

  Status GetResult(const Timestamp& evaluation_time,
                   DiagnosticAlignmentResult* output) const {
    if (!owner_) return Unconfigured();
    if (output == nullptr)
      return Status(StatusCode::kInvalidArgument, "null diagnostic result");
    if (!has_odometry_)
      return Status(StatusCode::kStateUnavailable, "ODOM is unavailable");
    const Status time_status = ValidateTimestamp(evaluation_time);
    if (!time_status.ok()) return time_status;
    if (evaluation_time.clock_domain != config_.clock_domain) {
      return Status(StatusCode::kClockDomainMismatch,
                    "evaluation clock differs from configuration");
    }
    if (evaluation_time.nanoseconds < latest_.timestamp.nanoseconds) {
      return Status(StatusCode::kInvalidTimestamp,
                    "evaluation precedes latest ODOM");
    }
    DiagnosticAlignmentResult result;
    result.timestamp = latest_.timestamp;
    result.evaluation_time = evaluation_time;
    result.epoch = latest_.epoch;
    result.reference = config_.reference;
    result.map_content_id = config_.map_content_id;
    result.map_frame = config_.map_frame;
    result.odom_frame = config_.odom_frame;
    result.base_frame = config_.base_frame;
    result.pending_observations = pending_.size();
    result.verification_pending = owner_->state().verifying;
    result.odometry_fresh =
        evaluation_time.nanoseconds - latest_.timestamp.nanoseconds <=
        config_.maximum_odometry_age_ns;
    result.last_full_pose_time = last_full_pose_time_;
    result.full_pose_evidence_fresh =
        last_full_pose_time_.has_value() && owner_->state().valid &&
        evaluation_time.nanoseconds - last_full_pose_time_->nanoseconds <=
            config_.maximum_full_pose_age_ns;
    result.decisions_dropped = decision_sequence_ - decisions_.size();
    result.decisions.assign(decisions_.begin(), decisions_.end());
    const auto state = owner_->Predict(Seconds(latest_.timestamp.nanoseconds));
    if (state.valid) {
      result.estimate = DiagnosticAlignmentEstimate{
          ValuePose(state.map_to_odom),
          ValuePose(state.map_to_odom * Transform(latest_.pose)),
          state.covariance, state.correction_id};
    }
    if (state.horizontal) {
      const auto& horizontal = *state.horizontal;
      const double yaw = horizontal.alignment.z();
      const Eigen::Matrix2d rotation =
          Eigen::Rotation2Dd(yaw).toRotationMatrix();
      DiagnosticHorizontalEstimate estimate;
      estimate.map_from_odom_xy_yaw.head<2>() =
          horizontal.alignment.head<2>() -
          rotation * horizontal.odom_reference_xy;
      estimate.map_from_odom_xy_yaw.z() = yaw;
      estimate.map_base_xy =
          rotation * (latest_.pose.translation.head<2>() -
                      horizontal.odom_reference_xy) +
          horizontal.alignment.head<2>();
      estimate.odom_reference_xy = horizontal.odom_reference_xy;
      estimate.covariance = horizontal.covariance;
      estimate.last_observation_time =
          {horizontal.last_observation_ns, config_.clock_domain};
      estimate.correction_id = state.correction_id;
      result.horizontal_estimate = estimate;
      result.horizontal_evidence_fresh =
          evaluation_time.nanoseconds - horizontal.last_observation_ns <=
              config_.maximum_full_pose_age_ns;
    }
    *output = std::move(result);
    return Status::Ok();
  }

 private:
  static Status Unconfigured() {
    return Status(StatusCode::kStateUnavailable,
                  "diagnostic alignment is not configured");
  }
  Status Apply(const GlobalObservationContext& context,
               const u::GlobalObservation& observation) {
    const uint64_t revision = owner_->state().correction_id;
    ObservationInnovation innovation;
    const auto observed = owner_->Observe(observation, &innovation);
    const Status status = observed.ok()
                              ? Status(StatusCode::kOk, observed.message)
                              : Convert(observed);
    const bool updated =
        status.ok() && owner_->state().correction_id != revision;
    if (updated && observation.kind == u::GlobalObservation::Kind::POSE) {
      last_full_pose_time_ = context.measurement_time;
    }
    Record(context, status, updated, observation.covariance_source,
           &innovation);
    return status;
  }
  void Record(const GlobalObservationContext& context, const Status& status,
              bool updated, const std::string& covariance_source,
              const ObservationInnovation* innovation = nullptr) {
    ObservationInnovation recorded;
    if (innovation != nullptr) {
      recorded = *innovation;
    } else {
      recorded.unavailable_reason = status.message();
    }
    decisions_.push_back({context, status, ++decision_sequence_,
                          owner_->state().correction_id, updated,
                          covariance_source, std::move(recorded)});
    while (decisions_.size() > config_.maximum_decisions)
      decisions_.pop_front();
  }
  Status Reject(const GlobalObservationContext& context, const Status& status,
                const std::string& covariance_source) {
    Record(context, status, false, covariance_source);
    return status;
  }

  DiagnosticAlignmentConfig config_;
  std::unique_ptr<u::GlobalAlignment> owner_;
  std::deque<Pending> pending_;
  std::deque<GlobalObservationDecision> decisions_;
  OdometrySample latest_;
  bool has_odometry_ = false;
  uint64_t local_sequence_ = 0;
  uint64_t decision_sequence_ = 0;
  std::optional<Timestamp> last_full_pose_time_;
};

DiagnosticGlobalAlignment::DiagnosticGlobalAlignment()
    : impl_(std::make_unique<Impl>()) {}
DiagnosticGlobalAlignment::~DiagnosticGlobalAlignment() = default;

Status DiagnosticGlobalAlignment::Configure(
    const DiagnosticAlignmentConfig& config) {
  return impl_->Configure(config);
}
Status DiagnosticGlobalAlignment::AddOdometry(
    const OdometrySample& sample,
    const Eigen::Matrix<double, 6, 6>& covariance) {
  return impl_->AddOdometry(sample, covariance);
}
Status DiagnosticGlobalAlignment::SubmitMapObservation(
    const GlobalMapObservation& input) {
  return impl_->SubmitMap(input);
}
Status DiagnosticGlobalAlignment::SubmitHorizontalObservation(
    const GlobalHorizontalObservation& input) {
  auto observation = Observation(input.context);
  observation.kind = u::GlobalObservation::Kind::HORIZONTAL_POSITION;
  observation.pose.translation().head<2>() = input.position;
  observation.covariance.topLeftCorner<2, 2>() = input.covariance;
  observation.point_in_base = input.point_in_base;
  observation.covariance_source = input.covariance_source;
  return impl_->Submit(input.context, std::move(observation));
}
Status DiagnosticGlobalAlignment::SubmitHeadingObservation(
    const GlobalHeadingObservation& input) {
  auto observation = Observation(input.context);
  observation.kind = u::GlobalObservation::Kind::HEADING;
  observation.heading_rad = input.heading_rad;
  observation.heading_variance = input.variance_rad2;
  observation.direction_in_base = input.direction_in_base;
  observation.covariance_source = input.covariance_source;
  return impl_->Submit(input.context, std::move(observation));
}
Status DiagnosticGlobalAlignment::GetResult(
    DiagnosticAlignmentResult* result) const {
  return impl_->GetResult(result);
}

Status DiagnosticGlobalAlignment::GetResult(
    const Timestamp& evaluation_time, DiagnosticAlignmentResult* result) const {
  return impl_->GetResult(evaluation_time, result);
}

Status DiagnosticGlobalAlignment::GetDecisions(
    std::vector<GlobalObservationDecision>* decisions) const {
  return impl_->GetDecisions(decisions);
}

}  // namespace localization
}  // namespace openloc
