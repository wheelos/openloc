#include "openloc/localization/online/localization_engine.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <utility>

#include "alignment_recovery.h"
#include "alignment_smoother.h"
#include "gnss_quality_gate.h"
#include "openloc/localization/online/validation.h"

namespace openloc {
namespace localization {
namespace {

constexpr int64_t kNanosecondsPerSecond = 1'000'000'000LL;
constexpr double kPi = 3.14159265358979323846;

struct PositionFix {
  Timestamp timestamp;
  FrameId frame;
  MapReference map_reference;
  Eigen::VectorXd position;
  Eigen::MatrixXd covariance;
  GnssReceiverQuality receiver;

  bool horizontal_only() const { return position.size() == 2; }
};

Status Invalid(const std::string& message) {
  return Status(StatusCode::kInvalidArgument, message);
}

double WrappedAngle(double angle) { return std::remainder(angle, 2.0 * kPi); }

double ElapsedSeconds(int64_t newer_ns, int64_t older_ns) {
  return static_cast<double>(newer_ns - older_ns) /
         static_cast<double>(kNanosecondsPerSecond);
}

Eigen::Vector3d BodyPosition(const Eigen::Vector4d& alignment,
                             const Eigen::Vector3d& odom_position) {
  return alignment.head<3>() +
         Eigen::AngleAxisd(alignment[3], Eigen::Vector3d::UnitZ()) *
             odom_position;
}

Eigen::Vector4d PoseDifference(const Eigen::Vector4d& candidate,
                               const Eigen::Vector4d& published,
                               const Eigen::Vector3d& odom_position) {
  Eigen::Vector4d difference;
  difference.head<3>() = BodyPosition(candidate, odom_position) -
                         BodyPosition(published, odom_position);
  difference[3] = WrappedAngle(candidate[3] - published[3]);
  return difference;
}

internal::GnssQualityGatePolicy GnssQualityPolicy(
    const LocalizationConfig& config) {
  internal::GnssQualityGatePolicy policy{};
  policy.minimum_satellites = config.minimum_satellites;
  policy.maximum_hdop = config.maximum_hdop;
  policy.maximum_differential_age_seconds =
      config.maximum_differential_age_seconds;
  policy.maximum_horizontal_accuracy_m = config.maximum_horizontal_accuracy_m;
  policy.maximum_heading_accuracy_rad = config.maximum_heading_accuracy_rad;
  policy.maximum_baseline_length_error_m =
      config.maximum_baseline_length_error_m;
  policy.fixed_solution_stability_ns = config.fixed_solution_stability_ns;
  policy.maximum_fixed_solution_gap_ns = config.maximum_fixed_solution_gap_ns;
  return policy;
}

}  // namespace

class LocalizationEngine::Impl {
 public:
  struct Segment {
    int64_t timestamp_ns = 0;
    uint64_t state_id = 0;
  };

  Status ValidateConfig(const LocalizationConfig& config) const {
    if (config.large_correction_policy !=
            LargeCorrectionPolicy::kHoldForStoppedReset &&
        config.large_correction_policy !=
            LargeCorrectionPolicy::kRateLimitedTracking) {
      return Invalid("unknown large correction policy");
    }
    if (config.odom_gravity_contract != OdomGravityContract::kGravityAligned &&
        config.odom_gravity_contract !=
            OdomGravityContract::kDiagnosticAssumption) {
      return Invalid(
          "4DoF alignment requires an explicit ODOM gravity contract");
    }
    if (config.map_reference.id.empty() ||
        config.map_reference.version.empty() ||
        config.map_reference.calibration_id.empty()) {
      return Invalid("map reference identity is incomplete");
    }
    Status geodetic_status =
        ValidateGeodeticReference(config.map_reference.geodetic);
    if (!geodetic_status.ok()) {
      return geodetic_status;
    }
    const Status quality_status =
        internal::ValidateGnssQualityGatePolicy(GnssQualityPolicy(config));
    if (!quality_status.ok()) {
      return quality_status;
    }
    if (config.map_frame.empty() || config.odom_frame.empty() ||
        config.base_frame.empty() || config.map_frame == config.odom_frame ||
        config.map_frame == config.base_frame ||
        config.odom_frame == config.base_frame ||
        !config.gnss_antenna_in_base.allFinite() ||
        !config.gnss_baseline_in_base.allFinite() ||
        !std::isfinite(config.alignment_lag_seconds) ||
        config.alignment_lag_seconds <= 0.0 ||
        config.alignment_lag_seconds >
            static_cast<double>(std::numeric_limits<int64_t>::max()) /
                kNanosecondsPerSecond ||
        !std::isfinite(config.alignment_segment_distance_m) ||
        config.alignment_segment_distance_m <= 0.0 ||
        config.alignment_segment_max_duration_ns <= 0 ||
        !config.initial_alignment_sigma.allFinite() ||
        (config.initial_alignment_sigma.array() <= 0.0).any() ||
        !config.alignment_random_walk_sigma_per_sqrt_meter.allFinite() ||
        (config.alignment_random_walk_sigma_per_sqrt_meter.array() <= 0.0)
            .any() ||
        !std::isfinite(config.innovation_gate_threshold) ||
        config.innovation_gate_threshold <= 0.0 ||
        !std::isfinite(config.horizontal_innovation_gate_threshold) ||
        config.horizontal_innovation_gate_threshold <= 0.0 ||
        !std::isfinite(config.heading_innovation_gate_threshold) ||
        config.heading_innovation_gate_threshold <= 0.0 ||
        config.minimum_gnss_admission_interval_ns < 0 ||
        config.minimum_heading_admission_interval_ns < 0 ||
        !std::isfinite(config.cross_check_absolute_distance_error_m) ||
        config.cross_check_absolute_distance_error_m < 0.0 ||
        !std::isfinite(config.cross_check_relative_distance_error) ||
        config.cross_check_relative_distance_error < 0.0 ||
        !std::isfinite(config.cross_check_direction_threshold_rad) ||
        config.cross_check_direction_threshold_rad <= 0.0 ||
        !std::isfinite(config.cross_check_minimum_speed_mps) ||
        config.cross_check_minimum_speed_mps < 0.0 ||
        !std::isfinite(config.direct_correction_translation_m) ||
        config.direct_correction_translation_m <= 0.0 ||
        !std::isfinite(config.direct_correction_yaw_rad) ||
        config.direct_correction_yaw_rad <= 0.0 ||
        !std::isfinite(config.large_reset_translation_m) ||
        config.large_reset_translation_m <=
            config.direct_correction_translation_m ||
        !std::isfinite(config.maximum_correction_translation_rate_mps) ||
        config.maximum_correction_translation_rate_mps <= 0.0 ||
        !std::isfinite(config.maximum_correction_yaw_rate_radps) ||
        config.maximum_correction_yaw_rate_radps <= 0.0 ||
        !std::isfinite(config.large_reset_maximum_speed_mps) ||
        config.large_reset_maximum_speed_mps < 0.0 ||
        config.large_reset_consistent_observations == 0 ||
        config.maximum_pending_fixes == 0 ||
        config.maximum_factor_records == 0) {
      return Invalid("localization configuration contains invalid limits");
    }
    OdometryBuffer buffer(config.odometry_buffer);
    OdometrySample sample;
    sample.timestamp = Timestamp{1, ClockDomain::kUnixEpoch};
    sample.epoch = OdometryEpoch{"configuration-validation", 1};
    sample.parent_frame = FrameId{config.odom_frame};
    sample.child_frame = FrameId{config.base_frame};
    return buffer.Add(sample).ok()
               ? Status::Ok()
               : Invalid("odometry buffer configuration is invalid");
  }

  Status Configure(const LocalizationConfig& requested_config) {
    if (configured_) {
      return Invalid("localization engine configuration is immutable");
    }
    Status status = ValidateConfig(requested_config);
    if (!status.ok()) {
      return status;
    }
    status = quality_gate_.Configure(GnssQualityPolicy(requested_config));
    if (!status.ok()) {
      return status;
    }
    config_ = requested_config;
    odometry_buffer_ =
        std::make_unique<OdometryBuffer>(config_.odometry_buffer);
    internal::AlignmentSmootherConfig smoother_config;
    smoother_config.lag_seconds = config_.alignment_lag_seconds;
    smoother_config.initial_alignment.setZero();
    smoother_config.initial_sigma = config_.initial_alignment_sigma;
    smoother_config.random_walk_sigma_per_sqrt_meter =
        config_.alignment_random_walk_sigma_per_sqrt_meter;
    smoother_config.innovation_gate_threshold =
        config_.innovation_gate_threshold;
    smoother_config.horizontal_innovation_gate_threshold =
        config_.horizontal_innovation_gate_threshold;
    smoother_config.heading_innovation_gate_threshold =
        config_.heading_innovation_gate_threshold;
    smoother_ = std::make_unique<internal::AlignmentSmoother>(smoother_config);
    configured_ = true;
    return Status::Ok();
  }

  Status AddOdometry(const OdometrySample& sample) {
    if (!configured_) {
      return Status(StatusCode::kStateUnavailable,
                    "localization engine is not configured");
    }
    if (sample.parent_frame.value != config_.odom_frame ||
        sample.child_frame.value != config_.base_frame) {
      return Status(StatusCode::kFrameMismatch,
                    "odometry frame pair differs from configured frames");
    }
    Status status = odometry_buffer_->Add(sample);
    if (!status.ok()) {
      return status;
    }
    const int64_t timestamp_ns = sample.timestamp.nanoseconds;
    if (has_odometry_) {
      const double distance =
          (sample.pose.translation - previous_odometry_.pose.translation)
              .norm();
      distance_since_segment_m_ += distance;
      total_travel_distance_m_ += distance;
      instantaneous_speed_mps_ =
          distance / ElapsedSeconds(timestamp_ns,
                                    previous_odometry_.timestamp.nanoseconds);
    } else {
      instantaneous_speed_mps_ = 0.0;
      alignment_reference_in_odom_ = sample.pose.translation;
      alignment_reference_in_odom_.z() = 0.0;
    }

    if (segments_.empty() ||
        distance_since_segment_m_ >= config_.alignment_segment_distance_m ||
        timestamp_ns - segments_.back().timestamp_ns >=
            config_.alignment_segment_max_duration_ns) {
      uint64_t state_id = 0;
      status = smoother_->AddState(timestamp_ns, distance_since_segment_m_,
                                   &state_id);
      if (!status.ok()) {
        return status;
      }
      segments_.push_back(Segment{timestamp_ns, state_id});
      distance_since_segment_m_ = 0.0;
    }

    previous_odometry_ = sample;
    has_odometry_ = true;
    latest_odometry_timestamp_ns_ = timestamp_ns;
    PruneSegments();
    if (config_.large_correction_policy ==
            LargeCorrectionPolicy::kRateLimitedTracking &&
        has_published_alignment_ && !CorrectionSourceFresh()) {
      // A queued fresh fix must not spend time accumulated during stale ODOM.
      last_published_timestamp_ns_ = latest_odometry_timestamp_ns_;
    }
    status = ProcessPendingFixes();
    if (!status.ok()) {
      return status;
    }
    status = ProcessPendingHeadings();
    if (!status.ok()) {
      return status;
    }
    if (config_.large_correction_policy ==
            LargeCorrectionPolicy::kRateLimitedTracking &&
        has_published_alignment_) {
      if (CorrectionSourceFresh()) {
        status = ApplyCorrectionPolicy();
        if (!status.ok()) {
          return status;
        }
      } else {
        // Do not bank rate-limit credit during an outage.
        last_published_timestamp_ns_ = latest_odometry_timestamp_ns_;
      }
    }
    return RefreshEstimate();
  }

  Status SubmitGnssFix(const GnssFix& fix) {
    return SubmitPositionFix({fix.timestamp, fix.frame, fix.map_reference,
                              fix.position, fix.covariance, fix.receiver});
  }

  Status SubmitGnssHorizontalFix(const GnssHorizontalFix& fix) {
    return SubmitPositionFix({fix.timestamp, fix.frame, fix.map_reference,
                              fix.position, fix.covariance, fix.receiver});
  }

  Status SubmitPositionFix(const PositionFix& fix) {
    if (!configured_) {
      return Status(StatusCode::kStateUnavailable,
                    "localization engine is not configured");
    }
    Status status = ValidateGnssFix(fix);
    if (!status.ok()) {
      recovery_.Reset();
      if (ValidateTimestamp(fix.timestamp).ok()) {
        AddRejectionRecord(fix, 0, status.message());
      }
      return status;
    }
    if (has_submitted_fix_ &&
        (fix.timestamp.clock_domain != last_submitted_fix_.clock_domain ||
         fix.timestamp.nanoseconds <= last_submitted_fix_.nanoseconds)) {
      recovery_.Reset();
      AddRejectionRecord(fix, 0, "out_of_order_gnss_timestamp");
      return Status(StatusCode::kOutOfOrderTimestamp,
                    "GNSS sensor timestamps must increase strictly");
    }
    last_submitted_fix_ = fix.timestamp;
    has_submitted_fix_ = true;
    if (!has_odometry_ ||
        fix.timestamp.nanoseconds > latest_odometry_timestamp_ns_) {
      if (pending_fixes_.size() >= config_.maximum_pending_fixes) {
        AddRejectionRecord(fix, 0, "pending_gnss_queue_full");
        return Status(StatusCode::kStateUnavailable,
                      "pending GNSS queue reached its configured capacity");
      }
      pending_fixes_.push_back(fix);
      AddRejectionRecord(fix, 0, "pending_odometry_bracket");
      return Status(StatusCode::kPending,
                    "GNSS fix awaits a right-hand odometry bracket");
    }
    return ProcessFix(fix);
  }

  Status SubmitGnssHeading(const GnssHeading& heading) {
    if (!configured_) {
      return Status(StatusCode::kStateUnavailable,
                    "localization engine is not configured");
    }
    Status status = ValidateGnssHeading(heading);
    if (!status.ok()) {
      if (ValidateTimestamp(heading.timestamp).ok()) {
        AddHeadingRejectionRecord(heading, status.message());
      }
      return status;
    }
    if (has_submitted_heading_ && (heading.timestamp.clock_domain !=
                                       last_submitted_heading_.clock_domain ||
                                   heading.timestamp.nanoseconds <=
                                       last_submitted_heading_.nanoseconds)) {
      AddHeadingRejectionRecord(heading, "out_of_order_heading_timestamp");
      return Status(StatusCode::kOutOfOrderTimestamp,
                    "GNSS heading sensor timestamps must increase strictly");
    }
    last_submitted_heading_ = heading.timestamp;
    has_submitted_heading_ = true;
    if (!has_odometry_ ||
        heading.timestamp.nanoseconds > latest_odometry_timestamp_ns_) {
      if (pending_headings_.size() >= config_.maximum_pending_fixes) {
        AddHeadingRejectionRecord(heading, "pending_heading_queue_full");
        return Status(
            StatusCode::kStateUnavailable,
            "pending GNSS heading queue reached its configured capacity");
      }
      pending_headings_.push_back(heading);
      AddHeadingRejectionRecord(heading, "pending_odometry_bracket");
      return Status(StatusCode::kPending,
                    "GNSS heading awaits a right-hand odometry bracket");
    }
    return ProcessHeading(heading);
  }

  Status GetResult(LocalizationResult* result) const {
    if (result == nullptr) {
      return Invalid("localization result output must not be null");
    }
    if (!configured_ || !has_odometry_) {
      return Status(StatusCode::kStateUnavailable,
                    "localization estimate is not available");
    }
    *result = result_;
    return Status::Ok();
  }

  Status Reset() {
    if (!configured_) {
      return Status(StatusCode::kStateUnavailable,
                    "localization engine is not configured");
    }
    ++reset_counter_;
    const LocalizationConfig saved_config = config_;
    configured_ = false;
    odometry_buffer_.reset();
    smoother_.reset();
    segments_.clear();
    pending_fixes_.clear();
    pending_headings_.clear();
    factor_records_.clear();
    has_odometry_ = false;
    has_submitted_fix_ = false;
    quality_gate_.Reset();
    has_previous_quality_fix_ = false;
    has_admitted_fix_ = false;
    has_admitted_vertical_fix_ = false;
    has_published_alignment_ = false;
    has_submitted_heading_ = false;
    has_admitted_heading_ = false;
    heading_initialized_ = false;
    distance_since_segment_m_ = 0.0;
    total_travel_distance_m_ = 0.0;
    instantaneous_speed_mps_ = 0.0;
    published_alignment_.setZero();
    alignment_reference_in_odom_.setZero();
    pending_large_reset_alignment_.setZero();
    pending_large_reset_count_ = 0;
    alignment_revision_ = 0;
    publication_state_ = CorrectionPublicationState::kUninitialized;
    recovery_.Reset();
    latest_odometry_timestamp_ns_ = 0;
    last_published_timestamp_ns_ = 0;
    last_submitted_fix_ = Timestamp{};
    last_submitted_heading_ = Timestamp{};
    last_admitted_fix_ = Timestamp{};
    last_admitted_heading_ = Timestamp{};
    previous_odometry_ = OdometrySample{};
    last_good_fix_ = PositionFix{};
    result_ = LocalizationResult{};
    result_.reset_event = true;
    const uint32_t next_reset_counter = reset_counter_;
    Status status = Configure(saved_config);
    reset_counter_ = next_reset_counter;
    result_.reset_counter = reset_counter_;
    return status;
  }

 private:
  Status ValidateGnssFix(const PositionFix& fix) const {
    Status status = ValidateTimestamp(fix.timestamp);
    if (!status.ok()) {
      return status;
    }
    status = ValidateFrameId(fix.frame);
    if (!status.ok()) {
      return status;
    }
    if (fix.frame.value != config_.map_frame) {
      return Status(StatusCode::kFrameMismatch,
                    "GNSS position must use the configured map frame");
    }
    if (fix.map_reference != config_.map_reference) {
      return Status(StatusCode::kMapReferenceMismatch,
                    "GNSS position map reference differs from configuration");
    }
    if ((fix.position.size() != 2 && fix.position.size() != 3) ||
        !fix.position.allFinite() || !std::isfinite(fix.receiver.hdop) ||
        fix.receiver.hdop <= 0.0 ||
        !std::isfinite(fix.receiver.differential_age_seconds) ||
        fix.receiver.differential_age_seconds < 0.0 ||
        !std::isfinite(fix.receiver.horizontal_accuracy_m) ||
        fix.receiver.horizontal_accuracy_m <= 0.0) {
      return Invalid(
          "GNSS position must be finite and receiver quality must be valid");
    }
    return ValidateCovariance(fix.covariance, fix.position.size());
  }

  Status ValidateGnssHeading(const GnssHeading& heading) const {
    Status status = ValidateTimestamp(heading.timestamp);
    if (!status.ok()) {
      return status;
    }
    status = ValidateFrameId(heading.frame);
    if (!status.ok()) {
      return status;
    }
    if (heading.frame.value != config_.map_frame) {
      return Status(StatusCode::kFrameMismatch,
                    "GNSS heading must use the configured map frame");
    }
    if (heading.map_reference != config_.map_reference) {
      return Status(StatusCode::kMapReferenceMismatch,
                    "GNSS heading map reference differs from configuration");
    }
    const GnssReceiverQuality& receiver = heading.receiver;
    if (!std::isfinite(heading.azimuth_rad) ||
        !std::isfinite(heading.accuracy_rad) || heading.accuracy_rad <= 0.0 ||
        !std::isfinite(heading.measured_baseline_length_m) ||
        heading.measured_baseline_length_m <= 0.0 ||
        !std::isfinite(receiver.hdop) || receiver.hdop <= 0.0 ||
        !std::isfinite(receiver.differential_age_seconds) ||
        receiver.differential_age_seconds < 0.0 ||
        !std::isfinite(receiver.horizontal_accuracy_m) ||
        receiver.horizontal_accuracy_m <= 0.0) {
      return Invalid(
          "GNSS heading must be finite and receiver quality must be valid");
    }
    return Status::Ok();
  }

  Status ProcessPendingFixes() {
    while (!pending_fixes_.empty()) {
      const PositionFix& fix = pending_fixes_.front();
      if (fix.timestamp.nanoseconds > latest_odometry_timestamp_ns_) {
        break;
      }
      Timestamp oldest_timestamp;
      if (odometry_buffer_->OldestTimestamp(&oldest_timestamp) &&
          fix.timestamp.nanoseconds < oldest_timestamp.nanoseconds) {
        AddRejectionRecord(fix, 0, "outside_odometry_history");
        pending_fixes_.pop_front();
        continue;
      }
      Status status = ProcessFix(fix);
      pending_fixes_.pop_front();
      if (!status.ok() && status.code() != StatusCode::kRejectedMeasurement &&
          status.code() != StatusCode::kOutsideHistory &&
          status.code() != StatusCode::kInterpolationGapTooLarge &&
          status.code() != StatusCode::kOutsideFixedLagWindow &&
          status.code() != StatusCode::kClockDomainMismatch) {
        return status;
      }
    }
    return Status::Ok();
  }

  Status ProcessPendingHeadings() {
    while (!pending_headings_.empty()) {
      const GnssHeading& heading = pending_headings_.front();
      if (heading.timestamp.nanoseconds > latest_odometry_timestamp_ns_) {
        break;
      }
      Timestamp oldest_timestamp;
      if (odometry_buffer_->OldestTimestamp(&oldest_timestamp) &&
          heading.timestamp.nanoseconds < oldest_timestamp.nanoseconds) {
        AddHeadingRejectionRecord(heading, "outside_odometry_history");
        pending_headings_.pop_front();
        continue;
      }
      Status status = ProcessHeading(heading);
      pending_headings_.pop_front();
      if (!status.ok() && status.code() != StatusCode::kRejectedMeasurement &&
          status.code() != StatusCode::kOutsideHistory &&
          status.code() != StatusCode::kInterpolationGapTooLarge &&
          status.code() != StatusCode::kOutsideFixedLagWindow &&
          status.code() != StatusCode::kClockDomainMismatch) {
        return status;
      }
    }
    return Status::Ok();
  }

  Status ProcessFix(const PositionFix& fix) {
    const auto start = std::chrono::steady_clock::now();
    const auto record_rejection = [&](const std::string& reason,
                                      bool reset_recovery = true) {
      if (reset_recovery) {
        recovery_.Reset();
      }
      pending_large_reset_count_ = 0;
      const double latency_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - start)
                                    .count();
      AddRejectionRecord(fix, 0, reason, latency_ms);
    };
    std::string rejection_reason;
    const auto quality =
        quality_gate_.CheckPosition(fix.timestamp, fix.receiver);
    if (!quality.status.ok()) {
      // Stability waiting admits no observation, but previously qualified
      // recovery evidence can survive a report gap within its total-age bound.
      record_rejection(quality.status.message(),
                       quality.outcome !=
                           internal::GnssQualityOutcome::kWaitingForStability);
      return quality.status;
    }
    Status status;

    if (!has_odometry_) {
      record_rejection("odometry_unavailable");
      return Status(StatusCode::kRejectedMeasurement,
                    "odometry is unavailable for GNSS association");
    }
    if (fix.timestamp.clock_domain !=
        previous_odometry_.timestamp.clock_domain) {
      record_rejection("clock_domain_mismatch");
      return Status(StatusCode::kClockDomainMismatch,
                    "GNSS and odometry clock domains differ");
    }

    OdometrySample antenna_pose;
    status = odometry_buffer_->Interpolate(fix.timestamp, &antenna_pose);
    if (!status.ok()) {
      record_rejection(status.message());
      return status;
    }
    if (segments_.empty() ||
        fix.timestamp.nanoseconds < segments_.front().timestamp_ns) {
      record_rejection("outside_fixed_lag_window");
      return Status(StatusCode::kOutsideFixedLagWindow,
                    "GNSS timestamp has no alignment state in the lag window");
    }

    status = CheckOdometryConsistency(fix, antenna_pose, &rejection_reason);
    if (!status.ok()) {
      record_rejection(rejection_reason.empty() ? status.message()
                                                : rejection_reason);
      return status;
    }
    if (has_admitted_fix_ &&
        fix.timestamp.nanoseconds - last_admitted_fix_.nanoseconds <
            config_.minimum_gnss_admission_interval_ns) {
      record_rejection("gnss_admission_rate_exceeded");
      return Status(StatusCode::kRejectedMeasurement,
                    "GNSS admission rate exceeds configured maximum");
    }

    const auto segment = SegmentFor(fix.timestamp.nanoseconds);
    if (segment == segments_.end()) {
      record_rejection("outside_fixed_lag_window");
      return Status(StatusCode::kOutsideFixedLagWindow,
                    "GNSS timestamp has no active alignment segment");
    }
    const Eigen::Vector3d antenna_in_odom =
        antenna_pose.pose.translation - alignment_reference_in_odom_ +
        antenna_pose.pose.rotation * config_.gnss_antenna_in_base;
    if (!has_admitted_fix_ && !heading_initialized_ &&
        config_.enable_single_antenna_bootstrap) {
      internal::GnssAlignmentRecoveryObservation observation;
      observation.timestamp_ns = fix.timestamp.nanoseconds;
      observation.odom_antenna_xy = antenna_in_odom.head<2>();
      observation.global_antenna_xy = fix.position.head<2>();
      observation.covariance_xy = fix.covariance.topLeftCorner<2, 2>();
      internal::GnssAlignmentRecoveryDiagnostics bootstrap_diagnostics;
      internal::GnssAlignmentRecoveryCandidate candidate;
      status = recovery_.AddObservation(observation, &bootstrap_diagnostics,
                                        &candidate);
      if (!status.ok()) {
        recovery_.Reset();
        return status;
      }

      FactorRecord record;
      record.timestamp_ns = fix.timestamp.nanoseconds;
      record.source = "gnss_single_antenna_bootstrap";
      record.type = "GLOBAL_XY_YAW";
      record.residual =
          Eigen::VectorXd::Constant(1, bootstrap_diagnostics.training_rms_m);
      record.normalized_innovation_squared =
          bootstrap_diagnostics.validation_nis;
      record.innovation_threshold =
          internal::GnssAlignmentRecoveryConfig{}.innovation_gate_threshold;
      record.covariance_diagonal = fix.covariance.diagonal();
      record.state_id = segment->state_id;
      record.reason = bootstrap_diagnostics.reason;
      record.accepted = bootstrap_diagnostics.candidate_validated;
      AppendRecord(std::move(record));
      if (!bootstrap_diagnostics.candidate_validated) {
        return Status(StatusCode::kRejectedMeasurement,
                      "single-antenna bootstrap awaits observable baseline "
                      "and GNSS holdout validation");
      }

      return CommitRecoveryCandidate(
          fix, candidate, bootstrap_diagnostics,
          "single_antenna_course_bootstrap_holdout_validated", true);
    }

    internal::GnssFactorDiagnostics diagnostics;
    const bool initializing = !has_admitted_fix_;
    if (fix.horizontal_only()) {
      status = smoother_->AddGnssHorizontalPosition(
          segment->state_id, fix.timestamp.nanoseconds, antenna_in_odom,
          fix.position, fix.covariance, &diagnostics, initializing);
    } else {
      status = smoother_->AddGnssPosition(
          segment->state_id, fix.timestamp.nanoseconds, antenna_in_odom,
          fix.position, fix.covariance, &diagnostics, initializing);
    }

    FactorRecord record;
    record.type = fix.horizontal_only() ? "ABS_POS2" : "ABS_POS3";
    record.timestamp_ns = fix.timestamp.nanoseconds;
    record.residual = diagnostics.residual;
    record.normalized_innovation_squared =
        diagnostics.normalized_innovation_squared;
    record.innovation_threshold =
        fix.horizontal_only() ? config_.horizontal_innovation_gate_threshold
                              : config_.innovation_gate_threshold;
    record.robust_weight = diagnostics.robust_weight;
    record.accepted = status.ok();
    record.reason =
        status.ok() ? (initializing ? "accepted_initialization" : "accepted")
                    : status.message();
    record.covariance_diagonal = fix.covariance.diagonal();
    record.state_id = segment->state_id;
    record.processing_latency_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - start)
                                       .count();
    AppendRecord(std::move(record));
    if (!status.ok()) {
      pending_large_reset_count_ = 0;
      if (config_.enable_gnss_reacquisition &&
          status.code() == StatusCode::kRejectedMeasurement &&
          diagnostics.horizontal_innovation_rejected) {
        internal::GnssAlignmentRecoveryObservation observation;
        observation.timestamp_ns = fix.timestamp.nanoseconds;
        observation.odom_antenna_xy = antenna_in_odom.head<2>();
        observation.global_antenna_xy = fix.position.head<2>();
        observation.covariance_xy = fix.covariance.topLeftCorner<2, 2>();
        internal::GnssAlignmentRecoveryDiagnostics recovery_diagnostics;
        internal::GnssAlignmentRecoveryCandidate candidate;
        const Status recovery_status = recovery_.AddObservation(
            observation, &recovery_diagnostics, &candidate);
        if (!recovery_status.ok()) {
          recovery_.Reset();
          return recovery_status;
        }
        if (recovery_diagnostics.candidate_validated) {
          return CommitRecoveryCandidate(
              fix, candidate, recovery_diagnostics,
              "recovery_holdout_validated_global_reanchor", false);
        }
      } else {
        recovery_.Reset();
      }
      return status;
    }

    recovery_.Reset();
    last_admitted_fix_ = fix.timestamp;
    has_admitted_fix_ = true;
    has_admitted_vertical_fix_ =
        has_admitted_vertical_fix_ || !fix.horizontal_only();
    last_good_fix_ = fix;
    has_previous_quality_fix_ = true;
    status = ApplyCorrectionPolicy();
    if (!status.ok()) {
      return status;
    }
    return RefreshEstimate();
  }

  Status CommitRecoveryCandidate(
      const PositionFix& fix,
      const internal::GnssAlignmentRecoveryCandidate& candidate,
      const internal::GnssAlignmentRecoveryDiagnostics& diagnostics,
      const std::string& reason, bool initial_bootstrap) {
    Eigen::Vector4d recovered_alignment;
    recovered_alignment.head<2>() = candidate.global_from_odom_xy_yaw.head<2>();
    recovered_alignment[3] = candidate.global_from_odom_xy_yaw[2];
    Eigen::Vector4d recovered_sigma = config_.initial_alignment_sigma;
    recovered_sigma.head<2>() = candidate.sigma_xy_yaw.head<2>();
    recovered_sigma[3] = candidate.sigma_xy_yaw[2];
    internal::AlignmentEstimate current_estimate;
    Status status = smoother_->Estimate(&current_estimate);
    if (!status.ok()) {
      recovery_.Reset();
      return status;
    }
    recovered_alignment[2] = current_estimate.map_from_odom[2];
    recovered_sigma[2] =
        std::max(recovered_sigma[2],
                 std::sqrt(std::max(0.0, current_estimate.covariance(2, 2))));
    uint64_t recovered_state_id = 0;
    status =
        smoother_->Reanchor(latest_odometry_timestamp_ns_, recovered_alignment,
                            recovered_sigma, &recovered_state_id);
    if (!status.ok()) {
      recovery_.Reset();
      return status;
    }
    segments_.clear();
    segments_.push_back(
        Segment{latest_odometry_timestamp_ns_, recovered_state_id});
    distance_since_segment_m_ = 0.0;
    pending_large_reset_count_ = 0;
    ++alignment_revision_;
    last_admitted_fix_ = fix.timestamp;
    has_admitted_fix_ = true;
    has_admitted_vertical_fix_ =
        has_admitted_vertical_fix_ || !fix.horizontal_only();
    last_good_fix_ = fix;
    has_previous_quality_fix_ = true;

    if (initial_bootstrap) {
      if (factor_records_.empty() ||
          factor_records_.back().timestamp_ns != fix.timestamp.nanoseconds ||
          factor_records_.back().source != "gnss_single_antenna_bootstrap") {
        recovery_.Reset();
        return Status(StatusCode::kStateUnavailable,
                      "validated bootstrap record is unavailable");
      }
      FactorRecord& bootstrap_record = factor_records_.back();
      bootstrap_record.residual =
          Eigen::VectorXd::Constant(1, diagnostics.training_rms_m);
      bootstrap_record.normalized_innovation_squared =
          diagnostics.validation_nis;
      bootstrap_record.accepted = true;
      bootstrap_record.reason = reason;
      bootstrap_record.state_id = recovered_state_id;
    } else {
      FactorRecord recovery_record;
      recovery_record.timestamp_ns = fix.timestamp.nanoseconds;
      recovery_record.source = "gnss_recovery";
      recovery_record.type = "GLOBAL_XY_YAW";
      recovery_record.residual =
          Eigen::VectorXd::Constant(1, diagnostics.training_rms_m);
      recovery_record.normalized_innovation_squared =
          diagnostics.validation_nis;
      recovery_record.innovation_threshold =
          internal::GnssAlignmentRecoveryConfig{}.innovation_gate_threshold;
      recovery_record.accepted = true;
      recovery_record.reason = reason;
      recovery_record.state_id = recovered_state_id;
      AppendRecord(std::move(recovery_record));
    }
    recovery_.Reset();
    status = ApplyCorrectionPolicy();
    if (!status.ok()) {
      return status;
    }
    return RefreshEstimate();
  }

  Status ProcessHeading(const GnssHeading& heading) {
    const auto start = std::chrono::steady_clock::now();
    const auto record_rejection = [&](const std::string& reason) {
      pending_large_reset_count_ = 0;
      AddHeadingRejectionRecord(heading, reason,
                                std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - start)
                                    .count());
    };
    const auto quality = quality_gate_.CheckHeading(
        heading.timestamp, heading.receiver, heading.accuracy_rad,
        heading.measured_baseline_length_m,
        config_.gnss_baseline_in_base.norm());
    if (!quality.status.ok()) {
      record_rejection(quality.status.message());
      return quality.status;
    }
    if (!has_odometry_) {
      record_rejection("odometry_unavailable");
      return Status(StatusCode::kRejectedMeasurement,
                    "odometry is unavailable for heading association");
    }
    if (heading.timestamp.clock_domain !=
        previous_odometry_.timestamp.clock_domain) {
      record_rejection("clock_domain_mismatch");
      return Status(StatusCode::kClockDomainMismatch,
                    "heading and odometry clock domains differ");
    }
    OdometrySample odometry_at_heading;
    Status status =
        odometry_buffer_->Interpolate(heading.timestamp, &odometry_at_heading);
    if (!status.ok()) {
      record_rejection(status.message());
      return status;
    }
    if (segments_.empty() ||
        heading.timestamp.nanoseconds < segments_.front().timestamp_ns) {
      record_rejection("outside_fixed_lag_window");
      return Status(
          StatusCode::kOutsideFixedLagWindow,
          "heading timestamp has no alignment state in the lag window");
    }
    if (has_admitted_heading_ &&
        heading.timestamp.nanoseconds - last_admitted_heading_.nanoseconds <
            config_.minimum_heading_admission_interval_ns) {
      record_rejection("heading_admission_rate_exceeded");
      return Status(StatusCode::kRejectedMeasurement,
                    "heading admission rate exceeds configured maximum");
    }
    const auto segment = SegmentFor(heading.timestamp.nanoseconds);
    if (segment == segments_.end()) {
      record_rejection("outside_fixed_lag_window");
      return Status(StatusCode::kOutsideFixedLagWindow,
                    "heading timestamp has no active alignment segment");
    }
    const double baseline_yaw_in_base = std::atan2(
        config_.gnss_baseline_in_base.y(), config_.gnss_baseline_in_base.x());
    const Eigen::Matrix3d odom_rotation =
        odometry_at_heading.pose.rotation.toRotationMatrix();
    const double odom_base_yaw =
        std::atan2(odom_rotation(1, 0), odom_rotation(0, 0));
    internal::HeadingFactorDiagnostics diagnostics;
    const bool initializing = !heading_initialized_;
    status = smoother_->AddGnssHeading(
        segment->state_id, heading.timestamp.nanoseconds, odom_base_yaw,
        baseline_yaw_in_base, heading.azimuth_rad,
        heading.accuracy_rad * heading.accuracy_rad, &diagnostics,
        initializing);

    FactorRecord record;
    record.timestamp_ns = heading.timestamp.nanoseconds;
    record.source = "gnss_heading";
    record.type = "ABS_YAW1";
    record.residual = Eigen::VectorXd::Constant(1, diagnostics.residual_rad);
    record.normalized_innovation_squared =
        diagnostics.normalized_innovation_squared;
    record.innovation_threshold = config_.heading_innovation_gate_threshold;
    record.robust_weight = diagnostics.robust_weight;
    record.accepted = status.ok();
    record.reason =
        status.ok()
            ? (initializing ? "accepted_heading_initialization" : "accepted")
            : status.message();
    record.covariance_diagonal = Eigen::VectorXd::Constant(
        1, heading.accuracy_rad * heading.accuracy_rad);
    record.state_id = segment->state_id;
    record.processing_latency_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - start)
                                       .count();
    AppendRecord(std::move(record));
    if (!status.ok()) {
      pending_large_reset_count_ = 0;
      return status;
    }
    status = ApplyCorrectionPolicy();
    if (!status.ok()) {
      return status;
    }
    heading_initialized_ = true;
    has_admitted_heading_ = true;
    last_admitted_heading_ = heading.timestamp;
    if (!has_admitted_fix_) {
      recovery_.Reset();
    }
    return RefreshEstimate();
  }

  Status CheckOdometryConsistency(const PositionFix& fix,
                                  const OdometrySample& odometry_at_fix,
                                  std::string* rejection_reason) const {
    if (!has_previous_quality_fix_) {
      return Status::Ok();
    }
    const int64_t interval_ns =
        fix.timestamp.nanoseconds - last_good_fix_.timestamp.nanoseconds;
    if (interval_ns <= 0 ||
        interval_ns > config_.maximum_fixed_solution_gap_ns) {
      return Status::Ok();
    }
    OdometrySample previous_odometry_at_fix;
    Status status = odometry_buffer_->Interpolate(last_good_fix_.timestamp,
                                                  &previous_odometry_at_fix);
    if (!status.ok()) {
      *rejection_reason = "previous_gnss_outside_odometry_history";
      return Status(StatusCode::kRejectedMeasurement, *rejection_reason);
    }
    const Eigen::Vector3d odometry_delta =
        odometry_at_fix.pose.translation -
        previous_odometry_at_fix.pose.translation;
    const Eigen::Vector2d gnss_delta =
        fix.position.head<2>() - last_good_fix_.position.head<2>();
    const double elapsed_seconds = ElapsedSeconds(
        fix.timestamp.nanoseconds, last_good_fix_.timestamp.nanoseconds);
    const double odometry_distance = odometry_delta.head<2>().norm();
    const double gnss_distance = gnss_delta.head<2>().norm();
    const double speed = odometry_distance / elapsed_seconds;
    const double distance_tolerance =
        config_.cross_check_absolute_distance_error_m +
        config_.cross_check_relative_distance_error * odometry_distance;
    if (std::abs(gnss_distance - odometry_distance) > distance_tolerance) {
      *rejection_reason = "gnss_odometry_distance_disagreement";
      return Status(StatusCode::kRejectedMeasurement, *rejection_reason);
    }
    if (result_.heading_initialized &&
        speed > config_.cross_check_minimum_speed_mps &&
        odometry_distance > 1e-6 && gnss_distance > 1e-6) {
      internal::AlignmentEstimate alignment;
      status = smoother_->Estimate(&alignment);
      if (!status.ok()) {
        return status;
      }
      const Eigen::Vector3d odometry_delta_in_map =
          Eigen::AngleAxisd(alignment.map_from_odom[3],
                            Eigen::Vector3d::UnitZ()) *
          odometry_delta;
      const double direction_error = std::abs(WrappedAngle(
          std::atan2(gnss_delta.y(), gnss_delta.x()) -
          std::atan2(odometry_delta_in_map.y(), odometry_delta_in_map.x())));
      if (direction_error > config_.cross_check_direction_threshold_rad) {
        *rejection_reason = "gnss_odometry_direction_disagreement";
        return Status(StatusCode::kRejectedMeasurement, *rejection_reason);
      }
    }
    return Status::Ok();
  }

  std::vector<Segment>::const_iterator SegmentFor(int64_t timestamp_ns) const {
    const auto next =
        std::upper_bound(segments_.begin(), segments_.end(), timestamp_ns,
                         [](int64_t timestamp, const Segment& segment) {
                           return timestamp < segment.timestamp_ns;
                         });
    return next == segments_.begin() ? segments_.end() : std::prev(next);
  }

  Status ApplyCorrectionPolicy() {
    if (config_.large_correction_policy ==
            LargeCorrectionPolicy::kRateLimitedTracking &&
        has_published_alignment_ && !CorrectionSourceFresh()) {
      last_published_timestamp_ns_ = latest_odometry_timestamp_ns_;
      return Status::Ok();
    }
    internal::AlignmentEstimate estimate;
    Status status = EstimateInOdometryFrame(&estimate);
    if (!status.ok()) {
      return status;
    }
    Eigen::Vector4d candidate = estimate.map_from_odom;
    candidate[3] = WrappedAngle(candidate[3]);
    const int64_t timestamp_ns = latest_odometry_timestamp_ns_;
    const Eigen::Vector3d& odom_position = previous_odometry_.pose.translation;
    if (!has_published_alignment_) {
      const bool material_initialization =
          PoseDifference(candidate, Eigen::Vector4d::Zero(), odom_position)
                  .head<3>()
                  .norm() > config_.direct_correction_translation_m ||
          std::abs(candidate[3]) > config_.direct_correction_yaw_rad;
      published_alignment_ = candidate;
      last_published_timestamp_ns_ = timestamp_ns;
      has_published_alignment_ = true;
      publication_state_ = CorrectionPublicationState::kAligned;
      pending_large_reset_count_ = 0;
      if (material_initialization) {
        ++reset_counter_;
        result_.reset_event = true;
      }
      return Status::Ok();
    }
    Eigen::Vector4d correction =
        PoseDifference(candidate, published_alignment_, odom_position);
    const double translation_delta = correction.head<3>().norm();
    const double yaw_delta = std::abs(correction[3]);
    const bool tracking = config_.large_correction_policy ==
                          LargeCorrectionPolicy::kRateLimitedTracking;
    if ((!tracking || (translation_delta < 1e-12 && yaw_delta < 1e-12)) &&
        translation_delta <= config_.direct_correction_translation_m &&
        yaw_delta <= config_.direct_correction_yaw_rad) {
      published_alignment_ = candidate;
      publication_state_ = CorrectionPublicationState::kAligned;
      last_published_timestamp_ns_ = timestamp_ns;
      pending_large_reset_count_ = 0;
      return Status::Ok();
    }
    if (!tracking && translation_delta > config_.large_reset_translation_m) {
      publication_state_ = CorrectionPublicationState::kHeldForReset;
      if (instantaneous_speed_mps_ > config_.large_reset_maximum_speed_mps) {
        pending_large_reset_count_ = 0;
        last_published_timestamp_ns_ = timestamp_ns;
        return Status::Ok();
      }
      const Eigen::Vector4d pending_delta = PoseDifference(
          candidate, pending_large_reset_alignment_, odom_position);
      if (pending_large_reset_count_ == 0 ||
          pending_delta.head<3>().norm() >
              config_.direct_correction_translation_m ||
          std::abs(pending_delta[3]) > config_.direct_correction_yaw_rad) {
        pending_large_reset_alignment_ = candidate;
        pending_large_reset_count_ = 1;
      } else if (timestamp_ns > last_published_timestamp_ns_) {
        ++pending_large_reset_count_;
        pending_large_reset_alignment_ = candidate;
      }
      if (pending_large_reset_count_ >=
              config_.large_reset_consistent_observations &&
          instantaneous_speed_mps_ <= config_.large_reset_maximum_speed_mps) {
        published_alignment_ = candidate;
        publication_state_ = CorrectionPublicationState::kAligned;
        ++reset_counter_;
        pending_large_reset_count_ = 0;
        result_.reset_event = true;
      }
      last_published_timestamp_ns_ = timestamp_ns;
      return Status::Ok();
    }

    const double elapsed_seconds =
        last_published_timestamp_ns_ == 0
            ? 0.0
            : std::max(0.0, ElapsedSeconds(timestamp_ns,
                                           last_published_timestamp_ns_));
    if (elapsed_seconds == 0.0) {
      publication_state_ = CorrectionPublicationState::kRateLimited;
      last_published_timestamp_ns_ = timestamp_ns;
      pending_large_reset_count_ = 0;
      return Status::Ok();
    }
    const double maximum_translation_step =
        config_.maximum_correction_translation_rate_mps * elapsed_seconds;
    if (translation_delta > maximum_translation_step &&
        translation_delta > 0.0) {
      correction.head<3>() *= maximum_translation_step / translation_delta;
    }
    const double maximum_yaw_step =
        config_.maximum_correction_yaw_rate_radps * elapsed_seconds;
    correction[3] =
        std::clamp(correction[3], -maximum_yaw_step, maximum_yaw_step);
    const Eigen::Vector3d published_position =
        BodyPosition(published_alignment_, odom_position) +
        correction.head<3>();
    published_alignment_[3] =
        WrappedAngle(published_alignment_[3] + correction[3]);
    published_alignment_.head<3>() =
        published_position -
        Eigen::AngleAxisd(published_alignment_[3], Eigen::Vector3d::UnitZ()) *
            odom_position;
    publication_state_ = CorrectionPublicationState::kRateLimited;
    last_published_timestamp_ns_ = timestamp_ns;
    pending_large_reset_count_ = 0;
    return Status::Ok();
  }

  bool CorrectionSourceFresh() const {
    return has_admitted_fix_ &&
           latest_odometry_timestamp_ns_ >= last_admitted_fix_.nanoseconds &&
           latest_odometry_timestamp_ns_ - last_admitted_fix_.nanoseconds <=
               config_.maximum_fixed_solution_gap_ns;
  }

  Status EstimateInOdometryFrame(internal::AlignmentEstimate* estimate) const {
    Status status = smoother_->Estimate(estimate);
    if (!status.ok()) {
      return status;
    }
    const double yaw = estimate->map_from_odom[3];
    const Eigen::Vector3d rotated_reference =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        alignment_reference_in_odom_;
    estimate->map_from_odom.head<3>() -= rotated_reference;
    Eigen::Matrix4d jacobian = Eigen::Matrix4d::Identity();
    jacobian(0, 3) = rotated_reference.y();
    jacobian(1, 3) = -rotated_reference.x();
    estimate->covariance =
        jacobian * estimate->covariance * jacobian.transpose();
    return Status::Ok();
  }

  void AddRejectionRecord(const PositionFix& fix, uint64_t state_id,
                          const std::string& reason,
                          double processing_latency_ms = 0.0) {
    FactorRecord record;
    record.type = fix.horizontal_only() ? "ABS_POS2" : "ABS_POS3";
    record.timestamp_ns = fix.timestamp.nanoseconds;
    record.innovation_threshold =
        fix.horizontal_only() ? config_.horizontal_innovation_gate_threshold
                              : config_.innovation_gate_threshold;
    record.accepted = false;
    record.reason = reason;
    if (fix.covariance.allFinite()) {
      record.covariance_diagonal = fix.covariance.diagonal();
    }
    record.state_id = state_id;
    record.processing_latency_ms = processing_latency_ms;
    if (reason != "pending_odometry_bracket") {
      pending_large_reset_count_ = 0;
    }
    AppendRecord(std::move(record));
  }

  void AddHeadingRejectionRecord(const GnssHeading& heading,
                                 const std::string& reason,
                                 double processing_latency_ms = 0.0) {
    FactorRecord record;
    record.timestamp_ns = heading.timestamp.nanoseconds;
    record.source = "gnss_heading";
    record.type = "ABS_YAW1";
    record.innovation_threshold = config_.heading_innovation_gate_threshold;
    record.accepted = false;
    record.reason = reason;
    if (std::isfinite(heading.accuracy_rad) && heading.accuracy_rad > 0.0) {
      record.covariance_diagonal = Eigen::VectorXd::Constant(
          1, heading.accuracy_rad * heading.accuracy_rad);
    }
    record.processing_latency_ms = processing_latency_ms;
    if (reason != "pending_odometry_bracket") {
      pending_large_reset_count_ = 0;
    }
    AppendRecord(std::move(record));
  }

  void AppendRecord(FactorRecord record) {
    if (record.reason != "pending_odometry_bracket") {
      const auto pending = std::find_if(
          factor_records_.rbegin(), factor_records_.rend(),
          [&record](const FactorRecord& existing) {
            const bool same_source =
                existing.source == record.source ||
                (existing.source == "gnss" &&
                 record.source == "gnss_single_antenna_bootstrap" &&
                 (existing.type == "ABS_POS2" || existing.type == "ABS_POS3"));
            return existing.timestamp_ns == record.timestamp_ns &&
                   same_source && existing.reason == "pending_odometry_bracket";
          });
      if (pending != factor_records_.rend()) {
        *pending = std::move(record);
        result_.factor_records = factor_records_;
        return;
      }
    }
    factor_records_.push_back(std::move(record));
    while (factor_records_.size() > config_.maximum_factor_records) {
      factor_records_.erase(factor_records_.begin());
    }
    result_.factor_records = factor_records_;
  }

  void PruneSegments() {
    const int64_t lag_ns = static_cast<int64_t>(config_.alignment_lag_seconds *
                                                kNanosecondsPerSecond);
    const int64_t cutoff_ns = latest_odometry_timestamp_ns_ > lag_ns
                                  ? latest_odometry_timestamp_ns_ - lag_ns
                                  : 0;
    while (segments_.size() > 1 && segments_[1].timestamp_ns < cutoff_ns) {
      segments_.erase(segments_.begin());
    }
  }

  Status RefreshEstimate() {
    if (!has_odometry_ || segments_.empty()) {
      return Status(StatusCode::kStateUnavailable,
                    "alignment estimate is not initialized");
    }
    internal::AlignmentEstimate estimate;
    Status status = EstimateInOdometryFrame(&estimate);
    if (!status.ok()) {
      return status;
    }
    if (!estimate.map_from_odom.allFinite() ||
        !estimate.covariance.allFinite()) {
      return Status(StatusCode::kOptimizationFailed,
                    "alignment estimate or covariance is not finite");
    }
    result_.timestamp = Timestamp{latest_odometry_timestamp_ns_,
                                  previous_odometry_.timestamp.clock_domain};
    result_.odometry_epoch = previous_odometry_.epoch;
    result_.map_reference = config_.map_reference;
    result_.calibration_state = config_.map_reference.geodetic.state;
    result_.odom_gravity_contract = config_.odom_gravity_contract;
    result_.observability.horizontal_position = has_admitted_fix_;
    result_.observability.vertical_position = has_admitted_vertical_fix_;
    result_.observability.heading = heading_initialized_;
    result_.map_from_odom = published_alignment_;
    result_.backend_map_from_odom = estimate.map_from_odom;
    result_.alignment_covariance = estimate.covariance;
    result_.alignment_reference_in_odom = alignment_reference_in_odom_;
    result_.large_correction_policy = config_.large_correction_policy;
    result_.correction_source_fresh = CorrectionSourceFresh();
    result_.publication_pose_offset =
        PoseDifference(estimate.map_from_odom, published_alignment_,
                       previous_odometry_.pose.translation);
    result_.published_mean_matches_backend =
        has_published_alignment_ &&
        result_.publication_pose_offset.head<3>().norm() < 1e-9 &&
        std::abs(result_.publication_pose_offset[3]) < 1e-9;
    result_.publication_state =
        result_.published_mean_matches_backend
            ? CorrectionPublicationState::kAligned
            : (has_published_alignment_ &&
                       publication_state_ ==
                           CorrectionPublicationState::kAligned
                   ? CorrectionPublicationState::kAwaitingPolicyUpdate
                   : publication_state_);
    result_.correction_pending =
        has_published_alignment_ && !result_.published_mean_matches_backend;
    result_.global_output_degraded = !has_admitted_fix_ ||
                                     result_.correction_pending ||
                                     !result_.correction_source_fresh;
    result_.alignment_revision = alignment_revision_;
    result_.global_alignment_valid = has_admitted_fix_;
    result_.vertical_position_initialized = has_admitted_vertical_fix_;
    result_.reset_counter = reset_counter_;
    result_.heading_initialized = heading_initialized_;
    result_.factor_records = factor_records_;
    return Status::Ok();
  }

  LocalizationConfig config_;
  std::unique_ptr<OdometryBuffer> odometry_buffer_;
  std::unique_ptr<internal::AlignmentSmoother> smoother_;
  internal::GnssAlignmentRecovery recovery_;
  internal::GnssQualityGate quality_gate_;
  std::vector<Segment> segments_;
  std::deque<PositionFix> pending_fixes_;
  std::deque<GnssHeading> pending_headings_;
  std::vector<FactorRecord> factor_records_;
  LocalizationResult result_;
  OdometrySample previous_odometry_;
  PositionFix last_good_fix_;
  Timestamp last_submitted_heading_;
  Timestamp last_admitted_heading_;
  Timestamp last_submitted_fix_;
  Timestamp last_admitted_fix_;
  Eigen::Vector4d published_alignment_ = Eigen::Vector4d::Zero();
  Eigen::Vector3d alignment_reference_in_odom_ = Eigen::Vector3d::Zero();
  Eigen::Vector4d pending_large_reset_alignment_ = Eigen::Vector4d::Zero();
  int64_t latest_odometry_timestamp_ns_ = 0;
  int64_t last_published_timestamp_ns_ = 0;
  double distance_since_segment_m_ = 0.0;
  double total_travel_distance_m_ = 0.0;
  double instantaneous_speed_mps_ = 0.0;
  uint32_t reset_counter_ = 0;
  uint32_t pending_large_reset_count_ = 0;
  uint64_t alignment_revision_ = 0;
  CorrectionPublicationState publication_state_ =
      CorrectionPublicationState::kUninitialized;
  bool configured_ = false;
  bool has_odometry_ = false;
  bool has_submitted_fix_ = false;
  bool has_submitted_heading_ = false;
  bool has_previous_quality_fix_ = false;
  bool has_admitted_fix_ = false;
  bool has_admitted_vertical_fix_ = false;
  bool has_admitted_heading_ = false;
  bool has_published_alignment_ = false;
  bool heading_initialized_ = false;
};

LocalizationEngine::LocalizationEngine() : impl_(std::make_unique<Impl>()) {}

LocalizationEngine::~LocalizationEngine() = default;

Status LocalizationEngine::Configure(const LocalizationConfig& config) {
  return impl_->Configure(config);
}

Status LocalizationEngine::AddOdometry(const OdometrySample& sample) {
  return impl_->AddOdometry(sample);
}

Status LocalizationEngine::SubmitGnssFix(const GnssFix& fix) {
  return impl_->SubmitGnssFix(fix);
}

Status LocalizationEngine::SubmitGnssHorizontalFix(
    const GnssHorizontalFix& fix) {
  return impl_->SubmitGnssHorizontalFix(fix);
}

Status LocalizationEngine::SubmitGnssHeading(const GnssHeading& heading) {
  return impl_->SubmitGnssHeading(heading);
}

Status LocalizationEngine::GetResult(LocalizationResult* result) const {
  return impl_->GetResult(result);
}

Status LocalizationEngine::Reset() { return impl_->Reset(); }

}  // namespace localization
}  // namespace openloc
