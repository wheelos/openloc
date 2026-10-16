#ifndef ONLINE_LOCALIZATION_INCLUDE_ONLINE_LOCALIZATION_LOCALIZATION_ENGINE_H_
#define ONLINE_LOCALIZATION_INCLUDE_ONLINE_LOCALIZATION_LOCALIZATION_ENGINE_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "openloc/localization/online/odometry_buffer.h"

namespace openloc {
namespace localization {

enum class GnssCarrierSolution {
  kInvalid,
  kFloat,
  kFixed,
};

struct GnssReceiverQuality {
  GnssCarrierSolution carrier_solution = GnssCarrierSolution::kInvalid;
  uint32_t satellite_count = 0;
  double hdop = 0.0;
  double differential_age_seconds = 0.0;
  double horizontal_accuracy_m = 0.0;
};

struct GnssFix {
  Timestamp timestamp;
  FrameId frame;
  MapReference map_reference;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
  GnssReceiverQuality receiver;
};

/// Antenna XY in the configured gravity-level global frame. No height value
/// or synthetic height variance; receiver admission is identical to GnssFix.
struct GnssHorizontalFix {
  Timestamp timestamp;
  FrameId frame;
  MapReference map_reference;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Identity();
  GnssReceiverQuality receiver;
};

/// Dual-antenna baseline heading in map-frame ENU, radians CCW from +X.
struct GnssHeading {
  Timestamp timestamp;
  FrameId frame;
  MapReference map_reference;
  double azimuth_rad = 0.0;
  double accuracy_rad = 0.0;
  double measured_baseline_length_m = 0.0;
  GnssReceiverQuality receiver;
};

enum class LargeCorrectionPolicy {
  kHoldForStoppedReset,
  /// Explicit diagnostic opt-in; follows a fresh backend estimate continuously
  /// under configured position/yaw rate bounds, without a large reset.
  kRateLimitedTracking,
};

struct LocalizationConfig {
  /// Name of the configured global frame (for example, a local ENU frame).
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";
  MapReference map_reference;
  Eigen::Vector3d gnss_antenna_in_base = Eigen::Vector3d::Zero();
  /// Optional heading baseline; required only when submitting GNSS headings.
  /// Vector from the primary antenna to the secondary antenna, in base_frame.
  Eigen::Vector3d gnss_baseline_in_base = Eigen::Vector3d::Zero();
  OdometryBufferConfig odometry_buffer;
  double alignment_lag_seconds = 10.0;
  double alignment_segment_distance_m = 15.0;
  int64_t alignment_segment_max_duration_ns = 5'000'000'000LL;
  /// Translation/yaw parameters at the fixed first-ODOM XY reference, not the
  /// caller's potentially distant ODOM origin. Z retains its original origin.
  Eigen::Vector4d initial_alignment_sigma =
      Eigen::Vector4d(1000.0, 1000.0, 100.0, 3.141592653589793);
  /// Diagonal process noise in the same reference-point parameterization.
  Eigen::Vector4d alignment_random_walk_sigma_per_sqrt_meter =
      Eigen::Vector4d(0.02, 0.02, 0.03, 0.005);
  double innovation_gate_threshold = 11.344866730144373;
  double horizontal_innovation_gate_threshold = 9.210340371976184;
  double heading_innovation_gate_threshold = 6.6348966010212145;
  /// Experimental global-only re-acquisition; disabled unless explicitly set.
  bool enable_gnss_reacquisition = false;
  uint32_t minimum_satellites = 12;
  double maximum_hdop = 1.5;
  double maximum_differential_age_seconds = 3.0;
  double maximum_horizontal_accuracy_m = 0.3;
  double maximum_heading_accuracy_rad = 0.03490658503988659;
  double maximum_baseline_length_error_m = 0.02;
  int64_t fixed_solution_stability_ns = 3'000'000'000LL;
  int64_t maximum_fixed_solution_gap_ns = 1'500'000'000LL;
  int64_t minimum_gnss_admission_interval_ns = 500'000'000LL;
  int64_t minimum_heading_admission_interval_ns = 1'000'000'000LL;
  double cross_check_absolute_distance_error_m = 0.1;
  double cross_check_relative_distance_error = 0.02;
  double cross_check_direction_threshold_rad = 0.05235987755982989;
  double cross_check_minimum_speed_mps = 2.0;
  LargeCorrectionPolicy large_correction_policy =
      LargeCorrectionPolicy::kHoldForStoppedReset;
  double direct_correction_translation_m = 0.3;
  double direct_correction_yaw_rad = 0.017453292519943295;
  double large_reset_translation_m = 1.5;
  double maximum_correction_translation_rate_mps = 0.3;
  double maximum_correction_yaw_rate_radps = 0.017453292519943295;
  double large_reset_maximum_speed_mps = 0.2;
  uint32_t large_reset_consistent_observations = 5;
  std::size_t maximum_pending_fixes = 128;
  std::size_t maximum_factor_records = 1000;
};

struct FactorRecord {
  int64_t timestamp_ns = 0;
  std::string source = "gnss";
  std::string type = "ABS_POS3";
  Eigen::VectorXd residual;
  double normalized_innovation_squared = 0.0;
  double innovation_threshold = 0.0;
  double robust_weight = 0.0;
  bool accepted = false;
  std::string reason;
  Eigen::VectorXd covariance_diagonal;
  uint64_t state_id = 0;
  double processing_latency_ms = 0.0;
};

enum class CorrectionPublicationState {
  kUninitialized,
  kAligned,
  kRateLimited,
  kHeldForReset,
  /// Backend changed during ODOM processing without an accepted absolute
  /// observation authorizing another publication-policy step.
  kAwaitingPolicyUpdate,
};

struct LocalizationResult {
  Timestamp timestamp;
  OdometryEpoch odometry_epoch;
  /// Identity of the configured global-frame origin and conventions.
  MapReference map_reference;
  /// Configured global-frame-from-odom transform; the global frame may be ENU.
  Eigen::Vector4d map_from_odom = Eigen::Vector4d::Zero();
  Eigen::Vector4d backend_map_from_odom = Eigen::Vector4d::Zero();
  /// Marginal covariance of backend_map_from_odom, before correction limiting.
  /// Not a covariance for a held/rate-limited published mean.
  Eigen::Matrix4d alignment_covariance = Eigen::Matrix4d::Zero();
  /// Internal alignment priors/process noise are anchored at this fixed first
  /// ODOM XY position; caller ODOM and returned transforms retain original
  /// axes.
  Eigen::Vector3d alignment_reference_in_odom = Eigen::Vector3d::Zero();
  CorrectionPublicationState publication_state =
      CorrectionPublicationState::kUninitialized;
  /// Backend minus published current body position (global XYZ) and yaw.
  /// Deterministic offset, not additional stochastic covariance.
  Eigen::Vector4d publication_pose_offset = Eigen::Vector4d::Zero();
  bool correction_pending = false;
  /// Transition/uninitialized output, not a calibrated integrity assessment.
  bool global_output_degraded = true;
  /// A position factor was accepted within maximum_fixed_solution_gap_ns of
  /// current ODOM sensor time. Arrival recency cannot renew stale evidence.
  bool correction_source_fresh = false;
  LargeCorrectionPolicy large_correction_policy =
      LargeCorrectionPolicy::kHoldForStoppedReset;
  /// Mean agreement only; does not establish calibrated uncertainty.
  bool published_mean_matches_backend = false;
  /// Increments on global backend re-anchor, even if publication is held.
  /// Restarts on explicit engine Reset.
  uint64_t alignment_revision = 0;
  /// True after an accepted absolute GNSS position initializes at least XY.
  /// A heading-only observation is insufficient; validity persists during
  /// GNSS outages while covariance grows. Not an integrity/accuracy flag;
  /// check publication_state and correction_pending separately.
  bool global_alignment_valid = false;
  /// An accepted XYZ fix has constrained Z; XY does not set this flag.
  /// Neither this flag nor covariance establishes calibrated vertical accuracy.
  bool vertical_position_initialized = false;
  /// Published discontinuities and explicit Reset(), not backend re-anchors.
  uint32_t reset_counter = 0;
  /// Backend dual-antenna heading initialization, not publication agreement.
  bool heading_initialized = false;
  /// Sticky until Reset; compare reset_counter to detect a new event.
  bool reset_event = false;
  std::vector<FactorRecord> factor_records;
};

/// Stateful middleware-neutral P1 engine; callers must serialize all methods.
class LocalizationEngine {
 public:
  /// Creates an unconfigured localization engine.
  LocalizationEngine();

  /// Releases the private estimator backend.
  ~LocalizationEngine();

  LocalizationEngine(const LocalizationEngine&) = delete;
  LocalizationEngine& operator=(const LocalizationEngine&) = delete;

  /// Validates and installs immutable runtime configuration.
  Status Configure(const LocalizationConfig& config);

  /// Adds a strictly ordered odometry sample and processes bracketed fixes.
  Status AddOdometry(const OdometrySample& sample);

  /// Submits a timestamped GNSS fix; returns kPending until odometry brackets
  /// it.
  Status SubmitGnssFix(const GnssFix& fix);

  /// Same time/quality/identity/ordering contract as SubmitGnssFix, but only
  /// XY is measured. XYZ and XY share one admission stream: do not submit both
  /// representations of the same receiver measurement.
  Status SubmitGnssHorizontalFix(const GnssHorizontalFix& fix);

  /// Submits a dual-antenna map-frame baseline heading at sensor time.
  Status SubmitGnssHeading(const GnssHeading& heading);

  /// Copies the current localization estimate and bounded factor history.
  Status GetResult(LocalizationResult* result) const;

  /// Clears runtime state and increments the reset sequence.
  Status Reset();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace localization
}  // namespace openloc

#endif  // ONLINE_LOCALIZATION_INCLUDE_ONLINE_LOCALIZATION_LOCALIZATION_ENGINE_H_
