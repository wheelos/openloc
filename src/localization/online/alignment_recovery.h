#ifndef OPENLOC_LOCALIZATION_ONLINE_ALIGNMENT_RECOVERY_H_
#define OPENLOC_LOCALIZATION_ONLINE_ALIGNMENT_RECOVERY_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

#include <Eigen/Core>

#include "openloc/localization/online/types.h"

namespace openloc {
namespace localization {
namespace internal {

struct GnssAlignmentRecoveryConfig {
  std::size_t training_observations = 8;
  std::size_t validation_observations = 3;
  /// Maximum total sensor-time age of training plus subsequent holdouts.
  int64_t maximum_window_ns = 30'000'000'000LL;
  double minimum_odom_baseline_m = 5.0;
  double maximum_training_rms_m = 2.0;
  double huber_delta = 1.345;
  double innovation_gate_threshold = 9.210340371976184;
  double minimum_alignment_sigma_m = 0.1;
  double minimum_yaw_sigma_rad = 0.01;
};

struct GnssAlignmentRecoveryObservation {
  int64_t timestamp_ns = 0;
  Eigen::Vector2d odom_antenna_xy = Eigen::Vector2d::Zero();
  Eigen::Vector2d global_antenna_xy = Eigen::Vector2d::Zero();
  Eigen::Matrix2d covariance_xy = Eigen::Matrix2d::Identity();
};

struct GnssAlignmentRecoveryCandidate {
  Eigen::Vector3d global_from_odom_xy_yaw = Eigen::Vector3d::Zero();
  Eigen::Vector3d sigma_xy_yaw = Eigen::Vector3d::Ones();
  Eigen::Vector2d odom_reference_xy = Eigen::Vector2d::Zero();
  double training_rms_m = 0.0;
  std::size_t training_count = 0;
  std::size_t validation_count = 0;
};

struct GnssAlignmentRecoveryDiagnostics {
  bool candidate_validated = false;
  double training_rms_m = 0.0;
  double validation_nis = 0.0;
  std::size_t training_count = 0;
  std::size_t validation_count = 0;
  std::string reason;
};

/// Shadow estimator for global XY/yaw recovery; it never modifies local ODOM.
class GnssAlignmentRecovery {
 public:
  explicit GnssAlignmentRecovery(GnssAlignmentRecoveryConfig config = {});

  Status AddRejectedObservation(
      const GnssAlignmentRecoveryObservation& observation,
      GnssAlignmentRecoveryDiagnostics* diagnostics,
      GnssAlignmentRecoveryCandidate* candidate);

  void Reset();

 private:
  bool FitCandidate(GnssAlignmentRecoveryCandidate* candidate,
                    std::string* reason) const;
  double ValidationNis(const GnssAlignmentRecoveryObservation& observation,
                       const GnssAlignmentRecoveryCandidate& candidate) const;
  void StartTrainingWindow(const GnssAlignmentRecoveryObservation& observation);

  GnssAlignmentRecoveryConfig config_;
  std::deque<GnssAlignmentRecoveryObservation> training_;
  std::deque<GnssAlignmentRecoveryObservation> validation_;
  GnssAlignmentRecoveryCandidate candidate_;
  int64_t last_timestamp_ns_ = 0;
  bool has_candidate_ = false;
};

}  // namespace internal
}  // namespace localization
}  // namespace openloc

#endif  // OPENLOC_LOCALIZATION_ONLINE_ALIGNMENT_RECOVERY_H_
