#ifndef OPENLOC_LOCALIZATION_ONLINE_ALIGNMENT_SMOOTHER_H_
#define OPENLOC_LOCALIZATION_ONLINE_ALIGNMENT_SMOOTHER_H_

#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>

#include "openloc/localization/online/types.h"

namespace openloc {
namespace localization {
namespace internal {

struct AlignmentSmootherConfig {
  double lag_seconds = 10.0;
  Eigen::Vector4d initial_alignment = Eigen::Vector4d::Zero();
  Eigen::Vector4d initial_sigma = Eigen::Vector4d(0.05, 0.05, 0.1, 0.05);
  Eigen::Vector4d random_walk_sigma_per_sqrt_meter =
      Eigen::Vector4d(0.02, 0.02, 0.03, 0.005);
  double minimum_distance_delta_m = 0.01;
  double huber_delta = 1.345;
  double innovation_gate_threshold = 11.344866730144373;
  double horizontal_innovation_gate_threshold = 9.210340371976184;
  double heading_innovation_gate_threshold = 6.6348966010212145;
};

struct AlignmentEstimate {
  int64_t timestamp_ns = 0;
  Eigen::Vector4d map_from_odom = Eigen::Vector4d::Zero();
  Eigen::Matrix4d covariance = Eigen::Matrix4d::Zero();
};

struct GnssFactorDiagnostics {
  Eigen::VectorXd residual = Eigen::Vector3d::Zero();
  double normalized_innovation_squared = 0.0;
  bool horizontal_innovation_rejected = false;
  double robust_weight = 0.0;
};

struct HeadingFactorDiagnostics {
  double residual_rad = 0.0;
  double normalized_innovation_squared = 0.0;
  double robust_weight = 0.0;
};

/// Private GTSAM fixed-lag backend for the four-DoF map-to-odom alignment.
class AlignmentSmoother {
 public:
  /// Creates a smoother with a validated fixed-lag/noise configuration.
  explicit AlignmentSmoother(AlignmentSmootherConfig config = {});

  /// Releases the GTSAM smoother state.
  ~AlignmentSmoother();

  /// Disables copying because the backend owns mutable optimizer state.
  AlignmentSmoother(const AlignmentSmoother&) = delete;

  /// Disables copying because the backend owns mutable optimizer state.
  AlignmentSmoother& operator=(const AlignmentSmoother&) = delete;

  /// Adds a monotonically timed alignment state and returns its opaque ID.
  Status AddState(int64_t timestamp_ns, double distance_delta_m,
                  uint64_t* state_id);

  /// Replaces only the global alignment history with a validated candidate.
  Status Reanchor(int64_t timestamp_ns, const Eigen::Vector4d& alignment,
                  const Eigen::Vector4d& prior_sigma, uint64_t* state_id);

  /// Gates and adds a GNSS factor to its timestamp-selected alignment state.
  Status AddGnssPosition(uint64_t state_id, int64_t timestamp_ns,
                         const Eigen::Vector3d& antenna_in_odom,
                         const Eigen::Vector3d& antenna_position_enu,
                         const Eigen::Matrix3d& covariance_enu,
                         GnssFactorDiagnostics* diagnostics,
                         bool initializing = false);

  /// Adds only measured XY; Z remains unobserved. Same frame assumptions as
  /// XYZ.
  Status AddGnssHorizontalPosition(uint64_t state_id, int64_t timestamp_ns,
                                   const Eigen::Vector3d& antenna_in_odom,
                                   const Eigen::Vector2d& antenna_position_xy,
                                   const Eigen::Matrix2d& covariance_xy,
                                   GnssFactorDiagnostics* diagnostics,
                                   bool initializing = false);

  /// Gates and adds a dual-antenna heading factor to an active alignment state.
  Status AddGnssHeading(uint64_t state_id, int64_t timestamp_ns,
                        double odom_base_yaw_rad,
                        double baseline_yaw_in_base_rad,
                        double heading_azimuth_map_rad,
                        double heading_variance_rad2,
                        HeadingFactorDiagnostics* diagnostics,
                        bool initializing = false);

  /// Returns the newest alignment mean and marginal covariance.
  Status Estimate(AlignmentEstimate* estimate) const;

 private:
  Status ValidateConfig() const;
  Status CheckAvailable() const;
  double RelativeSeconds(int64_t timestamp_ns) const;
  Status AddGnssPositionImpl(uint64_t state_id, int64_t timestamp_ns,
                             const Eigen::Vector3d& antenna_in_odom,
                             const Eigen::VectorXd& position,
                             const Eigen::MatrixXd& covariance,
                             GnssFactorDiagnostics* diagnostics,
                             bool initializing);

  AlignmentSmootherConfig config_;
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  int64_t origin_timestamp_ns_ = 0;
  int64_t latest_timestamp_ns_ = 0;
  int64_t last_reanchor_timestamp_ns_ = 0;
  uint64_t state_index_ = 0;
  bool has_ever_state_ = false;
  gtsam::Key latest_key_ = 0;
  Eigen::Vector4d latest_estimate_ = Eigen::Vector4d::Zero();
  std::map<uint64_t, int64_t> state_timestamps_;
  std::string failure_message_;
};

}  // namespace internal
}  // namespace localization
}  // namespace openloc

#endif  // OPENLOC_LOCALIZATION_ONLINE_ALIGNMENT_SMOOTHER_H_
