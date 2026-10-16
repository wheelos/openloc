#include "alignment_recovery.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>

namespace openloc {
namespace localization {
namespace internal {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

Status Invalid(const char* message) {
  return Status(StatusCode::kInvalidArgument, message);
}

}  // namespace

GnssAlignmentRecovery::GnssAlignmentRecovery(GnssAlignmentRecoveryConfig config)
    : config_(std::move(config)) {}

void GnssAlignmentRecovery::Reset() {
  training_.clear();
  validation_.clear();
  candidate_ = GnssAlignmentRecoveryCandidate{};
  last_timestamp_ns_ = 0;
  has_candidate_ = false;
}

void GnssAlignmentRecovery::StartTrainingWindow(
    const GnssAlignmentRecoveryObservation& observation) {
  training_.clear();
  validation_.clear();
  training_.push_back(observation);
  has_candidate_ = false;
}

Status GnssAlignmentRecovery::AddRejectedObservation(
    const GnssAlignmentRecoveryObservation& observation,
    GnssAlignmentRecoveryDiagnostics* diagnostics,
    GnssAlignmentRecoveryCandidate* candidate) {
  if (diagnostics == nullptr || candidate == nullptr) {
    return Invalid("GNSS recovery outputs must not be null");
  }
  *diagnostics = GnssAlignmentRecoveryDiagnostics{};
  if (observation.timestamp_ns <= 0 ||
      observation.timestamp_ns <= last_timestamp_ns_ ||
      !observation.odom_antenna_xy.allFinite() ||
      !observation.global_antenna_xy.allFinite() ||
      !observation.covariance_xy.allFinite() ||
      !observation.covariance_xy.isApprox(observation.covariance_xy.transpose(),
                                          1e-10)) {
    return Invalid("GNSS recovery observation is invalid or out of order");
  }
  Eigen::LDLT<Eigen::Matrix2d> covariance_decomposition(
      observation.covariance_xy);
  if (covariance_decomposition.info() != Eigen::Success ||
      (covariance_decomposition.vectorD().array() <= 0.0).any()) {
    return Invalid("GNSS recovery covariance must be positive definite");
  }
  if (config_.training_observations < 3 ||
      config_.validation_observations == 0 || config_.maximum_window_ns <= 0 ||
      !std::isfinite(config_.minimum_odom_baseline_m) ||
      config_.minimum_odom_baseline_m <= 0.0 ||
      !std::isfinite(config_.maximum_training_rms_m) ||
      config_.maximum_training_rms_m <= 0.0 ||
      !std::isfinite(config_.huber_delta) || config_.huber_delta <= 0.0 ||
      !std::isfinite(config_.innovation_gate_threshold) ||
      config_.innovation_gate_threshold <= 0.0 ||
      !std::isfinite(config_.minimum_alignment_sigma_m) ||
      config_.minimum_alignment_sigma_m <= 0.0 ||
      !std::isfinite(config_.minimum_yaw_sigma_rad) ||
      config_.minimum_yaw_sigma_rad <= 0.0) {
    return Invalid("GNSS recovery configuration is invalid");
  }
  last_timestamp_ns_ = observation.timestamp_ns;
  if (!training_.empty() &&
      observation.timestamp_ns - training_.front().timestamp_ns >
          config_.maximum_window_ns) {
    StartTrainingWindow(observation);
    diagnostics->training_count = training_.size();
    diagnostics->reason = "recovery_window_timeout";
    return Status::Ok();
  }

  if (has_candidate_) {
    const double nis = ValidationNis(observation, candidate_);
    diagnostics->validation_nis = nis;
    diagnostics->training_rms_m = candidate_.training_rms_m;
    diagnostics->training_count = candidate_.training_count;
    if (!std::isfinite(nis) || nis > config_.innovation_gate_threshold) {
      StartTrainingWindow(observation);
      diagnostics->reason = "recovery_holdout_inconsistent";
      return Status::Ok();
    }
    validation_.push_back(observation);
    diagnostics->validation_count = validation_.size();
    diagnostics->reason = "recovery_holdout_consistent";
    if (validation_.size() >= config_.validation_observations) {
      candidate_.validation_count = validation_.size();
      *candidate = candidate_;
      diagnostics->candidate_validated = true;
      diagnostics->reason = "recovery_candidate_validated";
    }
    return Status::Ok();
  }

  training_.push_back(observation);
  while (training_.size() > config_.training_observations) {
    training_.pop_front();
  }
  diagnostics->training_count = training_.size();
  if (training_.size() < config_.training_observations) {
    diagnostics->reason = "recovery_training_window";
    return Status::Ok();
  }
  std::string fit_reason;
  if (!FitCandidate(&candidate_, &fit_reason)) {
    diagnostics->reason = std::move(fit_reason);
    return Status::Ok();
  }
  has_candidate_ = true;
  diagnostics->training_rms_m = candidate_.training_rms_m;
  diagnostics->training_count = candidate_.training_count;
  diagnostics->reason = "recovery_candidate_waiting_for_holdout";
  return Status::Ok();
}

bool GnssAlignmentRecovery::FitCandidate(
    GnssAlignmentRecoveryCandidate* candidate, std::string* reason) const {
  Eigen::Vector2d odom_mean = Eigen::Vector2d::Zero();
  Eigen::Vector2d global_mean = Eigen::Vector2d::Zero();
  double maximum_baseline = 0.0;
  for (std::size_t first = 0; first < training_.size(); ++first) {
    odom_mean += training_[first].odom_antenna_xy;
    global_mean += training_[first].global_antenna_xy;
    for (std::size_t second = first + 1; second < training_.size(); ++second) {
      maximum_baseline = std::max(
          maximum_baseline,
          (training_[second].odom_antenna_xy - training_[first].odom_antenna_xy)
              .norm());
    }
  }
  odom_mean /= static_cast<double>(training_.size());
  global_mean /= static_cast<double>(training_.size());
  if (maximum_baseline < config_.minimum_odom_baseline_m) {
    *reason = "recovery_yaw_unobservable";
    return false;
  }

  std::vector<double> weights(training_.size(), 1.0);
  double yaw = 0.0;
  Eigen::Vector2d translation = Eigen::Vector2d::Zero();
  for (int iteration = 0; iteration < 5; ++iteration) {
    double dot = 0.0;
    double cross = 0.0;
    double weight_sum = 0.0;
    odom_mean.setZero();
    global_mean.setZero();
    for (std::size_t index = 0; index < training_.size(); ++index) {
      const double weight = weights[index];
      weight_sum += weight;
      odom_mean += weight * training_[index].odom_antenna_xy;
      global_mean += weight * training_[index].global_antenna_xy;
    }
    odom_mean /= weight_sum;
    global_mean /= weight_sum;
    for (std::size_t index = 0; index < training_.size(); ++index) {
      const Eigen::Vector2d local =
          training_[index].odom_antenna_xy - odom_mean;
      const Eigen::Vector2d global =
          training_[index].global_antenna_xy - global_mean;
      dot += weights[index] * local.dot(global);
      cross +=
          weights[index] * (local.x() * global.y() - local.y() * global.x());
    }
    if (std::hypot(dot, cross) <= std::numeric_limits<double>::epsilon()) {
      *reason = "recovery_yaw_unobservable";
      return false;
    }
    yaw = std::atan2(cross, dot);
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    Eigen::Matrix2d rotation;
    rotation << cosine, -sine, sine, cosine;
    translation = global_mean - rotation * odom_mean;
    for (std::size_t index = 0; index < training_.size(); ++index) {
      const Eigen::Vector2d residual =
          rotation * training_[index].odom_antenna_xy + translation -
          training_[index].global_antenna_xy;
      Eigen::LDLT<Eigen::Matrix2d> decomposition(
          training_[index].covariance_xy);
      const double nis = residual.dot(decomposition.solve(residual));
      const double normalized_error = std::sqrt(std::max(nis, 0.0));
      weights[index] = normalized_error <= config_.huber_delta
                           ? 1.0
                           : config_.huber_delta / normalized_error;
    }
  }

  double squared_error = 0.0;
  double maximum_residual_nis = 0.0;
  double mean_variance = 0.0;
  for (const auto& observation : training_) {
    Eigen::Matrix2d rotation;
    rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
    const Eigen::Vector2d residual = rotation * observation.odom_antenna_xy +
                                     translation -
                                     observation.global_antenna_xy;
    Eigen::LDLT<Eigen::Matrix2d> decomposition(observation.covariance_xy);
    maximum_residual_nis = std::max(
        maximum_residual_nis, residual.dot(decomposition.solve(residual)));
    squared_error += residual.squaredNorm();
    mean_variance += observation.covariance_xy.trace() / 2.0;
  }
  const double rms = std::sqrt(squared_error / training_.size());
  if (!std::isfinite(rms) || rms > config_.maximum_training_rms_m ||
      !std::isfinite(maximum_residual_nis) ||
      maximum_residual_nis > config_.innovation_gate_threshold * 4.0) {
    *reason = "recovery_training_inconsistent";
    return false;
  }

  candidate->global_from_odom_xy_yaw << translation.x(), translation.y(), yaw;
  candidate->odom_reference_xy = odom_mean;
  const double sigma = std::max(config_.minimum_alignment_sigma_m,
                                std::sqrt(mean_variance / training_.size() +
                                          rms * rms / training_.size()));
  candidate->sigma_xy_yaw << sigma, sigma,
      std::max(config_.minimum_yaw_sigma_rad,
               sigma / std::max(maximum_baseline, 1.0));
  candidate->training_rms_m = rms;
  candidate->training_count = training_.size();
  candidate->validation_count = 0;
  return true;
}

double GnssAlignmentRecovery::ValidationNis(
    const GnssAlignmentRecoveryObservation& observation,
    const GnssAlignmentRecoveryCandidate& candidate) const {
  const double yaw = candidate.global_from_odom_xy_yaw[2];
  Eigen::Matrix2d rotation;
  rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  const Eigen::Vector2d translation =
      candidate.global_from_odom_xy_yaw.head<2>();
  const Eigen::Vector2d residual = rotation * observation.odom_antenna_xy +
                                   translation - observation.global_antenna_xy;
  const double radius =
      (observation.odom_antenna_xy - candidate.odom_reference_xy).norm();
  const double candidate_variance =
      candidate.sigma_xy_yaw.head<2>().squaredNorm() / 2.0 +
      radius * radius * candidate.sigma_xy_yaw[2] * candidate.sigma_xy_yaw[2];
  const Eigen::Matrix2d innovation_covariance =
      observation.covariance_xy +
      Eigen::Matrix2d::Identity() * candidate_variance;
  Eigen::LDLT<Eigen::Matrix2d> decomposition(innovation_covariance);
  if (decomposition.info() != Eigen::Success ||
      (decomposition.vectorD().array() <= 0.0).any()) {
    return std::numeric_limits<double>::infinity();
  }
  return residual.dot(decomposition.solve(residual));
}

}  // namespace internal
}  // namespace localization
}  // namespace openloc
