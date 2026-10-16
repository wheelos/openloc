#include "alignment_smoother.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

#include <Eigen/Cholesky>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "openloc/localization/online/validation.h"

namespace openloc {
namespace localization {
namespace internal {
namespace {

constexpr double kNanosecondsPerSecond = 1e9;
constexpr double kTwoPi = 6.28318530717958647692;

class GnssPositionFactor : public gtsam::NoiseModelFactor1<Eigen::Vector4d> {
 public:
  GnssPositionFactor(const gtsam::SharedNoiseModel& noise_model, gtsam::Key key,
                     Eigen::Vector3d antenna_in_odom,
                     Eigen::VectorXd antenna_position_enu)
      : gtsam::NoiseModelFactor1<Eigen::Vector4d>(noise_model, key),
        antenna_in_odom_(std::move(antenna_in_odom)),
        antenna_position_enu_(std::move(antenna_position_enu)) {}

  using gtsam::NoiseModelFactor1<Eigen::Vector4d>::evaluateError;

  gtsam::Vector evaluateError(
      const Eigen::Vector4d& alignment,
      gtsam::OptionalMatrixType jacobian) const override {
    const double yaw = alignment[3];
    const double cosine = std::cos(yaw);
    const double sine = std::sin(yaw);
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation(0, 0) = cosine;
    rotation(0, 1) = -sine;
    rotation(1, 0) = sine;
    rotation(1, 1) = cosine;

    if (jacobian != nullptr) {
      jacobian->resize(antenna_position_enu_.size(), 4);
      jacobian->setZero();
      jacobian->leftCols(antenna_position_enu_.size()).setIdentity();
      (*jacobian)(0, 3) =
          -sine * antenna_in_odom_.x() - cosine * antenna_in_odom_.y();
      (*jacobian)(1, 3) =
          cosine * antenna_in_odom_.x() - sine * antenna_in_odom_.y();
    }

    const Eigen::Vector3d predicted =
        alignment.head<3>() + rotation * antenna_in_odom_;
    return predicted.head(antenna_position_enu_.size()) - antenna_position_enu_;
  }

 private:
  Eigen::Vector3d antenna_in_odom_;
  Eigen::VectorXd antenna_position_enu_;
};

class GnssHeadingFactor : public gtsam::NoiseModelFactor1<Eigen::Vector4d> {
 public:
  GnssHeadingFactor(const gtsam::SharedNoiseModel& noise_model, gtsam::Key key,
                    double odom_base_yaw_rad, double baseline_yaw_in_base_rad,
                    double heading_azimuth_map_rad)
      : gtsam::NoiseModelFactor1<Eigen::Vector4d>(noise_model, key),
        predicted_heading_offset_(odom_base_yaw_rad + baseline_yaw_in_base_rad),
        heading_azimuth_map_rad_(heading_azimuth_map_rad) {}

  using gtsam::NoiseModelFactor1<Eigen::Vector4d>::evaluateError;

  gtsam::Vector evaluateError(
      const Eigen::Vector4d& alignment,
      gtsam::OptionalMatrixType jacobian) const override {
    if (jacobian != nullptr) {
      jacobian->resize(1, 4);
      jacobian->setZero();
      (*jacobian)(0, 3) = 1.0;
    }
    Eigen::Matrix<double, 1, 1> residual;
    residual[0] = std::remainder(
        alignment[3] + predicted_heading_offset_ - heading_azimuth_map_rad_,
        kTwoPi);
    return residual;
  }

 private:
  double predicted_heading_offset_;
  double heading_azimuth_map_rad_;
};

class AlignmentRandomWalkFactor
    : public gtsam::NoiseModelFactor2<Eigen::Vector4d, Eigen::Vector4d> {
 public:
  AlignmentRandomWalkFactor(const gtsam::SharedNoiseModel& noise_model,
                            gtsam::Key previous_key, gtsam::Key next_key)
      : gtsam::NoiseModelFactor2<Eigen::Vector4d, Eigen::Vector4d>(
            noise_model, previous_key, next_key) {}

  using gtsam::NoiseModelFactor2<Eigen::Vector4d,
                                 Eigen::Vector4d>::evaluateError;

  gtsam::Vector evaluateError(
      const Eigen::Vector4d& previous_alignment,
      const Eigen::Vector4d& next_alignment,
      gtsam::OptionalMatrixType previous_jacobian,
      gtsam::OptionalMatrixType next_jacobian) const override {
    if (previous_jacobian != nullptr) {
      *previous_jacobian = -Eigen::Matrix4d::Identity();
    }
    if (next_jacobian != nullptr) {
      *next_jacobian = Eigen::Matrix4d::Identity();
    }
    Eigen::Vector4d residual = next_alignment - previous_alignment;
    residual[3] = std::remainder(residual[3], kTwoPi);
    return residual;
  }
};

Status Invalid(const char* message) {
  return Status(StatusCode::kInvalidArgument, message);
}

gtsam::SharedNoiseModel DiagonalNoise(const Eigen::Vector4d& sigmas) {
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

}  // namespace

AlignmentSmoother::AlignmentSmoother(AlignmentSmootherConfig config)
    : config_(std::move(config)) {}

AlignmentSmoother::~AlignmentSmoother() = default;

Status AlignmentSmoother::ValidateConfig() const {
  if (!std::isfinite(config_.lag_seconds) || config_.lag_seconds <= 0.0 ||
      config_.lag_seconds >
          static_cast<double>(std::numeric_limits<int64_t>::max()) /
              kNanosecondsPerSecond ||
      !config_.initial_sigma.allFinite() ||
      (config_.initial_sigma.array() <= 0.0).any() ||
      !config_.initial_alignment.allFinite() ||
      !config_.random_walk_sigma_per_sqrt_meter.allFinite() ||
      (config_.random_walk_sigma_per_sqrt_meter.array() <= 0.0).any() ||
      !std::isfinite(config_.minimum_distance_delta_m) ||
      config_.minimum_distance_delta_m <= 0.0 ||
      !std::isfinite(config_.huber_delta) || config_.huber_delta <= 0.0 ||
      !std::isfinite(config_.innovation_gate_threshold) ||
      config_.innovation_gate_threshold <= 0.0 ||
      !std::isfinite(config_.horizontal_innovation_gate_threshold) ||
      config_.horizontal_innovation_gate_threshold <= 0.0 ||
      !std::isfinite(config_.heading_innovation_gate_threshold) ||
      config_.heading_innovation_gate_threshold <= 0.0) {
    return Invalid(
        "alignment smoother configuration must be finite and positive");
  }
  return Status::Ok();
}

Status AlignmentSmoother::CheckAvailable() const {
  if (!failure_message_.empty()) {
    return Status(StatusCode::kOptimizationFailed, failure_message_);
  }
  if (!smoother_) {
    return Status(StatusCode::kStateUnavailable,
                  "alignment smoother has no state");
  }
  return Status::Ok();
}

double AlignmentSmoother::RelativeSeconds(int64_t timestamp_ns) const {
  const long double elapsed_ns = static_cast<long double>(timestamp_ns) -
                                 static_cast<long double>(origin_timestamp_ns_);
  return static_cast<double>(elapsed_ns / kNanosecondsPerSecond);
}

Status AlignmentSmoother::AddState(int64_t timestamp_ns,
                                   double distance_delta_m,
                                   uint64_t* state_id) {
  Status status = ValidateConfig();
  if (!status.ok()) {
    return status;
  }
  if (state_id == nullptr || timestamp_ns <= 0 ||
      !std::isfinite(distance_delta_m) || distance_delta_m < 0.0) {
    return Invalid("state output, timestamp, or traveled distance is invalid");
  }
  if (smoother_ && !failure_message_.empty()) {
    return CheckAvailable();
  }
  if (has_ever_state_ && (smoother_ ? timestamp_ns <= latest_timestamp_ns_
                                    : timestamp_ns < latest_timestamp_ns_)) {
    return Status(StatusCode::kOutOfOrderTimestamp,
                  "alignment state timestamps must increase strictly");
  }

  const uint64_t state_index = has_ever_state_ ? state_index_ + 1 : 0;
  const gtsam::Key next_key = gtsam::Symbol('a', state_index);
  const Eigen::Vector4d initial =
      smoother_ ? latest_estimate_ : config_.initial_alignment;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  gtsam::IncrementalFixedLagSmoother::KeyTimestampMap timestamps;
  if (!smoother_) {
    graph.emplace_shared<gtsam::PriorFactor<Eigen::Vector4d>>(
        next_key, initial, DiagonalNoise(config_.initial_sigma));
    origin_timestamp_ns_ = timestamp_ns;
  } else {
    const double distance =
        std::max(distance_delta_m, config_.minimum_distance_delta_m);
    const Eigen::Vector4d sigmas =
        config_.random_walk_sigma_per_sqrt_meter * std::sqrt(distance);
    const auto noise = DiagonalNoise(sigmas);
    graph.emplace_shared<AlignmentRandomWalkFactor>(noise, latest_key_,
                                                    next_key);
  }
  values.insert(next_key, initial);
  timestamps.emplace(next_key, RelativeSeconds(timestamp_ns));

  try {
    if (!smoother_) {
      smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(
          config_.lag_seconds);
    }
    smoother_->update(graph, values, timestamps);
    latest_estimate_ =
        smoother_->calculateEstimate().at<Eigen::Vector4d>(next_key);
  } catch (const std::exception& error) {
    failure_message_ =
        std::string("GTSAM fixed-lag update failed: ") + error.what();
    return Status(StatusCode::kOptimizationFailed, failure_message_);
  }

  latest_timestamp_ns_ = timestamp_ns;
  latest_key_ = next_key;
  state_index_ = state_index;
  has_ever_state_ = true;
  state_timestamps_[state_index] = timestamp_ns;
  *state_id = state_index;

  const int64_t lag_ns =
      static_cast<int64_t>(config_.lag_seconds * kNanosecondsPerSecond);
  const int64_t oldest_timestamp_ns =
      timestamp_ns > lag_ns ? timestamp_ns - lag_ns : 0;
  for (auto iterator = state_timestamps_.begin();
       iterator != state_timestamps_.end() &&
       iterator->second < oldest_timestamp_ns;) {
    iterator = state_timestamps_.erase(iterator);
  }
  return Status::Ok();
}

Status AlignmentSmoother::Reanchor(int64_t timestamp_ns,
                                   const Eigen::Vector4d& alignment,
                                   const Eigen::Vector4d& prior_sigma,
                                   uint64_t* state_id) {
  Status status = CheckAvailable();
  if (!status.ok()) {
    return status;
  }
  if (timestamp_ns < latest_timestamp_ns_ ||
      (last_reanchor_timestamp_ns_ != 0 &&
       timestamp_ns <= last_reanchor_timestamp_ns_) ||
      !alignment.allFinite() || !prior_sigma.allFinite() ||
      (prior_sigma.array() <= 0.0).any() || state_id == nullptr) {
    return Invalid("global alignment re-anchor inputs are invalid");
  }
  config_.initial_alignment = alignment;
  config_.initial_sigma = prior_sigma;
  latest_estimate_ = alignment;
  smoother_.reset();
  state_timestamps_.clear();
  failure_message_.clear();
  status = AddState(timestamp_ns, 0.0, state_id);
  if (status.ok()) {
    last_reanchor_timestamp_ns_ = timestamp_ns;
  }
  return status;
}

Status AlignmentSmoother::AddGnssPosition(
    uint64_t state_id, int64_t timestamp_ns,
    const Eigen::Vector3d& antenna_in_odom,
    const Eigen::Vector3d& antenna_position_enu,
    const Eigen::Matrix3d& covariance_enu, GnssFactorDiagnostics* diagnostics,
    bool initializing) {
  return AddGnssPositionImpl(state_id, timestamp_ns, antenna_in_odom,
                             antenna_position_enu, covariance_enu, diagnostics,
                             initializing);
}

Status AlignmentSmoother::AddGnssHorizontalPosition(
    uint64_t state_id, int64_t timestamp_ns,
    const Eigen::Vector3d& antenna_in_odom,
    const Eigen::Vector2d& antenna_position_xy,
    const Eigen::Matrix2d& covariance_xy, GnssFactorDiagnostics* diagnostics,
    bool initializing) {
  return AddGnssPositionImpl(state_id, timestamp_ns, antenna_in_odom,
                             antenna_position_xy, covariance_xy, diagnostics,
                             initializing);
}

Status AlignmentSmoother::AddGnssPositionImpl(
    uint64_t state_id, int64_t timestamp_ns,
    const Eigen::Vector3d& antenna_in_odom,
    const Eigen::VectorXd& antenna_position_enu,
    const Eigen::MatrixXd& covariance_enu, GnssFactorDiagnostics* diagnostics,
    bool initializing) {
  Status status = CheckAvailable();
  if (!status.ok()) {
    return status;
  }
  if (diagnostics == nullptr || !state_timestamps_.count(state_id)) {
    return Status(StatusCode::kOutsideFixedLagWindow,
                  "GNSS alignment state is outside the fixed-lag window");
  }
  if (timestamp_ns < state_timestamps_.at(state_id)) {
    return Status(StatusCode::kStateUnavailable,
                  "GNSS timestamp is outside its selected alignment interval");
  }
  if (!antenna_in_odom.allFinite() || !antenna_position_enu.allFinite()) {
    return Invalid("GNSS antenna positions must be finite");
  }
  const Eigen::Index dimension = antenna_position_enu.size();
  status = ValidateCovariance(covariance_enu, dimension);
  if (!status.ok()) {
    return status;
  }

  const gtsam::SharedNoiseModel gaussian =
      gtsam::noiseModel::Gaussian::Covariance(covariance_enu);
  const gtsam::SharedNoiseModel robust = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(config_.huber_delta),
      gaussian);

  const gtsam::Key key = gtsam::Symbol('a', state_id);
  Eigen::Vector4d alignment;
  Eigen::Matrix4d alignment_covariance;
  try {
    alignment = smoother_->calculateEstimate<Eigen::Vector4d>(key);
    alignment_covariance = smoother_->marginalCovariance(key);
  } catch (const std::exception& exception) {
    return Status(
        StatusCode::kOptimizationFailed,
        std::string("GTSAM GNSS state query failed: ") + exception.what());
  }
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(dimension, 4);
  const double yaw = alignment[3];
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  jacobian.leftCols(dimension).setIdentity();
  jacobian(0, 3) = -sine * antenna_in_odom.x() - cosine * antenna_in_odom.y();
  jacobian(1, 3) = cosine * antenna_in_odom.x() - sine * antenna_in_odom.y();
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d predicted =
      alignment.head<3>() + rotation * antenna_in_odom;
  const Eigen::VectorXd residual =
      predicted.head(dimension) - antenna_position_enu;
  const Eigen::MatrixXd innovation_covariance =
      jacobian * alignment_covariance * jacobian.transpose() + covariance_enu;
  if (!innovation_covariance.allFinite()) {
    return Status(StatusCode::kOptimizationFailed,
                  "GNSS innovation covariance is not finite");
  }
  const Eigen::LDLT<Eigen::MatrixXd> decomposition(innovation_covariance);
  if (decomposition.info() != Eigen::Success ||
      (decomposition.vectorD().array() <= 0.0).any()) {
    return Status(StatusCode::kOptimizationFailed,
                  "GNSS innovation covariance is not positive definite");
  }
  const double innovation = residual.dot(decomposition.solve(residual));
  if (!std::isfinite(innovation)) {
    return Status(StatusCode::kOptimizationFailed,
                  "GNSS normalized innovation is not finite");
  }
  diagnostics->residual = residual;
  diagnostics->normalized_innovation_squared = innovation;
  const Eigen::Matrix2d horizontal_innovation_covariance =
      innovation_covariance.topLeftCorner<2, 2>();
  const Eigen::LDLT<Eigen::Matrix2d> horizontal_decomposition(
      horizontal_innovation_covariance);
  if (horizontal_decomposition.info() != Eigen::Success ||
      (horizontal_decomposition.vectorD().array() <= 0.0).any()) {
    return Status(
        StatusCode::kOptimizationFailed,
        "GNSS horizontal innovation covariance is not positive definite");
  }
  const Eigen::Vector2d horizontal_residual = residual.head<2>();
  const double horizontal_innovation = horizontal_residual.dot(
      horizontal_decomposition.solve(horizontal_residual));
  if (!std::isfinite(horizontal_innovation)) {
    return Status(StatusCode::kOptimizationFailed,
                  "GNSS horizontal normalized innovation is not finite");
  }
  diagnostics->horizontal_innovation_rejected =
      !initializing &&
      horizontal_innovation > config_.horizontal_innovation_gate_threshold;
  const Eigen::LDLT<Eigen::MatrixXd> measurement_decomposition(covariance_enu);
  if (measurement_decomposition.info() != Eigen::Success ||
      (measurement_decomposition.vectorD().array() <= 0.0).any()) {
    return Invalid("GNSS measurement covariance is not positive definite");
  }
  const double factor_mahalanobis_squared =
      residual.dot(measurement_decomposition.solve(residual));
  diagnostics->robust_weight =
      std::min(1.0, config_.huber_delta /
                        std::sqrt(std::max(factor_mahalanobis_squared, 0.0)));
  const double threshold = dimension == 2
                               ? config_.horizontal_innovation_gate_threshold
                               : config_.innovation_gate_threshold;
  if (!initializing && innovation > threshold) {
    return Status(StatusCode::kRejectedMeasurement,
                  "GNSS normalized innovation exceeds the 99% chi-square gate");
  }

  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<GnssPositionFactor>(robust, key, antenna_in_odom,
                                           antenna_position_enu);
  try {
    smoother_->update(graph);
    latest_estimate_ =
        smoother_->calculateEstimate().at<Eigen::Vector4d>(latest_key_);
  } catch (const std::exception& exception) {
    failure_message_ =
        std::string("GTSAM GNSS factor update failed: ") + exception.what();
    return Status(StatusCode::kOptimizationFailed, failure_message_);
  }
  return Status::Ok();
}

Status AlignmentSmoother::AddGnssHeading(
    uint64_t state_id, int64_t timestamp_ns, double odom_base_yaw_rad,
    double baseline_yaw_in_base_rad, double heading_azimuth_map_rad,
    double heading_variance_rad2, HeadingFactorDiagnostics* diagnostics,
    bool initializing) {
  Status status = CheckAvailable();
  if (!status.ok()) {
    return status;
  }
  if (diagnostics == nullptr || !state_timestamps_.count(state_id)) {
    return Status(
        StatusCode::kOutsideFixedLagWindow,
        "GNSS heading alignment state is outside the fixed-lag window");
  }
  if (timestamp_ns < state_timestamps_.at(state_id)) {
    return Status(
        StatusCode::kStateUnavailable,
        "GNSS heading timestamp is outside its selected alignment interval");
  }
  if (!std::isfinite(odom_base_yaw_rad) ||
      !std::isfinite(baseline_yaw_in_base_rad) ||
      !std::isfinite(heading_azimuth_map_rad) ||
      !std::isfinite(heading_variance_rad2) || heading_variance_rad2 <= 0.0) {
    return Invalid("GNSS heading factor inputs must be finite and positive");
  }

  const gtsam::Key key = gtsam::Symbol('a', state_id);
  Eigen::Vector4d alignment;
  Eigen::Matrix4d alignment_covariance;
  try {
    alignment = smoother_->calculateEstimate<Eigen::Vector4d>(key);
    alignment_covariance = smoother_->marginalCovariance(key);
  } catch (const std::exception& exception) {
    return Status(
        StatusCode::kOptimizationFailed,
        std::string("GTSAM heading state query failed: ") + exception.what());
  }

  Eigen::Matrix<double, 1, 4> jacobian = Eigen::Matrix<double, 1, 4>::Zero();
  jacobian(0, 3) = 1.0;
  const double residual =
      std::remainder(alignment[3] + odom_base_yaw_rad +
                         baseline_yaw_in_base_rad - heading_azimuth_map_rad,
                     kTwoPi);
  const double innovation_variance =
      (jacobian * alignment_covariance * jacobian.transpose())(0, 0) +
      heading_variance_rad2;
  if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0) {
    return Status(StatusCode::kOptimizationFailed,
                  "GNSS heading innovation variance is invalid");
  }
  const double innovation = residual * residual / innovation_variance;
  if (!std::isfinite(innovation)) {
    return Status(StatusCode::kOptimizationFailed,
                  "GNSS heading normalized innovation is not finite");
  }
  diagnostics->residual_rad = residual;
  diagnostics->normalized_innovation_squared = innovation;
  const double measurement_mahalanobis_squared =
      residual * residual / heading_variance_rad2;
  diagnostics->robust_weight = std::min(
      1.0, config_.huber_delta / std::sqrt(measurement_mahalanobis_squared));
  if (!initializing && innovation > config_.heading_innovation_gate_threshold) {
    return Status(StatusCode::kRejectedMeasurement,
                  "GNSS heading normalized innovation exceeds the 99% gate");
  }

  const auto gaussian =
      gtsam::noiseModel::Isotropic::Sigma(1, std::sqrt(heading_variance_rad2));
  const auto robust = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(config_.huber_delta),
      gaussian);
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<GnssHeadingFactor>(robust, key, odom_base_yaw_rad,
                                          baseline_yaw_in_base_rad,
                                          heading_azimuth_map_rad);
  try {
    smoother_->update(graph);
    latest_estimate_ =
        smoother_->calculateEstimate().at<Eigen::Vector4d>(latest_key_);
  } catch (const std::exception& exception) {
    failure_message_ =
        std::string("GTSAM GNSS heading factor update failed: ") +
        exception.what();
    return Status(StatusCode::kOptimizationFailed, failure_message_);
  }
  return Status::Ok();
}

Status AlignmentSmoother::Estimate(AlignmentEstimate* estimate) const {
  if (estimate == nullptr) {
    return Invalid("alignment estimate output must not be null");
  }
  Status status = CheckAvailable();
  if (!status.ok()) {
    return status;
  }
  estimate->timestamp_ns = latest_timestamp_ns_;
  estimate->map_from_odom = latest_estimate_;
  estimate->map_from_odom[3] = std::remainder(latest_estimate_[3], kTwoPi);
  try {
    estimate->covariance = smoother_->marginalCovariance(latest_key_);
  } catch (const std::exception& exception) {
    return Status(
        StatusCode::kOptimizationFailed,
        std::string("GTSAM marginal covariance failed: ") + exception.what());
  }
  return Status::Ok();
}

}  // namespace internal
}  // namespace localization
}  // namespace openloc
