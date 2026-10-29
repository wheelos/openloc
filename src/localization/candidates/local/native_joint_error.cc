// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/local/native_joint_error.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <utility>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"
#include "Eigen/QR"
#include "Eigen/SVD"
#include "candidates/local/local_motion.h"

namespace openloc::localization::unified {

std::string NativeJointError::History(uint64_t sequence, bool propagated) {
  return std::string(propagated ? "pre:" : "history:") +
         std::to_string(sequence);
}

std::string NativeJointError::Point(uint64_t sequence, std::size_t index) {
  return "point:" + std::to_string(sequence) + ":" + std::to_string(index);
}

void NativeJointError::Reset() {
  active_ = false;
  frontier_finalized_ = false;
  dimension_ = 0;
  memory_.clear();
  history_.clear();
  pending_sources_.clear();
  driving_cross_.clear();
  anonymous_source_ = 0;
  forks_.clear();
  next_fork_ = 0;
  first_imu_ = second_imu_ = {};
  previous_cross_.resize(0, 0);
  status_ = {Reason::CORRELATION_UNQUALIFIED,
             "Native joint stochastic propagation has not initialized"};
}

void NativeJointError::Fail(const Result& result) {
  if (active_ || status_.ok()) status_ = result;
  active_ = false;
  memory_.clear();
  dimension_ = 0;
  previous_cross_.resize(0, 0);
  driving_cross_.clear();
  forks_.clear();
  pending_sources_.clear();
}

void NativeJointError::Initialize(const LocalState& state) {
  Reset();
  if (!config_.enable_native_joint_error) return;
  active_ = true;
  status_ = {};
  if (!state.covariance_model_valid) {
    Fail({Reason::CORRELATION_UNQUALIFIED,
          "Startup lacks raw-source covariance and IMU/registration cross terms"});
    return;
  }
  Independent("internal", state.covariance);
  if (!active_) return;
  Eigen::MatrixXd output = Eigen::MatrixXd::Zero(18, dimension_);
  output.topRows(15) = Get("internal");
  Set("output", output);
}

void NativeJointError::InitializeStationary(
    const LocalState& state, const Eigen::Vector3d& acceleration,
    uint32_t count, bool endpoint_in_mean) {
  Initialize(state);
  if (!active_) return;
  if (count == 0 || !std::isfinite(config_.native_imu_sample_period) ||
      config_.native_imu_sample_period <= 0) {
    Fail({Reason::CORRELATION_UNQUALIFIED,
          "Sampled IMU startup requires its explicit noise sample period"});
    return;
  }
  Matrix6d sample = Matrix6d::Zero();
  sample.diagonal().head<3>().setConstant(
      config_.accel_noise * config_.accel_noise / config_.native_imu_sample_period);
  sample.diagonal().tail<3>().setConstant(
      config_.gyro_noise * config_.gyro_noise / config_.native_imu_sample_period);
  Eigen::Matrix<double, 12, 12> covariance =
      Eigen::Matrix<double, 12, 12>::Zero();
  covariance.topLeftCorner<6, 6>() = sample / count;
  covariance.bottomRightCorner<6, 6>() = sample;
  if (endpoint_in_mean) {
    covariance.topRightCorner<6, 6>() = sample / count;
    covariance.bottomLeftCorner<6, 6>() = sample / count;
  }
  Independent("stationary_measurements", covariance, "stationary:measurements");
  if (!active_) return;
  const double horizontal = std::hypot(acceleration.y(), acceleration.z());
  const double squared = acceleration.squaredNorm();
  if (horizontal <= 1e-12 || squared <= 1e-24) {
    Fail({Reason::CORRELATION_UNQUALIFIED,
          "Stationary tilt branch has no unique raw-IMU derivative"});
    return;
  }
  const Eigen::RowVector3d roll(
      0, acceleration.z() / (horizontal * horizontal),
      -acceleration.y() / (horizontal * horizontal));
  const Eigen::RowVector3d pitch(
      -horizontal / squared,
      acceleration.x() * acceleration.y() / (horizontal * squared),
      acceleration.x() * acceleration.z() / (horizontal * squared));
  const double angle = std::atan2(acceleration.y(), acceleration.z());
  Eigen::Matrix<double, 15, 6> mean = Eigen::Matrix<double, 15, 6>::Zero();
  mean.block<3, 3>(6, 0) = Eigen::Vector3d::UnitX() * roll +
      (Eigen::AngleAxisd(-angle, Eigen::Vector3d::UnitX()) *
       Eigen::Vector3d::UnitY()) * pitch;
  mean.block<3, 3>(9, 3).setIdentity();
  Set("internal", Get("internal") +
      mean * Get("stationary_measurements").topRows(6));
  Set("imu:last", Get("stationary_measurements").bottomRows(6));
  Drop("stationary_measurements");
  if (!active_) return;
  Eigen::MatrixXd output = Eigen::MatrixXd::Zero(18, dimension_);
  output.topRows(15) = Get("internal");
  Set("output", output);
}

void NativeJointError::InitializeRelative(
    const LocalState& state, const Eigen::Matrix<double, 21, 21>& covariance) {
  Reset();
  if (!config_.enable_native_joint_error) return;
  active_ = true;
  status_ = {};
  // Retain the owner's declared residual initial uncertainty separately from
  // the propagated raw startup errors, as for stationary initialization.
  Eigen::Matrix<double, 21, 21> joint = covariance;
  joint.topLeftCorner<15, 15>() += state.covariance;
  Independent("startup:joint", joint, "startup:raw");
  if (!active_) return;
  Set("internal", Get("startup:joint").topRows(15));
  Set("imu:last", Get("startup:joint").bottomRows(6));
  Drop("startup:joint");
  Eigen::MatrixXd output = Eigen::MatrixXd::Zero(18, dimension_);
  output.topRows(15) = Get("internal");
  Set("output", output);
}

void NativeJointError::RejectInitialization(const Result& result) {
  Reset();
  status_ = result;
}

Eigen::MatrixXd NativeJointError::Get(const std::string& key) const {
  const auto found = memory_.find(key);
  return found == memory_.end() ? Eigen::MatrixXd() : found->second;
}

Result NativeJointError::CheckAdditionalCapacity(std::size_t rows,
                                                 std::size_t drivers) const {
  if (!active_) return status_;
  std::size_t stored_rows = 0;
  for (const auto& item : memory_) stored_rows += item.second.rows();
  const std::size_t maximum = config_.native_joint_max_memory_rows;
  const auto exceeds = [maximum](std::size_t stored, std::size_t added) {
    return stored > maximum || added > maximum - stored;
  };
  std::size_t pinned_rows = 0;
  for (const auto& fork : forks_)
    if (!fork.second.expired()) {
      if (exceeds(pinned_rows, drivers)) {
        pinned_rows = maximum + 1;
        break;
      }
      pinned_rows += drivers;
    }
  if (!exceeds(stored_rows, rows) &&
      !exceeds(stored_rows + rows, pinned_rows) &&
      !exceeds(static_cast<std::size_t>(dimension_), drivers))
    return {};
  std::ostringstream detail;
  detail << "Native joint batch capacity exceeded before allocation; no "
            "dependency dropped; stored_rows="
         << stored_rows << "; additional_rows=" << rows
         << "; driver_dimension=" << dimension_
         << "; additional_drivers=" << drivers
         << "; pinned_driver_rows=" << pinned_rows
         << "; maximum_rows=" << maximum;
  return {Reason::CORRELATION_UNQUALIFIED, detail.str()};
}

void NativeJointError::Set(const std::string& key,
                           const Eigen::MatrixXd& loading) {
  if (!active_) return;
  if (loading.cols() != dimension_ || loading.rows() == 0 ||
      !loading.allFinite()) {
    Fail({Reason::CORRELATION_UNQUALIFIED,
          "Missing or nonfinite native joint memory loading"});
    return;
  }
  Eigen::Index rows = loading.rows();
  for (const auto& item : memory_)
    if (item.first != key) rows += item.second.rows();
  if (rows > config_.native_joint_max_memory_rows) {
    Eigen::Index stored_rows = 0;
    std::map<std::string, std::pair<std::size_t, Eigen::Index>> groups;
    for (const auto& item : memory_) {
      stored_rows += item.second.rows();
      auto& group = groups[item.first.substr(0, item.first.find(':'))];
      ++group.first;
      group.second += item.second.rows();
    }
    const auto previous = memory_.find(key);
    std::ostringstream detail;
    detail << "Native joint physical-memory capacity exceeded; no dependency "
              "dropped"
           << "; key=" << key << "; stored_rows=" << stored_rows
           << "; incoming_rows=" << loading.rows() << "; replaced_rows="
           << (previous == memory_.end() ? 0 : previous->second.rows())
           << "; required_rows=" << rows
           << "; maximum_rows=" << config_.native_joint_max_memory_rows
           << "; driver_dimension=" << dimension_
           << "; snapshots=" << history_.size()
           << "; entries=" << memory_.size() << "; groups=";
    for (const auto& group : groups)
      detail << group.first << ':' << group.second.first << '/'
             << group.second.second << ',';
    Fail({Reason::CORRELATION_UNQUALIFIED, detail.str()});
    return;
  }
  memory_[key] = loading;
}

void NativeJointError::Drop(const std::string& key) { memory_.erase(key); }

void NativeJointError::Retain(const std::vector<std::string>& keys) {
  const std::set<std::string> retained(keys.begin(), keys.end());
  for (auto it = forks_.begin(); it != forks_.end();) {
    if (it->second.expired()) it = forks_.erase(it);
    else ++it;
  }
  for (auto it = memory_.begin(); it != memory_.end();) {
    bool pinned = false;
    for (const auto& fork : forks_)
      pinned |= it->first.compare(0, fork.first.size(), fork.first) == 0;
    if (retained.count(it->first) || pinned) ++it;
    else it = memory_.erase(it);
  }
}

std::shared_ptr<const int> NativeJointError::PinFork() {
  if (!active_) return {};
  const std::string key = "fork:" + std::to_string(++next_fork_) + ":";
  auto lease = std::make_shared<const int>(0);
  forks_[key] = lease;
  Set(key + "root", Eigen::MatrixXd::Identity(dimension_, dimension_));
  return lease;
}

void NativeJointError::Independent(const std::string& key,
                                  const Eigen::MatrixXd& covariance,
                                  const std::string& source) {
  if (!active_) return;
  if (covariance.rows() == 0 || covariance.rows() != covariance.cols() ||
      !covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-9)) {
    Fail({Reason::CORRELATION_UNQUALIFIED, "Invalid native driving covariance"});
    return;
  }
  Eigen::MatrixXd root;
  Eigen::LLT<Eigen::MatrixXd> positive(covariance);
  if (positive.info() == Eigen::Success) {
    root = positive.matrixL();
  } else {
    Eigen::VectorXd scale(covariance.rows());
    for (Eigen::Index i = 0; i < scale.size(); ++i)
      scale[i] = covariance(i, i) > 0 ? std::sqrt(covariance(i, i)) : 1;
    const Eigen::MatrixXd normalized = scale.cwiseInverse().asDiagonal() *
        covariance * scale.cwiseInverse().asDiagonal();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(normalized);
    if (spectrum.info() != Eigen::Success) {
      Fail({Reason::CORRELATION_UNQUALIFIED,
            "Native driving covariance decomposition failed"});
      return;
    }
    const auto& values = spectrum.eigenvalues();
    const double tolerance = 64 * std::numeric_limits<double>::epsilon() *
        values.cwiseAbs().maxCoeff();
    if (values.minCoeff() < -tolerance) {
      Fail({Reason::CORRELATION_UNQUALIFIED,
            "Native driving covariance is not PSD"});
      return;
    }
    Eigen::Index added = 0;
    for (Eigen::Index i = 0; i < values.size(); ++i) {
      if (values[i] > tolerance / 8 && values[i] <= tolerance) {
        Fail({Reason::CORRELATION_UNQUALIFIED,
              "Native driving covariance has unresolved stochastic rank"});
        return;
      }
      if (values[i] > tolerance) ++added;
    }
    root.resize(covariance.rows(), added);
    Eigen::Index column = 0;
    for (Eigen::Index i = 0; i < values.size(); ++i)
      if (values[i] > tolerance)
        root.col(column++) =
            scale.asDiagonal() * spectrum.eigenvectors().col(i) *
            std::sqrt(values[i]);
  }
  const Eigen::Index added = root.cols();
  const std::string identity = source.empty()
      ? "anonymous:" + std::to_string(++anonymous_source_) : source;
  if (driving_cross_.count(identity)) {
    Fail({Reason::CORRELATION_UNQUALIFIED,
          "A native primitive noise source was independently inserted twice"});
    return;
  }
  if (dimension_ + added > config_.native_joint_max_memory_rows) {
    Fail({Reason::CORRELATION_UNQUALIFIED,
          "Native joint driving-variable capacity exceeded"});
    return;
  }
  dimension_ += added;
  for (auto& item : driving_cross_) {
    item.second.conservativeResize(Eigen::NoChange, dimension_);
    item.second.rightCols(added).setZero();
  }
  for (auto& item : memory_) {
    item.second.conservativeResize(Eigen::NoChange, dimension_);
    item.second.rightCols(added).setZero();
  }
  if (previous_cross_.rows()) {
    previous_cross_.conservativeResize(Eigen::NoChange, dimension_);
    previous_cross_.rightCols(added).setZero();
  }
  Eigen::MatrixXd loading = Eigen::MatrixXd::Zero(covariance.rows(), dimension_);
  loading.rightCols(added) = root;
  Eigen::MatrixXd cross = Eigen::MatrixXd::Zero(added, dimension_);
  cross.rightCols(added).setIdentity();
  driving_cross_[identity] = std::move(cross);
  Set(key, loading);
  if (!active_) return;
  // Outstanding tasks may later reuse errors whose ordinary history clones
  // have retired. Pin their actual primitive drivers in the LIVE sufficient
  // state until task commit/discard; otherwise a late splice is non-Markov.
  for (const auto& fork : forks_) {
    if (added != 0 && !fork.second.expired()) {
      Set(fork.first + identity, driving_cross_.at(identity));
      if (!active_) return;
    }
  }
}

void NativeJointError::BeginImu(const ImuSample& first, const ImuSample& second) {
  if (!active_) return;
  if (!std::isfinite(config_.native_imu_sample_period) ||
      config_.native_imu_sample_period <= 0) {
    Fail({Reason::CORRELATION_UNQUALIFIED,
          "Sampled IMU propagation requires an explicit noise sample period"});
    return;
  }
  first_imu_ = first.stamp;
  second_imu_ = second.stamp;
  if (Get("imu:last").rows() == 6) Set("imu:first", Get("imu:last"));
  else {
    Matrix6d covariance = Matrix6d::Zero();
    covariance.diagonal().head<3>().setConstant(
        config_.accel_noise * config_.accel_noise / config_.native_imu_sample_period);
    covariance.diagonal().tail<3>().setConstant(
        config_.gyro_noise * config_.gyro_noise / config_.native_imu_sample_period);
    Independent("imu:first", covariance, "imu:" + std::to_string(first.stamp.sequence));
  }
  if (!active_) return;
  Matrix6d covariance = Matrix6d::Zero();
  covariance.diagonal().head<3>().setConstant(
      config_.accel_noise * config_.accel_noise / config_.native_imu_sample_period);
  covariance.diagonal().tail<3>().setConstant(
      config_.gyro_noise * config_.gyro_noise / config_.native_imu_sample_period);
  Independent("imu:last", covariance, "imu:" + std::to_string(second.stamp.sequence));
}

Eigen::MatrixXd NativeJointError::ImuAt(double time) const {
  const double alpha = (time - first_imu_.time) /
                       (second_imu_.time - first_imu_.time);
  return (1 - alpha) * Get("imu:first") + alpha * Get("imu:last");
}

void NativeJointError::AdvanceImu(
    const Matrix15d& transition,
    const Eigen::Matrix<double, 15, 6>& first,
    const Eigen::Matrix<double, 15, 6>& second, double start, double end) {
  if (!active_) return;
  Matrix6d covariance = Matrix6d::Zero();
  covariance.diagonal().head<3>().setConstant(
      config_.gyro_bias_noise * config_.gyro_bias_noise * (end - start));
  covariance.diagonal().tail<3>().setConstant(
      config_.accel_bias_noise * config_.accel_bias_noise * (end - start));
  // The physical bias driving variable is at the END of this actual
  // subinterval. The measurement noises are the original shared samples.
  std::ostringstream identity;
  identity << "bias:" << first_imu_.sequence << ":" << std::hexfloat
           << start << ":" << end;
  Independent("bias:driving", covariance, identity.str());
  if (!active_) return;
  Eigen::MatrixXd next = transition * Get("internal") +
      first * ImuAt(start) + second * ImuAt(end);
  next.bottomRows(6) += Get("bias:driving");
  Set("internal", next);
  Drop("bias:driving");
}

void NativeJointError::AddSource(const SourceSampleId& source) {
  if (active_) pending_sources_.push_back(source);
}

void NativeJointError::AdvanceInternal(const Matrix15d& transition,
                                      const Matrix15d& noise) {
  if (!active_) return;
  Independent("driving", noise);
  if (!active_) return;
  Set("internal", transition * Get("internal") + Get("driving"));
  Drop("driving");
}

void NativeJointError::AdvanceOutput(
    const ContinuousOutputErrorTransition& transition) {
  if (!active_) return;
  Set("output", transition.previous_internal * Get("previous_internal") +
                    transition.current_internal * Get("internal") +
                    transition.previous_output * Get("output"));
}

void NativeJointError::Compact() {
  if (!active_ || dimension_ == 0) return;
  Eigen::Index rows = 0;
  for (const auto& item : memory_) rows += item.second.rows();
  Eigen::MatrixXd normalized(rows, dimension_);
  Eigen::Index offset = 0;
  for (const auto& item : memory_) {
    for (Eigen::Index i = 0; i < item.second.rows(); ++i) {
      const double norm = item.second.row(i).norm();
      normalized.row(offset++) =
          item.second.row(i) / (norm == 0 ? 1 : norm);
    }
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(normalized, Eigen::ComputeThinV);
  if (svd.info() != Eigen::Success || !svd.singularValues().allFinite()) {
    Fail({Reason::CORRELATION_UNQUALIFIED,
          "Native physical-memory basis decomposition failed"});
    return;
  }
  // Row normalization preserves small physical scales. Do not multiply the
  // rank cutoff by the memory size: weak but nonzero smoother/geometry
  // directions would then be silently classified as deterministic.
  const double threshold = 8 * std::numeric_limits<double>::epsilon() *
      svd.singularValues()[0];
  Eigen::Index rank = 0;
  for (const double value : svd.singularValues())
    if (value > threshold) ++rank;
  if (rank < 15 || rank > config_.native_joint_max_dimension) {
    Fail({Reason::CORRELATION_UNQUALIFIED,
          "Native minimal joint basis exceeds capacity or loses published rank"});
    return;
  }
  // Jacobi rotations can accumulate enough loss of orthogonality to violate
  // the graph's exact-noise PSD test. Re-orthogonalize the retained subspace;
  // do not compensate with noise or a looser covariance admission threshold.
  Eigen::HouseholderQR<Eigen::MatrixXd> orthogonal(
      svd.matrixV().leftCols(rank));
  const Eigen::MatrixXd basis = orthogonal.householderQ() *
      Eigen::MatrixXd::Identity(dimension_, rank);
  // Reject a numerically unresolved positive direction, rather than silently
  // turning it into a deterministic constraint.
  const double discarded =
      (normalized - normalized * basis * basis.transpose()).norm();
  if (discarded > 1e-11 * std::max(1.0, normalized.norm())) {
    Fail({Reason::CORRELATION_UNQUALIFIED,
          "Native physical-memory nullspace is numerically unresolved"});
    return;
  }
  for (auto& item : memory_) item.second = (item.second * basis).eval();
  for (auto& item : driving_cross_)
    item.second = (item.second * basis).eval();
  if (previous_cross_.rows())
    previous_cross_ = (previous_cross_ * basis).eval();
  dimension_ = rank;
}

void NativeJointError::Publish(
    const LocalState& state, const std::vector<SourceSampleId>& sources) {
  if (!active_) return;
  Compact();
  if (!active_) return;
  LocalState modeled = state;
  modeled.covariance_square_root.reset();
  const Eigen::MatrixXd output = Get("output").topRows(15);
  modeled.covariance = output * output.transpose();
  modeled.covariance =
      (0.5 * (modeled.covariance + modeled.covariance.transpose())).eval();
  modeled.joint_error =
      JointLocalError{Eigen::MatrixXd::Identity(dimension_, dimension_), output};
  modeled.covariance_model_valid = true;
  modeled.uncertainty_qualified = false;
  if (Eigen::LLT<Matrix15d>(modeled.covariance).info() != Eigen::Success) {
    Fail({Reason::CORRELATION_UNQUALIFIED,
          "Native published stochastic marginal is not full rank"});
    return;
  }
  auto all_sources = sources;
  std::set<std::pair<std::string, uint64_t>> identities;
  for (const auto& source : sources)
    identities.emplace(source.source, source.sequence);
  for (const auto& source : pending_sources_)
    if (identities.emplace(source.source, source.sequence).second)
      all_sources.push_back(source);
  pending_sources_.clear();
  history_.push_back({std::move(modeled), previous_cross_, std::move(all_sources)});
  frontier_finalized_ = false;
  driving_cross_.clear();
  previous_cross_ = Eigen::MatrixXd::Identity(dimension_, dimension_);
  while (history_.size() > config_.native_joint_max_snapshots ||
         (!history_.empty() &&
          state.stamp.time - history_.front().state.stamp.time >
              config_.history_duration))
    history_.pop_front();
}

Result NativeJointError::FinalizeFrontier(const LocalState& state) {
  if (!active_) return status_;
  if (frontier_finalized_)
    return {Reason::DUPLICATE, "Joint frontier has already been exported"};
  if (history_.empty() ||
      history_.back().state.stamp.time != state.stamp.time ||
      history_.back().state.stamp.sequence != state.stamp.sequence ||
      history_.back().state.epoch != state.epoch)
    return {Reason::HISTORY_UNAVAILABLE, "No matching joint frontier to finalize"};
  Compact();
  if (!active_) return status_;
  auto& snapshot = history_.back();
  const Eigen::MatrixXd loading = Get("output").topRows(15);
  if (!(loading * loading.transpose()).isApprox(snapshot.state.covariance, 1e-9))
    return {Reason::CORRELATION_UNQUALIFIED,
            "Frontier finalization would replace a published marginal"};
  if (snapshot.previous_cross.rows())
    snapshot.previous_cross = (snapshot.previous_cross * previous_cross_).eval();
  snapshot.state.joint_error =
      JointLocalError{Eigen::MatrixXd::Identity(dimension_, dimension_), loading};
  std::set<std::pair<std::string, uint64_t>> identities;
  for (const auto& source : snapshot.sources)
    identities.emplace(source.source, source.sequence);
  for (const auto& source : pending_sources_)
    if (identities.emplace(source.source, source.sequence).second)
      snapshot.sources.push_back(source);
  pending_sources_.clear();
  driving_cross_.clear();
  previous_cross_ = Eigen::MatrixXd::Identity(dimension_, dimension_);
  frontier_finalized_ = true;
  return {};
}

Result NativeJointError::Lookup(double time, LocalState* state) const {
  if (!state || !std::isfinite(time) || time <= 0)
    return {Reason::INVALID_INPUT, "Invalid joint-error history request"};
  for (const auto& entry : history_) {
    if (entry.state.stamp.time != time) continue;
    *state = entry.state;
    return {};
  }
  if (!status_.ok()) return status_;
  return {Reason::HISTORY_UNAVAILABLE, "No exact retained joint-error snapshot"};
}

Result NativeJointError::MotionBetween(double start, double end,
                                      MotionIncrement* motion) const {
  if (!motion || !std::isfinite(start) || !std::isfinite(end) ||
      start <= 0 || end <= start)
    return {Reason::INVALID_INPUT, "Invalid native joint-error motion request"};
  LocalState first, last;
  auto status = Lookup(start, &first);
  if (status.ok()) status = Lookup(end, &last);
  if (!status.ok()) return status;
  Eigen::MatrixXd cross = first.joint_error->covariance;
  std::vector<SourceSampleId> sources;
  std::set<std::pair<std::string, uint64_t>> identities;
  for (const auto& entry : history_) {
    if (entry.state.stamp.time <= start) continue;
    if (entry.state.stamp.time > end) break;
    if (cross.cols() != entry.previous_cross.rows())
      return {Reason::CORRELATION_UNQUALIFIED,
              "Native joint-error motion crossed a stochastic-model break"};
    cross = (cross * entry.previous_cross).eval();
    for (const auto& source : entry.sources)
      if (identities.emplace(source.source, source.sequence).second)
        sources.push_back(source);
  }
  MotionIncrement next;
  const Matrix15d projected = first.joint_error->output_jacobian * cross *
      last.joint_error->output_jacobian.transpose();
  status = ComputeMotionIncrement(first, last, projected, sources, &next);
  if (!status.ok()) return status;
  next.joint_cross_covariance = std::move(cross);
  status = ValidateJointMotion(first, last, next);
  if (!status.ok()) return status;
  *motion = std::move(next);
  return {};
}

}  // namespace openloc::localization::unified
