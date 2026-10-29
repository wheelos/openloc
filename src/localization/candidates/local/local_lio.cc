// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/local/local_lio.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"
#include "Eigen/SVD"
#include "candidates/common/observed_subspace.h"
#include "candidates/local/local_motion.h"

namespace openloc {
namespace localization {
namespace unified {

Result DeskewLidarScan(const LidarScan& scan, const StateLookup& lookup,
                       DeskewedScan* deskewed,
                       std::vector<DeskewPointError>* errors) {
  if (deskewed == nullptr || scan.points.empty() ||
      scan.epoch.session.empty() || scan.epoch.generation == 0 ||
      scan.frame_id.empty() || scan.calibration_id.empty() ||
      !scan.base_from_lidar.matrix().allFinite() ||
      !(scan.base_from_lidar.linear().transpose() *
        scan.base_from_lidar.linear())
           .isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
      std::abs(scan.base_from_lidar.linear().determinant() - 1.0) > 1e-6 ||
      !scan.base_from_lidar.matrix().row(3).isApprox(
          Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), 1e-9)) {
    return {Reason::INVALID_INPUT, "Invalid raw LiDAR scan"};
  }
  LocalState end;
  Result result = lookup(scan.stamp.time, &end);
  if (!result.ok()) {
    return result;
  }
  if (end.epoch != scan.epoch) {
    return {Reason::EPOCH_MISMATCH, "LiDAR scan crosses local epoch"};
  }
  DeskewedScan next;
  next.stamp = scan.stamp;
  next.epoch = scan.epoch;
  next.points_in_end_base.reserve(scan.points.size());
  std::vector<DeskewPointError> next_errors;
  if (errors) next_errors.reserve(scan.points.size());
  const Eigen::Isometry3d end_from_local = Pose(end).inverse();
  for (const auto& point : scan.points) {
    if (!point.position.allFinite() || !std::isfinite(point.time) ||
        point.time <= 0.0) {
      return {Reason::POINT_TIME_MISSING,
              "Raw LiDAR point lacks a valid measurement timestamp"};
    }
    if (point.time > scan.stamp.time) {
      return {Reason::INVALID_INPUT, "LiDAR point timestamp exceeds scan end"};
    }
    LocalState at_point;
    result = lookup(point.time, &at_point);
    if (!result.ok()) {
      return result;
    }
    if (at_point.epoch != scan.epoch) {
      return {Reason::EPOCH_MISMATCH, "LiDAR deskew history changed epoch"};
    }
    next.points_in_end_base.push_back(end_from_local * Pose(at_point) *
                                      scan.base_from_lidar * point.position);
    if (errors) {
      DeskewPointError error;
      error.point_time = point.time;
      const Eigen::Matrix3d point_rotation =
          end_from_local.linear() * at_point.orientation.toRotationMatrix();
      error.point_state.block<3, 3>(0, 0) = end_from_local.linear();
      error.point_state.block<3, 3>(0, 6) =
          -point_rotation * Skew(scan.base_from_lidar * point.position);
      error.end_state.block<3, 3>(0, 0) = -end_from_local.linear();
      error.end_state.block<3, 3>(0, 6) =
          Skew(next.points_in_end_base.back());
      error.raw_point = point_rotation * scan.base_from_lidar.linear();
      if (!error.point_state.allFinite() || !error.end_state.allFinite() ||
          !error.raw_point.allFinite())
        return {Reason::INVALID_INPUT, "LiDAR deskew error map is nonfinite"};
      next_errors.push_back(std::move(error));
    }
  }
  *deskewed = std::move(next);
  if (errors) *errors = std::move(next_errors);
  return {};
}

std::size_t LocalLio::VoxelHash::operator()(const VoxelKey& key) const {
  std::size_t seed = std::hash<int>()(key.x);
  seed ^= std::hash<int>()(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<int>()(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

LocalLio::VoxelKey LocalLio::Key(const Eigen::Vector3d& point) const {
  return {static_cast<int>(std::floor(point.x() / config_.lidar_voxel_size)),
          static_cast<int>(std::floor(point.y() / config_.lidar_voxel_size)),
          static_cast<int>(std::floor(point.z() / config_.lidar_voxel_size))};
}

void LocalLio::Reset(const Epoch& epoch) {
  epoch_ = epoch;
  scans_.clear();
  planes_.clear();
  observed_pose_basis_.resize(0, 6);
}

std::vector<uint64_t> LocalLio::geometry_source_sequences() const {
  std::vector<uint64_t> sequences;
  sequences.reserve(scans_.size());
  for (const auto& scan : scans_) {
    sequences.push_back(scan.sequence);
  }
  return sequences;
}

std::vector<std::pair<uint64_t, std::size_t>>
LocalLio::stored_point_counts() const {
  std::vector<std::pair<uint64_t, std::size_t>> counts;
  for (const auto& scan : scans_)
    counts.emplace_back(scan.sequence, scan.points.size());
  return counts;
}

Result LocalLio::GeometryErrorMap(
    std::vector<StoredPlaneError>* errors,
    std::map<uint64_t, LocalState>* anchors) const {
  if (!errors)
    return {Reason::INVALID_INPUT, "Stored-geometry error destination is missing"};
  if (planes_.empty())
    return {Reason::GEOMETRY_UNAVAILABLE, "No retained local planes to linearize"};
  std::unordered_map<VoxelKey, std::vector<StoredPlanePointError>, VoxelHash> support;
  std::map<uint64_t, LocalState> next_anchors;
  std::unordered_set<uint64_t> source_sequences;
  for (const auto& scan : scans_) {
    if (scan.sequence == 0 || !source_sequences.insert(scan.sequence).second)
      return {Reason::CORRELATION_UNQUALIFIED,
              "Stored geometry has ambiguous raw source identities"};
    if (anchors) next_anchors.emplace(scan.sequence, scan.anchor);
    for (std::size_t index = 0; index < scan.points.size(); ++index) {
      const auto& point = scan.points[index];
      const auto& key = scan.voxel_keys[index];
      if (planes_.find(key) == planes_.end()) continue;
      const Eigen::Vector3d coordinate = point / config_.lidar_voxel_size;
      if ((coordinate - coordinate.array().round().matrix())
              .cwiseAbs().minCoeff() < 1e-8)
        return {Reason::CORRELATION_UNQUALIFIED,
                "Stored geometry lies on a voxel-assignment boundary"};
      StoredPlanePointError item;
      item.source_sequence = scan.sequence;
      item.point_index = index;
      item.point = point;
      item.deskewed_point = scan.anchor.orientation.toRotationMatrix();
      item.anchor_state.block<3, 3>(0, 0).setIdentity();
      item.anchor_state.block<3, 3>(0, 6) =
          -Skew(point - scan.anchor.position) * item.deskewed_point;
      support[key].push_back(std::move(item));
    }
  }
  std::vector<StoredPlaneError> next;
  next.reserve(planes_.size());
  for (const auto& plane : planes_) {
    StoredPlaneError error;
    error.center = plane.second.point;
    error.normal = plane.second.normal;
    error.points = std::move(support.at(plane.first));
    const double count = static_cast<double>(error.points.size());
    Eigen::Matrix3d second = Eigen::Matrix3d::Zero();
    for (const auto& point : error.points)
      second += point.point * point.point.transpose();
    Eigen::Matrix3d covariance =
        second / count - error.center * error.center.transpose();
    covariance = (0.5 * (covariance + covariance.transpose())).eval();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success)
      return {Reason::INVALID_INPUT, "Stored-plane eigendecomposition failed"};
    const auto& values = solver.eigenvalues();
    if (values[1] - values[0] <=
        64 * std::numeric_limits<double>::epsilon() *
            std::max(1.0, values.cwiseAbs().maxCoeff()))
      return {Reason::CORRELATION_UNQUALIFIED,
              "Stored-plane normal has no unique error derivative"};
    for (auto& point : error.points) {
      point.center = Eigen::Matrix3d::Identity() / count;
      const Eigen::Vector3d centered = point.point - error.center;
      for (int column = 1; column < 3; ++column) {
        const Eigen::Vector3d direction = solver.eigenvectors().col(column);
        point.normal +=
            direction *
            (centered.dot(error.normal) * direction.transpose() +
             centered.dot(direction) * error.normal.transpose()) /
            (count * (values[0] - values[column]));
      }
      if (!point.normal.allFinite())
        return {Reason::INVALID_INPUT, "Stored-plane error derivative is nonfinite"};
    }
    next.push_back(std::move(error));
  }
  *errors = std::move(next);
  if (anchors) *anchors = std::move(next_anchors);
  return {};
}

void LocalLio::Rebuild() {
  struct Accumulator {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d second = Eigen::Matrix3d::Zero();
    double anchor_variance = 0.0;
    uint32_t count = 0;
  };
  std::unordered_map<VoxelKey, Accumulator, VoxelHash> accumulators;
  for (const auto& scan : scans_) {
    for (std::size_t index = 0; index < scan.points.size(); ++index) {
      const auto& point = scan.points[index];
      auto& accumulator = accumulators[scan.voxel_keys[index]];
      accumulator.sum += point;
      accumulator.second += point * point.transpose();
      accumulator.anchor_variance =
          std::max(accumulator.anchor_variance, scan.anchor_variance);
      ++accumulator.count;
    }
  }
  planes_.clear();
  for (const auto& item : accumulators) {
    if (item.second.count < config_.lidar_min_points_per_plane) {
      continue;
    }
    const auto& accumulator = item.second;
    const Eigen::Vector3d mean = accumulator.sum / accumulator.count;
    Eigen::Matrix3d covariance =
        accumulator.second / accumulator.count - mean * mean.transpose();
    covariance = (0.5 * (covariance + covariance.transpose())).eval();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success ||
        solver.eigenvalues()(1) <=
            config_.lidar_point_noise_std * config_.lidar_point_noise_std) {
      continue;
    }
    Plane plane;
    plane.point = mean;
    plane.normal = solver.eigenvectors().col(0);
    plane.geometry_variance =
        std::max(solver.eigenvalues()(0), config_.lidar_plane_noise_std *
                                              config_.lidar_plane_noise_std);
    plane.variance = plane.geometry_variance +
        config_.lidar_anchor_variance_scale * accumulator.anchor_variance;
    planes_.emplace(item.first, plane);
    if (planes_.size() >= config_.lidar_max_voxels) {
      break;
    }
  }
}

Result LocalLio::BuildUpdate(const DeskewedScan& scan, const LocalState& state,
                             LioUpdate* update, bool tracking_gain) const {
  std::vector<Eigen::Matrix<double, 1, 15>> rows;
  std::vector<double> innovations;
  std::vector<double> variances;
  std::vector<double> geometry_variances;
  std::vector<LioUpdate::Residual> residuals;
  std::unordered_set<VoxelKey, VoxelHash> used;
  const Eigen::Matrix3d rotation = state.orientation.toRotationMatrix();
  for (std::size_t index = 0; index < scan.points_in_end_base.size(); ++index) {
    const auto& point_in_base = scan.points_in_end_base[index];
    const Eigen::Vector3d point = Pose(state) * point_in_base;
    const VoxelKey key = Key(point);
    const auto found = planes_.find(key);
    if (found == planes_.end() || !used.emplace(key).second) {
      continue;
    }
    const Plane& plane = found->second;
    const double residual = plane.normal.dot(point - plane.point);
    if (std::abs(residual) > config_.lidar_max_correspondence_distance) {
      continue;
    }
    Eigen::Matrix<double, 1, 15> row = Eigen::Matrix<double, 1, 15>::Zero();
    row.block<1, 3>(0, 0) = plane.normal.transpose();
    row.block<1, 3>(0, 6) =
        -plane.normal.transpose() * rotation * Skew(point_in_base);
    const double variance = plane.variance + config_.lidar_point_noise_std *
                                                 config_.lidar_point_noise_std;
    const double innovation_variance =
        (row * state.covariance * row.transpose())(0, 0) + variance;
    if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0 ||
        residual * residual / innovation_variance >
            config_.lidar_innovation_gate) {
      continue;
    }
    rows.push_back(row);
    innovations.push_back(-residual);
    const double geometry_variance =
        plane.geometry_variance +
        config_.lidar_point_noise_std * config_.lidar_point_noise_std;
    variances.push_back(tracking_gain ? geometry_variance : variance);
    geometry_variances.push_back(geometry_variance);
    residuals.push_back({index, plane.point, plane.normal});
  }
  if (rows.size() < config_.lidar_min_correspondences) {
    return {Reason::GEOMETRY_UNAVAILABLE,
            "Insufficient trusted local plane correspondences"};
  }

  Eigen::MatrixXd whitened_pose(rows.size(), 6);
  Eigen::MatrixXd geometry_pose(rows.size(), 6);
  Eigen::MatrixXd whitened_full(rows.size(), 15);
  Eigen::VectorXd whitened_innovation(rows.size());
  for (std::size_t i = 0; i < rows.size(); ++i) {
    whitened_pose.row(i).head<3>() =
        rows[i].block<1, 3>(0, 0) / std::sqrt(variances[i]);
    whitened_pose.row(i).tail<3>() =
        rows[i].block<1, 3>(0, 6) /
        (config_.lidar_rotation_length * std::sqrt(variances[i]));
    whitened_full.row(i) = rows[i] / std::sqrt(variances[i]);
    whitened_innovation(i) = innovations[i] / std::sqrt(variances[i]);
    geometry_pose.row(i) =
        whitened_pose.row(i) * std::sqrt(variances[i] / geometry_variances[i]);
  }
  // Absolute anchor uncertainty weights the correction; it cannot erase the
  // rank of unchanged relative geometry. Neither rank nor a nominal update
  // qualifies the reused planes as independent covariance information.
  Eigen::JacobiSVD<Eigen::MatrixXd> geometry_svd(geometry_pose);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      whitened_pose, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.info() != Eigen::Success || geometry_svd.info() != Eigen::Success) {
    return {Reason::DEGENERATE, "Local geometry SVD failed"};
  }
  uint32_t rank = 0;
  for (int i = 0; i < svd.singularValues().size(); ++i) {
    rank += geometry_svd.singularValues()(i) >=
            config_.lidar_observed_singular_value;
  }
  if (rank == 0) {
    return {Reason::DEGENERATE,
            "Local geometry has no trusted observed direction"};
  }
  if (svd.singularValues()(rank - 1) <=
      std::numeric_limits<double>::epsilon() * svd.singularValues()(0)) {
    return {Reason::DEGENERATE,
            "Local geometry statistical weighting is numerically singular"};
  }
  Matrix6d metric = Matrix6d::Identity();
  metric.bottomRightCorner<3, 3>() *= config_.lidar_rotation_length;
  const Eigen::MatrixXd raw_basis =
      svd.matrixV().leftCols(rank).transpose() * metric;
  Eigen::VectorXd observed_standard_deviation =
      svd.singularValues().head(rank).cwiseInverse();
  Eigen::MatrixXd observed_covariance =
      observed_standard_deviation.cwiseProduct(observed_standard_deviation)
          .asDiagonal();
  Eigen::MatrixXd basis;
  Eigen::MatrixXd canonical_covariance;
  const Result canonical = CanonicalizeObservedSubspace(
      raw_basis, observed_covariance, config_.lidar_rotation_length, &basis,
      &canonical_covariance);
  if (!canonical.ok()) {
    return canonical;
  }
  const Eigen::MatrixXd observed_rows =
      svd.matrixU().leftCols(rank).transpose();
  update->observation = observed_rows * whitened_full;
  update->innovation = observed_rows * whitened_innovation;
  update->noise = Eigen::MatrixXd::Identity(rank, rank);
  update->observed_pose_basis = basis;
  update->correspondences = rows.size();
  update->residuals = std::move(residuals);
  update->residual_projection = observed_rows;
  for (std::size_t i = 0; i < variances.size(); ++i)
    update->residual_projection.col(i) /= std::sqrt(variances[i]);
  return {};
}

Result LocalLio::ApplyUpdate(const LioUpdate& update, const LocalState& prior,
                             LocalState* state,
                             Eigen::MatrixXd* applied_gain,
                             const Matrix15d* tracking_prior,
                             Matrix15d* tracking_posterior) const {
  Eigen::Matrix<double, 15, 1> error;
  error.segment<3>(0) = state->position - prior.position;
  error.segment<3>(3) = state->velocity - prior.velocity;
  error.segment<3>(6) =
      LogRotation(prior.orientation.conjugate() * state->orientation);
  error.segment<3>(9) = state->gyro_bias - prior.gyro_bias;
  error.segment<3>(12) = state->accel_bias - prior.accel_bias;
  const Eigen::Vector3d attitude_error = error.segment<3>(6);
  const double squared_angle = attitude_error.squaredNorm();
  double first = 0.5;
  double second = 1.0 / 6.0;
  if (squared_angle < 1e-8) {
    first -= squared_angle / 24.0;
    second -= squared_angle / 120.0;
  } else {
    const double angle = std::sqrt(squared_angle);
    first = (1.0 - std::cos(angle)) / squared_angle;
    second = (angle - std::sin(angle)) / (angle * squared_angle);
  }
  const Eigen::Matrix3d skew = Skew(attitude_error);
  Matrix15d chart = Matrix15d::Identity();
  chart.block<3, 3>(6, 6) =
      Eigen::Matrix3d::Identity() - first * skew + second * skew * skew;
  const Matrix15d covariance =
      chart * (tracking_prior ? *tracking_prior : prior.covariance) *
      chart.transpose();
  const Eigen::Matrix<double, 15, 1> prior_step = chart * error;
  const Eigen::MatrixXd innovation_covariance =
      update.observation * covariance * update.observation.transpose() +
      update.noise;
  Eigen::LDLT<Eigen::MatrixXd> factor(innovation_covariance);
  if (factor.info() != Eigen::Success) {
    return {Reason::INVALID_INPUT, "LiDAR innovation covariance is invalid"};
  }
  const Eigen::MatrixXd gain =
      factor.solve(update.observation * covariance).transpose();
  // Relinearize one observation against its fixed prior, rather than counting
  // the same scan as another independent update on every iteration.
  const Eigen::Matrix<double, 15, 1> correction =
      gain * (update.innovation + update.observation * prior_step) - prior_step;
  LocalState next = *state;
  next.position += correction.segment<3>(0);
  next.velocity += correction.segment<3>(3);
  next.orientation =
      (next.orientation * ExpRotation(correction.segment<3>(6))).normalized();
  next.gyro_bias += correction.segment<3>(9);
  next.accel_bias += correction.segment<3>(12);
  // The local planes reuse earlier local states and raw points. Their exact
  // cross-correlation is not available here, so they may steer the nominal
  // state but cannot reduce the filter covariance as independent anchors.
  next.covariance_model_valid = false;
  next.uncertainty_qualified = false;
  Eigen::Matrix<double, 15, 1> total;
  total.segment<3>(0) = next.position - prior.position;
  total.segment<3>(3) = next.velocity - prior.velocity;
  total.segment<3>(6) =
      LogRotation(prior.orientation.conjugate() * next.orientation);
  total.segment<3>(9) = next.gyro_bias - prior.gyro_bias;
  total.segment<3>(12) = next.accel_bias - prior.accel_bias;
  next.covariance = prior.covariance;
  next.covariance.diagonal() += config_.lidar_correction_variance_scale *
                                total.cwiseProduct(total);
  next.covariance =
      (0.5 * (next.covariance + next.covariance.transpose())).eval();
  if (prior.covariance_square_root) {
    const auto modeled = AdvanceCovarianceSquareRoot(
        prior, Matrix15d::Identity(),
        std::sqrt(config_.lidar_correction_variance_scale) * total.asDiagonal(),
        &next);
    if (!modeled.ok()) return modeled;
  }
  if (!next.position.allFinite() || !next.velocity.allFinite() ||
      !next.orientation.coeffs().allFinite() || !ValidLocalCovariance(next)) {
    return {Reason::INVALID_INPUT, "LiDAR update is not finite SPD"};
  }
  if (tracking_posterior) {
    const Matrix15d residual =
        Matrix15d::Identity() - gain * update.observation;
    Matrix15d reset = Matrix15d::Identity();
    reset.block<3, 3>(6, 6) =
        RightRotationJacobian(correction.segment<3>(6));
    Matrix15d posterior = reset *
        (residual * covariance * residual.transpose() +
         gain * update.noise * gain.transpose()) * reset.transpose();
    posterior = (0.5 * (posterior + posterior.transpose())).eval();
    if (!posterior.allFinite() ||
        Eigen::LLT<Matrix15d>(posterior).info() != Eigen::Success)
      return {Reason::INVALID_INPUT, "LiDAR nominal-gain matrix is not finite SPD"};
    *tracking_posterior = posterior;
  }
  *state = next;
  if (applied_gain) *applied_gain = gain;
  return {};
}

void LocalLio::Insert(const DeskewedScan& scan, const LocalState& state) {
  StoredScan stored;
  stored.epoch = scan.epoch;
  stored.sequence = scan.stamp.sequence;
  stored.anchor = state;
  stored.anchor_variance = state.covariance.block<3, 3>(0, 0).trace() +
                           config_.lidar_voxel_size * config_.lidar_voxel_size *
                               state.covariance.block<3, 3>(6, 6).trace();
  stored.points.reserve(scan.points_in_end_base.size());
  stored.voxel_keys.reserve(scan.points_in_end_base.size());
  for (const auto& point : scan.points_in_end_base) {
    stored.points.push_back(Pose(state) * point);
    stored.voxel_keys.push_back(Key(stored.points.back()));
  }
  scans_.push_back(std::move(stored));
  while (scans_.size() > config_.lidar_max_scans) {
    scans_.pop_front();
  }
  Rebuild();
}

Result LocalLio::Observe(const LidarScan& scan, const StateLookup& lookup,
                         LocalState* state,
                         const LioIterationObserver& observer,
                         Matrix15d* tracking_gain_covariance) {
  if (state == nullptr || !state->valid || scan.epoch != state->epoch) {
    return {Reason::EPOCH_MISMATCH,
            "LiDAR update requires the current local epoch"};
  }
  if (tracking_gain_covariance &&
      (!tracking_gain_covariance->allFinite() ||
       !tracking_gain_covariance->isApprox(tracking_gain_covariance->transpose(), 1e-9) ||
       Eigen::LLT<Matrix15d>(*tracking_gain_covariance).info() != Eigen::Success)) {
    return {Reason::INVALID_INPUT, "Invalid LiDAR nominal-gain prior"};
  }
  if (epoch_ != state->epoch) {
    Reset(state->epoch);
  }
  DeskewedScan deskewed;
  Result result = DeskewLidarScan(scan, lookup, &deskewed);
  if (!result.ok()) {
    return result;
  }
  if (planes_.empty()) {
    Insert(deskewed, *state);
    return {Reason::GEOMETRY_UNAVAILABLE,
            "First local scan only initializes geometry"};
  }
  LocalState candidate = *state;
  LioUpdate update;
  Matrix15d tracking_posterior;
  for (uint32_t iteration = 0; iteration < config_.lidar_max_update_iterations;
       ++iteration) {
    result = BuildUpdate(deskewed, candidate, &update,
                         tracking_gain_covariance != nullptr);
    if (!result.ok()) {
      return result;
    }
    const LocalState before = candidate;
    result = ApplyUpdate(update, *state, &candidate,
                         observer ? &update.applied_gain : nullptr,
                         tracking_gain_covariance,
                         tracking_gain_covariance ? &tracking_posterior : nullptr);
    if (!result.ok()) {
      return result;
    }
    if (observer) observer(update, *state, before, candidate);
  }
  *state = candidate;
  if (tracking_gain_covariance)
    *tracking_gain_covariance = tracking_posterior;
  observed_pose_basis_ = update.observed_pose_basis;
  Insert(deskewed, *state);
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace openloc
