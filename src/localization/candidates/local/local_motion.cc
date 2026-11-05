// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "candidates/local/local_motion.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <utility>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"
#include "Eigen/QR"

namespace openloc {
namespace localization {
namespace unified {
namespace {

bool ValidEndpoint(const LocalState& state) {
  return state.valid && !state.epoch.session.empty() &&
         state.epoch.generation != 0 && !state.stamp.clock_id.empty() &&
         state.stamp.sequence != 0 && std::isfinite(state.stamp.time) &&
         state.stamp.time > 0.0 && state.position.allFinite() &&
         state.orientation.coeffs().allFinite() &&
         std::abs(state.orientation.norm() - 1.0) < 1e-9 &&
         state.covariance.allFinite() &&
         state.covariance.isApprox(state.covariance.transpose(), 1e-9) &&
         Eigen::LLT<Matrix15d>(state.covariance).info() == Eigen::Success;
}

}  // namespace

bool ValidLocalCovariance(const LocalState& state) {
  if (!state.covariance.allFinite() ||
      !state.covariance.isApprox(state.covariance.transpose(), 1e-9))
    return false;
  if (!state.covariance_square_root)
    return Eigen::LLT<Matrix15d>(state.covariance).info() == Eigen::Success;
  const auto& root = *state.covariance_square_root;
  if (!root.allFinite()) return false;
  Matrix15d projected = root * root.transpose();
  projected = (0.5 * (projected + projected.transpose())).eval();
  if (!(projected.array() == state.covariance.array()).all()) return false;
  Matrix15d normalized = root;
  for (int row = 0; row < 15; ++row) {
    const double scale = root.row(row).norm();
    if (!std::isfinite(scale) || scale == 0) return false;
    normalized.row(row) /= scale;
  }
  return Eigen::ColPivHouseholderQR<Matrix15d>(normalized).rank() == 15;
}

Result AdvanceCovarianceSquareRoot(const LocalState& previous,
                                   const Matrix15d& transition,
                                   const Eigen::MatrixXd& additive_root,
                                   LocalState* next) {
  if (!next || !transition.allFinite() || !additive_root.allFinite() ||
      additive_root.rows() != 15 || !ValidLocalCovariance(previous))
    return {Reason::INVALID_INPUT, "Invalid square-root covariance input"};
  Matrix15d root;
  if (previous.covariance_square_root) {
    root = *previous.covariance_square_root;
  } else {
    root = Eigen::LLT<Matrix15d>(previous.covariance).matrixL();
  }
  root = (transition * root).eval();
  if (!additive_root.isZero(0)) {
    Eigen::MatrixXd joined(15, 15 + additive_root.cols());
    joined.leftCols<15>() = root;
    joined.rightCols(additive_root.cols()) = additive_root;
    const Eigen::HouseholderQR<Eigen::MatrixXd> qr(joined.transpose());
    root = qr.matrixQR()
               .topRows<15>()
               .template triangularView<Eigen::Upper>()
               .transpose();
  }
  LocalState candidate = *next;
  candidate.covariance_square_root = root;
  candidate.covariance = root * root.transpose();
  candidate.covariance =
      (0.5 * (candidate.covariance + candidate.covariance.transpose())).eval();
  if (!ValidLocalCovariance(candidate))
    return {Reason::INVALID_INPUT,
            "Square-root covariance lost finite resolved rank"};
  next->covariance = candidate.covariance;
  next->covariance_square_root = std::move(candidate.covariance_square_root);
  return {};
}

Eigen::Matrix3d RightRotationJacobian(const Eigen::Vector3d& rotation) {
  const double squared = rotation.squaredNorm();
  const double angle = std::sqrt(squared);
  const Eigen::Matrix3d cross = Skew(rotation);
  const double a = squared < 1e-8
                       ? 0.5 - squared / 24.0
                       : (1 - std::cos(angle)) / squared;
  const double b = squared < 1e-8
                       ? 1.0 / 6.0 - squared / 120.0
                       : (angle - std::sin(angle)) / (squared * angle);
  return Eigen::Matrix3d::Identity() - a * cross + b * cross * cross;
}

Eigen::Matrix3d InverseRightRotationJacobian(const Eigen::Vector3d& rotation) {
  const double angle = rotation.norm();
  const Eigen::Matrix3d cross = Skew(rotation);
  const double coefficient =
      angle < 1e-4
          ? 1.0 / 12.0 + angle * angle / 720.0
          : (1.0 - 0.5 * angle / std::tan(0.5 * angle)) / (angle * angle);
  return Eigen::Matrix3d::Identity() + 0.5 * cross + coefficient * cross * cross;
}

Result RotationInterpolationJacobians(
    const Eigen::Quaterniond& first, const Eigen::Quaterniond& second,
    double alpha, Eigen::Matrix3d* first_error, Eigen::Matrix3d* second_error) {
  if (!first_error || !second_error || !std::isfinite(alpha) ||
      alpha < 0 || alpha > 1 || !first.coeffs().allFinite() ||
      !second.coeffs().allFinite() || std::abs(first.norm() - 1) > 1e-9 ||
      std::abs(second.norm() - 1) > 1e-9)
    return {Reason::INVALID_INPUT, "Invalid rotation interpolation error request"};
  const Eigen::Vector3d rotation = LogRotation(first.conjugate() * second);
  const double angle = rotation.norm();
  if (angle >= std::acos(-1.0) - 1e-6)
    return {Reason::CORRELATION_UNQUALIFIED,
            "History interpolation crosses a nonunique rotation branch"};
  const Eigen::Matrix3d end =
      alpha * RightRotationJacobian(alpha * rotation) *
      InverseRightRotationJacobian(rotation);
  const Eigen::Matrix3d start =
      ExpRotation(-alpha * rotation).toRotationMatrix() -
      end * ExpRotation(-rotation).toRotationMatrix();
  if (!start.allFinite() || !end.allFinite())
    return {Reason::INVALID_INPUT, "Rotation interpolation error map is nonfinite"};
  *first_error = start;
  *second_error = end;
  return {};
}

Eigen::MatrixXd JointErrorCovariance(const LocalState& state) {
  return state.joint_error ? state.joint_error->covariance
                           : Eigen::MatrixXd(state.covariance);
}

Eigen::MatrixXd JointOutputJacobian(const LocalState& state) {
  return state.joint_error ? state.joint_error->output_jacobian
                           : Eigen::MatrixXd(Matrix15d::Identity());
}

Result ValidateJointLocalError(const LocalState& state) {
  if (!state.joint_error) return {};
  const auto& model = *state.joint_error;
  const auto dimension = model.covariance.rows();
  if (dimension < 15 || model.covariance.cols() != dimension ||
      model.output_jacobian.rows() != 15 ||
      model.output_jacobian.cols() != dimension ||
      !model.covariance.allFinite() || !model.output_jacobian.allFinite() ||
      !model.covariance.isApprox(model.covariance.transpose(), 1e-9) ||
      Eigen::LLT<Eigen::MatrixXd>(model.covariance).info() != Eigen::Success) {
    return {Reason::INVALID_INPUT,
            "Joint local error requires a finite full-rank sufficient basis"};
  }
  const Matrix15d projected = model.output_jacobian * model.covariance *
                              model.output_jacobian.transpose();
  if (!projected.allFinite() || !projected.isApprox(state.covariance, 1e-9)) {
    return {Reason::INVALID_INPUT,
            "Joint error projection disagrees with published covariance"};
  }
  return {};
}

Result ValidateJointMotion(const LocalState& start, const LocalState& end,
                           const MotionIncrement& motion) {
  auto valid = ValidateJointLocalError(start);
  if (valid.ok()) valid = ValidateJointLocalError(end);
  if (!valid.ok()) return valid;
  return ValidateJointMotionWithValidatedLocalErrors(start, end, motion);
}

Result ValidateJointMotionWithValidatedLocalErrors(
    const LocalState& start, const LocalState& end,
    const MotionIncrement& motion) {
  if (!start.joint_error && !end.joint_error) {
    if (motion.joint_cross_covariance.size() != 0)
      return {Reason::INVALID_INPUT, "Unexpected joint motion without joint states"};
    return {};
  }
  const auto previous = JointErrorCovariance(start);
  const auto current = JointErrorCovariance(end);
  const auto& cross = motion.joint_cross_covariance;
  if (cross.rows() != previous.rows() || cross.cols() != current.rows() ||
      !cross.allFinite())
    return {Reason::INVALID_INPUT, "Joint motion endpoint dimensions differ"};
  const Matrix15d projected =
      JointOutputJacobian(start) * cross * JointOutputJacobian(end).transpose();
  if (!projected.allFinite() ||
      !projected.isApprox(motion.cross_covariance, 1e-9))
    return {Reason::INVALID_INPUT,
            "Joint motion projection disagrees with published cross-covariance"};
  const Eigen::MatrixXd transition = previous.llt().solve(cross).transpose();
  Eigen::MatrixXd noise = current - transition * cross;
  noise = (0.5 * (noise + noise.transpose())).eval();
  const auto scale = current.diagonal().cwiseSqrt().cwiseInverse().eval();
  const Eigen::MatrixXd normalized =
      scale.asDiagonal() * noise * scale.asDiagonal();
  if (!transition.allFinite() || !normalized.allFinite())
    return {Reason::INVALID_INPUT, "Joint motion conditioning overflow"};
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(
      normalized, Eigen::EigenvaluesOnly);
  if (spectrum.info() != Eigen::Success ||
      !spectrum.eigenvalues().allFinite())
    return {Reason::INVALID_INPUT, "Joint motion decomposition failed"};
  const double tolerance = 64 * std::numeric_limits<double>::epsilon() *
      std::max(1.0, spectrum.eigenvalues().cwiseAbs().maxCoeff());
  const double minimum_eigenvalue = spectrum.eigenvalues().minCoeff();
  if (minimum_eigenvalue < -tolerance) {
    std::ostringstream message;
    message << std::setprecision(17)
            << "Joint motion conditional noise is not PSD (minimum eigenvalue "
            << minimum_eigenvalue << ", tolerance " << tolerance << ")";
    return {Reason::INVALID_INPUT, message.str()};
  }
  return {};
}

void RelativeMotionJacobians(const Eigen::Isometry3d& start_pose,
                             const Eigen::Isometry3d& delta,
                             Matrix6x15d* start, Matrix6x15d* end) {
  start->setZero();
  end->setZero();
  start->block<3, 3>(0, 0) = -start_pose.linear().transpose();
  start->block<3, 3>(0, 6) = Skew(delta.translation());
  end->block<3, 3>(0, 0) = start_pose.linear().transpose();
  start->block<3, 3>(3, 6) = -delta.linear().transpose();
  end->block<3, 3>(3, 6).setIdentity();
}

Result ComputeMotionIncrement(const LocalState& start, const LocalState& end,
                              const Matrix15d& cross_covariance,
                              const std::vector<SourceSampleId>& sources,
                              MotionIncrement* output) {
  if (output == nullptr || !ValidEndpoint(start) || !ValidEndpoint(end) ||
      !cross_covariance.allFinite()) {
    return {Reason::INVALID_INPUT, "Invalid local motion endpoint/covariance"};
  }
  if (!start.covariance_model_valid || !end.covariance_model_valid) {
    return {Reason::CORRELATION_UNQUALIFIED,
            "Local motion endpoint covariance model is invalid"};
  }
  if (start.epoch != end.epoch) {
    return {Reason::EPOCH_MISMATCH, "Motion cannot cross an ODOM reset"};
  }
  if (start.stamp.clock_id != end.stamp.clock_id) {
    return {Reason::CLOCK_INVALID, "Motion endpoints use different clocks"};
  }
  if (end.stamp.time <= start.stamp.time ||
      end.stamp.sequence <= start.stamp.sequence) {
    return {Reason::TIMESTAMP_REGRESSION, "Motion endpoints must advance"};
  }
  bool has_start_imu = false;
  bool has_end_imu = false;
  std::set<std::pair<std::string, uint64_t>> identities;
  for (const auto& source : sources) {
    if (source.source.empty() || source.sequence == 0 ||
        !identities.emplace(source.source, source.sequence).second) {
      return {Reason::INVALID_INPUT, "Invalid/duplicate motion source sample"};
    }
    if (source.source == "imu") {
      has_start_imu = has_start_imu || source.sequence == start.stamp.sequence;
      has_end_imu = has_end_imu || source.sequence == end.stamp.sequence;
    }
  }
  if (!has_start_imu || !has_end_imu) {
    return {Reason::INVALID_INPUT, "Motion lacks its actual endpoint IMU samples"};
  }

  Eigen::Matrix<double, 30, 30> joint;
  joint.topLeftCorner<15, 15>() = start.covariance;
  joint.bottomRightCorner<15, 15>() = end.covariance;
  joint.topRightCorner<15, 15>() = cross_covariance;
  joint.bottomLeftCorner<15, 15>() = cross_covariance.transpose();
  // Check the joint distribution, not merely its two positive marginals.
  const Eigen::Matrix<double, 30, 1> scale =
      joint.diagonal().cwiseSqrt().cwiseInverse();
  const Eigen::Matrix<double, 30, 30> correlation =
      scale.asDiagonal() * joint * scale.asDiagonal();
  if (!correlation.allFinite()) {
    return {Reason::INVALID_INPUT, "Local joint covariance scale overflow"};
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 30, 30>> spectrum(
      correlation, Eigen::EigenvaluesOnly);
  if (spectrum.info() != Eigen::Success ||
      spectrum.eigenvalues().minCoeff() < -1e-9) {
    return {Reason::INVALID_INPUT, "Local cross-covariance is not jointly PSD"};
  }

  MotionIncrement next;
  next.epoch = end.epoch;
  next.start = start.stamp;
  next.end = end.stamp;
  next.start_pose = Pose(start);
  next.delta = Pose(start).inverse() * Pose(end);
  next.start_covariance = start.covariance;
  next.end_covariance = end.covariance;
  next.cross_covariance = cross_covariance;
  next.sources = sources;
  next.correlation_group =
      end.epoch.session + ":" + std::to_string(end.epoch.generation);

  Eigen::Matrix<double, 6, 30> jacobian =
      Eigen::Matrix<double, 6, 30>::Zero();
  Matrix6x15d first;
  Matrix6x15d second;
  RelativeMotionJacobians(next.start_pose, next.delta, &first, &second);
  jacobian.leftCols<15>() = first;
  jacobian.rightCols<15>() = second;
  next.covariance = jacobian * joint * jacobian.transpose();
  next.covariance =
      (0.5 * (next.covariance + next.covariance.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Matrix6d> relative(next.covariance);
  if (!next.delta.matrix().allFinite() || !next.covariance.allFinite() ||
      relative.info() != Eigen::Success ||
      relative.eigenvalues().minCoeff() <
          -1e-9 * std::max(1.0, next.covariance.norm())) {
    return {Reason::INVALID_INPUT, "Relative motion covariance is not PSD"};
  }
  *output = next;
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace openloc
