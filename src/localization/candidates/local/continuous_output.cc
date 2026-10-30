// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/local/continuous_output.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"
#include "candidates/local/local_motion.h"

namespace openloc {
namespace localization {
namespace unified {
namespace {

Eigen::Vector3d Limited(const Eigen::Vector3d& value, double limit) {
  const double norm = value.norm();
  if (norm <= limit || norm == 0.0) {
    return value;
  }
  return value * (limit / norm);
}

struct NominalStep {
  LocalState state;
  Eigen::Vector3d memory;
  Eigen::Vector3d translation_error;
  Eigen::Vector3d rotation_error;
  Eigen::Vector3d velocity_error;
  Eigen::Vector3d memory_step;
  Eigen::Vector3d translation_correction;
  Eigen::Vector3d rotation_correction;
  Eigen::Vector3d velocity_correction;
  Eigen::Vector3d target_memory;
};

NominalStep AdvanceNominal(const LocalConfig& config,
                          const LocalState& previous_internal,
                          const LocalState& internal, const LocalState& output,
                          const Eigen::Vector3d& memory, double dt) {
  NominalStep result;
  auto& next = result.state;
  next = output;
  next.stamp = internal.stamp;
  next.covariance_model_valid =
      output.covariance_model_valid && previous_internal.covariance_model_valid &&
      internal.covariance_model_valid;
  const Eigen::Isometry3d propagated =
      Pose(output) * Pose(previous_internal).inverse() * Pose(internal);
  next.position = propagated.translation();
  next.orientation = Eigen::Quaterniond(propagated.linear()).normalized();
  const Eigen::Vector3d propagated_velocity =
      (next.position - output.position) / dt;
  next.velocity += internal.velocity - previous_internal.velocity;
  next.gyro_bias += internal.gyro_bias - previous_internal.gyro_bias;
  next.accel_bias += internal.accel_bias - previous_internal.accel_bias;
  result.translation_error = internal.position - next.position;
  result.rotation_error =
      LogRotation(next.orientation.conjugate() * internal.orientation);
  result.velocity_error = internal.velocity - next.velocity;
  result.target_memory = Limited(result.translation_error / dt,
                                config.lidar_max_translation_correction_rate);
  result.memory_step = Limited(
      result.target_memory - memory,
      config.lidar_max_velocity_correction_acceleration * dt);
  result.memory = memory + result.memory_step;
  result.translation_correction = 0.5 * (memory + result.memory) * dt;
  result.rotation_correction =
      Limited(result.rotation_error, config.lidar_max_rotation_correction_rate * dt);
  next.position += result.translation_correction;
  next.orientation =
      (next.orientation * ExpRotation(result.rotation_correction)).normalized();
  const double transfer = std::clamp(dt, 0.0, 1.0);
  next.gyro_bias += transfer * (internal.gyro_bias - next.gyro_bias);
  next.accel_bias += transfer * (internal.accel_bias - next.accel_bias);
  next.velocity = (next.position - output.position) / dt;
  next.angular_velocity =
      LogRotation(output.orientation.conjugate() * next.orientation) / dt;
  result.velocity_correction = next.velocity - propagated_velocity;
  next.valid = true;
  return result;
}

Eigen::Matrix3d LimiterJacobian(const Eigen::Vector3d& value, double limit) {
  const double norm = value.norm();
  if (norm <= limit || norm == 0) return Eigen::Matrix3d::Identity();
  const Eigen::Vector3d direction = value / norm;
  return (limit / norm) *
         (Eigen::Matrix3d::Identity() - direction * direction.transpose());
}

}  // namespace

void ContinuousOutput::Reset(const LocalState& internal) {
  state_ = internal;
  active_ = internal.valid;
  pose_correction_velocity_.setZero();
  correction_budget_exceeded_ = false;
  correction_status_ = {};
  last_step_.reset();
}

Result ContinuousOutput::Advance(const LocalState& previous_internal,
                                 const LocalState& internal,
                                 const Matrix15d& error_transition,
                                 double dt,
                                 const Matrix15d* process_covariance) {
  if (!internal.valid || !std::isfinite(dt) || dt <= 0.0 ||
      internal.epoch != state_.epoch ||
      previous_internal.epoch != internal.epoch ||
      !error_transition.allFinite()) {
    return {Reason::INVALID_INPUT,
            "Continuous output needs a valid same-epoch forward state"};
  }
  if (!active_) {
    Reset(internal);
    return {};
  }

  const Matrix15d start_covariance = state_.covariance;
  const auto nominal = AdvanceNominal(config_, previous_internal, internal,
                                      state_, pose_correction_velocity_, dt);
  LocalState next = nominal.state;
  const auto& translation_error = nominal.translation_error;
  const auto& rotation_error = nominal.rotation_error;
  const auto& velocity_error = nominal.velocity_error;
  correction_budget_exceeded_ =
      translation_error.norm() >
          config_.lidar_max_translation_correction ||
      rotation_error.norm() > config_.lidar_max_rotation_correction ||
      velocity_error.norm() > config_.lidar_max_velocity_correction;
  const auto& pose_correction_acceleration_step = nominal.memory_step;
  correction_budget_exceeded_ =
      correction_budget_exceeded_ ||
      pose_correction_acceleration_step.norm() >
          config_.lidar_max_velocity_correction_acceleration * dt +
              std::numeric_limits<double>::epsilon();
  correction_status_ = {};
  if (correction_budget_exceeded_) {
    std::ostringstream detail;
    detail << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "Local geometry correction exceeds the declared output "
              "envelope: translation_m="
           << translation_error.norm()
           << " rotation_rad=" << rotation_error.norm()
           << " velocity_mps=" << velocity_error.norm()
           << " correction_acceleration_mps2="
           << pose_correction_acceleration_step.norm() / dt
           << " translation_limit_m="
           << config_.lidar_max_translation_correction
           << " rotation_limit_rad=" << config_.lidar_max_rotation_correction
           << " velocity_limit_mps=" << config_.lidar_max_velocity_correction
           << " correction_acceleration_limit_mps2="
           << config_.lidar_max_velocity_correction_acceleration;
    correction_status_ = {Reason::CORRECTION_BUDGET_EXCEEDED, detail.str()};
  }
  next.valid = true;
  // Subtracting large propagated marginals can destroy small PSD noise terms.
  Matrix15d process_noise;
  if (process_covariance) {
    process_noise = *process_covariance;
  } else {
    process_noise =
        internal.covariance -
        error_transition * previous_internal.covariance *
            error_transition.transpose();
  }
  if (!process_noise.allFinite()) {
    return {Reason::INVALID_INPUT, "Continuous-output process covariance is nonfinite"};
  }
  process_noise =
      (0.5 * (process_noise + process_noise.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Matrix15d> noise_spectrum(process_noise);
  if (noise_spectrum.info() != Eigen::Success) {
    return {Reason::INVALID_INPUT,
            "Continuous-output process covariance decomposition failed"};
  }
  if (noise_spectrum.eigenvalues().minCoeff() <
      -config_.lidar_process_noise_psd_tolerance) {
    return {Reason::INVALID_INPUT,
            "Local transition implies non-PSD process covariance"};
  }
  const Eigen::VectorXd eigenvalues =
      noise_spectrum.eigenvalues().cwiseMax(0.0);
  process_noise = noise_spectrum.eigenvectors() *
                  eigenvalues.asDiagonal() *
                  noise_spectrum.eigenvectors().transpose();
  next.covariance =
      error_transition * start_covariance * error_transition.transpose() +
      process_noise;
  const double scale = config_.lidar_correction_variance_scale;
  next.covariance.diagonal().segment<3>(0) +=
      scale * nominal.translation_correction.cwiseProduct(nominal.translation_correction);
  next.covariance.diagonal().segment<3>(3) +=
      scale * nominal.velocity_correction.cwiseProduct(nominal.velocity_correction);
  next.covariance.diagonal().segment<3>(6) +=
      scale * nominal.rotation_correction.cwiseProduct(nominal.rotation_correction);
  next.covariance =
      (0.5 * (next.covariance + next.covariance.transpose())).eval();
  if (state_.covariance_square_root || process_noise.isZero(0)) {
    Eigen::Matrix<double, 15, 30> additive_root;
    additive_root.leftCols<15>() =
        noise_spectrum.eigenvectors() * eigenvalues.cwiseSqrt().asDiagonal();
    additive_root.rightCols<15>().setZero();
    additive_root.block<3, 3>(0, 15) =
        std::sqrt(scale) * nominal.translation_correction.asDiagonal();
    additive_root.block<3, 3>(3, 18) =
        std::sqrt(scale) * nominal.velocity_correction.asDiagonal();
    additive_root.block<3, 3>(6, 21) =
        std::sqrt(scale) * nominal.rotation_correction.asDiagonal();
    const auto modeled = AdvanceCovarianceSquareRoot(state_, error_transition,
                                                     additive_root, &next);
    if (!modeled.ok()) return modeled;
  }
  if (!next.position.allFinite() ||
      !next.velocity.allFinite() || !next.orientation.coeffs().allFinite()) {
    return {Reason::INVALID_INPUT,
            "Continuous output update is nonfinite"};
  }
  if (!ValidLocalCovariance(next)) {
    return {Reason::INVALID_INPUT,
            "Continuous output covariance is not SPD"};
  }
  last_step_ = StepInput{previous_internal, internal, state_,
                         pose_correction_velocity_, dt};
  state_ = next;
  pose_correction_velocity_ = nominal.memory;
  return {};
}

Result ContinuousOutput::LinearizeLastStep(
    ContinuousOutputErrorTransition* transition) const {
  if (transition == nullptr)
    return {Reason::INVALID_INPUT, "Output transition destination is missing"};
  if (!last_step_)
    return {Reason::HISTORY_UNAVAILABLE, "No committed continuous-output step"};
  const auto& input = *last_step_;
  const auto nominal = AdvanceNominal(config_, input.previous_internal,
                                      input.internal, input.output,
                                      input.memory, input.dt);
  const double roundoff = 64 * std::numeric_limits<double>::epsilon();
  const double rate_roundoff =
      roundoff *
      (input.previous_internal.position.norm() + input.internal.position.norm() +
       input.output.position.norm() + nominal.translation_error.norm()) /
      input.dt;
  const auto at_boundary = [](double norm, double limit, double uncertainty) {
    return std::abs(norm - limit) <= uncertainty;
  };
  // Analytic derivatives need a resolved branch, not a finite-difference probe
  // radius. Account for cancellation in the positions before dividing by dt.
  if (at_boundary(nominal.translation_error.norm() / input.dt,
                  config_.lidar_max_translation_correction_rate,
                  rate_roundoff +
                      roundoff * config_.lidar_max_translation_correction_rate) ||
      at_boundary((nominal.target_memory - input.memory).norm(),
                  config_.lidar_max_velocity_correction_acceleration * input.dt,
                  rate_roundoff +
                      roundoff * (nominal.target_memory.norm() +
                                  input.memory.norm() +
                                  config_.lidar_max_velocity_correction_acceleration *
                                      input.dt)) ||
      at_boundary(nominal.rotation_error.norm(),
                  config_.lidar_max_rotation_correction_rate * input.dt,
                  roundoff * (1.0 + nominal.rotation_error.norm() +
                              config_.lidar_max_rotation_correction_rate *
                                  input.dt)) ||
      nominal.rotation_error.norm() >= std::acos(-1.0) - 1e-5) {
    return {Reason::CORRELATION_UNQUALIFIED,
            "Output limiter/rotation boundary has no unique local error Jacobian"};
  }
  ContinuousOutputErrorTransition next;
  next.epoch = input.internal.epoch;
  next.start = input.output.stamp;
  next.end = input.internal.stamp;
  next.previous_internal_state = input.previous_internal;
  next.current_internal_state = input.internal;
  next.previous_output_state = input.output;
  next.previous_correction_velocity = input.memory;
  next.current_correction_velocity = nominal.memory;
  using Matrix3x48 = Eigen::Matrix<double, 3, 48>;
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3d output_rotation =
      input.output.orientation.toRotationMatrix();
  const Eigen::Matrix3d previous_rotation =
      input.previous_internal.orientation.toRotationMatrix();
  const Eigen::Matrix3d current_rotation =
      input.internal.orientation.toRotationMatrix();
  const Eigen::Matrix3d translation_rotation =
      output_rotation * previous_rotation.transpose();
  const Eigen::Matrix3d relative_rotation =
      current_rotation.transpose() * previous_rotation;
  const Eigen::Vector3d relative_translation =
      previous_rotation.transpose() *
      (input.internal.position - input.previous_internal.position);
  Matrix3x48 propagated_position = Matrix3x48::Zero();
  propagated_position.block<3, 3>(0, 0) = -translation_rotation;
  propagated_position.block<3, 3>(0, 15) = translation_rotation;
  propagated_position.block<3, 3>(0, 30) = identity;
  propagated_position.block<3, 3>(0, 6) =
      output_rotation * Skew(relative_translation);
  propagated_position.block<3, 3>(0, 36) =
      -output_rotation * Skew(relative_translation);
  Matrix3x48 propagated_rotation = Matrix3x48::Zero();
  propagated_rotation.block<3, 3>(0, 6) = -relative_rotation;
  propagated_rotation.block<3, 3>(0, 21) = identity;
  propagated_rotation.block<3, 3>(0, 36) = relative_rotation;
  Matrix3x48 position_error = -propagated_position;
  position_error.block<3, 3>(0, 15) += identity;
  Matrix3x48 old_memory = Matrix3x48::Zero();
  old_memory.block<3, 3>(0, 45) = identity;
  const Matrix3x48 memory =
      old_memory +
      LimiterJacobian(
          nominal.target_memory - input.memory,
          config_.lidar_max_velocity_correction_acceleration * input.dt) *
          (LimiterJacobian(
               nominal.translation_error / input.dt,
               config_.lidar_max_translation_correction_rate) *
               position_error / input.dt -
           old_memory);
  const Matrix3x48 position =
      propagated_position + 0.5 * input.dt * (old_memory + memory);
  Matrix3x48 velocity = position / input.dt;
  velocity.block<3, 3>(0, 30) -= identity / input.dt;
  Matrix3x48 current_attitude = Matrix3x48::Zero();
  current_attitude.block<3, 3>(0, 21) = identity;
  const Matrix3x48 rotation_error =
      InverseRightRotationJacobian(nominal.rotation_error) *
      (current_attitude -
       ExpRotation(-nominal.rotation_error).toRotationMatrix() *
           propagated_rotation);
  const Matrix3x48 rotation =
      ExpRotation(-nominal.rotation_correction).toRotationMatrix() *
          propagated_rotation +
      RightRotationJacobian(nominal.rotation_correction) *
          LimiterJacobian(
              nominal.rotation_error,
              config_.lidar_max_rotation_correction_rate * input.dt) *
          rotation_error;
  Eigen::Matrix<double, 18, 48> jacobian =
      Eigen::Matrix<double, 18, 48>::Zero();
  jacobian.block<3, 48>(0, 0) = position;
  jacobian.block<3, 48>(3, 0) = velocity;
  jacobian.block<3, 48>(6, 0) = rotation;
  jacobian.block<3, 48>(15, 0) = memory;
  const double retained = 1.0 - std::clamp(input.dt, 0.0, 1.0);
  for (int bias : {9, 12}) {
    jacobian.block<3, 3>(bias, bias) = -retained * identity;
    jacobian.block<3, 3>(bias, 15 + bias) = identity;
    jacobian.block<3, 3>(bias, 30 + bias) = retained * identity;
  }
  if (!jacobian.allFinite())
    return {Reason::INVALID_INPUT, "Continuous-output Jacobian is nonfinite"};
  next.previous_internal = jacobian.leftCols<15>();
  next.current_internal = jacobian.middleCols<15>(15);
  next.previous_output = jacobian.rightCols<18>();
  *transition = std::move(next);
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace openloc
