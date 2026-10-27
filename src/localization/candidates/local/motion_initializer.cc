// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/local/motion_initializer.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "Eigen/SVD"

namespace openloc {
namespace localization {
namespace unified {
namespace {

bool Positive(double value) { return std::isfinite(value) && value > 0.0; }

bool Rigid(const Eigen::Isometry3d& pose) {
  return pose.matrix().allFinite() &&
         (pose.linear().transpose() * pose.linear())
             .isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
         std::abs(pose.linear().determinant() - 1.0) < 1e-6 &&
         pose.matrix().row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0),
                                       1e-9);
}

ImuSample At(const std::vector<ImuSample>& imu, double time) {
  const auto upper = std::lower_bound(
      imu.begin(), imu.end(), time,
      [](const ImuSample& sample, double t) { return sample.stamp.time < t; });
  if (upper->stamp.time == time) return *upper;
  const auto lower = std::prev(upper);
  const double a =
      (time - lower->stamp.time) / (upper->stamp.time - lower->stamp.time);
  ImuSample result;
  result.stamp.time = time;
  result.acceleration =
      (1.0 - a) * lower->acceleration + a * upper->acceleration;
  result.angular_velocity =
      (1.0 - a) * lower->angular_velocity + a * upper->angular_velocity;
  return result;
}

std::vector<double> IntervalTimes(const std::vector<ImuSample>& imu,
                                  double start, double end) {
  std::vector<double> times{start};
  for (const auto& sample : imu) {
    if (sample.stamp.time > start && sample.stamp.time < end)
      times.push_back(sample.stamp.time);
  }
  times.push_back(end);
  return times;
}

Eigen::Quaterniond IntegratedRotation(const std::vector<ImuSample>& imu,
                                      double start, double end,
                                      const Eigen::Vector3d& bias) {
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  const auto times = IntervalTimes(imu, start, end);
  for (std::size_t j = 1; j < times.size(); ++j) {
    const double dt = times[j] - times[j - 1];
    const auto first = At(imu, times[j - 1]);
    const auto second = At(imu, times[j]);
    rotation = (rotation * ExpRotation(dt * (0.5 * (first.angular_velocity +
                                                    second.angular_velocity) -
                                             bias)))
                   .normalized();
  }
  return rotation;
}

}  // namespace

Result InitializeFromRelativeMotion(const MotionInitializationConfig& c,
                                    const std::vector<RelativeLidarPose>& poses,
                                    const std::vector<ImuSample>& imu,
                                    MotionInitialization* output,
                                    MotionInitializationDiagnostics* diagnostics) {
  if (diagnostics) *diagnostics = {};
  for (double value :
       {c.gravity, c.minimum_duration, c.maximum_duration, c.maximum_imu_gap,
        c.position_residual_scale, c.velocity_residual_scale,
        c.maximum_normalized_residual, c.minimum_scaled_singular_value,
        c.maximum_gravity_error, c.maximum_accel_bias, c.maximum_gyro_bias,
        c.maximum_angular_residual, c.maximum_stationary_speed,
        c.maximum_stationary_angular_speed}) {
    if (!Positive(value))
      return {Reason::CONFIG_INVALID, "Invalid motion initialization budget"};
  }
  if (c.maximum_duration < c.minimum_duration)
    return {Reason::CONFIG_INVALID, "Invalid initialization time window"};
  if (output == nullptr || poses.size() < 4 || imu.size() < 2)
    return {Reason::HISTORY_UNAVAILABLE,
            "Insufficient relative-motion evidence"};
  for (std::size_t i = 0; i < poses.size(); ++i) {
    if (!Positive(poses[i].time) || !Rigid(poses[i].reference_from_base) ||
        (i && poses[i].time <= poses[i - 1].time))
      return {Reason::INVALID_INPUT, "Invalid relative LiDAR pose sequence"};
  }
  for (std::size_t i = 0; i < imu.size(); ++i) {
    if (!Positive(imu[i].stamp.time) || !imu[i].acceleration.allFinite() ||
        !imu[i].angular_velocity.allFinite() ||
        (i && imu[i].stamp.time <= imu[i - 1].stamp.time))
      return {Reason::INVALID_INPUT, "Invalid initialization IMU sequence"};
    if (i && imu[i].stamp.time - imu[i - 1].stamp.time > c.maximum_imu_gap)
      return {Reason::IMU_GAP, "Initialization cannot span an IMU gap"};
  }
  const double duration = poses.back().time - poses.front().time;
  if (duration < c.minimum_duration || duration > c.maximum_duration)
    return {Reason::HISTORY_UNAVAILABLE,
            "Initialization duration outside window"};
  if (imu.front().stamp.time > poses.front().time ||
      imu.back().stamp.time < poses.back().time)
    return {Reason::HISTORY_UNAVAILABLE, "Relative poses require IMU brackets"};

  const Eigen::Index count = poses.size();
  const Eigen::Index columns = 3 * count + 6;
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(6 * (count - 1), columns);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(a.rows());
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  // Fit rotation increments, not instantaneous constant angular rates:
  // motion within a scan interval need not be constant.
  for (int iteration = 0; iteration < 6; ++iteration) {
    Eigen::MatrixXd jacobian(3 * (count - 1), 3);
    Eigen::VectorXd residual(3 * (count - 1));
    for (Eigen::Index i = 0; i + 1 < count; ++i) {
      const double start = poses[i].time;
      const double end = poses[i + 1].time;
      const Eigen::Quaterniond observed(
          poses[i].reference_from_base.linear().transpose() *
          poses[i + 1].reference_from_base.linear());
      const auto error = [&](const Eigen::Vector3d& bias) -> Eigen::Vector3d {
        return LogRotation(
                   IntegratedRotation(imu, start, end, bias).conjugate() *
                   observed) /
               (end - start);
      };
      residual.segment<3>(3 * i) = error(gyro_bias);
      for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d step = Eigen::Vector3d::Zero();
        step[axis] = 1e-6;
        jacobian.block<3, 1>(3 * i, axis) =
            (error(gyro_bias + step) - error(gyro_bias - step)) / 2e-6;
      }
    }
    const Eigen::JacobiSVD<Eigen::MatrixXd> rotation_solver(
        jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (rotation_solver.info() != Eigen::Success ||
        rotation_solver.singularValues().minCoeff() < 1e-6)
      return {Reason::DEGENERATE, "Gyro bias alignment is unobservable"};
    const Eigen::Vector3d step = rotation_solver.solve(-residual);
    if (!step.allFinite())
      return {Reason::INVALID_INPUT, "Nonfinite gyro bias alignment"};
    gyro_bias += step;
    if (step.norm() < 1e-8) break;
  }
  double angular_squared = 0.0;
  double maximum_speed = 0.0;
  double maximum_angular_speed = 0.0;
  Eigen::Vector3d integrated_force = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> stationary_forces;
  for (Eigen::Index i = 0; i + 1 < count; ++i) {
    const double t0 = poses[i].time;
    const double t1 = poses[i + 1].time;
    const double dt = t1 - t0;
    const Eigen::Quaterniond q0(poses[i].reference_from_base.linear());
    const Eigen::Quaterniond q1(poses[i + 1].reference_from_base.linear());
    maximum_speed = std::max(
        maximum_speed,
        (poses[i + 1].reference_from_base.translation() -
         poses[i].reference_from_base.translation()).norm() / dt);
    maximum_angular_speed =
        std::max(maximum_angular_speed,
                 LogRotation(q0.conjugate() * q1).norm() / dt);
    const auto times = IntervalTimes(imu, t0, t1);
    Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
    Eigen::Vector3d beta = Eigen::Vector3d::Zero();
    Eigen::Matrix3d jp = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d jv = Eigen::Matrix3d::Zero();
    Eigen::Quaterniond rotation = q0;
    for (std::size_t j = 1; j < times.size(); ++j) {
      const double h = times[j] - times[j - 1];
      const auto s0 = At(imu, times[j - 1]);
      const auto s1 = At(imu, times[j]);
      const Eigen::Matrix3d r0 = rotation.toRotationMatrix();
      rotation =
          (rotation *
           ExpRotation(h * (0.5 * (s0.angular_velocity + s1.angular_velocity) -
                            gyro_bias)))
              .normalized();
      const Eigen::Matrix3d r1 = rotation.toRotationMatrix();
      const Eigen::Vector3d f0 = r0 * s0.acceleration;
      const Eigen::Vector3d f1 = r1 * s1.acceleration;
      if (c.allow_lidar_stationary_initialization) {
        stationary_forces.push_back(f0);
        stationary_forces.push_back(f1);
      }
      // Exact integrals for linearly interpolated world specific force/R.
      alpha += beta * h + h * h * (f0 / 3.0 + f1 / 6.0);
      jp += jv * h + h * h * (r0 / 3.0 + r1 / 6.0);
      beta += 0.5 * h * (f0 + f1);
      jv += 0.5 * h * (r0 + r1);
    }
    integrated_force += beta;
    angular_squared +=
        LogRotation(rotation.conjugate() * q1).squaredNorm() / dt;
    const Eigen::Index row = 6 * i;
    a.block<3, 3>(row, 3 * i) = dt * Eigen::Matrix3d::Identity();
    a.block<3, 3>(row, 3 * count) = 0.5 * dt * dt * Eigen::Matrix3d::Identity();
    a.block<3, 3>(row, 3 * count + 3) = -jp;
    b.segment<3>(row) = poses[i + 1].reference_from_base.translation() -
                        poses[i].reference_from_base.translation() - alpha;
    a.block<3, 3>(row + 3, 3 * i) = -Eigen::Matrix3d::Identity();
    a.block<3, 3>(row + 3, 3 * (i + 1)) = Eigen::Matrix3d::Identity();
    a.block<3, 3>(row + 3, 3 * count) = -dt * Eigen::Matrix3d::Identity();
    a.block<3, 3>(row + 3, 3 * count + 3) = jv;
    b.segment<3>(row + 3) = beta;
    a.middleRows(row, 3) /= c.position_residual_scale;
    b.segment<3>(row) /= c.position_residual_scale;
    a.middleRows(row + 3, 3) /= c.velocity_residual_scale;
    b.segment<3>(row + 3) /= c.velocity_residual_scale;
  }
  if (c.allow_lidar_stationary_initialization) {
    const Eigen::Vector3d mean_force = integrated_force / duration;
    double maximum_force_residual = 0.0;
    for (const auto& force : stationary_forces)
      maximum_force_residual =
          std::max(maximum_force_residual, (force - mean_force).norm());
    if (diagnostics) {
      diagnostics->maximum_lidar_speed = maximum_speed;
      diagnostics->maximum_lidar_angular_speed = maximum_angular_speed;
      diagnostics->maximum_stationary_force_residual = maximum_force_residual;
    }
    const double gravity_error = std::abs(mean_force.norm() - c.gravity);
    const double angular_rms = std::sqrt(angular_squared / duration);
    if (maximum_speed <= c.maximum_stationary_speed &&
        maximum_angular_speed <= c.maximum_stationary_angular_speed &&
        maximum_force_residual <= c.maximum_gravity_error &&
        gravity_error <= c.maximum_gravity_error &&
        gyro_bias.norm() <= c.maximum_gyro_bias &&
        angular_rms <= c.maximum_angular_residual) {
      MotionInitialization next;
      next.time = poses.back().time;
      const Eigen::Quaterniond level = Eigen::Quaterniond::FromTwoVectors(
          mean_force, Eigen::Vector3d(0.0, 0.0, c.gravity));
      next.orientation =
          (level * Eigen::Quaterniond(poses.back().reference_from_base.linear()))
              .normalized();
      next.gyro_bias = gyro_bias;
      next.angular_residual_rms = angular_rms;
      next.lidar_stationary_initialization = true;
      if (diagnostics) {
        diagnostics->lidar_stationary_initialization = true;
        diagnostics->gravity_error = gravity_error;
        diagnostics->gyro_bias_norm = gyro_bias.norm();
        diagnostics->angular_residual_rms = angular_rms;
      }
      *output = next;
      return {};
    }
  }
  Eigen::VectorXd scales(columns);
  for (Eigen::Index col = 0; col < columns; ++col) {
    scales[col] = a.col(col).norm();
    if (scales[col] <= 1e-12)
      return {Reason::DEGENERATE, "Unobserved initialization parameter"};
  }
  const Eigen::MatrixXd scaled = a * scales.cwiseInverse().asDiagonal();
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      scaled, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (diagnostics && svd.info() == Eigen::Success &&
      svd.singularValues().size() == columns) {
    diagnostics->linear_system_evaluated = true;
    diagnostics->minimum_scaled_singular_value =
        svd.singularValues().tail(1)[0];
  }
  if (svd.info() != Eigen::Success || svd.singularValues().size() < columns ||
      svd.singularValues().tail(1)[0] < c.minimum_scaled_singular_value)
    return {Reason::DEGENERATE,
            "Motion cannot separate gravity, velocity and accelerometer bias"};
  const Eigen::VectorXd solution =
      scales.cwiseInverse().asDiagonal() * svd.solve(b);
  const Eigen::Vector3d gravity = solution.segment<3>(3 * count);
  const Eigen::Vector3d accel_bias = solution.tail<3>();
  const double angular_rms = std::sqrt(angular_squared / duration);
  const double rms = (a * solution - b).norm() / std::sqrt(a.rows());
  if (diagnostics) {
    diagnostics->solution_available =
        solution.allFinite() && gyro_bias.allFinite() &&
        std::isfinite(rms) && std::isfinite(angular_rms);
    diagnostics->gravity_error = std::abs(gravity.norm() - c.gravity);
    diagnostics->accel_bias_norm = accel_bias.norm();
    diagnostics->gyro_bias_norm = gyro_bias.norm();
    diagnostics->normalized_residual_rms = rms;
    diagnostics->angular_residual_rms = angular_rms;
  }
  if (!solution.allFinite() || !gyro_bias.allFinite() ||
      std::abs(gravity.norm() - c.gravity) > c.maximum_gravity_error ||
      accel_bias.norm() > c.maximum_accel_bias ||
      gyro_bias.norm() > c.maximum_gyro_bias ||
      rms > c.maximum_normalized_residual ||
      angular_rms > c.maximum_angular_residual)
    return {Reason::INNOVATION_REJECTED,
            "Motion initialization exceeds budgets: gravity_error_mps2=" +
                std::to_string(std::abs(gravity.norm() - c.gravity)) +
                " accel_bias_mps2=" + std::to_string(accel_bias.norm()) +
                " gyro_bias_radps=" + std::to_string(gyro_bias.norm()) +
                " normalized_rms=" + std::to_string(rms) +
                " angular_rms_radps=" + std::to_string(angular_rms)};
  MotionInitialization next;
  next.time = poses.back().time;
  // Establish only a local gauge: gravity down, latest position at the origin.
  const Eigen::Quaterniond level = Eigen::Quaterniond::FromTwoVectors(
      gravity, Eigen::Vector3d(0.0, 0.0, -c.gravity));
  next.orientation =
      (level * Eigen::Quaterniond(poses.back().reference_from_base.linear()))
          .normalized();
  next.velocity = level * solution.segment<3>(3 * (count - 1));
  next.gyro_bias = gyro_bias;
  next.accel_bias = accel_bias;
  next.minimum_scaled_singular_value = svd.singularValues().tail(1)[0];
  next.normalized_residual_rms = rms;
  next.angular_residual_rms = angular_rms;
  *output = next;
  return {};
}
}  // namespace unified
}  // namespace localization
}  // namespace openloc
