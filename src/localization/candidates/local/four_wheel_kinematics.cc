// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.
#include "candidates/local/four_wheel_kinematics.h"

#include <cmath>

#include "Eigen/Cholesky"

namespace openloc {
namespace localization {
namespace unified {

Result FourWheelKinematics(const FourWheelGeometry& geometry,
                           const FourWheelInput& input,
                           PlanarMotionObservation* output) {
  constexpr double half_pi = 1.57079632679489661923;
  if (output == nullptr || !std::isfinite(geometry.wheelbase) ||
      geometry.wheelbase <= 0.0 ||
      !geometry.reference_from_center.allFinite() ||
      !std::isfinite(geometry.max_front_angle) ||
      !std::isfinite(geometry.max_rear_angle) ||
      geometry.max_front_angle <= 0.0 || geometry.max_front_angle >= half_pi ||
      geometry.max_rear_angle <= 0.0 || geometry.max_rear_angle >= half_pi) {
    return {Reason::CONFIG_INVALID, "Invalid four-wheel steering geometry"};
  }
  if (!std::isfinite(input.center_forward_speed) ||
      !std::isfinite(input.front_angle) || !std::isfinite(input.rear_angle) ||
      std::abs(input.front_angle) > geometry.max_front_angle ||
      std::abs(input.rear_angle) > geometry.max_rear_angle ||
      !input.covariance.allFinite() ||
      !input.covariance.isApprox(input.covariance.transpose(), 1e-12)) {
    return {Reason::INVALID_INPUT, "Invalid four-wheel steering observation"};
  }
  Eigen::LLT<Eigen::Matrix3d> factor(input.covariance);
  if (factor.info() != Eigen::Success) {
    return {Reason::INVALID_INPUT, "Steering input covariance must be SPD"};
  }
  const double front = std::tan(input.front_angle);
  const double rear = std::tan(input.rear_angle);
  const double speed = input.center_forward_speed;
  const double x = geometry.reference_from_center.x();
  const double y = geometry.reference_from_center.y();
  const double curvature = (front - rear) / geometry.wheelbase;
  const double lateral_ratio = 0.5 * (front + rear) + x * curvature;
  PlanarMotionObservation next;
  next.twist = Eigen::Vector3d(speed * (1.0 - y * curvature),
                               speed * lateral_ratio, speed * curvature);
  const double front_derivative = 1.0 + front * front;
  const double rear_derivative = 1.0 + rear * rear;
  Eigen::Matrix3d jacobian;
  jacobian << 1.0 - y * curvature,
      -y * speed * front_derivative / geometry.wheelbase,
      y * speed * rear_derivative / geometry.wheelbase, lateral_ratio,
      speed * front_derivative * (0.5 + x / geometry.wheelbase),
      speed * rear_derivative * (0.5 - x / geometry.wheelbase), curvature,
      speed * front_derivative / geometry.wheelbase,
      -speed * rear_derivative / geometry.wheelbase;
  next.covariance = jacobian * input.covariance * jacobian.transpose();
  next.covariance =
      (0.5 * (next.covariance + next.covariance.transpose())).eval();
  if (!next.twist.allFinite() || !next.covariance.allFinite()) {
    return {Reason::INVALID_INPUT, "Non-finite four-wheel motion conversion"};
  }
  *output = next;
  return {};
}

Result CounterSteeringMeanWheelKinematics(const FourWheelGeometry& geometry,
                                          double track,
                                          const MeanWheelSteeringInput& input,
                                          PlanarMotionObservation* output) {
  if (output == nullptr || !std::isfinite(track) || track <= 0.0) {
    return {Reason::CONFIG_INVALID, "Invalid wheel-center track or output"};
  }
  if (!std::isfinite(input.mean_rolling_speed) ||
      !std::isfinite(input.steering_percent) ||
      std::abs(input.steering_percent) > 100.0 ||
      !input.covariance.allFinite() ||
      !input.covariance.isApprox(input.covariance.transpose(), 1e-12)) {
    return {Reason::INVALID_INPUT, "Invalid mean-wheel steering input"};
  }
  Eigen::LLT<Eigen::Matrix2d> factor(input.covariance);
  if (factor.info() != Eigen::Success) {
    return {Reason::INVALID_INPUT, "Mean-wheel input covariance must be SPD"};
  }
  FourWheelInput probe;
  probe.front_angle = input.steering_percent * geometry.max_front_angle / 100.0;
  probe.rear_angle = -input.steering_percent * geometry.max_rear_angle / 100.0;
  probe.center_forward_speed = 1.0;
  probe.covariance.setIdentity();
  PlanarMotionObservation unit;
  const Result result = FourWheelKinematics(geometry, probe, &unit);
  if (!result.ok()) {
    return result;
  }
  const double front = std::tan(probe.front_angle);
  const double rear = std::tan(probe.rear_angle);
  const double df = (1.0 + front * front) * geometry.max_front_angle / 100.0;
  const double dr = -(1.0 + rear * rear) * geometry.max_rear_angle / 100.0;
  const double curvature = (front - rear) / geometry.wheelbase;
  const double lateral = 0.5 * (front + rear);
  const double dk = (df - dr) / geometry.wheelbase;
  const double db = 0.5 * (df + dr);
  double ratio = 0.0;
  double derivative = 0.0;
  for (double x : {-0.5 * geometry.wheelbase, 0.5 * geometry.wheelbase}) {
    for (double y : {-0.5 * track, 0.5 * track}) {
      const double vx = 1.0 - curvature * y;
      const double vy = lateral + curvature * x;
      // This signed-speed contract excludes an inner wheel reversing in place.
      if (vx <= 0.0) {
        return {Reason::INVALID_INPUT,
                "Mean-wheel model excludes inner-wheel reversal"};
      }
      const double magnitude = std::hypot(vx, vy);
      ratio += 0.25 * magnitude;
      derivative += 0.25 * (vx * (-dk * y) + vy * (db + dk * x)) / magnitude;
    }
  }
  const double center_speed = input.mean_rolling_speed / ratio;
  const Eigen::Vector3d unit_derivative(
      -geometry.reference_from_center.y() * dk,
      db + geometry.reference_from_center.x() * dk, dk);
  Eigen::Matrix<double, 3, 2> jacobian;
  jacobian.col(0) = unit.twist / ratio;
  jacobian.col(1) =
      center_speed * (unit_derivative - unit.twist * derivative / ratio);
  PlanarMotionObservation next;
  next.twist = center_speed * unit.twist;
  next.covariance = jacobian * input.covariance * jacobian.transpose();
  next.covariance =
      (0.5 * (next.covariance + next.covariance.transpose())).eval();
  if (!next.twist.allFinite() || !next.covariance.allFinite()) {
    return {Reason::INVALID_INPUT, "Non-finite mean-wheel conversion"};
  }
  *output = next;
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace openloc
