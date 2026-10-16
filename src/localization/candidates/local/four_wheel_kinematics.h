// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.
#pragma once

#include "candidates/common/types.h"

namespace openloc {
namespace localization {
namespace unified {

struct FourWheelGeometry {
  double wheelbase = 0.0;
  double max_front_angle = 0.0;
  double max_rear_angle = 0.0;
  // Motion reference relative to the midpoint of the two axle centers, FLU.
  Eigen::Vector2d reference_from_center = Eigen::Vector2d::Zero();
};

struct FourWheelInput {
  // Signed center body-X speed, NOT a wheel's rolling speed.
  double center_forward_speed = 0.0;
  // Actual equivalent axle steering angles, radians, positive left.
  double front_angle = 0.0;
  double rear_angle = 0.0;
  // Joint covariance of [center_forward_speed, front_angle, rear_angle].
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
};

struct PlanarMotionObservation {
  // [body-X velocity, body-Y velocity, body-Z angular rate] at the reference.
  Eigen::Vector3d twist = Eigen::Vector3d::Zero();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
};

// Planar rigid-body, ideal rolling two-axle model. Does not qualify slip,
// timestamps, individual wheel geometry, or gyro measurement uncertainty.
Result FourWheelKinematics(const FourWheelGeometry& geometry,
                           const FourWheelInput& input,
                           PlanarMotionObservation* output);

struct MeanWheelSteeringInput {
  // Arithmetic mean of four signed rolling speeds, m/s, forward positive.
  double mean_rolling_speed = 0.0;
  double steering_percent = 0.0;
  // Joint covariance of [mean_rolling_speed, steering_percent].
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
};

// Symmetric counter-phase axle steering; ideal rolling at four rectangular
// wheel-center positions. Track is wheel-center spacing, NOT vehicle width.
// Front angle = percent * max_front_angle / 100; rear angle has opposite sign.
Result CounterSteeringMeanWheelKinematics(const FourWheelGeometry& geometry,
                                          double track,
                                          const MeanWheelSteeringInput& input,
                                          PlanarMotionObservation* output);

}  // namespace unified
}  // namespace localization
}  // namespace openloc
