// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "openloc/localization/global/marker_observation.h"

#include "gtest/gtest.h"

namespace openloc {
namespace localization {
namespace {

struct Fixture {
  KnownMarker marker;
  MarkerDetection detection;
  MarkerExtrinsics extrinsics;
  Fixture() {
    marker.id = "door-7";
    marker.reference = {"site", "v1", "calibration"};
    marker.map_content_id = "survey-v1";
    marker.map_frame = {"map"};
    marker.marker_frame = {"door-7"};
    marker.map_from_marker.translation = {4, 3, 2};
    marker.map_from_marker.rotation = Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ());
    detection.context.measurement_time = {100, ClockDomain::kSensorMonotonic};
    detection.context.receive_time = {120, ClockDomain::kSensorMonotonic};
    detection.context.source = "camera-marker";
    detection.context.reference = marker.reference;
    detection.context.map_content_id = marker.map_content_id;
    detection.context.map_frame = marker.map_frame;
    detection.context.base_frame = {"base"};
    detection.marker_id = marker.id;
    detection.marker_frame = marker.marker_frame;
    detection.sensor_frame = {"camera"};
    detection.sensor_from_marker.translation = {1, -2, 3};
    detection.sensor_from_marker.rotation =
        Eigen::AngleAxisd(-.3, Eigen::Vector3d::UnitY());
    extrinsics.calibration_id = "calibration";
    extrinsics.base_frame = {"base"};
    extrinsics.sensor_frame = detection.sensor_frame;
    extrinsics.base_from_sensor.translation = {.2, .4, .1};
    extrinsics.base_from_sensor.rotation =
        Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitX());
  }
};

Eigen::Isometry3d Transform(const Pose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = pose.rotation.toRotationMatrix();
  result.translation() = pose.translation;
  return result;
}

TEST(MarkerObservationTest, CompositionAndCorrelatedCovarianceMatchFiniteDifferences) {
  Fixture f;
  MarkerJointCovariance loading = MarkerJointCovariance::Identity() * .02;
  loading.block<3, 3>(0, 6).setIdentity();
  loading.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * .01;
  const MarkerJointCovariance covariance = loading * loading.transpose();
  GlobalMapObservation output;
  ASSERT_TRUE(BuildMarkerObservation(f.marker, f.detection, f.extrinsics,
                                     covariance, &output).ok());
  const auto expected = Transform(f.marker.map_from_marker) *
      Transform(f.detection.sensor_from_marker).inverse() *
      Transform(f.extrinsics.base_from_sensor).inverse();
  EXPECT_TRUE(Transform(output.map_from_base).matrix().isApprox(expected.matrix(), 1e-12));
  Eigen::Matrix<double, 6, 18> jacobian;
  const double step = 1e-6;
  for (int col = 0; col < 18; ++col) {
    Eigen::Matrix<double, 6, 2> difference;
    for (int sign : {-1, 1}) {
      Fixture perturbed = f;
      Pose* pose = col < 6 ? &perturbed.marker.map_from_marker :
                   col < 12 ? &perturbed.detection.sensor_from_marker :
                              &perturbed.extrinsics.base_from_sensor;
      const int axis = col % 6;
      if (axis < 3) pose->translation[axis] += sign * step;
      else pose->rotation = Eigen::Quaterniond(Eigen::AngleAxisd(
          sign * step, Eigen::Vector3d::Unit(axis - 3))) * pose->rotation;
      GlobalMapObservation changed;
      ASSERT_TRUE(BuildMarkerObservation(perturbed.marker, perturbed.detection,
                                         perturbed.extrinsics, covariance, &changed).ok());
      const Eigen::AngleAxisd rotation(
          changed.map_from_base.rotation * output.map_from_base.rotation.conjugate());
      difference.col(sign < 0 ? 0 : 1).head<3>() = changed.map_from_base.translation;
      difference.col(sign < 0 ? 0 : 1).tail<3>() = rotation.axis() * rotation.angle();
    }
    jacobian.col(col) = (difference.col(1) - difference.col(0)) / (2 * step);
  }
  EXPECT_TRUE(output.covariance.isApprox(jacobian * covariance * jacobian.transpose(), 1e-8));
  EXPECT_EQ(f.detection.context.source, output.context.source);
  EXPECT_EQ(100, output.context.measurement_time.nanoseconds);
  EXPECT_FALSE(output.recovery);
}

TEST(MarkerObservationTest, ExplicitFixedSurveyAndExtrinsicsPreserveAmbiguity) {
  Fixture f;
  f.detection.ambiguous = true;
  MarkerJointCovariance covariance = MarkerJointCovariance::Zero();
  covariance.block<6, 6>(6, 6).setIdentity();
  GlobalMapObservation result;
  ASSERT_TRUE(BuildMarkerObservation(f.marker, f.detection, f.extrinsics,
                                     covariance, &result).ok());
  EXPECT_TRUE(result.ambiguous);
  EXPECT_GT(result.covariance.diagonal().minCoeff(), 0);
}

TEST(MarkerObservationTest, BadIdentityFramesClocksAndCovarianceLeaveOutputUnchanged) {
  for (int fault = 0; fault < 8; ++fault) {
    Fixture f;
    MarkerJointCovariance covariance = MarkerJointCovariance::Identity();
    if (fault == 0) f.detection.marker_id = "unknown";
    if (fault == 1) f.extrinsics.sensor_frame = {"other"};
    if (fault == 2) f.extrinsics.calibration_id = "other";
    if (fault == 3) f.detection.context.receive_time.clock_domain = ClockDomain::kUnixEpoch;
    if (fault == 4) f.detection.context.measurement_time.nanoseconds = 200;
    if (fault == 5) covariance(0, 0) = -1;
    if (fault == 6) covariance(0, 1) = .1;
    if (fault == 7) covariance.setZero();
    GlobalMapObservation output;
    output.context.source = "sentinel";
    EXPECT_FALSE(BuildMarkerObservation(f.marker, f.detection, f.extrinsics,
                                        covariance, &output).ok()) << fault;
    EXPECT_EQ("sentinel", output.context.source);
  }
}

}  // namespace
}  // namespace localization
}  // namespace openloc
