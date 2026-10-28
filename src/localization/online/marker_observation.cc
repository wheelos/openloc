// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "openloc/localization/global/marker_observation.h"

#include <algorithm>

#include "Eigen/Eigenvalues"
#include "openloc/localization/online/validation.h"

namespace openloc {
namespace localization {
namespace {

Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d result;
  result << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return result;
}

Eigen::Isometry3d Transform(const Pose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = pose.rotation.toRotationMatrix();
  result.translation() = pose.translation;
  return result;
}

}  // namespace

Status BuildMarkerObservation(const KnownMarker& marker,
                              const MarkerDetection& detection,
                              const MarkerExtrinsics& extrinsics,
                              const MarkerJointCovariance& covariance,
                              GlobalMapObservation* output) {
  const auto invalid = [](const char* text) {
    return Status(StatusCode::kInvalidArgument, text);
  };
  if (!output || marker.id.empty() || detection.context.source.empty() ||
      marker.reference.id.empty() || marker.reference.version.empty() ||
      marker.map_content_id.empty() || extrinsics.calibration_id.empty())
    return invalid("marker identity, source, reference and calibration required");
  if (marker.id != detection.marker_id)
    return Status(StatusCode::kMapReferenceMismatch, "marker identity mismatch");
  if (marker.reference != detection.context.reference ||
      marker.map_content_id != detection.context.map_content_id ||
      marker.reference.calibration_id != extrinsics.calibration_id)
    return Status(StatusCode::kMapReferenceMismatch,
                  "marker reference/content/calibration mismatch");
  for (const auto& frame : {marker.map_frame, marker.marker_frame,
                            detection.context.map_frame,
                            detection.context.base_frame,
                            detection.marker_frame, detection.sensor_frame,
                            extrinsics.base_frame, extrinsics.sensor_frame}) {
    const auto status = ValidateFrameId(frame);
    if (!status.ok()) return status;
  }
  if (marker.map_frame.value != detection.context.map_frame.value ||
      marker.marker_frame.value != detection.marker_frame.value ||
      detection.sensor_frame.value != extrinsics.sensor_frame.value ||
      detection.context.base_frame.value != extrinsics.base_frame.value)
    return Status(StatusCode::kFrameMismatch, "marker transform frames mismatch");
  auto status = ValidateTimestamp(detection.context.measurement_time);
  if (status.ok()) status = ValidateTimestamp(detection.context.receive_time);
  if (!status.ok()) return status;
  if (detection.context.measurement_time.clock_domain !=
      detection.context.receive_time.clock_domain)
    return Status(StatusCode::kClockDomainMismatch, "marker clock mismatch");
  if (detection.context.measurement_time.nanoseconds >
      detection.context.receive_time.nanoseconds)
    return Status(StatusCode::kInvalidTimestamp, "marker measurement is future");
  for (const auto& pose : {marker.map_from_marker, detection.sensor_from_marker,
                           extrinsics.base_from_sensor}) {
    status = ValidatePose(pose);
    if (!status.ok()) return status;
  }
  if (!covariance.allFinite() ||
      !covariance.isApprox(covariance.transpose(), 1e-10))
    return invalid("marker joint covariance must be finite symmetric PSD");
  Eigen::SelfAdjointEigenSolver<MarkerJointCovariance> eigen(covariance);
  if (eigen.info() != Eigen::Success ||
      eigen.eigenvalues().minCoeff() <
          -1e-12 * std::max(1.0, covariance.norm()))
    return invalid("marker joint covariance is not PSD");
  const auto a = Transform(marker.map_from_marker);
  const auto b = Transform(detection.sensor_from_marker);
  const auto c = Transform(extrinsics.base_from_sensor);
  const Eigen::Isometry3d pose = a * b.inverse() * c.inverse();
  const Eigen::Matrix3d map_from_sensor = a.linear() * b.linear().transpose();
  const Eigen::Matrix3d map_from_base = pose.linear();
  Eigen::Matrix<double, 6, 18> jacobian =
      Eigen::Matrix<double, 6, 18>::Zero();
  jacobian.block<3, 3>(0, 0).setIdentity();
  jacobian.block<3, 3>(0, 3) =
      -Skew(pose.translation() - a.translation());
  jacobian.block<3, 3>(3, 3).setIdentity();
  jacobian.block<3, 3>(0, 6) = -map_from_sensor;
  jacobian.block<3, 3>(0, 9) =
      -map_from_sensor * Skew(b.translation() + c.linear().transpose() * c.translation());
  jacobian.block<3, 3>(3, 9) = -map_from_sensor;
  jacobian.block<3, 3>(0, 12) = -map_from_base;
  jacobian.block<3, 3>(0, 15) = -map_from_base * Skew(c.translation());
  jacobian.block<3, 3>(3, 15) = -map_from_base;
  GlobalMapObservation result;
  result.context = detection.context;
  result.map_from_base.translation = pose.translation();
  result.map_from_base.rotation = Eigen::Quaterniond(pose.linear()).normalized();
  result.covariance = jacobian * covariance * jacobian.transpose();
  result.covariance =
      (0.5 * (result.covariance + result.covariance.transpose())).eval();
  status = ValidateCovariance(result.covariance, 6);
  if (!status.ok()) return status;
  result.covariance_source = "marker_joint_pose_propagation.v1:" + marker.id;
  result.ambiguous = detection.ambiguous;
  *output = std::move(result);
  return Status::Ok();
}

}  // namespace localization
}  // namespace openloc
