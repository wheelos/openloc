// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/local/local_estimator.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace openloc::localization::unified {
namespace {

std::string DeskewPoint(std::size_t index) {
  return "deskew:" + std::to_string(index);
}

std::string RetainedDeskewPoint(uint64_t sequence, std::size_t index) {
  return "scan_deskew:" + std::to_string(sequence) + ":" + std::to_string(index);
}

Eigen::Matrix<double, 15, 1> Difference(const LocalState& first,
                                     const LocalState& second) {
  Eigen::Matrix<double, 15, 1> difference;
  difference.segment<3>(0) = second.position - first.position;
  difference.segment<3>(3) = second.velocity - first.velocity;
  difference.segment<3>(6) =
      LogRotation(first.orientation.conjugate() * second.orientation);
  difference.segment<3>(9) = second.gyro_bias - first.gyro_bias;
  difference.segment<3>(12) = second.accel_bias - first.accel_bias;
  return difference;
}

}  // namespace

void LocalEstimator::SyncJointMemory() {
  if (!joint_error_.active()) return;
  std::vector<std::string> retained{"internal", "output", "imu:last"};
  for (const auto& entry : internal_history_) {
    retained.push_back(NativeJointError::History(entry.corrected.stamp.sequence));
    if (entry.has_lidar_correction)
      retained.push_back(
          NativeJointError::History(entry.corrected.stamp.sequence, true));
  }
  const auto counts = lio_.stored_point_counts();
  for (auto scan = joint_scans_.begin(); scan != joint_scans_.end();) {
    if (std::none_of(counts.begin(), counts.end(), [&](const auto& count) {
          return count.first == scan->first;
        }))
      scan = joint_scans_.erase(scan);
    else
      ++scan;
  }
  for (const auto& scan : counts)
    for (std::size_t index = 0; index < scan.second; ++index) {
      retained.push_back(NativeJointError::Point(scan.first, index));
      retained.push_back(RetainedDeskewPoint(scan.first, index));
    }
  joint_error_.Retain(retained);
}

Result LocalEstimator::PrepareJointLidarScan(uint64_t sequence,
                                           JointLidarScan* scan) {
  if (!scan || sequence == 0)
    return {Reason::INVALID_INPUT, "Invalid joint LiDAR scan export"};
  if (!joint_error_.active()) return joint_error_.status();
  const auto found = joint_scans_.find(sequence);
  if (found == joint_scans_.end())
    return {Reason::HISTORY_UNAVAILABLE, "No retained original LiDAR scan support"};
  auto status = joint_error_.FinalizeFrontier(output_state_);
  if (!status.ok()) return status;
  JointLidarScan next;
  status = joint_error_.Lookup(output_state_.stamp.time, &next.conditioning_state);
  if (!status.ok()) return status;
  next.reference = found->second.reference;
  next.lidar_revision = lidar_health_.accepted;
  next.points_in_reference_base = found->second.points;
  for (std::size_t index = 0; index < next.points_in_reference_base.size(); ++index) {
    auto loading = joint_error_.Get(RetainedDeskewPoint(sequence, index));
    if (loading.rows() != 3 ||
        loading.cols() != next.conditioning_state.joint_error->covariance.rows())
      return {Reason::CORRELATION_UNQUALIFIED,
              "Original LiDAR scan lost its shared stochastic point support"};
    next.point_jacobians.push_back(std::move(loading));
  }
  *scan = std::move(next);
  return {};
}

Result LocalEstimator::PrepareJointLidar(
    const LidarScan& scan, LidarDeskewErrorMap* deskew,
    std::vector<StoredPlaneError>* planes) {
  auto result = DeskewErrorMap(scan, deskew);
  if (!result.ok()) return result;
  result = lio_.GeometryErrorMap(planes);
  if (!result.ok() && result.reason != Reason::GEOMETRY_UNAVAILABLE)
    return result;
  joint_error_.Set("lidar_prior", joint_error_.Get("internal"));
  for (std::size_t index = 0; index < deskew->points.size(); ++index) {
    joint_error_.Independent(
        "raw_point", Eigen::Matrix3d::Identity() *
                         (config_.lidar_point_noise_std *
                          config_.lidar_point_noise_std),
        "lidar:" + std::to_string(scan.stamp.sequence) + ":" + std::to_string(index));
    if (!joint_error_.active()) return joint_error_.status();
    const auto& point = deskew->points[index];
    Eigen::MatrixXd loading = point.raw_point * joint_error_.Get("raw_point");
    for (const auto& dependency : point.history) {
      const auto& clone = deskew->history[dependency.history_index];
      const auto memory = joint_error_.Get(NativeJointError::History(
          clone.state.stamp.sequence, clone.before_lidar_correction));
      if (memory.rows() != 15)
        return {Reason::CORRELATION_UNQUALIFIED,
                "Deskew dependency has no retained stochastic history clone"};
      loading += dependency.jacobian * memory;
    }
    joint_error_.Set(DeskewPoint(index), loading);
    if (!joint_error_.active()) return joint_error_.status();
  }
  joint_error_.Drop("raw_point");
  return {};
}

void LocalEstimator::JointLidarIteration(
    const LioUpdate& update, const LocalState& prior,
    const LocalState& before, const LocalState& after,
    const LidarDeskewErrorMap& deskew,
    const std::vector<StoredPlaneError>& planes) {
  if (!joint_error_.active()) return;
  const Eigen::MatrixXd current = joint_error_.Get("internal");
  const Eigen::MatrixXd initial = joint_error_.Get("lidar_prior");
  Eigen::MatrixXd residuals(update.residuals.size(), current.cols());
  const Eigen::Matrix3d rotation = before.orientation.toRotationMatrix();
  for (std::size_t index = 0; index < update.residuals.size(); ++index) {
    const auto& residual = update.residuals[index];
    const auto plane = std::find_if(
        planes.begin(), planes.end(), [&](const StoredPlaneError& item) {
          return item.center.isApprox(residual.plane_center, 0) &&
                 item.normal.isApprox(residual.plane_normal, 0);
        });
    if (plane == planes.end()) {
      joint_error_.Fail(
          {Reason::CORRELATION_UNQUALIFIED,
           "Accepted LIO residual has no exact stored-plane stochastic support"});
      return;
    }
    const auto& point_in_base =
        deskew.points[residual.point_index].point_in_end_base;
    const Eigen::Vector3d point = Pose(before) * point_in_base;
    const Eigen::Vector3d coordinate = point / config_.lidar_voxel_size;
    if ((coordinate - coordinate.array().round().matrix())
            .cwiseAbs().minCoeff() < 1e-8) {
      joint_error_.Fail({Reason::CORRELATION_UNQUALIFIED,
                         "Accepted LIO point is on a voxel boundary"});
      return;
    }
    Eigen::Matrix<double, 1, 15> observation =
        Eigen::Matrix<double, 1, 15>::Zero();
    observation.block<1, 3>(0, 0) = plane->normal.transpose();
    observation.block<1, 3>(0, 6) =
        -plane->normal.transpose() * rotation * Skew(point_in_base);
    residuals.row(index) =
        observation * current +
        plane->normal.transpose() * rotation *
            joint_error_.Get(DeskewPoint(residual.point_index));
    for (const auto& support : plane->points) {
      const auto loading = joint_error_.Get(
          NativeJointError::Point(support.source_sequence, support.point_index));
      if (loading.rows() != 3) {
        joint_error_.Fail(
            {Reason::CORRELATION_UNQUALIFIED,
             "Stored plane lost an original point's stochastic memory"});
        return;
      }
      residuals.row(index) +=
          ((point - plane->center).transpose() * support.normal -
           plane->normal.transpose() * support.center) * loading;
    }
  }
  const auto difference = Difference(prior, before);
  const Eigen::Vector3d attitude = difference.segment<3>(6);
  if (attitude.norm() >= std::acos(-1.0) - 1e-6) {
    joint_error_.Fail({Reason::CORRELATION_UNQUALIFIED,
                       "LIO prior chart has a nonunique rotation"});
    return;
  }
  Eigen::MatrixXd difference_error = current - initial;
  difference_error.middleRows(6, 3) =
      InverseRightRotationJacobian(attitude) *
      (current.middleRows(6, 3) -
       ExpRotation(-attitude).toRotationMatrix() * initial.middleRows(6, 3));
  // The declared first-order model freezes the actual accepted gain,
  // whitening/projection and correspondence decisions at each iteration.
  // The SAME raw/plane/prior errors drive every iteration of this scan.
  const Eigen::MatrixXd correction_error =
      update.applied_gain * (-update.residual_projection * residuals +
                            update.observation * difference_error) -
      difference_error;
  Eigen::MatrixXd next = current + correction_error;
  const Eigen::Vector3d correction = Difference(before, after).segment<3>(6);
  next.middleRows(6, 3) =
      ExpRotation(-correction).toRotationMatrix() * current.middleRows(6, 3) +
      RightRotationJacobian(correction) * correction_error.middleRows(6, 3);
  joint_error_.Set("internal", next);
}

void LocalEstimator::StoreJointLidar(
    const LidarScan& scan, const LidarDeskewErrorMap& deskew) {
  if (!joint_error_.active()) return;
  RetainedJointScan retained;
  retained.reference = scan.stamp;
  const Eigen::Matrix3d rotation = state_.orientation.toRotationMatrix();
  for (std::size_t index = 0; index < deskew.points.size(); ++index) {
    retained.points.push_back(deskew.points[index].point_in_end_base);
    joint_error_.Set(RetainedDeskewPoint(scan.stamp.sequence, index),
                     joint_error_.Get(DeskewPoint(index)));
    Eigen::Matrix<double, 3, 15> anchor =
        Eigen::Matrix<double, 3, 15>::Zero();
    anchor.block<3, 3>(0, 0).setIdentity();
    anchor.block<3, 3>(0, 6) =
        -rotation * Skew(deskew.points[index].point_in_end_base);
    joint_error_.Set(
        NativeJointError::Point(scan.stamp.sequence, index),
        anchor * joint_error_.Get("internal") +
            rotation * joint_error_.Get(DeskewPoint(index)));
    if (!joint_error_.active()) return;
  }
  joint_scans_[scan.stamp.sequence] = std::move(retained);
}

}  // namespace openloc::localization::unified
