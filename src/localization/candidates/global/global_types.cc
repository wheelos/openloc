// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "candidates/global/global_types.h"

#include <cmath>

#include "Eigen/Cholesky"
#include "Eigen/LU"

namespace openloc {
namespace localization {
namespace unified {

bool ValidGlobalPose(const Eigen::Isometry3d& pose) {
  return pose.matrix().allFinite() &&
         (pose.linear().transpose() * pose.linear())
             .isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
         std::abs(pose.linear().determinant() - 1.0) < 1e-6 &&
         pose.matrix().row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0),
                                       1e-9);
}

bool ValidGlobalObservationGeometry(const GlobalObservation& observation) {
  switch (observation.kind) {
    case GlobalObservation::Kind::HEADING:
      return std::isfinite(observation.heading_rad) &&
             std::isfinite(observation.heading_variance) &&
             observation.heading_variance > 0.0 &&
             observation.direction_in_base.allFinite() &&
             std::abs(observation.direction_in_base.norm() - 1.0) < 1e-6;
    case GlobalObservation::Kind::POSE:
      return ValidGlobalPose(observation.pose) &&
             ValidCovariance(observation.covariance);
    case GlobalObservation::Kind::HORIZONTAL_POSITION: {
      const Eigen::Matrix2d covariance =
          observation.covariance.topLeftCorner<2, 2>();
      return observation.pose.translation().head<2>().allFinite() &&
             observation.point_in_base.allFinite() && covariance.allFinite() &&
             covariance.isApprox(covariance.transpose(), 1e-9) &&
             covariance.llt().info() == Eigen::Success;
    }
    case GlobalObservation::Kind::POSITION: {
      const Eigen::Matrix3d covariance =
          observation.covariance.topLeftCorner<3, 3>();
      return observation.pose.translation().allFinite() &&
             observation.point_in_base.allFinite() && covariance.allFinite() &&
             covariance.isApprox(covariance.transpose(), 1e-9) &&
             covariance.llt().info() == Eigen::Success;
    }
    case GlobalObservation::Kind::PROJECTED_POSE: {
      const auto& projection = observation.projection;
      const auto& covariance = observation.projected_covariance;
      return ValidGlobalPose(observation.pose) && projection.cols() == 6 &&
             projection.rows() > 0 && projection.rows() < 6 &&
             covariance.rows() == projection.rows() &&
             covariance.cols() == projection.rows() && projection.allFinite() &&
             covariance.allFinite() &&
             projection.fullPivLu().rank() == projection.rows() &&
             covariance.isApprox(covariance.transpose(), 1e-9) &&
             covariance.llt().info() == Eigen::Success;
    }
  }
  return false;
}

Matrix6d TransformCovarianceBound(const Eigen::Isometry3d& map_base,
                                  const LocalState& local,
                                  const Matrix6d& observation_covariance,
                                  Matrix6d* observation_bound,
                                  Matrix6d* local_bound) {
  const Eigen::Matrix3d rotation =
      map_base.linear() * local.orientation.toRotationMatrix().transpose();
  Matrix6d observation_jacobian = Matrix6d::Identity();
  observation_jacobian.topRightCorner<3, 3>() = Skew(rotation * local.position);
  Matrix6d local_jacobian = Matrix6d::Zero();
  local_jacobian.topLeftCorner<3, 3>() = -rotation;
  local_jacobian.topRightCorner<3, 3>() =
      -rotation * Skew(local.position) * local.orientation.toRotationMatrix();
  local_jacobian.bottomRightCorner<3, 3>() = -map_base.linear();
  if (observation_bound)
    *observation_bound = 2.0 * observation_jacobian * observation_covariance *
                         observation_jacobian.transpose();
  if (local_bound)
    *local_bound = 2.0 * local_jacobian * PoseCovariance(local) *
                   local_jacobian.transpose();
  // 2(Pa + Pb) bounds unknown cross-correlation; independence is not assumed.
  return 2.0 *
         (observation_jacobian * observation_covariance *
              observation_jacobian.transpose() +
          local_jacobian * PoseCovariance(local) * local_jacobian.transpose());
}

Matrix6d GlobalPoseCovarianceBound(const GlobalState& global,
                                   const LocalState& local) {
  Matrix6d alignment_jacobian = Matrix6d::Identity();
  alignment_jacobian.topRightCorner<3, 3>() =
      -Skew(global.map_to_odom.linear() * local.position);
  Matrix6d local_jacobian = Matrix6d::Zero();
  local_jacobian.topLeftCorner<3, 3>() = global.map_to_odom.linear();
  local_jacobian.bottomRightCorner<3, 3>() =
      global.map_to_odom.linear() * local.orientation.toRotationMatrix();
  return 2.0 *
         (alignment_jacobian * global.covariance *
              alignment_jacobian.transpose() +
          local_jacobian * PoseCovariance(local) * local_jacobian.transpose());
}

Matrix6d FixedGainCovarianceBound(const Matrix6d& prior,
                                  const Eigen::MatrixXd& observation,
                                  const Eigen::MatrixXd& projection,
                                  const Eigen::MatrixXd& gain) {
  const Matrix6d retained = Matrix6d::Identity() - gain * projection;
  // For arbitrary shared errors, Cov(a+b) <= 2 Cov(a) + 2 Cov(b).
  const Matrix6d bound = 2 * (retained * prior * retained.transpose() +
                              gain * observation * gain.transpose());
  return (0.5 * (bound + bound.transpose())).eval();
}

}  // namespace unified
}  // namespace localization
}  // namespace openloc
