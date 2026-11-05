// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <vector>

#include "candidates/common/types.h"

namespace openloc {
namespace localization {
namespace unified {

using Matrix6x15d = Eigen::Matrix<double, 6, 15>;

bool ValidLocalCovariance(const LocalState& state);
Result AdvanceCovarianceSquareRoot(const LocalState& previous,
                                   const Matrix15d& transition,
                                   const Eigen::MatrixXd& additive_root,
                                   LocalState* next);

Eigen::Matrix3d RightRotationJacobian(const Eigen::Vector3d& rotation);
Eigen::Matrix3d InverseRightRotationJacobian(const Eigen::Vector3d& rotation);

Result RotationInterpolationJacobians(const Eigen::Quaterniond& first,
                                      const Eigen::Quaterniond& second,
                                      double alpha, Eigen::Matrix3d* first_error,
                                      Eigen::Matrix3d* second_error);

void RelativeMotionJacobians(const Eigen::Isometry3d& start_pose,
                             const Eigen::Isometry3d& delta,
                             Matrix6x15d* start, Matrix6x15d* end);

Result ComputeMotionIncrement(const LocalState& start, const LocalState& end,
                              const Matrix15d& cross_covariance,
                              const std::vector<SourceSampleId>& sources,
                              MotionIncrement* output);

Eigen::MatrixXd JointErrorCovariance(const LocalState& state);
Eigen::MatrixXd JointOutputJacobian(const LocalState& state);
Result ValidateJointLocalError(const LocalState& state);
// Both endpoint error states must have passed ValidateJointLocalError.
Result ValidateJointMotionWithValidatedLocalErrors(
    const LocalState& start, const LocalState& end,
    const MotionIncrement& motion);
Result ValidateJointMotion(const LocalState& start, const LocalState& end,
                           const MotionIncrement& motion);

}  // namespace unified
}  // namespace localization
}  // namespace openloc
