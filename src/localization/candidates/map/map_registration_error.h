// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <vector>

#include "candidates/common/types.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace openloc::localization::unified {

// Frozen correspondence/voxel/normal-support/gain model of the actual ICP
// iterations. Pose errors are additive target translation and left rotation.
// These are raw-input loadings, not independent pose covariance.
// The explicitly deterministic-target mode leaves target loadings empty;
// it neither models nor qualifies map-construction uncertainty.
struct RegistrationErrorMap {
  Result assessment{Reason::CORRELATION_UNQUALIFIED,
                    "Raw registration error support was not requested"};
  Matrix6d initial_jacobian = Matrix6d::Zero();
  std::vector<Eigen::Matrix<double, 6, 3>> source_point_jacobians;
  std::vector<Eigen::Matrix<double, 6, 3>> target_point_jacobians;
};

struct IcpErrorCorrespondence {
  std::size_t source = 0;
  std::size_t target = 0;
  double weight = 0;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
};

struct IcpErrorStep {
  Eigen::Isometry3d before = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 1> delta =
      Eigen::Matrix<double, 6, 1>::Zero();
  Matrix6d information = Matrix6d::Zero();
  std::vector<IcpErrorCorrespondence> correspondences;
};

Result LinearizeRawIcp(
    const pcl::PointCloud<pcl::PointXYZ>& raw_source,
    const pcl::PointCloud<pcl::PointXYZ>& source,
    const pcl::PointCloud<pcl::PointXYZ>& raw_target,
    const pcl::PointCloud<pcl::PointXYZ>& target, double source_voxel_size,
    double target_voxel_size, uint32_t normal_neighbors,
    uint32_t maximum_registration_points, uint32_t maximum_error_points,
    const std::vector<IcpErrorStep>& steps, RegistrationErrorMap* output,
    bool deterministic_target = false);

}  // namespace openloc::localization::unified
