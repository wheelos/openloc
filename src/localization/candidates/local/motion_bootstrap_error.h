// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <cstddef>

#include "candidates/map/map_registration_error.h"
#include "openloc/localization/local/motion_initialization_types.h"

namespace openloc::localization::unified {

struct RelativeBootstrapFrame {
  LidarScan scan;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond imu_rotation = Eigen::Quaterniond::Identity();
  RegistrationErrorMap registration;
};

struct RelativeBootstrapEvidence {
  Result assessment;
  std::vector<RelativeBootstrapFrame> frames;
  // Retired initializer poses still carry raw registration ancestry.
  std::size_t first_pose_frame = 0;
  std::vector<ImuSample> imu;
  Eigen::Quaterniond first_imu_rotation = Eigen::Quaterniond::Identity();
  bool translation_deskew = false;
};

// The first 15 rows are initial local error; the last six are the original
// frontier acceleration/gyro sample, reused by the next trapezoid.
Result RelativeBootstrapCovariance(
    const MotionInitializationConfig& policy, const LocalConfig& config,
    const std::vector<RelativeLidarPose>& poses,
    const std::vector<ImuSample>& imu, const MotionInitialization& nominal,
    const RelativeBootstrapEvidence& evidence,
    Eigen::Matrix<double, 21, 21>* covariance);

}  // namespace openloc::localization::unified
