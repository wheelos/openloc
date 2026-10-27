// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <vector>

#include "openloc/localization/local/motion_initialization_types.h"

namespace openloc {
namespace localization {
namespace unified {

// Relative, metric, IMU-origin LiDAR poses only. No map/GNSS/ODOM seed and no
// bias prior are used to manufacture gravity/accelerometer observability.
// The separate opt-in LiDAR-rest path assumes zero velocity/accelerometer bias
// after metric rest and force-consistency checks; it does not identify that
// bias. Pose uncertainty and interpolation error are not statistically
// qualified.
Result InitializeFromRelativeMotion(
    const MotionInitializationConfig& config,
    const std::vector<RelativeLidarPose>& poses,
    const std::vector<ImuSample>& imu, MotionInitialization* output,
    MotionInitializationDiagnostics* diagnostics = nullptr);

}  // namespace unified
}  // namespace localization
}  // namespace openloc
