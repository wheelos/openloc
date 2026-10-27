// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include "openloc/localization/local/native_types.h"

namespace openloc::localization::detail {

inline Eigen::Vector3d BootstrapDeskewPoint(
    const unified::TimedLidarPoint& point,
    const Eigen::Isometry3d& base_from_lidar,
    const Eigen::Quaterniond& end_from_point,
    const Eigen::Vector3d& velocity_in_end_base, double end_time) {
  return end_from_point * (base_from_lidar * point.position) +
         velocity_in_end_base * (point.time - end_time);
}

}  // namespace openloc::localization::detail
