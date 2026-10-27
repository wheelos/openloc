// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "openloc/localization/local/native_motion_bootstrap.h"

#include <cmath>

#include "gtest/gtest.h"

#include "src/localization/local/bootstrap_deskew.h"

namespace openloc::replay {
namespace {
namespace u = localization::unified;
using localization::NativeMotionBootstrap;
using localization::detail::BootstrapDeskewPoint;

Eigen::Quaterniond Rotation(double time) {
  return Eigen::AngleAxisd(0.4 * time, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(-0.3 * time, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(0.2 * time, Eigen::Vector3d::UnitZ());
}

TEST(NativeMotionBootstrapTest, RollingPlanesRequireTranslationWithLeverArm) {
  constexpr double end_time = 2.0;
  const Eigen::Vector3d velocity(0.8, -0.3, 0.4);
  Eigen::Isometry3d base_from_lidar = Eigen::Isometry3d::Identity();
  base_from_lidar.translation() = Eigen::Vector3d(-0.011, -0.02329, 0.04412);
  base_from_lidar.linear() = Rotation(0.2).toRotationMatrix();
  const auto end = Rotation(end_time);
  double corrected_squared = 0.0;
  double rotation_only_squared = 0.0;
  int count = 0;
  for (int plane = 0; plane < 3; ++plane) {
    for (int i = 0; i < 30; ++i) {
      for (int j = 0; j < 30; ++j) {
        Eigen::Vector3d world(0.2 * i, 0.2 * j, 4.0);
        if (plane == 1) world = Eigen::Vector3d(4.0, 0.2 * i, 0.2 * j);
        if (plane == 2) world = Eigen::Vector3d(0.2 * i, 4.0, 0.2 * j);
        const double time = end_time - 0.1 + 0.1 * (i * 30 + j) / 899.0;
        const auto at = Rotation(time);
        u::TimedLidarPoint raw;
        raw.time = time;
        raw.position = base_from_lidar.inverse() *
                       (at.conjugate() * (world - velocity * time));
        const Eigen::Vector3d expected =
            end.conjugate() * (world - velocity * end_time);
        const auto corrected =
            BootstrapDeskewPoint(raw, base_from_lidar, end.conjugate() * at,
                                 end.conjugate() * velocity, end_time);
        const auto rotation_only =
            BootstrapDeskewPoint(raw, base_from_lidar, end.conjugate() * at,
                                 Eigen::Vector3d::Zero(), end_time);
        corrected_squared += (corrected - expected).squaredNorm();
        rotation_only_squared += (rotation_only - expected).squaredNorm();
        ++count;
      }
    }
  }
  EXPECT_LT(std::sqrt(corrected_squared / count), 1e-12);
  EXPECT_GT(std::sqrt(rotation_only_squared / count), 0.05);
}

TEST(NativeMotionBootstrapTest, RejectsUnboundedVoxelPolicy) {
  EXPECT_THROW(NativeMotionBootstrap(false, 0.0), std::invalid_argument);
  EXPECT_THROW(NativeMotionBootstrap(false, 0.6), std::invalid_argument);
  EXPECT_THROW(NativeMotionBootstrap(false, std::nan("")),
               std::invalid_argument);
}

TEST(NativeMotionBootstrapTest,
     VelocityUsesPreviousAcceptedScanAndClearsOnGap) {
  NativeMotionBootstrap bootstrap(true);
  u::LocalConfig config;
  config.enable_motion_initialization = true;
  localization::NativeOdometry estimator(config);
  const Eigen::Vector3d velocity(0.3, -0.1, 0.05);
  for (int index = 0; index <= 180; ++index) {
    const double time = 1.0 + index * 0.005;
    u::ImuSample imu;
    imu.stamp = {time, time, static_cast<uint64_t>(index + 1), "unix"};
    imu.acceleration = Eigen::Vector3d(0.0, 0.0, 9.80665);
    imu.angular_velocity.setZero();
    bootstrap.AddImu(imu);
    if (index == 0 || index % 60 != 0) continue;
    u::LidarScan scan;
    scan.stamp = imu.stamp;
    for (int i = 0; i < 30; ++i) {
      for (int j = 0; j < 30; ++j) {
        const double point_time = time - 0.1 + 0.1 * (i * 30 + j) / 899.0;
        for (const Eigen::Vector3d& world :
             {Eigen::Vector3d(0.2 * i, 0.2 * j, 4.0),
              Eigen::Vector3d(4.0, 0.2 * i, 0.2 * j),
              Eigen::Vector3d(0.2 * i, 4.0, 0.2 * j)}) {
          scan.points.push_back(
              {world - velocity * (point_time - 1.0), point_time});
        }
      }
    }
    const auto result = bootstrap.Observe(scan, &estimator);
    ASSERT_EQ(result.reason, u::Reason::HISTORY_UNAVAILABLE) << result.message;
    if (index < 180) {
      EXPECT_TRUE(bootstrap.deskew_velocity_in_end_base().isZero());
    } else {
      EXPECT_LT((bootstrap.deskew_velocity_in_end_base() - velocity).norm(),
                0.01);
    }
  }
  u::ImuSample gap;
  gap.stamp = {2.5, 2.5, 182, "unix"};
  bootstrap.AddImu(gap);
  EXPECT_TRUE(bootstrap.deskew_velocity_in_end_base().isZero());
  EXPECT_FALSE(estimator.state().valid);
}

}  // namespace
}  // namespace openloc::replay
