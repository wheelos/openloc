// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "openloc/localization/local/native_odometry.h"

#include <cmath>

#include "gtest/gtest.h"

#include "openloc/localization/local/native_motion_bootstrap.h"

namespace openloc::localization {
namespace {
namespace u = unified;

u::ImuSample Imu(double time, uint64_t sequence) {
  u::ImuSample sample;
  sample.stamp = {time, time, sequence, "unix"};
  sample.acceleration.z() = 9.80665;
  return sample;
}

u::LocalConfig Config() {
  u::LocalConfig config;
  config.session = "native-public-test";
  config.native_imu_sample_period = 0.01;
  config.allow_imu_only_initialization = true;
  config.stationary_samples = 3;
  config.stationary_duration = 0.015;
  config.enable_lidar = true;
  config.lidar_frame = "lidar";
  config.lidar_calibration_id = "test-lidar";
  config.lidar_point_noise_std = 0.01;
  config.lidar_plane_noise_std = 0.01;
  config.lidar_voxel_size = 1;
  config.lidar_max_voxels = 100;
  config.lidar_max_scans = 3;
  config.lidar_min_points_per_plane = 3;
  config.lidar_min_correspondences = 2;
  config.lidar_max_points = 1000;
  config.lidar_max_correspondence_distance = 0.3;
  config.lidar_observed_singular_value = 1;
  config.lidar_innovation_gate = 100;
  config.lidar_max_scan_age = 0.1;
  config.lidar_future_tolerance = 0.001;
  config.lidar_max_translation_correction_rate = 0.2;
  config.lidar_max_rotation_correction_rate = 0.1;
  config.lidar_max_velocity_correction_acceleration = 0.3;
  config.lidar_anchor_variance_scale = 1;
  config.lidar_correction_variance_scale = 2;
  config.lidar_max_translation_correction = 2;
  config.lidar_max_rotation_correction = 0.5;
  config.lidar_max_velocity_correction = 2;
  config.lidar_max_update_iterations = 2;
  config.lidar_process_noise_psd_tolerance = 1e-12;
  config.lidar_rotation_length = 1;
  return config;
}

u::LidarScan Scan(const u::LocalState& state, uint64_t sequence,
                  double offset) {
  u::LidarScan scan;
  scan.stamp = state.stamp;
  scan.stamp.sequence = sequence;
  scan.epoch = state.epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "test-lidar";
  for (double x : {0.1, 0.3, 0.5})
    for (double z : {0.1, 0.3, 0.5})
      for (double side : {-1.0, 1.0})
        scan.points.push_back(
            {Eigen::Vector3d(x, side + offset, z), state.stamp.time});
  return scan;
}

TEST(NativeOdometryPublicTest, SeparatesIntegrityFromCapabilityAndKeepsFaults) {
  NativeOdometry local(Config());
  ASSERT_TRUE(local.ValidateConfig().ok());
  for (uint64_t i = 1; i <= 3; ++i) local.AddImu(Imu(1 + 0.01 * (i - 1), i));
  ASSERT_TRUE(local.state().valid);
  EXPECT_TRUE(local.EvaluateIntegrity(1.02).ok());
  EXPECT_FALSE(local.Evaluate(1.02).ok());
  u::LocalState retained;
  ASSERT_TRUE(local.Lookup(1.02, &retained).ok());
  EXPECT_EQ(retained.stamp.sequence, 3);
  EXPECT_EQ(local.AddImu(Imu(1.2, 4)).reason, u::Reason::IMU_GAP);
  EXPECT_FALSE(local.EvaluateIntegrity(1.2).ok());
}

TEST(NativeOdometryPublicTest, ExportsActualJointMotionAndRejectsUnmodeledLio) {
  NativeOdometry local(Config());
  for (uint64_t index = 1; index <= 8; ++index)
    local.AddImu(Imu(1 + 0.01 * (index - 1), index));
  u::LocalState start;
  ASSERT_TRUE(local.Lookup(Imu(1.02, 3).stamp.time, &start).ok());
  const auto end = local.state();
  u::MotionIncrement motion;
  const auto exported = local.MotionBetween(start.stamp.time, end.stamp.time,
                                            &motion);
  ASSERT_TRUE(exported.ok()) << exported.message;
  EXPECT_EQ(motion.start.sequence, start.stamp.sequence);
  EXPECT_EQ(motion.end.sequence, end.stamp.sequence);
  EXPECT_EQ(motion.epoch, end.epoch);
  EXPECT_TRUE(motion.start_covariance.isApprox(start.covariance, 0));
  EXPECT_TRUE(motion.end_covariance.isApprox(end.covariance, 0));
  EXPECT_GT(motion.cross_covariance.norm(), 0);
  EXPECT_FALSE(start.uncertainty_qualified);
  EXPECT_FALSE(local.state().uncertainty_qualified);
  u::ContinuousOutputErrorTransition output_map;
  EXPECT_EQ(local.OutputErrorTransition(&output_map).reason,
            u::Reason::HISTORY_UNAVAILABLE);
  bool first_seen = false, last_seen = false;
  for (const auto& source : motion.sources) {
    EXPECT_EQ(source.source, "imu");
    first_seen |= source.sequence == start.stamp.sequence;
    last_seen |= source.sequence == end.stamp.sequence;
  }
  EXPECT_TRUE(first_seen);
  EXPECT_TRUE(last_seen);
  const auto retained = motion;
  EXPECT_EQ(local.MotionBetween(start.stamp.time + 0.001, end.stamp.time,
                                &motion).reason,
            u::Reason::HISTORY_UNAVAILABLE);
  EXPECT_TRUE(motion.cross_covariance.isApprox(retained.cross_covariance, 0));
  ASSERT_EQ(local.AddLidar(Scan(local.internal_state(), 1, 0)).reason,
            u::Reason::GEOMETRY_UNAVAILABLE);
  local.AddImu(Imu(1.08, 9));
  ASSERT_TRUE(local.AddLidar(Scan(local.internal_state(), 2, 0.1)).ok());
  EXPECT_EQ(local.MotionBetween(start.stamp.time, local.state().stamp.time,
                                &motion).reason,
            u::Reason::CORRELATION_UNQUALIFIED);
  EXPECT_EQ(motion.end.sequence, retained.end.sequence);
  EXPECT_TRUE(motion.cross_covariance.isApprox(retained.cross_covariance, 0));
  local.AddImu(Imu(1.09, 10));
  const auto mapped = local.OutputErrorTransition(&output_map);
  ASSERT_TRUE(mapped.ok()) << mapped.message;
  EXPECT_EQ(output_map.end.sequence, local.state().stamp.sequence);
  EXPECT_EQ(output_map.epoch, local.state().epoch);
  EXPECT_FALSE(local.state().covariance_model_valid);
  EXPECT_FALSE(local.motion_available());
  auto scan = Scan(local.internal_state(), 3, 0);
  scan.points.resize(4);
  scan.points[0].time = 1.075;
  scan.points[1].time = 1.085;
  scan.points[2].time = local.state().stamp.time;
  scan.points[3].time = 1.075;
  const auto before_deskew = local.state();
  const auto accepted_before_deskew = local.lidar_health().accepted;
  u::LidarDeskewErrorMap deskew;
  const auto deskewed = local.DeskewErrorMap(scan, &deskew);
  ASSERT_TRUE(deskewed.ok()) << deskewed.message;
  ASSERT_EQ(deskew.points.size(), 4);
  ASSERT_EQ(deskew.history.size(), 4);
  EXPECT_EQ(deskew.scan.sequence, scan.stamp.sequence);
  EXPECT_EQ(deskew.local_frontier.sequence, local.state().stamp.sequence);
  EXPECT_EQ(deskew.lidar_revision, accepted_before_deskew);
  bool before_correction = false, after_correction = false;
  for (const auto& clone : deskew.history) {
    if (clone.state.stamp.sequence != 9) continue;
    before_correction |= clone.before_lidar_correction;
    after_correction |= !clone.before_lidar_correction;
  }
  EXPECT_TRUE(before_correction);
  EXPECT_TRUE(after_correction);
  EXPECT_EQ(deskew.points[0].history.size(), 3);
  EXPECT_EQ(deskew.points[1].history.size(), 2);
  ASSERT_EQ(deskew.points[2].history.size(), 1);
  EXPECT_TRUE(deskew.points[2].history.front().jacobian.isZero(1e-12));
  EXPECT_TRUE(deskew.points[2].point_in_end_base.isApprox(
      scan.base_from_lidar * scan.points[2].position, 1e-12));
  ASSERT_EQ(deskew.points[0].history.size(), deskew.points[3].history.size());
  for (std::size_t index = 0; index < deskew.points[0].history.size(); ++index)
    EXPECT_EQ(deskew.points[0].history[index].history_index,
              deskew.points[3].history[index].history_index);
  EXPECT_EQ(local.lidar_health().accepted, accepted_before_deskew);
  EXPECT_TRUE(local.state().position.isApprox(before_deskew.position, 0));
  EXPECT_TRUE(local.state().covariance.isApprox(before_deskew.covariance, 0));
  scan.points[0].time = 0.5;
  EXPECT_EQ(local.DeskewErrorMap(scan, &deskew).reason,
            u::Reason::HISTORY_UNAVAILABLE);
  EXPECT_EQ(deskew.history.size(), 4);
  EXPECT_EQ(deskew.points.size(), 4);
  EXPECT_EQ(local.MotionBetween(start.stamp.time, end.stamp.time, nullptr).reason,
            u::Reason::INVALID_INPUT);
}

TEST(NativeOdometryPublicTest,
     StartupRejectsMissingOwnerAndIncompleteEvidence) {
  auto config = Config();
  config.allow_imu_only_initialization = false;
  config.enable_motion_initialization = true;
  NativeOdometry local(config);
  NativeMotionBootstrap bootstrap;
  const auto sample = Imu(1.0, 1);
  local.AddImu(sample);
  bootstrap.AddImu(sample);
  u::LidarScan scan;
  scan.stamp = sample.stamp;
  scan.points.push_back({Eigen::Vector3d(1, 2, 3), sample.stamp.time});
  u::MotionInitializationDiagnostics diagnostics;
  diagnostics.solution_available = true;
  EXPECT_EQ(bootstrap.Observe(scan, nullptr, &diagnostics).reason,
            u::Reason::INVALID_INPUT);
  EXPECT_FALSE(diagnostics.solution_available);
  EXPECT_EQ(bootstrap.Observe(scan, &local, &diagnostics).reason,
            u::Reason::HISTORY_UNAVAILABLE);
  EXPECT_FALSE(local.state().valid);
  EXPECT_FALSE(local.motion_available());
  EXPECT_FALSE(diagnostics.solution_available);
}

TEST(NativeOdometryPublicTest, ExportsActualStoredGeometryWithoutQualifyingIt) {
  auto config = Config();
  config.lidar_max_scans = 1;
  NativeOdometry local(config);
  for (uint64_t index = 1; index <= 3; ++index)
    local.AddImu(Imu(1 + 0.01 * (index - 1), index));
  u::LidarStoredGeometryErrorMap map;
  map.lidar_revision = 42;
  EXPECT_EQ(local.StoredGeometryErrorMap(&map).reason,
            u::Reason::GEOMETRY_UNAVAILABLE);
  EXPECT_EQ(map.lidar_revision, 42);
  auto scan = Scan(local.internal_state(), 1, 0);
  scan.points.clear();
  for (double offset : {0, 1})
    for (double x : {0.15, 0.35, 0.55})
      for (double y : {0.17, 0.37, 0.57})
        scan.points.push_back(
            {Eigen::Vector3d(x + offset, y, 0.42 + 0.1 * x + 0.2 * y),
             scan.stamp.time});
  ASSERT_EQ(local.AddLidar(scan).reason, u::Reason::GEOMETRY_UNAVAILABLE);
  ASSERT_TRUE(local.StoredGeometryErrorMap(&map).ok());
  EXPECT_EQ(map.planes.size(), 2);
  EXPECT_EQ(map.anchors.size(), 1);
  local.AddImu(Imu(1.03, 4));
  scan.stamp = local.internal_state().stamp;
  scan.stamp.sequence = 2;
  for (auto& point : scan.points) point.time = scan.stamp.time;
  const auto corrected = local.AddLidar(scan);
  ASSERT_TRUE(corrected.ok()) << corrected.message;
  const auto before = local.state();
  ASSERT_TRUE(local.StoredGeometryErrorMap(&map).ok());
  EXPECT_EQ(map.epoch, local.state().epoch);
  EXPECT_EQ(map.local_frontier.sequence, 4);
  EXPECT_EQ(map.lidar_revision, 2);
  ASSERT_EQ(map.planes.size(), 2);
  ASSERT_EQ(map.anchors.size(), 1);
  EXPECT_EQ(map.anchors.count(1), 0);
  ASSERT_EQ(map.anchors.count(2), 1);
  EXPECT_TRUE(map.anchors.at(2).position.isApprox(local.internal_state().position, 0));
  for (const auto& plane : map.planes) {
    EXPECT_EQ(plane.points.size(), 9);
    for (const auto& point : plane.points) {
      EXPECT_EQ(point.source_sequence, 2);
      ASSERT_LT(point.point_index, scan.points.size());
      EXPECT_TRUE(point.point.isApprox(
          u::Pose(map.anchors.at(2)) * scan.points[point.point_index].position, 1e-12));
    }
  }
  EXPECT_EQ(local.lidar_health().accepted, 2);
  EXPECT_TRUE(local.state().position.isApprox(before.position, 0));
  EXPECT_TRUE(local.state().covariance.isApprox(before.covariance, 0));
  EXPECT_FALSE(local.state().covariance_model_valid);
  EXPECT_FALSE(local.state().uncertainty_qualified);
  EXPECT_FALSE(local.motion_available());
  EXPECT_EQ(local.StoredGeometryErrorMap(nullptr).reason, u::Reason::INVALID_INPUT);
}

TEST(NativeOdometryPublicTest, OpaqueLidarTaskPreservesOwnerAndExactFrontier) {
  const auto config = Config();
  NativeOdometry local(config), other(config);
  for (uint64_t i = 1; i <= 3; ++i) local.AddImu(Imu(1 + 0.01 * (i - 1), i));
  ASSERT_TRUE(local.state().valid);
  ASSERT_EQ(local.AddLidar(Scan(local.internal_state(), 1, 0)).reason,
            u::Reason::GEOMETRY_UNAVAILABLE);
  local.AddImu(Imu(1.03, 4));
  std::unique_ptr<NativeOdometry::LidarTask> task;
  ASSERT_TRUE(
      local.PrepareLidarTask(Scan(local.internal_state(), 2, 0.1), &task).ok());
  EXPECT_EQ(
      local.PrepareLidarTask(Scan(local.internal_state(), 3, 0), &task).reason,
      u::Reason::INVALID_INPUT);
  const auto before = local.state();
  ASSERT_TRUE(task->Compute().ok());
  EXPECT_TRUE(local.state().position.isApprox(before.position, 0));
  EXPECT_EQ(other.CommitLidarTask(task.get()).reason, u::Reason::INVALID_INPUT);
  const auto next = Imu(1.04, 5);
  local.AddImu(next);
  EXPECT_EQ(local.CommitLidarTask(task.get()).reason,
            u::Reason::HISTORY_UNAVAILABLE);
  ASSERT_EQ(task->ReplayImu(next).reason, u::Reason::CORRELATION_UNQUALIFIED);
  const auto published = local.state();
  ASSERT_TRUE(local.CommitLidarTask(task.get()).ok());
  EXPECT_TRUE(local.state().position.isApprox(published.position, 0));
  EXPECT_FALSE(local.state().covariance_model_valid);
  EXPECT_FALSE(local.motion_available());
  EXPECT_EQ(local.motion_status().reason, u::Reason::CORRELATION_UNQUALIFIED);
  EXPECT_EQ(local.CommitLidarTask(task.get()).reason, u::Reason::INVALID_INPUT);
}

TEST(NativeOdometryPublicTest, JointObserverPreservesNominalAndSharedGeometry) {
  auto config = Config();
  config.enable_native_joint_error = true;
  config.lidar_max_scans = 2;
  config.history_duration = 0.035;
  NativeOdometry modeled(config);
  config.enable_native_joint_error = false;
  NativeOdometry nominal(config);
  auto scan_at = [](const u::LocalState& state, uint64_t sequence, double offset) {
    auto scan = Scan(state, sequence, offset);
    for (auto& point : scan.points) point.position.y() += 0.25;
    return scan;
  };
  for (uint64_t index = 1; index <= 19; ++index) {
    const auto imu = Imu(1 + 0.01 * (index - 1), index);
    const auto observed = modeled.AddImu(imu);
    const auto original = nominal.AddImu(imu);
    EXPECT_EQ(observed.reason, original.reason);
    EXPECT_EQ(observed.message, original.message);
    if (index < 3) continue;
    ASSERT_TRUE(modeled.joint_error_status().ok())
        << modeled.joint_error_status().message;
    u::LocalState snapshot;
    const auto lookup = modeled.JointErrorLookup(imu.stamp.time, &snapshot);
    ASSERT_TRUE(lookup.ok()) << lookup.message;
    ASSERT_TRUE(snapshot.joint_error);
    EXPECT_TRUE(snapshot.covariance_model_valid);
    EXPECT_FALSE(snapshot.uncertainty_qualified);
    EXPECT_TRUE(snapshot.position.isApprox(modeled.state().position, 0));
    const auto& error = *snapshot.joint_error;
    EXPECT_TRUE((error.output_jacobian * error.covariance *
                 error.output_jacobian.transpose()).isApprox(
        snapshot.covariance, 1e-12));
    EXPECT_LE(error.covariance.rows(), 256);
    if (index >= 5) {
      u::MotionIncrement motion;
      const auto connected = modeled.JointErrorMotionBetween(
          Imu(1 + 0.01 * (index - 3), index - 2).stamp.time,
          imu.stamp.time, &motion);
      ASSERT_TRUE(connected.ok()) << connected.message << " at IMU " << index
                                 << " joint dimension " << error.covariance.rows();
      EXPECT_EQ(motion.start.sequence, index - 2);
      EXPECT_EQ(motion.end.sequence, index);
      EXPECT_EQ(motion.joint_cross_covariance.cols(), error.covariance.rows());
    }
    if (index == 3 || index == 7 || index == 11 || index == 15) {
      const uint64_t sequence = (index + 1) / 4;
      const auto scan = scan_at(modeled.internal_state(), sequence,
                                sequence == 1 ? 0 : 0.02);
      const auto corrected = modeled.AddLidar(scan);
      const auto baseline = nominal.AddLidar(scan);
      EXPECT_EQ(corrected.reason, baseline.reason);
      ASSERT_TRUE(corrected.ok() ||
                  corrected.reason == u::Reason::GEOMETRY_UNAVAILABLE)
          << corrected.message;
      ASSERT_TRUE(modeled.joint_error_status().ok())
          << modeled.joint_error_status().message;
    }
    EXPECT_TRUE(modeled.state().position.isApprox(nominal.state().position, 0));
    EXPECT_TRUE(modeled.state().velocity.isApprox(nominal.state().velocity, 0));
    EXPECT_TRUE(modeled.state().orientation.isApprox(nominal.state().orientation, 0));
    EXPECT_TRUE(modeled.state().covariance.isApprox(nominal.state().covariance, 0));
    EXPECT_EQ(modeled.state().covariance_model_valid,
              nominal.state().covariance_model_valid);
  }
  EXPECT_FALSE(modeled.state().covariance_model_valid);
  EXPECT_FALSE(modeled.state().uncertainty_qualified);
  EXPECT_EQ(modeled.lidar_health().accepted, 4);
}

TEST(NativeOdometryPublicTest, DiagnosticGainMemorySurvivesDelayedCommit) {
  auto config = Config();
  config.diagnostic_lidar_tracking_gain = true;
  NativeOdometry direct(config), delayed(config);
  for (uint64_t i = 1; i <= 3; ++i) {
    direct.AddImu(Imu(1 + 0.01 * (i - 1), i));
    delayed.AddImu(Imu(1 + 0.01 * (i - 1), i));
  }
  const auto first = Scan(direct.internal_state(), 1, 0);
  ASSERT_EQ(direct.AddLidar(first).reason, u::Reason::GEOMETRY_UNAVAILABLE);
  ASSERT_EQ(delayed.AddLidar(first).reason, u::Reason::GEOMETRY_UNAVAILABLE);
  direct.AddImu(Imu(1.03, 4));
  delayed.AddImu(Imu(1.03, 4));
  const auto second = Scan(direct.internal_state(), 2, 0.1);
  ASSERT_TRUE(direct.AddLidar(second).ok());
  std::unique_ptr<NativeOdometry::LidarTask> task;
  ASSERT_TRUE(delayed.PrepareLidarTask(second, &task).ok());
  ASSERT_TRUE(task->Compute().ok());
  const auto next = Imu(1.04, 5);
  direct.AddImu(next);
  delayed.AddImu(next);
  task->ReplayImu(next);
  ASSERT_TRUE(delayed.CommitLidarTask(task.get()).ok());
  for (uint64_t sequence = 3; sequence <= 5; ++sequence) {
    const auto sample = Imu(1.02 + 0.01 * sequence, sequence + 3);
    direct.AddImu(sample);
    delayed.AddImu(sample);
    const auto scan = Scan(direct.internal_state(), sequence, 0.03);
    ASSERT_TRUE(direct.AddLidar(scan).ok());
    ASSERT_TRUE(delayed.AddLidar(scan).ok());
    EXPECT_TRUE(direct.internal_state().position.isApprox(
        delayed.internal_state().position, 1e-12));
    EXPECT_TRUE(direct.internal_state().velocity.isApprox(
        delayed.internal_state().velocity, 1e-12));
    EXPECT_TRUE(direct.internal_state().covariance.isApprox(
        delayed.internal_state().covariance, 1e-12));
  }
  EXPECT_FALSE(delayed.state().covariance_model_valid);
  EXPECT_FALSE(delayed.state().uncertainty_qualified);
  EXPECT_FALSE(delayed.motion_available());
}

TEST(NativeOdometryPublicTest, JointObserverRejectsCapacityWithoutChangingOdom) {
  auto config = Config();
  config.enable_native_joint_error = true;
  config.native_joint_max_dimension = 21;
  NativeOdometry local(config);
  for (uint64_t index = 1; index <= 4; ++index)
    local.AddImu(Imu(1 + 0.01 * (index - 1), index));
  ASSERT_TRUE(local.EvaluateIntegrity(1.03).ok());
  EXPECT_EQ(local.joint_error_status().reason, u::Reason::CORRELATION_UNQUALIFIED);
  u::LocalState sentinel;
  sentinel.stamp.sequence = 999;
  EXPECT_EQ(local.JointErrorLookup(1.03, &sentinel).reason,
            u::Reason::CORRELATION_UNQUALIFIED);
  EXPECT_EQ(sentinel.stamp.sequence, 999);
  ASSERT_TRUE(local.JointErrorLookup(1.02, &sentinel).ok());
  EXPECT_EQ(sentinel.joint_error->covariance.rows(), 21);
}

TEST(NativeOdometryPublicTest, ExportsSharedScanAtFinalizedSyncAndDelayedFrontiers) {
  for (const bool delayed : {false, true}) {
    SCOPED_TRACE(delayed);
    auto config = Config();
    config.enable_native_joint_error = true;
    config.history_duration = 0.035;
    config.lidar_max_scans = 1;
    NativeOdometry local(config);
    for (uint64_t i = 1; i <= 3; ++i)
      local.AddImu(Imu(1 + 0.01 * (i - 1), i));
    ASSERT_TRUE(local.EvaluateIntegrity(local.state().stamp.time).ok());
    auto make_scan = [&](uint64_t sequence, double offset) {
      auto scan = Scan(local.internal_state(), sequence, offset);
      for (auto& point : scan.points) point.position.y() += 0.25;
      return scan;
    };
    const auto first_scan = make_scan(1, 0);
    ASSERT_EQ(local.AddLidar(first_scan).reason, u::Reason::GEOMETRY_UNAVAILABLE);
    const auto nominal_first = local.state();
    u::JointLidarScan first;
    const auto exported = local.PrepareJointLidarScan(1, &first);
    ASSERT_TRUE(exported.ok()) << exported.message;
    ASSERT_TRUE(first.conditioning_state.joint_error);
    ASSERT_EQ(first.point_jacobians.size(), first_scan.points.size());
    EXPECT_EQ(first.reference.time, first_scan.stamp.time);
    EXPECT_EQ(first.reference.sequence, first_scan.stamp.sequence);
    const auto& initial = *first.conditioning_state.joint_error;
    for (std::size_t i = 0; i < first.point_jacobians.size(); ++i) {
      const auto& jacobian = first.point_jacobians[i];
      EXPECT_EQ(jacobian.cols(), initial.covariance.rows());
      EXPECT_TRUE(first.points_in_reference_base[i].isApprox(
          first_scan.points[i].position, 1e-12));
      EXPECT_TRUE((jacobian * jacobian.transpose()).isApprox(
          Eigen::Matrix3d::Identity() * 0.0001, 1e-10));
      EXPECT_TRUE((initial.output_jacobian * jacobian.transpose()).isZero(1e-12));
    }
    EXPECT_TRUE(local.state().position.isApprox(nominal_first.position, 0));
    EXPECT_TRUE(local.state().covariance.isApprox(nominal_first.covariance, 0));
    u::JointLidarScan sentinel;
    sentinel.reference.sequence = 999;
    EXPECT_EQ(local.PrepareJointLidarScan(1, &sentinel).reason, u::Reason::DUPLICATE);
    EXPECT_EQ(sentinel.reference.sequence, 999);
    for (uint64_t i = 4; i <= 7; ++i)
      local.AddImu(Imu(1 + 0.01 * (i - 1), i));
    ASSERT_TRUE(local.EvaluateIntegrity(local.state().stamp.time).ok());
    auto second_scan = make_scan(2, 0.02);
    second_scan.points[0].time -= 0.005;
    u::LidarDeskewErrorMap deskew;
    ASSERT_TRUE(local.DeskewErrorMap(second_scan, &deskew).ok());
    if (delayed) {
      std::unique_ptr<NativeOdometry::LidarTask> task;
      ASSERT_TRUE(local.PrepareLidarTask(second_scan, &task).ok());
      ASSERT_TRUE(task->Compute().ok());
      for (uint64_t i = 8; i <= 9; ++i) {
        const auto imu = Imu(1 + 0.01 * (i - 1), i);
        local.AddImu(imu);
        task->ReplayImu(imu);
      }
      ASSERT_TRUE(local.CommitLidarTask(task.get()).ok());
    } else {
      ASSERT_TRUE(local.AddLidar(second_scan).ok());
    }
    const auto nominal = local.state();
    EXPECT_EQ(local.PrepareJointLidarScan(1, &sentinel).reason,
              u::Reason::HISTORY_UNAVAILABLE);
    EXPECT_EQ(sentinel.reference.sequence, 999);
    u::JointLidarScan second;
    const auto next = local.PrepareJointLidarScan(2, &second);
    ASSERT_TRUE(next.ok()) << next.message;
    EXPECT_EQ(second.reference.time, second_scan.stamp.time);
    EXPECT_EQ(second.conditioning_state.stamp.time, local.state().stamp.time);
    if (delayed) {
      EXPECT_LT(second.reference.time, second.conditioning_state.stamp.time);
    }
    for (std::size_t i = 0; i < second.point_jacobians.size(); ++i)
      EXPECT_TRUE(second.points_in_reference_base[i].isApprox(
          deskew.points[i].point_in_end_base, 0));
    EXPECT_FALSE(second.conditioning_state.uncertainty_qualified);
    EXPECT_TRUE(local.state().position.isApprox(nominal.position, 0));
    EXPECT_TRUE(local.state().covariance.isApprox(nominal.covariance, 0));
    const auto end = local.state().stamp;
    const auto imu = Imu(end.time + 0.01, end.sequence + 1);
    local.AddImu(imu);
    u::MotionIncrement motion;
    const auto connected = local.JointErrorMotionBetween(
        end.time, imu.stamp.time, &motion);
    ASSERT_TRUE(connected.ok()) << connected.message;
    EXPECT_EQ(motion.joint_cross_covariance.rows(),
              second.conditioning_state.joint_error->covariance.rows());
    u::LocalState last;
    ASSERT_TRUE(local.JointErrorLookup(imu.stamp.time, &last).ok());
    const auto cross = second.point_jacobians[0] * motion.joint_cross_covariance *
                       last.joint_error->output_jacobian.transpose();
    EXPECT_GT(cross.norm(), 0);
  }
}

TEST(NativeOdometryPublicTest, JointGeometryRetainsTheSharedTranslationGauge) {
  auto config = Config();
  config.enable_native_joint_error = true;
  NativeOdometry local(config);
  for (uint64_t index = 1; index <= 3; ++index)
    local.AddImu(Imu(1 + 0.01 * (index - 1), index));
  u::LocalState first;
  ASSERT_TRUE(local.JointErrorLookup(1.02, &first).ok());
  auto scan = Scan(local.internal_state(), 1, 0);
  for (auto& point : scan.points) point.position.y() += 0.25;
  ASSERT_EQ(local.AddLidar(scan).reason, u::Reason::GEOMETRY_UNAVAILABLE);
  local.AddImu(Imu(1.03, 4));
  scan = Scan(local.internal_state(), 2, 0.02);
  for (auto& point : scan.points) point.position.y() += 0.25;
  ASSERT_TRUE(local.AddLidar(scan).ok());
  local.AddImu(Imu(1.04, 5));
  u::MotionIncrement motion;
  const auto exported = local.JointErrorMotionBetween(1.02, 1.04, &motion);
  ASSERT_TRUE(exported.ok()) << exported.message;
  for (int axis = 0; axis < 3; ++axis)
    EXPECT_NEAR(motion.cross_covariance(axis, axis),
                first.covariance(axis, axis), 1e-20);
  bool raw_first = false, raw_second = false;
  for (const auto& source : motion.sources) {
    if (source.source != "lidar_raw") continue;
    raw_first |= source.sequence == 1;
    raw_second |= source.sequence == 2;
  }
  EXPECT_TRUE(raw_first);
  EXPECT_TRUE(raw_second);
}

TEST(NativeOdometryPublicTest, JointObserverRetainsActualDelayedLiveOutputBridge) {
  auto config = Config();
  config.enable_native_joint_error = true;
  config.history_duration = 0.035;
  config.lidar_max_scans = 1;
  NativeOdometry local(config);
  auto scan_at = [&local](uint64_t sequence) {
    auto scan = Scan(local.internal_state(), sequence, 0.02);
    for (auto& point : scan.points) point.position.y() += 0.25;
    return scan;
  };
  for (uint64_t index = 1; index <= 3; ++index)
    local.AddImu(Imu(1 + 0.01 * (index - 1), index));
  ASSERT_EQ(local.AddLidar(scan_at(1)).reason, u::Reason::GEOMETRY_UNAVAILABLE);
  local.AddImu(Imu(1.03, 4));
  std::unique_ptr<NativeOdometry::LidarTask> immediate;
  ASSERT_TRUE(local.PrepareLidarTask(scan_at(2), &immediate).ok());
  ASSERT_TRUE(immediate->Compute().ok());
  ASSERT_TRUE(local.CommitLidarTask(immediate.get()).ok());
  ASSERT_TRUE(local.joint_error_status().ok());
  local.AddImu(Imu(1.04, 5));
  ASSERT_TRUE(local.joint_error_status().ok()) << local.joint_error_status().message;
  std::unique_ptr<NativeOdometry::LidarTask> delayed;
  ASSERT_TRUE(local.PrepareLidarTask(scan_at(3), &delayed).ok());
  ASSERT_TRUE(delayed->Compute().ok());
  for (uint64_t sequence = 6; sequence <= 11; ++sequence) {
    const auto imu = Imu(1 + 0.01 * (sequence - 1), sequence);
    local.AddImu(imu);
    delayed->ReplayImu(imu);
    ASSERT_TRUE(local.joint_error_status().ok())
        << local.joint_error_status().message;
  }
  u::LocalState published;
  EXPECT_EQ(local.JointErrorLookup(1.04, &published).reason,
            u::Reason::HISTORY_UNAVAILABLE);
  ASSERT_TRUE(local.JointErrorLookup(1.10, &published).ok());
  ASSERT_TRUE(local.CommitLidarTask(delayed.get()).ok());
  ASSERT_TRUE(local.joint_error_status().ok()) << local.joint_error_status().message;
  u::LocalState retained;
  ASSERT_TRUE(local.JointErrorLookup(1.10, &retained).ok());
  EXPECT_TRUE(retained.covariance.isApprox(published.covariance, 0));
  EXPECT_TRUE(retained.position.isApprox(published.position, 0));
  EXPECT_TRUE(retained.joint_error->output_jacobian.isApprox(
      published.joint_error->output_jacobian, 0));
  local.AddImu(Imu(1.11, 12));
  u::LocalState state;
  ASSERT_TRUE(local.JointErrorLookup(1.11, &state).ok())
      << local.joint_error_status().message;
  u::MotionIncrement motion;
  const auto connected = local.JointErrorMotionBetween(1.09, 1.11, &motion);
  ASSERT_TRUE(connected.ok()) << connected.message;
  EXPECT_FALSE(local.state().covariance_model_valid);
  EXPECT_FALSE(state.uncertainty_qualified);
  EXPECT_TRUE(local.EvaluateIntegrity(1.11).ok());
}

TEST(NativeOdometryPublicTest, StartupSnapshotNeedsExactCatchupAndCannotReplaceAnOwner) {
  auto config = Config();
  config.allow_imu_only_initialization = false;
  config.enable_motion_initialization = true;
  config.enable_lidar_stationary_initialization = true;
  config.enable_native_joint_error = true;
  NativeOdometry local(config), serial(config), other(config);
  for (uint64_t i = 0; i <= 400; ++i) {
    const auto sample = Imu(1 + 0.005 * i, i + 1);
    ASSERT_TRUE(local.AddImu(sample).ok());
    ASSERT_TRUE(serial.AddImu(sample).ok());
  }
  std::vector<u::RelativeLidarPose> poses;
  for (int i = 0; i < 9; ++i) {
    u::RelativeLidarPose pose;
    pose.time = 1 + 0.25 * i;
    poses.push_back(pose);
  }
  std::unique_ptr<NativeOdometry::StartupTask> task;
  ASSERT_TRUE(local.PrepareStartupTask(&task).ok());
  ASSERT_TRUE(task->InitializeMotion(poses).ok());
  ASSERT_TRUE(serial.InitializeMotion(poses).ok());
  EXPECT_FALSE(local.state().valid);
  EXPECT_EQ(other.CommitStartupTask(task.get()).reason, u::Reason::INVALID_INPUT);
  const auto next = Imu(3.005, 402);
  ASSERT_TRUE(local.AddImu(next).ok());
  ASSERT_EQ(serial.AddImu(next).reason, u::Reason::CORRELATION_UNQUALIFIED);
  EXPECT_EQ(local.CommitStartupTask(task.get()).reason, u::Reason::HISTORY_UNAVAILABLE);
  ASSERT_EQ(task->ReplayImu(next).reason, u::Reason::CORRELATION_UNQUALIFIED);
  ASSERT_TRUE(local.CommitStartupTask(task.get()).ok());
  EXPECT_TRUE(local.state().valid);
  EXPECT_EQ(local.state().stamp.sequence, next.stamp.sequence);
  EXPECT_TRUE(local.state().position.isApprox(serial.state().position, 0));
  EXPECT_TRUE(local.state().velocity.isApprox(serial.state().velocity, 0));
  EXPECT_TRUE(local.state().covariance.isApprox(serial.state().covariance, 0));
  EXPECT_FALSE(local.state().covariance_model_valid);
  EXPECT_FALSE(local.state().uncertainty_qualified);
  EXPECT_EQ(local.joint_error_status().reason, u::Reason::CORRELATION_UNQUALIFIED);
  EXPECT_NE(local.joint_error_status().message.find("Startup lacks raw-source"),
            std::string::npos);
  EXPECT_EQ(local.CommitStartupTask(task.get()).reason, u::Reason::INVALID_INPUT);
}

TEST(NativeOdometryPublicTest, BootstrapTaskRetainsOnlyCompletedRelativeHistory) {
  auto config = Config();
  config.allow_imu_only_initialization = false;
  config.enable_motion_initialization = true;
  NativeOdometry local(config);
  NativeMotionBootstrap bootstrap;
  NativeMotionBootstrap other(true, 0.2);
  auto sample = Imu(1, 1);
  local.AddImu(sample);
  bootstrap.AddImu(sample);
  other.AddImu(sample);
  auto scan = Scan(local.state(), 1, 0);
  std::unique_ptr<NativeMotionBootstrap::Task> task;
  ASSERT_TRUE(bootstrap.PrepareTask(scan, local, &task).ok());
  EXPECT_EQ(bootstrap.CommitTask(task.get(), &local).reason, u::Reason::INVALID_INPUT);
  EXPECT_EQ(task->Compute().reason, u::Reason::HISTORY_UNAVAILABLE);
  EXPECT_EQ(other.CommitTask(task.get(), &local).reason, u::Reason::INVALID_INPUT);
  sample = Imu(1.005, 2);
  local.AddImu(sample);
  bootstrap.AddImu(sample);
  EXPECT_EQ(bootstrap.CommitTask(task.get(), &local).reason, u::Reason::HISTORY_UNAVAILABLE);
  ASSERT_TRUE(task->ReplayImu(sample).ok());
  const auto committed = bootstrap.CommitTask(task.get(), &local);
  EXPECT_EQ(committed.reason, u::Reason::HISTORY_UNAVAILABLE);
  EXPECT_EQ(committed.message, "First relative bootstrap scan");
  EXPECT_FALSE(local.state().valid);
  EXPECT_EQ(bootstrap.CommitTask(task.get(), &local).reason, u::Reason::INVALID_INPUT);
}

TEST(NativeOdometryPublicTest, ActualRelativeBootstrapModelsRawScansAndFrontierImu) {
  for (const int mode : {0, 1, 2, 3}) {
    SCOPED_TRACE(mode);
    const bool asynchronous = mode == 1;
    const bool moving = mode == 2;
    const bool deferred = mode == 3;
    const auto orientation = [](double t) -> Eigen::Quaterniond {
      return Eigen::AngleAxisd(0.5 * std::sin(t), Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(0.4 * std::sin(1.7 * t), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0.2 * t, Eigen::Vector3d::UnitZ());
    };
    const auto position = [](double t) -> Eigen::Vector3d {
      return Eigen::Vector3d(0.4, 0.2, -0.1) * t +
          0.5 * Eigen::Vector3d(0.2, -0.1, 0.05) * t * t;
    };
    auto config = Config();
    config.allow_imu_only_initialization = false;
    config.enable_motion_initialization = true;
    config.enable_lidar_stationary_initialization = !moving;
    config.enable_native_joint_error = true;
    NativeOdometry local(config);
    config.enable_native_joint_error = false;
    NativeOdometry baseline(config);
    config.enable_motion_initialization = false;
    NativeOdometry waiting_baseline(config);
    config.enable_native_joint_error = true;
    NativeOdometry waiting(config);
    NativeMotionBootstrap bootstrap(true, 0.2), original(true, 0.2);
    for (uint64_t index = 0; index <= (deferred ? 950 : 500); ++index) {
      auto imu = Imu(1 + 0.01 * index, index + 1);
      if (moving) {
        const double t = imu.stamp.time - 1;
        imu.acceleration = orientation(t).conjugate() *
            Eigen::Vector3d(0.2, -0.1, 9.85665) +
            Eigen::Vector3d(0.1, -0.08, 0.06);
        imu.angular_velocity =
            u::LogRotation(orientation(t - 1e-5).conjugate() *
                           orientation(t + 1e-5)) / 2e-5 +
            Eigen::Vector3d(0.01, -0.015, 0.02);
      }
      local.AddImu(imu);
      baseline.AddImu(imu);
      waiting.AddImu(imu);
      waiting_baseline.AddImu(imu);
      bootstrap.AddImu(imu);
      original.AddImu(imu);
      if (index % 25 != 0) continue;
      u::LidarScan scan;
      scan.stamp = imu.stamp;
      scan.stamp.sequence = index / 25 + 1;
      scan.epoch = local.state().epoch;
      scan.frame_id = "lidar";
      scan.calibration_id = "test-lidar";
      for (int a = -5; a <= 5; ++a)
        for (int b = -5; b <= 5; ++b) {
          const double x = 0.32 * a + 0.03713;
          const double y = 0.32 * b + 0.04317;
          for (const Eigen::Vector3d& point : {
                   Eigen::Vector3d(x, y, 0.45119),
                   Eigen::Vector3d(2.73713, x, y + 1.6),
                   Eigen::Vector3d(x, 2.34317, y + 1.6)}) {
            const double time = index == 0 ? imu.stamp.time :
                imu.stamp.time - 0.007 - 0.002 * (scan.points.size() % 3);
            const Eigen::Vector3d raw = moving
                ? (orientation(time - 1).conjugate() *
                   (point - position(time - 1))).eval() : point;
            scan.points.push_back({raw, time});
          }
        }
      const bool collecting = deferred && index < 925;
      const auto reference = original.Observe(
          scan, collecting ? &waiting_baseline : &baseline);
      u::Result decision;
      if (asynchronous) {
        std::unique_ptr<NativeMotionBootstrap::Task> task;
        ASSERT_TRUE(bootstrap.PrepareTask(scan, local, &task).ok());
        const auto computed = task->Compute();
        decision = bootstrap.CommitTask(task.get(), &local);
        EXPECT_EQ(computed.reason, decision.reason);
      } else {
        decision = bootstrap.Observe(scan, collecting ? &waiting : &local);
      }
      EXPECT_EQ(reference.reason, decision.reason) << decision.message;
      EXPECT_TRUE(local.state().position.isApprox(baseline.state().position, 0));
      EXPECT_TRUE(local.state().orientation.isApprox(baseline.state().orientation, 0));
      EXPECT_TRUE(local.state().velocity.isApprox(baseline.state().velocity, 0));
      if (local.state().valid) break;
    }
    ASSERT_TRUE(local.state().valid);
    if (deferred) {
      EXPECT_GE(local.state().stamp.time, 10.25);
    }
    ASSERT_TRUE(local.joint_error_status().ok()) << local.joint_error_status().message;
    const auto initialized = local.state().stamp;
    u::LocalState startup;
    const auto lookup = local.JointErrorLookup(initialized.time, &startup);
    ASSERT_TRUE(lookup.ok()) << lookup.message;
    EXPECT_TRUE(startup.covariance_model_valid);
    EXPECT_FALSE(startup.uncertainty_qualified);
    EXPECT_FALSE(local.state().covariance_model_valid);
    const auto next = Imu(initialized.time + 0.01, initialized.sequence + 1);
    local.AddImu(next);
    baseline.AddImu(next);
    EXPECT_TRUE(local.state().position.isApprox(baseline.state().position, 0));
    EXPECT_TRUE(local.state().velocity.isApprox(baseline.state().velocity, 0));
    u::MotionIncrement motion;
    const auto connected =
        local.JointErrorMotionBetween(initialized.time, next.stamp.time, &motion);
    ASSERT_TRUE(connected.ok()) << connected.message;
    EXPECT_TRUE(motion.covariance.allFinite());
    EXPECT_EQ(motion.start.sequence, initialized.sequence);
    EXPECT_EQ(motion.end.sequence, next.stamp.sequence);
  }
}

}  // namespace
}  // namespace openloc::localization
