// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "openloc/localization/local/motion_initialization_types.h"

namespace openloc::localization {

namespace unified {
struct RelativeBootstrapEvidence;
}
class NativeMotionBootstrap;

// Explicit native-candidate entry, not a production default or global owner.
// The host serializes owner calls and owns clocks, decoding, calibration,
// startup policy, worker scheduling and publication. NativeMotionBootstrap
// optionally supplies relative startup registration, without an absolute map.
class NativeOdometry {
 public:
  class LidarTask;
  class StartupTask;
  using TraceCallback =
      std::function<void(const unified::LocalEstimatorTrace&)>;

  explicit NativeOdometry(const unified::LocalConfig& config);
  ~NativeOdometry();
  NativeOdometry(const NativeOdometry&) = delete;
  NativeOdometry& operator=(const NativeOdometry&) = delete;

  unified::Result ValidateConfig() const;
  unified::Result AddImu(const unified::ImuSample& sample);
  unified::Result AddWheel(const unified::WheelSample& sample);
  unified::Result AddLidar(const unified::LidarScan& scan);
  // Metric relative IMU-origin poses only; no absolute map/GNSS pose seed.
  unified::Result InitializeMotion(
      const std::vector<unified::RelativeLidarPose>& poses,
      unified::MotionInitializationDiagnostics* diagnostics = nullptr);
  unified::Result ResetStopped(double now);
  // Integrity does not establish capability, calibrated uncertainty or trust.
  unified::Result EvaluateIntegrity(double now) const;
  unified::Result Evaluate(double now) const;
  unified::Result Lookup(double time, unified::LocalState* state) const;
  // Exact retained endpoints only. Returns the actual owner's joint motion,
  // including shared source identities, without inferring independence or
  // qualifying uncertainty. Unmodeled LIO/startup remains explicitly rejected.
  unified::Result MotionBetween(double start_time, double end_time,
                                unified::MotionIncrement* motion) const;
  // Opt-in uncalibrated frozen-linearization model of the same nominal owner.
  // These snapshots/motions form a separate, coherent covariance contract;
  // never combine them with state()/Lookup() diagnostic marginals.
  unified::Result JointErrorLookup(double time, unified::LocalState* state) const;
  unified::Result JointErrorMotionBetween(
      double start_time, double end_time, unified::MotionIncrement* motion) const;
  const unified::Result& joint_error_status() const;
  // Once per IMU frontier, after serialized LiDAR commits and before exporting
  // that frontier to the graph. Finalizes its augmented basis without changing
  // ODOM or its published marginal. Delayed scans keep their original reference
  // time: conditioning_state may be later, never a substituted measurement time.
  unified::Result PrepareJointLidarScan(uint64_t sequence,
                                       unified::JointLidarScan* scan);
  // Exact source-aware alternative, not a dense sufficient graph state.
  // Available only in explicitly enabled structured mode. All raw-point and
  // temporal dependencies remain queryable; no uncertainty qualification.
  unified::Result StructuredErrorLookup(
      double time, unified::StructuredLocalState* state) const;
  unified::Result PrepareStructuredLidarScan(
      uint64_t sequence, unified::StructuredLidarScan* scan);
  // Finite endpoint/motion covariance only; not a Markov graph factor.
  unified::Result StructuredErrorMotionBetween(
      double start_time, double end_time, unified::MotionIncrement* motion) const;
  unified::NativeStructuredResourceUsage StructuredErrorResourceUsage() const;
  // On-demand local linearization includes the smoother's 3D correction-speed
  // memory. It is not a stochastic model or uncertainty qualification.
  unified::Result OutputErrorTransition(
      unified::ContinuousOutputErrorTransition* transition) const;
  // Exact retained history, including the pre-correction interpolation endpoint.
  // Shared clones are indexed once. Does not admit a scan, add covariance,
  // qualify uncertainty or alter ODOM.
  unified::Result DeskewErrorMap(const unified::LidarScan& scan,
                                unified::LidarDeskewErrorMap* error_map) const;
  // Structured support for a fresh scan BEFORE AddLidar. Does not insert raw
  // drivers, finalize a graph frontier, or replace PrepareJointLidarScan.
  unified::Result JointLidarInputErrorMap(
      const unified::LidarScan& scan,
      unified::JointLidarInputErrorMap* error_map) const;
  // Actual frozen plane support/source identities and anchor/point derivatives.
  // Does not manufacture the missing joint stochastic geometry model.
  unified::Result StoredGeometryErrorMap(
      unified::LidarStoredGeometryErrorMap* error_map) const;
  const unified::LocalState& state() const;
  const unified::LocalState& internal_state() const;
  bool motion_available() const;
  const unified::Result& motion_status() const;
  const unified::SourceCounters& imu_health() const;
  const unified::SourceCounters& wheel_health() const;
  const unified::SourceCounters& lidar_health() const;
  const Eigen::MatrixXd& lidar_observed_pose_basis() const;
  void SetTraceCallback(TraceCallback callback);

  // Compute/replay may run on one host worker while the owner advances.
  // Join before commit; replay every accepted IMU to the exact live frontier.
  // Wheel-active instances remain unsupported by the existing task contract.
  unified::Result PrepareLidarTask(const unified::LidarScan& scan,
                                   std::unique_ptr<LidarTask>* task) const;
  unified::Result CommitLidarTask(LidarTask* task);
  // A startup snapshot may initialize only from relative motion. Replay every
  // accepted IMU and join before committing at the exact uninitialized owner.
  unified::Result PrepareStartupTask(std::unique_ptr<StartupTask>* task) const;
  unified::Result CommitStartupTask(StartupTask* task);

 private:
  friend class NativeMotionBootstrap;
  bool joint_error_enabled() const;
  unified::Result InitializeFromBootstrap(
      const std::vector<unified::RelativeLidarPose>& poses,
      unified::MotionInitializationDiagnostics* diagnostics,
      const unified::RelativeBootstrapEvidence& evidence);
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class NativeOdometry::StartupTask {
 public:
  ~StartupTask();
  StartupTask(const StartupTask&) = delete;
  StartupTask& operator=(const StartupTask&) = delete;
  unified::Result InitializeMotion(
      const std::vector<unified::RelativeLidarPose>& poses,
      unified::MotionInitializationDiagnostics* diagnostics = nullptr);
  unified::Result ReplayImu(const unified::ImuSample& sample);
  const unified::LocalState& state() const;

 private:
  friend class NativeOdometry;
  friend class NativeMotionBootstrap;
  bool joint_error_enabled() const;
  unified::Result InitializeFromBootstrap(
      const std::vector<unified::RelativeLidarPose>& poses,
      unified::MotionInitializationDiagnostics* diagnostics,
      const unified::RelativeBootstrapEvidence& evidence);
  struct Impl;
  explicit StartupTask(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

class NativeOdometry::LidarTask {
 public:
  ~LidarTask();
  LidarTask(const LidarTask&) = delete;
  LidarTask& operator=(const LidarTask&) = delete;

  unified::Result Compute();
  unified::Result ReplayImu(const unified::ImuSample& sample);
  const unified::Result& decision() const;
  const unified::LocalState& internal_state() const;

 private:
  friend class NativeOdometry;
  struct Impl;
  explicit LidarTask(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace openloc::localization
