// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <cstddef>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <utility>

#include "candidates/local/continuous_output.h"
#include "candidates/local/local_lio.h"
#include "candidates/local/local_motion.h"
#include "candidates/local/motion_initializer.h"
#include "candidates/local/native_joint_error.h"
#include "candidates/local/stationarity_detector.h"

namespace openloc {
namespace localization {
namespace unified {

struct RelativeBootstrapEvidence;

// The local owner deliberately has no absolute-pose or global-reset input.
class LocalEstimator {
 public:
  struct InternalHistoryErrorMap {
    Stamp first;
    Stamp second;
    // Interior lookups use corrected left / pre-correction right history.
    // An exact lookup has first == second and only first_error is nonzero.
    bool second_propagated = false;
    Matrix15d first_error = Matrix15d::Zero();
    Matrix15d second_error = Matrix15d::Zero();
  };
  class LidarTask;
  using TraceCallback = std::function<void(const LocalEstimatorTrace&)>;

  explicit LocalEstimator(const LocalConfig& config);
  void SetTraceCallback(TraceCallback callback) {
    trace_callback_ = std::move(callback);
  }
  Result ValidateConfig() const;
  Result AddImu(const ImuSample& sample);
  Result AddWheel(const WheelSample& sample);
  Result AddLidar(const LidarScan& scan);
  // The host owns task execution/threading and retains every subsequent IMU.
  // Only IMU/LiDAR instances are supported; wheel updates need a separate
  // replay contract. A completed task must catch up to the exact live frontier.
  Result PrepareLidarTask(const LidarScan& scan,
                         std::unique_ptr<LidarTask>* task) const;
  Result CommitLidarTask(LidarTask* task);
  Result PrepareStartupSnapshot(std::unique_ptr<LocalEstimator>* snapshot) const;
  Result CommitStartupSnapshot(const LocalEstimator& snapshot);
  Result InitializeMotion(
      const std::vector<RelativeLidarPose>& relative_poses,
      MotionInitializationDiagnostics* diagnostics = nullptr,
      const RelativeBootstrapEvidence* evidence = nullptr);
  bool joint_error_enabled() const { return config_.enable_native_joint_error; }
  Result ResetStopped(double now);
  Result EvaluateIntegrity(double now) const;
  Result Evaluate(double now) const;
  // valid reports initialized, fresh, numerically sound propagation. Precision
  // and wheel-aided accuracy remain the separate Evaluate policy.
  const LocalState& state() const { return output_state_; }
  const LocalState& internal_state() const { return state_; }
  const SourceCounters& imu_health() const { return imu_health_; }
  const SourceCounters& wheel_health() const { return wheel_health_; }
  const SourceCounters& lidar_health() const { return lidar_health_; }
  const SourceCounters& zero_velocity_health() const { return zupt_health_; }
  // Null means no qualified JOINT_LOCAL_MARGINAL is available. In particular,
  // LiDAR-active continuous output remains publishable but motion-ineligible.
  const MotionIncrement* last_motion() const {
    return has_motion_ ? &last_motion_ : nullptr;
  }
  const Result& motion_status() const { return motion_status_; }
  // Exact committed measurement times only; no nearest-state substitution.
  Result Lookup(double time, LocalState* state) const;
  Result MotionBetween(double start_time, double end_time,
                       MotionIncrement* motion) const;
  Result JointErrorLookup(double time, LocalState* state) const {
    return joint_error_.Lookup(time, state);
  }
  Result JointErrorMotionBetween(double start, double end,
                                MotionIncrement* motion) const {
    return joint_error_.MotionBetween(start, end, motion);
  }
  const Result& joint_error_status() const { return joint_error_.status(); }
  Result PrepareJointLidarScan(uint64_t sequence, JointLidarScan* scan);
  Result PrepareStructuredLidarScan(uint64_t sequence, StructuredLidarScan* scan);
  Result StructuredErrorLookup(double time, StructuredLocalState* state) const {
    return joint_error_.StructuredLookup(time, state);
  }
  Result StructuredErrorMotionBetween(double start, double end,
                                      MotionIncrement* motion) const {
    return joint_error_.StructuredMotionBetween(start, end, motion);
  }
  NativeStructuredResourceUsage StructuredErrorResourceUsage() const {
    return joint_error_.StructuredResourceUsage();
  }
  Result OutputErrorTransition(ContinuousOutputErrorTransition* transition) const {
    if (!lio_active_)
      return {Reason::HISTORY_UNAVAILABLE, "Continuous LiDAR output is not active"};
    return continuous_output_.LinearizeLastStep(transition);
  }
  Result DeskewErrorMap(const LidarScan& scan,
                        LidarDeskewErrorMap* error_map) const;
  Result JointLidarInputErrorMap(
      const LidarScan& scan,
      unified::JointLidarInputErrorMap* error_map) const;
  Result StoredGeometryErrorMap(LidarStoredGeometryErrorMap* error_map) const;
  const Eigen::MatrixXd& lidar_observed_pose_basis() const {
    return lio_.observed_pose_basis();
  }

 private:
  friend class LocalEstimatorTestAccess;

  static constexpr std::size_t kMaxPendingWheelSamples = 64;

  Result Reject(Reason reason, const std::string& message,
                SourceCounters* counters);
  Result Initialize(const ImuSample& sample);
  Result Propagate(const ImuSample& start, const ImuSample& end,
                   LocalState* state, Matrix15d* error_transition,
                   NativeJointError* observer = nullptr,
                   Matrix15d* process_covariance = nullptr) const;
  Result UpdateWheel(const WheelSample& sample, const ImuSample& imu,
                     LocalState* state, Matrix15d* error_transition,
                     Matrix15d* process_covariance = nullptr);
  Result UpdateZeroVelocity(const ImuSample& imu, LocalState* state,
                            Matrix15d* error_transition,
                            Matrix15d* process_covariance = nullptr);
  void ConsumeStationaryWheels(double time);
  void RecordHistory(const Matrix15d& transition = Matrix15d::Identity(),
                     const std::vector<SourceSampleId>& sources = {},
                     bool motion_qualified = true);
  void RecordInternalHistory(bool lidar_correction = false);
  void SyncJointMemory();
  Result PrepareJointLidar(const LidarScan& scan,
                           LidarDeskewErrorMap* deskew,
                           std::vector<StoredPlaneError>* planes);
  void JointLidarIteration(const LioUpdate& update, const LocalState& prior,
                          const LocalState& before, const LocalState& after,
                          const LidarDeskewErrorMap& deskew,
                          const std::vector<StoredPlaneError>& planes);
  void StoreJointLidar(const LidarScan& scan,
                       const LidarDeskewErrorMap& deskew);
  Result LookupInternal(double time, LocalState* state,
                        InternalHistoryErrorMap* error_map = nullptr) const;

  struct HistoryEntry {
    LocalState state;
    Matrix15d transition;
    std::vector<SourceSampleId> sources;
    bool motion_qualified = true;
  };

  struct InternalHistoryEntry {
    LocalState propagated;
    LocalState corrected;
    bool has_lidar_correction = false;
  };

  LocalConfig config_;
  std::shared_ptr<const int> owner_identity_ = std::make_shared<const int>(0);
  LocalState state_;
  LocalState output_state_;
  // Working conditional gain, not the conservative published uncertainty.
  Matrix15d lidar_gain_covariance_ = Matrix15d::Identity();
  SourceCounters imu_health_;
  SourceCounters wheel_health_;
  SourceCounters lidar_health_;
  SourceCounters zupt_health_;
  WheelSample last_received_wheel_;
  WheelSample stationary_wheel_;
  WheelSample last_trusted_wheel_;
  ImuSample previous_imu_;
  bool initialized_ = false;
  bool faulted_ = false;
  bool has_imu_ = false;
  bool has_received_wheel_ = false;
  bool has_stationary_wheel_ = false;
  bool has_trusted_wheel_ = false;
  uint64_t last_imu_sequence_ = 0;
  uint64_t last_wheel_sequence_ = 0;
  StationarityDetector stationarity_;
  std::deque<WheelSample> pending_wheels_;
  std::deque<HistoryEntry> history_;
  std::deque<InternalHistoryEntry> internal_history_;
  std::deque<ImuSample> initialization_imu_;
  LocalLio lio_;
  ContinuousOutput continuous_output_;
  mutable NativeJointError joint_error_;
  struct RetainedJointScan {
    Stamp reference;
    std::vector<Eigen::Vector3d> points;
  };
  std::map<uint64_t, RetainedJointScan> joint_scans_;
  bool lio_active_ = false;
  std::vector<SourceSampleId> pending_lidar_sources_;
  MotionIncrement last_motion_;
  bool has_motion_ = false;
  TraceCallback trace_callback_;
  // Test-access-only ablations; normal callers always update both biases.
  bool wheel_updates_gyro_bias_ = true;
  bool wheel_updates_accel_bias_ = true;
  bool wheel_forward_velocity_only_ = false;
  Result motion_status_{Reason::HISTORY_UNAVAILABLE,
                        "Two committed local states are required"};
};

class LocalEstimator::LidarTask {
 public:
  Result Compute();
  Result ReplayImu(const ImuSample& sample);
  const Result& decision() const { return decision_; }
  const LocalState& internal_state() const { return snapshot_->internal_state(); }

 private:
  friend class LocalEstimator;
  LidarTask(const LocalEstimator& owner, const LidarScan& scan);
  std::unique_ptr<LocalEstimator> snapshot_;
  std::shared_ptr<const int> stochastic_fork_;
  LidarScan scan_;
  SourceCounters lidar_before_;
  Result decision_{Reason::HISTORY_UNAVAILABLE, "LiDAR task has not run"};
  bool computed_ = false;
  bool committed_ = false;
};

}  // namespace unified
}  // namespace localization
}  // namespace openloc
