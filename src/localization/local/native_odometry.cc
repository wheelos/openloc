// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "openloc/localization/local/native_odometry.h"

#include <utility>

#include "candidates/local/local_estimator.h"

namespace openloc::localization {

struct NativeOdometry::Impl {
  explicit Impl(const unified::LocalConfig& config) : owner(config) {}
  unified::LocalEstimator owner;
};

struct NativeOdometry::LidarTask::Impl {
  std::unique_ptr<unified::LocalEstimator::LidarTask> task;
};

struct NativeOdometry::StartupTask::Impl {
  std::unique_ptr<unified::LocalEstimator> snapshot;
  bool attempted = false;
  bool consumed = false;
};

NativeOdometry::NativeOdometry(const unified::LocalConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}
NativeOdometry::~NativeOdometry() = default;

unified::Result NativeOdometry::ValidateConfig() const {
  return impl_->owner.ValidateConfig();
}
unified::Result NativeOdometry::AddImu(const unified::ImuSample& sample) {
  return impl_->owner.AddImu(sample);
}
unified::Result NativeOdometry::AddWheel(const unified::WheelSample& sample) {
  return impl_->owner.AddWheel(sample);
}
unified::Result NativeOdometry::AddLidar(const unified::LidarScan& scan) {
  return impl_->owner.AddLidar(scan);
}
unified::Result NativeOdometry::InitializeMotion(
    const std::vector<unified::RelativeLidarPose>& poses,
    unified::MotionInitializationDiagnostics* diagnostics) {
  return impl_->owner.InitializeMotion(poses, diagnostics);
}
bool NativeOdometry::joint_error_enabled() const {
  return impl_->owner.joint_error_enabled();
}
unified::Result NativeOdometry::InitializeFromBootstrap(
    const std::vector<unified::RelativeLidarPose>& poses,
    unified::MotionInitializationDiagnostics* diagnostics,
    const unified::RelativeBootstrapEvidence& evidence) {
  return impl_->owner.InitializeMotion(poses, diagnostics, &evidence);
}
unified::Result NativeOdometry::ResetStopped(double now) {
  return impl_->owner.ResetStopped(now);
}
unified::Result NativeOdometry::EvaluateIntegrity(double now) const {
  return impl_->owner.EvaluateIntegrity(now);
}
unified::Result NativeOdometry::Evaluate(double now) const {
  return impl_->owner.Evaluate(now);
}
unified::Result NativeOdometry::Lookup(double time,
                                       unified::LocalState* state) const {
  return impl_->owner.Lookup(time, state);
}
unified::Result NativeOdometry::MotionBetween(
    double start_time, double end_time, unified::MotionIncrement* motion) const {
  return impl_->owner.MotionBetween(start_time, end_time, motion);
}
unified::Result NativeOdometry::OutputErrorTransition(
    unified::ContinuousOutputErrorTransition* transition) const {
  return impl_->owner.OutputErrorTransition(transition);
}
unified::Result NativeOdometry::JointErrorLookup(
    double time, unified::LocalState* state) const {
  return impl_->owner.JointErrorLookup(time, state);
}
unified::Result NativeOdometry::JointErrorMotionBetween(
    double start, double end, unified::MotionIncrement* motion) const {
  return impl_->owner.JointErrorMotionBetween(start, end, motion);
}
const unified::Result& NativeOdometry::joint_error_status() const {
  return impl_->owner.joint_error_status();
}
unified::Result NativeOdometry::PrepareJointLidarScan(
    uint64_t sequence, unified::JointLidarScan* scan) {
  return impl_->owner.PrepareJointLidarScan(sequence, scan);
}
unified::Result NativeOdometry::StructuredErrorLookup(
    double time, unified::StructuredLocalState* state) const {
  return impl_->owner.StructuredErrorLookup(time, state);
}
unified::Result NativeOdometry::PrepareStructuredLidarScan(
    uint64_t sequence, unified::StructuredLidarScan* scan) {
  return impl_->owner.PrepareStructuredLidarScan(sequence, scan);
}
unified::Result NativeOdometry::StructuredErrorMotionBetween(
    double start, double end, unified::MotionIncrement* motion) const {
  return impl_->owner.StructuredErrorMotionBetween(start, end, motion);
}
unified::NativeStructuredResourceUsage
NativeOdometry::StructuredErrorResourceUsage() const {
  return impl_->owner.StructuredErrorResourceUsage();
}
unified::Result NativeOdometry::DeskewErrorMap(
    const unified::LidarScan& scan,
    unified::LidarDeskewErrorMap* error_map) const {
  return impl_->owner.DeskewErrorMap(scan, error_map);
}

unified::Result NativeOdometry::JointLidarInputErrorMap(
    const unified::LidarScan& scan,
    unified::JointLidarInputErrorMap* error_map) const {
  return impl_->owner.JointLidarInputErrorMap(scan, error_map);
}
unified::Result NativeOdometry::StoredGeometryErrorMap(
    unified::LidarStoredGeometryErrorMap* error_map) const {
  return impl_->owner.StoredGeometryErrorMap(error_map);
}
const unified::LocalState& NativeOdometry::state() const {
  return impl_->owner.state();
}
const unified::LocalState& NativeOdometry::internal_state() const {
  return impl_->owner.internal_state();
}
bool NativeOdometry::motion_available() const {
  return impl_->owner.last_motion() != nullptr;
}
const unified::Result& NativeOdometry::motion_status() const {
  return impl_->owner.motion_status();
}
const unified::SourceCounters& NativeOdometry::imu_health() const {
  return impl_->owner.imu_health();
}
const unified::SourceCounters& NativeOdometry::wheel_health() const {
  return impl_->owner.wheel_health();
}
const unified::SourceCounters& NativeOdometry::lidar_health() const {
  return impl_->owner.lidar_health();
}
const Eigen::MatrixXd& NativeOdometry::lidar_observed_pose_basis() const {
  return impl_->owner.lidar_observed_pose_basis();
}
void NativeOdometry::SetTraceCallback(TraceCallback callback) {
  impl_->owner.SetTraceCallback(std::move(callback));
}

unified::Result NativeOdometry::PrepareLidarTask(
    const unified::LidarScan& scan, std::unique_ptr<LidarTask>* task) const {
  if (task == nullptr || *task)
    return {unified::Reason::INVALID_INPUT, "LiDAR task output must be empty"};
  auto next = std::make_unique<LidarTask::Impl>();
  const auto result = impl_->owner.PrepareLidarTask(scan, &next->task);
  if (!result.ok()) return result;
  *task = std::unique_ptr<LidarTask>(new LidarTask(std::move(next)));
  return result;
}

unified::Result NativeOdometry::PrepareStartupTask(
    std::unique_ptr<StartupTask>* task) const {
  if (!task || *task)
    return {unified::Reason::INVALID_INPUT, "Startup task output must be empty"};
  auto next = std::make_unique<StartupTask::Impl>();
  const auto result = impl_->owner.PrepareStartupSnapshot(&next->snapshot);
  if (!result.ok()) return result;
  *task = std::unique_ptr<StartupTask>(new StartupTask(std::move(next)));
  return {};
}

unified::Result NativeOdometry::CommitStartupTask(StartupTask* task) {
  if (!task || task->impl_->consumed)
    return {unified::Reason::INVALID_INPUT, "Startup task is missing or consumed"};
  const auto result = impl_->owner.CommitStartupSnapshot(*task->impl_->snapshot);
  if (result.ok()) task->impl_->consumed = true;
  return result;
}

NativeOdometry::StartupTask::StartupTask(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
NativeOdometry::StartupTask::~StartupTask() = default;
unified::Result NativeOdometry::StartupTask::InitializeMotion(
    const std::vector<unified::RelativeLidarPose>& poses,
    unified::MotionInitializationDiagnostics* diagnostics) {
  if (impl_->consumed || impl_->attempted)
    return {unified::Reason::INVALID_INPUT, "Startup task initialization already attempted"};
  impl_->attempted = true;
  return impl_->snapshot->InitializeMotion(poses, diagnostics);
}
bool NativeOdometry::StartupTask::joint_error_enabled() const {
  return impl_->snapshot->joint_error_enabled();
}
unified::Result NativeOdometry::StartupTask::InitializeFromBootstrap(
    const std::vector<unified::RelativeLidarPose>& poses,
    unified::MotionInitializationDiagnostics* diagnostics,
    const unified::RelativeBootstrapEvidence& evidence) {
  if (impl_->consumed || impl_->attempted)
    return {unified::Reason::INVALID_INPUT, "Startup task initialization already attempted"};
  impl_->attempted = true;
  return impl_->snapshot->InitializeMotion(poses, diagnostics, &evidence);
}
unified::Result NativeOdometry::StartupTask::ReplayImu(
    const unified::ImuSample& sample) {
  if (impl_->consumed)
    return {unified::Reason::INVALID_INPUT, "Startup task is consumed"};
  return impl_->snapshot->AddImu(sample);
}
const unified::LocalState& NativeOdometry::StartupTask::state() const {
  return impl_->snapshot->state();
}
unified::Result NativeOdometry::CommitLidarTask(LidarTask* task) {
  return impl_->owner.CommitLidarTask(task ? task->impl_->task.get() : nullptr);
}

NativeOdometry::LidarTask::LidarTask(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
NativeOdometry::LidarTask::~LidarTask() = default;
unified::Result NativeOdometry::LidarTask::Compute() {
  return impl_->task->Compute();
}
unified::Result NativeOdometry::LidarTask::ReplayImu(
    const unified::ImuSample& sample) {
  return impl_->task->ReplayImu(sample);
}
const unified::Result& NativeOdometry::LidarTask::decision() const {
  return impl_->task->decision();
}
const unified::LocalState& NativeOdometry::LidarTask::internal_state() const {
  return impl_->task->internal_state();
}

}  // namespace openloc::localization
