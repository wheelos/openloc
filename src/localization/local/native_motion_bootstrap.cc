// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "openloc/localization/local/native_motion_bootstrap.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <vector>

#include "candidates/map/map_matcher.h"
#include "candidates/local/motion_bootstrap_error.h"
#include "local/bootstrap_deskew.h"

namespace openloc::localization {

// Local relative registration only: target is the last raw scan,
// never a supplied prior map. Rotation-only deskew is provisional until joint
// alignment passes its excitation and residual checks.
struct NativeMotionBootstrap::Impl {
 public:
  using Imu = localization::unified::ImuSample;
  using Scan = localization::unified::LidarScan;
  using Result = localization::unified::Result;
  using Reason = localization::unified::Reason;
  using Cloud = localization::unified::Cloud;

  explicit Impl(bool translation_deskew, double voxel_size)
      : translation_deskew_(translation_deskew), voxel_size_(voxel_size),
        matcher_(Policy(voxel_size)) {}

  Impl(const Impl& other)
      : revision_(other.revision_), identity_(other.identity_),
        imu_(other.imu_), poses_(other.poses_),
        previous_(other.previous_), previous_rotation_(other.previous_rotation_),
        translation_deskew_(other.translation_deskew_),
        reference_velocity_(other.reference_velocity_),
        deskew_velocity_in_end_base_(other.deskew_velocity_in_end_base_),
        error_frames_(other.error_frames_), error_status_(other.error_status_),
        voxel_size_(other.voxel_size_), matcher_(Policy(voxel_size_)) {}

  const Eigen::Vector3d& deskew_velocity_in_end_base() const {
    return deskew_velocity_in_end_base_;
  }

  void AddImu(const Imu& sample) {
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    if (!imu_.empty()) {
      const double dt = sample.stamp.time - imu_.back().sample.stamp.time;
      if (dt <= 0.0 || dt > 0.05) {
        ++revision_;
        imu_.clear();
        poses_.clear();
        error_frames_.clear();
        error_status_ = {};
        previous_.reset();
        reference_velocity_.setZero();
        deskew_velocity_in_end_base_.setZero();
      } else {
        rotation =
            (imu_.back().rotation * localization::unified::ExpRotation(
                                        0.5 * dt *
                                        (sample.angular_velocity +
                                         imu_.back().sample.angular_velocity)))
                .normalized();
      }
    }
    imu_.push_back({sample, rotation});
    while (imu_.size() > 4096 ||
           (imu_.size() > 2 &&
            sample.stamp.time - imu_[1].sample.stamp.time > 10.0))
      imu_.pop_front();
  }

  Result Observe(const Scan& scan,
                 const std::function<Result(
                     const std::vector<localization::unified::RelativeLidarPose>&,
                     localization::unified::MotionInitializationDiagnostics*,
                     const unified::RelativeBootstrapEvidence*)>& initialize,
                 bool model_errors,
                 localization::unified::MotionInitializationDiagnostics*
                     diagnostics = nullptr) {
    if (diagnostics) *diagnostics = {};
    ++revision_;
    deskew_velocity_in_end_base_.setZero();
    if (imu_.empty() || scan.points.empty() ||
        scan.stamp.time != imu_.back().sample.stamp.time)
      return {Reason::HISTORY_UNAVAILABLE,
              "Bootstrap requires actual IMU frontier"};
    if (!poses_.empty() && scan.stamp.time - poses_.back().time < 0.25)
      return {Reason::HISTORY_UNAVAILABLE,
              "Bootstrap keyframe interval not reached"};
    if (scan.points.front().time < imu_.front().sample.stamp.time)
      return {Reason::HISTORY_UNAVAILABLE, "Bootstrap point lacks IMU bracket"};
    Cloud::Ptr cloud(new Cloud);
    const Eigen::Quaterniond end = imu_.back().rotation;
    if (translation_deskew_ && poses_.size() >= 2) {
      // Predict the current orientation from received gyro only. Translation
      // uses the last accepted scan interval, never the current/future match.
      const Eigen::Matrix3d reference_from_end =
          poses_.back().reference_from_base.linear() *
          (previous_rotation_.conjugate() * end).toRotationMatrix();
      deskew_velocity_in_end_base_ =
          reference_from_end.transpose() * reference_velocity_;
    }
    for (const auto& point : scan.points) {
      if (point.time < imu_.front().sample.stamp.time ||
          point.time > scan.stamp.time || !point.position.allFinite())
        return {Reason::POINT_TIME_MISSING,
                "Bootstrap point time/geometry invalid"};
      const auto upper = std::lower_bound(imu_.begin(), imu_.end(), point.time,
                                          [](const Entry& entry, double t) {
                                            return entry.sample.stamp.time < t;
                                          });
      Eigen::Quaterniond at = upper->rotation;
      if (upper->sample.stamp.time != point.time) {
        const auto lower = std::prev(upper);
        const double alpha =
            (point.time - lower->sample.stamp.time) /
            (upper->sample.stamp.time - lower->sample.stamp.time);
        at = lower->rotation.slerp(alpha, upper->rotation);
      }
      const Eigen::Vector3d p = detail::BootstrapDeskewPoint(
          point, scan.base_from_lidar, end.conjugate() * at,
          deskew_velocity_in_end_base_, scan.stamp.time);
      cloud->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
    }
    if (!previous_) {
      previous_ = cloud;
      previous_rotation_ = end;
      poses_.push_back({scan.stamp.time, Eigen::Isometry3d::Identity()});
      if (model_errors) RetainErrorFrame(scan, end, poses_.back().reference_from_base, {});
      return {Reason::HISTORY_UNAVAILABLE, "First relative bootstrap scan"};
    }
    auto result = matcher_.SetMap(previous_, model_errors);
    if (!result.ok()) return result;
    Eigen::Isometry3d initial = Eigen::Isometry3d::Identity();
    initial.linear() =
        (previous_rotation_.conjugate() * end).toRotationMatrix();
    localization::unified::MatchResult match;
    unified::RegistrationErrorMap errors;
    result = matcher_.Match(cloud, initial, &match, model_errors ? &errors : nullptr);
    if (!result.ok()) return result;
    if (!match.full_pose || match.ambiguous)
      return {Reason::DEGENERATE,
              "Bootstrap relative scan geometry is incomplete"};
    const Eigen::Isometry3d pose =
        poses_.back().reference_from_base * match.pose;
    reference_velocity_ =
        (pose.translation() - poses_.back().reference_from_base.translation()) /
        (scan.stamp.time - poses_.back().time);
    poses_.push_back({scan.stamp.time, pose});
    if (model_errors) RetainErrorFrame(scan, end, pose, std::move(errors));
    previous_ = cloud;
    previous_rotation_ = end;
    while (poses_.size() > 40 ||
           (poses_.size() > 1 && scan.stamp.time - poses_.front().time > 9.0))
      poses_.erase(poses_.begin());
    if (poses_.size() < 9 || poses_.back().time - poses_.front().time < 2.0)
      return {Reason::HISTORY_UNAVAILABLE, "Collecting relative motion window"};
    unified::RelativeBootstrapEvidence evidence;
    if (model_errors) {
      evidence.assessment = error_status_;
      evidence.frames = error_frames_;
      evidence.translation_deskew = translation_deskew_;
      evidence.first_imu_rotation = imu_.front().rotation;
      for (const auto& entry : imu_) evidence.imu.push_back(entry.sample);
      if (evidence.frames.size() >= poses_.size()) {
        evidence.first_pose_frame = evidence.frames.size() - poses_.size();
      } else if (evidence.assessment.ok()) {
        evidence.assessment = {
            Reason::CORRELATION_UNQUALIFIED,
            "Bootstrap raw registration history was retired before initialization"};
      }
    }
    return initialize(poses_, diagnostics, model_errors ? &evidence : nullptr);
  }

  uint64_t revision_ = 0;

 private:
  friend class NativeMotionBootstrap;
  void RetainErrorFrame(const Scan& scan, const Eigen::Quaterniond& rotation,
                        const Eigen::Isometry3d& pose,
                        unified::RegistrationErrorMap errors) {
    if (!error_status_.ok()) return;
    if (scan.points.size() > 100000 || error_frames_.size() >= 40) {
      error_status_ = {Reason::CORRELATION_UNQUALIFIED,
                       "Bootstrap raw registration support exceeded capacity"};
      error_frames_.clear();
      return;
    }
    if (!error_frames_.empty() && !errors.assessment.ok()) {
      error_status_ = errors.assessment;
      error_frames_.clear();
      return;
    }
    error_frames_.push_back({scan, pose, rotation, std::move(errors)});
  }
  static localization::unified::MatchConfig Policy(double voxel_size) {
    if (!std::isfinite(voxel_size) || voxel_size < 0.1 || voxel_size > 0.5)
      throw std::invalid_argument(
          "bootstrap voxel size must be in [0.1,0.5] m");
    localization::unified::MatchConfig policy;
    policy.source_voxel_size = voxel_size;
    policy.algorithm =
        localization::unified::RegistrationAlgorithm::kPointToPlaneIcp;
    policy.position_variance_floor = 0.01;
    policy.rotation_variance_floor = 0.001;
    return policy;
  }
  struct Entry {
    Imu sample;
    Eigen::Quaterniond rotation;
  };
  std::shared_ptr<const int> identity_ = std::make_shared<const int>(0);
  std::deque<Entry> imu_;
  std::vector<localization::unified::RelativeLidarPose> poses_;
  Cloud::Ptr previous_;
  Eigen::Quaterniond previous_rotation_ = Eigen::Quaterniond::Identity();
  bool translation_deskew_ = false;
  Eigen::Vector3d reference_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d deskew_velocity_in_end_base_ = Eigen::Vector3d::Zero();
  std::vector<unified::RelativeBootstrapFrame> error_frames_;
  Result error_status_;
  double voxel_size_;
  localization::unified::MapMatcher matcher_;
};

NativeMotionBootstrap::NativeMotionBootstrap(bool translation_deskew,
                                             double voxel_size)
    : impl_(std::make_unique<Impl>(translation_deskew, voxel_size)) {}

NativeMotionBootstrap::~NativeMotionBootstrap() = default;

void NativeMotionBootstrap::AddImu(const unified::ImuSample& sample) {
  impl_->AddImu(sample);
}

unified::Result NativeMotionBootstrap::Observe(
    const unified::LidarScan& scan, NativeOdometry* owner,
    unified::MotionInitializationDiagnostics* diagnostics) {
  if (diagnostics) *diagnostics = {};
  if (!owner)
    return {unified::Reason::INVALID_INPUT, "Bootstrap requires a local owner"};
  return impl_->Observe(scan,
      [owner](const auto& poses, auto* result, const auto* evidence) {
        return evidence ? owner->InitializeFromBootstrap(poses, result, *evidence)
                        : owner->InitializeMotion(poses, result);
      }, owner->joint_error_enabled(), diagnostics);
}

const Eigen::Vector3d& NativeMotionBootstrap::deskew_velocity_in_end_base()
    const {
  return impl_->deskew_velocity_in_end_base();
}

struct NativeMotionBootstrap::Task::Impl {
  std::unique_ptr<NativeMotionBootstrap::Impl> bootstrap;
  std::unique_ptr<NativeOdometry::StartupTask> owner;
  unified::LidarScan scan;
  unified::MotionInitializationDiagnostics diagnostics;
  unified::Result decision;
  uint64_t revision = 0;
  bool computed = false;
  bool consumed = false;
};

unified::Result NativeMotionBootstrap::PrepareTask(
    const unified::LidarScan& scan, const NativeOdometry& owner,
    std::unique_ptr<Task>* task) const {
  if (!task || *task || impl_->imu_.empty() ||
      scan.stamp.time != impl_->imu_.back().sample.stamp.time ||
      scan.stamp.time != owner.state().stamp.time ||
      scan.epoch != owner.state().epoch)
    return {unified::Reason::INVALID_INPUT, "Startup task needs the exact owner frontier"};
  auto next = std::make_unique<Task::Impl>();
  const auto result = owner.PrepareStartupTask(&next->owner);
  if (!result.ok()) return result;
  next->bootstrap = std::make_unique<Impl>(*impl_);
  next->scan = scan;
  next->revision = impl_->revision_;
  *task = std::unique_ptr<Task>(new Task(std::move(next)));
  return {};
}

unified::Result NativeMotionBootstrap::CommitTask(Task* task, NativeOdometry* owner) {
  if (!owner || !task || !task->impl_->computed || task->impl_->consumed ||
      impl_->identity_ != task->impl_->bootstrap->identity_ ||
      impl_->revision_ != task->impl_->revision)
    return {unified::Reason::INVALID_INPUT, "Startup task is incomplete, consumed or retired"};
  auto& candidate = *task->impl_;
  const auto& live = impl_->imu_.back().sample;
  const auto& replayed = candidate.bootstrap->imu_.back().sample;
  if (live.stamp.time != replayed.stamp.time ||
      live.stamp.receive_time != replayed.stamp.receive_time ||
      live.stamp.sequence != replayed.stamp.sequence ||
      live.stamp.clock_id != replayed.stamp.clock_id ||
      (live.acceleration.array() != replayed.acceleration.array()).any() ||
      (live.angular_velocity.array() != replayed.angular_velocity.array()).any())
    return {unified::Reason::HISTORY_UNAVAILABLE, "Startup task lacks bootstrap IMU catch-up"};
  const auto accepted = owner->CommitStartupTask(candidate.owner.get());
  if (!accepted.ok()) return accepted;
  impl_ = std::move(candidate.bootstrap);
  candidate.consumed = true;
  return candidate.decision;
}

NativeMotionBootstrap::Task::Task(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
NativeMotionBootstrap::Task::~Task() = default;
unified::Result NativeMotionBootstrap::Task::Compute() {
  if (impl_->computed || impl_->consumed)
    return {unified::Reason::INVALID_INPUT, "Startup task can only compute once"};
  impl_->computed = true;
  impl_->decision = impl_->bootstrap->Observe(impl_->scan,
      [this](const auto& poses, auto* diagnostics, const auto* evidence) {
        return evidence
            ? impl_->owner->InitializeFromBootstrap(poses, diagnostics, *evidence)
            : impl_->owner->InitializeMotion(poses, diagnostics);
      }, impl_->owner->joint_error_enabled(), &impl_->diagnostics);
  return impl_->decision;
}
unified::Result NativeMotionBootstrap::Task::ReplayImu(const unified::ImuSample& sample) {
  if (!impl_->computed || impl_->consumed)
    return {unified::Reason::INVALID_INPUT, "Startup catch-up requires a completed task"};
  const auto accepted = impl_->owner->ReplayImu(sample);
  if (!accepted.ok() &&
      !(impl_->owner->state().valid &&
        (accepted.reason == unified::Reason::CORRELATION_UNQUALIFIED ||
         accepted.reason == unified::Reason::WHEEL_STALE ||
         accepted.reason == unified::Reason::CORRECTION_BUDGET_EXCEEDED)))
    return accepted;
  impl_->bootstrap->AddImu(sample);
  return accepted;
}
const unified::MotionInitializationDiagnostics&
NativeMotionBootstrap::Task::diagnostics() const {
  return impl_->diagnostics;
}

}  // namespace openloc::localization
