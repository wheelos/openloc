// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <memory>

#include "openloc/localization/local/native_odometry.h"

namespace openloc::localization {

// Relative raw-scan startup only; a supplied map or absolute pose is never
// used. The host feeds the same calibrated IMU as the local owner, in order,
// and calls Observe at that owner's actual IMU frontier. Calls are serialized
// by the host. This provisional registration does not qualify uncertainty or
// graph motion.
class NativeMotionBootstrap {
 public:
  class Task;
  explicit NativeMotionBootstrap(bool translation_deskew = false,
                                 double voxel_size = 0.5);
  ~NativeMotionBootstrap();
  NativeMotionBootstrap(const NativeMotionBootstrap&) = delete;
  NativeMotionBootstrap& operator=(const NativeMotionBootstrap&) = delete;

  void AddImu(const unified::ImuSample& sample);
  unified::Result Observe(
      const unified::LidarScan& scan, NativeOdometry* owner,
      unified::MotionInitializationDiagnostics* diagnostics = nullptr);
  const Eigen::Vector3d& deskew_velocity_in_end_base() const;
  unified::Result PrepareTask(const unified::LidarScan& scan,
                             const NativeOdometry& owner,
                             std::unique_ptr<Task>* task) const;
  unified::Result CommitTask(Task* task, NativeOdometry* owner);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class NativeMotionBootstrap::Task {
 public:
  ~Task();
  Task(const Task&) = delete;
  Task& operator=(const Task&) = delete;
  unified::Result Compute();
  unified::Result ReplayImu(const unified::ImuSample& sample);
  const unified::MotionInitializationDiagnostics& diagnostics() const;

 private:
  friend class NativeMotionBootstrap;
  struct Impl;
  explicit Task(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace openloc::localization
