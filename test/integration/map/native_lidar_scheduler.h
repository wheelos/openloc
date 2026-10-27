// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <optional>
#include <stdexcept>

#include "openloc/localization/local/native_odometry.h"
#include "test/integration/map/replay_csv.h"

namespace openloc::replay {

// The local owner remains on the calling thread. Only an isolated snapshot's
// scan calculation runs in the worker; catch-up/commit happens on the owner.
class NativeLidarScheduler {
 public:
  using Local = localization::NativeOdometry;
  using State = localization::unified::LocalState;
  using Imu = localization::unified::ImuSample;
  using Scan = localization::unified::LidarScan;
  using Result = localization::unified::Result;
  using Reason = localization::unified::Reason;

  struct Completion {
    uint64_t sequence;
    Result decision;
    State before;
    State after;
    bool applied = false;
  };

  NativeLidarScheduler(const std::filesystem::path& output, int64_t delay_ns)
      : journal_(output / "local_lidar_tasks.csv"), delay_ns_(delay_ns) {
    if (!journal_ || delay_ns < 5000000 || delay_ns > 100000000)
      throw std::runtime_error("local LIO delay must be in [5,100] ms");
    journal_
        << "sequence,reference_ns,processing_ns,due_ns,status,reason,detail\n";
  }

  Result Submit(Local* owner, const Scan& scan, int64_t reference_ns) {
    if (task_) {
      const Result result{Reason::LIDAR_STALE,
                          "One local LiDAR snapshot is already outstanding"};
      Record(scan.stamp.sequence, reference_ns, reference_ns,
             reference_ns + delay_ns_, "rejected_busy", result);
      return result;
    }
    const Result prepared = owner->PrepareLidarTask(scan, &task_);
    if (!prepared.ok()) {
      Record(scan.stamp.sequence, reference_ns, reference_ns,
             reference_ns + delay_ns_, "prepare_rejected", prepared);
      return prepared;
    }
    sequence_ = scan.stamp.sequence;
    reference_ns_ = reference_ns;
    due_ns_ = reference_ns + delay_ns_;
    expired_ = false;
    imu_.clear();
    future_ = std::async(std::launch::async,
                         [task = task_.get()] { return task->Compute(); });
    const Result pending{Reason::HISTORY_UNAVAILABLE,
                         "Local LIO snapshot queued for delayed commit"};
    Record(sequence_, reference_ns_, reference_ns_, due_ns_, "queued", pending);
    return pending;
  }

  std::optional<Completion> Advance(Local* owner, const Imu& imu,
                                    int64_t processing_ns) {
    if (!task_) return std::nullopt;
    const bool ready =
        future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    if (expired_) {
      if (ready) {
        const auto result = future_.get();
        Record(sequence_, reference_ns_, processing_ns, due_ns_,
               "discarded_after_deadline", result);
        task_.reset();
      }
      return std::nullopt;
    }
    if (imu_.size() >= 128)
      throw std::runtime_error("local LIO IMU catch-up buffer overflow");
    imu_.push_back(imu);
    if (processing_ns < due_ns_) return std::nullopt;
    const State before = owner->internal_state();
    if (!ready) {
      expired_ = true;
      imu_.clear();
      const Result result{
          Reason::LIDAR_STALE,
          "Local LIO worker missed its source-time commit frontier"};
      Record(sequence_, reference_ns_, processing_ns, due_ns_,
             "deadline_missed", result);
      return Completion{sequence_, result, before, before, false};
    }
    const auto computed = future_.get();
    if (!computed.ok() && computed.reason != Reason::GEOMETRY_UNAVAILABLE) {
      Record(sequence_, reference_ns_, processing_ns, due_ns_,
             "compute_rejected", computed);
      task_.reset();
      return Completion{sequence_, computed, before, before, false};
    }
    for (const auto& received : imu_) {
      const auto result = task_->ReplayImu(received);
      if (!result.ok() && result.reason != Reason::WHEEL_STALE &&
          result.reason != Reason::CORRELATION_UNQUALIFIED &&
          result.reason != Reason::CORRECTION_BUDGET_EXCEEDED) {
        Record(sequence_, reference_ns_, processing_ns, due_ns_,
               "replay_failed", result);
        throw std::runtime_error(result.message);
      }
    }
    const Result result = owner->CommitLidarTask(task_.get());
    const bool applied =
        result.ok() || result.reason == Reason::GEOMETRY_UNAVAILABLE;
    Record(sequence_, reference_ns_, processing_ns, due_ns_,
           applied ? "committed" : "commit_rejected", result);
    const State after = owner->internal_state();
    task_.reset();
    imu_.clear();
    return Completion{sequence_, result, before, after, applied};
  }

  void Finish(int64_t processing_ns) {
    if (task_) {
      const auto result = future_.get();
      Record(sequence_, reference_ns_, processing_ns, due_ns_,
             expired_ ? "discarded_after_deadline" : "end_of_input_uncommitted",
             result);
      task_.reset();
    }
    journal_.flush();
    if (!journal_)
      throw std::runtime_error("incomplete local LIO task journal");
  }

 private:
  void Record(uint64_t sequence, int64_t reference, int64_t processing,
              int64_t due, const char* status, const Result& result) {
    journal_ << sequence << ',' << reference << ',' << processing << ',' << due
             << ',' << status << ','
             << localization::unified::ReasonName(result.reason) << ',';
    Text(journal_, result.message);
    journal_ << '\n';
    journal_.flush();
    if (!journal_)
      throw std::runtime_error("incomplete local LIO task journal");
  }

  std::ofstream journal_;
  const int64_t delay_ns_;
  uint64_t sequence_ = 0;
  int64_t reference_ns_ = 0;
  int64_t due_ns_ = 0;
  bool expired_ = false;
  std::deque<Imu> imu_;
  std::unique_ptr<Local::LidarTask> task_;
  // std::async's future joins before the task storage is destroyed, including
  // stack unwinding. The normal host calls Finish to surface worker errors.
  std::future<Result> future_;
};

}  // namespace openloc::replay
