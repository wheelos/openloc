// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include "candidates/common/types.h"

namespace openloc::replay {

// Host publication only: no extrapolation, timestamp renewal or estimator
// mutation. The callback is serialized on one timer thread, outside the lock.
class NativeTimerOutput {
 public:
  using Clock = std::chrono::steady_clock;
  struct Snapshot {
    localization::unified::LocalState state;
    bool integrity_valid;
    int64_t measurement_ns;
    int64_t receive_ns;
    Clock::time_point arrival;
  };
  struct Publication {
    std::shared_ptr<const Snapshot> snapshot;
    int64_t target_steady_ns;
    int64_t emitted_steady_ns;
    int64_t skipped_periods;
    bool usable = false;
    bool new_measurement = false;
    bool epoch_changed = false;
    int64_t measurement_age_ns = -1;
  };
  using Callback = std::function<void(const Publication&)>;

  explicit NativeTimerOutput(Callback callback, int64_t maximum_age_ns)
      : callback_(std::move(callback)), maximum_age_ns_(maximum_age_ns) {
    if (!callback_ || maximum_age_ns <= 0 || maximum_age_ns > 1000000000)
      throw std::invalid_argument("invalid timer callback or freshness bound");
    thread_ = std::thread([this] { Run(); });
  }

  ~NativeTimerOutput() { Stop(); }
  NativeTimerOutput(const NativeTimerOutput&) = delete;
  NativeTimerOutput& operator=(const NativeTimerOutput&) = delete;

  void Update(const localization::unified::LocalState& state,
              bool integrity_valid, int64_t measurement_ns, int64_t receive_ns,
              Clock::time_point arrival) {
    if (measurement_ns <= 0 || receive_ns <= 0 || arrival > Clock::now())
      throw std::invalid_argument("timer output requires sensor timestamps");
    auto next = std::make_shared<const Snapshot>(
        Snapshot{state, integrity_valid, measurement_ns, receive_ns, arrival});
    std::lock_guard<std::mutex> lock(mutex_);
    if (error_) std::rethrow_exception(error_);
    if (stopping_) throw std::runtime_error("timer output already stopped");
    if (latest_ && latest_->state.epoch == state.epoch &&
        (measurement_ns <= latest_->measurement_ns ||
         state.stamp.sequence <= latest_->state.stamp.sequence))
      throw std::runtime_error("timer output source frontier regressed");
    latest_ = std::move(next);
  }

  void Finish() {
    Stop();
    std::lock_guard<std::mutex> lock(mutex_);
    if (error_) std::rethrow_exception(error_);
  }

 private:
  static int64_t Nanoseconds(Clock::time_point time) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               time.time_since_epoch())
        .count();
  }

  void Stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_all();
    if (thread_.joinable()) thread_.join();
  }

  void Run() {
    try {
      constexpr int64_t period_ns = 10000000;
      const auto period = std::chrono::nanoseconds(period_ns);
      auto target = Clock::now() + period;
      std::shared_ptr<const Snapshot> last_emitted;
      for (;;) {
        std::shared_ptr<const Snapshot> snapshot;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          if (condition_.wait_until(lock, target, [this] { return stopping_; }))
            return;
          snapshot = latest_;
        }
        const auto now = Clock::now();
        Publication output{
            snapshot, Nanoseconds(target), Nanoseconds(now),
            (Nanoseconds(now) - Nanoseconds(target)) / period_ns};
        if (snapshot) {
          const int64_t age =
              std::max<int64_t>(
                  0, snapshot->receive_ns - snapshot->measurement_ns) +
              Nanoseconds(now) - Nanoseconds(snapshot->arrival);
          output.measurement_age_ns = age;
          output.usable = snapshot->state.valid && snapshot->integrity_valid &&
                          age <= maximum_age_ns_;
          output.epoch_changed =
              output.usable && last_emitted &&
              snapshot->state.epoch != last_emitted->state.epoch;
          output.new_measurement =
              output.usable &&
              (!last_emitted || output.epoch_changed ||
               snapshot->measurement_ns != last_emitted->measurement_ns);
          if (output.usable) last_emitted = snapshot;
        }
        callback_(output);
        // Missed ticks are explicit; never catch up with a publication burst.
        target += period * (output.skipped_periods + 1);
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      error_ = std::current_exception();
    }
  }

  Callback callback_;
  const int64_t maximum_age_ns_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::shared_ptr<const Snapshot> latest_;
  std::exception_ptr error_;
  bool stopping_ = false;
  std::thread thread_;
};

}  // namespace openloc::replay
