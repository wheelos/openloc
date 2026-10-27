// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <fstream>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace openloc::replay {

// Host-only single-owner diagnostic execution. Admission never waits for slow
// work; overflow and worker errors are explicit, not silent observation drops.
class NativeDiagnosticExecutor {
 public:
  NativeDiagnosticExecutor(const std::string& journal, std::size_t max_tasks,
                           std::size_t max_bytes)
      : journal_(journal), max_tasks_(max_tasks), max_bytes_(max_bytes) {
    if (!journal_ || max_tasks == 0 || max_bytes == 0)
      throw std::runtime_error("invalid diagnostic executor bounds or journal");
    journal_ << "task_id,kind,payload_bytes,admitted_tasks,admitted_bytes,"
                "enqueue_steady_ns,start_steady_ns,finish_steady_ns,status\n";
    worker_ = std::thread([this] { Work(); });
  }

  NativeDiagnosticExecutor(const NativeDiagnosticExecutor&) = delete;
  NativeDiagnosticExecutor& operator=(const NativeDiagnosticExecutor&) = delete;

  ~NativeDiagnosticExecutor() {
    Stop();
    if (worker_.joinable()) worker_.join();
  }

  void Submit(std::string kind, std::size_t bytes, std::function<void()> run) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (failure_) std::rethrow_exception(failure_);
    if (stopping_ || !run)
      throw std::runtime_error(
          "diagnostic executor is closed or task is empty");
    if (outstanding_tasks_ >= max_tasks_ ||
        bytes > max_bytes_ - outstanding_bytes_)
      throw std::runtime_error("diagnostic executor bounded queue overflow");
    ++outstanding_tasks_;
    outstanding_bytes_ += bytes;
    tasks_.push_back({++last_id_, std::move(kind), bytes, outstanding_tasks_,
                      outstanding_bytes_, Now(), std::move(run)});
    ready_.notify_one();
  }

  void Check() const {
    std::lock_guard<std::mutex> guard(mutex_);
    if (failure_) std::rethrow_exception(failure_);
  }

  void Finish() {
    Stop();
    if (worker_.joinable()) worker_.join();
    Check();
  }

 private:
  using Clock = std::chrono::steady_clock;
  struct Task {
    std::size_t id;
    std::string kind;
    std::size_t bytes;
    std::size_t admitted_tasks;
    std::size_t admitted_bytes;
    int64_t enqueued;
    std::function<void()> run;
  };

  static int64_t Now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               Clock::now().time_since_epoch())
        .count();
  }

  void Stop() {
    std::lock_guard<std::mutex> guard(mutex_);
    stopping_ = true;
    ready_.notify_one();
  }

  void Record(const Task& task, int64_t start, int64_t end,
              const char* status) {
    journal_ << task.id << ',' << task.kind << ',' << task.bytes << ','
             << task.admitted_tasks << ',' << task.admitted_bytes << ','
             << task.enqueued << ',' << start << ',' << end << ',' << status
             << '\n';
    journal_.flush();
    if (!journal_)
      throw std::runtime_error("incomplete diagnostic executor journal");
  }

  void Work() {
    for (;;) {
      Task task;
      {
        std::unique_lock<std::mutex> guard(mutex_);
        ready_.wait(guard, [this] { return stopping_ || !tasks_.empty(); });
        if (tasks_.empty()) return;
        task = std::move(tasks_.front());
        tasks_.pop_front();
      }
      const int64_t start = Now();
      try {
        task.run();
        Record(task, start, Now(), "completed");
      } catch (...) {
        // Transport the original exception to the producer/Finish caller.
        const auto failure = std::current_exception();
        std::lock_guard<std::mutex> guard(mutex_);
        failure_ = failure;
        stopping_ = true;
        journal_ << task.id << ',' << task.kind << ',' << task.bytes << ','
                 << task.admitted_tasks << ',' << task.admitted_bytes << ','
                 << task.enqueued << ',' << start << ',' << Now()
                 << ",failed\n";
        for (const auto& cancelled : tasks_) {
          journal_ << cancelled.id << ',' << cancelled.kind << ','
                   << cancelled.bytes << ',' << cancelled.admitted_tasks << ','
                   << cancelled.admitted_bytes << ',' << cancelled.enqueued
                   << ",0,0,cancelled_after_failure\n";
        }
        journal_.flush();
        tasks_.clear();
        outstanding_tasks_ = 0;
        outstanding_bytes_ = 0;
        return;
      }
      std::lock_guard<std::mutex> guard(mutex_);
      --outstanding_tasks_;
      outstanding_bytes_ -= task.bytes;
    }
  }

  std::ofstream journal_;
  const std::size_t max_tasks_;
  const std::size_t max_bytes_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<Task> tasks_;
  std::thread worker_;
  std::exception_ptr failure_;
  std::size_t outstanding_tasks_ = 0;
  std::size_t outstanding_bytes_ = 0;
  std::size_t last_id_ = 0;
  bool stopping_ = false;
};

}  // namespace openloc::replay
