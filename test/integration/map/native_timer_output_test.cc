// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "test/integration/map/native_timer_output.h"

#include <future>
#include <vector>

#include "gtest/gtest.h"

namespace openloc::replay {
namespace {

TEST(NativeTimerOutputTest,
     HoldsMeasurementIdentityThenExpiresWithoutNewInput) {
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeTimerOutput::Publication> records;
  NativeTimerOutput timer(
      [&](const auto& record) {
        std::lock_guard<std::mutex> lock(mutex);
        records.push_back(record);
        condition.notify_all();
      },
      1000000000);
  localization::unified::LocalState state;
  state.valid = true;
  state.epoch = {"session", 0};
  state.stamp = {1.0, 1.0, 1, "unix"};
  state.position.x() = 7;
  timer.Update(state, true, 1000000000, 1000000000,
               NativeTimerOutput::Clock::now());
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(3), [&] {
      return !records.empty() && records.back().snapshot &&
             !records.back().usable;
    }));
  }
  timer.Finish();
  std::size_t new_samples = 0, held_samples = 0;
  for (const auto& record : records) {
    if (!record.snapshot) continue;
    EXPECT_EQ(record.snapshot->measurement_ns, 1000000000);
    EXPECT_EQ(record.snapshot->state.stamp.sequence, 1);
    EXPECT_EQ(record.snapshot->state.position.x(), 7);
    new_samples += record.new_measurement;
    held_samples += record.usable && !record.new_measurement;
    EXPECT_GE(record.emitted_steady_ns, record.target_steady_ns);
  }
  EXPECT_EQ(new_samples, 1);
  EXPECT_GT(held_samples, 0);
  EXPECT_FALSE(records.back().usable);
  EXPECT_FALSE(records.back().new_measurement);
}

TEST(NativeTimerOutputTest, CallbackFailureIsSurfacedAndSourceCannotRegress) {
  std::mutex mutex;
  std::condition_variable condition;
  bool called = false;
  NativeTimerOutput timer(
      [&](const auto& record) {
        if (!record.snapshot) return;
        {
          std::lock_guard<std::mutex> lock(mutex);
          called = true;
        }
        condition.notify_all();
        throw std::runtime_error("publisher failed");
      },
      100000000);
  localization::unified::LocalState state;
  state.stamp.sequence = 1;
  timer.Update(state, false, 1000000000, 1000000000,
               NativeTimerOutput::Clock::now());
  EXPECT_THROW(timer.Update(state, false, 1000000000, 1000000000,
                            NativeTimerOutput::Clock::now()),
               std::runtime_error);
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(3),
                                   [&] { return called; }));
  }
  EXPECT_THROW(timer.Finish(), std::runtime_error);
}

TEST(NativeTimerOutputTest, DelayedDeliveryCannotRenewOldMeasurementFreshness) {
  std::mutex mutex;
  std::condition_variable condition;
  bool observed = false;
  NativeTimerOutput::Publication publication;
  NativeTimerOutput timer(
      [&](const auto& record) {
        if (!record.snapshot) return;
        std::lock_guard<std::mutex> lock(mutex);
        publication = record;
        observed = true;
        condition.notify_all();
      },
      100000000);
  localization::unified::LocalState state;
  state.valid = true;
  state.stamp.sequence = 1;
  timer.Update(state, true, 1000000000, 1000000000,
               NativeTimerOutput::Clock::now() - std::chrono::seconds(1));
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(3),
                                   [&] { return observed; }));
  }
  timer.Finish();
  EXPECT_FALSE(publication.usable);
  EXPECT_FALSE(publication.new_measurement);
}

TEST(NativeTimerOutputTest, SlowPublisherDoesNotHoldTheEstimatorHandoffLock) {
  std::promise<void> entered, release;
  auto released = release.get_future().share();
  bool first = true;
  NativeTimerOutput timer(
      [&](const auto& record) {
        if (!record.snapshot) return;
        if (first) {
          first = false;
          entered.set_value();
          released.wait();
        }
      },
      100000000);
  localization::unified::LocalState state;
  state.stamp.sequence = 1;
  timer.Update(state, false, 1000000000, 1000000000,
               NativeTimerOutput::Clock::now());
  auto started = entered.get_future();
  const auto ready = started.wait_for(std::chrono::seconds(3));
  if (ready != std::future_status::ready) {
    release.set_value();
    FAIL() << "timer callback did not start";
  }
  state.stamp.sequence = 2;
  auto handoff = std::async(std::launch::async, [&] {
    timer.Update(state, false, 1005000000, 1005000000,
                 NativeTimerOutput::Clock::now());
  });
  const auto handed = handoff.wait_for(std::chrono::milliseconds(500));
  release.set_value();
  handoff.get();
  timer.Finish();
  EXPECT_EQ(handed, std::future_status::ready);
}

TEST(NativeTimerOutputTest, EpochChangeIsExplicitWithoutRestampingThePose) {
  std::mutex mutex;
  std::condition_variable condition;
  std::vector<NativeTimerOutput::Publication> records;
  NativeTimerOutput timer(
      [&](const auto& record) {
        std::lock_guard<std::mutex> lock(mutex);
        records.push_back(record);
        condition.notify_all();
      },
      1000000000);
  localization::unified::LocalState state;
  state.valid = true;
  state.stamp.sequence = 1;
  state.epoch = {"session", 0};
  timer.Update(state, true, 1000000000, 1000000000,
               NativeTimerOutput::Clock::now());
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(3), [&] {
      return !records.empty() && records.back().usable;
    }));
  }
  state.epoch.generation = 1;
  timer.Update(state, true, 1000000000, 1000000000,
               NativeTimerOutput::Clock::now());
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(3), [&] {
      return !records.empty() && records.back().epoch_changed;
    }));
  }
  timer.Finish();
  std::size_t changes = 0, fresh = 0;
  for (const auto& record : records) {
    changes += record.epoch_changed;
    fresh += record.new_measurement;
    if (record.usable) EXPECT_EQ(record.snapshot->measurement_ns, 1000000000);
  }
  EXPECT_EQ(changes, 1);
  EXPECT_EQ(fresh, 2);
}

}  // namespace
}  // namespace openloc::replay
