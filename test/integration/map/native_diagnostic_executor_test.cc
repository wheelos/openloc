// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "test/integration/map/native_diagnostic_executor.h"

#include <filesystem>
#include <future>
#include <vector>

#include "gtest/gtest.h"

namespace openloc::replay {
namespace {

std::string Journal(const char* name) {
  return (std::filesystem::path(::testing::TempDir()) / name).string();
}

TEST(NativeDiagnosticExecutorTest,
     SlowTaskDoesNotBlockProducerAndPreservesOrder) {
  NativeDiagnosticExecutor executor(Journal("slow.csv"), 3, 100);
  std::promise<void> entered;
  std::promise<void> release;
  const auto gate = release.get_future().share();
  std::vector<int> committed;
  executor.Submit("slow", 60, [&] {
    entered.set_value();
    gate.wait();
    committed.push_back(1);
  });
  entered.get_future().wait();
  executor.Submit("next", 40, [&] { committed.push_back(2); });
  EXPECT_THROW(executor.Submit("overflow", 1, [] {}), std::runtime_error);
  release.set_value();
  executor.Finish();
  EXPECT_EQ(committed, (std::vector<int>{1, 2}));
  EXPECT_THROW(executor.Submit("closed", 1, [] {}), std::runtime_error);
}

TEST(NativeDiagnosticExecutorTest, RunningTaskCountsAgainstTaskBound) {
  NativeDiagnosticExecutor executor(Journal("count.csv"), 1, 100);
  std::promise<void> entered;
  std::promise<void> release;
  const auto gate = release.get_future().share();
  executor.Submit("slow", 1, [&] {
    entered.set_value();
    gate.wait();
  });
  entered.get_future().wait();
  EXPECT_THROW(executor.Submit("overflow", 1, [] {}), std::runtime_error);
  release.set_value();
  executor.Finish();
}

TEST(NativeDiagnosticExecutorTest, FailureIsRethrownAndLaterWorkIsCancelled) {
  NativeDiagnosticExecutor executor(Journal("failure.csv"), 2, 100);
  std::promise<void> entered;
  std::promise<void> release;
  const auto gate = release.get_future().share();
  bool executed = false;
  executor.Submit("failure", 1, [&] {
    entered.set_value();
    gate.wait();
    throw std::runtime_error("original worker failure");
  });
  entered.get_future().wait();
  executor.Submit("cancelled", 1, [&] { executed = true; });
  release.set_value();
  try {
    executor.Finish();
    FAIL() << "worker failure was hidden";
  } catch (const std::runtime_error& error) {
    EXPECT_STREQ(error.what(), "original worker failure");
  }
  EXPECT_FALSE(executed);
  EXPECT_THROW(executor.Check(), std::runtime_error);
}

}  // namespace
}  // namespace openloc::replay
