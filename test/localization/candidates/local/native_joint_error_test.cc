// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/local/native_joint_error.h"

#include <limits>

#include "gtest/gtest.h"

namespace openloc::localization::unified {
namespace {

LocalState Initial() {
  LocalState state;
  state.valid = true;
  state.epoch = {"joint-memory", 1};
  state.stamp = {1, 1, 1, "unix"};
  state.covariance = Matrix15d::Identity();
  return state;
}

void CopyOutput(NativeJointError* model) {
  const auto internal = model->Get("internal");
  Eigen::MatrixXd output = Eigen::MatrixXd::Zero(18, internal.cols());
  output.topRows(15) = internal;
  model->Set("output", output);
}

TEST(NativeJointErrorTest, SharedSingularMemoryGrowsAndRetiresWithoutNoise) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  NativeJointError model(config);
  auto state = Initial();
  model.Initialize(state);
  model.Set("clone", model.Get("internal"));
  model.Publish(state, {});
  LocalState first;
  ASSERT_TRUE(model.Lookup(1, &first).ok());
  ASSERT_EQ(first.joint_error->covariance.rows(), 15);

  model.Independent("raw", 2 * Eigen::Matrix3d::Identity());
  const Eigen::MatrixXd anchor = model.Get("internal").topRows(3);
  model.Set("point", anchor + model.Get("raw"));
  model.Drop("raw");
  model.Set("internal", 2 * model.Get("internal"));
  CopyOutput(&model);
  state.stamp = {2, 2, 2, "unix"};
  model.Publish(state, {{"imu", 1}, {"imu", 2}, {"lidar_raw", 1}});
  LocalState second;
  ASSERT_TRUE(model.Lookup(2, &second).ok()) << model.status().message;
  ASSERT_EQ(second.joint_error->covariance.rows(), 18);
  EXPECT_TRUE(second.covariance.isApprox(4 * Matrix15d::Identity(), 1e-12));
  const auto point = model.Get("point");
  EXPECT_TRUE((point * point.transpose()).isApprox(
      3 * Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_TRUE((model.Get("internal") * point.transpose()).topRows(3).isApprox(
      2 * Eigen::Matrix3d::Identity(), 1e-12));

  model.Retain({"internal", "output"});
  state.stamp = {3, 3, 3, "unix"};
  model.Publish(state, {{"imu", 2}, {"imu", 3}});
  LocalState third;
  ASSERT_TRUE(model.Lookup(3, &third).ok());
  ASSERT_EQ(third.joint_error->covariance.rows(), 15);
  MotionIncrement motion;
  const auto exported = model.MotionBetween(1, 3, &motion);
  ASSERT_TRUE(exported.ok()) << exported.message;
  EXPECT_TRUE(motion.cross_covariance.isApprox(
      2 * Matrix15d::Identity(), 1e-12));
  EXPECT_EQ(motion.joint_cross_covariance.rows(), 15);
  EXPECT_EQ(motion.joint_cross_covariance.cols(), 15);
}

TEST(NativeJointErrorTest, ReusedPointAcrossQueryGapNeedsHiddenTemporalMemory) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  NativeJointError model(config);
  auto state = Initial();
  model.Initialize(state);
  model.Independent("point:a", Eigen::Matrix3d::Identity());
  model.Independent("point:b", Eigen::Matrix3d::Identity());
  for (int step = 0; step < 3; ++step) {
    Eigen::MatrixXd output =
        Eigen::MatrixXd::Zero(18, model.Get("internal").cols());
    output.topRows(15) = model.Get("internal");
    if (step != 1) output.row(0) += model.Get("point:a").row(0);
    if (step != 0) output.row(0) += model.Get("point:b").row(0);
    model.Set("output", output);
    state.stamp = {1.0 + step, 1.0 + step, uint64_t(1 + step), "unix"};
    const std::vector<SourceSampleId> sources =
        step == 0 ? std::vector<SourceSampleId>{{"imu", 1}}
                  : std::vector<SourceSampleId>{{"imu", uint64_t(step)},
                                                {"imu", uint64_t(1 + step)}};
    model.Publish(state, sources);
    ASSERT_TRUE(model.active()) << model.status().message;
  }
  MotionIncrement first, second, whole;
  const auto first_result = model.MotionBetween(1, 2, &first);
  const auto second_result = model.MotionBetween(2, 3, &second);
  const auto whole_result = model.MotionBetween(1, 3, &whole);
  ASSERT_TRUE(first_result.ok()) << first_result.message;
  ASSERT_TRUE(second_result.ok()) << second_result.message;
  ASSERT_TRUE(whole_result.ok()) << whole_result.message;
  EXPECT_NEAR(2, whole.cross_covariance(0, 0), 1e-12);
  // Exact adjacent 15D marginals alone sever the reused point across t=2.
  const Matrix15d query_only_cross =
      first.cross_covariance *
      first.end_covariance.llt().solve(second.cross_covariance);
  EXPECT_NEAR(1, query_only_cross(0, 0), 1e-12);
  EXPECT_GT((whole.cross_covariance - query_only_cross).norm(), 0.9);
}

TEST(NativeJointErrorTest, CapacityAndInvalidDrivingNoiseDoNotFakeAState) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  config.native_joint_max_dimension = 15;
  NativeJointError model(config);
  auto state = Initial();
  model.Initialize(state);
  model.Publish(state, {});
  model.Independent("shared_point", Eigen::Matrix3d::Identity());
  state.stamp = {2, 2, 2, "unix"};
  model.Publish(state, {{"imu", 1}, {"imu", 2}});
  ASSERT_FALSE(model.active());
  EXPECT_EQ(model.status().reason, Reason::CORRELATION_UNQUALIFIED);
  LocalState unchanged = Initial();
  EXPECT_EQ(model.Lookup(2, &unchanged).reason, Reason::CORRELATION_UNQUALIFIED);
  EXPECT_DOUBLE_EQ(unchanged.stamp.time, 1);
  ASSERT_TRUE(model.Lookup(1, &unchanged).ok());
  model.Initialize(Initial());
  Eigen::Matrix3d invalid = Eigen::Matrix3d::Identity();
  invalid(0, 0) = -1;
  model.Independent("invalid", invalid);
  EXPECT_FALSE(model.active());
  EXPECT_NE(model.status().message.find("not PSD"), std::string::npos);
}

TEST(NativeJointErrorTest, PhysicalRowsCanExhaustBeforeIndependentDimension) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  config.native_joint_max_dimension = 100;
  config.native_joint_max_memory_rows = 39;
  NativeJointError model(config);
  auto state = Initial();
  model.Initialize(state);
  ASSERT_TRUE(model.active());
  const Eigen::MatrixXd internal = model.Get("internal");
  ASSERT_EQ(internal.cols(), 15);
  model.Set("shared-copy", internal.topRows(6));
  ASSERT_TRUE(model.active());
  EXPECT_EQ(model.Get("shared-copy").cols(), 15);
  model.Set("one-more-row", internal.topRows(1));
  EXPECT_FALSE(model.active());
  EXPECT_EQ(model.status().reason, Reason::CORRELATION_UNQUALIFIED);
  EXPECT_NE(model.status().message.find("physical-memory capacity"),
            std::string::npos);
  EXPECT_NE(model.status().message.find("no dependency dropped"),
            std::string::npos);
  EXPECT_NE(model.status().message.find("key=one-more-row"), std::string::npos);
  EXPECT_NE(model.status().message.find("stored_rows=39"), std::string::npos);
  EXPECT_NE(model.status().message.find("incoming_rows=1"), std::string::npos);
  EXPECT_NE(model.status().message.find("required_rows=40"), std::string::npos);
  EXPECT_NE(model.status().message.find("driver_dimension=15"),
            std::string::npos);
}

TEST(NativeJointErrorTest, PointBatchPreflightMatchesActualRowCapacity) {
  for (const std::size_t points : {1U, 4U, 12U}) {
    SCOPED_TRACE(points);
    LocalConfig config;
    config.enable_native_joint_error = true;
    config.native_joint_max_memory_rows = 33 + 18 + 3 * points;
    NativeJointError model(config);
    model.Initialize(Initial());
    const auto internal = model.Get("internal");
    const auto output = model.Get("output");
    EXPECT_TRUE(
        model.CheckAdditionalCapacity(18 + 3 * points, 3 * points).ok());
    const auto refused =
        model.CheckAdditionalCapacity(18 + 3 * (points + 1), 3 * (points + 1));
    EXPECT_EQ(refused.reason, Reason::CORRELATION_UNQUALIFIED);
    EXPECT_TRUE(model.active());
    EXPECT_TRUE(model.Get("internal").isApprox(internal, 0));
    EXPECT_TRUE(model.Get("output").isApprox(output, 0));
    model.Set("lidar_prior", model.Get("internal"));
    for (std::size_t point = 0; point < points; ++point) {
      model.Independent("raw_point", Eigen::Matrix3d::Identity());
      const Eigen::MatrixXd loading =
          model.Get("internal").topRows(3) + model.Get("raw_point");
      model.Set("deskew:" + std::to_string(point), loading);
      ASSERT_TRUE(model.active()) << model.status().message;
    }
    model.Set("one-more-row",
              Eigen::MatrixXd::Zero(1, model.Get("internal").cols()));
    EXPECT_FALSE(model.active());
  }
}

TEST(NativeJointErrorTest, PreflightChecksDriversAndOverflowWithoutMutation) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  config.native_joint_max_memory_rows = 64;
  NativeJointError model(config);
  model.Initialize(Initial());
  const auto internal = model.Get("internal");
  EXPECT_TRUE(model.CheckAdditionalCapacity(0, 49).ok());
  auto refused = model.CheckAdditionalCapacity(0, 50);
  EXPECT_EQ(refused.reason, Reason::CORRELATION_UNQUALIFIED);
  EXPECT_NE(refused.message.find("driver_dimension=15"), std::string::npos);
  EXPECT_NE(refused.message.find("additional_drivers=50"), std::string::npos);
  EXPECT_FALSE(
      model.CheckAdditionalCapacity(std::numeric_limits<std::size_t>::max(), 0)
          .ok());
  EXPECT_FALSE(
      model.CheckAdditionalCapacity(0, std::numeric_limits<std::size_t>::max())
          .ok());
  EXPECT_TRUE(model.active());
  EXPECT_TRUE(model.Get("internal").isApprox(internal, 0));
  model.Fail({Reason::CORRELATION_UNQUALIFIED, "original failure"});
  EXPECT_EQ(model.CheckAdditionalCapacity(0, 0).message, "original failure");
}

TEST(NativeJointErrorTest, IndependentPointsRetainSharedPoseAndFullNoiseRank) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  NativeJointError model(config);
  auto state = Initial();
  model.Initialize(state);
  const Eigen::Matrix3d transform =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  constexpr int points = 4;
  for (int point = 0; point < points; ++point) {
    model.Independent("raw", 2 * Eigen::Matrix3d::Identity());
    const Eigen::MatrixXd loading =
        3 * model.Get("internal").topRows(3) + transform * model.Get("raw");
    model.Set("deskew:" + std::to_string(point), loading);
  }
  model.Drop("raw");
  model.Publish(state, {});
  ASSERT_TRUE(model.active()) << model.status().message;
  LocalState snapshot;
  ASSERT_TRUE(model.Lookup(1, &snapshot).ok());
  EXPECT_EQ(snapshot.joint_error->covariance.rows(), 15 + 3 * points);
  const auto first = model.Get("deskew:0");
  const auto second = model.Get("deskew:1");
  EXPECT_TRUE((first * first.transpose())
                  .isApprox(11 * Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_TRUE((first * second.transpose())
                  .isApprox(9 * Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_TRUE((model.Get("internal") * first.transpose())
                  .topRows(3)
                  .isApprox(3 * Eigen::Matrix3d::Identity(), 1e-12));
  EXPECT_TRUE(((first - second) * (first - second).transpose())
                  .isApprox(4 * Eigen::Matrix3d::Identity(), 1e-12));
}

TEST(NativeJointErrorTest, PointBatchPreflightCountsOutstandingTaskPins) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  config.native_joint_max_memory_rows = 78;
  NativeJointError model(config);
  model.Initialize(Initial());
  auto fork = model.PinFork();
  ASSERT_TRUE(fork);
  // 33 initial + 15 fork-root + 24 prior/raw/deskew + 6 pinned raw drivers.
  EXPECT_TRUE(model.CheckAdditionalCapacity(24, 6).ok());
  const auto refused = model.CheckAdditionalCapacity(27, 9);
  EXPECT_EQ(refused.reason, Reason::CORRELATION_UNQUALIFIED);
  EXPECT_NE(refused.message.find("pinned_driver_rows=9"), std::string::npos);
  fork.reset();
  EXPECT_TRUE(model.CheckAdditionalCapacity(27, 9).ok());
}

TEST(NativeJointErrorTest, SingularNoisePreservesSmallPhysicalScales) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  NativeJointError model(config);
  model.Initialize(Initial());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance(0, 0) = 1e-30;
  covariance(1, 1) = 1;
  model.Independent("point", covariance);
  ASSERT_TRUE(model.active()) << model.status().message;
  const auto loading = model.Get("point");
  EXPECT_NEAR((loading * loading.transpose())(0, 0), 1e-30, 1e-44);
  model.Publish(Initial(), {});
  ASSERT_TRUE(model.active()) << model.status().message;
  LocalState snapshot;
  ASSERT_TRUE(model.Lookup(1, &snapshot).ok());
  EXPECT_EQ(snapshot.joint_error->covariance.rows(), 17);
}

TEST(NativeJointErrorTest, WeakSharedMemoryDirectionIsNotMadeDeterministic) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  NativeJointError model(config);
  model.Initialize(Initial());
  model.Independent("raw", 1e-24 * Eigen::Matrix3d::Identity());
  model.Set("point", model.Get("internal").topRows(3) + model.Get("raw"));
  model.Drop("raw");
  model.Publish(Initial(), {});
  ASSERT_TRUE(model.active()) << model.status().message;
  LocalState snapshot;
  ASSERT_TRUE(model.Lookup(1, &snapshot).ok());
  EXPECT_EQ(snapshot.joint_error->covariance.rows(), 18);
}

TEST(NativeJointErrorTest, ReusesTheSameRawImuEndpointAcrossAdjacentIntervals) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  config.native_imu_sample_period = 1;
  config.accel_noise = 1;
  config.gyro_noise = config.gyro_bias_noise = config.accel_bias_noise = 0;
  NativeJointError model(config);
  auto state = Initial();
  model.Initialize(state);
  model.Publish(state, {});
  ImuSample first, second, third;
  first.stamp = state.stamp;
  second.stamp = {2, 2, 2, "unix"};
  third.stamp = {3, 3, 3, "unix"};
  Eigen::Matrix<double, 15, 6> input = Eigen::Matrix<double, 15, 6>::Zero();
  input(3, 0) = 0.5;
  model.BeginImu(first, second);
  model.AdvanceImu(Matrix15d::Identity(), input, input, 1, 2);
  CopyOutput(&model);
  model.Retain({"internal", "output", "imu:last"});
  state.stamp = second.stamp;
  model.Publish(state, {{"imu", 1}, {"imu", 2}});
  LocalState at_second;
  ASSERT_TRUE(model.Lookup(2, &at_second).ok()) << model.status().message;
  EXPECT_NEAR(at_second.covariance(3, 3), 1.5, 1e-12);
  model.BeginImu(second, third);
  model.AdvanceImu(Matrix15d::Identity(), input, input, 2, 3);
  CopyOutput(&model);
  model.Retain({"internal", "output", "imu:last"});
  state.stamp = third.stamp;
  model.Publish(state, {{"imu", 2}, {"imu", 3}});
  LocalState at_third;
  ASSERT_TRUE(model.Lookup(3, &at_third).ok()) << model.status().message;
  EXPECT_NEAR(at_third.covariance(3, 3), 2.5, 1e-12);
  MotionIncrement motion;
  const auto connected = model.MotionBetween(2, 3, &motion);
  ASSERT_TRUE(connected.ok()) << connected.message;
  EXPECT_NEAR(motion.cross_covariance(3, 3), 1.75, 1e-12);
}

TEST(NativeJointErrorTest, StationaryMeanAndLastSampleShareTheSameError) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  config.native_imu_sample_period = 1;
  config.gyro_noise = 1;
  config.accel_noise = 0;
  NativeJointError model(config);
  model.InitializeStationary(Initial(), Eigen::Vector3d(0, 0, 10), 4, true);
  ASSERT_TRUE(model.active()) << model.status().message;
  const auto state = model.Get("internal");
  const auto sample = model.Get("imu:last");
  EXPECT_NEAR((state * sample.transpose())(9, 3), 0.25, 1e-12);
  EXPECT_NEAR((state * state.transpose())(9, 9), 1.25, 1e-12);
  model.InitializeStationary(Initial(), Eigen::Vector3d(0, 0, 10), 4, false);
  ASSERT_TRUE(model.active()) << model.status().message;
  EXPECT_NEAR((model.Get("internal") *
               model.Get("imu:last").transpose())(9, 3), 0, 1e-12);
}

TEST(NativeJointErrorTest, WheelSubintervalsReuseOriginalInterpolatedSamples) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  config.native_imu_sample_period = 1;
  config.accel_noise = 1;
  config.gyro_noise = config.gyro_bias_noise = config.accel_bias_noise = 0;
  NativeJointError model(config);
  model.Initialize(Initial());
  ImuSample first, second;
  first.stamp = {1, 1, 1, "unix"};
  second.stamp = {2, 2, 2, "unix"};
  model.BeginImu(first, second);
  Eigen::Matrix<double, 15, 6> input = Eigen::Matrix<double, 15, 6>::Zero();
  input(3, 0) = 0.25;
  model.AdvanceImu(Matrix15d::Identity(), input, input, 1, 1.5);
  model.AdvanceImu(Matrix15d::Identity(), input, input, 1.5, 2);
  ASSERT_TRUE(model.active()) << model.status().message;
  const auto loading = model.Get("internal");
  EXPECT_NEAR((loading * loading.transpose())(3, 3), 1.5, 1e-12);
}

TEST(NativeJointErrorTest, DelayedBranchesShareDriversAndPinRetiredDependencies) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  config.native_imu_sample_period = 1;
  config.accel_noise = 1;
  config.gyro_noise = config.gyro_bias_noise = config.accel_bias_noise = 0;
  NativeJointError live(config);
  auto state = Initial();
  live.Initialize(state);
  live.Publish(state, {});
  auto lease = live.PinFork();
  NativeJointError candidate = live;
  candidate.Independent("scan", Eigen::Matrix3d::Identity(), "lidar:1:0");
  Eigen::MatrixXd internal = candidate.Get("internal");
  internal.middleRows(3, 3) += candidate.Get("scan");
  candidate.Set("internal", internal);
  ImuSample first, second;
  first.stamp = state.stamp;
  second.stamp = {2, 2, 2, "unix"};
  Eigen::Matrix<double, 15, 6> input = Eigen::Matrix<double, 15, 6>::Zero();
  input(3, 0) = 0.5;
  state.stamp = second.stamp;
  for (auto* branch : {&live, &candidate}) {
    branch->BeginImu(first, second);
    branch->AdvanceImu(Matrix15d::Identity(), input, input, 1, 2);
    CopyOutput(branch);
    branch->Retain({"internal", "output", "imu:last"});
    branch->Publish(state, {{"imu", 1}, {"imu", 2}});
    ASSERT_TRUE(branch->active()) << branch->status().message;
  }
  const auto connected = live.SpliceCandidate(candidate, lease, 1);
  ASSERT_TRUE(connected.ok()) << connected.message;
  const auto output = live.Get("output");
  const auto corrected = live.Get("internal");
  EXPECT_NEAR((output * output.transpose())(3, 3), 1.5, 1e-12);
  EXPECT_NEAR((corrected * corrected.transpose())(3, 3), 2.5, 1e-12);
  EXPECT_NEAR((corrected * output.transpose())(3, 3), 1.5, 1e-12);
  // Releasing the task retires only the fork pins, not the corrected physical
  // state or the original live publication.
  lease.reset();
  live.Retain({"internal", "output", "imu:last"});
  state.stamp = {3, 3, 3, "unix"};
  live.Publish(state, {{"imu", 2}, {"imu", 3}});
  MotionIncrement motion;
  const auto exported = live.MotionBetween(1, 3, &motion);
  ASSERT_TRUE(exported.ok()) << exported.message;
  EXPECT_NEAR(motion.cross_covariance(3, 3), 1, 1e-12);
}

}  // namespace
}  // namespace openloc::localization::unified
