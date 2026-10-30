// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/local/native_joint_error.h"
#include "candidates/local/bounded_joint_error.h"

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

TEST(NativeJointErrorTest, StructuredQueriesPreserveHiddenTemporalPointReuse) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  config.enable_native_structured_error = true;
  NativeJointError model(config);
  auto state = Initial();
  model.Initialize(state);
  model.Independent("point:a", Eigen::Matrix3d::Identity(), "lidar:1:0");
  model.Independent("point:b", Eigen::Matrix3d::Identity(), "lidar:1:1");
  const auto original = model.Error("point:a");
  Eigen::Matrix<double, 18, 15> internal =
      Eigen::Matrix<double, 18, 15>::Zero();
  internal.topRows(15).setIdentity();
  Eigen::Matrix<double, 18, 3> point =
      Eigen::Matrix<double, 18, 3>::Zero();
  point(0, 0) = 1;
  for (int step = 0; step < 3; ++step) {
    std::vector<NativeJointError::LinearTerm> terms{{internal, "internal"}};
    if (step != 1) terms.push_back({point, "point:a"});
    if (step != 0) terms.push_back({point, "point:b"});
    model.Linear("output", terms);
    state.stamp = {1.0 + step, 1.0 + step, uint64_t(1 + step), "unix"};
    model.Publish(state, step == 0
                             ? std::vector<SourceSampleId>{{"imu", 1}}
                             : std::vector<SourceSampleId>{
                                   {"imu", uint64_t(step)},
                                   {"imu", uint64_t(step + 1)}});
    ASSERT_TRUE(model.active()) << model.status().message;
  }
  MotionIncrement first, second, whole;
  ASSERT_TRUE(model.StructuredMotionBetween(1, 2, &first).ok());
  ASSERT_TRUE(model.StructuredMotionBetween(2, 3, &second).ok());
  ASSERT_TRUE(model.StructuredMotionBetween(1, 3, &whole).ok());
  EXPECT_NEAR(whole.cross_covariance(0, 0), 2, 1e-12);
  const Matrix15d false_markov =
      first.cross_covariance *
      first.end_covariance.llt().solve(second.cross_covariance);
  EXPECT_NEAR(false_markov(0, 0), 1, 1e-12);
  model.Retain({"internal", "output"});
  StructuredLocalState retained;
  ASSERT_TRUE(model.StructuredLookup(3, &retained).ok());
  Eigen::MatrixXd cross;
  ASSERT_TRUE(original.CrossCovariance(retained.output_error, &cross).ok());
  EXPECT_NEAR(cross(0, 0), 1, 1e-12);
  LocalState sentinel = Initial();
  EXPECT_EQ(model.Lookup(3, &sentinel).reason, Reason::CORRELATION_UNQUALIFIED);
  EXPECT_EQ(sentinel.stamp.time, 1);
}

TEST(NativeJointErrorTest, StructuredDelayedBranchesKeepOriginalSourceIdentity) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  config.enable_native_structured_error = true;
  config.native_imu_sample_period = 1;
  config.accel_noise = 1;
  config.gyro_noise = config.gyro_bias_noise = config.accel_bias_noise = 0;
  NativeJointError live(config);
  auto state = Initial();
  live.Initialize(state);
  live.Publish(state, {});
  const auto lease = live.PinFork();
  NativeJointError candidate = live;
  candidate.Independent("scan", Eigen::Matrix3d::Identity(), "lidar:1:0");
  const auto original = candidate.Error("scan");
  Eigen::Matrix<double, 15, 3> scan =
      Eigen::Matrix<double, 15, 3>::Zero();
  scan.middleRows(3, 3).setIdentity();
  candidate.Linear("internal",
                   {{Matrix15d::Identity(), "internal"}, {scan, "scan"}});
  ImuSample first, second;
  first.stamp = state.stamp;
  second.stamp = {2, 2, 2, "unix"};
  Eigen::Matrix<double, 15, 6> input = Eigen::Matrix<double, 15, 6>::Zero();
  input(3, 0) = 0.5;
  state.stamp = second.stamp;
  for (auto* branch : {&live, &candidate}) {
    branch->BeginImu(first, second);
    branch->AdvanceImu(Matrix15d::Identity(), input, input, 1, 2);
    branch->CopyInternalToOutput();
    branch->Retain({"internal", "output", "imu:last"});
    branch->Publish(state, {{"imu", 1}, {"imu", 2}});
    ASSERT_TRUE(branch->active()) << branch->status().message;
  }
  ASSERT_TRUE(live.SpliceCandidate(candidate, lease, 1).ok());
  Eigen::MatrixXd output, corrected, cross, original_cross;
  ASSERT_TRUE(live.Error("output").Covariance(&output).ok());
  ASSERT_TRUE(live.Error("internal").Covariance(&corrected).ok());
  ASSERT_TRUE(live.Error("internal")
                  .CrossCovariance(live.Error("output"), &cross).ok());
  ASSERT_TRUE(original.CrossCovariance(live.Error("internal"),
                                        &original_cross).ok());
  EXPECT_NEAR(output(3, 3), 1.5, 1e-12);
  EXPECT_NEAR(corrected(3, 3), 2.5, 1e-12);
  EXPECT_NEAR(cross(3, 3), 1.5, 1e-12);
  EXPECT_NEAR(original_cross(0, 3), 1, 1e-12);
}

TEST(NativeJointErrorTest, StructuredSourceExhaustionDoesNotEvictQuerySupport) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  config.enable_native_structured_error = true;
  config.max_history_states = 2;
  NativeJointError model(config);
  model.Initialize(Initial());
  model.Publish(Initial(), {});
  model.Independent("a", 2 * Eigen::Matrix3d::Identity(), "lidar:1:0");
  const auto original = model.Error("a");
  model.Independent("b", Eigen::Matrix3d::Identity(), "lidar:1:1");
  model.Independent("c", Eigen::Matrix3d::Identity(), "lidar:1:2");
  ASSERT_TRUE(model.active());
  model.Independent("d", Eigen::Matrix<double, 1, 1>::Identity(), "lidar:1:3");
  ASSERT_FALSE(model.active());
  EXPECT_NE(model.status().message.find("source capacity exhausted"),
            std::string::npos);
  Eigen::MatrixXd covariance;
  ASSERT_TRUE(original.Covariance(&covariance).ok());
  EXPECT_TRUE(covariance.isApprox(2 * Eigen::Matrix3d::Identity(), 1e-12));
  StructuredLocalState old;
  ASSERT_TRUE(model.StructuredLookup(1, &old).ok());
  EXPECT_TRUE(old.state.covariance.isApprox(Matrix15d::Identity(), 1e-12));
}

TEST(NativeJointErrorTest, ResourceObservationDistinguishesHistoryFromSourceLifetime) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  config.enable_native_structured_error = true;
  config.native_joint_max_snapshots = 2;
  NativeJointError model(config);
  auto state = Initial();
  model.Initialize(state);
  const auto original = model.Error("internal");
  for (int step = 0; step < 4; ++step) {
    model.Independent("temporary", Eigen::Matrix3d::Identity(),
                      "point:" + std::to_string(step));
    model.Drop("temporary");
    state.stamp.time = 1.0 + .01 * step;
    model.Publish(state, {});
    ASSERT_TRUE(model.active()) << model.status().message;
  }
  const auto usage = model.StructuredResourceUsage();
  EXPECT_EQ(usage.snapshots, 2);
  EXPECT_EQ(usage.primitive_sources, 5);
  EXPECT_EQ(usage.primitive_rows, 27);
  EXPECT_EQ(usage.memory_keys, 2);
  EXPECT_EQ(usage.publish_queries, 4);
  EXPECT_GE(usage.publish_query_ms, 0);
  Eigen::MatrixXd covariance;
  ASSERT_TRUE(original.Covariance(&covariance).ok());
  EXPECT_TRUE(covariance.isApprox(Matrix15d::Identity(), 1e-12));
  model.Reset();
  EXPECT_EQ(model.StructuredResourceUsage().primitive_sources, 0);
  EXPECT_EQ(model.StructuredResourceUsage().publish_queries, 0);
  ASSERT_TRUE(original.Covariance(&covariance).ok());
  EXPECT_TRUE(covariance.isApprox(Matrix15d::Identity(), 1e-12));
}

TEST(NativeJointErrorTest, RegistryReclaimsOnlySourcesWithNoExpressionOrForkHolder) {
  LocalConfig config;
  config.enable_native_joint_error = true;
  config.enable_native_structured_error = true;
  config.max_history_states = 2;
  NativeJointError model(config);
  model.Initialize(Initial());
  NativeJointError replay = model;
  model.Independent("pinned", 2 * Eigen::Matrix3d::Identity(), "pinned");
  NativeJointError fork = model;
  const auto pinned = fork.Error("pinned");
  model.Drop("pinned");
  for (int i = 0; i < 20; ++i) {
    model.Independent("unused", Eigen::Matrix3d::Identity(), std::to_string(i));
    ASSERT_TRUE(model.active()) << model.status().message;
    model.Drop("unused");
  }
  const auto usage = model.StructuredResourceUsage();
  EXPECT_LE(usage.primitive_rows, 24);
  EXPECT_GT(usage.retired_registry_sources, 0);
  Eigen::MatrixXd covariance;
  ASSERT_TRUE(pinned.Covariance(&covariance).ok());
  EXPECT_TRUE(covariance.isApprox(2 * Eigen::Matrix3d::Identity(), 1e-12));
  replay.Independent("same-source", 2 * Eigen::Matrix3d::Identity(), "pinned");
  ASSERT_TRUE(replay.active()) << replay.status().message;
  Eigen::MatrixXd cross;
  ASSERT_TRUE(pinned.CrossCovariance(replay.Error("same-source"), &cross).ok());
  EXPECT_TRUE(cross.isApprox(2 * Eigen::Matrix3d::Identity(), 1e-12));
}

TEST(NativeJointErrorTest, SelfCovarianceMatchesDistinctQueryWithSharedPrimitivePaths) {
  const auto identity = std::make_shared<const int>(0);
  Eigen::Matrix<double, 3, 2> root;
  root << 1, 2, -3, 4, .5, -.25;
  const auto first = JointErrorExpression::Primitive(identity, root);
  const auto reused = JointErrorExpression::Primitive(identity, 2 * root);
  const auto independent = JointErrorExpression::Primitive(
      std::make_shared<const int>(0), Eigen::Matrix2d::Identity());
  Eigen::Matrix<double, 2, 3> a, b;
  a << 1, 2, 0, -.5, .25, 2;
  b << -1, 0, 3, 0, 2, -.25;
  const auto query = JointErrorExpression::Combine(
      2, {{a, first}, {b, reused}, {3 * Eigen::Matrix2d::Identity(), independent}});
  const auto distinct = JointErrorExpression::Combine(
      2, {{Eigen::Matrix2d::Identity(), query}});
  const Eigen::Matrix2d loading = (a + 2 * b) * root;
  const Eigen::Matrix2d expected =
      loading * loading.transpose() + 9 * Eigen::Matrix2d::Identity();
  Eigen::MatrixXd self, cross;
  ASSERT_TRUE(query.Covariance(&self).ok());
  ASSERT_TRUE(query.CrossCovariance(distinct, &cross).ok());
  EXPECT_TRUE(self.isApprox(expected, 1e-12));
  EXPECT_TRUE(cross.isApprox(expected, 1e-12));
}

TEST(NativeJointErrorTest, ExactZeroDerivativeDoesNotRetainAnUnusedPrimitive) {
  auto identity = std::make_shared<const int>(0);
  const std::weak_ptr<const int> unused = identity;
  auto primitive = JointErrorExpression::Primitive(
      identity, Eigen::Matrix3d::Identity());
  const auto query = JointErrorExpression::Combine(
      3, {{Eigen::Matrix3d::Zero(), primitive}});
  primitive = {};
  identity.reset();
  EXPECT_TRUE(unused.expired());
  Eigen::MatrixXd covariance;
  ASSERT_TRUE(query.Covariance(&covariance).ok());
  EXPECT_TRUE(covariance.isZero(0));
}

TEST(NativeJointErrorTest, ClosedFrontierPreservesReuseForksAndRetiredQueries) {
  BoundedJointError frontier(30);
  std::map<std::string, Eigen::MatrixXd> oracle;
  Eigen::Index dimension = 0;
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  const auto independent = [&](const std::string& key, double scale) {
    ASSERT_TRUE(frontier.Independent(key, scale * identity).ok());
    for (auto& entry : oracle) {
      entry.second.conservativeResize(Eigen::NoChange, dimension + 3);
      entry.second.rightCols(3).setZero();
    }
    Eigen::MatrixXd next = Eigen::MatrixXd::Zero(3, dimension + 3);
    next.rightCols(3) = scale * identity;
    dimension += 3;
    oracle[key] = std::move(next);
  };
  const auto linear = [&](const std::string& key,
                          const std::vector<BoundedJointError::Term>& terms) {
    ASSERT_TRUE(frontier.Linear(key, terms).ok());
    Eigen::MatrixXd next = Eigen::MatrixXd::Zero(3, dimension);
    for (const auto& term : terms)
      next.noalias() += term.coefficient * oracle.at(term.key);
    oracle[key] = std::move(next);
  };
  independent("state", 1);
  independent("point", 0.2);
  linear("held", {{identity, "state"}});
  for (int step = 0; step < 1000; ++step) {
    SCOPED_TRACE(step);
    independent("noise", 0.01);
    linear("state", {{0.99 * identity, "state"},
                     {0.02 * identity, "point"}, {identity, "noise"}});
    // Both branches reuse exactly the same sampled driver, not independent
    // reinsertions. Pins also stand for previously published query handles.
    if (step % 10 == 0 || step % 10 == 6)
      linear("fork", {{identity, "state"}});
    if (oracle.count("fork"))
      linear("fork", {{identity, "fork"}, {0.5 * identity, "noise"}});
    if (step % 10 == 4) {
      ASSERT_EQ(oracle.count("fork"), 1);
      linear("state", {{0.7 * identity, "state"}, {0.3 * identity, "fork"}});
      oracle.erase("fork");
    }
    if (step % 10 == 8) {
      ASSERT_EQ(oracle.count("fork"), 1);
      oracle.erase("fork");  // discarded task
    }
    oracle.erase("noise");
    if (step % 7 == 6) {
      oracle.erase("point");
      std::vector<std::string> keep;
      for (const auto& entry : oracle) keep.push_back(entry.first);
      ASSERT_TRUE(frontier.Retain(keep).ok());
      ASSERT_TRUE(frontier.Compact().ok());
      independent("point", 0.2);
    }
    std::vector<std::string> keep;
    for (const auto& entry : oracle) keep.push_back(entry.first);
    ASSERT_TRUE(frontier.Retain(keep).ok());
    ASSERT_TRUE(frontier.Compact().ok());
    EXPECT_LE(frontier.rows(), 12);
    EXPECT_LE(frontier.dimension(), 12);
    for (const auto& a : oracle)
      for (const auto& b : oracle) {
        Eigen::MatrixXd covariance;
        ASSERT_TRUE(frontier.CrossCovariance(a.first, b.first, &covariance).ok());
        const Eigen::MatrixXd expected = a.second * b.second.transpose();
        EXPECT_LE((covariance - expected).norm(),
                  1e-10 * std::max(1.0, expected.norm()));
      }
  }
}

TEST(NativeJointErrorTest, ClosedFrontierKeepsWeakSingularDirectionsAndFailsClosed) {
  BoundedJointError frontier(9);
  Eigen::Matrix3d root = Eigen::Matrix3d::Zero();
  root(0, 0) = 1;
  root(1, 1) = 1e-12;
  ASSERT_TRUE(frontier.Independent("state", root).ok());
  ASSERT_TRUE(frontier.Independent("retired", Eigen::Matrix3d::Identity()).ok());
  ASSERT_TRUE(frontier.Retain({"state"}).ok());
  ASSERT_TRUE(frontier.Compact().ok());
  Eigen::MatrixXd covariance;
  ASSERT_TRUE(frontier.CrossCovariance("state", "state", &covariance).ok());
  EXPECT_DOUBLE_EQ(covariance(0, 0), 1);
  EXPECT_NEAR(covariance(1, 1), 1e-24, 1e-38);
  EXPECT_DOUBLE_EQ(covariance(2, 2), 0);
  EXPECT_FALSE(frontier.Independent("too_large", Eigen::MatrixXd::Identity(7, 7)).ok());
  EXPECT_FALSE(frontier.Retain({"missing"}).ok());
  EXPECT_FALSE(frontier.Linear("state", {{root, "retired"}}).ok());
  Eigen::MatrixXd unchanged;
  ASSERT_TRUE(frontier.CrossCovariance("state", "state", &unchanged).ok());
  EXPECT_TRUE(unchanged.isApprox(covariance, 0));
}

}  // namespace
}  // namespace openloc::localization::unified
