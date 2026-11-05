// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "candidates/global/fixed_lag_graph.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "Eigen/Cholesky"
#include "gtest/gtest.h"
#include "gtsam/linear/GaussianConditional.h"

#include "candidates/global/global_alignment.h"

namespace openloc {
namespace localization {
namespace unified {
namespace {

LocalState Local(uint64_t index) {
  LocalState state;
  state.epoch = {"graph-local", 1};
  state.stamp = {10.0 + 0.1 * static_cast<double>(index),
                 10.0 + 0.1 * static_cast<double>(index), index + 1, "unix"};
  state.covariance = Matrix15d::Identity() * (0.1 + 0.001 * index);
  state.uncertainty_qualified = true;
  state.valid = true;
  return state;
}

GlobalObservation Observation(const LocalState& local, uint64_t sequence) {
  GlobalObservation observation;
  observation.stamp = local.stamp;
  observation.stamp.sequence = sequence;
  observation.epoch = local.epoch;
  observation.source = "independent-survey";
  observation.map_id = "site";
  observation.map_version = "v1";
  observation.calibration_id = "calibrated";
  observation.pose.translation().x() = 100.0;
  observation.covariance = Matrix6d::Identity() * 0.01;
  observation.quality_valid = true;
  observation.uncertainty_qualified = true;
  observation.independent_of_local = true;
  return observation;
}

FixedLagConfig Policy() {
  FixedLagConfig config;
  config.lag = 0.35;
  // Numerical tests do not qualify a production execution deadline.
  config.solve_budget_ms = 100000.0;
  return config;
}

MotionIncrement Motion(const LocalState& start, const LocalState& end) {
  MotionIncrement motion;
  const auto result = ComputeMotionIncrement(
      start, end, start.covariance,
      {{"imu", start.stamp.sequence}, {"imu", end.stamp.sequence}}, &motion);
  EXPECT_TRUE(result.ok()) << result.message;
  return motion;
}

TEST(FixedLagGraphTest, CroppedConditionalUsesSurvivingNoiseRowsIncludingExactZeros) {
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  const std::vector<std::pair<gtsam::Key, gtsam::Matrix>> terms{
      {1, identity.leftCols(1)}, {2, identity.rightCols(2)}};
  Eigen::Vector3d original;
  original << 7, 0, 2;
  gtsam::GaussianConditional conditional(
      terms, 2, Eigen::Vector3d::Zero(),
      gtsam::noiseModel::Constrained::MixedSigmas(original));
  Eigen::VectorXd sigmas;
  ASSERT_TRUE(ConditionalRowSigmas(conditional, &sigmas).ok());
  EXPECT_TRUE(sigmas.isApprox(original));
  conditional.matrixObject().firstBlock() += 1;
  conditional.matrixObject().rowStart() = 1;
  conditional.keys().erase(conditional.keys().begin());
  --conditional.nrFrontals();
  ASSERT_EQ(conditional.R().rows(), 2);
  ASSERT_EQ(conditional.get_model()->sigmas().size(), 3);
  ASSERT_TRUE(ConditionalRowSigmas(conditional, &sigmas).ok());
  ASSERT_EQ(sigmas.size(), 2);
  EXPECT_EQ(sigmas[0], 0);
  EXPECT_EQ(sigmas[1], 2);
  const Eigen::Matrix2d solved = conditional.R()
      .transpose().triangularView<Eigen::Lower>().solve(Eigen::Matrix2d::Identity());
  const Eigen::Matrix2d weighted = sigmas.asDiagonal() * solved;
  Eigen::Matrix2d expected = Eigen::Matrix2d::Zero();
  expected(1, 1) = 4;
  EXPECT_TRUE((weighted.transpose() * weighted).isApprox(expected, 1e-12));
}

TEST(FixedLagGraphTest, BatchReferenceMatchesLinearKnownAnswerAcrossLagAndExactNoise) {
  for (const double process_variance : {0.0, 0.001}) {
    FixedLagGraph graph(Policy());
    GraphEvaluationSnapshot missing;
    EXPECT_EQ(Reason::INVALID_INPUT,
              graph.EvaluateForTesting(true, &missing).reason);
    ASSERT_TRUE(graph.EnableEvaluationHistory().ok());
    auto previous = Local(0);
    ASSERT_TRUE(graph.Initialize(previous, Observation(previous, 1)).ok());
    double mean = 100.0;
    double variance = 0.01;
    for (uint64_t index = 1; index <= 8; ++index) {
      auto local = Local(index);
      local.covariance = Matrix15d::Identity() * (0.1 + process_variance * index);
      const auto added = graph.AddLocal(local, Motion(previous, local));
      ASSERT_TRUE(added.ok()) << added.message;
      auto fix = Observation(local, index + 1);
      fix.pose.translation().x() += 0.02;
      ASSERT_TRUE(graph.Observe(fix).ok());
      variance += process_variance;
      const double gain = variance / (variance + 0.01);
      mean += gain * (100.02 - mean);
      variance *= 1 - gain;
      GraphEvaluationSnapshot lag, batch;
      ASSERT_TRUE(graph.EvaluateForTesting(false, &lag).ok());
      const auto solved = graph.EvaluateForTesting(true, &batch);
      ASSERT_TRUE(solved.ok()) << solved.message;
      EXPECT_NEAR(mean, lag.map_from_base.translation().x(), 1e-7);
      EXPECT_NEAR(mean, batch.map_from_base.translation().x(), 1e-7);
      EXPECT_NEAR(variance, lag.pose_covariance(0, 0), 1e-7);
      EXPECT_NEAR(variance, batch.pose_covariance(0, 0), 1e-7);
      EXPECT_LT(batch.maximum_exact_residual, 1e-6);
      EXPECT_EQ(2 + 3 * index, batch.factors.size());
      EXPECT_EQ(index + 1, batch.archived_nodes);
      EXPECT_EQ(graph.state().correction_id, 1 + 2 * index);
      previous = local;
    }
    GraphEvaluationSnapshot final;
    ASSERT_TRUE(graph.EvaluateForTesting(true, &final).ok());
    EXPECT_LT(final.retained_nodes, final.archived_nodes);
    EXPECT_EQ(Reason::INVALID_INPUT, graph.EnableEvaluationHistory().reason);
  }
}

LocalState JointLocal(uint64_t index) {
  auto state = Local(index);
  const Eigen::Index dimension = index >= 3 && index <= 5 ? 17 : 16;
  JointLocalError model;
  model.covariance = Eigen::MatrixXd::Identity(dimension, dimension) * 0.1;
  model.covariance(15, 15) = 0.025;
  model.output_jacobian = Eigen::MatrixXd::Zero(15, dimension);
  model.output_jacobian.leftCols<15>().setIdentity();
  model.output_jacobian(0, 15) = 0.2 * index;
  state.covariance = model.output_jacobian * model.covariance *
                     model.output_jacobian.transpose();
  state.joint_error = std::move(model);
  return state;
}

MotionIncrement JointMotion(const LocalState& start, const LocalState& end) {
  const auto first = JointErrorCovariance(start);
  const auto second = JointErrorCovariance(end);
  Eigen::MatrixXd cross = Eigen::MatrixXd::Zero(first.rows(), second.rows());
  const auto shared = std::min(first.rows(), second.rows());
  cross.topLeftCorner(shared, shared) = first.topLeftCorner(shared, shared);
  const Matrix15d projected =
      JointOutputJacobian(start) * cross * JointOutputJacobian(end).transpose();
  MotionIncrement motion;
  const auto result = ComputeMotionIncrement(
      start, end, projected,
      {{"imu", start.stamp.sequence}, {"imu", end.stamp.sequence}}, &motion);
  EXPECT_TRUE(result.ok()) << result.message;
  motion.joint_cross_covariance = cross;
  return motion;
}

LocalState HiddenBasisLocal(uint64_t index, Eigen::Index dimension) {
  auto state = Local(index);
  JointLocalError model;
  model.covariance = Eigen::MatrixXd::Identity(dimension, dimension) * 0.1;
  model.output_jacobian = Eigen::MatrixXd::Zero(15, dimension);
  model.output_jacobian.leftCols<15>().setIdentity();
  state.covariance = model.output_jacobian * model.covariance *
                     model.output_jacobian.transpose();
  state.joint_error = std::move(model);
  return state;
}

TEST(FixedLagGraphTest, TurningTrajectoryRetainsNonlinearConstraintsAcrossLag) {
  for (const bool shared : {false, true}) {
    FixedLagGraph graph(Policy());
    ASSERT_TRUE(graph.EnableEvaluationHistory().ok());
    LocalState previous;
    for (uint64_t index = 0; index <= 8; ++index) {
      auto local = shared ? JointLocal(index) : Local(index);
      const double step = static_cast<double>(index);
      local.position = {2 * step, std::sin(step), 0.1 * step};
      local.orientation = ExpRotation({0.02 * step, -0.01 * step, 0.15 * step});
      if (index == 0) {
        auto anchor = Observation(local, 1);
        anchor.pose.linear() = local.orientation.toRotationMatrix();
        ASSERT_TRUE(graph.Initialize(local, anchor).ok());
      } else {
        const auto added = graph.AddLocal(
            local, shared ? JointMotion(previous, local) : Motion(previous, local));
        ASSERT_TRUE(added.ok()) << added.message;
        auto fix = Observation(local, index + 1);
        fix.pose.translation() += local.position +
            Eigen::Vector3d(0.02 * std::cos(step), -0.01 * std::sin(step), 0.005);
        fix.pose.linear() =
            ExpRotation({0.02 * std::sin(step), 0.04, -0.03})
                .toRotationMatrix() * local.orientation.toRotationMatrix();
        if (shared) {
          fix.independent_of_local = false;
          fix.conditional_local_error_qualified = true;
          fix.local_error_sequence = local.stamp.sequence;
          fix.local_error_cross_covariance =
              Eigen::MatrixXd::Zero(local.joint_error->covariance.rows(), 6);
          fix.local_error_cross_covariance(15, 0) = 0.005;
        }
        const auto observed = graph.Observe(fix);
        ASSERT_TRUE(observed.ok()) << observed.message;
        GraphEvaluationSnapshot lag, batch;
        ASSERT_TRUE(graph.EvaluateForTesting(false, &lag).ok());
        ASSERT_TRUE(graph.EvaluateForTesting(true, &batch).ok());
        EXPECT_LE(lag.maximum_exact_residual, 1e-6);
        EXPECT_LE(batch.maximum_exact_residual, 1e-6);
        EXPECT_TRUE(lag.pose_covariance.llt().info() == Eigen::Success);
        EXPECT_LT((lag.map_from_base.translation() -
                   batch.map_from_base.translation()).norm(), 0.002);
        if (index >= 4) {
          EXPECT_LT(lag.retained_nodes, lag.archived_nodes);
        }
        if (index == 8) {
          EXPECT_GT(lag.retained_relinearizations, 0);
        }
      }
      previous = local;
    }
  }
}

TEST(FixedLagGraphTest, JointMotionRejectsIndefiniteConditionalNoise) {
  const auto start = HiddenBasisLocal(0, 17);
  const auto end = HiddenBasisLocal(1, 17);
  MotionIncrement motion;
  ASSERT_TRUE(ComputeMotionIncrement(
      start, end, Matrix15d::Identity() * 0.1,
      {{"imu", start.stamp.sequence}, {"imu", end.stamp.sequence}},
      &motion).ok());
  motion.joint_cross_covariance =
      Eigen::MatrixXd::Identity(17, 17) * 0.1;
  EXPECT_TRUE(ValidateJointMotion(start, end, motion).ok());

  motion.joint_cross_covariance(16, 16) += 1e-12;
  EXPECT_EQ(Reason::INVALID_INPUT,
            ValidateJointMotion(start, end, motion).reason);

  motion.joint_cross_covariance =
      Eigen::MatrixXd::Identity(17, 17) * 0.1;
  Eigen::Matrix2d hidden_cross;
  hidden_cross << 1, -1, -1, 1;
  motion.joint_cross_covariance.bottomRightCorner<2, 2>() =
      0.1 / std::sqrt(2.0) * hidden_cross;
  EXPECT_EQ(Reason::INVALID_INPUT,
            ValidateJointMotion(start, end, motion).reason);
}

TEST(FixedLagGraphTest, HiddenSharedMemorySurvivesDimensionChangesAndLag) {
  FixedLagGraph graph(Policy());
  ASSERT_TRUE(graph.EnableEvaluationHistory().ok());
  auto previous = JointLocal(0);
  ASSERT_TRUE(graph.Initialize(previous, Observation(previous, 1)).ok());
  Eigen::Matrix2d information = Eigen::Matrix2d::Zero();
  information.diagonal() << 100.0, 40.0;
  Eigen::Vector2d right = Eigen::Vector2d::Zero();
  for (uint64_t index = 1; index <= 8; ++index) {
    const auto local = JointLocal(index);
    const auto added = graph.AddLocal(local, JointMotion(previous, local));
    ASSERT_TRUE(added.ok()) << added.message;
    if (index == 1 || index == 4) {
      const auto measured = JointLocal(index == 1 ? 1 : 2);
      auto fix = Observation(measured, 1);
      fix.source = index == 1 ? "shared-memory-a" : "shared-memory-b";
      fix.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
      fix.pose.translation().x() += 0.05;
      fix.covariance.topLeftCorner<2, 2>() =
          Eigen::Matrix2d::Identity() * 0.05;
      fix.independent_of_local = false;
      fix.conditional_local_error_qualified = true;
      fix.local_error_sequence = measured.stamp.sequence;
      fix.local_error_cross_covariance =
          Eigen::MatrixXd::Zero(JointErrorCovariance(measured).rows(), 2);
      fix.local_error_cross_covariance(15, 0) = 0.025;
      const auto observed = graph.Observe(fix);
      ASSERT_TRUE(observed.ok()) << observed.message;
      // z = x_anchor + (output loading + source loading) * memory + eta.
      const Eigen::Vector2d row(
          1.0, JointOutputJacobian(measured)(0, 15) + 1.0);
      information += row * row.transpose() / 0.025;
      right += row * 0.05 / 0.025;
    }
    const Eigen::Vector2d posterior = information.ldlt().solve(right);
    const double expected = 100 + posterior.x() + 0.2 * index * posterior.y();
    EXPECT_NEAR(graph.state().map_to_odom.translation().x(), expected, 1e-7);
    GraphEvaluationSnapshot batch;
    const auto solved = graph.EvaluateForTesting(true, &batch);
    ASSERT_TRUE(solved.ok()) << solved.message;
    EXPECT_NEAR(batch.map_from_base.translation().x(), expected, 1e-7);
    EXPECT_LT(batch.maximum_exact_residual, 1e-6);
    previous = local;
  }
}

TEST(FixedLagGraphTest, UnobservedAxisUncertaintyGrowsAcrossLag) {
  FixedLagGraph graph(Policy());
  ASSERT_TRUE(graph.EnableEvaluationHistory().ok());
  auto previous = Local(0);
  ASSERT_TRUE(graph.Initialize(previous, Observation(previous, 1)).ok());
  double expected_x = 0.01;
  for (uint64_t index = 1; index <= 8; ++index) {
    const auto local = Local(index);
    ASSERT_TRUE(graph.AddLocal(local, Motion(previous, local)).ok());
    auto fix = Observation(local, index + 1);
    fix.kind = GlobalObservation::Kind::PROJECTED_POSE;
    fix.projection = Eigen::MatrixXd::Zero(1, 6);
    fix.projection(0, 1) = 1;
    fix.projected_covariance = Eigen::MatrixXd::Constant(1, 1, 0.01);
    ASSERT_TRUE(graph.Observe(fix).ok());
    expected_x += 0.001;
    GraphEvaluationSnapshot lag, batch;
    ASSERT_TRUE(graph.EvaluateForTesting(false, &lag).ok());
    ASSERT_TRUE(graph.EvaluateForTesting(true, &batch).ok());
    EXPECT_NEAR(expected_x, lag.pose_covariance(0, 0), 1e-7);
    EXPECT_NEAR(expected_x, batch.pose_covariance(0, 0), 1e-7);
    EXPECT_LT(lag.pose_covariance(1, 1), lag.pose_covariance(0, 0));
    previous = local;
  }
}

TEST(FixedLagGraphTest, RejectsInvalidHiddenCovarianceWithoutConsumingNode) {
  auto policy = Policy();
  policy.max_error_dimension = 16;
  FixedLagGraph graph(policy);
  auto previous = JointLocal(0);
  auto malformed = previous;
  malformed.joint_error->output_jacobian(0, 15) = 1;
  EXPECT_EQ(graph.Initialize(malformed, Observation(malformed, 1)).reason,
            Reason::INVALID_INPUT);
  ASSERT_TRUE(graph.Initialize(previous, Observation(previous, 1)).ok());
  const auto local = JointLocal(1);
  auto motion = JointMotion(previous, local);
  const auto revision = graph.state().correction_id;
  motion.joint_cross_covariance(15, 15) *= 2;
  // Published marginals/cross-covariance still match, but the hidden block
  // implies negative conditional covariance and must not be discarded.
  motion.cross_covariance =
      JointOutputJacobian(previous) * motion.joint_cross_covariance *
      JointOutputJacobian(local).transpose();
  EXPECT_EQ(graph.AddLocal(local, motion).reason, Reason::INVALID_INPUT);
  EXPECT_EQ(graph.state().correction_id, revision);
  ASSERT_TRUE(graph.AddLocal(local, JointMotion(previous, local)).ok());
  auto over_capacity = JointLocal(3);
  EXPECT_EQ(graph.AddLocal(over_capacity, JointMotion(local, over_capacity)).reason,
            Reason::INVALID_INPUT);
}

TEST(FixedLagGraphTest, PositionOrCorrelatedEvidenceCannotCreateAnAnchor) {
  FixedLagGraph graph(Policy());
  auto observation = Observation(Local(0), 1);
  observation.kind = GlobalObservation::Kind::POSITION;
  EXPECT_EQ(Reason::GLOBAL_UNAVAILABLE,
            graph.Initialize(Local(0), observation).reason);
  EXPECT_FALSE(graph.initialized());
  observation.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
  EXPECT_EQ(Reason::GLOBAL_UNAVAILABLE,
            graph.Initialize(Local(0), observation).reason);
  EXPECT_FALSE(graph.initialized());
  observation.kind = GlobalObservation::Kind::POSE;
  observation.independent_of_local = false;
  EXPECT_EQ(Reason::INVALID_INPUT,
            graph.Initialize(Local(0), observation).reason);
  EXPECT_FALSE(graph.state().valid);
}

TEST(FixedLagGraphTest, HorizontalEvidencePreservesTheUnobservedSubspace) {
  FixedLagGraph graph(Policy());
  ASSERT_TRUE(graph.Initialize(Local(0), Observation(Local(0), 1)).ok());
  const auto before = graph.state();
  auto fix = Observation(Local(0), 1);
  fix.source = "independent-gnss-xy";
  fix.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
  fix.pose.translation().head<2>() = Eigen::Vector2d(100.03, 0.04);
  // Only XY and its 2x2 covariance are observations, not these unused fields.
  fix.pose.translation().z() = std::numeric_limits<double>::quiet_NaN();
  fix.pose.linear().setConstant(std::numeric_limits<double>::quiet_NaN());
  fix.covariance.setConstant(std::numeric_limits<double>::quiet_NaN());
  fix.covariance.topLeftCorner<2, 2>() << 0.02, 0.005, 0.005, 0.01;
  const auto observed = graph.Observe(fix);
  ASSERT_TRUE(observed.ok()) << observed.message;
  const auto& after = graph.state();
  EXPECT_GT(after.map_to_odom.translation().x(), 100.0);
  EXPECT_GT(after.map_to_odom.translation().y(), 0.0);
  EXPECT_NEAR(before.map_to_odom.translation().z(),
              after.map_to_odom.translation().z(), 1e-8);
  EXPECT_TRUE(
      before.map_to_odom.linear().isApprox(after.map_to_odom.linear(), 1e-8));
  EXPECT_NEAR(before.covariance(2, 2), after.covariance(2, 2), 1e-8);
  EXPECT_TRUE((before.covariance.bottomRightCorner<3, 3>().isApprox(
      after.covariance.bottomRightCorner<3, 3>(), 1e-8)));
  EXPECT_GT(std::abs(after.covariance(0, 1)), 1e-8);
  EXPECT_DOUBLE_EQ(before.last_full_observation, after.last_full_observation);
  EXPECT_EQ(before.correction_id + 1, after.correction_id);
}

TEST(FixedLagGraphTest, DelayedHorizontalFactorUsesItsHistoricalAntennaPose) {
  FixedLagGraph graph(Policy());
  auto first = Local(0);
  first.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  auto anchor = Observation(first, 1);
  anchor.pose = Pose(first);
  anchor.pose.translation().x() += 100.0;
  ASSERT_TRUE(graph.Initialize(first, anchor).ok());
  auto previous = first;
  for (uint64_t index = 1; index <= 3; ++index) {
    auto local = Local(index);
    local.position = first.position + Eigen::Vector3d(index, 0.0, 0.0);
    local.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(0.1 * index, Eigen::Vector3d::UnitZ()));
    const auto motion = Motion(previous, local);
    ASSERT_TRUE(graph.AddLocal(local, motion).ok());
    previous = local;
  }
  auto historical = Local(1);
  historical.position = first.position + Eigen::Vector3d(1.0, 0.0, 0.0);
  historical.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));
  auto fix = Observation(historical, 1);
  fix.source = "independent-delayed-xy";
  fix.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
  fix.point_in_base = Eigen::Vector3d(1.2, -0.3, 0.4);
  const Eigen::Vector3d measured =
      Eigen::Vector3d(100.0, 0.0, 0.0) + Pose(historical) * fix.point_in_base;
  fix.pose.translation().head<2>() = measured.head<2>();
  fix.stamp.receive_time = previous.stamp.time;
  const auto before = graph.state();
  const auto observed = graph.Observe(fix);
  ASSERT_TRUE(observed.ok()) << observed.message;
  EXPECT_TRUE(before.map_to_odom.matrix().isApprox(
      graph.state().map_to_odom.matrix(), 1e-6));
  EXPECT_DOUBLE_EQ(first.stamp.time, graph.state().last_full_observation);
  EXPECT_DOUBLE_EQ(historical.stamp.time, graph.state().last_observation);
  EXPECT_DOUBLE_EQ(previous.stamp.time, graph.state().covariance_time);
  EXPECT_EQ(Reason::DUPLICATE, graph.Observe(fix).reason);
}

TEST(FixedLagGraphTest, InvalidHorizontalNoiseDoesNotConsumeTheObservation) {
  FixedLagGraph graph(Policy());
  ASSERT_TRUE(graph.Initialize(Local(0), Observation(Local(0), 1)).ok());
  auto fix = Observation(Local(0), 1);
  fix.source = "independent-xy";
  fix.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
  fix.covariance.topLeftCorner<2, 2>() << 1.0, 2.0, 2.0, 1.0;
  const auto before = graph.state();
  EXPECT_EQ(Reason::INVALID_INPUT, graph.Observe(fix).reason);
  EXPECT_EQ(before.correction_id, graph.state().correction_id);
  EXPECT_TRUE(before.covariance.isApprox(graph.state().covariance, 0.0));
  fix.covariance.topLeftCorner<2, 2>().setIdentity();
  fix.independent_of_local = false;
  EXPECT_EQ(Reason::INVALID_INPUT, graph.Observe(fix).reason);
  fix.independent_of_local = true;
  EXPECT_TRUE(graph.Observe(fix).ok());
}

TEST(FixedLagGraphTest, UnmodeledLocalCovarianceCannotAnchorOrAdvanceGraph) {
  FixedLagGraph graph(Policy());
  auto local = Local(0);
  local.covariance_model_valid = false;
  EXPECT_EQ(Reason::GLOBAL_UNAVAILABLE,
            graph.Initialize(local, Observation(local, 1)).reason);
  EXPECT_FALSE(graph.initialized());
  ASSERT_TRUE(graph.Initialize(Local(0), Observation(Local(0), 1)).ok());
  const auto previous = graph.state();
  const auto motion = Motion(Local(0), Local(1));
  local = Local(1);
  local.covariance_model_valid = false;
  EXPECT_EQ(Reason::CORRELATION_UNQUALIFIED, graph.AddLocal(local, motion).reason);
  EXPECT_DOUBLE_EQ(previous.covariance_time, graph.state().covariance_time);
  EXPECT_TRUE(previous.map_to_odom.matrix().isApprox(
      graph.state().map_to_odom.matrix()));
}

TEST(FixedLagGraphTest,
     DelayedPositionUsesItsOwnStateWithoutRefreshingFullLease) {
  FixedLagGraph graph(Policy());
  ASSERT_TRUE(graph.Initialize(Local(0), Observation(Local(0), 1)).ok());
  for (uint64_t index = 1; index <= 3; ++index) {
    const auto motion = Motion(Local(index - 1), Local(index));
    const auto result = graph.AddLocal(Local(index), motion);
    ASSERT_TRUE(result.ok()) << result.message;
  }
  const double full_time = graph.state().last_full_observation;
  auto latest = Observation(Local(3), 3);
  latest.kind = GlobalObservation::Kind::POSITION;
  ASSERT_TRUE(graph.Observe(latest).ok());
  auto delayed = Observation(Local(1), 2);
  delayed.kind = GlobalObservation::Kind::POSITION;
  delayed.pose.translation().y() = 0.05;
  const auto observed = graph.Observe(delayed);
  ASSERT_TRUE(observed.ok()) << observed.message;
  EXPECT_GT(graph.state().map_to_odom.translation().y(), 0.0);
  EXPECT_DOUBLE_EQ(full_time, graph.state().last_full_observation);
  EXPECT_DOUBLE_EQ(Local(3).stamp.time, graph.state().last_observation);
  EXPECT_DOUBLE_EQ(Local(3).stamp.time, graph.state().covariance_time);
  EXPECT_EQ(Reason::DUPLICATE, graph.Observe(delayed).reason);
  delayed.stamp.sequence = 4;
  delayed.stamp.time += 0.01;
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE, graph.Observe(delayed).reason);
}

TEST(FixedLagGraphTest, MarginalizationRetainsAnchorAndRejectsOldTimes) {
  FixedLagGraph graph(Policy());
  ASSERT_TRUE(graph.Initialize(Local(0), Observation(Local(0), 1)).ok());
  for (uint64_t index = 1; index <= 8; ++index) {
    const auto motion = Motion(Local(index - 1), Local(index));
    const auto added = graph.AddLocal(Local(index), motion);
    ASSERT_TRUE(added.ok()) << added.message;
  }
  EXPECT_NEAR(100.0, graph.state().map_to_odom.translation().x(), 1e-8);
  EXPECT_TRUE(graph.state().valid);
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            graph.Observe(Observation(Local(0), 2)).reason);
  auto fresh = Observation(Local(8), 3);
  ASSERT_TRUE(graph.Observe(fresh).ok());
  EXPECT_DOUBLE_EQ(Local(8).stamp.time, graph.state().last_full_observation);
}

TEST(FixedLagGraphTest, DeterministicErrorChainSurvivesFixedLagMarginalization) {
  FixedLagGraph graph(Policy());
  const auto first = Local(0);
  ASSERT_TRUE(graph.Initialize(first, Observation(first, 1)).ok());
  const auto before = graph.state();
  auto previous = first;
  for (uint64_t index = 1; index <= 8; ++index) {
    auto local = Local(index);
    local.covariance = first.covariance;
    const auto result = graph.AddLocal(local, Motion(previous, local));
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_NEAR(graph.state().map_to_odom.translation().x(), 100.0, 1e-8);
    EXPECT_TRUE(graph.state().covariance.isApprox(before.covariance, 1e-8));
    previous = local;
  }
  auto fix = Observation(previous, 1);
  fix.source = "independent-latest-xy";
  fix.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
  fix.pose.translation().y() = 0.02;
  ASSERT_TRUE(graph.Observe(fix).ok());
  EXPECT_GT(graph.state().map_to_odom.translation().y(), 0.0);
  EXPECT_DOUBLE_EQ(graph.state().last_full_observation, first.stamp.time);
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            graph.Observe(Observation(first, 2)).reason);
}

TEST(FixedLagGraphTest, RankDeficientConditionalRetainsRotatedNoiseSubspace) {
  FixedLagGraph graph(Policy());
  auto previous = Local(0);
  previous.covariance = Matrix15d::Identity() * 0.125;
  ASSERT_TRUE(graph.Initialize(previous, Observation(previous, 1)).ok());
  for (uint64_t index = 1; index <= 8; ++index) {
    auto local = Local(index);
    Eigen::Matrix<double, 15, 1> direction =
        Eigen::Matrix<double, 15, 1>::Zero();
    direction[0] = 0.125;
    direction[1] = 0.125;
    local.covariance = previous.covariance + direction * direction.transpose();
    const auto added = graph.AddLocal(local, Motion(previous, local));
    ASSERT_TRUE(added.ok()) << added.message;
    previous = local;
  }
  const auto before = graph.state();
  auto fix = Observation(previous, 1);
  fix.source = "independent-xy";
  fix.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
  fix.pose.translation().head<2>() += Eigen::Vector2d(0.02, 0.02);
  ASSERT_TRUE(graph.Observe(fix).ok());
  const auto shift = graph.state().map_to_odom.translation() -
                     before.map_to_odom.translation();
  EXPECT_GT(shift.x(), 0);
  EXPECT_NEAR(shift.x(), shift.y(), 1e-8);
  EXPECT_NEAR(shift.z(), 0, 1e-8);
  EXPECT_TRUE(graph.state().map_to_odom.linear().isIdentity(1e-8));
}

TEST(FixedLagGraphTest, IndefiniteConditionalCannotCommitOrConsumeFrontier) {
  FixedLagGraph graph(Policy());
  const auto first = Local(0);
  ASSERT_TRUE(graph.Initialize(first, Observation(first, 1)).ok());
  const auto before = graph.state();
  auto next = Local(1);
  next.covariance = first.covariance;
  auto motion = Motion(first, next);
  // Within the endpoint joint-PSD audit tolerance, but not roundoff in the
  // conditional model. Do not convert this negative mode into a constraint.
  motion.cross_covariance(0, 0) += 1e-12;
  const auto invalid = graph.AddLocal(next, motion);
  EXPECT_EQ(invalid.reason, Reason::INVALID_INPUT);
  EXPECT_EQ(graph.state().correction_id, before.correction_id);
  EXPECT_TRUE(graph.state().covariance.isApprox(before.covariance, 0));
  ASSERT_TRUE(graph.AddLocal(next, Motion(first, next)).ok());
}

TEST(FixedLagGraphTest, SharedLocalNoiseUsesJointPoseErrorInnovation) {
  FixedLagGraph graph(Policy());
  const auto local = Local(0);
  ASSERT_TRUE(graph.Initialize(local, Observation(local, 1)).ok());
  auto fix = Observation(local, 1);
  fix.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
  fix.source = "shared-scan-first";
  fix.pose.translation().x() = 100.05;
  fix.covariance.topLeftCorner<2, 2>() = 0.05 * Eigen::Matrix2d::Identity();
  fix.independent_of_local = false;
  fix.conditional_local_error_qualified = true;
  fix.local_error_sequence = local.stamp.sequence;
  fix.local_error_cross_covariance = Eigen::MatrixXd::Zero(15, 2);
  fix.local_error_cross_covariance.topLeftCorner<2, 2>() =
      0.05 * Eigen::Matrix2d::Identity();
  ASSERT_TRUE(graph.Observe(fix).ok());
  EXPECT_NEAR(graph.state().map_to_odom.translation().x(),
              100 + 0.05 * 0.01 / (0.01 + 0.05), 1e-8);
  fix.source = "shared-scan-second";
  ASSERT_TRUE(graph.Observe(fix).ok());
  // Both sources share 0.025 of noise through the same local error. Only
  // their remaining 0.025 is independent and can be reduced by two samples.
  EXPECT_NEAR(graph.state().map_to_odom.translation().x(),
              100 + 0.05 * 0.01 / (0.01 + 0.025 + 0.025 / 2), 1e-8);
  EXPECT_NEAR(graph.state().map_to_odom.translation().y(), 0, 1e-8);
}

TEST(FixedLagGraphTest, InvalidConditionalSourceCannotConsumeOriginalIdentity) {
  FixedLagGraph graph(Policy());
  auto previous = Local(0);
  ASSERT_TRUE(graph.Initialize(previous, Observation(previous, 1)).ok());
  for (uint64_t i = 1; i <= 5; ++i) {
    auto next = Local(i);
    next.covariance = previous.covariance;
    ASSERT_TRUE(graph.AddLocal(next, Motion(previous, next)).ok());
    previous = next;
  }
  auto historical = Local(3);
  historical.covariance = Local(0).covariance;
  auto fix = Observation(historical, 1);
  fix.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
  fix.source = "delayed-shared-scan";
  fix.stamp.receive_time = previous.stamp.time;
  fix.independent_of_local = false;
  fix.conditional_local_error_qualified = true;
  fix.local_error_sequence = historical.stamp.sequence + 1;
  fix.local_error_cross_covariance = Eigen::MatrixXd::Zero(15, 2);
  fix.local_error_cross_covariance(0, 0) = 1;
  const auto before = graph.state();
  EXPECT_EQ(graph.Observe(fix).reason, Reason::CORRELATION_UNQUALIFIED);
  fix.local_error_sequence = historical.stamp.sequence;
  EXPECT_EQ(graph.Observe(fix).reason, Reason::INVALID_INPUT);
  EXPECT_EQ(graph.state().correction_id, before.correction_id);
  fix.local_error_cross_covariance(0, 0) = 0.01;
  ASSERT_TRUE(graph.Observe(fix).ok());
  EXPECT_EQ(graph.state().correction_id, before.correction_id + 1);
  EXPECT_EQ(graph.Observe(fix).reason, Reason::DUPLICATE);
  auto next = Local(6);
  next.covariance = previous.covariance;
  ASSERT_TRUE(graph.AddLocal(next, Motion(previous, next)).ok());
}

TEST(FixedLagGraphTest, ConditionalFullAnchorAndProjectedUpdateShareErrorState) {
  FixedLagGraph graph(Policy());
  const auto local = Local(0);
  auto anchor = Observation(local, 1);
  anchor.independent_of_local = false;
  anchor.conditional_local_error_qualified = true;
  anchor.local_error_sequence = local.stamp.sequence;
  anchor.local_error_cross_covariance = Eigen::MatrixXd::Zero(15, 6);
  anchor.local_error_cross_covariance(0, 0) = 0.01;
  ASSERT_TRUE(graph.Initialize(local, anchor).ok());
  const auto before = graph.state();
  auto projected = anchor;
  projected.kind = GlobalObservation::Kind::PROJECTED_POSE;
  projected.source = "shared-projected";
  projected.projection = Eigen::MatrixXd::Zero(1, 6);
  projected.projection(0, 0) = 1;
  projected.projected_covariance = Eigen::MatrixXd::Constant(1, 1, 0.01);
  projected.local_error_cross_covariance = Eigen::MatrixXd::Zero(15, 1);
  projected.local_error_cross_covariance(0, 0) = 0.01;
  projected.pose.translation().x() += 0.02;
  ASSERT_TRUE(graph.Observe(projected).ok());
  // Common error remains common; only the independent 0.009 noise is
  // averaged. An independent-factor implementation gives a different gain.
  EXPECT_NEAR(graph.state().map_to_odom.translation().x(), 100.01, 1e-8);
  EXPECT_NEAR(graph.state().map_to_odom.translation().y(), 0, 1e-8);
  EXPECT_DOUBLE_EQ(graph.state().last_full_observation, before.last_full_observation);
}

TEST(FixedLagGraphTest, RejectedUpdatesLeaveTheCommittedSnapshotUntouched) {
  FixedLagGraph graph(Policy());
  const LocalState local = Local(0);
  ASSERT_TRUE(graph.Initialize(local, Observation(local, 1)).ok());
  const auto before = graph.state();
  auto outlier = Observation(local, 2);
  outlier.source = "other-survey";
  outlier.pose.translation().x() += 1000.0;
  ObservationInnovation diagnostics;
  EXPECT_EQ(Reason::INNOVATION_REJECTED,
            graph.Observe(outlier, &diagnostics).reason);
  ASSERT_TRUE(diagnostics.available);
  EXPECT_EQ("map_from_base", diagnostics.prior_state_frame);
  ASSERT_EQ(diagnostics.observed_dimension, diagnostics.residual.size());
  ASSERT_EQ(diagnostics.residual.size(), diagnostics.covariance.rows());
  EXPECT_GT(diagnostics.prior_covariance.trace(), 0.0);
  EXPECT_NEAR(
      diagnostics.nis,
      diagnostics.residual.dot(
          diagnostics.covariance.ldlt().solve(diagnostics.residual)),
      1e-8);
  auto wrong_epoch = Local(1);
  wrong_epoch.epoch.generation = 2;
  EXPECT_EQ(Reason::EPOCH_MISMATCH,
            graph.AddLocal(wrong_epoch, Motion(local, Local(1))).reason);
  EXPECT_TRUE(
      before.map_to_odom.matrix().isApprox(graph.state().map_to_odom.matrix()));
  EXPECT_TRUE(before.covariance.isApprox(graph.state().covariance));
  EXPECT_EQ(before.correction_id, graph.state().correction_id);
  EXPECT_EQ(before.last_observation, graph.state().last_observation);
  EXPECT_TRUE(local.position.isZero());
  EXPECT_TRUE(local.covariance.isApprox(Local(0).covariance));
}

TEST(FixedLagGraphTest,
     RejectsMapContentAndFrameChangesWithoutConsumingIdentity) {
  FixedLagGraph graph(Policy());
  auto anchor = Observation(Local(0), 1);
  anchor.map_content_id = "survey-content";
  anchor.map_frame = "map";
  anchor.base_frame = "body";
  ASSERT_TRUE(graph.Initialize(Local(0), anchor).ok());
  auto observation = anchor;
  observation.source = "independent-survey-2";
  const auto revision = graph.state().correction_id;
  observation.map_content_id = "other-content";
  EXPECT_EQ(Reason::MAP_MISMATCH, graph.Observe(observation).reason);
  observation = anchor;
  observation.source = "independent-survey-2";
  observation.map_frame = "another-map-frame";
  EXPECT_EQ(Reason::MAP_MISMATCH, graph.Observe(observation).reason);
  observation.map_frame = anchor.map_frame;
  observation.base_frame = "another-body-frame";
  EXPECT_EQ(Reason::MAP_MISMATCH, graph.Observe(observation).reason);
  EXPECT_EQ(revision, graph.state().correction_id);
  observation.base_frame = anchor.base_frame;
  ASSERT_TRUE(graph.Observe(observation).ok());
  EXPECT_EQ(revision + 1, graph.state().correction_id);
}

TEST(FixedLagGraphTest, ProjectedEvidenceDoesNotClaimTheNullDirection) {
  FixedLagGraph graph(Policy());
  ASSERT_TRUE(graph.Initialize(Local(0), Observation(Local(0), 1)).ok());
  const auto before = graph.state();
  auto observation = Observation(Local(0), 1);
  observation.source = "independent-wall";
  observation.kind = GlobalObservation::Kind::PROJECTED_POSE;
  observation.pose.translation().y() = 0.05;
  observation.projection = Eigen::Matrix<double, 1, 6>::Zero();
  observation.projection(0, 1) = 1.0;
  observation.projected_covariance = Eigen::MatrixXd::Constant(1, 1, 0.01);
  ASSERT_TRUE(graph.Observe(observation).ok());
  EXPECT_NEAR(before.map_to_odom.translation().x(),
              graph.state().map_to_odom.translation().x(), 1e-8);
  EXPECT_NEAR(before.covariance(0, 0), graph.state().covariance(0, 0), 1e-8);
  EXPECT_GT(graph.state().map_to_odom.translation().y(), 0.0);
  EXPECT_DOUBLE_EQ(before.last_full_observation,
                   graph.state().last_full_observation);
}

TEST(FixedLagGraphTest, OwnerRequiresExplicitMotionQualification) {
  GlobalConfig config;
  config.map_id = "site";
  config.map_version = "v1";
  config.calibration_id = "calibrated";
  config.enable_fixed_lag_graph = true;
  EXPECT_EQ(Reason::CONFIG_INVALID,
            GlobalAlignment(config).ValidateConfig().reason);
  config.motion_error_model_qualified = true;
  EXPECT_TRUE(GlobalAlignment(config).ValidateConfig().ok());
}

TEST(FixedLagGraphTest,
     GlobalOwnerRecoversAfterLostMotionWithoutResettingOdom) {
  GlobalConfig config;
  config.map_id = "site";
  config.map_version = "v1";
  config.calibration_id = "calibrated";
  config.enable_fixed_lag_graph = true;
  config.motion_error_model_qualified = true;
  config.graph_solve_budget_ms = 100000.0;
  config.verification_frames = 2;
  GlobalAlignment owner(config);
  ASSERT_TRUE(owner.ValidateConfig().ok());
  ASSERT_TRUE(owner.AddLocal(Local(0)).ok());
  EXPECT_EQ(Reason::RELOCALIZATION_VERIFYING,
            owner.Observe(Observation(Local(0), 1)).reason);
  auto motion = Motion(Local(0), Local(1));
  ASSERT_TRUE(owner.AddLocal(Local(1), &motion).ok());
  ASSERT_TRUE(owner.Observe(Observation(Local(1), 2)).ok());
  ASSERT_TRUE(owner.state().valid);
  // Miss Local(2) on the transport. Do not request a local reset or freeze
  // the global history forever at Local(1).
  motion = Motion(Local(2), Local(3));
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            owner.AddLocal(Local(3), &motion).reason);
  EXPECT_FALSE(owner.state().valid);
  ASSERT_NE(nullptr, owner.latest_local());
  EXPECT_DOUBLE_EQ(Local(3).stamp.time, owner.latest_local()->stamp.time);
  EXPECT_EQ(Local(0).epoch, owner.latest_local()->epoch);
  EXPECT_EQ(Reason::RELOCALIZATION_VERIFYING,
            owner.Observe(Observation(Local(3), 3)).reason);
  motion = Motion(Local(3), Local(4));
  ASSERT_TRUE(owner.AddLocal(Local(4), &motion).ok());
  ASSERT_TRUE(owner.Observe(Observation(Local(4), 4)).ok());
  EXPECT_TRUE(owner.state().valid);
  EXPECT_EQ(Local(0).epoch, owner.state().epoch);
}

TEST(FixedLagGraphTest, GlobalOwnerRoutesMapAndHorizontalFactorsToOneGraph) {
  GlobalConfig config;
  config.map_id = "site";
  config.map_version = "v1";
  config.calibration_id = "calibrated";
  config.enable_fixed_lag_graph = true;
  config.motion_error_model_qualified = true;
  config.graph_solve_budget_ms = 100000.0;
  config.verification_frames = 2;
  GlobalAlignment owner(config);
  ASSERT_TRUE(owner.AddLocal(Local(0)).ok());
  EXPECT_EQ(Reason::RELOCALIZATION_VERIFYING,
            owner.Observe(Observation(Local(0), 1)).reason);
  auto motion = Motion(Local(0), Local(1));
  ASSERT_TRUE(owner.AddLocal(Local(1), &motion).ok());
  ASSERT_TRUE(owner.Observe(Observation(Local(1), 2)).ok());
  const auto revision = owner.state().correction_id;
  auto fix = Observation(Local(1), 1);
  fix.source = "independent-gnss-xy";
  fix.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
  fix.pose.translation().y() = 0.01;
  ASSERT_TRUE(owner.Observe(fix).ok());
  EXPECT_EQ(revision + 1, owner.state().correction_id);
  EXPECT_DOUBLE_EQ(Local(1).stamp.time, owner.state().last_full_observation);
  EXPECT_EQ(1u, owner.sources().at(fix.source).accepted);
  LocalState retained;
  ASSERT_TRUE(owner.Lookup(Local(1).stamp.time, &retained).ok());
  EXPECT_TRUE(retained.position.isApprox(Local(1).position, 0.0));
  EXPECT_TRUE(retained.covariance.isApprox(Local(1).covariance, 0.0));
  EXPECT_EQ(Local(1).epoch, retained.epoch);
}

TEST(FixedLagGraphTest, FailedBootstrapDoesNotCommitACorrectionRevision) {
  GlobalConfig config;
  config.map_id = "site";
  config.map_version = "v1";
  config.calibration_id = "calibrated";
  config.enable_fixed_lag_graph = true;
  config.motion_error_model_qualified = true;
  config.graph_solve_budget_ms = 100000.0;
  config.verification_frames = 2;
  GlobalAlignment owner(config);
  auto local = Local(0);
  // Pose covariance is positive but the full local error model is not SPD.
  local.covariance(3, 3) = -1.0;
  ASSERT_TRUE(owner.AddLocal(local).ok());
  EXPECT_EQ(Reason::RELOCALIZATION_VERIFYING,
            owner.Observe(Observation(local, 1)).reason);
  local = Local(1);
  local.covariance(3, 3) = -1.0;
  auto motion = MotionIncrement{};
  motion.epoch = local.epoch;
  motion.start = Local(0).stamp;
  motion.end = local.stamp;
  ASSERT_TRUE(owner.AddLocal(local, &motion).ok());
  const auto before = owner.state();
  EXPECT_EQ(Reason::GLOBAL_UNAVAILABLE,
            owner.Observe(Observation(local, 2)).reason);
  EXPECT_FALSE(owner.state().valid);
  EXPECT_EQ(before.correction_id, owner.state().correction_id);
  EXPECT_TRUE(before.map_to_odom.matrix().isApprox(
      owner.state().map_to_odom.matrix(), 0.0));
  EXPECT_TRUE(before.covariance.isApprox(owner.state().covariance, 0.0));
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace openloc
