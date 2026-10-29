// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "candidates/global/global_alignment.h"

#include "gtest/gtest.h"

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"

namespace openloc {
namespace localization {
namespace unified {
namespace {

GlobalConfig Policy() {
  GlobalConfig config;
  config.map_id = "campus";
  config.map_version = "v1";
  config.calibration_id = "calibration";
  return config;
}

TEST(GlobalAlignmentTest, FixedGainBoundRetainsArbitrarySharedCorrelation) {
  Matrix6d prior_root = Matrix6d::Identity();
  prior_root(0, 5) = 990;
  prior_root(0, 0) = 100;
  Eigen::Matrix<double, 2, 6> h = Eigen::Matrix<double, 2, 6>::Zero();
  h.leftCols<2>().setIdentity();
  h(0, 5) = -3;
  const Eigen::Matrix<double, 2, 2> rotation =
      Eigen::Rotation2Dd(0.7).toRotationMatrix();
  Eigen::Matrix<double, 6, 2> gain = Eigen::Matrix<double, 6, 2>::Zero();
  gain.topRows<2>() = 0.5 * rotation;
  const Matrix6d retained = Matrix6d::Identity() - gain * h;
  for (const double sign : {-1.0, 0.0, 1.0}) {
    Eigen::Matrix<double, 2, 6> observed_root = sign * h * prior_root;
    observed_root(0, 2) += 2;
    observed_root(1, 3) -= 3;
    const Matrix6d bound = FixedGainCovarianceBound(
        prior_root * prior_root.transpose(),
        observed_root * observed_root.transpose(), h, gain);
    const Eigen::Matrix<double, 6, 6> actual_root =
        retained * prior_root + gain * observed_root;
    const Matrix6d difference = bound - actual_root * actual_root.transpose();
    const Eigen::Matrix<double, 6, 6> gap_root =
        retained * prior_root - gain * observed_root;
    EXPECT_TRUE(difference.isApprox(gap_root * gap_root.transpose(), 1e-12));
    Eigen::SelfAdjointEigenSolver<Matrix6d> spectrum(difference);
    ASSERT_EQ(Eigen::Success, spectrum.info());
    EXPECT_GE(spectrum.eigenvalues().minCoeff(),
              -1e-12 * difference.cwiseAbs().maxCoeff());
  }
}

TEST(GlobalAlignmentTest, FixedGainCannotEnableQualifiedOrGraphOwner) {
  auto config = Policy();
  config.diagnostic_fixed_gain = true;
  EXPECT_EQ(Reason::CONFIG_INVALID,
            GlobalAlignment(config).ValidateConfig().reason);
  config.diagnostic_only = true;
  EXPECT_TRUE(GlobalAlignment(config).ValidateConfig().ok());
  config.enable_fixed_lag_graph = true;
  EXPECT_EQ(Reason::CONFIG_INVALID,
            GlobalAlignment(config).ValidateConfig().reason);
}

LocalState Local(double time, uint64_t sequence) {
  LocalState local;
  local.stamp = {time, time, sequence, "unix"};
  local.epoch = {"producer", 1};
  local.position.x() = time - 10.0;
  local.covariance = Matrix15d::Identity() * 0.0001;
  local.uncertainty_qualified = true;
  local.valid = true;
  return local;
}

GlobalObservation Observation(const LocalState& local, double offset = 100.0) {
  GlobalObservation observation;
  observation.stamp = local.stamp;
  observation.epoch = local.epoch;
  observation.source = "map";
  observation.map_id = "campus";
  observation.map_version = "v1";
  observation.calibration_id = "calibration";
  observation.pose = Pose(local);
  observation.pose.translation().x() += offset;
  observation.covariance = Matrix6d::Identity() * 0.001;
  observation.quality_valid = true;
  observation.uncertainty_qualified = true;
  return observation;
}

void Initialize(GlobalAlignment* alignment) {
  for (uint64_t sequence = 1; sequence <= 3; ++sequence) {
    auto local = Local(10.0 + sequence * 0.01, sequence);
    ASSERT_TRUE(alignment->AddLocal(local).ok());
    const auto result = alignment->Observe(Observation(local));
    if (sequence < 3) {
      EXPECT_EQ(Reason::RELOCALIZATION_VERIFYING, result.reason);
    } else {
      EXPECT_TRUE(result.ok()) << result.message;
    }
  }
}

TEST(GlobalAlignmentTest, FixedGainHeadingAndProjectionStayInObservedSubspace) {
  for (const auto kind : {GlobalObservation::Kind::HEADING,
                          GlobalObservation::Kind::PROJECTED_POSE}) {
    auto config = Policy();
    config.diagnostic_only = true;
    config.diagnostic_fixed_gain = true;
    GlobalAlignment alignment(config);
    Initialize(&alignment);
    auto local = Local(10.04, 4);
    local.position.setZero();
    ASSERT_TRUE(alignment.AddLocal(local).ok());
    const auto before = alignment.state();
    auto observation = Observation(local);
    observation.kind = kind;
    observation.source = "partial";
    observation.direction_in_base = Eigen::Vector3d::UnitX();
    observation.heading_rad = 0.02;
    observation.heading_variance = 0.01;
    observation.pose.linear() =
        ExpRotation(Eigen::Vector3d(0, 0, 0.02)).toRotationMatrix();
    observation.projection = Eigen::Matrix<double, 1, 6>::Zero();
    observation.projection(0, 5) = 1;
    observation.projected_covariance =
        Eigen::Matrix<double, 1, 1>::Constant(0.01);
    ObservationInnovation innovation;
    const auto result = alignment.Observe(observation, &innovation);
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_TRUE(alignment.state().map_to_odom.translation().isApprox(
        before.map_to_odom.translation(), 1e-12));
    EXPECT_NEAR(
        0.01,
        LogRotation(Eigen::Quaterniond(alignment.state().map_to_odom.linear()))
            .z(),
        1e-12);
    EXPECT_EQ(before.last_full_observation,
              alignment.state().last_full_observation);
  }
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& vector) {
  Eigen::Matrix3d skew;
  skew << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(),
      -vector.y(), vector.x(), 0.0;
  return skew;
}

TEST(GlobalAlignmentTest, RequiresVerifiedGlobalEvidenceWithoutMutatingLocal) {
  GlobalAlignment alignment(Policy());
  ASSERT_TRUE(alignment.ValidateConfig().ok());
  Initialize(&alignment);
  ASSERT_TRUE(alignment.state().valid);
  EXPECT_NEAR(100.0, alignment.state().map_to_odom.translation().x(), 1e-9);
  EXPECT_TRUE(alignment.Evaluate(10.03).ok());
  EXPECT_NEAR(0.03, alignment.latest_local()->position.x(), 1e-9);
  EXPECT_FALSE(alignment.state().georeferenced);
}

TEST(GlobalAlignmentTest, DelayedObservationUsesHistoricalPose) {
  GlobalAlignment alignment(Policy());
  auto first = Local(10.0, 1);
  auto second = Local(10.04, 2);
  ASSERT_TRUE(alignment.AddLocal(first).ok());
  ASSERT_TRUE(alignment.AddLocal(second).ok());
  LocalState interpolated;
  ASSERT_TRUE(alignment.Lookup(10.02, &interpolated).ok());
  EXPECT_NEAR(0.02, interpolated.position.x(), 1e-9);
  auto observation = Observation(interpolated);
  observation.stamp.receive_time = 10.04;
  EXPECT_EQ(Reason::RELOCALIZATION_VERIFYING,
            alignment.Observe(observation).reason);
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            alignment.Lookup(9.99, &interpolated).reason);
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            alignment.Lookup(10.05, &interpolated).reason);
}

TEST(GlobalAlignmentTest, UnmodeledLocalCovarianceRevokesAlignmentAndHistory) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  auto local = Local(10.04, 4);
  local.covariance_model_valid = false;
  local.uncertainty_qualified = false;
  EXPECT_EQ(Reason::INVALID_INPUT, alignment.AddLocal(local).reason);
  EXPECT_FALSE(alignment.state().valid);
  ASSERT_NE(nullptr, alignment.latest_local());
  EXPECT_TRUE(alignment.latest_local()->valid);
  EXPECT_FALSE(alignment.latest_local()->covariance_model_valid);
  LocalState output;
  EXPECT_EQ(Reason::HISTORY_UNAVAILABLE,
            alignment.Lookup(10.03, &output).reason);
  EXPECT_EQ(Reason::INVALID_INPUT, alignment.Lookup(10.04, &output).reason);
  EXPECT_EQ(Reason::INVALID_INPUT,
            alignment.Observe(Observation(local)).reason);
  EXPECT_FALSE(alignment.state().valid);
  EXPECT_EQ(Reason::TIMESTAMP_REGRESSION,
            alignment.AddLocal(Local(10.03, 3)).reason);
}

TEST(GlobalAlignmentTest, RejectsClockMapCovarianceAndEpochMismatch) {
  GlobalAlignment alignment(Policy());
  auto local = Local(10.0, 1);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  auto observation = Observation(local);
  observation.map_version = "v2";
  EXPECT_EQ(Reason::MAP_MISMATCH, alignment.Observe(observation).reason);
  observation = Observation(local);
  observation.epoch.generation = 2;
  EXPECT_EQ(Reason::EPOCH_MISMATCH, alignment.Observe(observation).reason);
  observation = Observation(local);
  observation.stamp.clock_id = "gps";
  EXPECT_EQ(Reason::CLOCK_INVALID, alignment.Observe(observation).reason);
  observation = Observation(local);
  observation.covariance.setZero();
  EXPECT_EQ(Reason::INVALID_INPUT, alignment.Observe(observation).reason);
  EXPECT_FALSE(alignment.state().valid);
}

TEST(GlobalAlignmentTest, RequiresQualifiedUncertaintyForTrustedFusion) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  auto local = Local(10.04, 4);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  const auto correction_id = alignment.state().correction_id;
  auto observation = Observation(local);
  observation.uncertainty_qualified = false;
  EXPECT_EQ(Reason::UNCERTAINTY_UNQUALIFIED,
            alignment.Observe(observation).reason);
  EXPECT_TRUE(alignment.state().valid);
  EXPECT_EQ(correction_id, alignment.state().correction_id);

  observation.uncertainty_qualified = true;
  EXPECT_TRUE(alignment.Observe(observation).ok());
  EXPECT_TRUE(alignment.state().valid);
  EXPECT_GT(alignment.state().correction_id, correction_id);
}

TEST(GlobalAlignmentTest, RequiresQualifiedLocalUncertaintyForTrustedFusion) {
  GlobalAlignment alignment(Policy());
  auto local = Local(10.0, 1);
  local.uncertainty_qualified = false;
  EXPECT_EQ(Reason::UNCERTAINTY_UNQUALIFIED, alignment.AddLocal(local).reason);
  EXPECT_EQ(nullptr, alignment.latest_local());

  local.uncertainty_qualified = true;
  EXPECT_TRUE(alignment.AddLocal(local).ok());
}

TEST(GlobalAlignmentTest, DiagnosticFusionAcceptsUnqualifiedUncertaintyOnly) {
  auto config = Policy();
  config.diagnostic_only = true;
  GlobalAlignment alignment(config);
  auto local = Local(10.0, 1);
  local.covariance_model_valid = false;
  local.uncertainty_qualified = false;
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  ASSERT_TRUE(alignment
                  .SeedDiagnostic(Eigen::Isometry3d::Identity(),
                                  Matrix6d::Identity(), local.stamp.time)
                  .ok());

  auto next = Local(10.01, 2);
  next.covariance_model_valid = false;
  next.uncertainty_qualified = false;
  ASSERT_TRUE(alignment.AddLocal(next).ok());
  auto observation = Observation(next);
  observation.pose = Pose(next);
  observation.uncertainty_qualified = false;
  EXPECT_TRUE(alignment.Observe(observation).ok());
  EXPECT_FALSE(alignment.Evaluate(next.stamp.time).ok());
}

TEST(GlobalAlignmentTest, RejectsNonUnitLocalQuaternionAndRevokesAlignment) {
  for (double scale : {0.0, 2.0}) {
    GlobalAlignment alignment(Policy());
    Initialize(&alignment);
    auto local = Local(10.04, 4);
    local.orientation.coeffs() *= scale;
    EXPECT_EQ(Reason::INVALID_INPUT, alignment.AddLocal(local).reason);
    EXPECT_FALSE(alignment.state().valid);
    EXPECT_NEAR(0.03, alignment.latest_local()->position.x(), 1e-9);
  }
}

TEST(GlobalAlignmentTest, MapAndGnssShareAlignmentWithoutChangingLocalHistory) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  const auto local = Local(10.04, 4);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  auto gnss = Observation(local);
  gnss.source = "gnss";
  gnss.kind = GlobalObservation::Kind::POSITION;
  gnss.point_in_base = Eigen::Vector3d(1.0, 0.5, 0.2);
  gnss.pose.translation() =
      alignment.state().map_to_odom * (Pose(local) * gnss.point_in_base);
  gnss.pose.translation().y() += 0.01;
  gnss.georeferenced = true;
  ASSERT_TRUE(alignment.Observe(gnss).ok());
  const auto after_gnss = alignment.state();
  auto map = Observation(local);
  map.pose = after_gnss.map_to_odom * Pose(local);
  map.pose.translation().y() += 0.01;
  ASSERT_TRUE(alignment.Observe(map).ok());
  EXPECT_GT(alignment.state().correction_id, after_gnss.correction_id);
  ASSERT_EQ(1U, alignment.sources().at("gnss").accepted);
  ASSERT_EQ(2U, alignment.sources().at("map").accepted);
  LocalState retained;
  ASSERT_TRUE(alignment.Lookup(local.stamp.time, &retained).ok());
  EXPECT_TRUE(retained.position.isApprox(local.position, 0.0));
  EXPECT_TRUE(
      retained.orientation.coeffs().isApprox(local.orientation.coeffs(), 0.0));
  EXPECT_TRUE(retained.covariance.isApprox(local.covariance, 0.0));
  const auto before_outlier = alignment.state();
  gnss.stamp.sequence += 1;
  gnss.stamp.time += 0.01;
  gnss.stamp.receive_time = gnss.stamp.time;
  const auto next = Local(gnss.stamp.time, 5);
  ASSERT_TRUE(alignment.AddLocal(next).ok());
  gnss.pose.translation().x() += 1000.0;
  ObservationInnovation diagnostics;
  EXPECT_EQ(Reason::INNOVATION_REJECTED,
            alignment.Observe(gnss, &diagnostics).reason);
  ASSERT_TRUE(diagnostics.available);
  ASSERT_EQ(diagnostics.observed_dimension, diagnostics.residual.size());
  EXPECT_NEAR(
      diagnostics.nis,
      diagnostics.residual.dot(
          diagnostics.covariance.ldlt().solve(diagnostics.residual)),
      1e-8);
  EXPECT_TRUE(alignment.state().map_to_odom.matrix().isApprox(
      before_outlier.map_to_odom.matrix(), 0.0));
  ASSERT_TRUE(alignment.Lookup(local.stamp.time, &retained).ok());
  EXPECT_TRUE(retained.position.isApprox(local.position, 0.0));
}

TEST(GlobalAlignmentTest, RejectsDuplicateAndLargeInnovation) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  auto local = Local(10.04, 4);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  EXPECT_EQ(Reason::INNOVATION_REJECTED,
            alignment.Observe(Observation(local, 200.0)).reason);
  EXPECT_EQ(Reason::DUPLICATE, alignment.Observe(Observation(local)).reason);
  EXPECT_NEAR(100.0, alignment.state().map_to_odom.translation().x(), 1e-9);
  EXPECT_NEAR(0.04, alignment.latest_local()->position.x(), 1e-9);
}

TEST(GlobalAlignmentTest, DiagnosticPriorCannotQualifyOrEnableGraph) {
  auto config = Policy();
  GlobalAlignment normal(config);
  auto local = Local(10.0, 1);
  ASSERT_TRUE(normal.AddLocal(local).ok());
  EXPECT_FALSE(normal
                   .SeedDiagnostic(Eigen::Isometry3d::Identity(),
                                   Matrix6d::Identity(), 10.0)
                   .ok());
  config.diagnostic_only = true;
  config.enable_fixed_lag_graph = true;
  config.motion_error_model_qualified = true;
  EXPECT_FALSE(GlobalAlignment(config).ValidateConfig().ok());
  config.enable_fixed_lag_graph = false;
  GlobalAlignment diagnostic(config);
  local.covariance_model_valid = false;
  ASSERT_TRUE(diagnostic.AddLocal(local).ok());
  ASSERT_TRUE(diagnostic
                  .SeedDiagnostic(Eigen::Isometry3d::Identity(),
                                  Matrix6d::Identity(), 10.0)
                  .ok());
  EXPECT_TRUE(diagnostic.state().diagnostic_only);
  EXPECT_FALSE(diagnostic.Evaluate(10.0).ok());
  EXPECT_FALSE(diagnostic
                   .SeedDiagnostic(Eigen::Isometry3d::Identity(),
                                   Matrix6d::Identity(), 10.0)
                   .ok());
  LocalState retained;
  ASSERT_TRUE(diagnostic.Lookup(10.0, &retained).ok());
  EXPECT_FALSE(retained.covariance_model_valid);
}

TEST(GlobalAlignmentTest, HorizontalPointUsesLeverAndDoesNotMeasureHeight) {
  auto config = Policy();
  config.diagnostic_only = true;
  GlobalAlignment alignment(config);
  const auto local = Local(10.0, 1);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  ASSERT_TRUE(alignment
                  .SeedDiagnostic(Eigen::Isometry3d::Identity(),
                                  Matrix6d::Identity(), 10.0)
                  .ok());
  auto gnss = Observation(local);
  gnss.source = "gnss";
  gnss.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
  gnss.point_in_base = Eigen::Vector3d(1.0, 0.5, 0.2);
  gnss.pose.translation() = Pose(local) * gnss.point_in_base;
  gnss.pose.translation().x() += 0.1;
  gnss.pose.translation().z() = 1000.0;
  gnss.covariance.setZero();
  gnss.covariance.topLeftCorner<2, 2>().setIdentity();
  const double before_z = alignment.state().covariance(2, 2);
  ASSERT_TRUE(alignment.Observe(gnss).ok());
  EXPECT_GT(alignment.state().map_to_odom.translation().x(), 0.0);
  EXPECT_DOUBLE_EQ(0.0, alignment.state().map_to_odom.translation().z());
  EXPECT_GE(alignment.state().covariance(2, 2), before_z);
  EXPECT_TRUE(alignment.latest_local()->position.isApprox(local.position, 0.0));
  EXPECT_EQ(Reason::DUPLICATE, alignment.Observe(gnss).reason);
}

TEST(GlobalAlignmentTest, SourceAndLocalUncertaintyControlHorizontalInfluence) {
  auto config = Policy();
  config.diagnostic_only = true;
  const auto correction = [&config](double receiver_variance,
                                    double local_variance) {
    GlobalAlignment alignment(config);
    auto local = Local(10.0, 1);
    local.covariance = Matrix15d::Identity() * local_variance;
    local.covariance_model_valid = false;
    EXPECT_TRUE(alignment.AddLocal(local).ok());
    EXPECT_TRUE(alignment
                    .SeedDiagnostic(Eigen::Isometry3d::Identity(),
                                    Matrix6d::Identity(), 10.0)
                    .ok());
    auto observation = Observation(local);
    observation.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
    observation.pose.translation() =
        local.position + Eigen::Vector3d(0.1, 0, 0);
    observation.covariance.topLeftCorner<2, 2>() =
        Eigen::Matrix2d::Identity() * receiver_variance;
    EXPECT_TRUE(alignment.Observe(observation).ok());
    return alignment.state().map_to_odom.translation().x();
  };
  const double good = correction(0.01, 0.0001);
  EXPECT_GT(good, correction(1.0, 0.0001));
  EXPECT_GT(good, correction(0.01, 1.0));
}

TEST(GlobalAlignmentTest, LargeUncertaintyCanAdmitTheSameLargePositionJump) {
  for (const double local_variance : {0.0001, 200.0}) {
    auto config = Policy();
    config.diagnostic_only = true;
    GlobalAlignment owner(config);
    auto local = Local(10.0, 1);
    local.position.setZero();
    local.covariance = Matrix15d::Identity() * local_variance;
    ASSERT_TRUE(owner.AddLocal(local).ok());
    ASSERT_TRUE(owner.SeedDiagnostic(Eigen::Isometry3d::Identity(),
                                    Matrix6d::Identity(), 10.0).ok());
    auto fix = Observation(local, 0.0);
    fix.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
    fix.pose.translation() = Eigen::Vector3d(50, -30, 0);
    fix.covariance.topLeftCorner<2, 2>().setIdentity();
    ObservationInnovation diagnostic;
    const auto revision = owner.state().correction_id;
    const auto result = owner.Observe(fix, &diagnostic);
    ASSERT_TRUE(diagnostic.available);
    ASSERT_TRUE(diagnostic.covariance_decomposition_available);
    const Eigen::Matrix2d expected =
        Eigen::Matrix2d::Identity() * (2.0 + 4.0 + 4.0 * local_variance);
    EXPECT_TRUE(diagnostic.covariance.isApprox(expected, 1e-12));
    EXPECT_TRUE(diagnostic.covariance.isApprox(
        diagnostic.prior_contribution + diagnostic.observation_contribution +
        diagnostic.local_contribution, 1e-12));
    EXPECT_NEAR(3400.0 / expected(0, 0), diagnostic.nis, 1e-10);
    if (local_variance < 1.0) {
      EXPECT_EQ(Reason::INNOVATION_REJECTED, result.reason);
      EXPECT_GT(diagnostic.nis, diagnostic.gate_threshold);
      EXPECT_EQ(revision, owner.state().correction_id);
    } else {
      EXPECT_TRUE(result.ok()) << result.message;
      EXPECT_LT(diagnostic.nis, diagnostic.gate_threshold);
      EXPECT_EQ(revision + 1, owner.state().correction_id);
    }
    EXPECT_TRUE(owner.latest_local()->position.isZero(0));
    EXPECT_TRUE(owner.state().diagnostic_only);
    EXPECT_FALSE(owner.Evaluate(10.0).ok());
  }
}

TEST(GlobalAlignmentTest, StaleMapDoesNotRevokeDiagnosticHorizontalFusion) {
  auto config = Policy();
  config.diagnostic_only = true;
  GlobalAlignment owner(config);
  auto local = Local(10.0, 1);
  ASSERT_TRUE(owner.AddLocal(local).ok());
  ASSERT_TRUE(owner
                  .SeedDiagnostic(Eigen::Isometry3d::Identity(),
                                  Matrix6d::Identity(), 10.0)
                  .ok());
  local = Local(20.0, 2);
  ASSERT_TRUE(owner.AddLocal(local).ok());
  auto map = Observation(local, 0.0);
  ASSERT_TRUE(owner.Observe(map).ok());
  EXPECT_TRUE(owner.state().valid);
  auto gnss = Observation(local, 0.0);
  gnss.source = "gnss";
  gnss.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
  gnss.pose.translation().x() += 0.1;
  const auto before = owner.state().correction_id;
  ASSERT_TRUE(owner.Observe(gnss).ok());
  EXPECT_GT(owner.state().correction_id, before);
  EXPECT_TRUE(owner.state().diagnostic_only);
  EXPECT_FALSE(owner.Evaluate(local.stamp.time).ok());
}

TEST(GlobalAlignmentTest, DiagnosticXyDoesNotRepeatedlyDiscountFreeHeight) {
  auto config = Policy();
  config.diagnostic_only = true;
  GlobalAlignment owner(config);
  auto local = Local(10.0, 1);
  local.position.setZero();
  ASSERT_TRUE(owner.AddLocal(local).ok());
  ASSERT_TRUE(owner
                  .SeedDiagnostic(Eigen::Isometry3d::Identity(),
                                  Matrix6d::Identity(), 10.0)
                  .ok());
  for (uint64_t index = 1; index <= 100; ++index) {
    auto observation = Observation(local, 0.0);
    observation.stamp.sequence = index;
    observation.kind = GlobalObservation::Kind::HORIZONTAL_POSITION;
    // Distinct source frontiers at an exact retained ODOM epoch.
    observation.source = "xy-" + std::to_string(index % 2);
    if (index > 2) {
      local.stamp.time += 0.01;
      local.stamp.receive_time = local.stamp.time;
      local.stamp.sequence += 1;
      ASSERT_TRUE(owner.AddLocal(local).ok());
      observation.stamp = local.stamp;
    }
    ASSERT_TRUE(owner.Observe(observation).ok());
    EXPECT_LT(owner.state().covariance(2, 2), 1.1);
    EXPECT_DOUBLE_EQ(owner.state().map_to_odom.translation().z(), 0.0);
  }
  EXPECT_TRUE(owner.state().diagnostic_only);
  EXPECT_FALSE(owner.Evaluate(local.stamp.time).ok());
}

TEST(GlobalAlignmentTest,
     GlobalLossDoesNotInvalidateLocalAndNeedsReverification) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  auto local = Local(12.1, 4);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  EXPECT_EQ(Reason::GLOBAL_STALE, alignment.Evaluate(12.1).reason);
  EXPECT_TRUE(alignment.latest_local()->valid);
  EXPECT_EQ(Reason::RELOCALIZATION_VERIFYING,
            alignment.Observe(Observation(local)).reason);
  EXPECT_FALSE(alignment.state().valid);
}

TEST(GlobalAlignmentTest, EpochChangeInvalidatesAlignmentAndRetiredSessions) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  auto reset = Local(10.04, 4);
  reset.epoch.generation = 2;
  ASSERT_TRUE(alignment.AddLocal(reset).ok());
  EXPECT_FALSE(alignment.state().valid);
  auto old = Local(10.05, 5);
  EXPECT_EQ(Reason::EPOCH_MISMATCH, alignment.AddLocal(old).reason);
  reset = Local(10.06, 1);
  reset.epoch = {"new-producer", 1};
  ASSERT_TRUE(alignment.AddLocal(reset).ok());
  old = Local(10.07, 6);
  EXPECT_EQ(Reason::EPOCH_MISMATCH, alignment.AddLocal(old).reason);
}

TEST(GlobalAlignmentTest, AmbiguityCannotBecomeSingleGlobalPose) {
  GlobalAlignment alignment(Policy());
  auto local = Local(10.0, 1);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  auto observation = Observation(local);
  observation.ambiguous = true;
  EXPECT_EQ(Reason::RELOCALIZATION_AMBIGUOUS,
            alignment.Observe(observation).reason);
  EXPECT_FALSE(alignment.state().valid);
}

TEST(GlobalAlignmentTest, CorrelationBoundsDoNotClaimIndependentPrecision) {
  auto local = Local(10.0, 1);
  const auto covariance =
      TransformCovarianceBound(Pose(local), local, Matrix6d::Identity() * 0.01);
  EXPECT_GE(covariance(0, 0), 2.0 * (0.01 + 0.0001));
  EXPECT_TRUE(ValidCovariance(covariance));
}

TEST(GlobalAlignmentTest, AlignmentBoundRetainsLeverArmVarianceAndCorrelation) {
  for (const double distance : {0.0, 10.0, 100.0, 1000.0}) {
    SCOPED_TRACE(distance);
    auto local = Local(10.0, 1);
    local.position = Eigen::Vector3d(distance, 0, 0);
    const Matrix6d observation = Matrix6d::Identity() * 0.001;
    Matrix6d observation_bound, local_bound;
    const auto bound = TransformCovarianceBound(
        Pose(local), local, observation, &observation_bound, &local_bound);
    EXPECT_TRUE(bound.isApprox(observation_bound + local_bound, 1e-12));
    EXPECT_TRUE(observation_bound.isApprox(10 * local_bound, 1e-12));
    const double variance = 2 * (0.001 + 0.0001);
    EXPECT_NEAR(variance, bound(0, 0), 1e-10);
    EXPECT_NEAR(variance * (1 + distance * distance), bound(1, 1), 1e-8);
    EXPECT_NEAR(variance * (1 + distance * distance), bound(2, 2), 1e-8);
    EXPECT_NEAR(-variance * distance, bound(1, 5), 1e-10);
    EXPECT_NEAR(variance * distance, bound(2, 4), 1e-10);
    EXPECT_NEAR(variance, bound(5, 5), 1e-10);
    EXPECT_TRUE(ValidCovariance(bound));
  }
}

TEST(GlobalAlignmentTest, UnanchoredPositionIsEvidenceNotFullAlignment) {
  GlobalAlignment alignment(Policy());
  const auto local = Local(10.0, 1);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  auto observation = Observation(local);
  observation.kind = GlobalObservation::Kind::POSITION;
  observation.covariance.bottomRightCorner<3, 3>().setZero();
  ASSERT_TRUE(alignment.Observe(observation).ok());
  EXPECT_FALSE(alignment.state().valid);
  EXPECT_FALSE(alignment.Evaluate(10.0).ok());
  ASSERT_EQ(1, alignment.evidence().size());
  EXPECT_DOUBLE_EQ(local.position.x(), alignment.latest_local()->position.x());
}

TEST(GlobalAlignmentTest, CorridorUpdateDoesNotInventLongitudinalInformation) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  const auto before = alignment.state();
  const auto local = Local(10.04, 4);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  auto observation = Observation(local);
  observation.kind = GlobalObservation::Kind::PROJECTED_POSE;
  observation.pose.translation().y() = 0.001;
  observation.projection = Eigen::Matrix<double, 1, 6>::Zero();
  observation.projection(0, 1) = 1.0;
  observation.projected_covariance =
      Eigen::Matrix<double, 1, 1>::Constant(0.001);
  ASSERT_TRUE(alignment.Observe(observation).ok());
  EXPECT_DOUBLE_EQ(before.map_to_odom.translation().x(),
                   alignment.state().map_to_odom.translation().x());
  EXPECT_GE(alignment.state().covariance(0, 0), before.covariance(0, 0));
  EXPECT_DOUBLE_EQ(before.last_full_observation,
                   alignment.state().last_full_observation);
  EXPECT_DOUBLE_EQ(local.position.y(), alignment.latest_local()->position.y());
}

TEST(GlobalAlignmentTest, PartialUpdatesCannotRenewFullPoseLease) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  auto local = Local(12.04, 4);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  auto observation = Observation(local);
  observation.kind = GlobalObservation::Kind::POSITION;
  ASSERT_TRUE(alignment.Observe(observation).ok());
  EXPECT_EQ(Reason::GLOBAL_STALE, alignment.Evaluate(12.04).reason);
  EXPECT_TRUE(alignment.latest_local()->valid);
}

TEST(GlobalAlignmentTest, AlignmentUncertaintyGrowsWithoutNewGlobalEvidence) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  const auto before = alignment.state();
  const auto predicted = alignment.Predict(10.13);
  EXPECT_GT(predicted.covariance(0, 0), before.covariance(0, 0));
  EXPECT_GT(predicted.covariance(5, 5), before.covariance(5, 5));
  EXPECT_TRUE(
      predicted.map_to_odom.matrix().isApprox(before.map_to_odom.matrix()));
  EXPECT_DOUBLE_EQ(before.last_full_observation,
                   predicted.last_full_observation);
}

TEST(GlobalAlignmentTest, ProcessNoiseIsInvariantToTranslatedOdomOrigin) {
  auto config = Policy();
  config.diagnostic_only = true;
  auto local = Local(10.0, 1);
  local.position = Eigen::Vector3d(2.0, -3.0, 1.0);
  GlobalAlignment alignment(config);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  Eigen::Isometry3d prior =
      Eigen::Translation3d(4.0, 5.0, -2.0) *
      Eigen::AngleAxisd(0.3, Eigen::Vector3d(1.0, 2.0, 3.0).normalized());
  const Matrix6d prior_covariance = 0.2 * Matrix6d::Identity();
  ASSERT_TRUE(
      alignment.SeedDiagnostic(prior, prior_covariance, local.stamp.time).ok());

  const Eigen::Vector3d origin_shift(1000.0, -700.0, 250.0);
  auto shifted_local = local;
  shifted_local.position -= origin_shift;
  Eigen::Isometry3d shifted_prior = prior;
  shifted_prior.translation() += prior.linear() * origin_shift;
  Matrix6d coordinate_change = Matrix6d::Identity();
  coordinate_change.topRightCorner<3, 3>() =
      -Skew(prior.linear() * origin_shift);
  const Matrix6d shifted_prior_covariance =
      coordinate_change * prior_covariance * coordinate_change.transpose();
  GlobalAlignment shifted_alignment(config);
  ASSERT_TRUE(shifted_alignment.AddLocal(shifted_local).ok());
  ASSERT_TRUE(shifted_alignment
                  .SeedDiagnostic(shifted_prior, shifted_prior_covariance,
                                  shifted_local.stamp.time)
                  .ok());

  const Matrix6d expected = coordinate_change *
                            alignment.Predict(11.0).covariance *
                            coordinate_change.transpose();
  EXPECT_TRUE(
      shifted_alignment.Predict(11.0).covariance.isApprox(expected, 1e-7));
}

TEST(GlobalAlignmentTest, RankDeficientRowsCannotCompleteRecovery) {
  GlobalAlignment alignment(Policy());
  Initialize(&alignment);
  auto local = Local(10.04, 4);
  ASSERT_TRUE(alignment.AddLocal(local).ok());
  auto observation = Observation(local);
  observation.kind = GlobalObservation::Kind::PROJECTED_POSE;
  observation.projection = Eigen::Matrix<double, 2, 6>::Zero();
  observation.projection(0, 1) = 1.0;
  observation.projection(1, 1) = 1.0;
  observation.projected_covariance = Eigen::Matrix2d::Identity();
  EXPECT_EQ(Reason::INVALID_INPUT, alignment.Observe(observation).reason);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace openloc
