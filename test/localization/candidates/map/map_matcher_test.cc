// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "candidates/map/map_matcher.h"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"

namespace openloc {
namespace localization {
namespace unified {
namespace {

MatchConfig Config() {
  MatchConfig config;
  config.minimum_points = 20;
  config.source_voxel_size = 0.1;
  config.target_voxel_size = 0.1;
  config.resolution = 0.5;
  config.correspondence_distance = 0.8;
  config.minimum_overlap = 0.9;
  config.max_fitness = 0.3;
  config.max_translation = 1.0;
  config.max_rotation = 0.3;
  config.position_variance_floor = 0.01;
  config.rotation_variance_floor = 0.001;
  return config;
}

Cloud::Ptr AsymmetricMap() {
  Cloud::Ptr map(new Cloud);
  for (int x = 0; x <= 40; ++x) {
    for (int y = 0; y <= 30; ++y) {
      const double px = -4.0 + 0.2 * x;
      const double py = -3.0 + 0.2 * y;
      const double pz = 0.05 * px + 0.02 * py * py + 0.002 * px * py;
      map->push_back(pcl::PointXYZ(px, py, pz));
    }
  }
  for (int y = 0; y <= 20; ++y) {
    for (int z = 1; z <= 15; ++z) {
      const double py = -2.0 + 0.2 * y;
      const double pz = 0.2 * z;
      const double px = 3.5 + 0.03 * std::sin(py) + 0.01 * pz;
      map->push_back(pcl::PointXYZ(px, py, pz));
    }
  }
  for (int x = 0; x <= 25; ++x) {
    for (int z = 1; z <= 13; ++z) {
      const double px = -3.0 + 0.2 * x;
      const double pz = 0.2 * z;
      const double py = 3.1 + 0.025 * std::sin(px) + 0.015 * pz;
      map->push_back(pcl::PointXYZ(px, py, pz));
    }
  }
  return map;
}

Cloud::Ptr ScanInSensorFrame(const Cloud& map,
                             const Eigen::Isometry3d& map_from_scan) {
  Cloud::Ptr scan(new Cloud);
  scan->reserve(map.size());
  const Eigen::Isometry3d scan_from_map = map_from_scan.inverse();
  for (const auto& point : map) {
    const Eigen::Vector3d scan_point =
        scan_from_map * Eigen::Vector3d(point.x, point.y, point.z);
    scan->push_back(
        pcl::PointXYZ(scan_point.x(), scan_point.y(), scan_point.z()));
  }
  return scan;
}

TEST(MapMatcherTest, MissingMapAndEmptyHypothesesAreExplicitErrors) {
  MapMatcher matcher(Config());
  Cloud::Ptr cloud(new Cloud);
  MatchResult result;
  EXPECT_EQ(
      Reason::MAP_NOT_READY,
      matcher.Match(cloud, Eigen::Isometry3d::Identity(), &result).reason);
  EXPECT_EQ(Reason::INVALID_INPUT, matcher.Recover(cloud, {}, &result).reason);
  EXPECT_EQ(Reason::MAP_NOT_READY, matcher.SetMap(cloud).reason);
}

TEST(MapMatcherTest, RejectsInvalidVoxelAndSolverBudgets) {
  const Cloud::Ptr map = AsymmetricMap();
  MatchConfig config = Config();
  config.source_voxel_size = 0.0;
  EXPECT_EQ(Reason::CONFIG_INVALID, MapMatcher(config).SetMap(map).reason);

  config = Config();
  config.target_voxel_size = -0.1;
  EXPECT_EQ(Reason::CONFIG_INVALID, MapMatcher(config).SetMap(map).reason);

  config = Config();
  config.maximum_iterations = 0;
  EXPECT_EQ(Reason::CONFIG_INVALID, MapMatcher(config).SetMap(map).reason);
}

TEST(MapMatcherTest, RejectsNonfiniteMapsAndUncalibratedCovarianceModel) {
  Cloud::Ptr cloud(new Cloud);
  for (int index = 0; index < 30; ++index) {
    cloud->push_back(pcl::PointXYZ(index, 0.0f, 0.0f));
  }
  (*cloud)[0].x = std::numeric_limits<float>::quiet_NaN();
  MapMatcher matcher(Config());
  EXPECT_EQ(Reason::INVALID_INPUT, matcher.SetMap(cloud).reason);
  auto config = Config();
  config.position_variance_floor = 0.0;
  MapMatcher no_model(config);
  EXPECT_EQ(Reason::CONFIG_INVALID, no_model.SetMap(cloud).reason);
}

TEST(MapMatcherTest, PlanarGeometryCannotProduceTrustedSixDofPose) {
  Cloud::Ptr plane(new Cloud);
  for (int x = 0; x < 30; ++x) {
    for (int y = 0; y < 30; ++y) {
      plane->push_back(pcl::PointXYZ(x * 0.15f, y * 0.15f, 0.0f));
    }
  }
  MapMatcher matcher(Config());
  ASSERT_TRUE(matcher.SetMap(plane).ok());
  MatchResult result;
  const auto matched =
      matcher.Match(plane, Eigen::Isometry3d::Identity(), &result);
  if (matched.ok()) {
    EXPECT_FALSE(result.full_pose);
    EXPECT_LT(result.projection.rows(), 6);
  }
}

TEST(MapMatcherTest, RecoversKnownPoseFromAsymmetricThreeDimensionalMap) {
  const Cloud::Ptr map = AsymmetricMap();
  const Eigen::Isometry3d map_from_scan =
      Eigen::Translation3d(0.35, -0.22, 0.18) *
      Eigen::AngleAxisd(0.09, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX());
  const Cloud::Ptr scan = ScanInSensorFrame(*map, map_from_scan);
  MatchConfig config = Config();
  MapMatcher matcher(config);
  ASSERT_TRUE(matcher.SetMap(map).ok());

  Eigen::Isometry3d initial = map_from_scan;
  initial.translation() += Eigen::Vector3d(0.05, -0.04, 0.03);
  initial.linear() =
      Eigen::AngleAxisd(0.02, Eigen::Vector3d(1.0, 1.0, 0.5).normalized())
          .toRotationMatrix() *
      initial.linear();

  MatchResult first;
  const Result first_status = matcher.Match(scan, initial, &first);
  ASSERT_TRUE(first_status.ok()) << first_status.message;
  ASSERT_TRUE(first.full_pose);
  ASSERT_TRUE(first.ndt_support_available);
  EXPECT_LE(first.registration_source_points, first.source_points);
  EXPECT_GT(first.registration_target_points, 0U);
  EXPECT_LE(first.ndt_distance_supported_points,
            first.registration_source_points);
  EXPECT_GE(first.ndt_distance_supported_points,
            first.ndt_normal_supported_points);
  EXPECT_EQ(first.ndt_normal_supported_points, first.matched_points);
  EXPECT_EQ(first.ndt_distance_supported_points,
            first.ndt_normal_supported_points + first.ndt_rank_rejected_points +
                first.ndt_nonplanar_rejected_points +
                first.ndt_other_normal_rejected_points);
  EXPECT_DOUBLE_EQ(first.overlap,
                   static_cast<double>(first.ndt_normal_supported_points) /
                       first.registration_source_points);

  constexpr double kTranslationToleranceMeters = 0.12;
  constexpr double kRotationToleranceRadians = 0.06;
  EXPECT_LT((first.pose.translation() - map_from_scan.translation()).norm(),
            kTranslationToleranceMeters);
  EXPECT_LT(LogRotation(Eigen::Quaterniond(first.pose.linear() *
                                           map_from_scan.linear().transpose()))
                .norm(),
            kRotationToleranceRadians);

  MatchResult repeated;
  const Result repeated_status = matcher.Match(scan, initial, &repeated);
  EXPECT_EQ(first_status.reason, repeated_status.reason);
  ASSERT_TRUE(repeated_status.ok()) << repeated_status.message;
  EXPECT_LT((first.pose.translation() - repeated.pose.translation()).norm(),
            1e-6);
  EXPECT_LT(LogRotation(Eigen::Quaterniond(first.pose.linear() *
                                           repeated.pose.linear().transpose()))
                .norm(),
            1e-6);
}

TEST(MapMatcherTest, NonplanarTargetRejectionIsCountedWithoutAdmittingMapPose) {
  Cloud::Ptr volume(new Cloud);
  for (int x = -4; x <= 4; ++x) {
    for (int y = -4; y <= 4; ++y) {
      for (int z = -4; z <= 4; ++z) {
        volume->push_back(pcl::PointXYZ(x * 0.25f, y * 0.25f, z * 0.25f));
      }
    }
  }
  MapMatcher matcher(Config());
  ASSERT_TRUE(matcher.SetMap(volume).ok());
  MatchResult result;
  EXPECT_FALSE(
      matcher.Match(volume, Eigen::Isometry3d::Identity(), &result).ok());
  ASSERT_TRUE(result.ndt_support_available);
  EXPECT_GT(result.ndt_nonplanar_rejected_points, 0U);
  EXPECT_EQ(result.ndt_distance_supported_points,
            result.ndt_normal_supported_points +
                result.ndt_rank_rejected_points +
                result.ndt_nonplanar_rejected_points +
                result.ndt_other_normal_rejected_points);
}

TEST(MapMatcherTest, SeparateCoverageStillRequiresPlanarInformation) {
  Cloud::Ptr scene = AsymmetricMap();
  for (int x = -4; x <= 4; ++x) {
    for (int y = -4; y <= 4; ++y) {
      for (int z = -4; z <= 4; ++z) {
        scene->push_back(
            pcl::PointXYZ(10.0f + x * 0.25f, y * 0.25f, z * 0.25f));
      }
    }
  }
  auto config = Config();
  config.minimum_overlap = 0.99;
  MapMatcher legacy(config);
  ASSERT_TRUE(legacy.SetMap(scene).ok());
  MatchResult rejected;
  EXPECT_FALSE(
      legacy.Match(scene, Eigen::Isometry3d::Identity(), &rejected).ok());
  ASSERT_TRUE(rejected.ndt_support_available);
  EXPECT_LT(rejected.overlap, config.minimum_overlap);

  config.separate_ndt_support = true;
  MapMatcher separated(config);
  ASSERT_TRUE(separated.SetMap(scene).ok());
  MatchResult accepted;
  const auto status =
      separated.Match(scene, Eigen::Isometry3d::Identity(), &accepted);
  ASSERT_TRUE(status.ok()) << status.message;
  EXPECT_DOUBLE_EQ(rejected.overlap, accepted.overlap);
  EXPECT_EQ(rejected.ndt_normal_supported_points, accepted.matched_points);
  EXPECT_GE(accepted.projection.rows(), 1);
  EXPECT_TRUE(accepted.projected_covariance.allFinite());

  config.minimum_points = accepted.matched_points + 1;
  MapMatcher insufficient(config);
  ASSERT_TRUE(insufficient.SetMap(scene).ok());
  EXPECT_FALSE(
      insufficient.Match(scene, Eigen::Isometry3d::Identity(), &accepted).ok());
}

TEST(MapMatcherTest, PointToPlaneIcpConvergesWithPackagerSolverSettings) {
  const Cloud::Ptr map = AsymmetricMap();
  const Eigen::Isometry3d truth =
      Eigen::Translation3d(0.35, -0.22, 0.18) *
      Eigen::AngleAxisd(0.09, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX());
  const Cloud::Ptr scan = ScanInSensorFrame(*map, truth);
  MatchConfig config = Config();
  config.algorithm = RegistrationAlgorithm::kPointToPlaneIcp;
  config.correspondence_distance = 1.5;
  config.minimum_overlap = 0.4;
  config.maximum_iterations = 40;
  config.normal_neighbors = 15;
  config.normal_planarity_ratio = 0.1;
  config.icp_max_rmse = 0.2;
  config.icp_condition_ratio = 1e-6;
  config.icp_translation_tolerance = 0.001;
  config.icp_rotation_tolerance = 0.0001;
  MapMatcher matcher(config);
  ASSERT_TRUE(matcher.SetMap(map).ok());

  Eigen::Isometry3d initial = truth;
  initial.translation() += Eigen::Vector3d(0.03, -0.02, 0.01);
  initial.linear() =
      Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      initial.linear();
  MatchResult result;
  const Result status = matcher.Match(scan, initial, &result);

  ASSERT_TRUE(status.ok()) << status.message;
  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.iterations, 0U);
  EXPECT_LE(result.iterations, config.maximum_iterations);
  EXPECT_LT((result.pose.translation() - truth.translation()).norm(), 0.05);
  EXPECT_LT(LogRotation(Eigen::Quaterniond(result.pose.linear() *
                                           truth.linear().transpose()))
                .norm(),
            0.02);
  EXPECT_GE(result.source_overlap, config.minimum_overlap);
  EXPECT_GE(result.target_overlap, config.minimum_overlap);
  EXPECT_LT(result.score, config.icp_max_rmse);
}

TEST(MapMatcherTest, IcpUncertaintyIsInvariantToMapOriginTranslation) {
  const Cloud::Ptr map = AsymmetricMap();
  const Eigen::Isometry3d truth =
      Eigen::Translation3d(0.35, -0.22, 0.18) *
      Eigen::AngleAxisd(0.09, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX());
  const Cloud::Ptr scan = ScanInSensorFrame(*map, truth);
  MatchConfig config = Config();
  config.algorithm = RegistrationAlgorithm::kPointToPlaneIcp;
  config.correspondence_distance = 1.5;
  config.minimum_overlap = 0.4;
  config.maximum_iterations = 40;
  config.normal_neighbors = 15;
  config.normal_planarity_ratio = 0.1;
  config.icp_max_rmse = 0.2;
  config.icp_condition_ratio = 1e-6;
  config.icp_translation_tolerance = 0.001;
  config.icp_rotation_tolerance = 0.0001;

  MapMatcher matcher(config);
  ASSERT_TRUE(matcher.SetMap(map).ok());
  MatchResult result;
  ASSERT_TRUE(matcher.Match(scan, truth, &result).ok());

  const Eigen::Vector3d map_origin_shift(1000.0, -700.0, 250.0);
  Cloud::Ptr shifted_map(new Cloud);
  shifted_map->reserve(map->size());
  for (const auto& point : *map) {
    shifted_map->emplace_back(
        static_cast<float>(point.x + map_origin_shift.x()),
        static_cast<float>(point.y + map_origin_shift.y()),
        static_cast<float>(point.z + map_origin_shift.z()));
  }
  MapMatcher shifted_matcher(config);
  ASSERT_TRUE(shifted_matcher.SetMap(shifted_map).ok());
  Eigen::Isometry3d shifted_initial =
      Eigen::Translation3d(map_origin_shift) * truth;
  MatchResult shifted_result;
  const Result shifted_status =
      shifted_matcher.Match(scan, shifted_initial, &shifted_result);
  ASSERT_TRUE(shifted_status.ok()) << shifted_status.message;

  EXPECT_TRUE(shifted_result.projection.rows() > 0);
  EXPECT_TRUE(result.projection.isApprox(shifted_result.projection, 2e-3));
  EXPECT_TRUE(result.projected_covariance.isApprox(
      shifted_result.projected_covariance, 2e-3));
  EXPECT_TRUE(shifted_result.pose.matrix().isApprox(
      (Eigen::Translation3d(map_origin_shift) * result.pose).matrix(), 2e-3));
}

TEST(MapMatcherTest, RejectsInvalidAndInsufficientOverlapExplicitly) {
  const Cloud::Ptr map = AsymmetricMap();
  MatchConfig config = Config();
  config.max_translation = 0.25;
  MapMatcher matcher(config);
  ASSERT_TRUE(matcher.SetMap(map).ok());
  const Eigen::Isometry3d truth =
      Eigen::Translation3d(0.35, -0.22, 0.18) *
      Eigen::AngleAxisd(0.09, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitX());
  const Cloud::Ptr scan = ScanInSensorFrame(*map, truth);

  Cloud::Ptr nonfinite_scan(new Cloud(*scan));
  (*nonfinite_scan)[0].x = std::numeric_limits<float>::quiet_NaN();
  MatchResult result;
  EXPECT_EQ(Reason::INVALID_INPUT,
            matcher.Match(nonfinite_scan, truth, &result).reason);

  Cloud::Ptr nonoverlapping_scan(new Cloud);
  for (int index = 0; index < 100; ++index) {
    nonoverlapping_scan->push_back(pcl::PointXYZ(40.0f + index, -40.0f, 20.0f));
  }
  EXPECT_EQ(
      Reason::MATCH_FAILED,
      matcher.Match(nonoverlapping_scan, Eigen::Isometry3d::Identity(), &result)
          .reason);

  Eigen::Isometry3d low_overlap_seed = truth;
  low_overlap_seed.translation().x() += 1.0;
  const Result low_overlap_status =
      matcher.Match(scan, low_overlap_seed, &result);
  EXPECT_EQ(Reason::MATCH_FAILED, low_overlap_status.reason);
  EXPECT_LT(result.overlap, config.minimum_overlap);
}

TEST(MapMatcherTest, CorridorInformationRetainsNormalButNotLongitudinal) {
  Matrix6d information = Matrix6d::Zero();
  information(1, 1) = 100.0;
  information(2, 2) = 100.0;
  information(3, 3) = 20.0;
  information(4, 4) = 20.0;
  information(5, 5) = 40.0;
  MatchResult result;
  ASSERT_TRUE(ProjectMapInformation(information, 0.01, Config(), &result).ok());
  EXPECT_FALSE(result.full_pose);
  ASSERT_EQ(5, result.projection.rows());
  EXPECT_TRUE(result.projection.col(0).isZero());
  EXPECT_GT(result.projection.col(1).norm(), 0.0);
  EXPECT_TRUE(result.covariance.isZero());
}

TEST(MapMatcherTest, ZeroOrIndefiniteInformationDoesNotBecomeCertain) {
  MatchResult result;
  EXPECT_EQ(
      Reason::DEGENERATE,
      ProjectMapInformation(Matrix6d::Zero(), 0.01, Config(), &result).reason);
  Matrix6d information = Matrix6d::Identity();
  information(0, 0) = -1.0;
  EXPECT_EQ(Reason::INVALID_INPUT,
            ProjectMapInformation(information, 0.01, Config(), &result).reason);
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace openloc
