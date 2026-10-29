// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/local/local_lio.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>

#include "gtest/gtest.h"

#include "Eigen/Eigenvalues"
#include "Eigen/SVD"
#include "candidates/local/local_motion.h"

namespace openloc {
namespace localization {
namespace unified {
namespace {

LocalState State(double time, const Epoch& epoch,
                 const Eigen::Vector3d& position,
                 const Eigen::Quaterniond& orientation) {
  LocalState state;
  state.stamp = {time, time, static_cast<uint64_t>(time * 100.0), "clock"};
  state.epoch = epoch;
  state.position = position;
  state.orientation = orientation;
  state.covariance.setIdentity();
  state.covariance *= 0.01;
  state.valid = true;
  return state;
}

LocalConfig LioConfig() {
  LocalConfig config;
  config.enable_lidar = true;
  config.lidar_point_noise_std = 0.01;
  config.lidar_plane_noise_std = 0.01;
  config.lidar_voxel_size = 1.0;
  config.lidar_max_voxels = 100;
  config.lidar_max_scans = 3;
  config.lidar_min_points_per_plane = 3;
  config.lidar_min_correspondences = 2;
  config.lidar_max_points = 1000;
  config.lidar_max_correspondence_distance = 0.2;
  config.lidar_observed_singular_value = 1.0;
  config.lidar_innovation_gate = 100.0;
  config.lidar_anchor_variance_scale = 1.0;
  config.lidar_correction_variance_scale = 1.0;
  config.lidar_max_update_iterations = 2;
  config.lidar_rotation_length = 1.0;
  return config;
}

TEST(LocalLioDeskewTest, UsesPointRotationLeverArmAndMeasurementTime) {
  const Epoch epoch{"session", 4};
  const LocalState start = State(1.0, epoch, Eigen::Vector3d::Zero(),
                                 Eigen::Quaterniond::Identity());
  const LocalState end =
      State(2.0, epoch, Eigen::Vector3d::Zero(),
            Eigen::Quaterniond(Eigen::AngleAxisd(0.5 * std::acos(-1.0),
                                                 Eigen::Vector3d::UnitZ())));
  const std::map<double, LocalState> history{{1.0, start}, {2.0, end}};
  LidarScan scan;
  scan.stamp = end.stamp;
  scan.epoch = epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "calibration";
  scan.base_from_lidar.translation().x() = 1.0;
  scan.points.push_back({Eigen::Vector3d(1.0, 0.0, 0.0), 1.0});
  DeskewedScan deskewed;
  const Result result = DeskewLidarScan(
      scan,
      [&history](double time, LocalState* state) {
        const auto found = history.find(time);
        if (found == history.end()) {
          return Result{Reason::HISTORY_UNAVAILABLE, "missing"};
        }
        *state = found->second;
        return Result{};
      },
      &deskewed);
  ASSERT_TRUE(result.ok()) << result.message;
  ASSERT_EQ(deskewed.points_in_end_base.size(), 1U);
  EXPECT_NEAR(deskewed.points_in_end_base[0].x(), 0.0, 1e-12);
  EXPECT_NEAR(deskewed.points_in_end_base[0].y(), -2.0, 1e-12);
}

TEST(LocalLioDeskewTest, ErrorMapsRetainSharedEndpointAndRawPointDependence) {
  const Epoch epoch{"deskew-error", 1};
  const auto point_state = State(
      1, epoch, Eigen::Vector3d(1, 2, 0.5),
      Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX())));
  const auto end_state = State(
      2, epoch, Eigen::Vector3d(2, 2.5, 0.7),
      Eigen::Quaterniond(Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitZ())));
  LidarScan scan;
  scan.stamp = end_state.stamp;
  scan.epoch = epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "lever-arm";
  scan.base_from_lidar.translation() = Eigen::Vector3d(0.7, 0.1, 0.2);
  scan.base_from_lidar.linear() =
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  scan.points.push_back({Eigen::Vector3d(2, -1, 0.3), 1});
  scan.points.push_back({Eigen::Vector3d(1, 2, 0.4), 2});
  const auto lookup = [&](double time, LocalState* state) {
    *state = time == 1 ? point_state : end_state;
    return Result{};
  };
  DeskewedScan deskewed;
  std::vector<DeskewPointError> errors;
  ASSERT_TRUE(DeskewLidarScan(scan, lookup, &deskewed, &errors).ok());
  ASSERT_EQ(errors.size(), scan.points.size());
  EXPECT_DOUBLE_EQ(errors[0].point_time, 1);
  EXPECT_DOUBLE_EQ(errors[1].point_time, 2);
  // The end-time point is independent of the shared local pose exactly.
  EXPECT_TRUE((errors[1].point_state + errors[1].end_state).isZero(1e-12));
  EXPECT_TRUE(errors[1].raw_point.isApprox(scan.base_from_lidar.linear(), 1e-12));
  const auto perturbed = [](LocalState state, int column, double delta) {
    if (column < 3) state.position[column] += delta;
    else {
      Eigen::Vector3d rotation = Eigen::Vector3d::Zero();
      rotation[column - 3] = delta;
      state.orientation =
          (state.orientation * ExpRotation(rotation)).normalized();
    }
    return state;
  };
  constexpr double step = 1e-6;
  for (int input = 0; input < 3; ++input) {
    const int dimension = input == 2 ? 3 : 6;
    for (int column = 0; column < dimension; ++column) {
      const auto evaluate = [&](double delta) -> Eigen::Vector3d {
        auto point = point_state;
        auto end = end_state;
        auto raw = scan.points[0].position;
        if (input == 0) point = perturbed(point, column, delta);
        else if (input == 1) end = perturbed(end, column, delta);
        else raw[column] += delta;
        return Pose(end).inverse() * Pose(point) * scan.base_from_lidar * raw;
      };
      const Eigen::Vector3d derivative =
          (evaluate(step) - evaluate(-step)) / (2 * step);
      const int state_column = column < 3 ? column : column + 3;
      Eigen::Vector3d actual;
      if (input == 0) actual = errors[0].point_state.col(state_column);
      else if (input == 1) actual = errors[0].end_state.col(state_column);
      else actual = errors[0].raw_point.col(column);
      EXPECT_TRUE(actual.isApprox(derivative, 1e-8))
          << "input=" << input << " column=" << column;
    }
  }
  scan.points[1].time = 0;
  EXPECT_EQ(DeskewLidarScan(scan, lookup, &deskewed, &errors).reason,
            Reason::POINT_TIME_MISSING);
  EXPECT_EQ(errors.size(), 2);
  EXPECT_DOUBLE_EQ(errors[1].point_time, 2);
}

TEST(LocalLioDeskewTest, RejectsMissingTimeAndEpochChange) {
  const Epoch epoch{"session", 1};
  const LocalState state = State(1.0, epoch, Eigen::Vector3d::Zero(),
                                 Eigen::Quaterniond::Identity());
  LidarScan scan;
  scan.stamp = state.stamp;
  scan.epoch = epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "calibration";
  scan.points.push_back({Eigen::Vector3d::Zero(), 0.0});
  DeskewedScan deskewed;
  auto lookup = [&state](double, LocalState* output) {
    *output = state;
    return Result{};
  };
  EXPECT_EQ(DeskewLidarScan(scan, lookup, &deskewed).reason,
            Reason::POINT_TIME_MISSING);
  scan.points[0].time = 1.0;
  auto changed_epoch = [state](double, LocalState* output) mutable {
    *output = state;
    output->epoch.generation = 2;
    return Result{};
  };
  EXPECT_EQ(DeskewLidarScan(scan, changed_epoch, &deskewed).reason,
            Reason::EPOCH_MISMATCH);
}

TEST(LocalLioDeskewTest, RejectsNonRigidExtrinsicBeforeLookingUpHistory) {
  const Epoch epoch{"session", 1};
  LidarScan scan;
  scan.stamp = {1.0, 1.0, 1, "clock"};
  scan.epoch = epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "calibration";
  scan.points.push_back({Eigen::Vector3d::UnitX(), 1.0});
  DeskewedScan output;
  output.points_in_end_base.push_back(Eigen::Vector3d(7.0, 8.0, 9.0));
  int lookups = 0;
  const auto lookup = [&lookups](double, LocalState*) {
    ++lookups;
    return Result{};
  };
  for (int invalid = 0; invalid < 3; ++invalid) {
    scan.base_from_lidar.setIdentity();
    if (invalid == 0) {
      scan.base_from_lidar.linear()(0, 0) = 2.0;
    } else if (invalid == 1) {
      scan.base_from_lidar.linear()(0, 0) = -1.0;
    } else {
      scan.base_from_lidar.matrix()(3, 0) = 0.1;
    }
    EXPECT_EQ(Reason::INVALID_INPUT,
              DeskewLidarScan(scan, lookup, &output).reason);
    EXPECT_EQ(0, lookups);
    ASSERT_EQ(1U, output.points_in_end_base.size());
    EXPECT_TRUE(output.points_in_end_base.front().isApprox(
        Eigen::Vector3d(7.0, 8.0, 9.0)));
  }
}

TEST(LocalLioGeometryTest, StoredPlaneErrorRetainsSupportAndOriginalNoiseReuse) {
  const Epoch epoch{"stored-plane-error", 1};
  auto config = LioConfig();
  config.lidar_max_scans = 2;
  LocalState state = State(2, epoch, Eigen::Vector3d::Zero(),
                            Eigen::Quaterniond::Identity());
  LidarScan scan;
  scan.stamp = state.stamp;
  scan.stamp.sequence = 1;
  scan.epoch = epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "calibration";
  for (double offset : {0, 1})
    for (double x : {0.15, 0.35, 0.55})
      for (double y : {0.17, 0.37, 0.57})
        scan.points.push_back(
            {Eigen::Vector3d(x + offset, y, 0.42 + 0.1 * x + 0.2 * y), 2});
  const auto lookup = [&](double, LocalState* output) {
    *output = state;
    return Result{};
  };
  LocalLio lio(config);
  EXPECT_EQ(lio.Observe(scan, lookup, &state).reason, Reason::GEOMETRY_UNAVAILABLE);
  std::vector<StoredPlaneError> errors;
  ASSERT_TRUE(lio.GeometryErrorMap(&errors).ok());
  ASSERT_EQ(errors.size(), 2);
  const auto select_plane = [](const std::vector<StoredPlaneError>& planes)
      -> const StoredPlaneError& {
    return *std::min_element(planes.begin(), planes.end(),
                            [](const StoredPlaneError& first,
                               const StoredPlaneError& second) {
                              return first.center.x() < second.center.x();
                            });
  };
  const auto nominal = select_plane(errors);
  ASSERT_EQ(nominal.points.size(), scan.points.size() / 2);
  const auto check_moment_queries = [&scan](const std::vector<StoredPlaneError>&
                                                planes) {
    const Eigen::Index drivers = 9 * scan.points.size() + 6;
    std::vector<Eigen::MatrixXd> direct, reduced;
    for (const auto& plane : planes) {
      Eigen::MatrixXd point_query = Eigen::MatrixXd::Zero(6, drivers);
      std::map<uint64_t, Eigen::MatrixXd> scan_moments;
      Eigen::Matrix3d second = Eigen::Matrix3d::Zero();
      Eigen::MatrixXd one_point;
      for (const auto& point : plane.points) {
        Eigen::MatrixXd loading = Eigen::MatrixXd::Zero(3, drivers);
        loading.block<3, 3>(
            0, 3 * ((point.source_sequence - 1) * scan.points.size() +
                    point.point_index)) = 0.02 * Eigen::Matrix3d::Identity();
        loading.rightCols<6>().leftCols<3>() =
            0.01 * Eigen::Matrix3d::Identity();
        loading.rightCols<3>() = -0.005 * Skew(point.point);
        one_point = loading;
        Eigen::Matrix<double, 9, 3> derivative =
            Eigen::Matrix<double, 9, 3>::Zero();
        derivative.topRows<3>().setIdentity();
        const auto& p = point.point;
        derivative.row(3) << 2 * p.x(), 0, 0;
        derivative.row(4) << p.y(), p.x(), 0;
        derivative.row(5) << p.z(), 0, p.x();
        derivative.row(6) << 0, 2 * p.y(), 0;
        derivative.row(7) << 0, p.z(), p.y();
        derivative.row(8) << 0, 0, 2 * p.z();
        auto& moments = scan_moments[point.source_sequence];
        if (moments.size() == 0) moments = Eigen::MatrixXd::Zero(9, drivers);
        moments += derivative * loading;
        point_query.topRows<3>() += point.center * loading;
        point_query.bottomRows<3>() += point.normal * loading;
        second += p * p.transpose();
      }
      const double count = plane.points.size();
      const Eigen::Matrix3d covariance =
          second / count - plane.center * plane.center.transpose();
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> spectrum(covariance);
      EXPECT_EQ(spectrum.info(), Eigen::Success);
      Eigen::Matrix<double, 6, 9> projection =
          Eigen::Matrix<double, 6, 9>::Zero();
      projection.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() / count;
      for (int column = 0; column < 9; ++column) {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Matrix3d change = Eigen::Matrix3d::Zero();
        if (column < 3) {
          center[column] = 1 / count;
        } else {
          constexpr int row[] = {0, 0, 0, 1, 1, 2};
          constexpr int col[] = {0, 1, 2, 1, 2, 2};
          change(row[column - 3], col[column - 3]) = 1 / count;
          change(col[column - 3], row[column - 3]) = 1 / count;
        }
        change -= center * plane.center.transpose() +
                  plane.center * center.transpose();
        for (int axis = 1; axis < 3; ++axis) {
          const Eigen::Vector3d direction = spectrum.eigenvectors().col(axis);
          projection.block<3, 1>(3, column) +=
              direction *
              (direction.dot(change * plane.normal) /
               (spectrum.eigenvalues()[0] - spectrum.eigenvalues()[axis]));
        }
      }
      Eigen::MatrixXd moments = Eigen::MatrixXd::Zero(9, drivers);
      for (const auto& retained : scan_moments) moments += retained.second;
      const Eigen::MatrixXd compressed = projection * moments;
      EXPECT_TRUE(compressed.isApprox(point_query, 1e-12));
      direct.push_back(point_query);
      reduced.push_back(compressed);

      // Plane moments do not retain arbitrary future raw-point queries.
      const Eigen::JacobiSVD<Eigen::MatrixXd> support(moments,
                                                      Eigen::ComputeThinV);
      const auto basis = support.matrixV().leftCols(support.rank()).eval();
      const Eigen::MatrixXd missing =
          one_point - one_point * basis * basis.transpose();
      EXPECT_GT(missing.norm(),
                64 * std::numeric_limits<double>::epsilon() * one_point.norm());
    }
    for (std::size_t first = 0; first < direct.size(); ++first)
      for (std::size_t last = 0; last < direct.size(); ++last)
        EXPECT_TRUE(
            (direct[first] * direct[last].transpose())
                .isApprox(reduced[first] * reduced[last].transpose(), 1e-12));
  };
  check_moment_queries(errors);
  Eigen::Matrix3d center_sum = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d normal_sum = Eigen::Matrix3d::Zero();
  Eigen::Matrix<double, 3, 15> center_anchor =
      Eigen::Matrix<double, 3, 15>::Zero();
  Eigen::Matrix<double, 3, 15> normal_anchor =
      Eigen::Matrix<double, 3, 15>::Zero();
  constexpr double step = 1e-6;
  for (const auto& point : nominal.points) {
    EXPECT_EQ(point.source_sequence, 1);
    EXPECT_TRUE((nominal.normal.transpose() * point.normal).isZero(1e-12));
    center_sum += point.center;
    normal_sum += point.normal;
    center_anchor += point.center * point.anchor_state;
    normal_anchor += point.normal * point.anchor_state;
    EXPECT_TRUE(point.deskewed_point.isIdentity(1e-12));
    for (int axis = 0; axis < 3; ++axis) {
      const auto evaluate = [&](double delta) {
        auto perturbed_scan = scan;
        perturbed_scan.points[point.point_index].position[axis] += delta;
        LocalLio candidate(config);
        auto anchor = state;
        EXPECT_EQ(candidate.Observe(perturbed_scan, lookup, &anchor).reason,
                  Reason::GEOMETRY_UNAVAILABLE);
        std::vector<StoredPlaneError> map;
        EXPECT_TRUE(candidate.GeometryErrorMap(&map).ok());
        EXPECT_EQ(map.size(), 2);
        return map;
      };
      const auto plus = evaluate(step);
      const auto minus = evaluate(-step);
      ASSERT_EQ(plus.size(), 2);
      ASSERT_EQ(minus.size(), 2);
      const Eigen::Vector3d center_derivative =
          (select_plane(plus).center - select_plane(minus).center) / (2 * step);
      Eigen::Vector3d plus_normal = select_plane(plus).normal;
      Eigen::Vector3d minus_normal = select_plane(minus).normal;
      if (plus_normal.dot(nominal.normal) < 0) plus_normal *= -1;
      if (minus_normal.dot(nominal.normal) < 0) minus_normal *= -1;
      const Eigen::Vector3d normal_derivative =
          (plus_normal - minus_normal) / (2 * step);
      EXPECT_TRUE(center_derivative.isApprox(point.center.col(axis), 1e-8));
      EXPECT_LE((normal_derivative - point.normal.col(axis)).norm(),
                1e-7 * std::max(1.0, point.normal.col(axis).norm()));
    }
  }
  EXPECT_TRUE(center_sum.isIdentity(1e-12));
  EXPECT_TRUE(normal_sum.isZero(1e-12));
  EXPECT_TRUE((center_anchor.block<3, 3>(0, 0).isIdentity(1e-12)));
  EXPECT_TRUE((center_anchor.block<3, 3>(0, 6).isApprox(
      -Skew(nominal.center), 1e-12)));
  EXPECT_TRUE((normal_anchor.block<3, 3>(0, 0).isZero(1e-12)));
  EXPECT_TRUE((normal_anchor.block<3, 3>(0, 6).isApprox(
      -Skew(nominal.normal), 1e-12)));
  for (uint64_t sequence : {2, 3}) {
    scan.stamp.sequence = sequence;
    scan.stamp.time += 1;
    state.stamp = scan.stamp;
    for (auto& point : scan.points) point.time = scan.stamp.time;
    const auto observed = lio.Observe(scan, lookup, &state);
    ASSERT_TRUE(observed.ok()) << ReasonName(observed.reason) << ": " << observed.message;
  }
  std::map<uint64_t, LocalState> anchors;
  ASSERT_TRUE(lio.GeometryErrorMap(&errors, &anchors).ok());
  ASSERT_EQ(errors.size(), 2);
  check_moment_queries(errors);
  EXPECT_EQ(anchors.size(), 2);
  EXPECT_EQ(anchors.count(1), 0);
  EXPECT_EQ(anchors.count(2), 1);
  EXPECT_EQ(anchors.count(3), 1);
  EXPECT_EQ(lio.scan_count(), 2);
  EXPECT_EQ(select_plane(errors).points.size(), 2 * nominal.points.size());
  bool second_seen = false, third_seen = false;
  for (const auto& point : select_plane(errors).points) {
    EXPECT_NE(point.source_sequence, 1);
    second_seen |= point.source_sequence == 2;
    third_seen |= point.source_sequence == 3;
  }
  EXPECT_TRUE(second_seen);
  EXPECT_TRUE(third_seen);
  lio.Reset(epoch);
  EXPECT_EQ(lio.GeometryErrorMap(&errors).reason, Reason::GEOMETRY_UNAVAILABLE);
  EXPECT_EQ(errors.size(), 2);
}

TEST(LocalLioGeometryTest, FrozenVoxelSupportUsesWorldPointsAndResetsWithEpoch) {
  LocalLio lio(LioConfig());
  for (uint64_t generation : {1, 2}) {
    const Epoch epoch{"voxel-cache", generation};
    LocalState state = State(
        2, epoch, generation == 1 ? Eigen::Vector3d(-0.4, 0.2, -0.8)
                                 : Eigen::Vector3d(0.6, -0.8, 1.2),
        Eigen::Quaterniond::Identity());
    LidarScan scan;
    scan.stamp = state.stamp;
    scan.stamp.sequence = generation;
    scan.epoch = epoch;
    scan.frame_id = "lidar";
    scan.calibration_id = "calibration";
    std::map<std::array<int, 3>, std::vector<Eigen::Vector3d>> expected;
    for (double x : {0.11, 0.31, 0.51, 0.71, 0.91}) {
      for (double y : {0.13, 0.23, 0.33, 0.43, 0.53}) {
        const Eigen::Vector3d point(x, y, 0.37 + 0.1 * x + 0.2 * y);
        scan.points.push_back({point, 2});
        const Eigen::Vector3d world = Pose(state) * point;
        const std::array<int, 3> key{{
            static_cast<int>(std::floor(world.x())),
            static_cast<int>(std::floor(world.y())),
            static_cast<int>(std::floor(world.z()))}};
        expected[key].push_back(world);
      }
    }
    EXPECT_EQ(Reason::GEOMETRY_UNAVAILABLE,
              lio.Observe(scan, [&](double, LocalState* output) {
                *output = state;
                return Result{};
              }, &state).reason);
    std::vector<StoredPlaneError> errors;
    ASSERT_TRUE(lio.GeometryErrorMap(&errors).ok());
    EXPECT_EQ(expected.size(), errors.size());
    EXPECT_EQ(1U, lio.scan_count());
    for (const auto& plane : errors) {
      const auto& center = plane.center;
      const std::array<int, 3> key{{
          static_cast<int>(std::floor(center.x())),
          static_cast<int>(std::floor(center.y())),
          static_cast<int>(std::floor(center.z()))}};
      ASSERT_EQ(1U, expected.count(key));
      ASSERT_EQ(expected.at(key).size(), plane.points.size());
      for (size_t i = 0; i < plane.points.size(); ++i) {
        EXPECT_EQ(generation, plane.points[i].source_sequence);
        EXPECT_TRUE(plane.points[i].point.isApprox(expected.at(key)[i], 1e-12));
      }
    }
  }
}

TEST(LocalLioGeometryTest, UndefinedStoredGeometryBranchesCannotReplaceErrorMap) {
  const Epoch epoch{"stored-plane-branch", 1};
  LocalState state = State(2, epoch, Eigen::Vector3d::Zero(),
                            Eigen::Quaterniond::Identity());
  LidarScan scan;
  scan.stamp = state.stamp;
  scan.epoch = epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "calibration";
  const auto lookup = [&](double, LocalState* output) {
    *output = state;
    return Result{};
  };
  std::vector<StoredPlaneError> errors(1);
  errors[0].center.x() = 42;
  for (bool voxel_boundary : {false, true}) {
    scan.points.clear();
    if (voxel_boundary) {
      for (double x : {0.0, 0.2, 0.4})
        for (double y : {0.2, 0.4, 0.6})
          scan.points.push_back({Eigen::Vector3d(x, y, 0.5), 2});
    } else {
      for (double x : {0.2, 0.4})
        for (double y : {0.25, 0.45})
          for (double z : {0.3, 0.5})
            scan.points.push_back({Eigen::Vector3d(x, y, z), 2});
    }
    LocalLio lio(LioConfig());
    EXPECT_EQ(lio.Observe(scan, lookup, &state).reason, Reason::GEOMETRY_UNAVAILABLE);
    EXPECT_EQ(lio.GeometryErrorMap(&errors).reason, Reason::CORRELATION_UNQUALIFIED);
    ASSERT_EQ(errors.size(), 1);
    EXPECT_DOUBLE_EQ(errors[0].center.x(), 42);
    EXPECT_EQ(lio.GeometryErrorMap(nullptr).reason, Reason::INVALID_INPUT);
    EXPECT_EQ(lio.scan_count(), 1);
  }
}

TEST(LocalLioGeometryTest, CorridorDoesNotCreateLongitudinalInformation) {
  const Epoch epoch{"session", 1};
  LocalState state = State(3.0, epoch, Eigen::Vector3d::Zero(),
                           Eigen::Quaterniond::Identity());
  state.uncertainty_qualified = true;
  LocalConfig config = LioConfig();
  LocalLio lio(config);
  LidarScan scan;
  scan.stamp = state.stamp;
  scan.epoch = epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "calibration";
  for (double x : {0.1, 0.3, 0.5}) {
    for (double z : {0.1, 0.3, 0.5}) {
      scan.points.push_back({Eigen::Vector3d(x, 1.0, z), 3.0});
      scan.points.push_back({Eigen::Vector3d(x, -1.0, z), 3.0});
    }
  }
  auto lookup = [&state](double, LocalState* output) {
    *output = state;
    return Result{};
  };
  EXPECT_EQ(lio.Observe(scan, lookup, &state).reason,
            Reason::GEOMETRY_UNAVAILABLE);
  EXPECT_TRUE(state.covariance_model_valid);
  EXPECT_TRUE(state.uncertainty_qualified);
  scan.points.front().time = 0.0;
  EXPECT_EQ(lio.Observe(scan, lookup, &state).reason,
            Reason::POINT_TIME_MISSING);
  EXPECT_TRUE(state.covariance_model_valid);
  EXPECT_TRUE(state.uncertainty_qualified);
  scan.points.front().time = state.stamp.time;
  const Matrix15d covariance_before = state.covariance;
  scan.stamp.sequence += 1;
  ASSERT_TRUE(lio.Observe(scan, lookup, &state).ok());
  EXPECT_TRUE(state.valid);
  EXPECT_FALSE(state.covariance_model_valid);
  EXPECT_FALSE(state.uncertainty_qualified);
  EXPECT_TRUE(state.covariance.isApprox(covariance_before));
  ASSERT_GT(lio.observed_pose_basis().rows(), 0);
  EXPECT_LT(lio.observed_pose_basis().col(0).norm(), 1e-10);
  EXPECT_LT(lio.observed_pose_basis().rows(), 6);
  EXPECT_LE(lio.scan_count(), config.lidar_max_scans);
}

TEST(LocalLioGeometryTest, RelinearizationDoesNotReapplyTheSamePlanarObservation) {
  const Epoch epoch{"session", 1};
  const auto estimate = [&epoch](uint32_t iterations, bool use_factor = false) {
    LocalConfig config = LioConfig();
    config.lidar_min_correspondences = 10;
    config.lidar_max_update_iterations = iterations;
    LocalLio lio(config);
    LocalState state = State(1.0, epoch, Eigen::Vector3d::Zero(),
                             Eigen::Quaterniond::Identity());
    // Keep the prior weakly sensitive to attitude, so this is a linear
    // translation/velocity observation with an unchanged plane association.
    state.covariance.block<3, 3>(6, 6) *= 1e-10;
    state.covariance(0, 3) = state.covariance(3, 0) = 0.002;
    if (use_factor) {
      EXPECT_TRUE(AdvanceCovarianceSquareRoot(state, Matrix15d::Identity(),
                                              Matrix15d::Zero(), &state)
                      .ok());
    }
    LidarScan scan;
    scan.stamp = state.stamp;
    scan.epoch = epoch;
    scan.frame_id = "lidar";
    scan.calibration_id = "calibration";
    for (int y = -2; y <= 2; ++y) {
      for (int z = -2; z <= 2; ++z) {
        for (double dy : {0.15, 0.35, 0.55}) {
          for (double dz : {0.15, 0.35, 0.55}) {
            scan.points.push_back(
                {Eigen::Vector3d(1.2, y + dy, z + dz), 1.0});
          }
        }
      }
    }
    auto lookup = [&state](double, LocalState* output) {
      *output = state;
      return Result{};
    };
    EXPECT_EQ(lio.Observe(scan, lookup, &state).reason,
              Reason::GEOMETRY_UNAVAILABLE);
    state.position.x() = 0.05;
    scan.stamp.sequence++;
    EXPECT_TRUE(lio.Observe(scan, lookup, &state).ok());
    EXPECT_FALSE(state.uncertainty_qualified);
    return state;
  };
  const LocalState once = estimate(1);
  const LocalState iterated = estimate(5);
  EXPECT_LT(once.position.x(), 0.05);
  EXPECT_GT(once.position.x(), 0.0);
  EXPECT_NEAR(iterated.position.x(), once.position.x(), 1e-10);
  EXPECT_NEAR(iterated.velocity.x(), once.velocity.x(), 1e-10);
  EXPECT_TRUE(iterated.covariance.isApprox(once.covariance, 1e-10));
  for (const uint32_t iterations : {1U, 5U}) {
    const auto factored = estimate(iterations, true);
    EXPECT_TRUE(factored.covariance_square_root);
    EXPECT_TRUE(ValidLocalCovariance(factored));
    EXPECT_TRUE(factored.position.isApprox(iterated.position, 1e-10));
    EXPECT_TRUE(factored.covariance.isApprox(iterated.covariance, 1e-10));
    EXPECT_FALSE(factored.covariance_model_valid);
  }
}

TEST(LocalLioGeometryTest, LargeAnchorUncertaintyDoesNotErasePlanarGeometry) {
  const Epoch epoch{"uncertain-gauge", 1};
  auto config = LioConfig();
  config.lidar_min_correspondences = 10;
  for (const double variance : {0.01, 1e8}) {
    LocalLio lio(config);
    auto state = State(1, epoch, Eigen::Vector3d::Zero(),
                       Eigen::Quaterniond::Identity());
    state.covariance = Matrix15d::Identity() * variance;
    LidarScan scan;
    scan.stamp = state.stamp;
    scan.epoch = epoch;
    scan.frame_id = "lidar";
    scan.calibration_id = "calibration";
    for (int x = -2; x <= 2; ++x)
      for (int y = -2; y <= 2; ++y)
        for (double dx : {0.15, 0.35, 0.55})
          for (double dy : {0.17, 0.37, 0.57})
            scan.points.push_back({Eigen::Vector3d(x + dx, y + dy, 0.42), 1});
    const auto lookup = [&](double, LocalState* output) {
      *output = state;
      return Result{};
    };
    ASSERT_EQ(Reason::GEOMETRY_UNAVAILABLE, lio.Observe(scan, lookup, &state).reason);
    scan.stamp.sequence++;
    state.position.z() = 0.01;
    const auto prior = state.covariance;
    const auto result = lio.Observe(scan, lookup, &state);
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(3, lio.observed_pose_basis().rows());
    EXPECT_LT(lio.observed_pose_basis().col(0).norm(), 1e-10);
    EXPECT_LT(lio.observed_pose_basis().col(1).norm(), 1e-10);
    EXPECT_GT(state.position.z(), 0);
    EXPECT_LT(state.position.z(), 0.01);
    EXPECT_GE(state.covariance(2, 2), prior(2, 2));
    EXPECT_FALSE(state.covariance_model_valid);
    EXPECT_FALSE(state.uncertainty_qualified);
  }
}

TEST(LocalLioGeometryTest, DiagnosticGainMemoryCannotReplaceReportedUncertainty) {
  const Epoch epoch{"gain-memory", 1};
  auto config = LioConfig();
  config.lidar_min_correspondences = 10;
  config.lidar_max_update_iterations = 3;
  LocalLio lio(config);
  auto state = State(1, epoch, Eigen::Vector3d::Zero(),
                     Eigen::Quaterniond::Identity());
  state.covariance *= 1e10;
  Matrix15d gain_covariance = Matrix15d::Identity() * 0.01;
  gain_covariance.block<3, 3>(6, 6) *= 1e-10;
  LidarScan scan;
  scan.stamp = state.stamp;
  scan.epoch = epoch;
  scan.frame_id = "lidar";
  scan.calibration_id = "calibration";
  for (int x = -2; x <= 2; ++x)
    for (int y = -2; y <= 2; ++y)
      for (double dx : {0.15, 0.35, 0.55})
        for (double dy : {0.17, 0.37, 0.57})
          scan.points.push_back({Eigen::Vector3d(x + dx, y + dy, 0.42), 1});
  const auto lookup = [&](double, LocalState* output) {
    *output = state;
    return Result{};
  };
  ASSERT_EQ(Reason::GEOMETRY_UNAVAILABLE,
            lio.Observe(scan, lookup, &state, {}, &gain_covariance).reason);
  state.position.z() = 0.01;
  scan.stamp.sequence++;
  const auto reported = state.covariance;
  const auto gain_prior = gain_covariance;
  Eigen::MatrixXd last_gain, last_h, last_noise;
  const auto observed = [&](const LioUpdate& update, const auto&, const auto&, const auto&) {
    last_gain = update.applied_gain;
    last_h = update.observation;
    last_noise = update.noise;
  };
  ASSERT_TRUE(lio.Observe(scan, lookup, &state, observed, &gain_covariance).ok());
  const Matrix15d residual = Matrix15d::Identity() - last_gain * last_h;
  const Matrix15d expected =
      residual * gain_prior * residual.transpose() +
      last_gain * last_noise * last_gain.transpose();
  EXPECT_NEAR(expected(2, 2), gain_covariance(2, 2), 1e-10);
  EXPECT_NEAR(0.01 * gain_covariance(2, 2) / gain_prior(2, 2),
              state.position.z(), 1e-9);
  EXPECT_LT(gain_covariance(2, 2), gain_prior(2, 2));
  EXPECT_GE(state.covariance(2, 2), reported(2, 2));
  EXPECT_FALSE(state.covariance_model_valid);
  EXPECT_FALSE(state.uncertainty_qualified);
  const auto committed = gain_covariance;
  scan.points[0].time = 0;
  EXPECT_EQ(Reason::POINT_TIME_MISSING,
            lio.Observe(scan, lookup, &state, {}, &gain_covariance).reason);
  EXPECT_TRUE(committed.isApprox(gain_covariance, 0));
}

}  // namespace
}  // namespace unified
}  // namespace localization
}  // namespace openloc
