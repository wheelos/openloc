// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <deque>
#include <functional>
#include <map>
#include <unordered_map>
#include <vector>

#include "candidates/common/types.h"

namespace openloc {
namespace localization {
namespace unified {

using StateLookup = std::function<Result(double, LocalState*)>;

struct DeskewedScan {
  Stamp stamp;
  Epoch epoch;
  std::vector<Eigen::Vector3d> points_in_end_base;
};

struct DeskewPointError {
  double point_time = 0;
  // Perturbations use position_odom/right-attitude_body at each actual time.
  // Equal times are the SAME variable; never add their marginals separately.
  Eigen::Matrix<double, 3, 15> point_state =
      Eigen::Matrix<double, 3, 15>::Zero();
  Eigen::Matrix<double, 3, 15> end_state =
      Eigen::Matrix<double, 3, 15>::Zero();
  Eigen::Matrix3d raw_point = Eigen::Matrix3d::Zero();
};

Result DeskewLidarScan(const LidarScan& scan, const StateLookup& lookup,
                       DeskewedScan* deskewed,
                       std::vector<DeskewPointError>* errors = nullptr);

struct LioUpdate {
  struct Residual {
    std::size_t point_index = 0;
    Eigen::Vector3d plane_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d plane_normal = Eigen::Vector3d::Zero();
  };
  Eigen::MatrixXd observation;
  Eigen::VectorXd innovation;
  Eigen::MatrixXd noise;
  Eigen::MatrixXd observed_pose_basis;
  uint32_t correspondences = 0;
  // Actual accepted residuals and their observed-subspace whitening map.
  std::vector<Residual> residuals;
  Eigen::MatrixXd residual_projection;
  Eigen::MatrixXd applied_gain;
};

using LioIterationObserver = std::function<void(
    const LioUpdate&, const LocalState&, const LocalState&, const LocalState&)>;

class LocalLio {
 public:
  explicit LocalLio(const LocalConfig& config) : config_(config) {}

  // A successful correlated correction preserves numerical covariance but
  // revokes its error-model and calibration claims. Geometry-only startup and
  // rejected corrections do not change the supplied state.
  Result Observe(const LidarScan& scan, const StateLookup& lookup,
                 LocalState* state,
                 const LioIterationObserver& observer = {},
                 Matrix15d* tracking_gain_covariance = nullptr);
  void Reset(const Epoch& epoch);
  const Eigen::MatrixXd& observed_pose_basis() const {
    return observed_pose_basis_;
  }
  uint32_t scan_count() const { return scans_.size(); }
  std::vector<uint64_t> geometry_source_sequences() const;
  std::vector<std::pair<uint64_t, std::size_t>> stored_point_counts() const;
  // Derivatives w.r.t. the actual frozen stored points, not independent noise.
  // Undefined voxel/eigenvector branches reject without changing geometry.
  Result GeometryErrorMap(
      std::vector<StoredPlaneError>* errors,
      std::map<uint64_t, LocalState>* anchors = nullptr) const;

 private:
  struct Plane {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double variance = 0.0;
    double geometry_variance = 0.0;
  };

  struct VoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;
    bool operator==(const VoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelHash {
    std::size_t operator()(const VoxelKey& key) const;
  };

  struct StoredScan {
    Epoch epoch;
    uint64_t sequence = 0;
    std::vector<Eigen::Vector3d> points;
    // Frozen points and the owning configuration never change before retirement.
    std::vector<VoxelKey> voxel_keys;
    LocalState anchor;
    double anchor_variance = 0.0;
  };

  VoxelKey Key(const Eigen::Vector3d& point) const;
  void Rebuild();
  Result BuildUpdate(const DeskewedScan& scan, const LocalState& state,
                     LioUpdate* update, bool tracking_gain = false) const;
  Result ApplyUpdate(const LioUpdate& update, const LocalState& prior,
                     LocalState* state,
                     Eigen::MatrixXd* applied_gain = nullptr,
                     const Matrix15d* tracking_prior = nullptr,
                     Matrix15d* tracking_posterior = nullptr) const;
  void Insert(const DeskewedScan& scan, const LocalState& state);

  LocalConfig config_;
  Epoch epoch_;
  std::deque<StoredScan> scans_;
  std::unordered_map<VoxelKey, Plane, VoxelHash> planes_;
  Eigen::MatrixXd observed_pose_basis_;
};

}  // namespace unified
}  // namespace localization
}  // namespace openloc
