// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <memory>

#include "candidates/global/global_types.h"
#include "candidates/local/local_motion.h"
#include "openloc/localization/global/observation_types.h"

namespace gtsam {
class GaussianConditional;
}

namespace openloc {
namespace localization {
namespace unified {

Result ConditionalRowSigmas(const gtsam::GaussianConditional& conditional,
                            Eigen::VectorXd* sigmas);

struct FixedLagConfig {
  double lag = 3.0;
  double solve_budget_ms = 50.0;
  double innovation_gate = 22.458;
  double robust_threshold = 2.5;
  uint32_t max_states = 1000;
  uint32_t max_observations = 2000;
  uint32_t max_error_dimension = 256;
  uint32_t max_iterations = 5;
  double max_linearization_angle = 0.2;
};

struct GraphFactorEvaluation {
  std::string type;
  std::vector<uint64_t> keys;
  Eigen::VectorXd residual;
  double error = 0;
  double weight = 1;
};

struct GraphEvaluationSnapshot {
  double cutoff_time = 0;
  uint64_t retained_nodes = 0;
  uint64_t archived_nodes = 0;
  uint64_t retained_relinearizations = 0;
  Eigen::Isometry3d map_from_base = Eigen::Isometry3d::Identity();
  // [translation_map metres, left_rotation_map radians], not alignment bound.
  Matrix6d pose_covariance = Matrix6d::Zero();
  double objective = 0;
  uint32_t iterations = 0;
  double maximum_exact_residual = 0;
  std::vector<GraphFactorEvaluation> factors;
};

struct GraphResourceUsage {
  uint64_t retained_nodes = 0;
  uint64_t retained_observations = 0;
  uint64_t factor_slots = 0;
  uint64_t active_factors = 0;
  uint64_t archived_values = 0;
  uint32_t solve_iterations = 0;
  uint32_t damped_iterations = 0;
  double maximum_motion_residual = 0;
};

// All state here, including the nuisance error chain, is global-owned.
// There is intentionally no output carrying local state/bias corrections.
class FixedLagGraph {
 public:
  explicit FixedLagGraph(const FixedLagConfig& config);
  ~FixedLagGraph();
  Result ValidateConfig() const;
  static Result ValidateConditionalObservation(
      const LocalState& local, const GlobalObservation& observation);
  // The caller must first verify a full-pose hypothesis with either independent
  // noise or a qualified conditional joint model against the same local error.
  Result Initialize(const LocalState& local, const GlobalObservation& anchor,
                    const std::vector<LocalState>& following = {},
                    const std::vector<MotionIncrement>& motions = {});
  Result AddLocal(const LocalState& local, const MotionIncrement& motion);
  // Only exact retained measurement times; never use a nearest keyframe.
  // Point factors constrain only the supplied XY/XYZ antenna coordinates.
  // Partial observations neither bootstrap nor renew the full-pose lease.
  Result Observe(const GlobalObservation& observation,
                 ObservationInnovation* innovation = nullptr);
  bool initialized() const;
  const GlobalState& state() const;
  GraphResourceUsage ResourceUsage() const;

  // Offline evaluation only. Opt in before initialization; archives the exact
  // committed factors/initial values without eviction. Not a production mode.
  Result EnableEvaluationHistory();
  // Batch solves the archived factors with no marginalization and the same
  // cutoff/admission set. Does not change this owner's state or decisions.
  Result EvaluateForTesting(bool batch, GraphEvaluationSnapshot* output) const;

 private:
  struct Impl;
  FixedLagConfig config_;
  std::unique_ptr<Impl> impl_;
  bool retain_evaluation_history_ = false;
};

}  // namespace unified
}  // namespace localization
}  // namespace openloc
