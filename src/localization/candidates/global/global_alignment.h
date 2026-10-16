// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "candidates/global/global_types.h"
#include "candidates/local/local_motion.h"

namespace openloc {
namespace localization {
namespace unified {

class FixedLagGraph;

class GlobalAlignment {
 public:
  explicit GlobalAlignment(const GlobalConfig& config);
  ~GlobalAlignment();
  Result ValidateConfig() const;
  // Explicit oracle/prior for diagnostic ablations only; never a measurement.
  Result SeedDiagnostic(const Eigen::Isometry3d& map_to_odom,
                        const Matrix6d& covariance, double time);
  Result AddLocal(const LocalState& state,
                  const MotionIncrement* motion = nullptr);
  Result Lookup(double time, LocalState* state) const;
  Result Observe(const GlobalObservation& observation);
  Result Evaluate(double now) const;
  GlobalState Predict(double time) const;
  void Invalidate(Reason reason);
  const GlobalState& state() const { return state_; }
  const LocalState* latest_local() const;
  const std::map<std::string, SourceCounters>& sources() const {
    return sources_;
  }
  const GlobalConfig& config() const { return config_; }
  Reason reason() const { return reason_; }
  const std::map<std::string, GlobalEvidence>& evidence() const {
    return evidence_;
  }

 private:
  Result Reject(Reason reason, const std::string& message,
                SourceCounters* source);
  void ClearEpoch(const Epoch& epoch);
  Result ObservePartial(const GlobalObservation& observation,
                        const LocalState& local, SourceCounters* source);
  Result InitializeGraph(const GlobalObservation& observation,
                         const LocalState& local);
  void AdoptGraphState();
  GlobalConfig config_;
  GlobalState state_;
  std::deque<LocalState> history_;
  Eigen::Vector3d process_reference_in_odom_ = Eigen::Vector3d::Zero();
  std::map<std::string, SourceCounters> sources_;
  std::map<std::string, GlobalEvidence> evidence_;
  std::set<std::string> retired_sessions_;
  Eigen::Isometry3d candidate_ = Eigen::Isometry3d::Identity();
  Matrix6d candidate_covariance_ = Matrix6d::Zero();
  uint32_t candidate_frames_ = 0;
  double candidate_time_ = 0.0;
  bool candidate_georeferenced_ = false;
  Reason reason_ = Reason::GLOBAL_UNAVAILABLE;
  std::unique_ptr<FixedLagGraph> graph_;
  std::map<double, MotionIncrement> motion_history_;
};

}  // namespace unified
}  // namespace localization
}  // namespace openloc
