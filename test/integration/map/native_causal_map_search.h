// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>

#include "candidates/map/likelihood_field.h"
#include "openloc/localization/map/diagnostic_regional_search.h"
#include "test/integration/map/replay_csv.h"

namespace openloc::replay {

// Replay owns policy I/O and journals; the shared public component owns search.
class NativeCausalMapSearch {
 public:
  using State = localization::unified::LocalState;
  NativeCausalMapSearch(const std::filesystem::path& policy_path,
                       const std::filesystem::path& output,
                       const localization::unified::FieldIdentity& identity,
                       const localization::map::PointCloud& cloud)
      : journal_(output / "causal_search.csv"),
        candidates_(output / "causal_candidates.csv") {
    if (!journal_ || !candidates_)
      throw std::runtime_error("cannot open causal search journal");
    journal_ << "sequence,reference_ns,status,reason,modes,verified_modes,"
                "truncated,stable_tracks,attempt,diagnostic_only,generation\n";
    candidates_ << std::setprecision(17)
                << "sequence,reference_ns,mode,valid,full_rank,ambiguous,"
                   "reason,detail,source_overlap,target_overlap,score,"
                   "x,y,z,qx,qy,qz,qw\n";
    std::map<std::string, double> values;
    for (const auto& row : ReadRows(policy_path.string(), "name,value", 2))
      if (!values.emplace(row[0], Number(row[1])).second)
        throw std::runtime_error("duplicate causal search policy key");
    auto take = [&values](const std::string& name) {
      const auto found = values.find(name);
      if (found == values.end())
        throw std::runtime_error("missing causal search policy: " + name);
      const double value = found->second;
      values.erase(found);
      return value;
    };
    auto vector = [&take](const std::string& prefix) {
      Eigen::Vector3d value;
      for (int axis = 0; axis < 3; ++axis)
        value[axis] = take(prefix + std::string(1, "xyz"[axis]));
      return value;
    };
    localization::map::DiagnosticRegionConfig policy;
    policy.calibration_id = identity.calibration;
    policy.field_minimum = vector("field_min_");
    policy.field_maximum = vector("field_max_");
    policy.search_minimum = vector("search_min_");
    policy.search_maximum = vector("search_max_");
    policy.observed_minimum = vector("observed_min_");
    policy.observed_maximum = vector("observed_max_");
    policy.yaw_minimum = take("yaw_min_rad");
    policy.yaw_maximum = take("yaw_max_rad");
    const double qx = take("tilt_qx"), qy = take("tilt_qy");
    const double qz = take("tilt_qz"), qw = take("tilt_qw");
    policy.tilt = Eigen::Quaterniond(qw, qx, qy, qz);
    policy.field_resolution_m = take("field_resolution_m");
    policy.distance_sigma_m = take("distance_sigma_m");
    policy.inlier_distance_m = take("inlier_distance_m");
    if (!values.empty()) throw std::runtime_error("unknown causal search policy key");
    localization::map::MapMetadata metadata;
    metadata.map_id = identity.map_id;
    metadata.version = identity.version;
    metadata.map_frame = identity.frame;
    metadata.reference_state = localization::map::MapReferenceState::kDiagnosticOnly;
    metadata.integrity_verified = true;
    search_ = std::make_unique<localization::map::DiagnosticRegionalSearch>(
        policy, metadata, cloud);
  }

  std::optional<Eigen::Isometry3d> Observe(
      const State& state, uint64_t sequence, int64_t stamp,
      const localization::map::ScanToMapRequest& request,
      localization::map::ScanToMapLocalizer* matcher) {
    const auto result = search_->Observe(state, sequence, request, matcher);
    size_t index = 0;
    for (const auto& match : result.candidates) {
      candidates_ << sequence << ',' << stamp << ',' << ++index << ','
                  << match.valid << ',' << match.geometry_full_rank << ','
                  << match.ambiguous << ',';
      Text(candidates_, match.reason);
      candidates_ << ',';
      Text(candidates_, match.detail);
      candidates_ << ',' << match.source_overlap << ',' << match.target_overlap
                  << ',' << match.score;
      for (int axis = 0; axis < 3; ++axis)
        candidates_ << ',' << match.map_from_base.translation()[axis];
      const Eigen::Quaterniond orientation(match.map_from_base.linear());
      for (int axis = 0; axis < 4; ++axis)
        candidates_ << ',' << orientation.coeffs()[axis];
      candidates_ << '\n';
      if (!candidates_) throw std::runtime_error("incomplete causal candidate journal");
    }
    journal_ << sequence << ',' << stamp << ',' << result.status << ','
             << localization::unified::ReasonName(result.reason) << ','
             << result.modes << ',' << result.verified_modes << ','
             << result.truncated << ',' << result.stable_tracks << ','
             << result.attempt << ",1," << generation_ << '\n';
    journal_.flush();
    if (!journal_) throw std::runtime_error("incomplete causal search journal");
    return result.map_from_base;
  }
  void Finish() {
    journal_.flush();
    candidates_.flush();
    if (!journal_ || !candidates_) throw std::runtime_error("incomplete causal search journal");
  }
  void Restart() { search_->Restart(); ++generation_; }
  uint64_t generation() const { return generation_; }

 private:
  std::ofstream journal_;
  std::ofstream candidates_;
  std::unique_ptr<localization::map::DiagnosticRegionalSearch> search_;
  uint64_t generation_ = 0;
};

}  // namespace openloc::replay
