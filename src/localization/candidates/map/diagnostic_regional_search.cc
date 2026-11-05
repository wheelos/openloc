// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "openloc/localization/map/diagnostic_regional_search.h"

#include <cmath>
#include <stdexcept>

#include "candidates/map/particle_hypothesis_tracker.h"

namespace openloc::localization::map {
namespace u = unified;
namespace {

u::ParticlePolicy SearchPolicy(const DiagnosticRegionConfig& config) {
  u::ParticlePolicy policy;
  policy.particles = 256;
  policy.maximum_scan_points = 100000;
  policy.maximum_groups = 128;
  policy.minimum_groups = 8;
  policy.maximum_modes = 8;
  policy.maximum_support_regions = 1;
  policy.group_resolution = 0.5;
  policy.minimum_observed_fraction = 0.25;
  policy.minimum_inlier_fraction = 0.25;
  policy.outlier_floor = 0.05;
  policy.score_temperature = 8;
  policy.resample_ess_fraction = 0.8;
  policy.mode_position_radius = 2;
  policy.mode_angle_radius = 0.35;
  policy.translation_diffusion = 0.1;
  policy.rotation_diffusion = 0.02;
  policy.maximum_interval = 0.5;
  policy.maximum_motion_translation = 5;
  policy.maximum_motion_rotation = 1;
  policy.maximum_query_age = 0.5;
  policy.future_tolerance = 0.01;
  policy.solve_budget_ms = 50;
  policy.distance_sigma = config.distance_sigma_m;
  policy.inlier_distance = config.inlier_distance_m;
  return policy;
}

}  // namespace

class DiagnosticRegionalSearch::Impl {
 public:
  Impl(const DiagnosticRegionConfig& config, const MapMetadata& metadata,
       const PointCloud& cloud)
      : tracker({64, 3, 0.2, 0.5, 0.5, 0.1, 0.5, 0.1}) {
    if (!metadata.integrity_verified || config.calibration_id.empty() ||
        !std::isfinite(config.yaw_minimum) ||
        !std::isfinite(config.yaw_maximum) ||
        config.yaw_minimum >= config.yaw_maximum ||
        config.yaw_minimum < -3.14159265358979323846 ||
        config.yaw_maximum > 3.14159265358979323846 ||
        !config.tilt.coeffs().allFinite() ||
        std::abs(config.tilt.norm() - 1) > 1e-9 ||
        !std::isfinite(config.distance_sigma_m) ||
        config.distance_sigma_m <= 0 ||
        !std::isfinite(config.inlier_distance_m) ||
        config.inlier_distance_m <= 0 ||
        metadata.reference_state != MapReferenceState::kDiagnosticOnly)
      throw std::invalid_argument(
          "regional search needs verified diagnostic map");
    u::FieldConfig field_config;
    field_config.identity = {metadata.map_id, metadata.version,
                             metadata.map_frame, config.calibration_id};
    field_config.minimum = config.field_minimum;
    field_config.maximum = config.field_maximum;
    field_config.support_regions = {{"declared-search-region",
                                     config.search_minimum,
                                     config.search_maximum}};
    field_config.observed_regions = {{"declared-map-coverage",
                                      config.observed_minimum,
                                      config.observed_maximum}};
    field_config.resolution = config.field_resolution_m;
    field_config.maximum_distance = 3;
    field_config.maximum_cells = 500000;
    field_config.maximum_map_points = 2000000;
    const auto built = u::LikelihoodField::Build(cloud, field_config, &field);
    if (!built.ok()) throw std::invalid_argument(built.message);
    if (config.distance_sigma_m < field->quantization_radius() ||
        config.inlier_distance_m <= field->quantization_radius() ||
        config.inlier_distance_m >= field_config.maximum_distance)
      throw std::invalid_argument(
          "regional score scales violate field quantization");
    domain.minimum = config.search_minimum;
    domain.maximum = config.search_maximum;
    domain.yaw_minimum = config.yaw_minimum;
    domain.yaw_maximum = config.yaw_maximum;
    domain.tilt = config.tilt;
    search =
        std::make_unique<u::ParticleRelocalizer>(SearchPolicy(config), field);
  }

  DiagnosticRegionResult Observe(const u::LocalState& state, uint64_t sequence,
                                 const ScanToMapRequest& request,
                                 ScanToMapLocalizer* matcher) {
    if (!matcher || !matcher->ready() || !request.deskewed ||
        request.clock_domain != state.stamp.clock_id ||
        request.timestamp_ns <= 0 ||
        state.stamp.time != request.timestamp_ns / 1e9)
      throw std::invalid_argument(
          "regional search needs ready matcher/deskewed clock");
    DiagnosticRegionResult output;
    output.attempt = attempts;
    if (!initialized) {
      if (attempts >= 10) {
        output.status = "attempt_budget_exhausted";
        output.reason = u::Reason::HISTORY_UNAVAILABLE;
        return output;
      }
      const auto result = search->Initialize(domain, state, 17 + attempts);
      output.attempt = ++attempts;
      if (!result.ok()) throw std::invalid_argument(result.message);
      initialized = true;
    }
    u::ParticleQuery query;
    query.identity = field->config().identity;
    query.local = state;
    query.source = "native-causal-map-search";
    query.scan_sequence = sequence;
    query.cloud = request.cloud;
    query.measurement_time_compensated = true;
    u::ParticleSnapshot particles;
    const auto searched = search->Update(query, &particles);
    if (!searched.ok()) {
      output.status = "search_rejected";
      output.message = searched.message;
      output.reason = searched.reason;
      search->Reset();
      tracker.Reset();
      initialized = false;
      return output;
    }
    output.modes = particles.modes.size();
    output.truncated = particles.modes_truncated;
    if (particles.modes_truncated) {
      output.status = "verification_incomplete";
      output.reason = u::Reason::GEOMETRY_UNAVAILABLE;
      return output;
    }
    auto verified = particles;
    verified.modes.clear();
    for (const auto& mode : particles.modes) {
      auto candidate = request;
      candidate.map_from_base_initial = mode.pose;
      const auto match = matcher->Match(candidate);
      output.candidates.push_back(match);
      if (!match.valid || !match.geometry_full_rank || match.ambiguous)
        continue;
      bool duplicate = false;
      for (const auto& existing : verified.modes) {
        if ((existing.pose.translation() - match.map_from_base.translation())
                    .norm() <= 0.5 &&
            u::LogRotation(
                Eigen::Quaterniond(existing.pose.linear().transpose() *
                                   match.map_from_base.linear()))
                    .norm() <= 0.1)
          duplicate = true;
      }
      if (!duplicate) {
        auto refined = mode;
        refined.pose = match.map_from_base;
        verified.modes.push_back(std::move(refined));
      }
    }
    output.verified_modes = verified.modes.size();
    if (verified.modes.empty()) {
      output.status = "verification_incomplete";
      output.reason = u::Reason::GEOMETRY_UNAVAILABLE;
      return output;
    }
    for (auto& mode : verified.modes)
      mode.search_mass = 1.0 / verified.modes.size();
    u::HypothesisTrackingSnapshot tracked;
    const auto tracking = tracker.Update(verified, state, &tracked);
    if (!tracking.ok()) {
      output.status = "tracking_rejected";
      output.reason = tracking.reason;
      tracker.Reset();
      return output;
    }
    const u::TrackedHypothesis* proposal = nullptr;
    for (const auto& track : tracked.tracks) {
      if (track.observed_this_update && track.consistency_span_satisfied) {
        ++output.stable_tracks;
        proposal = &track;
      }
    }
    const bool ready = output.stable_tracks == 1 &&
                       tracked.tracks.size() == 1 && verified.modes.size() == 1;
    output.status = ready ? "diagnostic_proposal" : "verifying";
    if (ready) output.map_from_base = proposal->map_base;
    return output;
  }

  std::shared_ptr<const u::LikelihoodField> field;
  u::ParticleDomain domain;
  std::unique_ptr<u::ParticleRelocalizer> search;
  u::ParticleHypothesisTracker tracker;
  bool initialized = false;
  uint32_t attempts = 0;
};

DiagnosticRegionalSearch::DiagnosticRegionalSearch(
    const DiagnosticRegionConfig& config, const MapMetadata& metadata,
    const PointCloud& cloud)
    : impl_(std::make_unique<Impl>(config, metadata, cloud)) {}
DiagnosticRegionalSearch::~DiagnosticRegionalSearch() = default;
DiagnosticRegionResult DiagnosticRegionalSearch::Observe(
    const u::LocalState& state, uint64_t sequence,
    const ScanToMapRequest& request, ScanToMapLocalizer* matcher) {
  return impl_->Observe(state, sequence, request, matcher);
}
void DiagnosticRegionalSearch::Restart() {
  impl_->search->Reset();
  impl_->tracker.Reset();
  impl_->initialized = false;
  impl_->attempts = 0;
}

}  // namespace openloc::localization::map
