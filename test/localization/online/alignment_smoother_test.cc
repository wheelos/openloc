#include "alignment_smoother.h"

#include <cmath>
#include <iostream>
#include <string>

#include <Eigen/Geometry>

namespace openloc {
namespace localization {
namespace internal {
namespace {

bool Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
  }
  return condition;
}

bool TestRequiresInitialState() {
  AlignmentSmoother smoother;
  AlignmentEstimate estimate;
  return Expect(
      smoother.Estimate(&estimate).code() == StatusCode::kStateUnavailable,
      "estimate must fail before the first state");
}

bool TestReanchorReplacesOnlyAlignmentHistory() {
  AlignmentSmoother smoother;
  uint64_t first = 0;
  if (!Expect(smoother.AddState(1'000'000'000, 0.0, &first).ok(),
              "initial alignment state should be added")) {
    return false;
  }
  const Eigen::Vector4d candidate(12.0, -4.0, 0.0, 0.25);
  uint64_t reanchored = 0;
  if (!Expect(smoother
                  .Reanchor(2'000'000'000, candidate,
                            Eigen::Vector4d::Constant(0.5), &reanchored)
                  .ok(),
              "validated candidate should re-anchor global alignment") ||
      !Expect(reanchored > first,
              "alignment state IDs must remain monotonic across re-anchor") ||
      !Expect(smoother.Reanchor(2'000'000'000, candidate,
                                Eigen::Vector4d::Constant(0.5), &reanchored)
                      .code() == StatusCode::kInvalidArgument,
              "re-anchor must require a strictly newer state time")) {
    return false;
  }
  AlignmentEstimate estimate;
  if (!Expect(smoother.Estimate(&estimate).ok(),
              "re-anchored estimate should be available") ||
      !Expect(estimate.map_from_odom.isApprox(candidate, 1e-12),
              "new global prior should define the re-anchored estimate")) {
    return false;
  }
  GnssFactorDiagnostics diagnostics;
  if (!Expect(smoother.AddGnssPosition(
                          first, 1'000'000'000, Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity(),
                          &diagnostics)
                      .code() == StatusCode::kOutsideFixedLagWindow,
              "retired alignment state must no longer accept observations")) {
    return false;
  }
  uint64_t next = 0;
  return Expect(smoother.AddState(3'000'000'000, 1.0, &next).ok(),
                "new alignment chain should continue after re-anchor") &&
         Expect(next > reanchored, "new chain state IDs must stay monotonic");
}

bool TestStateAndGnssFactor() {
  AlignmentSmootherConfig config;
  config.initial_sigma = Eigen::Vector4d::Constant(1.0);
  config.random_walk_sigma_per_sqrt_meter = Eigen::Vector4d::Constant(0.1);
  AlignmentSmoother smoother(config);

  uint64_t state_id = 0;
  if (!Expect(smoother.AddState(1'000'000'000, 0.0, &state_id).ok(),
              "first alignment state should initialize")) {
    return false;
  }
  const Eigen::Vector3d antenna_in_odom = Eigen::Vector3d::Zero();
  const Eigen::Vector3d antenna_in_enu(0.0, 1.0, 0.0);
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 1e-4;
  GnssFactorDiagnostics diagnostics;
  if (!Expect(smoother
                  .AddGnssPosition(state_id, 1'000'000'000, antenna_in_odom,
                                   antenna_in_enu, covariance, &diagnostics)
                  .ok(),
              "GNSS position factor should be added")) {
    return false;
  }

  AlignmentEstimate estimate;
  if (!Expect(smoother.Estimate(&estimate).ok(),
              "alignment estimate should be available")) {
    return false;
  }
  if (!Expect(estimate.map_from_odom.allFinite(),
              "alignment estimate should be finite")) {
    return false;
  }
  if (!Expect(
          estimate.map_from_odom.y() > 0.9 && estimate.map_from_odom.y() < 1.1,
          "GNSS factor should correct map-from-odom translation")) {
    return false;
  }
  uint64_t next_state_id = 0;
  if (!Expect(smoother.AddState(1'100'000'000, 1.0, &next_state_id).ok(),
              "later alignment state should be added")) {
    return false;
  }
  if (!Expect(smoother.AddState(1'100'000'000, 0.0, &next_state_id).code() ==
                  StatusCode::kOutOfOrderTimestamp,
              "duplicate state time should be rejected")) {
    return false;
  }
  if (!Expect(smoother
                  .AddGnssPosition(state_id, 1'000'000'000, antenna_in_odom,
                                   antenna_in_enu, covariance, &diagnostics)
                  .ok(),
              "delayed GNSS must retain its original alignment state")) {
    return false;
  }
  return true;
}

bool TestGnssLeverArmConstrainsYaw() {
  AlignmentSmootherConfig config;
  config.initial_sigma = Eigen::Vector4d::Constant(10.0);
  AlignmentSmoother smoother(config);
  uint64_t state_id = 0;
  if (!Expect(smoother.AddState(2'000'000'000, 0.0, &state_id).ok(),
              "first alignment state should initialize")) {
    return false;
  }

  constexpr double kYaw = 0.2;
  const Eigen::Vector3d translation(0.5, -0.25, 0.1);
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(kYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d first_antenna_in_odom = Eigen::Vector3d::Zero();
  const Eigen::Vector3d second_antenna_in_odom(2.0, 0.0, 0.0);
  GnssFactorDiagnostics diagnostics;
  if (!Expect(
          smoother
              .AddGnssPosition(state_id, 2'000'000'000, first_antenna_in_odom,
                               translation, covariance, &diagnostics)
              .ok(),
          "first lever-arm observation should be accepted") ||
      !Expect(
          smoother
              .AddGnssPosition(state_id, 2'000'000'000, second_antenna_in_odom,
                               translation + rotation * second_antenna_in_odom,
                               covariance, &diagnostics)
              .ok(),
          "second lever-arm observation should be accepted")) {
    return false;
  }

  AlignmentEstimate estimate;
  if (!Expect(smoother.Estimate(&estimate).ok(),
              "lever-arm alignment estimate should be available")) {
    return false;
  }
  return Expect(std::abs(estimate.map_from_odom[3] - kYaw) < 0.05,
                "lever-arm observations should constrain alignment yaw");
}

bool TestHorizontalPositionPreservesUnobservedHeightAndRejectsBadEvidence() {
  AlignmentSmootherConfig config;
  config.initial_sigma = Eigen::Vector4d(10.0, 10.0, 3.0, 1.0);
  config.lag_seconds = 1.0;
  AlignmentSmoother smoother(config);
  uint64_t first = 0;
  if (!Expect(smoother.AddState(1'000'000'000, 0.0, &first).ok(),
              "XY test state must initialize")) {
    return false;
  }
  AlignmentEstimate before;
  if (!Expect(smoother.Estimate(&before).ok(), "initial estimate required")) {
    return false;
  }
  GnssFactorDiagnostics diagnostics;
  const Eigen::Vector3d antenna(0.0, 0.0, 1000.0);
  const Eigen::Vector2d measured(0.5, -0.25);
  const Eigen::Matrix2d covariance = Eigen::Matrix2d::Identity() * 0.01;
  if (!Expect(smoother
                  .AddGnssHorizontalPosition(first, 1'000'000'000, antenna,
                                             measured, covariance, &diagnostics)
                  .ok(),
              "measured XY should be admitted") ||
      !Expect(diagnostics.residual.size() == 2,
              "XY residual must have exactly two components")) {
    return false;
  }
  AlignmentEstimate after;
  if (!Expect(smoother.Estimate(&after).ok(), "XY estimate required") ||
      !Expect((after.map_from_odom.head<2>() - measured).norm() < 0.01,
              "XY should correct translation") ||
      !Expect(std::abs(after.map_from_odom.z() - before.map_from_odom.z()) <
                      1e-12 &&
                  std::abs(after.covariance(2, 2) - before.covariance(2, 2)) <
                      1e-12,
              "XY must neither measure Z nor reduce its uncertainty") ||
      !Expect(
          std::abs(after.covariance(3, 3) - before.covariance(3, 3)) < 1e-12,
          "single XY at zero horizontal lever must not observe yaw")) {
    return false;
  }
  const auto rejected = smoother.AddGnssHorizontalPosition(
      first, 1'000'000'000, antenna, Eigen::Vector2d(1000.0, 1000.0),
      covariance, &diagnostics);
  AlignmentEstimate retained;
  if (!Expect(rejected.code() == StatusCode::kRejectedMeasurement,
              "horizontal outlier must be rejected") ||
      !Expect(smoother.Estimate(&retained).ok() &&
                  retained.map_from_odom.isApprox(after.map_from_odom, 1e-12) &&
                  retained.covariance.isApprox(after.covariance, 1e-12),
              "rejected XY must preserve committed estimate") ||
      !Expect(smoother.AddGnssHorizontalPosition(
                          first, 1'000'000'000, antenna, measured,
                          Eigen::Matrix2d::Zero(), &diagnostics)
                      .code() == StatusCode::kInvalidArgument,
              "singular XY covariance must be rejected")) {
    return false;
  }
  uint64_t next = 0;
  return Expect(smoother.AddState(1'500'000'000, 1.0, &next).ok(),
                "advance within lag after rejected evidence") &&
         Expect(
             smoother
                 .AddGnssHorizontalPosition(first, 1'000'000'000, antenna,
                                            measured, covariance, &diagnostics)
                 .ok(),
             "delayed valid XY must recover within the lag") &&
         Expect(smoother.AddState(3'000'000'000, 1.0, &next).ok(),
                "advance beyond fixed lag") &&
         Expect(smoother.AddGnssHorizontalPosition(first, 1'000'000'000,
                                                   antenna, measured,
                                                   covariance, &diagnostics)
                        .code() == StatusCode::kOutsideFixedLagWindow,
                "expired XY must be rejected explicitly");
}

bool TestHorizontalGateUsesTwoMeasuredDimensions() {
  AlignmentSmootherConfig config;
  config.initial_sigma = Eigen::Vector4d::Ones();
  AlignmentSmoother smoother(config);
  uint64_t state = 0;
  GnssFactorDiagnostics diagnostics;
  return Expect(smoother.AddState(1'000'000'000, 0.0, &state).ok(),
                "horizontal gate test must initialize") &&
         Expect(
             smoother.AddGnssHorizontalPosition(
                         state, 1'000'000'000, Eigen::Vector3d::Zero(),
                         Eigen::Vector2d(std::sqrt(20.0), 0.0),
                         Eigen::Matrix2d::Identity(), &diagnostics)
                     .code() == StatusCode::kRejectedMeasurement,
             "NIS 10 must reject under the XY gate, not pass the XYZ gate") &&
         Expect(
             std::abs(diagnostics.normalized_innovation_squared - 10.0) < 1e-9,
             "horizontal NIS should use its two-dimensional innovation") &&
         Expect(
             diagnostics.horizontal_innovation_rejected,
             "horizontal outlier must be eligible for horizontal recovery") &&
         Expect(smoother.AddGnssPosition(
                            state, 1'000'000'000, Eigen::Vector3d::Zero(),
                            Eigen::Vector3d(0.1, 0.0, 100.0),
                            Eigen::Matrix3d::Identity(), &diagnostics)
                            .code() == StatusCode::kRejectedMeasurement &&
                    !diagnostics.horizontal_innovation_rejected,
                "Z-only innovation must not qualify for XY recovery");
}

bool TestRejectsInvalidNoiseAndMeasurement() {
  AlignmentSmootherConfig invalid_config;
  invalid_config.lag_seconds = 0.0;
  AlignmentSmoother invalid_smoother(invalid_config);
  uint64_t state_id = 0;
  if (!Expect(invalid_smoother.AddState(1'000'000'000, 0.0, &state_id).code() ==
                  StatusCode::kInvalidArgument,
              "invalid smoother config should be rejected")) {
    return false;
  }

  AlignmentSmoother smoother;
  if (!Expect(smoother.AddState(1'000'000'000, 0.0, &state_id).ok(),
              "first state should initialize")) {
    return false;
  }
  Eigen::Matrix3d singular = Eigen::Matrix3d::Zero();
  GnssFactorDiagnostics diagnostics;
  return Expect(
      smoother.AddGnssPosition(state_id, 1'000'000'000, Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero(), singular, &diagnostics)
              .code() == StatusCode::kInvalidArgument,
      "singular GNSS covariance should be rejected");
}

}  // namespace
}  // namespace internal
}  // namespace localization
}  // namespace openloc

int main() {
  using openloc::localization::internal::TestGnssLeverArmConstrainsYaw;
  using openloc::localization::internal::
      TestHorizontalGateUsesTwoMeasuredDimensions;
  using openloc::localization::internal::
      TestHorizontalPositionPreservesUnobservedHeightAndRejectsBadEvidence;
  using openloc::localization::internal::
      TestReanchorReplacesOnlyAlignmentHistory;
  using openloc::localization::internal::TestRejectsInvalidNoiseAndMeasurement;
  using openloc::localization::internal::TestRequiresInitialState;
  using openloc::localization::internal::TestStateAndGnssFactor;
  return TestRequiresInitialState() &&
                 TestReanchorReplacesOnlyAlignmentHistory() &&
                 TestStateAndGnssFactor() && TestGnssLeverArmConstrainsYaw() &&
                 TestHorizontalPositionPreservesUnobservedHeightAndRejectsBadEvidence() &&
                 TestHorizontalGateUsesTwoMeasuredDimensions() &&
                 TestRejectsInvalidNoiseAndMeasurement()
             ? 0
             : 1;
}
