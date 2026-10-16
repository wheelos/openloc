#include "openloc/localization/online/localization_engine.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace openloc {
namespace localization {
namespace {

constexpr int64_t kStartTimeNs = 1'000'000'000LL;

bool Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
  }
  return condition;
}

Timestamp Time(int64_t timestamp_ns) {
  return Timestamp{timestamp_ns, ClockDomain::kUnixEpoch};
}

MapReference SyntheticMapReference() {
  MapReference reference{"site", "v1", "calibration"};
  reference.geodetic.state = GeodeticReferenceState::kVerified;
  reference.geodetic.horizontal_crs_realization = "TEST:synthetic-crs";
  reference.geodetic.coordinate_epoch_decimal_year = 2026.0;
  reference.geodetic.vertical_datum = "TEST:synthetic-vertical-datum";
  reference.geodetic.enu_origin_latitude_deg = 0.0;
  reference.geodetic.enu_origin_longitude_deg = 0.0;
  reference.geodetic.enu_origin_height_m = 0.0;
  return reference;
}

OdometrySample Odometry(int64_t timestamp_ns, double x,
                        OdometryEpoch epoch = {"odometry", 1}) {
  OdometrySample sample;
  sample.timestamp = Time(timestamp_ns);
  sample.epoch = epoch;
  sample.parent_frame = FrameId{"odom"};
  sample.child_frame = FrameId{"base_link"};
  sample.pose.translation.x() = x;
  return sample;
}

GnssFix Fix(int64_t timestamp_ns, const Eigen::Vector3d& position) {
  GnssFix fix;
  fix.timestamp = Time(timestamp_ns);
  fix.frame = FrameId{"map"};
  fix.map_reference = SyntheticMapReference();
  fix.position = position;
  fix.covariance = Eigen::Matrix3d::Identity() * 0.01;
  fix.receiver.carrier_solution = GnssCarrierSolution::kFixed;
  fix.receiver.satellite_count = 20;
  fix.receiver.hdop = 0.8;
  fix.receiver.differential_age_seconds = 0.2;
  fix.receiver.horizontal_accuracy_m = 0.03;
  return fix;
}

GnssHorizontalFix HorizontalFix(int64_t timestamp_ns,
                                const Eigen::Vector2d& position) {
  const GnssFix fixture = Fix(timestamp_ns, Eigen::Vector3d::Zero());
  return {fixture.timestamp,
          fixture.frame,
          fixture.map_reference,
          position,
          fixture.covariance.topLeftCorner<2, 2>(),
          fixture.receiver};
}

GnssHeading Heading(int64_t timestamp_ns, double azimuth_rad) {
  GnssHeading heading;
  heading.timestamp = Time(timestamp_ns);
  heading.frame = FrameId{"map"};
  heading.map_reference = SyntheticMapReference();
  heading.azimuth_rad = azimuth_rad;
  heading.accuracy_rad = 0.01;
  heading.measured_baseline_length_m = 1.0;
  heading.receiver.carrier_solution = GnssCarrierSolution::kFixed;
  heading.receiver.satellite_count = 20;
  heading.receiver.hdop = 0.8;
  heading.receiver.differential_age_seconds = 0.2;
  heading.receiver.horizontal_accuracy_m = 0.03;
  return heading;
}

LocalizationConfig TestConfig() {
  LocalizationConfig config;
  config.map_reference = SyntheticMapReference();
  config.gnss_baseline_in_base = Eigen::Vector3d(1.0, 0.0, 0.0);
  config.fixed_solution_stability_ns = 0;
  config.minimum_heading_admission_interval_ns = 0;
  config.alignment_segment_distance_m = 0.5;
  config.alignment_segment_max_duration_ns = 500'000'000LL;
  config.odometry_buffer.maximum_interpolation_gap_ns = 200'000'000LL;
  return config;
}

bool TestDualAntennaHeadingInitializationAndGate() {
  LocalizationEngine engine;
  LocalizationConfig config = TestConfig();
  if (!Expect(engine.Configure(config).ok(),
              "engine should accept a configured dual-antenna baseline") ||
      !Expect(engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok(),
              "first odometry should initialize heading test")) {
    return false;
  }

  const double expected_heading = 0.7;
  Status status = engine.SubmitGnssHeading(
      Heading(kStartTimeNs + 50'000'000LL, expected_heading));
  if (!Expect(status.code() == StatusCode::kPending,
              "heading newer than odometry should await its right bracket") ||
      !Expect(
          engine.AddOdometry(Odometry(kStartTimeNs + 100'000'000LL, 0.0)).ok(),
          "second odometry should process pending heading")) {
    return false;
  }
  LocalizationResult result;
  if (!Expect(engine.GetResult(&result).ok() && result.heading_initialized,
              "accepted heading should set heading_initialized") ||
      !Expect(!result.global_alignment_valid,
              "heading alone must not initialize global position") ||
      !Expect(std::abs(result.map_from_odom[3] - expected_heading) < 0.02,
              "dual-antenna factor should initialize map-to-odom yaw") ||
      !Expect(
          !result.factor_records.empty() &&
              result.factor_records.back().source == "gnss_heading" &&
              result.factor_records.back().type == "ABS_YAW1" &&
              result.factor_records.back().residual.size() == 1 &&
              result.factor_records.size() == 1,
          "pending heading should resolve to one scalar yaw factor record")) {
    return false;
  }

  GnssHeading inconsistent = Heading(kStartTimeNs + 70'000'000LL, -2.0);
  status = engine.SubmitGnssHeading(inconsistent);
  if (!Expect(status.code() == StatusCode::kRejectedMeasurement,
              "large heading innovation should be rejected") ||
      !Expect(engine.GetResult(&result).ok() &&
                  result.factor_records.back().source == "gnss_heading" &&
                  !result.factor_records.back().accepted,
              "rejected heading should remain visible in factor diagnostics")) {
    return false;
  }

  GnssHeading bad_baseline = Heading(kStartTimeNs + 80'000'000LL, 0.7);
  bad_baseline.measured_baseline_length_m = 1.1;
  status = engine.SubmitGnssHeading(bad_baseline);
  if (!Expect(status.code() == StatusCode::kRejectedMeasurement,
              "measured baseline mismatch should reject heading")) {
    return false;
  }
  GnssHeading inaccurate = Heading(kStartTimeNs + 90'000'000LL, 0.7);
  inaccurate.accuracy_rad = 0.1;
  status = engine.SubmitGnssHeading(inaccurate);
  if (!Expect(status.code() == StatusCode::kRejectedMeasurement,
              "heading accuracy outside policy should reject heading")) {
    return false;
  }
  GnssHeading float_solution = Heading(kStartTimeNs + 100'000'000LL, 0.7);
  float_solution.receiver.carrier_solution = GnssCarrierSolution::kFloat;
  status = engine.SubmitGnssHeading(float_solution);
  return Expect(status.code() == StatusCode::kRejectedMeasurement,
                "non-fixed carrier solution should reject heading");
}

bool TestConfigurationAndLifecycle() {
  LocalizationEngine engine;
  LocalizationResult result;
  if (!Expect(engine.GetResult(&result).code() == StatusCode::kStateUnavailable,
              "unconfigured engine must not return an estimate")) {
    return false;
  }
  LocalizationConfig invalid = TestConfig();
  invalid.alignment_segment_distance_m = 0.0;
  if (!Expect(engine.Configure(invalid).code() == StatusCode::kInvalidArgument,
              "invalid config must fail explicitly")) {
    return false;
  }
  if (!Expect(engine.Configure(TestConfig()).ok(),
              "valid config should configure engine")) {
    return false;
  }
  if (!Expect(
          engine.Configure(TestConfig()).code() == StatusCode::kInvalidArgument,
          "configuration must be immutable after initialization")) {
    return false;
  }
  if (!Expect(engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok(),
              "first odometry sample should initialize the backend")) {
    return false;
  }
  if (!Expect(engine.GetResult(&result).ok() &&
                  result.timestamp.nanoseconds == kStartTimeNs &&
                  result.alignment_covariance.allFinite() &&
                  !result.global_alignment_valid,
              "estimate should include a finite covariance and sensor time")) {
    return false;
  }
  if (!Expect(engine.Reset().ok(), "explicit reset should succeed")) {
    return false;
  }
  if (!Expect(engine.GetResult(&result).code() == StatusCode::kStateUnavailable,
              "reset must clear the old estimate")) {
    return false;
  }
  return true;
}

bool TestPositionOnlyConfigurationNeedsNoHeadingBaseline() {
  LocalizationConfig config = TestConfig();
  config.gnss_baseline_in_base.setZero();
  LocalizationEngine engine;
  if (!Expect(engine.Configure(config).ok(),
              "position-only GNSS must not require a heading baseline") ||
      !Expect(engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok(),
              "initial odometry should be accepted") ||
      !Expect(
          engine.SubmitGnssFix(Fix(kStartTimeNs, Eigen::Vector3d::Zero())).ok(),
          "absolute GNSS position should initialize global alignment")) {
    return false;
  }
  LocalizationResult result;
  return Expect(engine.GetResult(&result).ok() && result.global_alignment_valid,
                "position-only alignment should become globally valid");
}

bool TestHorizontalFixSharesAdmissionWithoutInventingHeight() {
  LocalizationEngine engine;
  LocalizationConfig config = TestConfig();
  config.gnss_antenna_in_base = Eigen::Vector3d(0.0, 0.0, 3.0);
  config.cross_check_absolute_distance_error_m = 2000.0;
  config.minimum_gnss_admission_interval_ns = 0;
  OdometrySample first = Odometry(kStartTimeNs, 0.0);
  first.pose.translation.z() = 7.0;
  if (!Expect(engine.Configure(config).ok() && engine.AddOdometry(first).ok(),
              "initialize horizontal engine with nonzero body/antenna Z")) {
    return false;
  }
  LocalizationResult before;
  if (!Expect(engine.GetResult(&before).ok(),
              "read pre-observation Z variance")) {
    return false;
  }
  const int64_t fix_time = kStartTimeNs + 50'000'000LL;
  GnssHorizontalFix fix = HorizontalFix(fix_time, Eigen::Vector2d(2.05, 0.1));
  GnssHorizontalFix invalid = fix;
  invalid.covariance(0, 0) = -1.0;
  if (!Expect(engine.SubmitGnssHorizontalFix(invalid).code() ==
                  StatusCode::kInvalidArgument,
              "horizontal covariance must be positive definite") ||
      !Expect(
          engine.SubmitGnssHorizontalFix(fix).code() == StatusCode::kPending,
          "horizontal fix must await sensor-time ODOM bracket")) {
    return false;
  }
  OdometrySample second = Odometry(kStartTimeNs + 100'000'000LL, 0.1);
  second.pose.translation.z() = 7.0;
  LocalizationResult result;
  if (!Expect(engine.AddOdometry(second).ok() && engine.GetResult(&result).ok(),
              "resolve pending horizontal observation") ||
      !Expect(result.global_alignment_valid &&
                  !result.vertical_position_initialized &&
                  std::abs(result.backend_map_from_odom.z()) < 1e-10 &&
                  std::abs(result.alignment_covariance(2, 2) -
                           before.alignment_covariance(2, 2)) < 1e-6,
              "XY must neither initialize Z nor reduce its variance") ||
      !Expect(
          result.factor_records.back().type == "ABS_POS2" &&
              result.factor_records.back().accepted &&
              result.factor_records.back().residual.size() == 2 &&
              result.factor_records.back().covariance_diagonal.size() == 2 &&
              result.factor_records.back().timestamp_ns == fix_time &&
              result.factor_records.back().innovation_threshold ==
                  config.horizontal_innovation_gate_threshold,
          "factor ledger must retain exact time and measured XY shape")) {
    return false;
  }
  if (!Expect(
          engine.SubmitGnssFix(Fix(fix_time, Eigen::Vector3d(2.05, 0.1, 10.0)))
                  .code() == StatusCode::kOutOfOrderTimestamp,
          "XYZ and XY must not admit two representations of one epoch")) {
    return false;
  }
  for (int index = 2; index <= 3; ++index) {
    const int64_t timestamp = kStartTimeNs + index * 100'000'000LL;
    OdometrySample sample = Odometry(timestamp, index * 0.1);
    sample.pose.translation.z() = 7.0;
    GnssHorizontalFix bad = HorizontalFix(timestamp, Eigen::Vector2d(1000, 0));
    if (index == 2) {
      bad.receiver.carrier_solution = GnssCarrierSolution::kFloat;
    }
    if (!Expect(engine.AddOdometry(sample).ok() &&
                    engine.SubmitGnssHorizontalFix(bad).code() ==
                        StatusCode::kRejectedMeasurement &&
                    engine.GetResult(&result).ok() &&
                    !result.factor_records.back().accepted &&
                    result.factor_records.back().type == "ABS_POS2",
                "XY must share receiver quality and innovation rejection")) {
      return false;
    }
    if (index == 3 &&
        !Expect(result.factor_records.back().normalized_innovation_squared >
                    config.horizontal_innovation_gate_threshold,
                "horizontal outlier must reach the 2D innovation gate")) {
      return false;
    }
  }
  return true;
}

bool TestHorizontalFixPreservesPreviouslyObservedHeight() {
  LocalizationEngine engine;
  if (!Expect(engine.Configure(TestConfig()).ok() &&
                  engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok() &&
                  engine
                      .SubmitGnssFix(
                          Fix(kStartTimeNs, Eigen::Vector3d(0.0, 0.0, 5.0)))
                      .ok(),
              "initialize height from a real XYZ observation")) {
    return false;
  }
  LocalizationResult before;
  LocalizationResult after;
  return Expect(
      engine.GetResult(&before).ok() &&
          engine.AddOdometry(Odometry(kStartTimeNs + 1'000'000'000LL, 1.0))
              .ok() &&
          engine
              .SubmitGnssHorizontalFix(HorizontalFix(
                  kStartTimeNs + 1'000'000'000LL, Eigen::Vector2d(1.0, 0.0)))
              .ok() &&
          engine.GetResult(&after).ok() &&
          after.vertical_position_initialized &&
          std::abs(after.backend_map_from_odom.z() -
                   before.backend_map_from_odom.z()) < 1e-8 &&
          after.alignment_covariance(2, 2) >=
              before.alignment_covariance(2, 2) - 1e-8,
      "switching XYZ to XY must not fabricate another height factor");
}

bool TestPendingFixUsesSensorTimestampAndBracket() {
  LocalizationConfig config = TestConfig();
  config.gnss_antenna_in_base = Eigen::Vector3d(0.0, 1.0, 0.0);
  LocalizationEngine engine;
  if (!Expect(engine.Configure(config).ok(), "engine configuration failed") ||
      !Expect(engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok(),
              "initial odometry should be accepted")) {
    return false;
  }

  const int64_t fix_time_ns = kStartTimeNs + 50'000'000LL;
  if (!Expect(engine.SubmitGnssFix(Fix(fix_time_ns, {0.25, 0.9, 0.0})).code() ==
                  StatusCode::kPending,
              "fix newer than odometry must wait for a right-hand bracket")) {
    return false;
  }
  if (!Expect(
          engine.AddOdometry(Odometry(kStartTimeNs + 100'000'000LL, 0.1)).ok(),
          "right-hand odometry sample should process the pending fix")) {
    return false;
  }
  LocalizationResult result;
  if (!Expect(engine.GetResult(&result).ok() &&
                  result.factor_records.size() == 1 &&
                  result.factor_records.front().accepted &&
                  result.factor_records.front().timestamp_ns == fix_time_ns,
              "pending fix record should resolve once at sensor time")) {
    return false;
  }
  if (!Expect(std::abs(result.map_from_odom.x() - 0.2) < 0.05 &&
                  std::abs(result.map_from_odom.y() + 0.1) < 0.05,
              "antenna lever arm and interpolated odometry must be applied")) {
    return false;
  }
  return true;
}

bool TestFirstQualifiedFixInitializesGlobalAlignment() {
  LocalizationEngine engine;
  if (!Expect(engine.Configure(TestConfig()).ok(),
              "engine configuration failed") ||
      !Expect(engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok(),
              "initial odometry should be accepted")) {
    return false;
  }
  if (!Expect(engine
                  .SubmitGnssFix(
                      Fix(kStartTimeNs, Eigen::Vector3d(100.0, 20.0, 0.0)))
                  .ok(),
              "first quality-approved GNSS fix should initialize alignment")) {
    return false;
  }
  LocalizationResult result;
  if (!Expect(
          engine.GetResult(&result).ok() && result.reset_event &&
              result.reset_counter == 1 && result.global_alignment_valid &&
              (result.map_from_odom.head<3>() -
               Eigen::Vector3d(100.0, 20.0, 0.0))
                      .norm() < 0.1 &&
              result.factor_records.back().reason == "accepted_initialization",
          "initial global alignment should be seeded and diagnosed")) {
    return false;
  }
  return true;
}

bool TestGnssUpdateLeavesOdometryInputUnchanged() {
  LocalizationEngine engine;
  if (!Expect(engine.Configure(TestConfig()).ok(),
              "engine configuration failed")) {
    return false;
  }
  OdometrySample odometry = Odometry(kStartTimeNs, 1.0);
  odometry.pose.rotation =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  const OdometrySample original = odometry;
  if (!Expect(engine.AddOdometry(odometry).ok(),
              "initial odometry should be accepted") ||
      !Expect(engine
                  .SubmitGnssFix(
                      Fix(kStartTimeNs, Eigen::Vector3d(100.0, 20.0, 0.0)))
                  .ok(),
              "qualified GNSS should initialize global alignment")) {
    return false;
  }

  LocalizationResult result;
  return Expect(
      engine.GetResult(&result).ok() && result.global_alignment_valid &&
          std::abs(result.map_from_odom.x() - 99.0) < 0.1 &&
          odometry.timestamp.nanoseconds == original.timestamp.nanoseconds &&
          odometry.timestamp.clock_domain == original.timestamp.clock_domain &&
          odometry.epoch == original.epoch &&
          odometry.parent_frame.value == original.parent_frame.value &&
          odometry.child_frame.value == original.child_frame.value &&
          odometry.pose.translation.isApprox(original.pose.translation, 0.0) &&
          odometry.pose.rotation.coeffs().isApprox(
              original.pose.rotation.coeffs(), 0.0),
      "GNSS alignment must not modify the caller-owned odometry sample");
}

bool TestQualityAndInnovationRejectionsAreRecorded() {
  LocalizationEngine engine;
  LocalizationConfig config = TestConfig();
  if (!Expect(engine.Configure(config).ok(), "engine configuration failed") ||
      !Expect(engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok(),
              "initial odometry should be accepted")) {
    return false;
  }

  GnssFix low_quality = Fix(kStartTimeNs, Eigen::Vector3d::Zero());
  low_quality.receiver.satellite_count = 4;
  if (!Expect(engine.SubmitGnssFix(low_quality).code() ==
                  StatusCode::kRejectedMeasurement,
              "low satellite count should fail L0 quality checks")) {
    return false;
  }
  if (!Expect(
          engine.AddOdometry(Odometry(kStartTimeNs + 100'000'000LL, 0.0)).ok(),
          "odometry should advance to the valid fix time")) {
    return false;
  }
  if (!Expect(engine
                  .SubmitGnssFix(Fix(kStartTimeNs + 100'000'000LL,
                                     Eigen::Vector3d::Zero()))
                  .ok(),
              "valid fixed solution should be accepted")) {
    return false;
  }
  for (int step = 2; step <= 7; ++step) {
    if (!Expect(
            engine
                .AddOdometry(Odometry(
                    kStartTimeNs + static_cast<int64_t>(step) * 100'000'000LL,
                    0.0))
                .ok(),
            "stationary odometry history should remain valid")) {
      return false;
    }
  }
  const Status biased_status = engine.SubmitGnssFix(
      Fix(kStartTimeNs + 700'000'000LL, Eigen::Vector3d(3.0, 0.0, 0.0)));
  if (!Expect(biased_status.code() == StatusCode::kRejectedMeasurement,
              "3 m stationary GNSS bias should be rejected by cross-check")) {
    return false;
  }

  LocalizationResult result;
  if (!Expect(
          engine.GetResult(&result).ok() && result.factor_records.size() == 3 &&
              result.factor_records[0].reason == "insufficient_satellites" &&
              result.factor_records[1].accepted &&
              result.global_alignment_valid &&
              result.factor_records[2].reason ==
                  "gnss_odometry_distance_disagreement",
          "each GNSS decision should have a structured diagnostic")) {
    return false;
  }
  return Expect(result.alignment_covariance.allFinite(),
                "published alignment covariance must remain finite");
}

bool TestMissingReceiverQualityMetricsAreRejected() {
  LocalizationEngine engine;
  if (!Expect(engine.Configure(TestConfig()).ok(),
              "engine configuration failed")) {
    return false;
  }
  GnssFix fix = Fix(kStartTimeNs, Eigen::Vector3d::Zero());
  fix.receiver.hdop = 0.0;
  if (!Expect(engine.SubmitGnssFix(fix).code() == StatusCode::kInvalidArgument,
              "defaulted zero HDOP must not pass receiver validation")) {
    return false;
  }
  fix = Fix(kStartTimeNs, Eigen::Vector3d::Zero());
  fix.receiver.horizontal_accuracy_m = 0.0;
  if (!Expect(engine.SubmitGnssFix(fix).code() == StatusCode::kInvalidArgument,
              "defaulted zero receiver accuracy must not pass validation")) {
    return false;
  }
  GnssHeading heading = Heading(kStartTimeNs, 0.0);
  heading.receiver.hdop = 0.0;
  if (!Expect(engine.SubmitGnssHeading(heading).code() ==
                  StatusCode::kInvalidArgument,
              "heading with defaulted zero HDOP must be rejected")) {
    return false;
  }
  heading = Heading(kStartTimeNs, 0.0);
  heading.receiver.horizontal_accuracy_m = 0.0;
  return Expect(
      engine.SubmitGnssHeading(heading).code() == StatusCode::kInvalidArgument,
      "heading with defaulted receiver accuracy must be rejected");
}

bool TestMapReferenceMustMatchConfiguredMap() {
  LocalizationEngine engine;
  LocalizationConfig config = TestConfig();
  config.map_reference.version.clear();
  if (!Expect(engine.Configure(config).code() == StatusCode::kInvalidArgument,
              "map frame configuration must identify its origin/version")) {
    return false;
  }
  config = TestConfig();
  if (!Expect(engine.Configure(config).ok(), "engine configuration failed")) {
    return false;
  }
  GnssFix fix = Fix(kStartTimeNs, Eigen::Vector3d::Zero());
  fix.map_reference.version = "v2";
  if (!Expect(
          engine.SubmitGnssFix(fix).code() == StatusCode::kMapReferenceMismatch,
          "GNSS position from another map version must be rejected")) {
    return false;
  }
  GnssHeading heading = Heading(kStartTimeNs, 0.0);
  heading.map_reference.id = "another-site";
  if (!Expect(engine.SubmitGnssHeading(heading).code() ==
                  StatusCode::kMapReferenceMismatch,
              "GNSS heading from another map origin must be rejected")) {
    return false;
  }
  GnssFix fix_with_different_datum = Fix(kStartTimeNs, Eigen::Vector3d::Zero());
  fix_with_different_datum.map_reference.geodetic.vertical_datum =
      "TEST:other-vertical-datum";
  if (!Expect(engine.SubmitGnssFix(fix_with_different_datum).code() ==
                  StatusCode::kMapReferenceMismatch,
              "GNSS position with another vertical datum must be rejected")) {
    return false;
  }
  heading = Heading(kStartTimeNs, 0.0);
  heading.map_reference.calibration_id = "different-extrinsics";
  return Expect(engine.SubmitGnssHeading(heading).code() ==
                    StatusCode::kMapReferenceMismatch,
                "GNSS heading with another calibration must be rejected");
}

bool TestGeodeticReferenceConfigurationIsFailClosed() {
  LocalizationConfig config = TestConfig();
  config.map_reference.geodetic.state = GeodeticReferenceState::kUnspecified;
  if (!Expect(LocalizationEngine().Configure(config).code() ==
                  StatusCode::kInvalidArgument,
              "configuration without an explicit geodetic-reference state "
              "must be rejected")) {
    return false;
  }

  config = TestConfig();
  config.map_reference.geodetic.horizontal_crs_realization.clear();
  if (!Expect(LocalizationEngine().Configure(config).code() ==
                  StatusCode::kInvalidArgument,
              "verified reference without CRS realization must be rejected")) {
    return false;
  }
  config = TestConfig();
  config.map_reference.geodetic.coordinate_epoch_decimal_year.reset();
  if (!Expect(LocalizationEngine().Configure(config).code() ==
                  StatusCode::kInvalidArgument,
              "verified reference without coordinate epoch must be rejected")) {
    return false;
  }
  config = TestConfig();
  config.map_reference.geodetic.vertical_datum.clear();
  if (!Expect(LocalizationEngine().Configure(config).code() ==
                  StatusCode::kInvalidArgument,
              "verified reference without vertical datum must be rejected")) {
    return false;
  }
  config = TestConfig();
  config.map_reference.geodetic.enu_origin_latitude_deg = 91.0;
  if (!Expect(LocalizationEngine().Configure(config).code() ==
                  StatusCode::kInvalidArgument,
              "verified reference with invalid origin latitude must be "
              "rejected")) {
    return false;
  }
  config = TestConfig();
  config.map_reference.geodetic.enu_origin_longitude_deg = 181.0;
  if (!Expect(LocalizationEngine().Configure(config).code() ==
                  StatusCode::kInvalidArgument,
              "verified reference with invalid origin longitude must be "
              "rejected")) {
    return false;
  }
  config = TestConfig();
  config.map_reference.geodetic.enu_origin_height_m.reset();
  return Expect(LocalizationEngine().Configure(config).code() ==
                    StatusCode::kInvalidArgument,
                "verified reference without origin height must be rejected");
}

bool TestDiagnosticOnlyGeodeticReferenceIsExplicit() {
  LocalizationConfig config = TestConfig();
  config.map_reference.geodetic =
      GeodeticReference{GeodeticReferenceState::kDiagnosticOnly};
  LocalizationEngine engine;
  if (!Expect(engine.Configure(config).ok(),
              "explicit diagnostic-only reference should be allowed") ||
      !Expect(engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok(),
              "diagnostic-only engine should accept initial odometry") ||
      !Expect(engine.Configure(config).code() == StatusCode::kInvalidArgument,
              "engine configuration must remain immutable")) {
    return false;
  }
  LocalizationResult result;
  return Expect(engine.GetResult(&result).ok() &&
                    result.map_reference == config.map_reference &&
                    result.map_reference.geodetic.state ==
                        GeodeticReferenceState::kDiagnosticOnly,
                "result must preserve diagnostic-only reference state");
}

bool TestOdometryEpochRequiresExplicitEngineReset() {
  LocalizationEngine engine;
  if (!Expect(engine.Configure(TestConfig()).ok(),
              "engine configuration failed") ||
      !Expect(engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok(),
              "initial odometry epoch should be accepted")) {
    return false;
  }
  const OdometryEpoch next_epoch{"odometry", 2};
  if (!Expect(engine.AddOdometry(
                        Odometry(kStartTimeNs + 100'000'000LL, 0.0, next_epoch))
                      .code() == StatusCode::kEpochMismatch,
              "a new odometry epoch must not be mixed into active history")) {
    return false;
  }
  if (!Expect(engine.Reset().ok(),
              "engine reset should explicitly retire the previous epoch") ||
      !Expect(engine
                  .AddOdometry(
                      Odometry(kStartTimeNs + 200'000'000LL, 0.0, next_epoch))
                  .ok(),
              "new odometry epoch should be accepted after reset")) {
    return false;
  }
  LocalizationResult result;
  return Expect(
      engine.GetResult(&result).ok() && result.reset_counter == 1 &&
          result.reset_event && result.odometry_epoch == next_epoch &&
          result.map_reference == SyntheticMapReference(),
      "result should expose reset epoch and map/calibration identity");
}

bool TestLargeCorrectionRequiresConsistentStoppedFixes() {
  LocalizationEngine engine;
  LocalizationConfig config = TestConfig();
  config.alignment_random_walk_sigma_per_sqrt_meter =
      Eigen::Vector4d::Constant(10.0);
  config.large_reset_consistent_observations = 3;
  config.cross_check_absolute_distance_error_m = 10.0;
  if (!Expect(engine.Configure(config).ok(), "engine configuration failed")) {
    return false;
  }
  for (int index = 0; index < 12; ++index) {
    const int64_t timestamp_ns =
        kStartTimeNs + static_cast<int64_t>(index) * 1'000'000'000LL;
    if (!Expect(engine.AddOdometry(Odometry(timestamp_ns, 0.0)).ok(),
                "stationary odometry sample should be accepted")) {
      return false;
    }
    if (index == 0) {
      if (!Expect(
              engine.SubmitGnssFix(Fix(timestamp_ns, Eigen::Vector3d::Zero()))
                  .ok(),
              "initial GNSS alignment should initialize at the map origin")) {
        return false;
      }
    } else if (index >= 3) {
      GnssFix fix = Fix(timestamp_ns, Eigen::Vector3d(5.0, 0.0, 0.0));
      if (index == 4) {
        fix.receiver.carrier_solution = GnssCarrierSolution::kFloat;
      }
      const Status status = engine.SubmitGnssFix(fix);
      if (!Expect(
              index == 4 ? status.code() == StatusCode::kRejectedMeasurement
                         : status.ok(),
              "large consistent corrections should enter reset confirmation")) {
        return false;
      }
      if (index == 3 &&
          !Expect(engine.SubmitGnssHeading(Heading(timestamp_ns, 0.0)).ok(),
                  "same-epoch heading must not bypass reset confirmation")) {
        return false;
      }
      LocalizationResult interim;
      if (index <= 6 &&
          !Expect(engine.GetResult(&interim).ok() &&
                      interim.reset_counter == 0 &&
                      interim.publication_state ==
                          CorrectionPublicationState::kHeldForReset &&
                      interim.map_from_odom.head<3>().norm() < 1e-8,
                  "repeat epoch cannot count twice and bad quality must break "
                  "consecutive stopped confirmation")) {
        return false;
      }
    }
  }

  LocalizationResult result;
  if (!Expect(engine.GetResult(&result).ok() && result.reset_counter == 1 &&
                  result.reset_event &&
                  std::abs(result.map_from_odom.x() - 5.0) < 0.1,
              "confirmed large stopped correction should request a reset")) {
    return false;
  }
  return true;
}

bool TestIntermediateCorrectionIsRateLimited() {
  LocalizationEngine engine;
  LocalizationConfig config = TestConfig();
  config.alignment_random_walk_sigma_per_sqrt_meter =
      Eigen::Vector4d::Constant(10.0);
  config.cross_check_absolute_distance_error_m = 1.0;
  if (!Expect(engine.Configure(config).ok(), "engine configuration failed") ||
      !Expect(engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok(),
              "initial odometry should be accepted") ||
      !Expect(
          engine.SubmitGnssFix(Fix(kStartTimeNs, Eigen::Vector3d::Zero())).ok(),
          "initial GNSS fix should establish map alignment") ||
      !Expect(engine.AddOdometry(Odometry(kStartTimeNs + 1'000'000'000LL, 0.0))
                  .ok(),
              "odometry should create a second alignment segment")) {
    return false;
  }
  if (!Expect(engine
                  .SubmitGnssFix(Fix(kStartTimeNs + 1'000'000'000LL,
                                     Eigen::Vector3d(0.5, 0.0, 0.0)))
                  .ok(),
              "moderate GNSS correction should be admitted")) {
    return false;
  }
  LocalizationResult result;
  return Expect(engine.GetResult(&result).ok() &&
                    result.backend_map_from_odom.x() > 0.45 &&
                    std::abs(result.map_from_odom.x() - 0.3) < 0.05 &&
                    result.correction_pending &&
                    !result.published_mean_matches_backend &&
                    result.publication_state ==
                        CorrectionPublicationState::kRateLimited &&
                    result.publication_pose_offset.x() > 0.15 &&
                    result.alignment_covariance.allFinite(),
                "moderate map correction must obey the configured rate limit");
}

bool TestYawCorrectionUsesCurrentBodyPosition() {
  for (double heading : {0.0005, 0.01}) {
    LocalizationEngine engine;
    const LocalizationConfig config = TestConfig();
    if (!Expect(engine.Configure(config).ok() &&
                    engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok() &&
                    engine.SubmitGnssHeading(Heading(kStartTimeNs, 0.0)).ok(),
                "initialize yaw publication test") ||
        !Expect(
            engine.AddOdometry(Odometry(kStartTimeNs + 1'000'000'000LL, 1000.0))
                    .ok() &&
                engine
                    .SubmitGnssHeading(
                        Heading(kStartTimeNs + 1'000'000'000LL, heading))
                    .ok(),
            "admit yaw observation far from ODOM origin")) {
      return false;
    }
    LocalizationResult result;
    if (!Expect(engine.GetResult(&result).ok(),
                "read current-body publication diagnostics")) {
      return false;
    }
    const auto position = [](const Eigen::Vector4d& alignment) {
      return Eigen::Vector3d(
          alignment.head<3>() +
          Eigen::AngleAxisd(alignment[3], Eigen::Vector3d::UnitZ()) *
              Eigen::Vector3d(1000.0, 0.0, 0.0));
    };
    const double jump =
        (position(result.map_from_odom) - Eigen::Vector3d(1000.0, 0.0, 0.0))
            .norm();
    if (!Expect(result.backend_map_from_odom.head<3>().norm() < 1e-8 &&
                    result.reset_counter == 0 && result.correction_pending &&
                    !result.published_mean_matches_backend,
                "small alignment translation must not hide yaw-induced jump") ||
        !Expect(
            (result.publication_pose_offset.head<3>() -
             (position(result.backend_map_from_odom) -
              position(result.map_from_odom)))
                    .norm() < 1e-9,
            "reported offset must equal composed current-body difference")) {
      return false;
    }
    if (heading < 0.001) {
      if (!Expect(
              result.publication_state ==
                      CorrectionPublicationState::kRateLimited &&
                  std::abs(jump - 0.3) < 1e-8,
              "moderate yaw-induced position change must be rate limited")) {
        return false;
      }
    } else if (!Expect(
                   result.publication_state ==
                           CorrectionPublicationState::kHeldForReset &&
                       jump < 1e-9 &&
                       result.publication_pose_offset.head<3>().norm() > 9.0,
                   "large pose change must be held despite zero translation "
                   "parameter change")) {
      return false;
    }
  }
  return true;
}

bool TestDelayedFixUsesPublicationEpochForRateLimit() {
  LocalizationEngine engine;
  LocalizationConfig config = TestConfig();
  config.alignment_random_walk_sigma_per_sqrt_meter =
      Eigen::Vector4d::Constant(10.0);
  config.cross_check_absolute_distance_error_m = 1.0;
  if (!Expect(
          engine.Configure(config).ok() &&
              engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok() &&
              engine.SubmitGnssFix(Fix(kStartTimeNs, Eigen::Vector3d::Zero()))
                  .ok() &&
              engine.AddOdometry(Odometry(kStartTimeNs + 1'000'000'000LL, 0.0))
                  .ok() &&
              engine.AddOdometry(Odometry(kStartTimeNs + 2'000'000'000LL, 0.0))
                  .ok(),
          "prepare delayed fix and retained historical state") ||
      !Expect(engine
                  .SubmitGnssFix(Fix(kStartTimeNs + 1'000'000'000LL,
                                     Eigen::Vector3d(0.8, 0.0, 0.0)))
                  .ok(),
              "associate delayed fix with historical ODOM")) {
    return false;
  }
  LocalizationResult result;
  return Expect(
      engine.GetResult(&result).ok() &&
          result.timestamp.nanoseconds == kStartTimeNs + 2'000'000'000LL &&
          result.factor_records.back().timestamp_ns ==
              kStartTimeNs + 1'000'000'000LL &&
          result.backend_map_from_odom.x() > 0.7 &&
          std::abs(result.map_from_odom.x() - 0.6) < 1e-8 &&
          result.correction_pending,
      "association uses sensor time but publication rate uses latest "
      "ODOM time without moving the output clock backward");
}

bool TestSegmentBoundaryAndFixedLagRejection() {
  LocalizationConfig config = TestConfig();
  config.alignment_segment_max_duration_ns = 500'000'000LL;
  config.alignment_lag_seconds = 1.0;
  LocalizationEngine engine;
  if (!Expect(engine.Configure(config).ok(), "engine configuration failed")) {
    return false;
  }
  for (int step = 0; step <= 15; ++step) {
    const int64_t timestamp_ns =
        kStartTimeNs + static_cast<int64_t>(step) * 100'000'000LL;
    if (!Expect(engine.AddOdometry(Odometry(timestamp_ns, 0.0)).ok(),
                "odometry should advance through the fixed-lag window")) {
      return false;
    }
  }
  const Status old_fix_status = engine.SubmitGnssFix(
      Fix(kStartTimeNs + 300'000'000LL, Eigen::Vector3d::Zero()));
  LocalizationResult result;
  if (!Expect(
          old_fix_status.code() == StatusCode::kOutsideFixedLagWindow,
          "a fix whose alignment state was marginalized must be rejected") ||
      !Expect(engine.GetResult(&result).ok() &&
                  result.factor_records.back().reason ==
                      "GNSS alignment state is outside the fixed-lag window",
              "fixed-lag rejection must be recorded")) {
    return false;
  }

  LocalizationEngine boundary_engine;
  config.alignment_lag_seconds = 10.0;
  if (!Expect(boundary_engine.Configure(config).ok(),
              "boundary engine configuration failed")) {
    return false;
  }
  for (int step = 0; step <= 5; ++step) {
    const int64_t timestamp_ns =
        kStartTimeNs + static_cast<int64_t>(step) * 100'000'000LL;
    if (!Expect(boundary_engine.AddOdometry(Odometry(timestamp_ns, 0.0)).ok(),
                "boundary odometry should be accepted")) {
      return false;
    }
  }
  if (!Expect(boundary_engine
                  .SubmitGnssFix(Fix(kStartTimeNs + 500'000'000LL,
                                     Eigen::Vector3d::Zero()))
                  .ok(),
              "fix exactly at a segment boundary should be accepted")) {
    return false;
  }
  if (!Expect(boundary_engine.GetResult(&result).ok() &&
                  result.factor_records.back().state_id == 1,
              "segment intervals must be half-open at the boundary")) {
    return false;
  }
  return true;
}

bool TestInnovationGateRejectsAbsoluteJump() {
  LocalizationConfig config = TestConfig();
  config.cross_check_absolute_distance_error_m = 100.0;
  config.cross_check_relative_distance_error = 100.0;
  LocalizationEngine engine;
  if (!Expect(engine.Configure(config).ok(), "engine configuration failed") ||
      !Expect(engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok(),
              "initial odometry should be accepted") ||
      !Expect(
          engine.SubmitGnssFix(Fix(kStartTimeNs, Eigen::Vector3d::Zero())).ok(),
          "initial valid GNSS fix should be accepted") ||
      !Expect(engine.AddOdometry(Odometry(kStartTimeNs + 1'000'000'000LL, 0.0))
                  .ok(),
              "second alignment segment should be created")) {
    return false;
  }
  const Status biased_status = engine.SubmitGnssFix(
      Fix(kStartTimeNs + 1'000'000'000LL, Eigen::Vector3d(3.0, 0.0, 0.0)));
  LocalizationResult result;
  if (!Expect(biased_status.code() == StatusCode::kRejectedMeasurement,
              "fixed-lag innovation gate should reject an absolute jump") ||
      !Expect(engine.GetResult(&result).ok() &&
                  result.factor_records.back().normalized_innovation_squared >
                      config.innovation_gate_threshold &&
                  result.factor_records.back().reason.find("innovation") !=
                      std::string::npos,
              "innovation rejection must expose NIS and its reason")) {
    return false;
  }
  return true;
}

bool TestConsistentRejectedGnssReanchorsGlobalWithoutChangingOdometry() {
  auto replay = [](bool enable_recovery, bool horizontal_only) {
    LocalizationEngine engine;
    LocalizationConfig config = TestConfig();
    config.enable_gnss_reacquisition = enable_recovery;
    config.cross_check_absolute_distance_error_m = 100.0;
    config.cross_check_relative_distance_error = 100.0;
    config.cross_check_direction_threshold_rad = 3.14159265358979323846;
    if (!engine.Configure(config).ok()) {
      return std::make_pair(false, LocalizationResult{});
    }
    LocalizationResult result;
    std::vector<OdometrySample> odometry;
    for (int index = 0; index <= 18; ++index) {
      const int64_t timestamp =
          kStartTimeNs + static_cast<int64_t>(index) * 1'000'000'000LL;
      const double x = static_cast<double>(std::min(index, 12));
      OdometrySample sample = Odometry(timestamp, x);
      odometry.push_back(sample);
      if (!engine.AddOdometry(sample).ok()) {
        return std::make_pair(false, LocalizationResult{});
      }
      const Eigen::Vector3d global_position(index == 0 ? 0.0 : x + 10.0, 0.0,
                                            0.0);
      const Status status =
          horizontal_only
              ? engine.SubmitGnssHorizontalFix(
                    HorizontalFix(timestamp, global_position.head<2>()))
              : engine.SubmitGnssFix(Fix(timestamp, global_position));
      if (index == 0) {
        if (!status.ok()) {
          return std::make_pair(false, LocalizationResult{});
        }
      } else if (enable_recovery && index >= 11) {
        if (!status.ok()) {
          return std::make_pair(false, LocalizationResult{});
        }
      } else if (status.code() != StatusCode::kRejectedMeasurement) {
        return std::make_pair(false, LocalizationResult{});
      }
      if (enable_recovery && index >= 11 && index <= 16) {
        if (!engine.GetResult(&result).ok() || result.alignment_revision != 1 ||
            result.reset_counter != 0 || result.reset_event ||
            !result.correction_pending ||
            result.published_mean_matches_backend ||
            result.publication_state !=
                CorrectionPublicationState::kHeldForReset ||
            std::abs(result.map_from_odom.x()) > 1e-8 ||
            std::abs(result.backend_map_from_odom.x() - 10.0) > 1e-6) {
          return std::make_pair(false, LocalizationResult{});
        }
      }
    }
    if (!engine.GetResult(&result).ok()) {
      return std::make_pair(false, LocalizationResult{});
    }
    for (const auto& sample : odometry) {
      if (sample.pose.translation.x() !=
          std::min(12.0, static_cast<double>(sample.timestamp.nanoseconds -
                                             kStartTimeNs) /
                             1e9)) {
        return std::make_pair(false, LocalizationResult{});
      }
    }
    return std::make_pair(true, result);
  };

  const auto baseline = replay(false, false);
  const auto recovered = replay(true, false);
  const auto horizontal_baseline = replay(false, true);
  const auto horizontal_recovered = replay(true, true);
  if (!Expect(baseline.first && recovered.first && horizontal_baseline.first &&
                  horizontal_recovered.first,
              "baseline and recovery A/B replays must both complete")) {
    return false;
  }
  if (!Expect((horizontal_recovered.second.map_from_odom -
               recovered.second.map_from_odom)
                          .norm() < 1e-8 &&
                  horizontal_recovered.second.alignment_revision == 1 &&
                  horizontal_recovered.second.reset_counter == 1 &&
                  !horizontal_recovered.second.vertical_position_initialized &&
                  recovered.second.vertical_position_initialized &&
                  horizontal_baseline.second.reset_counter == 0,
              "public XY must share global recovery/publication without "
              "claiming Z")) {
    return false;
  }
  const auto recovery_record = std::find_if(
      recovered.second.factor_records.rbegin(),
      recovered.second.factor_records.rend(), [](const FactorRecord& record) {
        return record.source == "gnss_recovery" && record.accepted;
      });
  if (!Expect(recovery_record != recovered.second.factor_records.rend(),
              "validated holdout should record explicit global re-anchor") ||
      !Expect(
          recovered.second.reset_counter == 1 && recovered.second.reset_event,
          "stopped confirmation should emit one published reset event") ||
      !Expect(recovered.second.alignment_revision == 1 &&
                  recovered.second.published_mean_matches_backend &&
                  !recovered.second.correction_pending &&
                  recovered.second.publication_state ==
                      CorrectionPublicationState::kAligned,
              "backend revision must be distinct from publication state") ||
      !Expect(recovered.second.odometry_epoch == baseline.second.odometry_epoch,
              "global re-anchor must preserve the local ODOM epoch") ||
      !Expect(
          std::abs(recovered.second.map_from_odom.x() - 10.0) < 1e-6 &&
              std::abs(recovered.second.map_from_odom.y()) < 1e-6 &&
              std::abs(recovered.second.map_from_odom.z()) < 1e-12,
          "validated recovery should correct XY and preserve unobserved Z") ||
      !Expect(std::abs(baseline.second.map_from_odom.x()) < 1e-6,
              "disabled A baseline must retain the original alignment")) {
    return false;
  }
  return Expect(!baseline.second.reset_event,
                "disabled baseline must not emit a recovery reset");
}

bool TestRecoverySurvivesStabilityWaitingButNotBadQualityOrExpiry() {
  auto replay = [](bool invalid_quality, bool expired) {
    LocalizationEngine engine;
    LocalizationConfig config = TestConfig();
    config.enable_gnss_reacquisition = true;
    config.fixed_solution_stability_ns = 3'000'000'000LL;
    config.cross_check_absolute_distance_error_m = 100.0;
    if (!engine.Configure(config).ok()) {
      return std::make_pair(false, LocalizationResult{});
    }
    for (int index = 0; index <= 36; ++index) {
      const int64_t timestamp = kStartTimeNs + index * 1'000'000'000LL +
                                (expired && index >= 21 ? 31'000'000'000LL : 0);
      const OdometrySample sample = Odometry(timestamp, 2.0 * index);
      if (!engine.AddOdometry(sample).ok()) {
        return std::make_pair(false, LocalizationResult{});
      }
      if (index >= 20 && index % 10 == 0) {
        continue;
      }
      GnssHorizontalFix fix = HorizontalFix(
          timestamp, Eigen::Vector2d(2.0 * index + (index >= 15 ? 10 : 0), 0));
      if (invalid_quality && index == 21) {
        fix.receiver.carrier_solution = GnssCarrierSolution::kInvalid;
      }
      const Status status = engine.SubmitGnssHorizontalFix(fix);
      if (!status.ok() && status.code() != StatusCode::kRejectedMeasurement) {
        return std::make_pair(false, LocalizationResult{});
      }
    }
    LocalizationResult result;
    return std::make_pair(engine.GetResult(&result).ok(), result);
  };
  const auto waiting = replay(false, false);
  const auto bad_quality = replay(true, false);
  const auto expired = replay(false, true);
  if (!Expect(waiting.first && bad_quality.first && expired.first,
              "intermittent recovery controls must finish") ||
      !Expect(waiting.second.alignment_revision == 1 &&
                  waiting.second.reset_counter == 0 &&
                  waiting.second.correction_pending &&
                  !waiting.second.vertical_position_initialized,
              "qualified recovery evidence must survive stability waiting "
              "without publishing a moving reset") ||
      !Expect(
          bad_quality.second.alignment_revision == 0 &&
              expired.second.alignment_revision == 0,
          "invalid receiver quality and 30 s expiry must clear old evidence")) {
    return false;
  }
  const auto& records = waiting.second.factor_records;
  return Expect(std::any_of(records.begin(), records.end(),
                            [](const FactorRecord& row) {
                              return row.reason ==
                                         "fixed_solution_not_stable" &&
                                     !row.accepted;
                            }),
                "stability waiting must remain a rejected observation");
}

bool TestVerticalRejectionCannotTriggerHorizontalRecovery() {
  LocalizationEngine engine;
  LocalizationConfig config = TestConfig();
  config.enable_gnss_reacquisition = true;
  if (!Expect(engine.Configure(config).ok(),
              "configure Z-only recovery test") ||
      !Expect(engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok(),
              "initialize Z-only odometry") ||
      !Expect(
          engine.SubmitGnssFix(Fix(kStartTimeNs, Eigen::Vector3d::Zero())).ok(),
          "initialize Z-only global alignment")) {
    return false;
  }
  for (int index = 1; index <= 14; ++index) {
    const int64_t timestamp =
        kStartTimeNs + static_cast<int64_t>(index) * 1'000'000'000LL;
    if (!Expect(engine.AddOdometry(Odometry(timestamp, index)).ok(),
                "advance Z-only odometry") ||
        !Expect(
            engine.SubmitGnssFix(Fix(timestamp, Eigen::Vector3d(index + 0.05,
                                                                0.0, 1000.0)))
                    .code() == StatusCode::kRejectedMeasurement,
            "vertical outlier must remain rejected after recovery window")) {
      return false;
    }
  }
  LocalizationResult result;
  return Expect(engine.GetResult(&result).ok() && result.reset_counter == 0 &&
                    result.map_from_odom.head<3>().norm() < 1e-8,
                "vertical rejects must not reset or move global alignment") &&
         Expect(std::none_of(result.factor_records.begin(),
                             result.factor_records.end(),
                             [](const FactorRecord& record) {
                               return record.source == "gnss_recovery";
                             }),
                "vertical rejects must not be relabeled as XY recovery");
}

bool TestStableFixedSolutionAndDropoutUncertainty() {
  LocalizationConfig config = TestConfig();
  config.fixed_solution_stability_ns = 3'000'000'000LL;
  LocalizationEngine engine;
  if (!Expect(engine.Configure(config).ok(), "engine configuration failed")) {
    return false;
  }
  LocalizationResult result;
  for (int second = 0; second <= 4; ++second) {
    const int64_t timestamp_ns =
        kStartTimeNs + static_cast<int64_t>(second) * 1'000'000'000LL;
    if (!Expect(engine.AddOdometry(Odometry(timestamp_ns, 0.0)).ok(),
                "stable-fix odometry should be accepted")) {
      return false;
    }
    const Status status =
        engine.SubmitGnssFix(Fix(timestamp_ns, Eigen::Vector3d::Zero()));
    const bool should_be_stable = second >= 3;
    if (!Expect(
            should_be_stable
                ? status.ok()
                : status.code() == StatusCode::kRejectedMeasurement,
            "RTK fixed status must remain stable for the configured time")) {
      return false;
    }
  }
  if (!Expect(
          engine.GetResult(&result).ok() && result.factor_records.size() == 5 &&
              result.factor_records[0].reason == "fixed_solution_not_stable" &&
              result.factor_records.back().accepted &&
              result.global_alignment_valid,
          "fixed-solution stability decisions should be recorded")) {
    return false;
  }
  const double covariance_after_fix = result.alignment_covariance(0, 0);
  const int64_t next_timestamp_ns = kStartTimeNs + 4 * 1'000'000'000LL +
                                    config.alignment_segment_max_duration_ns;
  if (!Expect(engine.AddOdometry(Odometry(next_timestamp_ns, 0.0)).ok(),
              "odometry should continue through GNSS dropout")) {
    return false;
  }
  if (!Expect(
          engine.GetResult(&result).ok() &&
              result.alignment_covariance(0, 0) > covariance_after_fix &&
              result.global_alignment_valid,
          "alignment uncertainty should grow without new GNSS information")) {
    return false;
  }
  if (!Expect(
          engine.SubmitGnssFix(Fix(next_timestamp_ns, Eigen::Vector3d::Zero()))
              .ok(),
          "GNSS should be accepted again after an outage")) {
    return false;
  }
  return Expect(engine.GetResult(&result).ok() &&
                    result.global_alignment_valid &&
                    result.factor_records.back().accepted,
                "accepted GNSS recovery should be visible in the result");
}

bool TestSyntheticReplayIsDeterministic() {
  auto replay = []() {
    LocalizationEngine engine;
    LocalizationConfig config = TestConfig();
    if (!engine.Configure(config).ok() ||
        !engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok() ||
        !engine.SubmitGnssFix(Fix(kStartTimeNs, Eigen::Vector3d::Zero()))
             .ok()) {
      return std::make_pair(false, LocalizationResult{});
    }
    for (int step = 1; step <= 10; ++step) {
      const int64_t timestamp_ns =
          kStartTimeNs + static_cast<int64_t>(step) * 100'000'000LL;
      if (!engine.AddOdometry(Odometry(timestamp_ns, step * 0.1)).ok()) {
        return std::make_pair(false, LocalizationResult{});
      }
    }
    if (!engine
             .SubmitGnssFix(Fix(kStartTimeNs + 1'000'000'000LL,
                                Eigen::Vector3d(1.0, 0.0, 0.0)))
             .ok()) {
      return std::make_pair(false, LocalizationResult{});
    }
    LocalizationResult result;
    return std::make_pair(engine.GetResult(&result).ok(), result);
  };

  const auto first_replay = replay();
  const auto second_replay = replay();
  const LocalizationResult& first = first_replay.second;
  const LocalizationResult& second = second_replay.second;
  if (!Expect(
          first_replay.first && second_replay.first &&
              (first.map_from_odom - second.map_from_odom).norm() < 1e-10 &&
              (first.backend_map_from_odom - second.backend_map_from_odom)
                      .norm() < 1e-10 &&
              (first.alignment_covariance - second.alignment_covariance)
                      .norm() < 1e-10 &&
              first.factor_records.size() == second.factor_records.size(),
          "replaying identical synthetic sensor input must be deterministic")) {
    return false;
  }
  for (std::size_t index = 0; index < first.factor_records.size(); ++index) {
    if (!Expect(
            first.factor_records[index].accepted ==
                    second.factor_records[index].accepted &&
                first.factor_records[index].reason ==
                    second.factor_records[index].reason &&
                std::abs(
                    first.factor_records[index].normalized_innovation_squared -
                    second.factor_records[index]
                        .normalized_innovation_squared) < 1e-10,
            "factor decisions and NIS must replay deterministically")) {
      return false;
    }
  }
  return true;
}

bool TestTranslatedOdometryOriginPreservesPhysicalEstimateAndCovariance() {
  LocalizationEngine original;
  LocalizationEngine shifted;
  LocalizationConfig config = TestConfig();
  config.enable_gnss_reacquisition = true;
  config.large_correction_policy = LargeCorrectionPolicy::kRateLimitedTracking;
  if (!Expect(original.Configure(config).ok() && shifted.Configure(config).ok(),
              "configure equivalent first-ODOM-referenced estimators")) {
    return false;
  }
  const Eigen::Vector3d shift(1000.0, -500.0, 0.0);
  for (int index = 0; index < 24; ++index) {
    const int64_t timestamp = kStartTimeNs + index * 1'000'000'000LL;
    OdometrySample sample = Odometry(timestamp, index);
    sample.pose.translation.y() = 0.1 * index;
    OdometrySample moved = sample;
    moved.pose.translation += shift;
    if (!Expect(original.AddOdometry(sample).ok() &&
                    shifted.AddOdometry(moved).ok(),
                "add immutable equivalent ODOM poses")) {
      return false;
    }
    const GnssHorizontalFix fix = HorizontalFix(
        timestamp, sample.pose.translation.head<2>() +
                       Eigen::Vector2d(0.05 * std::sin(index), 0.0));
    const Status a = original.SubmitGnssHorizontalFix(fix);
    const Status b = shifted.SubmitGnssHorizontalFix(fix);
    if (!Expect(a.code() == b.code(),
                "origin translation must preserve observation admission")) {
      return false;
    }
    LocalizationResult first;
    LocalizationResult second;
    if (!Expect(
            original.GetResult(&first).ok() && shifted.GetResult(&second).ok(),
            "read equivalent physical outputs")) {
      return false;
    }
    const auto position = [](const Eigen::Vector4d& alignment,
                             const Eigen::Vector3d& point) {
      return Eigen::Vector3d(
          alignment.head<3>() +
          Eigen::AngleAxisd(alignment[3], Eigen::Vector3d::UnitZ()) * point);
    };
    const auto body_covariance = [](const LocalizationResult& result,
                                    const Eigen::Vector3d& point) {
      const Eigen::Vector3d rotated =
          Eigen::AngleAxisd(result.backend_map_from_odom[3],
                            Eigen::Vector3d::UnitZ()) *
          point;
      Eigen::Matrix<double, 3, 4> jacobian =
          Eigen::Matrix<double, 3, 4>::Zero();
      jacobian.leftCols<3>().setIdentity();
      jacobian(0, 3) = -rotated.y();
      jacobian(1, 3) = rotated.x();
      return Eigen::Matrix3d(jacobian * result.alignment_covariance *
                             jacobian.transpose());
    };
    const Eigen::Matrix3d first_covariance =
        body_covariance(first, sample.pose.translation);
    const Eigen::Matrix3d second_covariance =
        body_covariance(second, moved.pose.translation);
    const Eigen::Vector3d inverse_scale =
        first_covariance.diagonal().cwiseMax(1.0).cwiseSqrt().cwiseInverse();
    const Eigen::Matrix3d covariance_difference =
        first_covariance - second_covariance;
    const double scaled_covariance_error =
        (inverse_scale.asDiagonal() * covariance_difference *
         inverse_scale.asDiagonal())
            .norm();
    const bool covariance_matches =
        covariance_difference.topLeftCorner<2, 2>().norm() < 1e-6 &&
        scaled_covariance_error < 1e-6;
    if (!covariance_matches) {
      std::cerr << std::setprecision(17)
                << "origin covariance diagnostic index=" << index
                << "\nbackend original="
                << first.backend_map_from_odom.transpose()
                << "\nbackend shifted="
                << second.backend_map_from_odom.transpose()
                << "\noriginal alignment covariance:\n"
                << first.alignment_covariance
                << "\nshifted alignment covariance:\n"
                << second.alignment_covariance
                << "\noriginal body covariance:\n"
                << body_covariance(first, sample.pose.translation)
                << "\nshifted body covariance:\n"
                << body_covariance(second, moved.pose.translation) << '\n';
    }
    if (!Expect(
            (position(first.backend_map_from_odom, sample.pose.translation) -
             position(second.backend_map_from_odom, moved.pose.translation))
                        .norm() < 1e-7 &&
                (position(first.map_from_odom, sample.pose.translation) -
                 position(second.map_from_odom, moved.pose.translation))
                        .norm() < 1e-7,
            "composed backend and publication must ignore ODOM translation") ||
        !Expect(
            covariance_matches,
            "full covariance transform must retain XY/yaw cross terms; index=" +
                std::to_string(index) +
                " norm=" + std::to_string(covariance_difference.norm()) +
                " scaled_norm=" + std::to_string(scaled_covariance_error)) ||
        !Expect(first.publication_state == second.publication_state &&
                    first.alignment_revision == second.alignment_revision &&
                    first.correction_pending == second.correction_pending,
                "origin must not change publication decisions")) {
      return false;
    }
  }
  return true;
}

bool TestTrackingIsBoundedAndStopsOnStaleEvidence() {
  LocalizationEngine engine;
  LocalizationConfig config = TestConfig();
  config.large_correction_policy = LargeCorrectionPolicy::kRateLimitedTracking;
  config.alignment_random_walk_sigma_per_sqrt_meter =
      Eigen::Vector4d::Constant(10.0);
  config.cross_check_absolute_distance_error_m = 100.0;
  if (!Expect(engine.Configure(config).ok(), "configure opt-in tracking") ||
      !Expect(engine.AddOdometry(Odometry(kStartTimeNs, 0.0)).ok() &&
                  engine
                      .SubmitGnssHorizontalFix(
                          HorizontalFix(kStartTimeNs, Eigen::Vector2d::Zero()))
                      .ok(),
              "initialize tracking without a reset")) {
    return false;
  }
  LocalizationResult previous;
  if (!engine.GetResult(&previous).ok()) {
    return false;
  }
  for (int index = 1; index <= 20; ++index) {
    const int64_t timestamp = kStartTimeNs + index * 1'000'000'000LL;
    if (!Expect(engine.AddOdometry(Odometry(timestamp, index)).ok(),
                "advance moving ODOM during tracking")) {
      return false;
    }
    if (index <= 3 || index >= 15) {
      if (!Expect(engine
                      .SubmitGnssHorizontalFix(HorizontalFix(
                          timestamp, Eigen::Vector2d(index + 5, 0)))
                      .ok(),
                  "admit large correction without relaxing NIS")) {
        return false;
      }
    }
    LocalizationResult result;
    if (!Expect(engine.GetResult(&result).ok(), "read tracking state")) {
      return false;
    }
    const auto body = [index](const Eigen::Vector4d& alignment) {
      return Eigen::Vector3d(
          alignment.head<3>() +
          Eigen::AngleAxisd(alignment[3], Eigen::Vector3d::UnitZ()) *
              Eigen::Vector3d(index, 0, 0));
    };
    if (!Expect((body(result.map_from_odom) - body(previous.map_from_odom))
                            .norm() <= 0.3 + 1e-8 &&
                    result.reset_counter == 0 &&
                    result.odometry_epoch == previous.odometry_epoch,
                "all moving corrections must obey rate bound without reset")) {
      return false;
    }
    if (index >= 5 && index <= 14) {
      if (!Expect(!result.correction_source_fresh &&
                      result.global_output_degraded &&
                      result.map_from_odom.isApprox(previous.map_from_odom, 0),
                  "outage must stop tracking and expose degraded freshness")) {
        return false;
      }
    }
    if (index == 15 &&
        !Expect((body(result.map_from_odom) - body(previous.map_from_odom))
                        .norm() <= 0.3 + 1e-8,
                "reacquisition must not spend banked outage time")) {
      return false;
    }
    previous = result;
  }
  if (!Expect(previous.correction_pending && previous.global_output_degraded,
              "unfinished transition must not report normal global output")) {
    return false;
  }
  const int64_t after_gap = kStartTimeNs + 100'000'000'000LL;
  if (!Expect(engine.SubmitGnssHorizontalFix(
                        HorizontalFix(after_gap, Eigen::Vector2d(105, 0)))
                          .code() == StatusCode::kPending &&
                  engine.AddOdometry(Odometry(after_gap, 100)).ok(),
              "a fresh pending fix must resolve after a long ODOM gap")) {
    return false;
  }
  LocalizationResult after_gap_result;
  return Expect(
      engine.GetResult(&after_gap_result).ok() &&
          after_gap_result.correction_source_fresh &&
          after_gap_result.map_from_odom.isApprox(previous.map_from_odom, 0),
      "fresh pending fix must not spend correction credit from a stale gap");
}

}  // namespace
}  // namespace localization
}  // namespace openloc

int main() {
  using openloc::localization::TestConfigurationAndLifecycle;
  using openloc::localization::
      TestConsistentRejectedGnssReanchorsGlobalWithoutChangingOdometry;
  using openloc::localization::TestDelayedFixUsesPublicationEpochForRateLimit;
  using openloc::localization::TestDiagnosticOnlyGeodeticReferenceIsExplicit;
  using openloc::localization::TestDualAntennaHeadingInitializationAndGate;
  using openloc::localization::TestFirstQualifiedFixInitializesGlobalAlignment;
  using openloc::localization::TestGeodeticReferenceConfigurationIsFailClosed;
  using openloc::localization::TestGnssUpdateLeavesOdometryInputUnchanged;
  using openloc::localization::
      TestHorizontalFixPreservesPreviouslyObservedHeight;
  using openloc::localization::
      TestHorizontalFixSharesAdmissionWithoutInventingHeight;
  using openloc::localization::TestInnovationGateRejectsAbsoluteJump;
  using openloc::localization::TestIntermediateCorrectionIsRateLimited;
  using openloc::localization::
      TestLargeCorrectionRequiresConsistentStoppedFixes;
  using openloc::localization::TestMapReferenceMustMatchConfiguredMap;
  using openloc::localization::TestMissingReceiverQualityMetricsAreRejected;
  using openloc::localization::TestOdometryEpochRequiresExplicitEngineReset;
  using openloc::localization::TestPendingFixUsesSensorTimestampAndBracket;
  using openloc::localization::
      TestPositionOnlyConfigurationNeedsNoHeadingBaseline;
  using openloc::localization::TestQualityAndInnovationRejectionsAreRecorded;
  using openloc::localization::
      TestRecoverySurvivesStabilityWaitingButNotBadQualityOrExpiry;
  using openloc::localization::TestSegmentBoundaryAndFixedLagRejection;
  using openloc::localization::TestStableFixedSolutionAndDropoutUncertainty;
  using openloc::localization::TestSyntheticReplayIsDeterministic;
  using openloc::localization::TestTrackingIsBoundedAndStopsOnStaleEvidence;
  using openloc::localization::
      TestTranslatedOdometryOriginPreservesPhysicalEstimateAndCovariance;
  using openloc::localization::
      TestVerticalRejectionCannotTriggerHorizontalRecovery;
  using openloc::localization::TestYawCorrectionUsesCurrentBodyPosition;
  return TestRecoverySurvivesStabilityWaitingButNotBadQualityOrExpiry() &&
                 TestTranslatedOdometryOriginPreservesPhysicalEstimateAndCovariance() &&
                 TestTrackingIsBoundedAndStopsOnStaleEvidence() &&
                 TestConfigurationAndLifecycle() &&
                 TestDualAntennaHeadingInitializationAndGate() &&
                 TestPendingFixUsesSensorTimestampAndBracket() &&
                 TestFirstQualifiedFixInitializesGlobalAlignment() &&
                 TestGnssUpdateLeavesOdometryInputUnchanged() &&
                 TestPositionOnlyConfigurationNeedsNoHeadingBaseline() &&
                 TestHorizontalFixSharesAdmissionWithoutInventingHeight() &&
                 TestHorizontalFixPreservesPreviouslyObservedHeight() &&
                 TestQualityAndInnovationRejectionsAreRecorded() &&
                 TestMissingReceiverQualityMetricsAreRejected() &&
                 TestMapReferenceMustMatchConfiguredMap() &&
                 TestGeodeticReferenceConfigurationIsFailClosed() &&
                 TestDiagnosticOnlyGeodeticReferenceIsExplicit() &&
                 TestOdometryEpochRequiresExplicitEngineReset() &&
                 TestIntermediateCorrectionIsRateLimited() &&
                 TestYawCorrectionUsesCurrentBodyPosition() &&
                 TestDelayedFixUsesPublicationEpochForRateLimit() &&
                 TestLargeCorrectionRequiresConsistentStoppedFixes() &&
                 TestSegmentBoundaryAndFixedLagRejection() &&
                 TestInnovationGateRejectsAbsoluteJump() &&
                 TestStableFixedSolutionAndDropoutUncertainty() &&
                 TestConsistentRejectedGnssReanchorsGlobalWithoutChangingOdometry() &&
                 TestVerticalRejectionCannotTriggerHorizontalRecovery() &&
                 TestSyntheticReplayIsDeterministic()
             ? 0
             : 1;
}
