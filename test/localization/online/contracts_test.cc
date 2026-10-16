#include <cmath>
#include <iostream>
#include <string>

#include "openloc/localization/online/odometry_buffer.h"
#include "openloc/localization/online/validation.h"

namespace openloc {
namespace localization {
namespace {

bool Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
  }
  return condition;
}

Timestamp UnixTime(int64_t nanoseconds) {
  return Timestamp{nanoseconds, ClockDomain::kUnixEpoch};
}

OdometrySample Sample(int64_t timestamp_ns, double x,
                      const std::string& parent = "odom",
                      const std::string& child = "base_link",
                      OdometryEpoch epoch = {"odometry", 1}) {
  OdometrySample sample;
  sample.timestamp = UnixTime(timestamp_ns);
  sample.epoch = epoch;
  sample.parent_frame = FrameId{parent};
  sample.child_frame = FrameId{child};
  sample.pose.translation.x() = x;
  return sample;
}

bool TestMeasurementValidation() {
  Measurement measurement;
  measurement.timestamp = UnixTime(1'000'000'000);
  measurement.source = "gnss";
  measurement.type = MeasurementType::kAbsolutePosition3;
  measurement.frame = FrameId{"map"};
  measurement.value = Eigen::Vector3d(1.0, 2.0, 3.0);
  measurement.covariance = Eigen::Matrix3d::Identity();
  measurement.quality = 1.0;
  if (!Expect(ValidateMeasurement(measurement).ok(),
              "valid position measurement should pass")) {
    return false;
  }

  measurement.covariance(0, 1) = 0.5;
  if (!Expect(ValidateMeasurement(measurement).code() ==
                  StatusCode::kInvalidArgument,
              "asymmetric covariance should be rejected")) {
    return false;
  }

  measurement.covariance = Eigen::Matrix3d::Identity();
  measurement.quality = 1.1;
  if (!Expect(ValidateMeasurement(measurement).code() ==
                  StatusCode::kInvalidArgument,
              "quality outside [0, 1] should be rejected")) {
    return false;
  }

  measurement.quality = 1.0;
  measurement.precheck_status = PrecheckStatus::kRejected;
  if (!Expect(ValidateMeasurement(measurement).code() ==
                  StatusCode::kInvalidArgument,
              "rejected precheck without a reason should be rejected")) {
    return false;
  }

  measurement.precheck_reason = "receiver_fix_invalid";
  measurement.covariance.setZero();
  return Expect(
      ValidateMeasurement(measurement).code() == StatusCode::kInvalidArgument,
      "singular covariance should be rejected");
}

bool TestOdometryOrderingAndFrameContract() {
  OdometryBuffer buffer;
  if (!Expect(buffer.Add(Sample(1'000'000'000, 0.0)).ok(),
              "first odometry sample should be accepted")) {
    return false;
  }
  if (!Expect(buffer.Add(Sample(1'000'000'000, 1.0)).code() ==
                  StatusCode::kDuplicateTimestamp,
              "duplicate timestamps should be rejected")) {
    return false;
  }
  if (!Expect(buffer.Add(Sample(900'000'000, 1.0)).code() ==
                  StatusCode::kOutOfOrderTimestamp,
              "out-of-order timestamps should be rejected")) {
    return false;
  }
  if (!Expect(buffer.Add(Sample(1'100'000'000, 1.0, "map")).code() ==
                  StatusCode::kFrameMismatch,
              "odometry frame changes should be rejected")) {
    return false;
  }
  return Expect(buffer.size() == 1,
                "rejected samples must not mutate odometry history");
}

bool TestOdometryEpochIsRequiredAndCannotBeMixed() {
  OdometryBuffer buffer;
  OdometrySample missing_epoch = Sample(1'000'000'000, 0.0);
  missing_epoch.epoch = OdometryEpoch{};
  if (!Expect(ValidateOdometrySample(missing_epoch).code() ==
                  StatusCode::kInvalidArgument,
              "odometry samples require a producer epoch")) {
    return false;
  }
  if (!Expect(buffer.Add(Sample(1'000'000'000, 0.0)).ok(),
              "first sample in an odometry epoch should be accepted")) {
    return false;
  }
  if (!Expect(buffer.Add(Sample(1'100'000'000, 1.0, "odom", "base_link",
                                OdometryEpoch{"odometry", 2}))
                  .code() == StatusCode::kEpochMismatch,
              "history must reject a sample from a different odometry epoch")) {
    return false;
  }
  if (!Expect(buffer.Add(Sample(1'100'000'000, 1.0)).ok(),
              "same-epoch sample should provide an interpolation bracket")) {
    return false;
  }
  OdometrySample interpolated;
  if (!Expect(buffer.Interpolate(UnixTime(1'050'000'000), &interpolated).ok() &&
                  interpolated.epoch == OdometryEpoch{"odometry", 1},
              "interpolation must preserve the bracketed odometry epoch")) {
    return false;
  }
  buffer.Clear();
  return Expect(buffer.Add(Sample(1'100'000'000, 1.0, "odom", "base_link",
                                  OdometryEpoch{"odometry", 2}))
                    .ok(),
                "a cleared history may start a new odometry epoch");
}

bool TestInterpolationAndNoExtrapolation() {
  OdometryBufferConfig config;
  config.maximum_interpolation_gap_ns = 200'000'000;
  OdometryBuffer buffer(config);
  OdometrySample first = Sample(1'000'000'000, 0.0);
  OdometrySample second = Sample(1'100'000'000, 10.0);
  second.pose.rotation =
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  if (!Expect(buffer.Add(first).ok() && buffer.Add(second).ok(),
              "bracketing odometry samples should be accepted")) {
    return false;
  }

  OdometrySample interpolated;
  Status status = buffer.Interpolate(UnixTime(1'050'000'000), &interpolated);
  if (!Expect(status.ok(), "interpolation inside history should succeed") ||
      !Expect(std::abs(interpolated.pose.translation.x() - 5.0) < 1e-12,
              "translation should be linearly interpolated")) {
    return false;
  }
  const Eigen::Vector3d halfway =
      interpolated.pose.rotation * Eigen::Vector3d::UnitX();
  if (!Expect(
          std::abs(halfway.x()) < 1e-12 && std::abs(halfway.y() - 1.0) < 1e-12,
          "orientation should use shortest-path quaternion slerp")) {
    return false;
  }

  if (!Expect(buffer.Interpolate(UnixTime(900'000'000), &interpolated).code() ==
                  StatusCode::kOutsideHistory,
              "interpolation must not extrapolate before history")) {
    return false;
  }
  if (!Expect(
          buffer.Interpolate(UnixTime(1'200'000'000), &interpolated).code() ==
              StatusCode::kOutsideHistory,
          "interpolation must not extrapolate after history")) {
    return false;
  }
  return Expect(buffer.Interpolate(UnixTime(1'200'000'000), nullptr).code() ==
                    StatusCode::kInvalidArgument,
                "null interpolation output must be rejected");
}

bool TestInterpolationGapAndClockDomain() {
  OdometryBufferConfig config;
  config.maximum_interpolation_gap_ns = 50'000'000;
  OdometryBuffer buffer(config);
  if (!Expect(buffer.Add(Sample(1'000'000'000, 0.0)).ok() &&
                  buffer.Add(Sample(1'100'000'000, 1.0)).ok(),
              "odometry samples should be accepted")) {
    return false;
  }
  OdometrySample result;
  if (!Expect(buffer.Interpolate(UnixTime(1'050'000'000), &result).code() ==
                  StatusCode::kInterpolationGapTooLarge,
              "large odometry gap should not be interpolated")) {
    return false;
  }
  Timestamp monotonic{1'050'000'000, ClockDomain::kSensorMonotonic};
  return Expect(buffer.Interpolate(monotonic, &result).code() ==
                    StatusCode::kClockDomainMismatch,
                "mixed clock domains should be rejected");
}

bool TestBoundedHistoryAndInvalidConfiguration() {
  OdometryBufferConfig config;
  config.history_duration_ns = 150'000'000;
  config.maximum_interpolation_gap_ns = 100'000'000;
  config.maximum_samples = 3;
  OdometryBuffer buffer(config);
  if (!Expect(buffer.Add(Sample(1'000'000'000, 0.0)).ok() &&
                  buffer.Add(Sample(1'100'000'000, 1.0)).ok() &&
                  buffer.Add(Sample(1'200'000'000, 2.0)).ok(),
              "bounded history samples should be accepted")) {
    return false;
  }
  if (!Expect(buffer.Add(Sample(1'300'000'000, 3.0)).ok(),
              "new sample should advance bounded history")) {
    return false;
  }
  OdometrySample result;
  if (!Expect(buffer.size() == 2 &&
                  buffer.Interpolate(UnixTime(1'050'000'000), &result).code() ==
                      StatusCode::kOutsideHistory,
              "history duration should evict old samples")) {
    return false;
  }
  if (!Expect(buffer.Interpolate(UnixTime(1'300'000'000), &result).ok(),
              "latest exact sample should remain queryable")) {
    return false;
  }

  OdometryBufferConfig invalid_config;
  invalid_config.maximum_samples = 1;
  OdometryBuffer invalid_buffer(invalid_config);
  return Expect(invalid_buffer.Add(Sample(1'000'000'000, 0.0)).code() ==
                    StatusCode::kInvalidArgument,
                "invalid buffer configuration should return an error");
}

}  // namespace
}  // namespace localization
}  // namespace openloc

int main() {
  using openloc::localization::TestBoundedHistoryAndInvalidConfiguration;
  using openloc::localization::TestInterpolationAndNoExtrapolation;
  using openloc::localization::TestInterpolationGapAndClockDomain;
  using openloc::localization::TestMeasurementValidation;
  using openloc::localization::TestOdometryEpochIsRequiredAndCannotBeMixed;
  using openloc::localization::TestOdometryOrderingAndFrameContract;
  const bool passed = TestMeasurementValidation() &&
                      TestOdometryOrderingAndFrameContract() &&
                      TestOdometryEpochIsRequiredAndCannotBeMixed() &&
                      TestInterpolationAndNoExtrapolation() &&
                      TestInterpolationGapAndClockDomain() &&
                      TestBoundedHistoryAndInvalidConfiguration();
  return passed ? 0 : 1;
}
