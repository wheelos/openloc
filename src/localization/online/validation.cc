#include "openloc/localization/online/validation.h"

#include <cmath>
#include <string>

#include <Eigen/Cholesky>

namespace openloc {
namespace localization {
namespace {

Status Invalid(StatusCode code, const std::string& message) {
  return Status(code, message);
}

}  // namespace

Status ValidateTimestamp(const Timestamp& timestamp) {
  switch (timestamp.clock_domain) {
    case ClockDomain::kUnixEpoch:
      if (timestamp.nanoseconds > 0) {
        return Status::Ok();
      }
      break;
    case ClockDomain::kSensorMonotonic:
      if (timestamp.nanoseconds >= 0) {
        return Status::Ok();
      }
      break;
  }
  return Invalid(StatusCode::kInvalidTimestamp,
                 "timestamp must be positive Unix time or nonnegative "
                 "monotonic sensor time");
}

Status ValidateFrameId(const FrameId& frame) {
  if (frame.value.empty()) {
    return Invalid(StatusCode::kInvalidArgument, "frame id must not be empty");
  }
  if (frame.value.find_first_of(" \t\r\n") != std::string::npos) {
    return Invalid(StatusCode::kInvalidArgument,
                   "frame id must not contain whitespace");
  }
  return Status::Ok();
}

Status ValidatePose(const Pose& pose, double quaternion_norm_tolerance) {
  if (!pose.translation.allFinite() || !pose.rotation.coeffs().allFinite()) {
    return Invalid(StatusCode::kInvalidArgument,
                   "pose translation and quaternion must be finite");
  }
  if (!std::isfinite(quaternion_norm_tolerance) ||
      quaternion_norm_tolerance < 0.0) {
    return Invalid(StatusCode::kInvalidArgument,
                   "quaternion norm tolerance must be finite and nonnegative");
  }
  const double norm = pose.rotation.norm();
  if (!std::isfinite(norm) ||
      std::abs(norm - 1.0) > quaternion_norm_tolerance) {
    return Invalid(StatusCode::kInvalidArgument,
                   "pose quaternion must be normalized");
  }
  return Status::Ok();
}

Status ValidateCovariance(const Eigen::MatrixXd& covariance,
                          Eigen::Index dimension) {
  if (dimension <= 0 || covariance.rows() != dimension ||
      covariance.cols() != dimension) {
    return Invalid(StatusCode::kInvalidArgument,
                   "covariance dimension does not match measurement type");
  }
  if (!covariance.allFinite()) {
    return Invalid(StatusCode::kInvalidArgument,
                   "covariance must contain only finite values");
  }
  constexpr double kSymmetryTolerance = 1e-10;
  if (!covariance.isApprox(covariance.transpose(), kSymmetryTolerance)) {
    return Invalid(StatusCode::kInvalidArgument,
                   "covariance must be symmetric");
  }
  Eigen::LLT<Eigen::MatrixXd> factorization(covariance);
  if (factorization.info() != Eigen::Success ||
      (factorization.matrixL().toDenseMatrix().diagonal().array() <= 0.0)
          .any()) {
    return Invalid(StatusCode::kInvalidArgument,
                   "covariance must be positive definite");
  }
  return Status::Ok();
}

Eigen::Index MeasurementDimension(MeasurementType type) {
  switch (type) {
    case MeasurementType::kAbsolutePose6:
    case MeasurementType::kRelativePose6:
      return 6;
    case MeasurementType::kAbsolutePosition3:
      return 3;
    case MeasurementType::kAbsolutePosition2:
      return 2;
    case MeasurementType::kLateralAndYaw:
      return 2;
    case MeasurementType::kHeading:
      return 1;
  }
  return 0;
}

Status ValidateMeasurement(const Measurement& measurement) {
  Status status = ValidateTimestamp(measurement.timestamp);
  if (!status.ok()) {
    return status;
  }
  if (measurement.source.empty()) {
    return Invalid(StatusCode::kInvalidArgument,
                   "measurement source must not be empty");
  }
  status = ValidateFrameId(measurement.frame);
  if (!status.ok()) {
    return status;
  }
  if (measurement.type == MeasurementType::kRelativePose6) {
    status = ValidateFrameId(measurement.reference_frame);
    if (!status.ok()) {
      return Invalid(StatusCode::kInvalidArgument,
                     "relative pose requires a reference frame");
    }
  }
  switch (measurement.covariance_ordering) {
    case CovarianceOrdering::kPositionThenRotation:
    case CovarianceOrdering::kRotationThenPosition:
      break;
    default:
      return Invalid(StatusCode::kInvalidArgument,
                     "measurement covariance ordering is invalid");
  }
  switch (measurement.precheck_status) {
    case PrecheckStatus::kNotRun:
    case PrecheckStatus::kAccepted:
      break;
    case PrecheckStatus::kRejected:
      if (measurement.precheck_reason.empty()) {
        return Invalid(StatusCode::kInvalidArgument,
                       "a rejected precheck must provide a reason");
      }
      break;
    default:
      return Invalid(StatusCode::kInvalidArgument,
                     "measurement precheck status is invalid");
  }

  const Eigen::Index dimension = MeasurementDimension(measurement.type);
  if (dimension == 0 || measurement.value.size() != dimension ||
      !measurement.value.allFinite()) {
    return Invalid(StatusCode::kInvalidArgument,
                   "measurement value dimension or contents are invalid");
  }
  status = ValidateCovariance(measurement.covariance, dimension);
  if (!status.ok()) {
    return status;
  }
  if (!std::isfinite(measurement.quality) || measurement.quality < 0.0 ||
      measurement.quality > 1.0) {
    return Invalid(StatusCode::kInvalidArgument,
                   "measurement quality must be finite and in [0, 1]");
  }
  return Status::Ok();
}

Status ValidateOdometrySample(const OdometrySample& sample) {
  Status status = ValidateTimestamp(sample.timestamp);
  if (!status.ok()) {
    return status;
  }
  if (sample.epoch.producer_session.empty() || sample.epoch.generation == 0) {
    return Invalid(StatusCode::kInvalidArgument,
                   "odometry epoch requires a producer session and generation");
  }
  status = ValidateFrameId(sample.parent_frame);
  if (!status.ok()) {
    return status;
  }
  status = ValidateFrameId(sample.child_frame);
  if (!status.ok()) {
    return status;
  }
  if (sample.parent_frame.value == sample.child_frame.value) {
    return Invalid(StatusCode::kInvalidArgument,
                   "odometry parent and child frames must differ");
  }
  return ValidatePose(sample.pose);
}

}  // namespace localization
}  // namespace openloc
