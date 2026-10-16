#ifndef OPENLOC_LOCALIZATION_ONLINE_TYPES_H_
#define OPENLOC_LOCALIZATION_ONLINE_TYPES_H_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace openloc {
namespace localization {

enum class ClockDomain {
  kUnixEpoch,
  kSensorMonotonic,
};

struct Timestamp {
  // Unix epoch nanoseconds or monotonic sensor-clock nanoseconds, as tagged.
  int64_t nanoseconds = 0;
  ClockDomain clock_domain = ClockDomain::kUnixEpoch;
};

struct FrameId {
  std::string value;
};

enum class GeodeticReferenceState {
  kUnspecified,
  kVerified,
  kDiagnosticOnly,
};

struct GeodeticReference {
  GeodeticReferenceState state = GeodeticReferenceState::kUnspecified;
  std::string horizontal_crs_realization;
  // Decimal year in the declared horizontal CRS realization.
  std::optional<double> coordinate_epoch_decimal_year;
  std::string vertical_datum;
  // Origin height is expressed in vertical_datum.
  std::optional<double> enu_origin_latitude_deg;
  std::optional<double> enu_origin_longitude_deg;
  std::optional<double> enu_origin_height_m;

  bool operator==(const GeodeticReference& other) const {
    return state == other.state &&
           horizontal_crs_realization == other.horizontal_crs_realization &&
           coordinate_epoch_decimal_year ==
               other.coordinate_epoch_decimal_year &&
           vertical_datum == other.vertical_datum &&
           enu_origin_latitude_deg == other.enu_origin_latitude_deg &&
           enu_origin_longitude_deg == other.enu_origin_longitude_deg &&
           enu_origin_height_m == other.enu_origin_height_m;
  }
  bool operator!=(const GeodeticReference& other) const {
    return !(*this == other);
  }
};

struct MapReference {
  // Identifies the immutable configured global-frame origin and orientation
  // contract. The frame may be a local ENU frame and need not be a map.
  std::string id;
  std::string version;
  std::string calibration_id;
  GeodeticReference geodetic;

  bool operator==(const MapReference& other) const {
    return id == other.id && version == other.version &&
           calibration_id == other.calibration_id && geodetic == other.geodetic;
  }
  bool operator!=(const MapReference& other) const { return !(*this == other); }
};

struct Pose {
  // Rigid transform T_parent_child; rotation is a normalized Eigen quaternion.
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};

struct OdometryEpoch {
  // Identifies one producer lifecycle and each explicit local-origin reset.
  std::string producer_session;
  uint64_t generation = 0;

  bool operator==(const OdometryEpoch& other) const {
    return producer_session == other.producer_session &&
           generation == other.generation;
  }
  bool operator!=(const OdometryEpoch& other) const {
    return !(*this == other);
  }
};

// Six-dimensional pose covariance order. Rotation components are the local
// SO(3) tangent vector, not Euler angles. Lower-dimensional measurement
// components follow the documented order for their MeasurementType.
enum class CovarianceOrdering {
  kPositionThenRotation,
  kRotationThenPosition,
};

enum class MeasurementType {
  kAbsolutePose6,
  kAbsolutePosition3,
  kAbsolutePosition2,
  kLateralAndYaw,
  kRelativePose6,
  kHeading,
};

enum class PrecheckStatus {
  kNotRun,
  kAccepted,
  kRejected,
};

struct Measurement {
  Timestamp timestamp;
  std::string source;
  MeasurementType type = MeasurementType::kAbsolutePosition3;
  FrameId frame;
  FrameId reference_frame;
  // Type-specific values: pose [translation, rotation tangent], position
  // [frame x,y,(z)], lateral/yaw [meters,radians], heading [radians].
  Eigen::VectorXd value;
  Eigen::MatrixXd covariance;
  CovarianceOrdering covariance_ordering =
      CovarianceOrdering::kPositionThenRotation;
  double quality = 0.0;
  PrecheckStatus precheck_status = PrecheckStatus::kNotRun;
  std::string precheck_reason;
};

struct OdometrySample {
  Timestamp timestamp;
  // Must remain constant within one history; clear/reset before switching.
  OdometryEpoch epoch;
  FrameId parent_frame;
  FrameId child_frame;
  Pose pose;
};

enum class StatusCode {
  kOk,
  kInvalidArgument,
  kInvalidTimestamp,
  kClockDomainMismatch,
  kFrameMismatch,
  kDuplicateTimestamp,
  kOutOfOrderTimestamp,
  kOutsideHistory,
  kInterpolationGapTooLarge,
  kOutsideFixedLagWindow,
  kRejectedMeasurement,
  kPending,
  kStateUnavailable,
  kOptimizationFailed,
  kEpochMismatch,
  kMapReferenceMismatch,
};

class Status {
 public:
  Status() = default;

  Status(StatusCode code, std::string message)
      : code_(code), message_(std::move(message)) {}

  static Status Ok() { return {}; }

  bool ok() const { return code_ == StatusCode::kOk; }
  StatusCode code() const { return code_; }
  const std::string& message() const { return message_; }

 private:
  StatusCode code_ = StatusCode::kOk;
  std::string message_;
};

}  // namespace localization
}  // namespace openloc

#endif  // OPENLOC_LOCALIZATION_ONLINE_TYPES_H_
