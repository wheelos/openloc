#include "openloc/localization/online/odometry_buffer.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "openloc/localization/online/validation.h"

namespace openloc {
namespace localization {
namespace {

Status Error(StatusCode code, const char* message) {
  return Status(code, message);
}

}  // namespace

OdometryBuffer::OdometryBuffer(OdometryBufferConfig config)
    : config_(std::move(config)) {}

Status OdometryBuffer::ValidateConfig() const {
  if (config_.history_duration_ns <= 0 ||
      config_.maximum_interpolation_gap_ns <= 0 ||
      config_.maximum_interpolation_gap_ns > config_.history_duration_ns ||
      config_.maximum_samples < 2) {
    return Error(StatusCode::kInvalidArgument,
                 "odometry buffer limits are inconsistent");
  }
  return Status::Ok();
}

Status OdometryBuffer::Add(const OdometrySample& sample) {
  Status status = ValidateConfig();
  if (!status.ok()) {
    return status;
  }
  status = ValidateOdometrySample(sample);
  if (!status.ok()) {
    return status;
  }
  if (!samples_.empty()) {
    const OdometrySample& previous = samples_.back();
    if (sample.timestamp.clock_domain != previous.timestamp.clock_domain) {
      return Error(StatusCode::kClockDomainMismatch,
                   "odometry clock domain changed within history");
    }
    if (sample.parent_frame.value != previous.parent_frame.value ||
        sample.child_frame.value != previous.child_frame.value) {
      return Error(StatusCode::kFrameMismatch,
                   "odometry frame pair changed within history");
    }
    if (sample.epoch != previous.epoch) {
      return Error(StatusCode::kEpochMismatch,
                   "odometry epoch changed without clearing history");
    }
    if (sample.timestamp.nanoseconds == previous.timestamp.nanoseconds) {
      return Error(StatusCode::kDuplicateTimestamp,
                   "duplicate odometry timestamp");
    }
    if (sample.timestamp.nanoseconds < previous.timestamp.nanoseconds) {
      return Error(StatusCode::kOutOfOrderTimestamp,
                   "odometry timestamps must be strictly increasing");
    }
  }

  samples_.push_back(sample);
  while (samples_.size() > config_.maximum_samples) {
    samples_.pop_front();
  }
  while (samples_.size() > 2 && samples_.back().timestamp.nanoseconds -
                                        samples_.front().timestamp.nanoseconds >
                                    config_.history_duration_ns) {
    samples_.pop_front();
  }
  return Status::Ok();
}

Status OdometryBuffer::Interpolate(const Timestamp& timestamp,
                                   OdometrySample* result) const {
  if (result == nullptr) {
    return Error(StatusCode::kInvalidArgument,
                 "interpolation result pointer must not be null");
  }
  Status status = ValidateConfig();
  if (!status.ok()) {
    return status;
  }
  status = ValidateTimestamp(timestamp);
  if (!status.ok()) {
    return status;
  }
  if (samples_.empty()) {
    return Error(StatusCode::kOutsideHistory, "odometry history is empty");
  }
  if (timestamp.clock_domain != samples_.front().timestamp.clock_domain) {
    return Error(StatusCode::kClockDomainMismatch,
                 "query clock domain differs from odometry history");
  }

  const auto upper =
      std::lower_bound(samples_.begin(), samples_.end(), timestamp.nanoseconds,
                       [](const OdometrySample& sample, int64_t query_ns) {
                         return sample.timestamp.nanoseconds < query_ns;
                       });
  if (upper == samples_.end() ||
      upper->timestamp.nanoseconds == timestamp.nanoseconds) {
    if (upper == samples_.end()) {
      return Error(StatusCode::kOutsideHistory,
                   "odometry extrapolation is not permitted");
    }
    *result = *upper;
    return Status::Ok();
  }
  if (upper == samples_.begin()) {
    return Error(StatusCode::kOutsideHistory,
                 "odometry extrapolation is not permitted");
  }

  const auto lower = std::prev(upper);
  const int64_t gap_ns =
      upper->timestamp.nanoseconds - lower->timestamp.nanoseconds;
  if (gap_ns > config_.maximum_interpolation_gap_ns) {
    return Error(StatusCode::kInterpolationGapTooLarge,
                 "odometry interpolation gap exceeds configured limit");
  }
  const double alpha = static_cast<double>(timestamp.nanoseconds -
                                           lower->timestamp.nanoseconds) /
                       static_cast<double>(gap_ns);

  OdometrySample interpolated;
  interpolated.timestamp = timestamp;
  interpolated.epoch = lower->epoch;
  interpolated.parent_frame = lower->parent_frame;
  interpolated.child_frame = lower->child_frame;
  interpolated.pose.translation =
      lower->pose.translation +
      alpha * (upper->pose.translation - lower->pose.translation);
  interpolated.pose.rotation =
      lower->pose.rotation.slerp(alpha, upper->pose.rotation).normalized();
  *result = std::move(interpolated);
  return Status::Ok();
}

bool OdometryBuffer::OldestTimestamp(Timestamp* timestamp) const {
  if (timestamp == nullptr || samples_.empty()) {
    return false;
  }
  *timestamp = samples_.front().timestamp;
  return true;
}

void OdometryBuffer::Clear() { samples_.clear(); }

}  // namespace localization
}  // namespace openloc
