#ifndef OPENLOC_LOCALIZATION_ONLINE_ODOMETRY_BUFFER_H_
#define OPENLOC_LOCALIZATION_ONLINE_ODOMETRY_BUFFER_H_

#include <cstddef>
#include <deque>

#include "openloc/localization/online/types.h"

namespace openloc {
namespace localization {

struct OdometryBufferConfig {
  int64_t history_duration_ns = 10'000'000'000LL;
  int64_t maximum_interpolation_gap_ns = 100'000'000LL;
  std::size_t maximum_samples = 2000;
};

/// Bounded odometry history; calls must be serialized by the owning engine.
class OdometryBuffer {
 public:
  /// Creates a buffer with bounded history and interpolation intervals.
  explicit OdometryBuffer(OdometryBufferConfig config = {});

  /// Adds an odometry sample if its time, frames, and pose are valid.
  Status Add(const OdometrySample& sample);

  /// Interpolates an in-range sample without extrapolating or crossing gaps.
  Status Interpolate(const Timestamp& timestamp, OdometrySample* result) const;

  /// Returns the oldest retained timestamp, or false for an empty buffer/null
  /// output.
  bool OldestTimestamp(Timestamp* timestamp) const;

  /// Removes all retained samples.
  void Clear();

  /// Reports whether the buffer contains any samples.
  bool empty() const { return samples_.empty(); }

  /// Returns the number of retained samples.
  std::size_t size() const { return samples_.size(); }

  /// Returns the immutable limits used by this buffer.
  const OdometryBufferConfig& config() const { return config_; }

 private:
  Status ValidateConfig() const;

  OdometryBufferConfig config_;
  std::deque<OdometrySample> samples_;
};

}  // namespace localization
}  // namespace openloc

#endif  // OPENLOC_LOCALIZATION_ONLINE_ODOMETRY_BUFFER_H_
