#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "openloc/localization/online/localization_engine.h"

namespace openloc {
namespace localization {
namespace {

constexpr double kNanosecondsPerSecond = 1e9;
constexpr int64_t kNanosecondsPerDay = 86'400'000'000'000LL;
constexpr double kHorizontalSigmaMeters = 1.0;
constexpr double kUnconstrainedVerticalVarianceM2 = 1e6;
constexpr int64_t kDiagnosticMaximumInterpolationGapNs = 120'000'000LL;
const Eigen::Vector3d kGnssAntennaInFastLioBodyMeters(-0.131, -0.02329,
                                                      0.00412);
const GeodeticReference kDiagnosticGeodeticReference{
    GeodeticReferenceState::kDiagnosticOnly,
    "EPSG:32650",
    std::nullopt,
    "ellipsoidal_height_from_GGA_H_plus_N",
    22.7152590047,
    114.3831726317,
    91.5139};
const MapReference kDiagnosticMapReference{
    "20261008141912", "first_gga_utm_origin_v1",
    "user_lidar_gnss_lever_arm_composed_with_profile_T_I_L",
    kDiagnosticGeodeticReference};

struct OdomRow {
  OdometrySample sample;
};

struct GnssRow {
  GnssFix fix;
};

struct GgaMeasurement {
  int64_t time_of_day_ns = 0;
  double altitude_msl_m = 0.0;
  double geoid_separation_m = 0.0;
  double ellipsoidal_height_m = 0.0;
};

double ParseDouble(const std::string& value);

std::vector<std::string> Split(const std::string& line, char delimiter) {
  std::vector<std::string> fields;
  std::istringstream stream(line);
  std::string field;
  while (std::getline(stream, field, delimiter)) {
    if (!field.empty() && field.back() == '\r') {
      field.pop_back();
    }
    fields.push_back(field);
  }
  return fields;
}

GgaMeasurement ParseGgaMeasurement(const std::string& input_line) {
  std::string line = input_line;
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  if (line.empty() || line.front() != '$') {
    throw std::runtime_error("GGA sentence has no NMEA start marker");
  }
  const std::size_t checksum_position = line.find('*');
  if (checksum_position == std::string::npos ||
      checksum_position + 3 != line.size()) {
    throw std::runtime_error("GGA sentence has a malformed checksum suffix");
  }
  unsigned int checksum = 0;
  for (std::size_t index = 1; index < checksum_position; ++index) {
    checksum ^= static_cast<unsigned char>(line[index]);
  }
  std::size_t parsed_checksum_size = 0;
  const unsigned long expected_checksum =
      std::stoul(line.substr(checksum_position + 1), &parsed_checksum_size, 16);
  if (parsed_checksum_size != 2 || checksum != expected_checksum) {
    throw std::runtime_error("GGA sentence checksum does not match");
  }

  const auto fields = Split(line.substr(1, checksum_position - 1), ',');
  if (fields.size() < 13 || fields[0].size() < 3 ||
      fields[0].compare(fields[0].size() - 3, 3, "GGA") != 0) {
    throw std::runtime_error("sentence is not a complete NMEA GGA record");
  }
  if (fields[10] != "M" || fields[12] != "M") {
    throw std::runtime_error(
        "GGA altitude and geoid separation must be meters");
  }

  const double time_value = ParseDouble(fields[1]);
  const int hours = static_cast<int>(time_value / 10000.0);
  const int minutes = static_cast<int>(
      (time_value - static_cast<double>(hours) * 10000.0) / 100.0);
  const double seconds = time_value - static_cast<double>(hours) * 10000.0 -
                         static_cast<double>(minutes) * 100.0;
  if (hours >= 24 || minutes >= 60 || seconds < 0.0 || seconds >= 60.0) {
    throw std::runtime_error("GGA UTC time-of-day is outside its valid range");
  }
  const int64_t time_of_day_ns = static_cast<int64_t>(std::llround(
      (static_cast<double>(hours * 3600 + minutes * 60) + seconds) *
      kNanosecondsPerSecond));
  const double altitude_msl_m = ParseDouble(fields[9]);
  const double geoid_separation_m = ParseDouble(fields[11]);
  return GgaMeasurement{time_of_day_ns, altitude_msl_m, geoid_separation_m,
                        altitude_msl_m + geoid_separation_m};
}

std::map<int64_t, double> ReadGgaEllipsoidalHeights(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open raw GGA NMEA input");
  }
  std::map<int64_t, double> heights;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    const GgaMeasurement measurement = ParseGgaMeasurement(line);
    const auto inserted = heights.emplace(measurement.time_of_day_ns,
                                          measurement.ellipsoidal_height_m);
    if (!inserted.second && std::abs(inserted.first->second -
                                     measurement.ellipsoidal_height_m) > 1e-4) {
      throw std::runtime_error("duplicate GGA epoch has conflicting heights");
    }
  }
  if (heights.empty()) {
    throw std::runtime_error("raw GGA NMEA input contains no measurements");
  }
  return heights;
}

double ParseDouble(const std::string& value) {
  std::size_t consumed = 0;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::runtime_error("input contains a non-finite or malformed number");
  }
  return parsed;
}

std::map<std::string, std::size_t> HeaderIndex(
    const std::vector<std::string>& header) {
  std::map<std::string, std::size_t> indices;
  for (std::size_t index = 0; index < header.size(); ++index) {
    indices.emplace(header[index], index);
  }
  return indices;
}

std::size_t RequireColumn(const std::map<std::string, std::size_t>& indices,
                          const std::string& name) {
  const auto iterator = indices.find(name);
  if (iterator == indices.end()) {
    throw std::runtime_error("required CSV column is absent: " + name);
  }
  return iterator->second;
}

int64_t TimestampNs(const std::string& seconds) {
  return static_cast<int64_t>(
      std::llround(ParseDouble(seconds) * kNanosecondsPerSecond));
}

std::vector<OdomRow> ReadOdometry(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open ODOM CSV");
  }
  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("ODOM CSV is empty");
  }
  const auto columns = HeaderIndex(Split(line, ','));
  const std::size_t timestamp = RequireColumn(columns, "timestamp");
  const std::size_t x = RequireColumn(columns, "x");
  const std::size_t y = RequireColumn(columns, "y");
  const std::size_t z = RequireColumn(columns, "z");
  const std::size_t qx = RequireColumn(columns, "qx");
  const std::size_t qy = RequireColumn(columns, "qy");
  const std::size_t qz = RequireColumn(columns, "qz");
  const std::size_t qw = RequireColumn(columns, "qw");

  std::vector<OdomRow> rows;
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = Split(line, ',');
    OdometrySample sample;
    sample.timestamp =
        Timestamp{TimestampNs(fields.at(timestamp)), ClockDomain::kUnixEpoch};
    sample.epoch = OdometryEpoch{"fast_lio2_gnss_off", 1};
    sample.parent_frame = FrameId{"camera_init"};
    sample.child_frame = FrameId{"body"};
    sample.pose.translation =
        Eigen::Vector3d(ParseDouble(fields.at(x)), ParseDouble(fields.at(y)),
                        ParseDouble(fields.at(z)));
    sample.pose.rotation = Eigen::Quaterniond(
        ParseDouble(fields.at(qw)), ParseDouble(fields.at(qx)),
        ParseDouble(fields.at(qy)), ParseDouble(fields.at(qz)));
    if (sample.pose.rotation.norm() <= 0.0) {
      throw std::runtime_error("ODOM CSV contains a zero-length quaternion");
    }
    sample.pose.rotation.normalize();
    if (!rows.empty() && sample.timestamp.nanoseconds <=
                             rows.back().sample.timestamp.nanoseconds) {
      throw std::runtime_error("ODOM timestamps are not strictly increasing");
    }
    rows.push_back(OdomRow{std::move(sample)});
  }
  return rows;
}

std::vector<GnssRow> ReadGnss(const std::string& csv_path,
                              const std::string& reference_path,
                              const std::string& gga_text_path) {
  std::ifstream csv(csv_path);
  std::ifstream reference(reference_path);
  if (!csv || !reference) {
    throw std::runtime_error("cannot open GNSS diagnostic input");
  }
  const auto ellipsoidal_heights = ReadGgaEllipsoidalHeights(gga_text_path);
  std::string line;
  if (!std::getline(csv, line)) {
    throw std::runtime_error("GNSS CSV is empty");
  }
  const auto columns = HeaderIndex(Split(line, ','));
  const std::size_t timestamp = RequireColumn(columns, "timestamp");
  const std::size_t status = RequireColumn(columns, "status");
  const std::size_t mode = RequireColumn(columns, "mode");
  const std::size_t quality = RequireColumn(columns, "quality");
  const std::size_t satellites = RequireColumn(columns, "satellites_used");
  const std::size_t hdop = RequireColumn(columns, "hdop");
  const std::size_t age = RequireColumn(columns, "correction_age_s");

  std::vector<GnssRow> rows;
  double origin_ellipsoidal_height_m = 0.0;
  while (std::getline(csv, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = Split(line, ',');
    std::string reference_line;
    while (std::getline(reference, reference_line) &&
           (reference_line.empty() || reference_line[0] == '#')) {
    }
    if (!reference) {
      throw std::runtime_error("GNSS reference has fewer rows than its CSV");
    }
    const auto coordinates = Split(reference_line, ' ');
    std::vector<std::string> values;
    for (const auto& coordinate : coordinates) {
      if (!coordinate.empty()) {
        values.push_back(coordinate);
      }
    }
    if (values.size() != 4) {
      throw std::runtime_error("GNSS reference row must have four fields");
    }

    GnssFix fix;
    fix.timestamp =
        Timestamp{TimestampNs(fields.at(timestamp)), ClockDomain::kUnixEpoch};
    const int64_t reference_timestamp_ns = TimestampNs(values[3]);
    if (std::abs(fix.timestamp.nanoseconds - reference_timestamp_ns) > 1) {
      throw std::runtime_error(
          "GNSS sensor time differs between CSV and projected reference");
    }
    int64_t time_of_day_ns = fix.timestamp.nanoseconds % kNanosecondsPerDay;
    if (time_of_day_ns < 0) {
      time_of_day_ns += kNanosecondsPerDay;
    }
    const auto height = ellipsoidal_heights.find(time_of_day_ns);
    if (height == ellipsoidal_heights.end()) {
      throw std::runtime_error(
          "raw GGA has no epoch matching the GNSS payload timestamp");
    }
    if (rows.empty()) {
      origin_ellipsoidal_height_m = height->second;
    }
    fix.frame = FrameId{"enu"};
    fix.map_reference = kDiagnosticMapReference;
    fix.position =
        Eigen::Vector3d(ParseDouble(values[0]), ParseDouble(values[1]),
                        height->second - origin_ellipsoidal_height_m);
    fix.covariance = Eigen::Matrix3d::Zero();
    fix.covariance.diagonal()
        << kHorizontalSigmaMeters * kHorizontalSigmaMeters,
        kHorizontalSigmaMeters * kHorizontalSigmaMeters,
        kUnconstrainedVerticalVarianceM2;
    fix.receiver.carrier_solution = fields.at(quality) == "4"
                                        ? GnssCarrierSolution::kFixed
                                        : GnssCarrierSolution::kFloat;
    fix.receiver.satellite_count =
        static_cast<uint32_t>(ParseDouble(fields.at(satellites)));
    fix.receiver.hdop = ParseDouble(fields.at(hdop));
    fix.receiver.differential_age_seconds = ParseDouble(fields.at(age));
    fix.receiver.horizontal_accuracy_m = kHorizontalSigmaMeters;
    if (fields.at(status) != "A" || fields.at(mode) != "D") {
      fix.receiver.carrier_solution = GnssCarrierSolution::kInvalid;
    }
    if (!rows.empty() &&
        fix.timestamp.nanoseconds <= rows.back().fix.timestamp.nanoseconds) {
      throw std::runtime_error("GNSS timestamps are not strictly increasing");
    }
    rows.push_back(GnssRow{std::move(fix)});
  }
  std::string extra_reference_line;
  if (std::getline(reference, extra_reference_line)) {
    throw std::runtime_error("GNSS reference has more rows than its CSV");
  }
  return rows;
}

TEST(GgaNmeaAdapter, ConvertsMslAltitudeAndGeoidSeparationToEllipsoidHeight) {
  const GgaMeasurement measurement = ParseGgaMeasurement(
      "$GNGGA,062755.00,2242.91554396,N,11422.99035723,E,4,37,0.5,"
      "93.5572,M,-2.0266,M,1.0,271*4E");

  EXPECT_NEAR(measurement.altitude_msl_m, 93.5572, 1e-6);
  EXPECT_NEAR(measurement.geoid_separation_m, -2.0266, 1e-6);
  EXPECT_NEAR(measurement.ellipsoidal_height_m, 91.5306, 1e-6);
}

TEST(GgaNmeaAdapter, RejectsInvalidChecksum) {
  EXPECT_THROW(ParseGgaMeasurement(
                   "$GNGGA,062755.00,2242.91554396,N,11422.99035723,E,4,37,0.5,"
                   "93.5572,M,-2.0266,M,1.0,271*00"),
               std::runtime_error);
}

double Percentile(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
      std::ceil(fraction * static_cast<double>(values.size())) - 1.0);
  return values.at(std::min(index, values.size() - 1));
}

int64_t MaximumGnssOdomBracketGapNs(const std::vector<OdomRow>& odometry,
                                    const std::vector<GnssRow>& gnss) {
  int64_t maximum_gap_ns = 0;
  for (const auto& row : gnss) {
    const int64_t timestamp_ns = row.fix.timestamp.nanoseconds;
    const auto next =
        std::lower_bound(odometry.begin(), odometry.end(), timestamp_ns,
                         [](const OdomRow& odom, int64_t timestamp) {
                           return odom.sample.timestamp.nanoseconds < timestamp;
                         });
    if (next == odometry.end() ||
        next->sample.timestamp.nanoseconds == timestamp_ns) {
      continue;
    }
    if (next == odometry.begin()) {
      throw std::runtime_error("GNSS epoch has no preceding ODOM sample");
    }
    const auto previous = std::prev(next);
    maximum_gap_ns =
        std::max(maximum_gap_ns, next->sample.timestamp.nanoseconds -
                                     previous->sample.timestamp.nanoseconds);
  }
  return maximum_gap_ns;
}

TEST(OpenLocDatasetDiagnostic, ReplaysGnssOffOdomWithSameRecordGnss) {
  const char* odometry_path = std::getenv("OPENLOC_GNSS_DIAGNOSTIC_ODOM_CSV");
  const char* gnss_path = std::getenv("OPENLOC_GNSS_DIAGNOSTIC_GNSS_CSV");
  const char* reference_path = std::getenv("OPENLOC_GNSS_DIAGNOSTIC_REFERENCE");
  const char* gga_text_path = std::getenv("OPENLOC_GNSS_DIAGNOSTIC_GGA_TXT");
  const char* result_path = std::getenv("OPENLOC_GNSS_DIAGNOSTIC_RESULT");
  if (odometry_path == nullptr || gnss_path == nullptr ||
      reference_path == nullptr || gga_text_path == nullptr ||
      result_path == nullptr) {
    GTEST_SKIP() << "Set the five OPENLOC_GNSS_DIAGNOSTIC_* variables to run "
                    "the external 20261008141912 diagnostic";
  }

  const auto odometry = ReadOdometry(odometry_path);
  const auto gnss = ReadGnss(gnss_path, reference_path, gga_text_path);
  ASSERT_FALSE(odometry.empty());
  ASSERT_FALSE(gnss.empty());
  ASSERT_LE(odometry.front().sample.timestamp.nanoseconds,
            gnss.front().fix.timestamp.nanoseconds);
  ASSERT_GE(odometry.back().sample.timestamp.nanoseconds,
            gnss.back().fix.timestamp.nanoseconds);
  const int64_t maximum_bracket_gap_ns =
      MaximumGnssOdomBracketGapNs(odometry, gnss);
  ASSERT_LE(maximum_bracket_gap_ns, kDiagnosticMaximumInterpolationGapNs);

  LocalizationConfig config;
  config.map_frame = "enu";
  config.odom_frame = "camera_init";
  config.base_frame = "body";
  config.map_reference = kDiagnosticMapReference;
  config.gnss_antenna_in_base = kGnssAntennaInFastLioBodyMeters;
  config.odometry_buffer.maximum_interpolation_gap_ns =
      kDiagnosticMaximumInterpolationGapNs;
  config.maximum_differential_age_seconds = 3.1;
  config.maximum_horizontal_accuracy_m = kHorizontalSigmaMeters;
  config.fixed_solution_stability_ns = 3'000'000'000LL;

  LocalizationEngine engine;
  ASSERT_TRUE(engine.Configure(config).ok());

  std::size_t next_gnss = 0;
  LocalizationResult result;
  std::map<std::string, std::size_t> decisions;
  std::vector<double> accepted_noninitial_residual_m;
  std::size_t accepted_count = 0;
  for (const auto& row : odometry) {
    ASSERT_TRUE(engine.AddOdometry(row.sample).ok());
    while (next_gnss < gnss.size() &&
           gnss[next_gnss].fix.timestamp.nanoseconds <=
               row.sample.timestamp.nanoseconds) {
      const auto& fix = gnss[next_gnss].fix;
      const Status status = engine.SubmitGnssFix(fix);
      ASSERT_TRUE(status.ok() ||
                  status.code() == StatusCode::kRejectedMeasurement)
          << "Unexpected GNSS processing status: " << status.message();
      ASSERT_TRUE(engine.GetResult(&result).ok());
      ASSERT_FALSE(result.factor_records.empty());
      const FactorRecord& record = result.factor_records.back();
      ++decisions[record.accepted ? "accepted" : record.reason];
      if (record.accepted) {
        ++accepted_count;
        if (record.reason != "accepted_initialization" &&
            record.residual.size() >= 2) {
          accepted_noninitial_residual_m.push_back(
              record.residual.head<2>().norm());
        }
      }
      ++next_gnss;
    }
  }
  ASSERT_EQ(next_gnss, gnss.size());
  ASSERT_GT(accepted_count, 0U);
  ASSERT_TRUE(result.global_alignment_valid);
  ASSERT_TRUE(result.alignment_covariance.allFinite());

  std::ofstream output(result_path);
  ASSERT_TRUE(output.good()) << "Cannot write diagnostic result";
  output << "{\n"
         << "  \"recording\": \"20261008141912\",\n"
         << "  \"odometry_source\": \"GNSS-off FAST-LIO2 export\",\n"
         << "  \"gnss_source\": \"same-record GGA payload-time reference\",\n"
         << "  \"gnss_vertical_datum\": "
            "\"ellipsoidal height = GGA altitude + geoid separation\",\n"
         << "  \"gnss_vertical_origin\": "
            "\"first diagnostic GGA ellipsoidal height; diagnostic only\",\n"
         << "  \"gnss_horizontal_sigma_assumption_m\": "
         << kHorizontalSigmaMeters << ",\n"
         << "  \"vertical_variance_placeholder_m2\": "
         << kUnconstrainedVerticalVarianceM2 << ",\n"
         << "  \"gnss_antenna_in_fast_lio_body_m\": ["
         << kGnssAntennaInFastLioBodyMeters.x() << ", "
         << kGnssAntennaInFastLioBodyMeters.y() << ", "
         << kGnssAntennaInFastLioBodyMeters.z() << "],\n"
         << "  \"lever_arm_source\": "
            "\"user-provided_T_L_G_composed_with_profile_T_I_L_unverified\",\n"
         << "  \"maximum_interpolation_gap_ms\": "
         << kDiagnosticMaximumInterpolationGapNs / 1e6 << ",\n"
         << "  \"maximum_gnss_odom_bracket_gap_ms\": "
         << static_cast<double>(maximum_bracket_gap_ns) / 1e6 << ",\n"
         << "  \"odometry_samples\": " << odometry.size() << ",\n"
         << "  \"gnss_epochs\": " << gnss.size() << ",\n"
         << "  \"accepted_fixes\": " << accepted_count << ",\n"
         << "  \"decisions\": {";
  bool first = true;
  for (const auto& decision : decisions) {
    output << (first ? "\n" : ",\n") << "    \"" << decision.first
           << "\": " << decision.second;
    first = false;
  }
  output << (first ? "" : "\n") << "  },\n"
         << "  \"rejection_reason_codes\": [";
  first = true;
  for (const auto& decision : decisions) {
    if (decision.first == "accepted" ||
        decision.first == "accepted_initialization") {
      continue;
    }
    output << (first ? "" : ", ") << "\"" << decision.first << "\"";
    first = false;
  }
  output << "],\n"
         << "  \"accepted_noninitial_prefactor_residual_m\": {\n"
         << "    \"count\": " << accepted_noninitial_residual_m.size() << ",\n"
         << "    \"median\": "
         << Percentile(accepted_noninitial_residual_m, 0.5) << ",\n"
         << "    \"p95\": " << Percentile(accepted_noninitial_residual_m, 0.95)
         << ",\n"
         << "    \"max\": "
         << (accepted_noninitial_residual_m.empty()
                 ? 0.0
                 : *std::max_element(accepted_noninitial_residual_m.begin(),
                                     accepted_noninitial_residual_m.end()))
         << "\n"
         << "  },\n"
         << "  \"global_alignment_valid\": "
         << (result.global_alignment_valid ? "true" : "false") << ",\n"
         << "  \"valid\": "
         << (result.global_alignment_valid ? "true" : "false") << ",\n"
         << "  \"alignment_covariance_state_order\": "
            "[\"x_m\", \"y_m\", \"z_m\", \"yaw_rad\"],\n"
         << "  \"alignment_covariance\": [";
  for (Eigen::Index row = 0; row < result.alignment_covariance.rows(); ++row) {
    output << (row == 0 ? "[" : ", [");
    for (Eigen::Index column = 0; column < result.alignment_covariance.cols();
         ++column) {
      output << (column == 0 ? "" : ", ")
             << result.alignment_covariance(row, column);
    }
    output << "]";
  }
  output << "],\n"
         << "  \"observability\": {"
         << "\"gnss_position_observed\": true, "
         << "\"direct_heading_observed\": "
         << (result.heading_initialized ? "true" : "false") << ", "
         << "\"full_pose_observed\": false, "
         << "\"status\": \"position-only input; full pose observability "
            "not certified\"},\n"
         << "  \"calibration_state\": \"uncalibrated\",\n"
         << "  \"production_admission\": false,\n"
         << "  \"accuracy_claim\": false\n"
         << "}\n";
  ASSERT_TRUE(output.good());
  SUCCEED() << "Processed " << gnss.size() << " GNSS epochs; accepted "
            << accepted_count << "; global alignment is valid.";
}

TEST(OpenLocDatasetDiagnostic, ConvertsRawGgaHeightToEllipsoidalMeters) {
  const char* gga_text_path = std::getenv("OPENLOC_GNSS_DIAGNOSTIC_GGA_TXT");
  if (gga_text_path == nullptr) {
    GTEST_SKIP() << "Set OPENLOC_GNSS_DIAGNOSTIC_GGA_TXT to run the raw GGA "
                    "height adapter check";
  }

  const auto ellipsoidal_heights = ReadGgaEllipsoidalHeights(gga_text_path);
  constexpr int64_t kFirstRawGgaEpochNs = 23'275'000'000'000LL;
  const auto measurement = ellipsoidal_heights.find(kFirstRawGgaEpochNs);
  ASSERT_NE(measurement, ellipsoidal_heights.end());
  EXPECT_NEAR(measurement->second, 91.5306, 1e-4);
}

}  // namespace
}  // namespace localization
}  // namespace openloc
