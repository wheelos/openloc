#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "candidates/local/local_estimator.h"

namespace openloc {
namespace localization {
namespace unified {
class LocalEstimatorTestAccess {
 public:
  static void SetWheelBiasFeedback(LocalEstimator* estimator, bool gyro,
                                   bool accel, bool forward_only) {
    estimator->wheel_updates_gyro_bias_ = gyro;
    estimator->wheel_updates_accel_bias_ = accel;
    estimator->wheel_forward_velocity_only_ = forward_only;
  }
  static void SeedOfflineDiagnostic(LocalEstimator* estimator,
                                    const LocalState& seed,
                                    const ImuSample& imu) {
    if (estimator->initialized_ || !estimator->has_received_wheel_ ||
        !estimator->pending_wheels_.empty()) {
      throw std::runtime_error(
          "offline seed requires fresh uninitialized replay");
    }
    LocalState initial = seed;
    initial.epoch = estimator->state_.epoch;
    initial.stamp = imu.stamp;
    initial.angular_velocity = imu.angular_velocity - initial.gyro_bias;
    initial.valid = true;
    estimator->state_ = initial;
    estimator->output_state_ = initial;
    estimator->continuous_output_.Reset(initial);
    estimator->initialized_ = true;
    estimator->last_trusted_wheel_ = estimator->last_received_wheel_;
    estimator->has_trusted_wheel_ = true;
    estimator->RecordHistory();
    estimator->RecordInternalHistory();
  }
};
}  // namespace unified
}  // namespace localization
}  // namespace openloc

namespace {

using openloc::localization::unified::ImuSample;
using openloc::localization::unified::LocalConfig;
using openloc::localization::unified::LocalEstimator;
using openloc::localization::unified::LocalEstimatorTrace;
using openloc::localization::unified::LocalEstimatorTraceKind;
using openloc::localization::unified::ReasonName;
using openloc::localization::unified::WheelSample;

std::vector<std::string> Split(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

void StripCarriageReturn(std::string* line) {
  if (!line->empty() && line->back() == '\r') {
    line->pop_back();
  }
}

double ParseDouble(const std::string& value, const std::string& field,
                   std::size_t line_number) {
  std::size_t parsed = 0;
  const double output = std::stod(value, &parsed);
  if (parsed != value.size() || !std::isfinite(output)) {
    throw std::runtime_error("invalid " + field + " at CSV line " +
                             std::to_string(line_number));
  }
  return output;
}

uint64_t ParseSequence(const std::string& value, std::size_t line_number) {
  std::size_t parsed = 0;
  const uint64_t output = std::stoull(value, &parsed);
  if (parsed != value.size() || output == 0) {
    throw std::runtime_error("invalid sequence at CSV line " +
                             std::to_string(line_number));
  }
  return output;
}

openloc::localization::unified::LocalState ReadDiagnosticSeed(
    const std::string& path) {
  std::ifstream input(path);
  std::string header;
  if (!std::getline(input, header)) {
    throw std::runtime_error("cannot read offline diagnostic seed");
  }
  StripCarriageReturn(&header);
  std::string expected = "qw,qx,qy,qz,bgx,bgy,bgz,bax,bay,baz,speed";
  for (int i = 0; i < 15; ++i) {
    expected += ",cov_" + std::to_string(i);
  }
  std::string line;
  if (header != expected || !std::getline(input, line)) {
    throw std::runtime_error("unexpected diagnostic seed schema");
  }
  StripCarriageReturn(&line);
  const auto fields = Split(line);
  if (fields.size() != 26) {
    throw std::runtime_error("diagnostic seed needs 26 fields");
  }
  std::vector<double> values;
  for (const auto& field : fields) {
    values.push_back(ParseDouble(field, "diagnostic seed", 2));
  }
  if (std::getline(input, line)) {
    throw std::runtime_error("diagnostic seed must contain exactly one state");
  }
  openloc::localization::unified::LocalState seed;
  seed.orientation =
      Eigen::Quaterniond(values[0], values[1], values[2], values[3]);
  if (std::abs(seed.orientation.norm() - 1.0) > 1e-6) {
    throw std::runtime_error("diagnostic seed quaternion must be unit length");
  }
  seed.orientation.normalize();
  seed.gyro_bias = {values[4], values[5], values[6]};
  seed.accel_bias = {values[7], values[8], values[9]};
  seed.velocity = seed.orientation * Eigen::Vector3d(values[10], 0.0, 0.0);
  for (int i = 0; i < 15; ++i) {
    if (!(values[11 + i] > 0.0)) {
      throw std::runtime_error(
          "diagnostic seed needs positive diagonal covariance");
    }
    seed.covariance(i, i) = values[11 + i];
  }
  return seed;
}

void WriteHeader(std::ostream& output) {
  output << "time,receive_time,px,py,pz,vx,vy,vz,qw,qx,qy,qz,"
            "gyro_bias_x,gyro_bias_y,gyro_bias_z,accel_bias_x,accel_bias_y,"
            "accel_bias_z,valid,covariance_model_valid,update_reason,"
            "integrity_reason,precision_reason,imu_accepted,imu_rejected,"
            "wheel_accepted,wheel_rejected";
  for (int i = 0; i < 15; ++i) {
    output << ",cov_" << i;
  }
  output << '\n';
}

void WriteState(std::ostream& output, const LocalEstimator& estimator,
                double receive_time, const std::string& update_reason,
                const std::string& integrity_reason,
                const std::string& precision_reason) {
  const auto& state = estimator.state();
  const auto& imu_health = estimator.imu_health();
  const auto& wheel_health = estimator.wheel_health();
  output << std::setprecision(17) << state.stamp.time << ',' << receive_time
         << ',' << state.position.x() << ',' << state.position.y() << ','
         << state.position.z() << ',' << state.velocity.x() << ','
         << state.velocity.y() << ',' << state.velocity.z() << ','
         << state.orientation.w() << ',' << state.orientation.x() << ','
         << state.orientation.y() << ',' << state.orientation.z() << ','
         << state.gyro_bias.x() << ',' << state.gyro_bias.y() << ','
         << state.gyro_bias.z() << ',' << state.accel_bias.x() << ','
         << state.accel_bias.y() << ',' << state.accel_bias.z() << ','
         << (state.valid ? 1 : 0) << ','
         << (state.covariance_model_valid ? 1 : 0) << ',' << update_reason
         << ',' << integrity_reason << ',' << precision_reason << ','
         << imu_health.accepted << ',' << imu_health.rejected << ','
         << wheel_health.accepted << ',' << wheel_health.rejected;
  for (int i = 0; i < 15; ++i) {
    output << ',' << state.covariance(i, i);
  }
  output << '\n';
}

void WriteTraceHeader(std::ostream& output) {
  output << "kind,time,receive_time,sequence,dt,"
            "specific_force_body_x,specific_force_body_y,specific_force_body_z,"
            "angular_rate_body_x,angular_rate_body_y,angular_rate_body_z,"
            "linear_acceleration_local_x,linear_acceleration_local_y,"
            "linear_acceleration_local_z,"
            "px_before,py_before,pz_before,px_after,py_after,pz_after,"
            "vx_before,vy_before,vz_before,vx_after,vy_after,vz_after,"
            "q_before_w,q_before_x,q_before_y,q_before_z,"
            "q_mid_w,q_mid_x,q_mid_y,q_mid_z,"
            "q_after_w,q_after_x,q_after_y,q_after_z,"
            "wheel_speed,wheel_body_vx_before,wheel_body_vy_before,"
            "wheel_body_vz_before,wheel_body_vx_after,wheel_body_vy_after,"
            "wheel_body_vz_after,wheel_innovation_x,wheel_innovation_y,"
            "wheel_innovation_z,wheel_innovation_distance,"
            "delta_vx,delta_vy,delta_vz,delta_gyro_bias_x,delta_gyro_bias_y,"
            "delta_gyro_bias_z,delta_accel_bias_x,delta_accel_bias_y,"
            "delta_accel_bias_z,bg_before_x,bg_before_y,bg_before_z,"
            "bg_after_x,bg_after_y,bg_after_z,ba_before_x,ba_before_y,"
            "ba_before_z,ba_after_x,ba_after_y,ba_after_z,"
            "attitude_error_correction_x,attitude_error_correction_y,"
            "attitude_error_correction_z,delta_vy_from_position,"
            "delta_vy_from_velocity,delta_vy_from_attitude,"
            "delta_vy_from_gyro_bias,delta_vy_from_accel_bias";
  const char* state_names[] = {"px",   "py",      "pz",      "vx",      "vy",
                               "vz",   "theta_x", "theta_y", "theta_z", "bg_x",
                               "bg_y", "bg_z",    "ba_x",    "ba_y",    "ba_z"};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 15; ++column) {
      output << ",wheel_h_" << row << '_' << state_names[column];
    }
  }
  for (int state = 0; state < 15; ++state) {
    for (int measurement = 0; measurement < 3; ++measurement) {
      output << ",wheel_k_" << state_names[state] << '_' << measurement;
    }
  }
  for (int state = 0; state < 15; ++state) {
    for (int measurement = 0; measurement < 3; ++measurement) {
      output << ",raw_k_" << state_names[state] << '_' << measurement;
    }
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      output << ",wheel_s_" << row << '_' << column;
    }
  }
  for (int state = 0; state < 15; ++state) {
    output << ",p_vy_before_" << state;
  }
  for (int state = 0; state < 15; ++state) {
    output << ",p_vy_after_" << state;
  }
  for (int measurement = 0; measurement < 3; ++measurement) {
    for (int block = 0; block < 5; ++block) {
      output << ",p_vy_h_block_" << measurement << '_' << block;
    }
  }
  for (int row = 0; row < 15; ++row) {
    for (int column = 0; column < 15; ++column) {
      output << ",f_" << row << '_' << column;
    }
  }
  for (int row = 0; row < 15; ++row) {
    for (int column = 0; column < 15; ++column) {
      output << ",covariance_prediction_delta_" << row << '_' << column;
    }
  }
  for (int row = 0; row < 15; ++row) {
    for (int column = 0; column < 15; ++column) {
      output << ",covariance_update_delta_" << row << '_' << column;
    }
  }
  for (int row = 0; row < 15; ++row) {
    for (int column = 0; column < 15; ++column) {
      output << ",covariance_after_" << row << '_' << column;
    }
  }
  for (int state = 0; state < 15; ++state) {
    output << ",fpft_vy_" << state;
  }
  for (int row = 0; row < 15; ++row) {
    for (int column = 0; column < 15; ++column) {
      output << ",q_discrete_" << row << '_' << column;
    }
  }
  const char* noise_sources[] = {"accel_white", "gyro_white", "gyro_bias_rw",
                                 "accel_bias_rw"};
  for (const char* source : noise_sources) {
    for (int state = 0; state < 15; ++state) {
      output << ",q_vy_" << source << '_' << state;
    }
  }
  output << '\n';
}

void WriteVector(std::ostream& output, const Eigen::Vector3d& vector) {
  output << ',' << vector.x() << ',' << vector.y() << ',' << vector.z();
}

void WriteQuaternion(std::ostream& output,
                     const Eigen::Quaterniond& quaternion) {
  output << ',' << quaternion.w() << ',' << quaternion.x() << ','
         << quaternion.y() << ',' << quaternion.z();
}

void WriteTrace(std::ostream& output, const LocalEstimatorTrace& trace) {
  output << std::setprecision(17)
         << (trace.kind == LocalEstimatorTraceKind::IMU_PROPAGATION
                 ? "IMU_PROPAGATION"
                 : "WHEEL_UPDATE")
         << ',' << trace.stamp.time << ',' << trace.stamp.receive_time << ','
         << trace.stamp.sequence << ',' << trace.dt;
  WriteVector(output, trace.specific_force_body);
  WriteVector(output, trace.angular_rate_body);
  WriteVector(output, trace.linear_acceleration_local);
  WriteVector(output, trace.position_before);
  WriteVector(output, trace.position_after);
  WriteVector(output, trace.velocity_before);
  WriteVector(output, trace.velocity_after);
  WriteQuaternion(output, trace.orientation_before);
  WriteQuaternion(output, trace.orientation_midpoint);
  WriteQuaternion(output, trace.orientation_after);
  output << ',' << trace.wheel_speed;
  WriteVector(output, trace.wheel_body_velocity_before);
  WriteVector(output, trace.wheel_body_velocity_after);
  WriteVector(output, trace.wheel_innovation);
  output << ',' << trace.wheel_innovation_distance;
  WriteVector(output, trace.velocity_correction);
  WriteVector(output, trace.gyro_bias_correction);
  WriteVector(output, trace.accel_bias_correction);
  WriteVector(output, trace.gyro_bias_before);
  WriteVector(output, trace.gyro_bias_after);
  WriteVector(output, trace.accel_bias_before);
  WriteVector(output, trace.accel_bias_after);
  WriteVector(output, trace.attitude_error_correction);
  for (int block = 0; block < 5; ++block) {
    output << ',' << trace.wheel_lateral_correction_by_state_block(block);
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 15; ++column) {
      output << ',' << trace.wheel_observation(row, column);
    }
  }
  for (int state = 0; state < 15; ++state) {
    for (int measurement = 0; measurement < 3; ++measurement) {
      output << ',' << trace.wheel_gain(state, measurement);
    }
  }
  for (int state = 0; state < 15; ++state) {
    for (int measurement = 0; measurement < 3; ++measurement) {
      output << ',' << trace.unconstrained_wheel_gain(state, measurement);
    }
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      output << ',' << trace.wheel_innovation_covariance(row, column);
    }
  }
  for (int state = 0; state < 15; ++state) {
    output << ',' << trace.velocity_y_covariance_row_before(state);
  }
  for (int state = 0; state < 15; ++state) {
    output << ',' << trace.velocity_y_covariance_row_after(state);
  }
  for (int measurement = 0; measurement < 3; ++measurement) {
    for (int block = 0; block < 5; ++block) {
      output << ','
             << trace.wheel_lateral_cross_covariance_by_state_block(measurement,
                                                                    block);
    }
  }
  for (int row = 0; row < 15; ++row) {
    for (int column = 0; column < 15; ++column) {
      output << ',' << trace.error_transition(row, column);
    }
  }
  for (int row = 0; row < 15; ++row) {
    for (int column = 0; column < 15; ++column) {
      output << ',' << trace.covariance_prediction_delta(row, column);
    }
  }
  for (int row = 0; row < 15; ++row) {
    for (int column = 0; column < 15; ++column) {
      output << ',' << trace.covariance_update_delta(row, column);
    }
  }
  for (int row = 0; row < 15; ++row) {
    for (int column = 0; column < 15; ++column) {
      output << ',' << trace.covariance_after(row, column);
    }
  }
  for (int state = 0; state < 15; ++state) {
    output << ',' << trace.propagated_covariance_y_row(state);
  }
  for (int row = 0; row < 15; ++row) {
    for (int column = 0; column < 15; ++column) {
      output << ',' << trace.process_noise_covariance(row, column);
    }
  }
  for (int source = 0; source < 4; ++source) {
    for (int state = 0; state < 15; ++state) {
      output << ',' << trace.process_noise_velocity_y_by_source(source, state);
    }
  }
  output << '\n';
}

int Run(const std::string& events_path, const std::string& trajectory_path,
        double wheel_variance, bool use_nonholonomic_constraint,
        bool use_wheel_updates, const std::string& diagnostics_path,
        double initial_gyro_bias_std, const std::string& bias_feedback,
        double stationary_duration, const std::string& seed_path) {
  if (!(wheel_variance > 0.0) || !std::isfinite(wheel_variance)) {
    throw std::runtime_error("wheel variance must be finite and positive");
  }
  std::ifstream input(events_path);
  if (!input) {
    throw std::runtime_error("cannot open events CSV: " + events_path);
  }
  std::ofstream output(trajectory_path);
  if (!output) {
    throw std::runtime_error("cannot create trajectory CSV: " +
                             trajectory_path);
  }
  std::ofstream diagnostics;
  if (!diagnostics_path.empty()) {
    diagnostics.open(diagnostics_path);
    if (!diagnostics) {
      throw std::runtime_error("cannot create diagnostics CSV: " +
                               diagnostics_path);
    }
    diagnostics.exceptions(std::ios::failbit | std::ios::badbit);
    WriteTraceHeader(diagnostics);
  }

  LocalConfig config;
  config.session = "1010-chassis-imu-dr";
  config.clock_id = "unix";
  config.allow_imu_only_initialization = false;
  config.enable_zero_velocity_update = false;
  config.use_nonholonomic_constraint = use_nonholonomic_constraint;
  config.use_wheel_velocity_update = use_wheel_updates;
  config.initial_gyro_bias_std = initial_gyro_bias_std;
  config.stationary_samples = 50;
  config.stationary_duration = stationary_duration;
  const auto diagnostic_seed =
      seed_path.empty() ? openloc::localization::unified::LocalState()
                        : ReadDiagnosticSeed(seed_path);
  bool seed_applied = false;
  LocalEstimator estimator(config);
  openloc::localization::unified::LocalEstimatorTestAccess::
      SetWheelBiasFeedback(&estimator,
                           bias_feedback == "all" || bias_feedback == "gyro",
                           bias_feedback == "all" || bias_feedback == "accel",
                           bias_feedback == "forward");
  if (diagnostics.is_open()) {
    estimator.SetTraceCallback(
        [&diagnostics](const LocalEstimatorTrace& trace) {
          WriteTrace(diagnostics, trace);
        });
  }
  const auto config_result = estimator.ValidateConfig();
  if (!config_result.ok()) {
    throw std::runtime_error("invalid estimator configuration: " +
                             config_result.message);
  }

  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("events CSV is empty");
  }
  StripCarriageReturn(&line);
  if (line !=
      "kind,measurement_time,receive_time,sequence,ax,ay,az,gx,gy,gz,speed_"
      "mps") {
    throw std::runtime_error("events CSV has an unexpected header");
  }
  WriteHeader(output);
  std::size_t line_number = 1;
  std::size_t imu_samples = 0;
  std::size_t wheel_samples = 0;
  bool initialized = false;
  double last_receive_time = -std::numeric_limits<double>::infinity();
  while (std::getline(input, line)) {
    ++line_number;
    StripCarriageReturn(&line);
    if (line.empty()) {
      continue;
    }
    const auto fields = Split(line);
    if (fields.size() != 11) {
      throw std::runtime_error("expected 11 CSV columns at line " +
                               std::to_string(line_number));
    }
    const double measurement_time =
        ParseDouble(fields[1], "measurement_time", line_number);
    const double receive_time =
        ParseDouble(fields[2], "receive_time", line_number);
    if (receive_time < last_receive_time) {
      throw std::runtime_error("receive-time regression at CSV line " +
                               std::to_string(line_number));
    }
    last_receive_time = receive_time;
    if (fields[0] == "W") {
      WheelSample sample;
      sample.stamp.time = measurement_time;
      sample.stamp.receive_time = receive_time;
      sample.stamp.sequence = ParseSequence(fields[3], line_number);
      sample.stamp.clock_id = config.clock_id;
      sample.speed = ParseDouble(fields[10], "speed_mps", line_number);
      sample.variance = wheel_variance;
      const auto result = estimator.AddWheel(sample);
      if (!result.ok()) {
        throw std::runtime_error(
            "wheel sample rejected at CSV line " + std::to_string(line_number) +
            ": " + ReasonName(result.reason) + ": " + result.message);
      }
      ++wheel_samples;
      continue;
    }
    if (fields[0] != "I") {
      throw std::runtime_error("unknown event kind at CSV line " +
                               std::to_string(line_number));
    }
    ImuSample sample;
    sample.stamp.time = measurement_time;
    sample.stamp.receive_time = receive_time;
    sample.stamp.sequence = ParseSequence(fields[3], line_number);
    sample.stamp.clock_id = config.clock_id;
    sample.acceleration = {ParseDouble(fields[4], "ax", line_number),
                           ParseDouble(fields[5], "ay", line_number),
                           ParseDouble(fields[6], "az", line_number)};
    sample.angular_velocity = {ParseDouble(fields[7], "gx", line_number),
                               ParseDouble(fields[8], "gy", line_number),
                               ParseDouble(fields[9], "gz", line_number)};
    const auto result = estimator.AddImu(sample);
    if (!seed_path.empty() && !seed_applied && result.ok()) {
      openloc::localization::unified::LocalEstimatorTestAccess::
          SeedOfflineDiagnostic(&estimator, diagnostic_seed, sample);
      seed_applied = true;
    }
    const bool committed =
        estimator.state().stamp.time == sample.stamp.time &&
        estimator.state().stamp.sequence == sample.stamp.sequence;
    if (!result.ok() && !committed) {
      throw std::runtime_error(
          "IMU sample rejected at CSV line " + std::to_string(line_number) +
          ": " + ReasonName(result.reason) + ": " + result.message);
    }
    ++imu_samples;
    initialized = initialized || estimator.state().valid;
    const auto integrity = estimator.EvaluateIntegrity(receive_time);
    const auto precision = estimator.Evaluate(receive_time);
    const bool waiting_for_initialization =
        integrity.reason ==
            openloc::localization::unified::Reason::WAITING_FOR_STATIONARY &&
        !estimator.state().valid;
    if ((!integrity.ok() && !waiting_for_initialization) ||
        (!estimator.state().valid && !waiting_for_initialization)) {
      throw std::runtime_error(
          "IMU sample committed without valid propagation at CSV line " +
          std::to_string(line_number) + ": " + ReasonName(integrity.reason) +
          ": " + integrity.message);
    }
    WriteState(output, estimator, receive_time, ReasonName(result.reason),
               ReasonName(integrity.reason), ReasonName(precision.reason));
  }
  if (imu_samples == 0 || wheel_samples == 0 || !initialized) {
    throw std::runtime_error(
        "replay did not initialize with both sensor streams");
  }
  output.flush();
  if (!output) {
    throw std::runtime_error("failed writing trajectory CSV");
  }
  if (diagnostics.is_open()) {
    diagnostics.flush();
  }
  std::cerr << "Replay accepted " << imu_samples << " IMU and " << wheel_samples
            << " chassis-speed samples; initialized=" << initialized
            << "; wheel_variance=" << wheel_variance
            << "; NHC=" << use_nonholonomic_constraint
            << "; wheel_updates=" << use_wheel_updates
            << "; initial_gyro_bias_std=" << initial_gyro_bias_std
            << "; wheel_bias_feedback=" << bias_feedback
            << "; stationary_duration=" << stationary_duration
            << "; offline_diagnostic_seed=" << !seed_path.empty() << '\n';
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 9 || (argc - 9) % 2 != 0 || std::string(argv[1]) != "--events" ||
      std::string(argv[3]) != "--trajectory" ||
      std::string(argv[5]) != "--wheel-variance" ||
      std::string(argv[7]) != "--use-nhc") {
    std::cerr << "usage: local_estimator_replay --events events.csv "
                 "--trajectory trajectory.csv --wheel-variance variance "
                 "--use-nhc true|false [--wheel-updates true|false] "
                 "[--diagnostics diagnostics.csv] "
                 "[--initial-gyro-bias-std rad_per_s] "
                 "[--wheel-bias-feedback all|gyro|accel|none|forward] "
                 "[--diagnostic-stationary-duration seconds] "
                 "[--offline-diagnostic-seed seed.csv]\n";
    return 2;
  }
  try {
    const std::string nhc = argv[8];
    if (nhc != "true" && nhc != "false") {
      throw std::runtime_error("--use-nhc must be true or false");
    }
    bool use_wheel_updates = true;
    bool wheel_updates_seen = false;
    std::string diagnostics_path;
    bool diagnostics_seen = false;
    double initial_gyro_bias_std = LocalConfig{}.initial_gyro_bias_std;
    bool initial_gyro_bias_std_seen = false;
    std::string bias_feedback = "all";
    bool bias_feedback_seen = false;
    double stationary_duration = 2.0;
    bool stationary_duration_seen = false;
    std::string seed_path;
    bool seed_seen = false;
    for (int argument = 9; argument < argc; argument += 2) {
      const std::string option = argv[argument];
      const std::string value = argv[argument + 1];
      if (option == "--wheel-updates") {
        if (wheel_updates_seen) {
          throw std::runtime_error("duplicate option: " + option);
        }
        wheel_updates_seen = true;
        if (value != "true" && value != "false") {
          throw std::runtime_error("--wheel-updates must be true or false");
        }
        use_wheel_updates = value == "true";
      } else if (option == "--diagnostics" && !diagnostics_seen) {
        diagnostics_seen = true;
        if (value.empty()) {
          throw std::runtime_error("--diagnostics path must be non-empty");
        }
        diagnostics_path = value;
      } else if (option == "--initial-gyro-bias-std" &&
                 !initial_gyro_bias_std_seen) {
        initial_gyro_bias_std_seen = true;
        std::size_t parsed = 0;
        initial_gyro_bias_std = std::stod(value, &parsed);
        if (parsed != value.size() || !(initial_gyro_bias_std > 0.0) ||
            !std::isfinite(initial_gyro_bias_std)) {
          throw std::runtime_error(
              "--initial-gyro-bias-std must be finite and positive");
        }
      } else if (option == "--wheel-bias-feedback" && !bias_feedback_seen) {
        bias_feedback_seen = true;
        if (value != "all" && value != "gyro" && value != "accel" &&
            value != "none" && value != "forward") {
          throw std::runtime_error(
              "--wheel-bias-feedback must be all, gyro, accel, none, or "
              "forward");
        }
        bias_feedback = value;
      } else if (option == "--diagnostic-stationary-duration" &&
                 !stationary_duration_seen) {
        stationary_duration_seen = true;
        stationary_duration = ParseDouble(value, option, 0);
        if (!(stationary_duration > 0.0)) {
          throw std::runtime_error(
              "diagnostic stationary duration must be positive");
        }
      } else if (option == "--offline-diagnostic-seed" && !seed_seen) {
        seed_seen = true;
        if (value.empty()) {
          throw std::runtime_error("offline diagnostic seed path is empty");
        }
        seed_path = value;
      } else {
        throw std::runtime_error("unknown or duplicate option: " + option);
      }
    }
    return Run(argv[2], argv[4], std::stod(argv[6]), nhc == "true",
               use_wheel_updates, diagnostics_path, initial_gyro_bias_std,
               bias_feedback, stationary_duration, seed_path);
  } catch (const std::exception& error) {
    std::cerr << "local_estimator_replay: " << error.what() << '\n';
    return 1;
  }
}
