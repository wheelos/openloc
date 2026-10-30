// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "openloc/localization/local/native_motion_bootstrap.h"
#include "openloc/localization/local/native_odometry.h"
#include "test/integration/map/native_diagnostic_executor.h"
#include "test/integration/map/native_health_diagnostic.h"
#include "test/integration/map/native_joint_query_probe.h"
#include "test/integration/map/native_lidar_scheduler.h"
#include "test/integration/map/native_map_diagnostic.h"
#include "test/integration/map/native_scan_reader.h"
#include "test/integration/map/native_structured_closure.h"
#include "test/integration/map/native_timer_output.h"
#include "test/integration/map/replay_csv.h"

namespace {
namespace u = openloc::localization::unified;
namespace csv = openloc::replay;
using Clock = std::chrono::steady_clock;

double Seconds(const std::string& text) {
  return static_cast<double>(csv::Time(text)) / 1e9;
}

void WriteImuTrace(std::ostream& stream, const u::LocalEstimatorTrace& trace) {
  if (trace.kind != u::LocalEstimatorTraceKind::IMU_PROPAGATION)
    throw std::runtime_error("raw native replay received a non-IMU trace");
  stream << std::setprecision(17) << trace.stamp.sequence << ','
         << trace.stamp.time << ',' << trace.dt;
  for (const auto* vector :
       {&trace.specific_force_body, &trace.angular_rate_body,
        &trace.linear_acceleration_local, &trace.position_before,
        &trace.position_after, &trace.velocity_before, &trace.velocity_after,
        &trace.gyro_bias_before, &trace.accel_bias_before}) {
    stream << ','
           << vector->transpose().format(
                  Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","));
  }
  stream << ','
         << trace.orientation_midpoint.coeffs().transpose().format(
                Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","));
  for (int offset : {0, 3, 6, 9, 12})
    stream << ',' << trace.covariance_after.block<3, 3>(offset, offset).trace();
  for (int offset : {3, 6, 9, 12})
    stream << ',' << trace.covariance_after.block<3, 3>(offset, 0).norm();
  stream << '\n';
  if (!stream) throw std::runtime_error("incomplete IMU trace write");
}

void WriteCovarianceTrace(std::ostream& stream,
                          const u::LocalEstimatorTrace& trace,
                          const u::LocalState& internal_before,
                          const u::LocalState& output_before,
                          const u::LocalState& internal_after,
                          const u::LocalState& output_after, double scale) {
  const Eigen::Isometry3d propagated = u::Pose(output_before) *
                                       u::Pose(internal_before).inverse() *
                                       u::Pose(internal_after);
  const Eigen::Vector3d translation =
      output_after.position - propagated.translation();
  const Eigen::Vector3d velocity =
      output_after.velocity -
      (propagated.translation() - output_before.position) / trace.dt;
  const Eigen::Vector3d rotation =
      u::LogRotation(Eigen::Quaterniond(propagated.linear()).conjugate() *
                     output_after.orientation);
  stream << std::setprecision(17) << trace.stamp.sequence << ','
         << trace.stamp.time << ',' << trace.dt << ',' << scale << ','
         << output_after.covariance_model_valid;
  for (const auto* matrix :
       {&internal_before.covariance, &internal_after.covariance,
        &output_before.covariance, &output_after.covariance,
        &trace.error_transition, &trace.process_noise_covariance}) {
    stream << ',';
    for (int row = 0; row < 15; ++row)
      for (int col = 0; col < 15; ++col) {
        if (row || col) stream << ';';
        stream << (*matrix)(row, col);
      }
  }
  for (const auto* vector :
       {&translation, &velocity, &rotation, &output_after.position}) {
    stream << ','
           << vector->transpose().format(
                  Eigen::IOFormat(Eigen::FullPrecision, 0, ";", ";"));
  }
  stream << ','
         << output_after.orientation.coeffs().transpose().format(
                Eigen::IOFormat(Eigen::FullPrecision, 0, ";", ";"))
         << '\n';
  if (!stream) throw std::runtime_error("incomplete covariance trace write");
}

void WriteLocalCorrection(std::ostream& stream, uint64_t sequence,
                          int64_t stamp, const u::Result& decision,
                          const u::LocalState& before,
                          const u::LocalState& after,
                          const u::LocalState& output) {
  stream << sequence << ',' << stamp << ',' << u::ReasonName(decision.reason);
  for (const Eigen::Vector3d& vector :
       {Eigen::Vector3d(after.position - before.position),
        Eigen::Vector3d(after.velocity - before.velocity)})
    stream << ','
           << vector.transpose().format(
                  Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","));
  stream << ','
         << u::LogRotation(before.orientation.conjugate() * after.orientation)
                .norm()
         << ',' << after.gyro_bias.norm() << ',' << after.accel_bias.norm();
  for (const auto* vector :
       {&after.position, &output.position, &after.velocity, &output.velocity})
    stream << ','
           << vector->transpose().format(
                  Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","));
  stream << '\n';
}

u::LocalConfig Configuration(const std::string& path) {
  u::LocalConfig c;
  c.session = "native-raw-lio-diagnostic";
  c.allow_imu_only_initialization = true;
  c.enable_lidar = true;
  c.lidar_frame = "livox_frame";
  c.lidar_calibration_id = "mid360-manual-T_I_L";
  std::map<std::string, double*> doubles{
      {"gyro_noise", &c.gyro_noise},
      {"accel_noise", &c.accel_noise},
      {"gyro_bias_noise", &c.gyro_bias_noise},
      {"accel_bias_noise", &c.accel_bias_noise},
      {"point_noise_std", &c.lidar_point_noise_std},
      {"plane_noise_std", &c.lidar_plane_noise_std},
      {"voxel_size", &c.lidar_voxel_size},
      {"max_correspondence_distance", &c.lidar_max_correspondence_distance},
      {"observed_singular_value", &c.lidar_observed_singular_value},
      {"innovation_gate", &c.lidar_innovation_gate},
      {"max_scan_age", &c.lidar_max_scan_age},
      {"translation_correction_rate", &c.lidar_max_translation_correction_rate},
      {"rotation_correction_rate", &c.lidar_max_rotation_correction_rate},
      {"velocity_correction_acceleration",
       &c.lidar_max_velocity_correction_acceleration},
      {"anchor_variance_scale", &c.lidar_anchor_variance_scale},
      {"correction_variance_scale", &c.lidar_correction_variance_scale},
      {"max_translation_correction", &c.lidar_max_translation_correction},
      {"max_rotation_correction", &c.lidar_max_rotation_correction},
      {"max_velocity_correction", &c.lidar_max_velocity_correction},
      {"process_noise_psd_tolerance", &c.lidar_process_noise_psd_tolerance},
      {"rotation_length", &c.lidar_rotation_length}};
  std::map<std::string, uint32_t*> integers{
      {"max_voxels", &c.lidar_max_voxels},
      {"max_scans", &c.lidar_max_scans},
      {"min_points_per_plane", &c.lidar_min_points_per_plane},
      {"min_correspondences", &c.lidar_min_correspondences},
      {"max_points", &c.lidar_max_points},
      {"max_update_iterations", &c.lidar_max_update_iterations}};
  for (const auto& row : csv::ReadRows(path, "name,value", 2)) {
    const auto real = doubles.find(row[0]);
    const auto integer = integers.find(row[0]);
    if (real != doubles.end()) {
      *real->second = csv::Number(row[1]);
      doubles.erase(real);
    } else if (integer != integers.end()) {
      const auto value = csv::Time(row[1]);
      if (value > UINT32_MAX)
        throw std::runtime_error("config integer overflow");
      *integer->second = static_cast<uint32_t>(value);
      integers.erase(integer);
    } else {
      throw std::runtime_error("unknown or duplicate config key: " + row[0]);
    }
  }
  if (!doubles.empty() || !integers.empty())
    throw std::runtime_error("incomplete native LIO configuration");
  return c;
}

void Run(const std::filesystem::path& input,
         const std::filesystem::path& output, bool motion_bootstrap,
         const std::filesystem::path& map_bundle,
         const std::string& map_digests,
         const std::filesystem::path& gnss_bundle,
         const std::string& gnss_digests, int64_t global_holdback_ns,
         bool motion_translation_deskew, bool lidar_stationary_initialization,
         double motion_bootstrap_voxel_size, bool imu_trace, bool async_global,
         bool arrival_paced, int64_t async_local_delay_ns, bool prefetch_lidar,
         bool timer_output, bool level_odom_gnss_bootstrap,
         double native_joint_sample_period,
         const std::map<std::string, uint32_t>& native_joint_limits,
         const openloc::replay::NativeGlobalOdometryFault& odometry_fault,
         std::pair<int64_t, int64_t> map_holdout, uint32_t arrival_rate,
         bool tracking_gain, bool profile_map_timing, bool profile_local_timing,
         bool fixed_gain_update, bool native_query_probe,
         bool native_structured,
         const std::filesystem::path& structured_selection) {
  if (!std::filesystem::create_directory(output))
    throw std::runtime_error("output must be a new directory");
  auto config = Configuration((input / "config.csv").string());
  config.diagnostic_lidar_tracking_gain = tracking_gain;
  if (tracking_gain) {
    std::ofstream policy(output / "local_gain_policy.csv");
    policy << "mode,uncertainty_export,trust_ceiling\n"
              "diagnostic_conditional_gain,original_conservative_marginal,diagnostic\n";
    if (!policy) throw std::runtime_error("incomplete diagnostic gain policy");
  }
  config.enable_motion_initialization = motion_bootstrap;
  config.enable_lidar_stationary_initialization =
      lidar_stationary_initialization;
  config.enable_native_joint_error = native_joint_sample_period > 0;
  config.enable_native_structured_error = native_structured;
  config.native_imu_sample_period = native_joint_sample_period;
  for (const auto& limit : native_joint_limits) {
    if (limit.first == "--native-joint-max-dimension")
      config.native_joint_max_dimension = limit.second;
    else if (limit.first == "--native-joint-max-memory-rows")
      config.native_joint_max_memory_rows = limit.second;
    else
      config.native_joint_max_snapshots = limit.second;
  }
  openloc::localization::NativeOdometry local(config);
  openloc::replay::NativeHealthDiagnostic health(output);
  std::unique_ptr<openloc::replay::NativeMapDiagnostic> map;
  if (!map_bundle.empty() || !gnss_bundle.empty())
    map = std::make_unique<openloc::replay::NativeMapDiagnostic>(
        map_bundle, output, map_digests, gnss_bundle, gnss_digests,
        global_holdback_ns, async_global, level_odom_gnss_bootstrap,
        odometry_fault, map_holdout, profile_map_timing, fixed_gain_update,
        !structured_selection.empty());
  std::unique_ptr<openloc::replay::NativeStructuredClosure> closure;
  if (!structured_selection.empty())
    closure = std::make_unique<openloc::replay::NativeStructuredClosure>(
        structured_selection, gnss_bundle, output, map.get());
  if (fixed_gain_update) {
    std::ofstream policy(output / "global_mean_policy.csv");
    policy << "mode,covariance,trust_ceiling\n"
              "fixed_half_residual,unknown_correlation_bound,diagnostic\n";
    if (!policy) throw std::runtime_error("incomplete diagnostic mean policy");
  }
  std::atomic<bool> global_initialized{false};
  std::unique_ptr<openloc::replay::NativeDiagnosticExecutor> global_executor;
  if (async_global)
    global_executor =
        std::make_unique<openloc::replay::NativeDiagnosticExecutor>(
            (output / "global_scheduler.csv").string(), 256, 64 * 1024 * 1024);
  auto submit_global = [&](const char* kind, std::size_t bytes,
                           std::function<void()> run) {
    if (global_executor) {
      global_executor->Submit(kind, bytes, std::move(run));
    } else {
      run();
    }
  };
  openloc::localization::NativeMotionBootstrap bootstrap(
      motion_translation_deskew, motion_bootstrap_voxel_size);
  std::unique_ptr<openloc::replay::NativeLidarScheduler> local_scheduler;
  if (async_local_delay_ns)
    local_scheduler = std::make_unique<openloc::replay::NativeLidarScheduler>(
        output, async_local_delay_ns);
  const auto configured = local.ValidateConfig();
  if (!configured.ok()) throw std::runtime_error(configured.message);
  const auto imu =
      csv::ReadRows((input / "imu.csv").string(),
                    "timestamp_ns,receive_ns,ax,ay,az,gx,gy,gz", 8);
  const auto scans = csv::ReadRows((input / "scans.csv").string(),
                                   "end_ns,receive_ns,points", 3);
  const auto extrinsic =
      csv::ReadRows((input / "extrinsic.csv").string(), "tx,ty,tz", 3);
  if (imu.empty() || scans.empty() || extrinsic.size() != 1)
    throw std::runtime_error("raw IMU, scans and one extrinsic are required");
  std::unique_ptr<openloc::replay::NativeScanReader> scan_reader;
  if (prefetch_lidar) {
    std::vector<std::string> names;
    for (const auto& scan : scans) names.push_back(scan[2]);
    scan_reader = std::make_unique<openloc::replay::NativeScanReader>(
        input, std::move(names), config.lidar_max_points, output);
  }
  Eigen::Isometry3d base_from_lidar = Eigen::Isometry3d::Identity();
  for (int axis = 0; axis < 3; ++axis)
    base_from_lidar.translation()[axis] = csv::Number(extrinsic[0][axis]);
  std::ofstream states(output / "odometry.csv");
  std::ofstream decisions(output / "lidar.csv");
  std::ofstream initialization(output / "motion_initialization.csv");
  std::ofstream capability(output / "capability.csv");
  std::ofstream corrections(output / "local_corrections.csv");
  std::ofstream joint_audit;
  std::ofstream native_resources;
  std::ofstream local_profile;
  if (profile_local_timing) {
    local_profile.open(output / "local_profile.csv");
    if (!local_profile) throw std::runtime_error("cannot open local timing profile");
    local_profile << std::setprecision(17)
                  << "sequence,frontier_ns,bootstrap,points,decode_ms,"
                     "estimator_ms,decode_start_ns,decode_end_ns,"
                     "estimator_start_ns,estimator_end_ns\n";
    if (native_structured) {
      native_resources.open(output / "native_resources.csv");
      native_resources << std::setprecision(17)
                       << "sequence,timestamp_ns,primitive_sources,primitive_rows,"
                          "memory_keys,snapshots,publish_queries,publish_query_ms,"
                          "retired_registry_sources,retired_registry_rows\n";
      if (!native_resources)
        throw std::runtime_error("cannot open structured resource journal");
    }
  }
  if (config.enable_native_joint_error) {
    joint_audit.open(output / "native_joint.csv");
    if (!joint_audit) throw std::runtime_error("cannot open native joint audit");
    joint_audit
        << "sequence,timestamp_ns,receive_ns,reason,detail,dimension,"
           "imu_sample_period_s,maximum_dimension,maximum_memory_rows,"
           "maximum_snapshots,history_duration_s,diagnostic_only,trusted";
    if (native_structured) joint_audit << ",representation";
    joint_audit << '\n';
  }
  std::ofstream host_timing;
  if (arrival_paced) {
    host_timing.open(output / "host_timing.csv");
    if (!host_timing)
      throw std::runtime_error("cannot open host timing journal");
    host_timing << "timestamp_ns,receive_ns,valid,target_steady_ns,"
                   "emitted_steady_ns,deadline_ns\n";
  }
  std::ofstream trace;
  std::ofstream covariance_trace;
  std::optional<u::LocalEstimatorTrace> propagation_trace;
  if (imu_trace) {
    trace.open(output / "imu_trace.csv");
    if (!trace) throw std::runtime_error("cannot open IMU trace");
    covariance_trace.open(output / "local_covariance.csv");
    if (!covariance_trace)
      throw std::runtime_error("cannot open local covariance trace");
    covariance_trace
        << "sequence,time_s,dt_s,correction_variance_scale,"
           "covariance_model_valid,internal_before,internal_after,"
           "output_before,output_after,transition,process_noise,"
           "translation_correction,velocity_correction,rotation_correction,"
           "output_position,output_quaternion_xyzw\n";
    trace
        << "sequence,time_s,dt_s,force_x,force_y,force_z,rate_x,rate_y,rate_z,"
           "acceleration_x,acceleration_y,acceleration_z,"
           "position_before_x,position_before_y,position_before_z,"
           "position_after_x,position_after_y,position_after_z,"
           "velocity_before_x,velocity_before_y,velocity_before_z,"
           "velocity_after_x,velocity_after_y,velocity_after_z,"
           "gyro_bias_x,gyro_bias_y,gyro_bias_z,"
           "accel_bias_x,accel_bias_y,accel_bias_z,"
           "midpoint_qx,midpoint_qy,midpoint_qz,midpoint_qw,"
           "position_covariance_trace,velocity_covariance_trace,"
           "attitude_covariance_trace,gyro_bias_covariance_trace,"
           "accel_bias_covariance_trace,velocity_position_cross_norm,"
           "attitude_position_cross_norm,gyro_bias_position_cross_norm,"
           "accel_bias_position_cross_norm\n";
    local.SetTraceCallback([&](const u::LocalEstimatorTrace& item) {
      WriteImuTrace(trace, item);
      propagation_trace = item;
    });
  }
  if (!states || !decisions || !initialization || !capability || !corrections)
    throw std::runtime_error("cannot open outputs");
  states << std::setprecision(17)
         << "timestamp_ns,receive_ns,valid,px,py,pz,qx,qy,qz,qw,vx,vy,vz,"
            "covariance_model_valid,motion_available,imu_ms,integrity_reason,"
            "capability_reason\n";
  decisions << std::setprecision(17)
            << "sequence,raw_end_ns,source_receive_ns,reference_ns,"
               "processing_frontier_receive_ns,reason,detail,"
               "observed_rank,processing_ms,bootstrap_stage\n";
  initialization << std::setprecision(17)
                 << "sequence,reference_ns,reason,linear_system_evaluated,"
                    "solution_available,minimum_scaled_singular_value,"
                    "gravity_error_mps2,accel_bias_mps2,gyro_bias_radps,"
                    "normalized_rms,angular_rms_radps,translation_deskew,"
                    "deskew_velocity_x,deskew_velocity_y,deskew_velocity_z,"
                    "lidar_stationary_initialization,maximum_lidar_speed,"
                    "maximum_lidar_angular_speed,"
                    "maximum_stationary_force_residual,bootstrap_voxel_m\n";
  capability << "timestamp_ns,receive_ns,valid,integrity_reason,"
                "capability_reason,detail\n";
  corrections << std::setprecision(17)
              << "sequence,reference_ns,reason,position_dx,position_dy,"
                 "position_dz,velocity_dx,velocity_dy,velocity_dz,"
                 "rotation_rad,gyro_bias_norm,accel_bias_norm,"
                 "internal_px,internal_py,internal_pz,output_px,output_py,"
                 "output_pz,internal_vx,internal_vy,internal_vz,"
                 "output_vx,output_vy,output_vz\n";
  std::size_t next_scan = 0;
  bool native_query_written = false;
  uint64_t initialized_ns = 0;
  uint64_t valid_states = 0;
  uint64_t lidar_updates = 0;
  uint64_t warmup_scans = 0;
  int64_t last_imu = 0;
  int64_t last_receive = 0;
  const auto arrival_origin = Clock::now();
  const int64_t first_receive = csv::Time(imu.front()[1]);
  if (arrival_paced) {
    std::ofstream clock(output / "replay_clock.csv");
    clock << "first_receive_ns,arrival_rate,scope\n"
          << first_receive << ',' << arrival_rate
          << ",wall_schedule_only_original_sensor_timestamps_preserved\n";
    if (!clock) throw std::runtime_error("incomplete replay clock declaration");
  }
  std::ofstream timer_journal;
  std::unique_ptr<csv::NativeTimerOutput> timer;
  if (timer_output) {
    timer_journal.open(output / "timer_output.csv");
    if (!timer_journal) throw std::runtime_error("cannot open timer output");
    timer_journal
        << std::setprecision(17)
        << "publication_sequence,target_steady_ns,emitted_steady_ns,"
           "skipped_periods,usable,new_measurement,epoch_changed,"
           "measurement_ns,receive_ns,source_sequence,session,generation,"
           "px,py,pz,qx,qy,qz,qw,vx,vy,vz,measurement_age_ns,"
           "maximum_age_ns,diagnostic_only,trusted\n";
    timer = std::make_unique<csv::NativeTimerOutput>(
        [&, sequence = uint64_t{0}](
            const csv::NativeTimerOutput::Publication& p) mutable {
          timer_journal << ++sequence << ',' << p.target_steady_ns << ','
                        << p.emitted_steady_ns << ',' << p.skipped_periods
                        << ',' << p.usable << ',' << p.new_measurement << ','
                        << p.epoch_changed;
          if (p.snapshot) {
            const auto& snapshot = *p.snapshot;
            timer_journal << ',' << snapshot.measurement_ns << ','
                          << snapshot.receive_ns << ','
                          << snapshot.state.stamp.sequence << ',';
            csv::Text(timer_journal, snapshot.state.epoch.session);
            timer_journal << ',' << snapshot.state.epoch.generation;
          } else {
            timer_journal << ",,,,,";
          }
          for (int axis = 0; axis < 10; ++axis) {
            timer_journal << ',';
            if (p.usable) {
              const auto& state = p.snapshot->state;
              if (axis < 3)
                timer_journal << state.position[axis];
              else if (axis < 7)
                timer_journal << state.orientation.coeffs()[axis - 3];
              else
                timer_journal << state.velocity[axis - 7];
            }
          }
          timer_journal << ',' << p.measurement_age_ns << ','
                        << static_cast<int64_t>(config.max_sample_age * 1e9)
                        << ",1,0\n";
          timer_journal.flush();
          if (!timer_journal)
            throw std::runtime_error("incomplete timer output");
        },
        static_cast<int64_t>(config.max_sample_age * 1e9));
  }
  for (std::size_t index = 0; index < imu.size(); ++index) {
    const auto& row = imu[index];
    const int64_t stamp = csv::Time(row[0]);
    const int64_t receive = csv::Time(row[1]);
    if (stamp <= last_imu || receive < last_receive)
      throw std::runtime_error("IMU measurement/arrival order regressed");
    last_imu = stamp;
    last_receive = receive;
    if (global_executor) global_executor->Check();
    const auto target =
        arrival_origin +
        std::chrono::nanoseconds((receive - first_receive) / arrival_rate);
    if (arrival_paced) std::this_thread::sleep_until(target);
    u::ImuSample sample;
    sample.stamp = {Seconds(row[0]), Seconds(row[1]), index + 1, "unix"};
    for (int axis = 0; axis < 3; ++axis) {
      sample.acceleration[axis] = csv::Number(row[axis + 2]);
      sample.angular_velocity[axis] = csv::Number(row[axis + 5]);
    }
    u::LocalState internal_before, output_before;
    if (imu_trace) {
      internal_before = local.internal_state();
      output_before = local.state();
      propagation_trace.reset();
    }
    auto started = Clock::now();
    const auto result = local.AddImu(sample);
    const double imu_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - started)
            .count();
    const auto integrity = local.EvaluateIntegrity(sample.stamp.receive_time);
    if (!result.ok()) {
      capability << stamp << ',' << receive << ',' << local.state().valid << ','
                 << u::ReasonName(integrity.reason) << ','
                 << u::ReasonName(result.reason) << ',';
      csv::Text(capability, result.message);
      capability << '\n';
      capability.flush();
    }
    if (!result.ok() &&
        !((result.reason == u::Reason::WHEEL_STALE ||
           result.reason == u::Reason::CORRELATION_UNQUALIFIED ||
           result.reason == u::Reason::CORRECTION_BUDGET_EXCEEDED) &&
          integrity.ok()))
      throw std::runtime_error(result.message);
    const auto& state = local.state();
    if (imu_trace && state.valid && output_before.valid && propagation_trace) {
      if (propagation_trace->stamp.sequence != sample.stamp.sequence)
        throw std::runtime_error("covariance trace frontier mismatch");
      WriteCovarianceTrace(covariance_trace, *propagation_trace,
                           internal_before, output_before,
                           local.internal_state(), state,
                           config.lidar_correction_variance_scale);
    }
    if (timer) timer->Update(state, integrity.ok(), stamp, receive, target);
    health.Update(state, integrity, stamp, receive, global_initialized.load());
    if (motion_bootstrap && !state.valid) bootstrap.AddImu(sample);
    if (state.valid) {
      ++valid_states;
      if (initialized_ns == 0) initialized_ns = stamp;
      if (map && !closure) {
        submit_global("odometry", sizeof(state),
                      [&, snapshot = state, stamp, receive] {
                        map->AddOdometry(snapshot, stamp, receive);
                        global_initialized.store(map->initialized());
                      });
      }
    }
    if (map && !closure) {
      submit_global("gnss", sizeof(receive), [&, receive] {
        map->ProcessGnss(receive);
        global_initialized.store(map->initialized());
      });
    }
    states << stamp << ',' << receive << ',' << state.valid << ','
           << state.position.transpose().format(
                  Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","))
           << ','
           << state.orientation.coeffs().transpose().format(
                  Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","))
           << ','
           << state.velocity.transpose().format(
                  Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","))
           << ',' << state.covariance_model_valid << ','
           << local.motion_available() << ',' << imu_ms << ','
           << u::ReasonName(integrity.reason) << ','
           << u::ReasonName(result.reason) << '\n';
    if (arrival_paced) {
      states.flush();
      if (!states) throw std::runtime_error("incomplete paced ODOM write");
      const auto emitted = Clock::now();
      const auto to_ns = [](Clock::time_point time) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   time.time_since_epoch())
            .count();
      };
      host_timing << stamp << ',' << receive << ',' << state.valid << ','
                  << to_ns(target) << ',' << to_ns(emitted) << ",10000000\n";
    }
    if (local_scheduler) {
      const auto completion = local_scheduler->Advance(&local, sample, stamp);
      if (completion) {
        WriteLocalCorrection(corrections, completion->sequence, stamp,
                             completion->decision, completion->before,
                             completion->after, local.state());
        if (completion->applied && completion->decision.ok()) ++lidar_updates;
        if (!completion->applied &&
            completion->decision.reason != u::Reason::LIDAR_STALE &&
            completion->decision.reason != u::Reason::GEOMETRY_UNAVAILABLE)
          throw std::runtime_error(completion->decision.message);
      }
    }
    // Re-reference a received scan to a real committed IMU frontier. No
    // synthetic IMU sample or nearest-state substitution is introduced.
    while (next_scan < scans.size() &&
           csv::Time(scans[next_scan][1]) <= receive &&
           csv::Time(scans[next_scan][0]) <= stamp) {
      std::optional<openloc::replay::NativeScanReader::Decoded> decoded;
      if (scan_reader && (state.valid || motion_bootstrap)) {
        decoded = scan_reader->Take(next_scan, stamp);
        if (!decoded) break;
      }
      const auto& source = scans[next_scan];
      const auto sequence = ++next_scan;
      u::Result decision;
      u::MotionInitializationDiagnostics motion_diagnostics;
      double elapsed = 0.0;
      double decode_ms = 0.0;
      std::size_t decoded_points = 0;
      Clock::time_point decode_start, decode_end, estimator_start, estimator_end;
      const bool bootstrapping = !state.valid;
      if (bootstrapping && !motion_bootstrap) {
        ++warmup_scans;
        decision = {u::Reason::WAITING_FOR_STATIONARY,
                    "Waiting for native IMU initialization"};
      } else {
        u::LidarScan scan;
        scan.stamp = sample.stamp;
        scan.stamp.sequence = sequence;
        scan.epoch = state.epoch;
        scan.frame_id = config.lidar_frame;
        scan.calibration_id = config.lidar_calibration_id;
        scan.base_from_lidar = base_from_lidar;
        if (profile_local_timing) decode_start = Clock::now();
        if (decoded) {
          scan.points = std::move(decoded->points);
        } else {
          const auto points = csv::ReadRows((input / source[2]).string(),
                                            "timestamp_ns,x,y,z", 4);
          for (const auto& point : points) {
            scan.points.push_back(
                {{csv::Number(point[1]), csv::Number(point[2]),
                  csv::Number(point[3])},
                 Seconds(point[0])});
          }
          if (profile_local_timing) {
            decode_end = Clock::now();
            decode_ms = std::chrono::duration<double, std::milli>(
                            decode_end - decode_start).count();
            decoded_points = scan.points.size();
          }
        }
        if (scan.points.empty()) throw std::runtime_error("empty raw scan");
        if (bootstrapping) {
          started = Clock::now();
          estimator_start = started;
          decision = bootstrap.Observe(scan, &local, &motion_diagnostics);
          if (profile_local_timing) estimator_end = Clock::now();
          elapsed =
              std::chrono::duration<double, std::milli>(Clock::now() - started)
                  .count();
          ++warmup_scans;
          if (decision.ok()) initialized_ns = stamp;
        } else if (scan.points.front().time <
                   static_cast<double>(initialized_ns) / 1e9) {
          ++warmup_scans;
          decision = {u::Reason::HISTORY_UNAVAILABLE,
                      "Scan overlaps pre-initialization history"};
        } else {
          if (native_query_probe && !native_query_written) {
            u::JointLidarInputErrorMap query;
            const auto modeled = local.JointLidarInputErrorMap(scan, &query);
            if (!modeled.ok()) throw std::runtime_error(modeled.message);
            csv::WriteNativeJointQueryProbe(output, query);
            native_query_written = true;
          }
          const u::LocalState before = local.internal_state();
          started = Clock::now();
          estimator_start = started;
          decision = local_scheduler
                         ? local_scheduler->Submit(&local, scan, stamp)
                         : local.AddLidar(scan);
          if (profile_local_timing) estimator_end = Clock::now();
          elapsed =
              std::chrono::duration<double, std::milli>(Clock::now() - started)
                  .count();
          const auto& after = local.internal_state();
          if (!local_scheduler) {
            WriteLocalCorrection(corrections, sequence, stamp, decision, before,
                                 after, state);
            if (decision.ok()) ++lidar_updates;
          }
          // Use the pre-LIO continuous-output frontier already submitted to
          // the global owner, not a mixture with corrected internal history.
          if (map) {
            if (closure) {
              closure->Observe(&local, sequence, csv::Time(source[0]), stamp,
                               receive);
            } else {
              const std::size_t bytes =
                  sizeof(scan) + scan.points.size() * sizeof(u::TimedLidarPoint);
              submit_global("scan", bytes, [&, snapshot = scan, stamp, receive] {
                map->Observe(snapshot, stamp, receive);
                global_initialized.store(map->initialized());
              });
            }
          }
        }
      }
      if (profile_local_timing) {
        const auto ns = [](Clock::time_point time) {
          return std::chrono::duration_cast<std::chrono::nanoseconds>(
                     time.time_since_epoch()).count();
        };
        local_profile << sequence << ',' << stamp << ',' << bootstrapping << ','
                      << decoded_points << ',' << decode_ms << ',' << elapsed
                      << ',' << ns(decode_start) << ',' << ns(decode_end) << ','
                      << ns(estimator_start) << ',' << ns(estimator_end) << '\n';
        if (!local_profile)
          throw std::runtime_error("incomplete local timing profile");
      }
      decisions << sequence << ',' << source[0] << ',' << source[1] << ','
                << stamp << ',' << receive << ','
                << u::ReasonName(decision.reason) << ',';
      csv::Text(decisions, decision.message);
      decisions << ','
                << (local_scheduler ? 0
                                    : local.lidar_observed_pose_basis().rows())
                << ',' << elapsed << ',' << bootstrapping << '\n';
      decisions.flush();
      if (bootstrapping && motion_bootstrap) {
        initialization
            << sequence << ',' << stamp << ',' << u::ReasonName(decision.reason)
            << ',' << motion_diagnostics.linear_system_evaluated << ','
            << motion_diagnostics.solution_available << ','
            << motion_diagnostics.minimum_scaled_singular_value << ','
            << motion_diagnostics.gravity_error << ','
            << motion_diagnostics.accel_bias_norm << ','
            << motion_diagnostics.gyro_bias_norm << ','
            << motion_diagnostics.normalized_residual_rms << ','
            << motion_diagnostics.angular_residual_rms << ','
            << motion_translation_deskew << ','
            << bootstrap.deskew_velocity_in_end_base().transpose().format(
                   Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","))
            << ',' << motion_diagnostics.lidar_stationary_initialization << ','
            << motion_diagnostics.maximum_lidar_speed << ','
            << motion_diagnostics.maximum_lidar_angular_speed << ','
            << motion_diagnostics.maximum_stationary_force_residual << ','
            << motion_bootstrap_voxel_size << '\n';
        initialization.flush();
      }
      if (!decision.ok() &&
          !(local_scheduler &&
            (decision.reason == u::Reason::HISTORY_UNAVAILABLE ||
             decision.reason == u::Reason::LIDAR_STALE)) &&
          decision.reason != u::Reason::WAITING_FOR_STATIONARY &&
          decision.reason != u::Reason::GEOMETRY_UNAVAILABLE &&
          decision.reason != u::Reason::DEGENERATE &&
          !(bootstrapping && motion_bootstrap &&
            (decision.reason == u::Reason::HISTORY_UNAVAILABLE ||
             decision.reason == u::Reason::DEGENERATE ||
             decision.reason == u::Reason::MATCH_FAILED ||
             decision.reason == u::Reason::INNOVATION_REJECTED)) &&
          !(decision.reason == u::Reason::HISTORY_UNAVAILABLE &&
            decision.message == "Scan overlaps pre-initialization history"))
        throw std::runtime_error(decision.message);
    }
    if (closure) closure->AddOdometry(&local, stamp, receive);
    if (map && !closure) {
      submit_global("drain", sizeof(receive), [&, receive] {
        map->DrainGlobal(receive);
        global_initialized.store(map->initialized());
      });
    }
    if (config.enable_native_joint_error && local.state().valid) {
      u::LocalState joint;
      u::StructuredLocalState structured;
      auto assessment = native_structured
          ? local.StructuredErrorLookup(local.state().stamp.time, &structured)
          : local.JointErrorLookup(local.state().stamp.time, &joint);
      const auto& observer_status = local.joint_error_status();
      if (native_structured && !observer_status.ok())
        assessment = observer_status;
      joint_audit << index + 1 << ',' << stamp << ',' << receive << ','
                  << u::ReasonName(assessment.reason) << ',';
      csv::Text(joint_audit, assessment.message);
      joint_audit << ','
                  << (assessment.ok() && !native_structured
                          ? joint.joint_error->covariance.rows() : 0)
                  << ',' << std::setprecision(17)
                  << config.native_imu_sample_period << ','
                  << config.native_joint_max_dimension << ','
                  << config.native_joint_max_memory_rows << ','
                  << config.native_joint_max_snapshots << ','
                  << config.history_duration << ",1,0";
      if (native_structured)
        joint_audit << ",structured_sources_not_dense_graph_state";
      joint_audit << '\n';
      joint_audit.flush();
      if (!joint_audit)
        throw std::runtime_error("incomplete native joint audit");
      if (profile_local_timing && native_structured) {
        const auto usage = local.StructuredErrorResourceUsage();
        native_resources << index + 1 << ',' << stamp << ','
                         << usage.primitive_sources << ',' << usage.primitive_rows
                         << ',' << usage.memory_keys << ',' << usage.snapshots
                         << ',' << usage.publish_queries << ','
                         << usage.publish_query_ms << ','
                         << usage.retired_registry_sources << ','
                         << usage.retired_registry_rows << '\n';
        native_resources.flush();
        if (!native_resources)
          throw std::runtime_error("incomplete structured resource journal");
      }
      if (!assessment.ok())
        throw std::runtime_error("native joint audit: " + assessment.message);
    }
  }
  if (scan_reader) scan_reader->Finish();
  if (local_scheduler) local_scheduler->Finish(last_imu);
  if (closure) closure->Finish();
  if (map && !closure) {
    submit_global("finish", 0, [&] { map->Finish(); });
    if (global_executor) global_executor->Finish();
  }
  if (timer) timer->Finish();
  if (profile_local_timing) {
    local_profile.flush();
    if (!local_profile) throw std::runtime_error("incomplete local timing profile");
  }
  states.flush();
  initialization.flush();
  capability.flush();
  corrections.flush();
  if (arrival_paced) {
    host_timing.flush();
    if (!host_timing) throw std::runtime_error("incomplete host timing write");
  }
  if (imu_trace) {
    trace.flush();
    if (!trace) throw std::runtime_error("incomplete IMU trace write");
  }
  if (!states || !decisions || !initialization || !capability || !corrections)
    throw std::runtime_error("incomplete output write");
  std::ofstream summary(output / "summary.csv");
  summary << "imu_samples,valid_states,scans_consumed,scans_unconsumed,"
             "warmup_scans,lidar_updates,initialized_ns\n"
          << imu.size() << ',' << valid_states << ',' << next_scan << ','
          << scans.size() - next_scan << ',' << warmup_scans << ','
          << lidar_updates << ',' << initialized_ns << '\n';
  summary.close();
  if (!summary) throw std::runtime_error("incomplete summary write");
  if (!valid_states || !lidar_updates)
    throw std::runtime_error(
        "Native initialization/LIO update not established");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 3)
      throw std::runtime_error(
          "usage: native_lio_replay INPUT_DIR NEW_OUTPUT_DIR "
          "[--motion-bootstrap] [--imu-trace] [--diagnostic-map BUNDLE "
          "--map-input-sha256 PCD_IDENTITY_SEED_SHA256_CONCATENATED] "
          "[--global-odom-drift START_NS X_MPS Y_MPS | "
          "--global-odom-step START_NS X_M Y_M]");
    bool motion_bootstrap = false;
    bool tracking_gain = false;
    bool motion_translation_deskew = false;
    bool lidar_stationary_initialization = false;
    double motion_bootstrap_voxel_size = 0.5;
    bool has_bootstrap_voxel_size = false;
    std::filesystem::path map_bundle;
    std::string map_digests;
    std::filesystem::path gnss_bundle;
    std::string gnss_digests;
    int64_t global_holdback_ns = 0;
    bool has_holdback = false;
    bool imu_trace = false;
    bool async_global = false;
    bool arrival_paced = false;
    uint32_t arrival_rate = 1;
    bool has_arrival_rate = false;
    int64_t async_local_delay_ns = 0;
    bool prefetch_lidar = false;
    bool timer_output = false;
    bool profile_map_timing = false;
    bool profile_local_timing = false;
    bool level_odom_gnss_bootstrap = false;
    bool fixed_gain_update = false;
    bool native_query_probe = false;
    bool native_structured = false;
    std::filesystem::path structured_selection;
    double native_joint_sample_period = 0;
    std::map<std::string, uint32_t> native_joint_limits;
    openloc::replay::NativeGlobalOdometryFault odometry_fault;
    std::pair<int64_t, int64_t> map_holdout;
    for (int i = 3; i < argc; ++i) {
      const std::string option(argv[i]);
      if (option == "--diagnostic-lidar-tracking-gain" && !tracking_gain) {
        tracking_gain = true;
        continue;
      }
      if (option == "--motion-bootstrap" && !motion_bootstrap) {
        motion_bootstrap = true;
      } else if (option == "--imu-trace" && !imu_trace) {
        imu_trace = true;
      } else if (option == "--diagnostic-fixed-gain" && !fixed_gain_update) {
        fixed_gain_update = true;
      } else if (option == "--native-joint-query-probe" && !native_query_probe) {
        native_query_probe = true;
      } else if (option == "--native-joint-structured" && !native_structured) {
        native_structured = true;
      } else if (option == "--offline-structured-closure" &&
                 structured_selection.empty() && i + 1 < argc) {
        structured_selection = argv[++i];
      } else if (option == "--global-map-holdout" && !map_holdout.first &&
                 i + 2 < argc) {
        map_holdout.first = csv::Time(argv[++i]);
        map_holdout.second = csv::Time(argv[++i]);
        if (map_holdout.second <= map_holdout.first)
          throw std::runtime_error("map holdout interval must increase");
      } else if (option == "--native-joint-audit-period-s" &&
                 native_joint_sample_period == 0 && i + 1 < argc) {
        native_joint_sample_period = csv::Number(argv[++i]);
        if (native_joint_sample_period <= 0)
          throw std::runtime_error("native joint audit requires a positive IMU period");
      } else if ((option == "--native-joint-max-dimension" ||
                  option == "--native-joint-max-memory-rows" ||
                  option == "--native-joint-max-snapshots") &&
                 !native_joint_limits.count(option) && i + 1 < argc) {
        const auto limit = csv::Time(argv[++i]);
        if (limit == 0 || limit > UINT32_MAX)
          throw std::runtime_error("native joint limit must be a positive uint32");
        native_joint_limits.emplace(option, static_cast<uint32_t>(limit));
      } else if (option == "--async-global" && !async_global) {
        async_global = true;
      } else if (option == "--arrival-paced" && !arrival_paced) {
        arrival_paced = true;
      } else if (option == "--arrival-rate" && !has_arrival_rate &&
                 i + 1 < argc) {
        const auto rate = csv::Time(argv[++i]);
        if (rate != 1 && rate != 2 && rate != 5)
          throw std::runtime_error("arrival rate must be 1, 2 or 5");
        arrival_rate = static_cast<uint32_t>(rate);
        has_arrival_rate = true;
      } else if (option == "--prefetch-lidar" && !prefetch_lidar) {
        prefetch_lidar = true;
      } else if (option == "--timer-output" && !timer_output) {
        timer_output = true;
      } else if (option == "--profile-map-timing" && !profile_map_timing) {
        profile_map_timing = true;
      } else if (option == "--profile-local-timing" && !profile_local_timing) {
        profile_local_timing = true;
      } else if (option == "--diagnostic-level-odom-gnss-bootstrap" &&
                 !level_odom_gnss_bootstrap) {
        level_odom_gnss_bootstrap = true;
      } else if (option == "--global-odom-drift" &&
                 odometry_fault.kind ==
                     openloc::replay::NativeGlobalOdometryFault::Kind::kNone &&
                 i + 3 < argc) {
        odometry_fault.kind =
            openloc::replay::NativeGlobalOdometryFault::Kind::kDrift;
        odometry_fault.start_ns = csv::Time(argv[++i]);
        const double x_mps = csv::Number(argv[++i]);
        const double y_mps = csv::Number(argv[++i]);
        odometry_fault.value = {x_mps, y_mps};
      } else if (option == "--global-odom-step" &&
                 odometry_fault.kind ==
                     openloc::replay::NativeGlobalOdometryFault::Kind::kNone &&
                 i + 3 < argc) {
        odometry_fault.kind =
            openloc::replay::NativeGlobalOdometryFault::Kind::kStep;
        odometry_fault.start_ns = csv::Time(argv[++i]);
        const double x_m = csv::Number(argv[++i]);
        const double y_m = csv::Number(argv[++i]);
        odometry_fault.value = {x_m, y_m};
      } else if (option == "--async-local-lio-ms" &&
                 async_local_delay_ns == 0 && i + 1 < argc) {
        const auto delay_ms = csv::Time(argv[++i]);
        if (delay_ms < 5 || delay_ms > 100)
          throw std::runtime_error("local LIO delay must be in [5,100] ms");
        async_local_delay_ns = delay_ms * 1000000;
      } else if (option == "--motion-translation-deskew" &&
                 !motion_translation_deskew) {
        motion_translation_deskew = true;
      } else if (option == "--lidar-stationary-initialization" &&
                 !lidar_stationary_initialization) {
        lidar_stationary_initialization = true;
      } else if (option == "--motion-bootstrap-voxel-m" &&
                 !has_bootstrap_voxel_size && i + 1 < argc) {
        motion_bootstrap_voxel_size = csv::Number(argv[++i]);
        has_bootstrap_voxel_size = true;
      } else if (option == "--diagnostic-map" && map_bundle.empty() &&
                 i + 1 < argc) {
        map_bundle = argv[++i];
      } else if (option == "--map-input-sha256" && map_digests.empty() &&
                 i + 1 < argc) {
        map_digests = argv[++i];
      } else if (option == "--diagnostic-gnss" && gnss_bundle.empty() &&
                 i + 1 < argc) {
        gnss_bundle = argv[++i];
      } else if (option == "--gnss-input-sha256" && gnss_digests.empty() &&
                 i + 1 < argc) {
        gnss_digests = argv[++i];
      } else if (option == "--global-holdback-ns" && !has_holdback &&
                 i + 1 < argc) {
        const std::string value(argv[++i]);
        std::size_t used = 0;
        global_holdback_ns = std::stoll(value, &used);
        if (used != value.size() || global_holdback_ns < 0 ||
            global_holdback_ns > 500000000)
          throw std::runtime_error(
              "global holdback must be in [0,500000000] ns");
        has_holdback = true;
      } else {
        throw std::runtime_error("unknown or duplicate replay option");
      }
    }
    if (map_bundle.empty() != map_digests.empty())
      throw std::runtime_error(
          "map bundle and explicit input hashes required together");
    if (fixed_gain_update && map_bundle.empty() && gnss_bundle.empty())
      throw std::runtime_error("diagnostic fixed gain requires a global owner");
    if (profile_map_timing && map_bundle.empty())
      throw std::runtime_error("map timing profiling requires map input");
    if (profile_local_timing && (prefetch_lidar || async_local_delay_ns))
      throw std::runtime_error(
          "local timing profiling requires synchronous decode and LIO");
    if (map_holdout.first && map_bundle.empty())
      throw std::runtime_error("map holdout requires an explicit map");
    if (!native_joint_limits.empty() && native_joint_sample_period == 0)
      throw std::runtime_error("native joint limits require native joint audit");
    if (native_query_probe && native_joint_sample_period == 0)
      throw std::runtime_error("native query probe requires native joint audit");
    if (native_structured && native_joint_sample_period == 0)
      throw std::runtime_error("structured native mode requires native joint audit");
    if (native_structured && native_query_probe)
      throw std::runtime_error(
          "structured native mode cannot export a dense query basis");
    if (!structured_selection.empty() &&
        (!native_structured || map_bundle.empty() || gnss_bundle.empty() ||
         async_global || arrival_paced || async_local_delay_ns || prefetch_lidar ||
         timer_output || fixed_gain_update || tracking_gain ||
         level_odom_gnss_bootstrap || map_holdout.first || has_holdback ||
         odometry_fault.kind !=
             openloc::replay::NativeGlobalOdometryFault::Kind::kNone))
      throw std::runtime_error(
          "offline structured closure requires structured native audit, pinned "
          "map/GNSS inputs and synchronous unmodified native replay");
    if (native_joint_sample_period > 0 &&
        (((!map_bundle.empty() || !gnss_bundle.empty()) &&
          structured_selection.empty()) || arrival_paced ||
         async_local_delay_ns || prefetch_lidar || timer_output))
      throw std::runtime_error(
          "native joint audit requires deterministic local-only replay");
    if (gnss_bundle.empty() != gnss_digests.empty())
      throw std::runtime_error(
          "GNSS bundle and explicit input hashes required together");
    if (level_odom_gnss_bootstrap && gnss_bundle.empty())
      throw std::runtime_error("horizontal bootstrap requires GNSS input");
    if (has_holdback && map_bundle.empty() && gnss_bundle.empty())
      throw std::runtime_error("global holdback requires map or GNSS input");
    if (odometry_fault.kind !=
            openloc::replay::NativeGlobalOdometryFault::Kind::kNone &&
        map_bundle.empty() && gnss_bundle.empty())
      throw std::runtime_error("global ODOM injection requires a global owner");
    if (async_global && (map_bundle.empty() && gnss_bundle.empty()))
      throw std::runtime_error("async global requires map or GNSS input");
    if (async_global && !arrival_paced)
      throw std::runtime_error("async global requires arrival-paced replay");
    if (has_arrival_rate && !arrival_paced)
      throw std::runtime_error("arrival rate requires arrival-paced replay");
    if (async_local_delay_ns && !arrival_paced)
      throw std::runtime_error("async local LIO requires arrival-paced replay");
    if (prefetch_lidar && !arrival_paced)
      throw std::runtime_error("LiDAR prefetch requires arrival-paced replay");
    if (timer_output && !arrival_paced)
      throw std::runtime_error("timer output requires arrival-paced replay");
    if (motion_translation_deskew && !motion_bootstrap)
      throw std::runtime_error("translation deskew requires motion bootstrap");
    if (lidar_stationary_initialization && !motion_bootstrap)
      throw std::runtime_error(
          "LiDAR rest initialization requires motion bootstrap");
    if (has_bootstrap_voxel_size && !motion_bootstrap)
      throw std::runtime_error(
          "bootstrap voxel size requires motion bootstrap");
    Run(argv[1], argv[2], motion_bootstrap, map_bundle, map_digests,
        gnss_bundle, gnss_digests, global_holdback_ns,
        motion_translation_deskew, lidar_stationary_initialization,
        motion_bootstrap_voxel_size, imu_trace, async_global, arrival_paced,
        async_local_delay_ns, prefetch_lidar, timer_output,
        level_odom_gnss_bootstrap, native_joint_sample_period,
        native_joint_limits, odometry_fault, map_holdout, arrival_rate,
        tracking_gain, profile_map_timing, profile_local_timing,
        fixed_gain_update, native_query_probe, native_structured,
        structured_selection);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "native_lio_replay: " << error.what() << '\n';
    return 1;
  }
}
