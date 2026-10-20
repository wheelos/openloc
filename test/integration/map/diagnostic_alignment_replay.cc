// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <optional>
#include <variant>

#include "openloc/localization/global/diagnostic_alignment.h"
#include "test/integration/map/replay_csv.h"

namespace {
namespace loc = openloc::localization;
namespace csv = openloc::replay;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

constexpr char kProvenanceHeader[] =
    "acquisition_id,preparation_id,odometry_initialization,odometry_deskew,"
    "shared_local_scans,gnss_map_construction";

const char* DependencyName(loc::ObservationDependency dependency) {
  switch (dependency) {
    case loc::ObservationDependency::kUnknown:
      return "unknown";
    case loc::ObservationDependency::kAbsent:
      return "absent";
    case loc::ObservationDependency::kPresent:
      return "present";
  }
  throw std::runtime_error("invalid dependency");
}

loc::ObservationDependency Dependency(const std::string& text) {
  if (text == "unknown") return loc::ObservationDependency::kUnknown;
  if (text == "absent") return loc::ObservationDependency::kAbsent;
  if (text == "present") return loc::ObservationDependency::kPresent;
  throw std::runtime_error("dependency must be unknown, absent or present");
}

void WriteProvenance(std::ostream& output,
                     const loc::GlobalObservationProvenance& provenance) {
  output << ',';
  csv::Text(output, provenance.acquisition_id);
  output << ',';
  csv::Text(output, provenance.preparation_id);
  for (const auto dependency :
       {provenance.odometry_initialization, provenance.odometry_deskew,
        provenance.shared_local_scans, provenance.gnss_map_construction}) {
    output << ',' << DependencyName(dependency);
  }
}

struct OdomInput {
  loc::OdometrySample sample;
  Matrix6d covariance;
};

struct Event {
  int64_t arrival_ns;
  loc::GlobalObservationContext context;
  std::variant<OdomInput, loc::GlobalMapObservation,
               loc::GlobalHorizontalObservation>
      value;
};

loc::Timestamp Timestamp(int64_t ns) {
  return {ns, loc::ClockDomain::kUnixEpoch};
}

loc::Pose Pose(const std::vector<std::string>& row, std::size_t offset) {
  loc::Pose pose;
  for (int i = 0; i < 3; ++i)
    pose.translation[i] = csv::Number(row[offset + i]);
  pose.rotation = Eigen::Quaterniond(
      csv::Number(row[offset + 6]), csv::Number(row[offset + 3]),
      csv::Number(row[offset + 4]), csv::Number(row[offset + 5]));
  return pose;
}

bool Flag(const std::string& value) {
  if (value != "0" && value != "1")
    throw std::runtime_error("flag must be 0 or 1");
  return value == "1";
}

void Require(const loc::Status& status) {
  if (!status.ok()) throw std::runtime_error(status.message());
}

loc::DiagnosticAlignmentConfig Config(const std::string& path) {
  const auto rows = csv::ReadRows(
      path,
      "map_id,map_version,map_content_id,calibration_id,map_frame,"
      "odom_frame,base_frame",
      7);
  if (rows.size() != 1)
    throw std::runtime_error("exactly one identity contract required");
  const auto& row = rows.front();
  loc::DiagnosticAlignmentConfig config;
  config.reference = {row[0], row[1], row[3]};
  config.reference.geodetic.state =
      loc::GeodeticReferenceState::kDiagnosticOnly;
  config.map_content_id = row[2];
  config.map_frame = {row[4]};
  config.odom_frame = {row[5]};
  config.base_frame = {row[6]};
  return config;
}

loc::GlobalObservationContext Context(
    const std::vector<std::string>& row,
    const loc::DiagnosticAlignmentConfig& config, uint64_t sequence,
    const std::string& source) {
  loc::GlobalObservationContext context;
  context.receive_time = Timestamp(csv::Time(row[0]));
  context.measurement_time = Timestamp(csv::Time(row[1]));
  context.epoch = {row[2], static_cast<uint64_t>(csv::Time(row[3]))};
  context.sequence = sequence;
  context.source = source;
  context.reference = config.reference;
  context.map_content_id = config.map_content_id;
  context.map_frame = config.map_frame;
  context.base_frame = config.base_frame;
  return context;
}

std::vector<Event> ReadEvents(char** argv,
                              const loc::DiagnosticAlignmentConfig& config) {
  std::vector<Event> events;
  const auto odom =
      csv::ReadRows(argv[1],
                    "arrival_ns,timestamp_ns,session,generation,px,py,pz,"
                    "qx,qy,qz,qw,covariance",
                    12);
  if (odom.empty()) throw std::runtime_error("ODOM input is empty");
  uint64_t sequence = 0;
  for (const auto& row : odom) {
    auto context = Context(row, config, ++sequence, "odom");
    if (context.receive_time.nanoseconds < context.measurement_time.nanoseconds)
      throw std::runtime_error("ODOM arrives before its measurement");
    OdomInput input;
    input.sample.timestamp = context.measurement_time;
    input.sample.epoch = context.epoch;
    input.sample.parent_frame = config.odom_frame;
    input.sample.child_frame = config.base_frame;
    input.sample.pose = Pose(row, 4);
    input.covariance = csv::SquareMatrix(row[11], 6);
    events.push_back({context.receive_time.nanoseconds, context, input});
  }
  const auto add_stream = [&](const std::string& path, bool map) {
    if (path == "-") return;
    const auto rows = csv::ReadRows(
        path,
        map ? "arrival_ns,timestamp_ns,session,generation,sequence,source,kind,"
              "px,py,pz,qx,qy,qz,qw,covariance,projection,projected_covariance,"
              "covariance_source,ambiguous,recovery"
            : "arrival_ns,timestamp_ns,session,generation,sequence,source,"
              "x,y,covariance,lx,ly,lz,covariance_source",
        map ? 20 : 13, kProvenanceHeader);
    int64_t previous = 0;
    for (const auto& row : rows) {
      auto context = Context(row, config, csv::Time(row[4]), row[5]);
      const std::size_t offset = map ? 20 : 13;
      if (row.size() > offset) {
        context.provenance.acquisition_id = row[offset];
        context.provenance.preparation_id = row[offset + 1];
        context.provenance.odometry_initialization =
            Dependency(row[offset + 2]);
        context.provenance.odometry_deskew = Dependency(row[offset + 3]);
        context.provenance.shared_local_scans = Dependency(row[offset + 4]);
        context.provenance.gnss_map_construction = Dependency(row[offset + 5]);
      }
      if (context.receive_time.nanoseconds < previous)
        throw std::runtime_error("source arrival order regressed");
      previous = context.receive_time.nanoseconds;
      if (map) {
        loc::GlobalMapObservation observation;
        observation.context = context;
        observation.map_from_base = Pose(row, 7);
        if (row[6] == "full") {
          observation.covariance = csv::SquareMatrix(row[14], 6);
          if (row[15] != "-" || row[16] != "-")
            throw std::runtime_error("full pose has unexpected projection");
        } else if (row[6] == "projected") {
          if (row[14] != "-")
            throw std::runtime_error("projected pose has full covariance");
          observation.kind = loc::GlobalMapObservation::Kind::kProjectedPose;
          observation.projection = csv::ReadMatrix(row[15], 6);
          observation.projected_covariance =
              csv::SquareMatrix(row[16], observation.projection.rows());
        } else {
          throw std::runtime_error("unknown map observation kind");
        }
        observation.covariance_source = row[17];
        observation.ambiguous = Flag(row[18]);
        observation.recovery = Flag(row[19]);
        events.push_back(
            {context.receive_time.nanoseconds, context, observation});
      } else {
        loc::GlobalHorizontalObservation observation;
        observation.context = context;
        observation.position = {csv::Number(row[6]), csv::Number(row[7])};
        observation.covariance = csv::SquareMatrix(row[8], 2);
        observation.point_in_base = {csv::Number(row[9]), csv::Number(row[10]),
                                     csv::Number(row[11])};
        observation.covariance_source = row[12];
        events.push_back(
            {context.receive_time.nanoseconds, context, observation});
      }
    }
  };
  int64_t previous_arrival = 0;
  for (const auto& event : events) {
    if (event.arrival_ns < previous_arrival)
      throw std::runtime_error("ODOM arrival order regressed");
    previous_arrival = event.arrival_ns;
  }
  add_stream(argv[2], true);
  add_stream(argv[3], false);
  // Equal arrivals: ODOM first, then map, then XY; file order is retained.
  // Measurement time never determines delivery order.
  std::stable_sort(events.begin(), events.end(),
                   [](const Event& a, const Event& b) {
                     return a.arrival_ns < b.arrival_ns;
                   });
  return events;
}

void WritePose(std::ostream& output, const loc::Pose& pose) {
  for (double value : {pose.translation.x(), pose.translation.y(),
                       pose.translation.z(), pose.rotation.x(),
                       pose.rotation.y(), pose.rotation.z(), pose.rotation.w()})
    output << ',' << value;
}

int Replay(char** argv) {
  const std::string arm = argv[6];
  if ((arm != "odom" && arm != "gnss" && arm != "map" && arm != "all") ||
      std::string(argv[7]) != "--diagnostic-only")
    throw std::runtime_error("explicit diagnostic flag and valid arm required");
  const auto config = Config(argv[4]);
  const auto events = ReadEvents(argv, config);
  loc::DiagnosticGlobalAlignment owner;
  Require(owner.Configure(config));
  const std::filesystem::path directory(argv[5]);
  if (!std::filesystem::create_directory(directory))
    throw std::runtime_error("output directory already exists");
  std::ofstream output(directory / "events.csv");
  std::ofstream decisions(directory / "decisions.csv");
  std::ofstream summary(directory / "summary.csv");
  if (!output || !decisions || !summary)
    throw std::runtime_error("cannot open output files");
  output << std::setprecision(17);
  output
      << "kind,arrival_ns,timestamp_ns,source,sequence,event_status,message,"
         "result_timestamp_ns,session,generation,odom_px,odom_py,odom_pz,"
         "odom_qx,odom_qy,odom_qz,odom_qw,alignment_initialized,correction_id,"
         "x,y,z,qx,qy,qz,qw,alignment_covariance,verification_pending,"
         "odom_fresh,full_pose_evidence_fresh,pending_observations,"
         "diagnostic_only,global_trusted,covariance_source,"
      << kProvenanceHeader << '\n';
  decisions << "decision_id,arrival_ns,timestamp_ns,source,sequence,session,"
               "generation,status,message,correction_id,alignment_updated,"
               "covariance_source,"
            << kProvenanceHeader << ",decision_time_ns,queue_wait_ns\n";
  std::ofstream metrics(directory / "metrics.csv");
  if (!metrics) throw std::runtime_error("cannot open metrics output");
  metrics << std::setprecision(17)
          << "event_index,kind,arrival_ns,arrival_delay_ns,processing_time_ns,"
             "correction_translation_m,correction_rotation_rad\n";
  std::optional<loc::OdometrySample> latest;
  loc::DiagnosticAlignmentResult result;
  uint64_t last_decision = 0, updates = 0, rejected = 0, retained = 0;
  std::size_t selected = 0, odom_count = 0;
  std::size_t peak_pending = 0, event_index = 0;
  std::size_t pending_count = 0;
  int64_t maximum_queue_wait_ns = 0;
  double maximum_translation = 0.0, maximum_rotation = 0.0;
  std::vector<int64_t> processing_times;
  processing_times.reserve(events.size());
  for (const auto& event : events) {
    const auto started = std::chrono::steady_clock::now();
    const auto previous_estimate = result.estimate;
    const auto previous_epoch = result.epoch;
    const auto* odom = std::get_if<OdomInput>(&event.value);
    const auto* map = std::get_if<loc::GlobalMapObservation>(&event.value);
    const char* kind = odom ? "odom" : map ? "map" : "gnss";
    const bool requested = odom || arm == "all" || arm == kind;
    loc::Status status;
    if (odom) {
      status = owner.AddOdometry(odom->sample, odom->covariance);
      Require(status);
      latest = odom->sample;
      ++odom_count;
    } else if (requested) {
      ++selected;
      status =
          map ? owner.SubmitMapObservation(*map)
              : owner.SubmitHorizontalObservation(
                    std::get<loc::GlobalHorizontalObservation>(event.value));
      if (status.code() == loc::StatusCode::kPending) {
        peak_pending = std::max(peak_pending, ++pending_count);
      }
    }
    output << kind << ',' << event.arrival_ns << ','
           << event.context.measurement_time.nanoseconds << ',';
    csv::Text(output, event.context.source);
    output << ',' << event.context.sequence << ',';
    if (requested)
      output << static_cast<int>(status.code());
    else
      output << "not_requested";
    output << ',';
    csv::Text(output, requested ? status.message() : "arm excludes source");
    if (!latest) {
      output << std::string(24, ',') << ",1,0";
    } else {
      Require(owner.GetResult(Timestamp(event.arrival_ns), &result));
      pending_count = result.pending_observations;
      output << ',' << result.timestamp.nanoseconds << ',';
      csv::Text(output, result.epoch.producer_session);
      output << ',' << result.epoch.generation;
      WritePose(output, latest->pose);
      output << ',' << result.estimate.has_value();
      if (result.estimate) {
        output << ',' << result.estimate->correction_id;
        WritePose(output, result.estimate->map_from_base);
        output << ',';
        csv::Matrix(output, result.estimate->alignment_covariance);
      } else {
        output << std::string(9, ',');
      }
      output << ',' << result.verification_pending << ','
             << result.odometry_fresh << ',' << result.full_pose_evidence_fresh
             << ',' << result.pending_observations << ",1,0";
    }
    output << ',';
    csv::Text(output,
              odom  ? "explicit-diagnostic-odom-input"
              : map ? map->covariance_source
                    : std::get<loc::GlobalHorizontalObservation>(event.value)
                          .covariance_source);
    WriteProvenance(output, event.context.provenance);
    output << '\n';
    const auto processing_time_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - started)
            .count();
    processing_times.push_back(processing_time_ns);
    metrics << ++event_index << ',' << kind << ',' << event.arrival_ns << ','
            << event.arrival_ns - event.context.measurement_time.nanoseconds
            << ',' << processing_time_ns << ',';
    if (latest && previous_estimate && result.estimate &&
        previous_epoch == result.epoch) {
      const auto& before = previous_estimate->map_from_odom;
      const auto& after = result.estimate->map_from_odom;
      // Compare both alignments at the same current body, excluding motion.
      const double translation =
          ((after.translation + after.rotation * latest->pose.translation) -
           (before.translation + before.rotation * latest->pose.translation))
              .norm();
      const double rotation = before.rotation.angularDistance(after.rotation);
      metrics << translation << ',' << rotation;
      maximum_translation = std::max(maximum_translation, translation);
      maximum_rotation = std::max(maximum_rotation, rotation);
    } else {
      metrics << ',';
    }
    metrics << '\n';
    std::vector<loc::GlobalObservationDecision> journal;
    Require(owner.GetDecisions(&journal));
    for (const auto& decision : journal) {
      if (decision.decision_id <= last_decision) continue;
      if (decision.decision_id != last_decision + 1)
        throw std::runtime_error("decision journal overrun");
      last_decision = decision.decision_id;
      decisions << decision.decision_id << ','
                << decision.context.receive_time.nanoseconds << ','
                << decision.context.measurement_time.nanoseconds << ',';
      csv::Text(decisions, decision.context.source);
      decisions << ',' << decision.context.sequence << ',';
      csv::Text(decisions, decision.context.epoch.producer_session);
      decisions << ',' << decision.context.epoch.generation << ','
                << static_cast<int>(decision.status.code()) << ',';
      csv::Text(decisions, decision.status.message());
      decisions << ',' << decision.correction_id << ','
                << decision.alignment_updated << ',';
      csv::Text(decisions, decision.covariance_source);
      WriteProvenance(decisions, decision.context.provenance);
      const int64_t wait_ns =
          event.arrival_ns - decision.context.receive_time.nanoseconds;
      if (wait_ns < 0)
        throw std::runtime_error("decision precedes observation arrival");
      maximum_queue_wait_ns = std::max(maximum_queue_wait_ns, wait_ns);
      decisions << ',' << event.arrival_ns << ',' << wait_ns << '\n';
      updates += decision.alignment_updated;
      rejected += !decision.status.ok();
      retained += decision.status.ok() && !decision.alignment_updated;
    }
  }
  std::sort(processing_times.begin(), processing_times.end());
  if (selected != updates + rejected + retained + result.pending_observations)
    throw std::runtime_error("observation decision accounting mismatch");
  struct rusage resources {};
  if (getrusage(RUSAGE_SELF, &resources) != 0)
    throw std::runtime_error("cannot query replay resource usage");
  summary << std::setprecision(17);
  summary << "arm,input_events,odometry_events,selected_observations,updates,"
             "rejected,retained,pending,diagnostic_only,global_trusted,"
             "peak_pending,maximum_queue_wait_ns,maximum_translation_m,"
             "maximum_rotation_rad,processing_p50_ns,processing_p95_ns,"
             "processing_max_ns,process_peak_rss_kib\n"
          << arm << ',' << events.size() << ',' << odom_count << ',' << selected
          << ',' << updates << ',' << rejected << ',' << retained << ','
          << result.pending_observations << ",1,0," << peak_pending << ','
          << maximum_queue_wait_ns << ',' << maximum_translation << ','
          << maximum_rotation << ','
          << processing_times[(processing_times.size() - 1) / 2] << ','
          << processing_times[(95 * processing_times.size() + 99) / 100 - 1]
          << ',' << processing_times.back() << ',' << resources.ru_maxrss
          << '\n';
  if (!output.flush() || !decisions.flush() || !metrics.flush() ||
      !summary.flush())
    throw std::runtime_error("incomplete replay output");
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 8) {
    std::cerr
        << "usage: odom.csv map.csv|- xy.csv|- contract.csv new-output-dir "
           "odom|gnss|map|all --diagnostic-only\n";
    return 2;
  }
  try {
    return Replay(argv);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
