// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

#include "candidates/global/fixed_lag_graph.h"
#include "candidates/local/bounded_joint_error.h"
#include "test/integration/map/replay_csv.h"

namespace {
namespace u = openloc::localization::unified;
namespace csv = openloc::replay;

void Require(const u::Result& result) {
  if (!result.ok()) throw std::runtime_error(result.message);
}

Eigen::Isometry3d Pose(const std::vector<std::string>& row, std::size_t offset) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  for (int axis = 0; axis < 3; ++axis)
    result.translation()[axis] = csv::Number(row[offset + axis]);
  const Eigen::Quaterniond rotation(
      csv::Number(row[offset + 6]), csv::Number(row[offset + 3]),
      csv::Number(row[offset + 4]), csv::Number(row[offset + 5]));
  if (std::abs(rotation.norm() - 1) > 1e-9)
    throw std::runtime_error("non-unit input rotation");
  result.linear() = rotation.toRotationMatrix();
  return result;
}

Eigen::MatrixXd Matrix(const std::string& value) {
  const auto separator = value.find(':');
  const auto cross = value.find('x');
  if (separator == std::string::npos || cross >= separator)
    throw std::runtime_error("invalid matrix dimensions");
  const auto height = csv::Time(value.substr(0, cross));
  const auto width = csv::Time(value.substr(cross + 1, separator - cross - 1));
  if (height > 15 || width > 15)
    throw std::runtime_error("matrix exceeds declared-model dimensions");
  const int rows = static_cast<int>(height);
  const int columns = static_cast<int>(width);
  const auto fields = csv::Fields(value.substr(separator + 1), ';');
  if (fields.size() != static_cast<std::size_t>(rows * columns))
    throw std::runtime_error("invalid matrix field count");
  Eigen::MatrixXd result(rows, columns);
  for (int row = 0; row < rows; ++row)
    for (int column = 0; column < columns; ++column)
      result(row, column) = csv::Number(fields[row * columns + column]);
  return result;
}

void WriteMatrix(std::ostream& out, const Eigen::MatrixXd& value) {
  out << value.rows() << 'x' << value.cols() << ':';
  for (Eigen::Index row = 0; row < value.rows(); ++row)
    for (Eigen::Index column = 0; column < value.cols(); ++column) {
      if (row || column) out << ';';
      out << value(row, column);
    }
}

void WritePose(std::ostream& out, const Eigen::Isometry3d& pose) {
  const Eigen::Quaterniond q(pose.linear());
  out << pose.translation().transpose().format(
             Eigen::IOFormat(Eigen::FullPrecision, 0, ",", ","))
      << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
}

void Run(const std::filesystem::path& input,
         const std::filesystem::path& output, const std::string& arm,
         double lag_seconds) {
  if (std::filesystem::exists(input / "temporal_model.csv"))
    throw std::runtime_error(
        "temporal model requires causal receive-order mode");
  if (arm != "lio" && arm != "gnss" && arm != "map" && arm != "all")
    throw std::runtime_error("invalid source arm");
  const auto nodes = csv::ReadRows(
      (input / "nodes.csv").string(), "timestamp_ns,px,py,pz,qx,qy,qz,qw", 8);
  if (nodes.size() < 4 || nodes.size() > 256)
    throw std::runtime_error("engineering replay requires 4..256 nodes");
  const auto model = csv::ReadRows(
      (input / "model.csv").string(),
      "initial_variances,random_walk_variance_rates,innovation_gate,robust_threshold", 4);
  if (model.size() != 1)
    throw std::runtime_error("one explicit approximation model required");
  const auto initial = Matrix(model[0][0]);
  const auto rates = Matrix(model[0][1]);
  if (initial.rows() != 15 || initial.cols() != 1 ||
      rates.rows() != 15 || rates.cols() != 1 ||
      (initial.array() <= 0).any() || (rates.array() <= 0).any())
    throw std::runtime_error("model requires positive 15D diagonal variances/rates");
  u::FixedLagConfig config;
  config.lag = lag_seconds;
  // Offline only: failure remains an error; no realtime claim.
  config.solve_budget_ms = 100000;
  config.innovation_gate = csv::Number(model[0][2]);
  config.robust_threshold = csv::Number(model[0][3]);
  u::FixedLagGraph graph(config);
  Require(graph.ValidateConfig());
  Require(graph.EnableEvaluationHistory());
  std::map<int64_t, u::LocalState> local;
  int64_t last = 0;
  for (const auto& row : nodes) {
    const auto stamp = csv::Time(row[0]);
    if (stamp <= last || (!local.empty() &&
                         stamp * 1e-9 <= local.rbegin()->second.stamp.time))
      throw std::runtime_error("node timestamps regress or alias");
    const auto pose = Pose(row, 1);
    u::LocalState state;
    state.stamp = {stamp * 1e-9, stamp * 1e-9, local.size() + 1, "unix"};
    state.epoch = {"experimental-independent-keyframes", 1};
    state.position = pose.translation();
    state.orientation = Eigen::Quaterniond(pose.linear());
    state.valid = true;
    state.uncertainty_qualified = false;
    state.covariance = initial.col(0).asDiagonal();
    local.emplace(stamp, std::move(state));
    last = stamp;
  }
  std::multimap<int64_t, u::GlobalObservation> measurements;
  std::map<std::pair<std::string, uint64_t>, bool> identities;
  for (const auto& row : csv::ReadRows(
           (input / "measurements.csv").string(),
           "kind,timestamp_ns,receive_ns,sequence,px,py,pz,qx,qy,qz,qw,"
           "projection,covariance,lever_x,lever_y,lever_z,ambiguous", 17)) {
    const auto stamp = csv::Time(row[1]);
    const auto receive = csv::Time(row[2]);
    if (!local.count(stamp) || receive < stamp)
      throw std::runtime_error("measurement needs its exact node and causal reception");
    u::GlobalObservation observation;
    observation.source = row[0];
    observation.stamp = {stamp * 1e-9, receive * 1e-9,
                         static_cast<uint64_t>(csv::Time(row[3])), "unix"};
    if (!identities.emplace(
             std::make_pair(observation.source, observation.stamp.sequence), true).second)
      throw std::runtime_error("duplicate measurement identity");
    observation.measurement_timestamp_ns = stamp;
    observation.receive_timestamp_ns = receive;
    observation.epoch = local.at(stamp).epoch;
    observation.map_id = "frozen-engineering-input";
    observation.map_version = "v1";
    observation.calibration_id = "explicit-unqualified-input";
    observation.pose = Pose(row, 4);
    observation.quality_valid = true;
    // ONLY the declared approximation model assumes independence. This is not
    // a qualification of actual LIO/map/GNSS correlations or native covariance.
    observation.independent_of_local = true;
    observation.uncertainty_qualified = false;
    const auto covariance = Matrix(row[12]);
    const auto projection = Matrix(row[11]);
    if (row[0] == "gnss") {
      if (covariance.rows() != 2 || covariance.cols() != 2 ||
          projection.rows() != 2 || projection.cols() != 2 ||
          !projection.isIdentity(1e-12))
        throw std::runtime_error("invalid XY observation dimensions");
      observation.kind = u::GlobalObservation::Kind::HORIZONTAL_POSITION;
      observation.covariance.topLeftCorner<2, 2>() = covariance;
    } else if (row[0] == "map" || row[0] == "projected_map") {
      if (projection.cols() != 6 || covariance.rows() != projection.rows() ||
          covariance.cols() != projection.rows())
        throw std::runtime_error("invalid map observation dimensions");
      if (row[0] == "map") {
        if (projection.rows() != 6 || !projection.isIdentity(1e-12))
          throw std::runtime_error("full pose requires identity 6D projection");
        observation.covariance = covariance;
      } else {
        observation.kind = u::GlobalObservation::Kind::PROJECTED_POSE;
        observation.projection = projection;
        observation.projected_covariance = covariance;
      }
    } else {
      throw std::runtime_error("unknown measurement kind");
    }
    for (int axis = 0; axis < 3; ++axis)
      observation.point_in_base[axis] = csv::Number(row[13 + axis]);
    if (row[16] != "0" && row[16] != "1")
      throw std::runtime_error("invalid ambiguity flag");
    observation.ambiguous = row[16] == "1";
    if (observation.ambiguous)
      throw std::runtime_error("ambiguous observations require explicit rejection");
    measurements.emplace(stamp, std::move(observation));
  }
  const auto first = measurements.equal_range(local.begin()->first);
  const u::GlobalObservation* anchor = nullptr;
  for (auto it = first.first; it != first.second; ++it)
    if (it->second.kind == u::GlobalObservation::Kind::POSE &&
        it->second.source == "map") {
      if (anchor) throw std::runtime_error("exactly one common full-pose anchor required");
      anchor = &it->second;
    }
  if (!anchor) throw std::runtime_error("common full-pose anchor missing");
  if (!std::filesystem::create_directory(output))
    throw std::runtime_error("output must be new");
  std::ofstream declaration(output / "model_scope.csv");
  declaration << "model,experimental_only,native_model,source_correlation_unmodeled,"
                 "independent_accuracy_verified,trusted,common_map_anchor,"
                 "offline_measurement_time_order,synthetic_motion_source_ids\n"
                 "independent-keyframe-random-walk,1,0,1,0,0,1,1,1\n";
  std::ofstream states(output / "states.csv"), decisions(output / "decisions.csv");
  states << std::setprecision(17)
         << "timestamp_ns,px,py,pz,qx,qy,qz,qw\n";
  decisions << std::setprecision(17)
            << "timestamp_ns,receive_ns,source,sequence,accepted,reason,"
               "innovation_available,residual,covariance,nis\n";
  u::LocalState previous;
  for (auto& entry : local) {
    auto& state = entry.second;
    if (!previous.valid) {
      Require(graph.Initialize(state, *anchor));
    } else {
      const double dt = (entry.first - last) * 1e-9;
      u::Matrix15d transition = u::Matrix15d::Identity();
      transition.block<3, 3>(6, 6) =
          state.orientation.toRotationMatrix().transpose() *
          previous.orientation.toRotationMatrix();
      state.covariance =
          transition * previous.covariance * transition.transpose();
      state.covariance.diagonal() += dt * rates.col(0);
      u::MotionIncrement motion;
      // These are the declared model's endpoint tokens required by the graph
      // kernel, NOT identities or dependency claims for original IMU samples.
      Require(u::ComputeMotionIncrement(
          previous, state, previous.covariance * transition.transpose(),
          {{"imu", previous.stamp.sequence}, {"imu", state.stamp.sequence},
           {"declared-keyframe-model", state.stamp.sequence}}, &motion));
      Require(graph.AddLocal(state, motion));
    }
    const auto observations = measurements.equal_range(entry.first);
    for (auto it = observations.first; it != observations.second; ++it) {
      const auto& measurement = it->second;
      if (&measurement == anchor ||
          (measurement.source == "gnss" && arm != "gnss" && arm != "all") ||
          (measurement.source != "gnss" && arm != "map" && arm != "all"))
        continue;
      openloc::localization::ObservationInnovation innovation;
      const auto status = graph.Observe(measurement, &innovation);
      decisions << entry.first << ',' << *measurement.receive_timestamp_ns << ','
                << measurement.source << ',' << measurement.stamp.sequence << ','
                << status.ok() << ',';
      csv::Text(decisions, status.message);
      decisions << ',' << innovation.available << ',';
      WriteMatrix(decisions, innovation.residual);
      decisions << ',';
      WriteMatrix(decisions, innovation.covariance);
      decisions << ',' << innovation.nis << '\n';
      if (!status.ok() && status.reason != u::Reason::INNOVATION_REJECTED)
        Require(status);
    }
    states << entry.first << ',';
    WritePose(states, graph.state().map_to_odom * u::Pose(state));
    states << '\n';
    previous = state;
    last = entry.first;
  }
  u::GraphEvaluationSnapshot retained, batch;
  Require(graph.EvaluateForTesting(false, &retained));
  Require(graph.EvaluateForTesting(true, &batch));
  std::ofstream comparison(output / "comparison.csv");
  comparison << std::setprecision(17)
             << "nodes,retained_nodes,archived_nodes,batch_factors,"
                "translation_difference_m,rotation_difference_rad,"
                "batch_objective,batch_iterations,batch_exact_residual,"
                "retained_exact_residual\n"
             << local.size() << ',' << retained.retained_nodes << ','
             << batch.archived_nodes << ',' << batch.factors.size() << ','
             << (retained.map_from_base.translation() -
                 batch.map_from_base.translation()).norm() << ','
             << u::LogRotation(Eigen::Quaterniond(
                    retained.map_from_base.linear() *
                    batch.map_from_base.linear().transpose())).norm() << ','
             << batch.objective << ',' << batch.iterations << ','
             << batch.maximum_exact_residual << ','
             << retained.maximum_exact_residual << '\n';
  std::ofstream factors(output / "factors.csv");
  factors << std::setprecision(17)
          << "scope,type,keys,residual,error,weight\n";
  for (const auto* snapshot : {&retained, &batch})
    for (const auto& factor : snapshot->factors) {
      factors << (snapshot == &retained ? "retained" : "batch") << ','
              << factor.type << ',';
      for (std::size_t i = 0; i < factor.keys.size(); ++i)
        factors << (i ? ";" : "") << factor.keys[i];
      factors << ',';
      WriteMatrix(factors, factor.residual);
      factors << ',' << factor.error << ',' << factor.weight << '\n';
    }
  for (auto* stream : {&declaration, &states, &decisions, &comparison, &factors}) {
    stream->flush();
    if (!*stream) throw std::runtime_error("incomplete engineering replay output");
  }
}

class CausalRows {
 public:
  CausalRows(const std::filesystem::path& path, const std::string& header,
             std::size_t fields, std::size_t receive)
      : input_(path), fields_(fields), receive_(receive) {
    std::string line;
    if (!csv::ReadLine(input_, &line) || line != header)
      throw std::runtime_error("invalid causal input/header: " + path.string());
    Next();
  }
  bool available() const { return !row.empty(); }
  int64_t receive() const { return csv::Time(row.at(receive_)); }
  void Next() {
    std::string line;
    row.clear();
    if (!csv::ReadLine(input_, &line)) {
      if (!input_.eof()) throw std::runtime_error("incomplete causal stream");
      return;
    }
    row = csv::Fields(line);
    if (row.size() != fields_ || line.find('"') != std::string::npos ||
        line.back() == ',')
      throw std::runtime_error("invalid causal row");
    const auto now = receive();
    if (now < last_) throw std::runtime_error("receive times regress");
    last_ = now;
  }
  std::vector<std::string> row;

 private:
  std::ifstream input_;
  std::size_t fields_, receive_;
  int64_t last_ = 0;
};

u::GlobalObservation CausalObservation(const std::vector<std::string>& row,
                                       const u::Epoch& epoch) {
  const auto stamp = csv::Time(row[1]), receive = csv::Time(row[2]);
  if (receive < stamp) throw std::runtime_error("noncausal measurement");
  u::GlobalObservation result;
  result.source = row[0];
  result.stamp = {stamp * 1e-9, receive * 1e-9,
                  static_cast<uint64_t>(csv::Time(row[3])), "unix"};
  result.measurement_timestamp_ns = stamp;
  result.receive_timestamp_ns = receive;
  result.epoch = epoch;
  result.map_id = "frozen-engineering-input";
  result.map_version = "v1";
  result.calibration_id = "explicit-unqualified-input";
  result.pose = Pose(row, 4);
  result.quality_valid = true;
  result.independent_of_local = true;
  result.uncertainty_qualified = false;
  const auto projection = Matrix(row[11]), covariance = Matrix(row[12]);
  if (row[0] == "gnss") {
    if (projection.rows() != 2 || projection.cols() != 2 ||
        !projection.isIdentity(1e-12) ||
        covariance.rows() != 2 || covariance.cols() != 2)
      throw std::runtime_error("invalid causal XY dimensions");
    result.kind = u::GlobalObservation::Kind::HORIZONTAL_POSITION;
    result.covariance.topLeftCorner<2, 2>() = covariance;
  } else if (row[0] == "map" || row[0] == "projected_map") {
    if (projection.cols() != 6 || projection.rows() <= 0 ||
        covariance.rows() != projection.rows() ||
        covariance.cols() != projection.rows())
      throw std::runtime_error("invalid causal map dimensions");
    if (row[0] == "map") {
      if (projection.rows() != 6 || !projection.isIdentity(1e-12))
        throw std::runtime_error("full pose requires identity 6D projection");
      result.covariance = covariance;
    } else {
      result.kind = u::GlobalObservation::Kind::PROJECTED_POSE;
      result.projection = projection;
      result.projected_covariance = covariance;
    }
  } else {
    throw std::runtime_error("unknown causal measurement kind");
  }
  for (int i = 0; i < 3; ++i) result.point_in_base[i] = csv::Number(row[13 + i]);
  if (row[16] != "0") throw std::runtime_error("ambiguous observations unsupported");
  return result;
}

void RunCausal(const std::filesystem::path& input,
               const std::filesystem::path& output, const std::string& arm,
               double lag) {
  if (arm != "lio" && arm != "gnss" && arm != "map" && arm != "all")
    throw std::runtime_error("invalid source arm");
  const auto model = csv::ReadRows(
      (input / "model.csv").string(),
      "initial_variances,random_walk_variance_rates,innovation_gate,robust_threshold", 4);
  if (model.size() != 1) throw std::runtime_error("one explicit model required");
  const auto initial = Matrix(model[0][0]), rates = Matrix(model[0][1]);
  if (initial.rows() != 15 || initial.cols() != 1 ||
      rates.rows() != 15 || rates.cols() != 1 ||
      (initial.array() <= 0).any() || (rates.array() <= 0).any())
    throw std::runtime_error("invalid causal random-walk model");
  double decay_rate = 0.0;
  if (std::filesystem::exists(input / "temporal_model.csv")) {
    const auto temporal = csv::ReadRows((input / "temporal_model.csv").string(),
                                        "decay_rate_per_s", 1);
    if (temporal.size() != 1)
      throw std::runtime_error("one explicit temporal model required");
    decay_rate = csv::Number(temporal[0][0]);
    if (decay_rate <= 0)
      throw std::runtime_error("temporal decay rate must be positive");
  }
  CausalRows nodes(input / "nodes.csv",
      "timestamp_ns,px,py,pz,qx,qy,qz,qw,receive_ns", 9, 8);
  CausalRows observations(input / "measurements.csv",
      "kind,timestamp_ns,receive_ns,sequence,px,py,pz,qx,qy,qz,qw,"
      "projection,covariance,lever_x,lever_y,lever_z,ambiguous", 17, 2);
  u::FixedLagConfig config;
  config.lag = lag;
  config.solve_budget_ms = 100000;  // Diagnostic replay, not a deadline claim.
  config.innovation_gate = csv::Number(model[0][2]);
  config.robust_threshold = csv::Number(model[0][3]);
  u::FixedLagGraph graph(config);
  Require(graph.ValidateConfig());
  u::BoundedJointError frontier(45);
  const u::Epoch epoch{"experimental-independent-keyframes", 1};
  std::map<int64_t, u::LocalState> retained;
  std::map<int64_t, u::MotionIncrement> startup_motions;
  u::LocalState previous;
  uint64_t count = 0;
  int64_t last_stamp = 0;
  if (!std::filesystem::create_directory(output))
    throw std::runtime_error("causal output must be new");
  std::ofstream scope(output / "model_scope.csv");
  scope << "model,experimental_only,native_model,source_correlation_unmodeled,"
           "independent_accuracy_verified,trusted,receive_time_order,"
           "evaluation_archive\n"
        << (decay_rate > 0 ? "closed-frontier-keyframe-gauss-markov"
                           : "closed-frontier-keyframe-random-walk")
        << ",1,0,1,0,0,1,0\n";
  std::ofstream events(output / "events.csv"), decisions(output / "decisions.csv");
  std::ofstream motion_models(output / "motion_model.csv");
  motion_models
      << std::setprecision(17)
      << "timestamp_ns,dt_s,decay_rate_per_s,transition,process_covariance,"
         "state_covariance,previous_state_cross_covariance\n";
  events << std::setprecision(17)
         << "receive_ns,kind,latest_measurement_ns,initialized,px,py,pz,qx,qy,qz,qw,"
            "frontier_rows,frontier_dimension,buffered_nodes,graph_nodes,"
            "graph_observations,factor_slots,active_factors,archived_values,"
            "solve_iterations,damped_iterations,maximum_motion_residual,elapsed_ms\n";
  decisions << std::setprecision(17)
            << "timestamp_ns,receive_ns,source,sequence,accepted,reason,"
               "innovation_available,residual,covariance,nis\n";
  while (nodes.available() || observations.available()) {
    const auto started = std::chrono::steady_clock::now();
    const bool node_event = nodes.available() &&
        (!observations.available() || nodes.receive() <= observations.receive());
    const auto receive = node_event ? nodes.receive() : observations.receive();
    if (node_event) {
      const auto stamp = csv::Time(nodes.row[0]);
      if (stamp <= last_stamp || receive < stamp ||
          (previous.valid && stamp * 1e-9 <= previous.stamp.time))
        throw std::runtime_error("causal nodes regress, alias or precede measurement");
      u::LocalState state;
      const auto pose = Pose(nodes.row, 1);
      state.stamp = {stamp * 1e-9, receive * 1e-9, ++count, "unix"};
      state.epoch = epoch;
      state.position = pose.translation();
      state.orientation = Eigen::Quaterniond(pose.linear());
      state.valid = true;
      u::MotionIncrement motion;
      if (!previous.valid) {
        Require(frontier.Independent("state", initial.col(0).cwiseSqrt().asDiagonal()));
      } else {
        u::Matrix15d transition = u::Matrix15d::Identity();
        transition.block<3, 3>(6, 6) =
            state.orientation.toRotationMatrix().transpose() *
            previous.orientation.toRotationMatrix();
        const double dt = (stamp - last_stamp) * 1e-9;
        // Diffusion rates are declared in the interval's destination chart.
        // Scalar decay commutes with the attitude chart transport.
        const double decay = std::exp(-decay_rate * dt);
        transition *= decay;
        const double noise_time =
            decay_rate > 0
                ? -std::expm1(-2 * decay_rate * dt) / (2 * decay_rate)
                : dt;
        const Eigen::VectorXd process_variances = noise_time * rates.col(0);
        if (!process_variances.allFinite() ||
            (process_variances.array() <= 0).any())
          throw std::runtime_error(
              "temporal process variance outside numeric support");
        Require(frontier.Linear("previous", {{u::Matrix15d::Identity(), "state"}}));
        Require(frontier.Independent(
            "noise", process_variances.cwiseSqrt().asDiagonal()));
        Require(frontier.Linear("state", {{transition, "previous"},
                                          {u::Matrix15d::Identity(), "noise"}}));
        Eigen::MatrixXd covariance, cross;
        Require(frontier.CrossCovariance("state", "state", &covariance));
        state.covariance = covariance;
        Require(frontier.CrossCovariance("previous", "state", &cross));
        motion_models << stamp << ',' << dt << ',' << decay_rate << ',';
        WriteMatrix(motion_models, transition);
        motion_models << ',';
        WriteMatrix(motion_models, process_variances.asDiagonal());
        motion_models << ',';
        WriteMatrix(motion_models, covariance);
        motion_models << ',';
        WriteMatrix(motion_models, cross);
        motion_models << '\n';
        Require(u::ComputeMotionIncrement(
            previous, state, cross,
            {{"imu", previous.stamp.sequence}, {"imu", state.stamp.sequence},
             {"declared-keyframe-model", state.stamp.sequence}}, &motion));
      }
      if (!previous.valid) {
        Eigen::MatrixXd covariance;
        Require(frontier.CrossCovariance("state", "state", &covariance));
        state.covariance = covariance;
      }
      Require(frontier.Retain({"state"}));
      Require(frontier.Compact());
      if (graph.initialized()) {
        Require(graph.AddLocal(state, motion));
      } else if (previous.valid) {
        startup_motions.emplace(stamp, motion);
      }
      retained.emplace(stamp, state);
      while (!retained.empty() &&
             retained.begin()->second.stamp.time < state.stamp.time - lag) {
        startup_motions.erase(retained.begin()->first);
        retained.erase(retained.begin());
      }
      if (retained.size() > config.max_states)
        throw std::runtime_error("causal node buffer capacity exhausted");
      previous = state;
      last_stamp = stamp;
      nodes.Next();
    } else {
      const auto measurement = CausalObservation(observations.row, epoch);
      const bool full_map = measurement.source == "map";
      const bool selected = measurement.source == "gnss"
          ? arm == "gnss" || arm == "all" : arm == "map" || arm == "all";
      if (selected || (!graph.initialized() && full_map)) {
        openloc::localization::ObservationInnovation innovation;
        u::Result status;
        const auto stamp = *measurement.measurement_timestamp_ns;
        if (!graph.initialized() && full_map && retained.count(stamp)) {
          std::vector<u::LocalState> following;
          std::vector<u::MotionIncrement> motions;
          for (auto it = retained.upper_bound(stamp); it != retained.end(); ++it) {
            following.push_back(it->second);
            motions.push_back(startup_motions.at(it->first));
          }
          status = graph.Initialize(retained.at(stamp), measurement, following, motions);
          if (status.ok()) startup_motions.clear();
        } else {
          status = graph.Observe(measurement, &innovation);
        }
        decisions << stamp << ',' << receive << ',' << measurement.source << ','
                  << measurement.stamp.sequence << ',' << status.ok() << ',';
        csv::Text(decisions, u::ReasonName(status.reason));
        decisions << ',' << innovation.available << ',';
        WriteMatrix(decisions, innovation.residual);
        decisions << ',';
        WriteMatrix(decisions, innovation.covariance);
        decisions << ',' << innovation.nis << '\n';
        if (!status.ok() && status.reason != u::Reason::INNOVATION_REJECTED &&
            status.reason != u::Reason::HISTORY_UNAVAILABLE &&
            status.reason != u::Reason::GLOBAL_UNAVAILABLE &&
            status.reason != u::Reason::DUPLICATE)
          Require(status);
      }
      observations.Next();
    }
    const auto usage = graph.ResourceUsage();
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    if (graph.initialized())
      pose = graph.state().map_to_odom * u::Pose(previous);
    events << receive << ',' << (node_event ? "node" : "observation") << ','
           << last_stamp << ',' << graph.initialized() << ',';
    WritePose(events, pose);
    events << ',' << frontier.rows() << ',' << frontier.dimension() << ','
           << retained.size() << ',' << usage.retained_nodes << ','
           << usage.retained_observations << ',' << usage.factor_slots << ','
           << usage.active_factors << ',' << usage.archived_values << ','
           << usage.solve_iterations << ',' << usage.damped_iterations << ','
           << usage.maximum_motion_residual << ','
           << std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - started).count() << '\n';
    events.flush();
  }
  for (auto* stream : {&scope, &events, &decisions, &motion_models}) {
    stream->flush();
    if (!*stream) throw std::runtime_error("incomplete causal output");
  }
  if (!graph.initialized()) throw std::runtime_error("no causal full-pose anchor");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if ((argc != 6 && argc != 7) ||
        std::string(argv[5]) != "--experimental-independent-keyframes")
      throw std::runtime_error(
          "usage: keyframe_graph_replay INPUT NEW_OUTPUT lio|gnss|map|all LAG_S "
          "--experimental-independent-keyframes [--causal-receive-order]; "
          "NOT a native model qualification");
    if (argc == 7) {
      if (std::string(argv[6]) != "--causal-receive-order")
        throw std::runtime_error("unknown causal mode");
      RunCausal(argv[1], argv[2], argv[3], csv::Number(argv[4]));
    } else {
      Run(argv[1], argv[2], argv[3], csv::Number(argv[4]));
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "keyframe_graph_replay: " << error.what() << '\n';
    return 1;
  }
}
