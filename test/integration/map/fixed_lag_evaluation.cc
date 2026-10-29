// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"
#include "candidates/global/fixed_lag_graph.h"
#include "test/integration/map/replay_csv.h"

namespace {
namespace u = openloc::localization::unified;
namespace csv = openloc::replay;
using Clock = std::chrono::steady_clock;

void Require(const u::Result& result) {
  if (!result.ok()) throw std::runtime_error(result.message);
}

void Matrix(std::ostream& out, const Eigen::MatrixXd& value) {
  out << value.rows() << 'x' << value.cols() << ':';
  for (Eigen::Index row = 0; row < value.rows(); ++row)
    for (Eigen::Index col = 0; col < value.cols(); ++col) {
      if (row || col) out << ';';
      out << value(row, col);
    }
}

u::GlobalObservation Observation(const u::LocalState& local,
                                 const Eigen::Isometry3d& truth,
                                 const Eigen::VectorXd& error,
                                 const std::string& model, bool horizontal,
                                 std::mt19937_64* random) {
  std::normal_distribution<double> normal;
  u::GlobalObservation result;
  result.stamp = local.stamp;
  result.epoch = local.epoch;
  result.source = horizontal ? "synthetic-gnss" : "synthetic-map";
  result.kind = horizontal ? u::GlobalObservation::Kind::HORIZONTAL_POSITION
                           : u::GlobalObservation::Kind::POSE;
  result.map_id = "semi-synthetic";
  result.map_version = "v1";
  result.calibration_id = "synthetic-known";
  result.pose = truth;
  result.covariance = u::Matrix6d::Identity() * 0.0004;
  result.covariance.bottomRightCorner<3, 3>() *= 0.01;
  const bool correlated = model == "ar_modeled" || model == "ar_unmodeled" ||
                          model == "shared_modeled" ||
                          model == "shared_unmodeled";
  const bool modeled = model == "ar_modeled" || model == "shared_modeled";
  if (correlated) {
    result.pose.translation() += error.tail<3>();
    result.covariance.topLeftCorner<3, 3>().diagonal().array() += 0.0016;
  }
  for (int axis = 0; axis < 3; ++axis)
    result.pose.translation()[axis] += 0.02 * normal(*random);
  if (!horizontal) {
    Eigen::Vector3d rotation;
    for (int axis = 0; axis < 3; ++axis) rotation[axis] = 0.002 * normal(*random);
    result.pose.linear() =
        u::ExpRotation(rotation).toRotationMatrix() * truth.linear();
  }
  result.quality_valid = true;
  result.uncertainty_qualified = false;
  result.independent_of_local = !modeled;
  if (modeled) {
    result.conditional_local_error_qualified = true;
    result.local_error_sequence = local.stamp.sequence;
    result.local_error_cross_covariance =
        Eigen::MatrixXd::Zero(18, horizontal ? 2 : 6);
    result.local_error_cross_covariance.block(15, 0, horizontal ? 2 : 3,
                                             horizontal ? 2 : 3)
        .setIdentity();
    result.local_error_cross_covariance *= 0.0016;
  }
  if (model == "independent_overconfident") result.covariance *= .25;
  if (model == "independent_conservative") result.covariance *= 4;
  return result;
}

void Run(const std::filesystem::path& input,
         const std::filesystem::path& output, uint64_t trials, uint64_t seed,
         double lag_seconds, double map_loss_begin = 0,
         double map_loss_end = 0) {
  const auto nominal = csv::ReadRows(
      input.string(), "timestamp_ns,px,py,pz,qx,qy,qz,qw", 8);
  if (nominal.size() < 4 || nominal.size() > 64 || trials == 0 || trials > 10000)
    throw std::runtime_error("evaluation requires 4..64 nodes and 1..10000 trials");
  if (!std::isfinite(map_loss_begin) || !std::isfinite(map_loss_end) ||
      map_loss_begin < 0 || map_loss_end < 0 ||
      ((map_loss_begin != 0 || map_loss_end != 0) &&
       (map_loss_begin <= 0 || map_loss_end <= map_loss_begin)))
    throw std::runtime_error(
        "map loss must follow the anchor and have positive duration");
  if (!std::filesystem::create_directory(output))
    throw std::runtime_error("output must be new");
  std::ofstream results(output / "trials.csv");
  std::ofstream decisions(output / "decisions.csv");
  std::ofstream factors(output / "factors.csv");
  std::ofstream failures(output / "failures.csv");
  std::ofstream generated(output / "generated.csv");
  generated << std::setprecision(17)
            << "model,trial,node,source,measurement_error,declared_covariance,"
               "submitted\n";
  results << std::setprecision(17)
          << "model,trial,seed,cutoff_ns,nodes,retained_nodes,archived_factors,"
             "accepted,rejected,nees,pose_error,covariance,"
             "batch_translation_difference_m,batch_rotation_difference_rad,"
             "batch_objective,batch_iterations,batch_exact_residual,"
             "lag_exact_residual,posterior_information_eigenvalues,"
             "posterior_information_condition,update_p50_ms,update_p99_ms,update_max_ms,"
             "retained_relinearizations\n";
  decisions << std::setprecision(17)
            << "model,trial,node,source,timestamp_ns,receive_ns,status,reason,"
               "innovation_available,residual,covariance,nis,gate\n";
  factors << std::setprecision(17)
          << "model,trial,cutoff_ns,scope,factor_index,type,keys,residual,error,"
             "weight,weight_time\n";
  failures << "model,trial,seed,node,stage,reason\n";
  for (const std::string model :
       {"independent", "ar_modeled", "ar_unmodeled", "shared_modeled",
        "shared_unmodeled", "independent_overconfident",
        "independent_conservative"}) {
    for (uint64_t trial = 0; trial < trials; ++trial) {
      std::size_t active_node = 0;
      std::string stage = "initialization";
      try {
      // Same driving random sequence in each model; separate independent trials.
      std::mt19937_64 random(seed + trial);
      std::normal_distribution<double> normal;
      u::FixedLagConfig config;
      config.lag = lag_seconds;
      config.solve_budget_ms = 100000;
      // A fixed admission/near-Gaussian study, not production anomaly admission.
      config.innovation_gate = 1e6;
      config.robust_threshold = 1e6;
      u::FixedLagGraph graph(config);
      Require(graph.EnableEvaluationHistory());
      Eigen::VectorXd error = Eigen::VectorXd::Zero(18);
      for (int axis = 0; axis < 3; ++axis) {
        error[axis] = 0.02 * normal(random);
        error[6 + axis] = 0.002 * normal(random);
        error[15 + axis] = 0.04 * normal(random);
      }
      u::LocalState previous;
      Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
      std::vector<double> timings;
      uint64_t accepted = 0, rejected = 0;
      for (std::size_t node = 0; node < nominal.size(); ++node) {
        active_node = node;
        const auto started = Clock::now();
        const auto& row = nominal[node];
        const int64_t stamp = csv::Time(row[0]);
        const double elapsed = (stamp - csv::Time(nominal.front()[0])) * 1e-9;
        const bool map_available =
            !(map_loss_end > 0 && elapsed >= map_loss_begin &&
              elapsed < map_loss_end);
        if (node) {
          const double dt = (stamp - csv::Time(nominal[node - 1][0])) * 1e-9;
          if (dt <= 0) throw std::runtime_error("non-increasing nominal timestamps");
          for (int axis = 0; axis < 3; ++axis) {
            error[axis] += 0.005 * std::sqrt(dt) * normal(random);
            error[6 + axis] += 0.0005 * std::sqrt(dt) * normal(random);
            if (model == "ar_modeled" || model == "ar_unmodeled")
              error[15 + axis] = 0.85 * error[15 + axis] +
                  0.04 * std::sqrt(1 - 0.85 * 0.85) * normal(random);
          }
        }
        for (int axis = 0; axis < 3; ++axis)
          truth.translation()[axis] = csv::Number(row[1 + axis]);
        const Eigen::Quaterniond rotation(csv::Number(row[7]), csv::Number(row[4]),
                                          csv::Number(row[5]), csv::Number(row[6]));
        if (std::abs(rotation.norm() - 1) > 1e-9)
          throw std::runtime_error("nominal rotation is not unit");
        truth.linear() = rotation.toRotationMatrix();
        u::LocalState local;
        local.stamp = {stamp * 1e-9, stamp * 1e-9, node + 1, "unix"};
        local.epoch = {"synthetic-correction-error", 1};
        local.position = truth.translation() - error.head<3>();
        local.orientation = rotation * u::ExpRotation(-error.segment<3>(6));
        local.valid = true;
        local.covariance = u::Matrix15d::Identity() * 0.0001;
        local.covariance.topLeftCorner<3, 3>() =
            Eigen::Matrix3d::Identity() * (0.0004 + elapsed * 0.000025);
        local.covariance.block<3, 3>(6, 6) =
            Eigen::Matrix3d::Identity() * (0.000004 + elapsed * 0.00000025);
        const bool modeled = model == "ar_modeled" || model == "shared_modeled";
        if (modeled) {
          local.joint_error.emplace();
          auto& joint = *local.joint_error;
          joint.covariance = Eigen::MatrixXd::Zero(18, 18);
          joint.covariance.topLeftCorner<15, 15>() = local.covariance;
          joint.covariance.bottomRightCorner<3, 3>() =
              Eigen::Matrix3d::Identity() * 0.0016;
          joint.output_jacobian = Eigen::MatrixXd::Zero(15, 18);
          joint.output_jacobian.leftCols<15>().setIdentity();
        }
        const auto observation = [&](bool horizontal) {
          auto value =
              Observation(local, truth, error, model, horizontal, &random);
          Eigen::Matrix<double, 6, 1> noise;
          noise.head<3>() = value.pose.translation() - truth.translation();
          noise.tail<3>() = u::LogRotation(Eigen::Quaterniond(
              value.pose.linear() * truth.linear().transpose()));
          generated << model << ',' << trial << ',' << node << ','
                    << value.source << ',';
          if (horizontal)
            Matrix(generated, noise.head<2>());
          else
            Matrix(generated, noise);
          generated << ',';
          if (horizontal)
            Matrix(generated, value.covariance.topLeftCorner<2, 2>());
          else
            Matrix(generated, value.covariance);
          generated << ',' << (horizontal || map_available) << '\n';
          return value;
        };
        u::Result status;
        if (!node) {
          stage = "anchor";
          status = graph.Initialize(local, observation(false));
          Require(status);
          ++accepted;
        } else {
          u::MotionIncrement motion;
          Require(u::ComputeMotionIncrement(
              previous, local, previous.covariance,
              {{"imu", previous.stamp.sequence}, {"imu", local.stamp.sequence}}, &motion));
          if (modeled) {
            motion.joint_cross_covariance = previous.joint_error->covariance;
            if (model == "ar_modeled")
              motion.joint_cross_covariance.bottomRightCorner<3, 3>() *= 0.85;
          }
          stage = "motion";
          Require(graph.AddLocal(local, motion));
        }
        for (const bool horizontal : {false, true}) {
          if (!node && !horizontal) continue;
          const auto measured = observation(horizontal);
          if (!horizontal && !map_available) continue;
          stage = horizontal ? "gnss" : "map";
          openloc::localization::ObservationInnovation innovation;
          status = graph.Observe(measured, &innovation);
          accepted += status.ok();
          rejected += !status.ok();
          decisions << model << ',' << trial << ',' << node << ','
                    << measured.source << ',' << stamp << ',' << stamp << ','
                    << static_cast<int>(status.reason) << ',';
          csv::Text(decisions, status.message);
          decisions << ',' << innovation.available << ',';
          Matrix(decisions, innovation.residual);
          decisions << ',';
          Matrix(decisions, innovation.covariance);
          decisions << ',' << innovation.nis << ',' << innovation.gate_threshold << '\n';
          Require(status);
        }
        timings.push_back(std::chrono::duration<double, std::milli>(
            Clock::now() - started).count());
        previous = local;
      }
      u::GraphEvaluationSnapshot lag, batch;
      stage = "retained_evaluation";
      Require(graph.EvaluateForTesting(false, &lag));
      stage = "full_batch_evaluation";
      Require(graph.EvaluateForTesting(true, &batch));
      Eigen::Matrix<double, 6, 1> pose_error;
      pose_error.head<3>() = lag.map_from_base.translation() - truth.translation();
      pose_error.tail<3>() = u::LogRotation(Eigen::Quaterniond(
          lag.map_from_base.linear() * truth.linear().transpose()));
      Eigen::LLT<u::Matrix6d> inverse(lag.pose_covariance);
      if (inverse.info() != Eigen::Success)
        throw std::runtime_error("NEES pose covariance is not positive definite");
      const double nees = pose_error.dot(inverse.solve(pose_error));
      Eigen::SelfAdjointEigenSolver<u::Matrix6d> information(
          inverse.solve(u::Matrix6d::Identity()));
      std::sort(timings.begin(), timings.end());
      results << model << ',' << trial << ',' << seed + trial << ','
              << nominal.back()[0] << ',' << nominal.size() << ','
              << lag.retained_nodes << ',' << batch.factors.size() << ','
              << accepted << ',' << rejected << ',' << nees << ',';
      Matrix(results, pose_error);
      results << ',';
      Matrix(results, lag.pose_covariance);
      results << ',' << (lag.map_from_base.translation() - batch.map_from_base.translation()).norm()
              << ',' << u::LogRotation(Eigen::Quaterniond(
                  lag.map_from_base.linear() * batch.map_from_base.linear().transpose())).norm()
              << ',' << batch.objective << ',' << batch.iterations << ','
              << batch.maximum_exact_residual << ',' << lag.maximum_exact_residual << ',';
      Matrix(results, information.eigenvalues());
      results << ',' << information.eigenvalues().maxCoeff() /
                           information.eigenvalues().minCoeff()
              << ',' << timings[timings.size() / 2] << ','
              << timings[static_cast<std::size_t>(0.99 * (timings.size() - 1))]
              << ',' << timings.back() << ',' << lag.retained_relinearizations << '\n';
      for (const auto* snapshot : {&lag, &batch}) {
        std::size_t id = 0;
        for (const auto& factor : snapshot->factors) {
          factors << model << ',' << trial << ',' << nominal.back()[0] << ','
                  << (snapshot == &lag ? "retained" : "full_batch") << ','
                  << ++id << ',' << factor.type << ',';
          for (std::size_t k = 0; k < factor.keys.size(); ++k)
            factors << (k ? ";" : "") << factor.keys[k];
          factors << ',';
          Matrix(factors, factor.residual);
          factors << ',' << factor.error << ',' << factor.weight
                  << ",final_linearization\n";
        }
      }
      } catch (const std::exception& error) {
        failures << model << ',' << trial << ',' << seed + trial << ','
                 << active_node << ',' << stage << ',';
        csv::Text(failures, error.what());
        failures << '\n';
      }
    }
  }
  if (!results || !decisions || !factors || !failures || !generated)
    throw std::runtime_error("incomplete evaluation output");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 6 && argc != 8)
      throw std::runtime_error(
          "usage: fixed_lag_evaluation NOMINAL NEW_OUTPUT TRIALS SEED LAG_S "
          "[MAP_LOSS_BEGIN_S MAP_LOSS_END_S]");
    Run(argv[1], argv[2], csv::Time(argv[3]), csv::Time(argv[4]),
        csv::Number(argv[5]), argc == 8 ? csv::Number(argv[6]) : 0,
        argc == 8 ? csv::Number(argv[7]) : 0);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
