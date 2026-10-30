// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0

#include "candidates/global/fixed_lag_graph.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <utility>

#include "Eigen/Cholesky"
#include "Eigen/Eigenvalues"
#include "Eigen/LU"
#include "gtsam/base/numericalDerivative.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/linear/JacobianFactor.h"
#include "gtsam/linear/GaussianConditional.h"
#include "gtsam/linear/linearExceptions.h"
#include "gtsam/nonlinear/IncrementalFixedLagSmoother.h"
#include "gtsam/nonlinear/GaussNewtonOptimizer.h"
#include "gtsam/nonlinear/LinearContainerFactor.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/NonlinearFactor.h"
#include "gtsam/slam/PriorFactor.h"

namespace openloc {
namespace localization {
namespace unified {

Result ConditionalRowSigmas(const gtsam::GaussianConditional& conditional,
                            Eigen::VectorXd* sigmas) {
  const Eigen::Index rows = conditional.R().rows();
  if (!sigmas || rows <= 0)
    return {Reason::INVALID_INPUT, "Invalid conditional noise query"};
  const auto& noise = conditional.get_model();
  if (!noise) {
    *sigmas = Eigen::VectorXd::Ones(rows);
    return {};
  }
  const auto& full = noise->sigmas();
  if (full.size() < rows ||
      full.size() - rows > conditional.matrixObject().rowStart() ||
      !full.allFinite() || (full.array() < 0).any())
    return {Reason::DEGENERATE, "Conditional noise does not match its active rows"};
  // GTSAM marginalizeLeaves crops the frontal matrix view, but retains the
  // original noise vector. The surviving equations are its trailing rows.
  *sigmas = full.tail(rows);
  return {};
}

namespace {

using Clock = std::chrono::steady_clock;
using OptionalJacobian = gtsam::OptionalMatrixType;

gtsam::Key PoseKey(uint64_t index) { return gtsam::Symbol('x', index); }
gtsam::Key ErrorKey(uint64_t index) { return gtsam::Symbol('e', index); }

class FrozenMarginalFactor final : public gtsam::NoiseModelFactor {
 public:
  FrozenMarginalFactor(const gtsam::JacobianFactor& factor,
                       const gtsam::Values& reference)
      : gtsam::NoiseModelFactor(
            factor.get_model() ? factor.get_model()
                               : gtsam::noiseModel::Unit::Create(factor.getb().size()),
            factor.keys()),
        factor_(factor) {
    for (const auto key : keys()) reference_.insert(key, reference.at(key));
  }

  gtsam::Vector unwhitenedError(
      const gtsam::Values& values,
      gtsam::OptionalMatrixVecType derivatives = nullptr) const override {
    gtsam::Values subset;
    for (const auto key : keys()) subset.insert(key, values.at(key));
    if (derivatives) {
      derivatives->resize(size());
      for (std::size_t i = 0; i < size(); ++i) {
        const auto key = keys()[i];
        const auto block = factor_.getA(factor_.begin() + i);
        if (gtsam::Symbol(key).chr() == 'x') {
          const auto& reference = reference_.at<gtsam::Pose3>(key);
          const std::function<gtsam::Vector(const gtsam::Pose3&)> chart =
              [&](const gtsam::Pose3& pose) {
                return reference.localCoordinates(pose);
              };
          (*derivatives)[i] = block *
              gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
                  chart, values.at<gtsam::Pose3>(key));
        } else {
          (*derivatives)[i] = block;
        }
      }
    }
    return factor_.unweighted_error(reference_.localCoordinates(subset));
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<FrozenMarginalFactor>(*this);
  }

 private:
  gtsam::JacobianFactor factor_;
  gtsam::Values reference_;
};

gtsam::ISAM2Params Parameters() {
  gtsam::ISAM2Params params;
  params.factorization = gtsam::ISAM2Params::QR;
  params.findUnusedFactorSlots = true;
  params.relinearizeSkip = 1;
  params.relinearizeThreshold = 1e-6;
  // Hard constraints require a complete delta, not wildfire-truncated updates.
  params.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.0);
  return params;
}

class AlignmentSmoother final : public gtsam::IncrementalFixedLagSmoother {
 public:
  explicit AlignmentSmoother(double lag)
      : gtsam::IncrementalFixedLagSmoother(lag, Parameters()) {}

  gtsam::NonlinearFactorGraph RetainedFactors() const {
    gtsam::NonlinearFactorGraph factors;
    for (const auto& factor : getFactors()) {
      if (!factor) continue;
      const auto* marginal =
          dynamic_cast<const gtsam::LinearContainerFactor*>(factor.get());
      if (marginal) {
        // Keep the marginalized information and original chart. Only the chart
        // derivative changes; eliminated observations are never reconstructed.
        const auto& reference = marginal->hasLinearizationPoint()
            ? *marginal->linearizationPoint() : getLinearizationPoint();
        const gtsam::JacobianFactor linear(*marginal->linearize(reference));
        factors.emplace_shared<FrozenMarginalFactor>(
            linear, reference);
      } else {
        factors.push_back(factor);
      }
    }
    return factors;
  }

  void AdoptEstimate(const gtsam::NonlinearFactorGraph& factors,
                     const gtsam::Values& estimate) {
    class Posterior final : public gtsam::ISAM2 {
     public:
      Posterior() : gtsam::ISAM2(Parameters()) {}
      void KeepAcceptedPoint() {
        // The LM iterate owns the mean. This undamped tree supplies covariance;
        // do not apply its unrestricted GN step a second time.
        getDelta();
        delta_.setZero();
        deltaReplacedMask_.clear();
      }
    };
    gtsam::FastMap<gtsam::Key, int> groups;
    for (const auto& value : estimate)
      groups[value.key] = gtsam::Symbol(value.key).chr() == 'x' ? 0 : 1;
    Posterior replacement;
    isamResult_ = replacement.update(factors, estimate, {}, groups);
    replacement.KeepAcceptedPoint();
    isam_ = std::move(replacement);
  }

  void RelinearizeRetained() {
    const auto estimate = calculateEstimate();
    const auto factors = RetainedFactors();
    gtsam::FastMap<gtsam::Key, int> groups;
    for (const auto& value : estimate)
      groups[value.key] = gtsam::Symbol(value.key).chr() == 'x' ? 0 : 1;
    gtsam::ISAM2 replacement(Parameters());
    isamResult_ = replacement.update(factors, estimate, {}, groups);
    isam_ = std::move(replacement);
  }

  void UpdateConditionalFactors(
      const gtsam::NonlinearFactorGraph& factors,
      const gtsam::Values& values = {},
      const gtsam::FixedLagSmoother::KeyTimestampMap& timestamps = {}) {
    updateKeyTimestampMap(timestamps);
    gtsam::FastMap<gtsam::Key, int> groups;
    gtsam::FastList<gtsam::Key> keys;
    for (const auto& value : getLinearizationPoint()) {
      keys.push_back(value.key);
      groups[value.key] = gtsam::Symbol(value.key).chr() == 'x' ? 0 : 1;
    }
    for (const auto& value : values)
      groups[value.key] = gtsam::Symbol(value.key).chr() == 'x' ? 0 : 1;
    // A cross-time exact source joins old pose and new error cliques. Eliminate
    // all retained poses before the unit-error priors, from the SAME factors;
    // do not re-eliminate a separator assembled from constrained conditionals.
    isamResult_ = isam_.update(factors, values, {}, groups, {}, keys, true);
  }
};

Result PosteriorCovariance(
    const gtsam::IncrementalFixedLagSmoother& smoother,
    const std::vector<std::pair<gtsam::Key, Eigen::Index>>& query,
    Eigen::MatrixXd* covariance) {
  Eigen::Index dimension = 0;
  for (const auto& entry : query) dimension += entry.second;
  std::map<gtsam::Key, Eigen::MatrixXd> adjoints;
  Eigen::Index offset = 0;
  for (const auto& entry : query) {
    if (!smoother.getLinearizationPoint().exists(entry.first) ||
        entry.second <= 0 || adjoints.count(entry.first))
      return {Reason::INVALID_INPUT, "Invalid posterior covariance query"};
    auto& loading = adjoints[entry.first];
    loading = Eigen::MatrixXd::Zero(entry.second, dimension);
    loading.middleCols(offset, entry.second).setIdentity();
    offset += entry.second;
  }
  Eigen::MatrixXd result = Eigen::MatrixXd::Zero(dimension, dimension);
  // R x = D epsilon also holds for exact constraints (zero entries of D).
  // Solve R^T y = selection^T leaf-to-root, then accumulate y^T D^2 y.
  // Re-eliminating constrained cliques can lose rank in GTSAM's scalar QR.
  const auto visit = [&](auto&& self, const auto& clique) -> Result {
    for (const auto& child : clique->children) {
      const auto valid = self(self, child);
      if (!valid.ok()) return valid;
    }
    const auto& conditional = *clique->conditional();
    const Eigen::MatrixXd r = conditional.R();
    Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(r.rows(), dimension);
    Eigen::Index row = 0;
    for (auto it = conditional.beginFrontals();
         it != conditional.endFrontals(); ++it) {
      const auto found = adjoints.find(*it);
      const Eigen::Index size = conditional.getDim(it);
      if (found != adjoints.end()) {
        if (found->second.rows() != size)
          return {Reason::INVALID_INPUT, "Posterior query dimension mismatch"};
        rhs.middleRows(row, size) = found->second;
        adjoints.erase(found);
      }
      row += size;
    }
    if (!r.allFinite() || (r.diagonal().array() == 0).any())
      return {Reason::DEGENERATE, "Singular posterior triangular factor"};
    const Eigen::MatrixXd solved =
        r.transpose().triangularView<Eigen::Lower>().solve(rhs);
    if (!solved.allFinite())
      return {Reason::DEGENERATE, "Posterior covariance triangular solve failed"};
    Eigen::VectorXd sigmas;
    const auto noise_status = ConditionalRowSigmas(conditional, &sigmas);
    if (!noise_status.ok()) return noise_status;
    const Eigen::MatrixXd weighted = sigmas.asDiagonal() * solved;
    result.noalias() += weighted.transpose() * weighted;
    for (auto it = conditional.beginParents();
         it != conditional.endParents(); ++it) {
      auto& parent = adjoints[*it];
      if (parent.size() == 0)
        parent = Eigen::MatrixXd::Zero(conditional.getDim(it), dimension);
      parent.noalias() -= conditional.S(it).transpose() * solved;
    }
    return {};
  };
  for (const auto& root : smoother.getISAM2().roots()) {
    const auto valid = visit(visit, root);
    if (!valid.ok()) return valid;
  }
  if (!adjoints.empty())
    return {Reason::DEGENERATE,
            "Incomplete posterior covariance: unresolved adjoint keys=" +
                std::to_string(adjoints.size())};
  if (!result.allFinite())
    return {Reason::DEGENERATE, "Nonfinite posterior covariance accumulation"};
  *covariance = (0.5 * (result + result.transpose())).eval();
  return {};
}

gtsam::Pose3 ToPose(const Eigen::Isometry3d& pose) {
  return gtsam::Pose3(pose.matrix());
}

Eigen::Isometry3d FromPose(const gtsam::Pose3& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.matrix() = pose.matrix();
  return result;
}

class ErrorChainFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> {
 public:
  ErrorChainFactor(gtsam::Key first, gtsam::Key second,
                   const Eigen::MatrixXd& transition,
                   const Eigen::MatrixXd& projection,
                   const gtsam::SharedNoiseModel& noise,
                   const Eigen::VectorXd& sigmas)
      : gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector>(
            noise, first, second),
        transition_(projection * transition), projection_(projection),
        sigmas_(sigmas) {}

  gtsam::Vector evaluateError(
      const gtsam::Vector& first, const gtsam::Vector& second,
      OptionalJacobian h1 = OptionalNone,
      OptionalJacobian h2 = OptionalNone) const override {
    if (h1) {
      *h1 = -transition_;
    }
    if (h2) {
      *h2 = projection_;
    }
    return projection_ * second - transition_ * first;
  }

  bool ConstraintsConsistent(const gtsam::Vector& first,
                            const gtsam::Vector& second) const {
    const auto residual = evaluateError(first, second);
    if (!residual.allFinite()) return false;
    for (Eigen::Index i = 0; i < sigmas_.size(); ++i) {
      if (sigmas_[i] == 0 && std::abs(residual[i]) > 1e-6) return false;
    }
    return true;
  }

 private:
  Eigen::MatrixXd transition_;
  Eigen::MatrixXd projection_;
  Eigen::VectorXd sigmas_;
};

class MotionFactor final
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector, gtsam::Pose3,
                                      gtsam::Vector> {
 public:
  MotionFactor(uint64_t first, uint64_t second, const MotionIncrement& motion,
               const LocalState& start, const LocalState& end)
      : gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector, gtsam::Pose3,
                                 gtsam::Vector>(
            gtsam::noiseModel::Constrained::All(6), PoseKey(first),
            ErrorKey(first), PoseKey(second), ErrorKey(second)),
        delta_(motion.delta) {
    Matrix6x15d start_jacobian, end_jacobian;
    RelativeMotionJacobians(motion.start_pose, motion.delta,
                            &start_jacobian, &end_jacobian);
    first_ = start_jacobian * JointOutputJacobian(start);
    second_ = end_jacobian * JointOutputJacobian(end);
  }

  gtsam::Vector RelativeError(const gtsam::Pose3& first,
                              const gtsam::Pose3& second) const {
    const Eigen::Isometry3d relative = FromPose(first.between(second));
    gtsam::Vector error(6);
    error.head<3>() = relative.translation() - delta_.translation();
    error.tail<3>() = LogRotation(
        Eigen::Quaterniond(delta_.linear().transpose() * relative.linear()));
    return error;
  }

  gtsam::Vector evaluateError(
      const gtsam::Pose3& first, const gtsam::Vector& first_error,
      const gtsam::Pose3& second, const gtsam::Vector& second_error,
      OptionalJacobian h1 = OptionalNone, OptionalJacobian h2 = OptionalNone,
      OptionalJacobian h3 = OptionalNone,
      OptionalJacobian h4 = OptionalNone) const override {
    if (h1) {
      const std::function<gtsam::Vector(const gtsam::Pose3&)> function =
          [&](const gtsam::Pose3& value) {
            return RelativeError(value, second);
          };
      *h1 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(function,
                                                                      first);
    }
    if (h2) {
      *h2 = -first_;
    }
    if (h3) {
      const std::function<gtsam::Vector(const gtsam::Pose3&)> function =
          [&](const gtsam::Pose3& value) {
            return RelativeError(first, value);
          };
      *h3 = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(function,
                                                                      second);
    }
    if (h4) {
      *h4 = -second_;
    }
    return RelativeError(first, second) - first_ * first_error -
           second_ * second_error;
  }

 private:
  Eigen::Isometry3d delta_;
  Eigen::MatrixXd first_;
  Eigen::MatrixXd second_;
};

Eigen::MatrixXd ObservationNoise(const GlobalObservation& observation) {
  if (observation.kind == GlobalObservation::Kind::HEADING) {
    return Eigen::Matrix<double, 1, 1>::Constant(observation.heading_variance);
  }
  if (observation.kind == GlobalObservation::Kind::HORIZONTAL_POSITION) {
    return observation.covariance.topLeftCorner<2, 2>();
  }
  if (observation.kind == GlobalObservation::Kind::POSITION) {
    return observation.covariance.topLeftCorner<3, 3>();
  }
  if (observation.kind == GlobalObservation::Kind::PROJECTED_POSE) {
    return observation.projected_covariance;
  }
  return observation.covariance;
}

class ObservationFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  ObservationFactor(gtsam::Key key, const GlobalObservation& observation,
                    double robust_threshold)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(
            gtsam::noiseModel::Robust::Create(
                gtsam::noiseModel::mEstimator::Huber::Create(robust_threshold),
                gtsam::noiseModel::Gaussian::Covariance(
                    ObservationNoise(observation))),
            key),
        observation_(observation) {}

  gtsam::Vector PoseResidual(const gtsam::Pose3& pose) const {
    const Eigen::Isometry3d estimate = FromPose(pose);
    if (observation_.kind == GlobalObservation::Kind::HEADING) {
      const Eigen::Vector3d direction =
          estimate.linear() * observation_.direction_in_base;
      const double angle = std::atan2(direction.y(), direction.x()) -
                           observation_.heading_rad;
      return gtsam::Vector1(std::atan2(std::sin(angle), std::cos(angle)));
    }
    gtsam::Vector residual(6);
    residual.head<3>() =
        estimate.translation() - observation_.pose.translation();
    residual.tail<3>() = LogRotation(Eigen::Quaterniond(
        estimate.linear() * observation_.pose.linear().transpose()));
    if (observation_.kind == GlobalObservation::Kind::PROJECTED_POSE) {
      return observation_.projection * residual;
    }
    return residual;
  }

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      OptionalJacobian h = OptionalNone) const override {
    if (observation_.kind == GlobalObservation::Kind::POSITION ||
        observation_.kind == GlobalObservation::Kind::HORIZONTAL_POSITION) {
      const Eigen::Index dimension =
          observation_.kind == GlobalObservation::Kind::POSITION ? 3 : 2;
      const Eigen::Matrix3d rotation = pose.rotation().matrix();
      if (h) {
        Eigen::Matrix<double, 3, 6> jacobian;
        jacobian.topLeftCorner<3, 3>() =
            -rotation * Skew(observation_.point_in_base);
        jacobian.topRightCorner<3, 3>() = rotation;
        *h = jacobian.topRows(dimension);
      }
      const Eigen::Vector3d antenna =
          pose.translation() + rotation * observation_.point_in_base;
      return (antenna - observation_.pose.translation()).head(dimension);
    }
    if (h) {
      const std::function<gtsam::Vector(const gtsam::Pose3&)> function =
          [&](const gtsam::Pose3& value) { return PoseResidual(value); };
      *h = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(function,
                                                                     pose);
    }
    return PoseResidual(pose);
  }

 private:
  GlobalObservation observation_;
};

struct ConditionalNoise {
  Eigen::MatrixXd projection;
  Eigen::VectorXd sigmas;
  gtsam::SharedNoiseModel model;
};

Result MakeConditionalNoise(const Eigen::MatrixXd& covariance,
                            const Eigen::MatrixXd& marginal,
                            double robust_threshold, ConditionalNoise* output) {
  const auto scale = marginal.diagonal().cwiseSqrt().cwiseInverse().eval();
  const Eigen::MatrixXd normalized =
      scale.asDiagonal() * covariance * scale.asDiagonal();
  if (!normalized.allFinite())
    return {Reason::INVALID_INPUT, "Conditional covariance scaling overflow"};
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> spectrum(normalized);
  if (spectrum.info() != Eigen::Success || !spectrum.eigenvalues().allFinite())
    return {Reason::INVALID_INPUT, "Conditional covariance decomposition failed"};
  const double tolerance = 64 * std::numeric_limits<double>::epsilon() *
      std::max(1.0, spectrum.eigenvalues().cwiseAbs().maxCoeff());
  if (spectrum.eigenvalues().minCoeff() < -tolerance)
    return {Reason::INVALID_INPUT, "Conditional covariance is not PSD"};
  output->projection = spectrum.eigenvectors().transpose() * scale.asDiagonal();
  if (!output->projection.allFinite())
    return {Reason::INVALID_INPUT, "Conditional covariance projection overflow"};
  output->sigmas.resize(covariance.rows());
  for (Eigen::Index i = 0; i < covariance.rows(); ++i)
    output->sigmas[i] = spectrum.eigenvalues()[i] <= tolerance
        ? 0 : std::sqrt(spectrum.eigenvalues()[i]);
  if (output->sigmas.minCoeff() > 0) {
    output->projection.setIdentity();
    output->model = gtsam::noiseModel::Gaussian::Covariance(covariance);
    if (robust_threshold > 0)
      output->model = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(robust_threshold),
          output->model);
  } else {
    // Exact equations must not be softened by a robust residual weight.
    output->model = gtsam::noiseModel::Constrained::MixedSigmas(output->sigmas);
  }
  return {};
}

class ConditionalObservationFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector> {
 public:
  ConditionalObservationFactor(uint64_t key, uint64_t error_key,
                              const GlobalObservation& observation,
                              const Eigen::MatrixXd& loading,
                              const ConditionalNoise& noise,
                              double robust_threshold)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector>(
            noise.model, PoseKey(key), ErrorKey(error_key)),
        residual_(PoseKey(key), observation, robust_threshold),
        loading_(noise.projection * loading),
        projection_(noise.projection), sigmas_(noise.sigmas) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose, const gtsam::Vector& error,
      OptionalJacobian h1 = OptionalNone,
      OptionalJacobian h2 = OptionalNone) const override {
    if (h2) *h2 = loading_;
    const auto residual = residual_.evaluateError(pose, h1);
    if (h1) *h1 = (projection_ * *h1).eval();
    return projection_ * residual + loading_ * error;
  }

  bool ConstraintsConsistent(const gtsam::Pose3& pose,
                            const gtsam::Vector& error) const {
    const auto residual = evaluateError(pose, error);
    if (!residual.allFinite()) return false;
    for (Eigen::Index i = 0; i < sigmas_.size(); ++i)
      if (sigmas_[i] == 0 && std::abs(residual[i]) > 1e-6) return false;
    return true;
  }

 private:
  ObservationFactor residual_;
  Eigen::MatrixXd loading_;
  Eigen::MatrixXd projection_;
  Eigen::VectorXd sigmas_;
};

Result ConditionalObservationModel(const GlobalObservation& observation,
                                   const LocalState& local,
                                   Eigen::MatrixXd* loading,
                                   Eigen::MatrixXd* noise) {
  const auto& cross = observation.local_error_cross_covariance;
  const auto marginal = ObservationNoise(observation);
  const auto valid = ValidateJointLocalError(local);
  if (!valid.ok()) return valid;
  const auto covariance = JointErrorCovariance(local);
  if (!observation.conditional_local_error_qualified ||
      observation.independent_of_local ||
      observation.local_error_sequence != local.stamp.sequence ||
      observation.local_error_time.value_or(observation.stamp.time) != local.stamp.time ||
      cross.rows() != covariance.rows() || cross.cols() != marginal.rows() ||
      !cross.allFinite()) {
    return {Reason::CORRELATION_UNQUALIFIED,
            "Conditional observation needs its qualified exact local error state"};
  }
  *loading = covariance.llt().solve(cross).transpose();
  *noise = marginal - *loading * cross;
  *noise = (0.5 * (*noise + noise->transpose())).eval();
  if (!loading->allFinite() || !noise->allFinite()) {
    return {Reason::INVALID_INPUT,
            "Conditional observation residual covariance is nonfinite"};
  }
  ConditionalNoise model;
  return MakeConditionalNoise(*noise, marginal, 0, &model);
}

struct JointMotionModel {
  MotionIncrement verified;
  Eigen::MatrixXd transition;
  ConditionalNoise noise;
};

Result ModelMotion(const LocalState& previous, const LocalState& local,
                   const MotionIncrement& motion, const FixedLagConfig& config,
                   JointMotionModel* model) {
  if (motion.epoch != previous.epoch || local.epoch != motion.epoch)
    return {Reason::EPOCH_MISMATCH, "Graph motion changed ODOM epoch"};
  if (motion.start.time != previous.stamp.time ||
      motion.end.time != local.stamp.time ||
      motion.start.sequence != previous.stamp.sequence ||
      motion.end.sequence != local.stamp.sequence ||
      !motion.start_covariance.isApprox(previous.covariance, 1e-9) ||
      !motion.end_covariance.isApprox(local.covariance, 1e-9))
    return {Reason::HISTORY_UNAVAILABLE, "Graph requires a contiguous local error chain"};
  auto valid = ComputeMotionIncrement(previous, local, motion.cross_covariance,
                                      motion.sources, &model->verified);
  if (!valid.ok()) return valid;
  if (!motion.delta.matrix().isApprox(model->verified.delta.matrix(), 1e-9) ||
      motion.correlation_group != model->verified.correlation_group)
    return {Reason::INVALID_INPUT, "Motion identity or relative pose mismatch"};
  if (local.joint_error &&
      local.joint_error->covariance.rows() > config.max_error_dimension)
    return {Reason::INVALID_INPUT, "Joint error dimension exceeds graph capacity"};
  valid = ValidateJointMotion(previous, local, motion);
  if (!valid.ok()) return valid;
  const auto previous_covariance = JointErrorCovariance(previous);
  const auto covariance = JointErrorCovariance(local);
  const Eigen::MatrixXd cross = previous.joint_error || local.joint_error
      ? motion.joint_cross_covariance : Eigen::MatrixXd(motion.cross_covariance);
  model->transition = previous_covariance.llt().solve(cross).transpose();
  Eigen::MatrixXd noise = covariance - model->transition * cross;
  noise = (0.5 * (noise + noise.transpose())).eval();
  if (!model->transition.allFinite() || !noise.allFinite())
    return {Reason::INVALID_INPUT, "Nonfinite conditional motion model"};
  return MakeConditionalNoise(noise, covariance, 0, &model->noise);
}

Result CheckObservation(const GlobalObservation& observation) {
  const bool observes_rotation =
      observation.kind == GlobalObservation::Kind::POSE ||
      observation.kind == GlobalObservation::Kind::PROJECTED_POSE;
  if (!ValidGlobalObservationGeometry(observation) ||
      (observes_rotation &&
       (!(observation.pose.linear().transpose() * observation.pose.linear())
             .isApprox(Eigen::Matrix3d::Identity(), 1e-9) ||
        std::abs(observation.pose.linear().determinant() - 1.0) > 1e-9)) ||
      !observation.quality_valid || observation.ambiguous ||
      (!observation.independent_of_local &&
       !observation.conditional_local_error_qualified) ||
      (observation.conditional_local_error_qualified &&
       observation.independent_of_local) ||
      (!observation.conditional_local_error_qualified &&
       observation.local_error_cross_covariance.size() != 0) ||
      observation.source.empty() ||
      observation.map_id.empty() || observation.map_version.empty() ||
      observation.calibration_id.empty() || observation.stamp.sequence == 0 ||
      !std::isfinite(observation.stamp.time) || observation.stamp.time <= 0.0 ||
      !std::isfinite(observation.stamp.receive_time) ||
      observation.stamp.receive_time <= 0.0) {
    return {Reason::INVALID_INPUT,
            "Graph needs valid, explicitly independent observed information"};
  }
  return {};
}

}  // namespace

struct FixedLagGraph::Impl {
  struct Node {
    uint64_t key;
    LocalState local;
  };
  explicit Impl(double lag) : smoother(lag) {}
  AlignmentSmoother smoother;
  std::map<double, Node> nodes;
  std::map<std::pair<std::string, uint64_t>, double> observations;
  std::map<std::string, uint64_t> retired_sequences;
  std::set<std::string> sources;
  GlobalObservation anchor;
  GlobalState output;
  uint64_t next_key = 1;
  gtsam::NonlinearFactorGraph evaluation_factors;
  gtsam::Values evaluation_initial;
  uint64_t retained_relinearizations = 0;
  uint32_t solve_iterations = 0;
  uint32_t damped_iterations = 0;
  double maximum_motion_residual = 0;

  void Archive(const gtsam::NonlinearFactorGraph& factors,
               const gtsam::Values& values) {
    evaluation_factors.push_back(factors);
    evaluation_initial.insert(values);
  }

  Result Finalize(const FixedLagConfig& config, Clock::time_point started) {
    damped_iterations = 0;
    std::unique_ptr<gtsam::LevenbergMarquardtOptimizer> refinement;
    for (uint32_t iteration = 0; iteration < config.max_iterations;
         ++iteration) {
      solve_iterations = iteration + 1;
      const gtsam::Values estimate =
          refinement ? refinement->values() : smoother.calculateEstimate();
      for (const auto& node : nodes) {
        const auto& error =
            estimate.at<gtsam::Vector>(ErrorKey(node.second.key));
        const auto projection = JointOutputJacobian(node.second.local);
        if (error.size() != projection.cols() || !error.allFinite() ||
            (projection * error).segment<3>(6).norm() >
                config.max_linearization_angle) {
          return {Reason::INNOVATION_REJECTED,
                  "Global nuisance error exceeded its linearization domain"};
        }
      }
      bool consistent = true;
      maximum_motion_residual = 0;
      const auto& factors =
          refinement ? refinement->graph() : smoother.getFactors();
      for (const auto& factor : factors) {
        const auto* marginal =
            dynamic_cast<const FrozenMarginalFactor*>(factor.get());
        if (marginal) {
          const auto constrained =
              std::dynamic_pointer_cast<gtsam::noiseModel::Constrained>(
                  marginal->noiseModel());
          if (constrained) {
            const auto residual = marginal->unwhitenedError(estimate);
            consistent = consistent && residual.allFinite();
            for (Eigen::Index i = 0; i < residual.size(); ++i)
              if (constrained->sigmas()[i] == 0)
                consistent = consistent && std::abs(residual[i]) <= 1e-6;
          }
        }
        const auto* observation =
            dynamic_cast<const ConditionalObservationFactor*>(factor.get());
        if (observation)
          consistent = consistent && observation->ConstraintsConsistent(
              estimate.at<gtsam::Pose3>(observation->keys()[0]),
              estimate.at<gtsam::Vector>(observation->keys()[1]));
        const auto* chain = dynamic_cast<const ErrorChainFactor*>(factor.get());
        if (chain != nullptr) {
          consistent = consistent && chain->ConstraintsConsistent(
              estimate.at<gtsam::Vector>(chain->keys()[0]),
              estimate.at<gtsam::Vector>(chain->keys()[1]));
        }
        const auto* motion = dynamic_cast<const MotionFactor*>(factor.get());
        if (motion == nullptr) {
          continue;
        }
        const auto& keys = motion->keys();
        const gtsam::Vector residual =
            motion->evaluateError(estimate.at<gtsam::Pose3>(keys[0]),
                                  estimate.at<gtsam::Vector>(keys[1]),
                                  estimate.at<gtsam::Pose3>(keys[2]),
                                  estimate.at<gtsam::Vector>(keys[3]));
        if (!residual.allFinite()) {
          return {Reason::INVALID_INPUT, "Nonfinite graph motion constraint"};
        }
        maximum_motion_residual = std::max(
            maximum_motion_residual, residual.cwiseAbs().maxCoeff());
        consistent = consistent && residual.cwiseAbs().maxCoeff() <= 1e-6;
      }
      if (consistent) {
        if (refinement) {
          smoother.AdoptEstimate(refinement->graph(), estimate);
          ++retained_relinearizations;
        }
        return Snapshot(config.solve_budget_ms, started);
      }
      if (std::chrono::duration<double, std::milli>(Clock::now() - started)
              .count() > config.solve_budget_ms) {
        return {Reason::GLOBAL_STALE, "Graph exhausted its solve budget"};
      }
      if (iteration + 1 < config.max_iterations) {
        if (iteration == 0) {
          if (!smoother.getISAM2().getFixedVariables().empty()) {
            smoother.RelinearizeRetained();
            ++retained_relinearizations;
          } else {
            smoother.update();
          }
          continue;
        }
        if (!refinement) {
          // Full GN steps can oscillate along weakly observed directions.
          // Damping controls the step, not the measurement information.
          auto parameters = gtsam::LevenbergMarquardtParams::CeresDefaults();
          parameters.setLinearSolverType("MULTIFRONTAL_QR");
          parameters.setMaxIterations(config.max_iterations - iteration - 1);
          refinement.reset(new gtsam::LevenbergMarquardtOptimizer(
              smoother.RetainedFactors(), estimate, parameters));
        }
        refinement->iterate();
        ++damped_iterations;
      }
    }
    return {Reason::MATCH_FAILED,
            "Graph motion constraints did not converge; maximum residual=" +
                std::to_string(maximum_motion_residual) +
                "; fixed linearization keys=" +
                std::to_string(smoother.getISAM2().getFixedVariables().size())};
  }

  Result Snapshot(double budget_ms, Clock::time_point started) {
    const auto& latest = nodes.rbegin()->second;
    const auto pose =
        smoother.calculateEstimate<gtsam::Pose3>(PoseKey(latest.key));
    const Eigen::Isometry3d map_base = FromPose(pose);
    gtsam::Matrix tangent;
    const auto marginal =
        PosteriorCovariance(smoother, {{PoseKey(latest.key), 6}}, &tangent);
    if (!marginal.ok()) return marginal;
    if (tangent.rows() != 6 || tangent.cols() != 6 ||
        !map_base.matrix().allFinite() || !tangent.allFinite()) {
      return {Reason::INVALID_INPUT, "Nonfinite graph pose/marginal"};
    }
    // GTSAM Pose3 tangent is [rotation_body, translation_body].
    Matrix6d axes = Matrix6d::Zero();
    axes.topRightCorner<3, 3>() = map_base.linear();
    axes.bottomLeftCorner<3, 3>() = map_base.linear();
    const Matrix6d covariance = axes * tangent * axes.transpose();
    if (!ValidCovariance(covariance)) {
      return {Reason::DEGENERATE, "Graph marginal is not full-rank"};
    }
    if (std::chrono::duration<double, std::milli>(Clock::now() - started)
            .count() > budget_ms) {
      return {Reason::GLOBAL_STALE,
              "Graph solve exceeded its publication budget"};
    }
    const Eigen::Isometry3d alignment = map_base * Pose(latest.local).inverse();
    const Matrix6d alignment_covariance =
        TransformCovarianceBound(map_base, latest.local, covariance);
    if (!alignment.matrix().allFinite() ||
        !ValidCovariance(alignment_covariance)) {
      return {Reason::INVALID_INPUT, "Graph alignment composition is invalid"};
    }
    output.map_to_odom = alignment;
    output.covariance = alignment_covariance;
    output.covariance_time = latest.local.stamp.time;
    output.epoch = latest.local.epoch;
    output.valid = true;
    ++output.correction_id;
    return {};
  }

  void Prune(double cutoff) {
    while (!nodes.empty() && nodes.begin()->first < cutoff) {
      nodes.erase(nodes.begin());
    }
    for (auto it = observations.begin(); it != observations.end();) {
      if (it->second < cutoff) {
        auto& frontier = retired_sequences[it->first.first];
        frontier = std::max(frontier, it->first.second);
        it = observations.erase(it);
      } else {
        ++it;
      }
    }
  }
};

FixedLagGraph::FixedLagGraph(const FixedLagConfig& config)
    : config_(config), impl_(new Impl(config.lag)) {}
FixedLagGraph::~FixedLagGraph() = default;

Result FixedLagGraph::ValidateConfig() const {
  for (double value :
       {config_.lag, config_.solve_budget_ms, config_.innovation_gate,
        config_.robust_threshold, config_.max_linearization_angle}) {
    if (!std::isfinite(value) || value <= 0.0) {
      return {Reason::CONFIG_INVALID, "Invalid fixed-lag graph budget"};
    }
  }
  if (config_.max_states < 2 || config_.max_observations < 1 ||
      config_.max_iterations == 0 || config_.max_iterations > 100 ||
      config_.max_error_dimension < 15 ||
      config_.max_linearization_angle >= 1.0) {
    return {Reason::CONFIG_INVALID, "Invalid fixed-lag graph capacity"};
  }
  return {};
}

bool FixedLagGraph::initialized() const { return !impl_->nodes.empty(); }
const GlobalState& FixedLagGraph::state() const { return impl_->output; }

GraphResourceUsage FixedLagGraph::ResourceUsage() const {
  GraphResourceUsage result;
  result.retained_nodes = impl_->nodes.size();
  result.retained_observations = impl_->observations.size();
  result.factor_slots = impl_->smoother.getFactors().size();
  for (const auto& factor : impl_->smoother.getFactors())
    if (factor) ++result.active_factors;
  result.archived_values = impl_->evaluation_initial.size();
  result.solve_iterations = impl_->solve_iterations;
  result.damped_iterations = impl_->damped_iterations;
  result.maximum_motion_residual = impl_->maximum_motion_residual;
  return result;
}

Result FixedLagGraph::EnableEvaluationHistory() {
  if (initialized())
    return {Reason::INVALID_INPUT, "Evaluation history must start before initialization"};
  retain_evaluation_history_ = true;
  return {};
}

Result FixedLagGraph::EvaluateForTesting(
    bool batch, GraphEvaluationSnapshot* output) const {
  if (!output || !initialized() || !retain_evaluation_history_)
    return {Reason::INVALID_INPUT, "No enabled initialized evaluation history"};
  GraphEvaluationSnapshot result;
  result.cutoff_time = impl_->nodes.rbegin()->first;
  result.retained_nodes = impl_->nodes.size();
  result.archived_nodes = impl_->evaluation_initial.size() / 2;
  result.retained_relinearizations = impl_->retained_relinearizations;
  AlignmentSmoother reference(1.0);
  const AlignmentSmoother* owner = &impl_->smoother;
  try {
    if (batch) {
      gtsam::GaussNewtonParams parameters;
      parameters.setLinearSolverType("MULTIFRONTAL_QR");
      parameters.setMaxIterations(30);
      parameters.setRelativeErrorTol(1e-10);
      parameters.setAbsoluteErrorTol(1e-10);
      gtsam::GaussNewtonOptimizer optimizer(
          impl_->evaluation_factors, impl_->evaluation_initial, parameters);
      const auto solution = optimizer.optimize();
      result.iterations = optimizer.iterations();
      // One full elimination at the solution. No timestamps, marginalization,
      // pruning, capacity eviction or omitted historical factors.
      reference.UpdateConditionalFactors(impl_->evaluation_factors, solution);
      owner = &reference;
    }
    const auto estimate = owner->calculateEstimate();
    const auto key = PoseKey(impl_->nodes.rbegin()->second.key);
    const auto pose = estimate.at<gtsam::Pose3>(key);
    result.map_from_base = FromPose(pose);
    Eigen::MatrixXd tangent;
    const auto valid = PosteriorCovariance(*owner, {{key, 6}}, &tangent);
    if (!valid.ok()) return valid;
    Matrix6d axes = Matrix6d::Zero();
    axes.topRightCorner<3, 3>() = pose.rotation().matrix();
    axes.bottomLeftCorner<3, 3>() = pose.rotation().matrix();
    result.pose_covariance = axes * tangent * axes.transpose();
    const auto& factors = batch ? impl_->evaluation_factors : owner->getFactors();
    for (const auto& factor : factors) {
      if (!factor) continue;
      GraphFactorEvaluation item;
      item.keys.assign(factor->keys().begin(), factor->keys().end());
      const auto* noise = dynamic_cast<const gtsam::NoiseModelFactor*>(factor.get());
      item.type = dynamic_cast<const FrozenMarginalFactor*>(factor.get()) ? "marginal" :
          dynamic_cast<const MotionFactor*>(factor.get()) ? "motion_exact" :
          dynamic_cast<const ErrorChainFactor*>(factor.get()) ? "error_chain" :
          dynamic_cast<const ConditionalObservationFactor*>(factor.get()) ? "observation_conditional" :
          dynamic_cast<const ObservationFactor*>(factor.get()) ? "observation_independent" :
          noise ? "error_prior" : "marginal";
      item.error = factor->error(estimate);
      if (noise) {
        item.residual = noise->unwhitenedError(estimate);
        item.weight = noise->weight(estimate);
        const auto constrained = std::dynamic_pointer_cast<gtsam::noiseModel::Constrained>(
            noise->noiseModel());
        if (constrained) {
          for (Eigen::Index i = 0; i < item.residual.size(); ++i)
            if (constrained->sigmas()[i] == 0)
              result.maximum_exact_residual = std::max(
                  result.maximum_exact_residual, std::abs(item.residual[i]));
        }
      }
      if (!std::isfinite(item.error) || !std::isfinite(item.weight) ||
          !item.residual.allFinite())
        return {Reason::INVALID_INPUT, "Nonfinite evaluation factor"};
      result.objective += item.error;
      result.factors.push_back(std::move(item));
    }
  } catch (const std::exception& error) {
    return {Reason::MATCH_FAILED, std::string("Evaluation: ") + error.what()};
  }
  *output = std::move(result);
  return {};
}

Result FixedLagGraph::ValidateConditionalObservation(
    const LocalState& local, const GlobalObservation& observation) {
  Eigen::MatrixXd loading, noise;
  return ConditionalObservationModel(observation, local, &loading, &noise);
}

Result FixedLagGraph::Initialize(const LocalState& local,
                                 const GlobalObservation& anchor,
                                 const std::vector<LocalState>& following,
                                 const std::vector<MotionIncrement>& motions) {
  Result valid = ValidateConfig();
  if (!valid.ok()) {
    return valid;
  }
  valid = CheckObservation(anchor);
  if (!valid.ok()) {
    return valid;
  }
  if (anchor.kind != GlobalObservation::Kind::POSE || !local.valid ||
      !local.covariance_model_valid || anchor.epoch != local.epoch ||
      local.epoch.generation == 0 || local.epoch.session.empty() ||
      local.stamp.sequence == 0 || local.stamp.clock_id.empty() ||
      anchor.stamp.time != local.stamp.time ||
      anchor.stamp.clock_id != local.stamp.clock_id ||
      !local.covariance.allFinite() ||
      !local.covariance.isApprox(local.covariance.transpose(), 1e-9) ||
      Eigen::LLT<Matrix15d>(local.covariance).info() != Eigen::Success ||
      !Pose(local).matrix().allFinite() ||
      !local.orientation.coeffs().allFinite() ||
      std::abs(local.orientation.norm() - 1.0) > 1e-9) {
    return {
        Reason::GLOBAL_UNAVAILABLE,
        "Graph initialization requires a verified full-pose same-time anchor"};
  }
  if (local.joint_error &&
      local.joint_error->covariance.rows() > config_.max_error_dimension)
    return {Reason::INVALID_INPUT, "Joint error dimension exceeds graph capacity"};
  valid = ValidateJointLocalError(local);
  if (!valid.ok()) return valid;
  if (following.size() != motions.size() ||
      following.size() + 1 > config_.max_states ||
      (!following.empty() &&
       following.back().stamp.time - local.stamp.time > config_.lag))
    return {Reason::HISTORY_UNAVAILABLE, "Bootstrap trajectory exceeds the fixed lag"};
  const auto covariance = JointErrorCovariance(local);
  const auto started = Clock::now();
  std::unique_ptr<Impl> candidate(new Impl(config_.lag));
  candidate->anchor = anchor;
  candidate->nodes.emplace(local.stamp.time, Impl::Node{1, local});
  candidate->next_key = 2;
  candidate->output.last_observation = anchor.stamp.time;
  candidate->output.last_full_observation = anchor.stamp.time;
  candidate->output.last_georeference =
      anchor.georeferenced ? anchor.stamp.time : 0.0;
  candidate->output.georeferenced = anchor.georeferenced;
  candidate->observations[{anchor.source, anchor.stamp.sequence}] =
      anchor.stamp.time;
  candidate->sources.insert(anchor.source);
  gtsam::NonlinearFactorGraph factors;
  gtsam::Values values;
  gtsam::FixedLagSmoother::KeyTimestampMap timestamps{
      {PoseKey(1), local.stamp.time}, {ErrorKey(1), local.stamp.time}};
  values.insert(PoseKey(1), ToPose(anchor.pose));
  values.insert(ErrorKey(1), gtsam::Vector::Zero(covariance.rows()).eval());
  const Eigen::Isometry3d alignment = anchor.pose * Pose(local).inverse();
  for (std::size_t i = 0; i < following.size(); ++i) {
    const auto previous = candidate->nodes.rbegin()->second;
    const auto& next = following[i];
    JointMotionModel model;
    valid = ModelMotion(previous.local, next, motions[i], config_, &model);
    if (!valid.ok()) return valid;
    const uint64_t key = candidate->next_key++;
    candidate->nodes.emplace(next.stamp.time, Impl::Node{key, next});
    factors.emplace_shared<ErrorChainFactor>(
        ErrorKey(previous.key), ErrorKey(key), model.transition,
        model.noise.projection, model.noise.model, model.noise.sigmas);
    factors.emplace_shared<MotionFactor>(previous.key, key, model.verified,
                                         previous.local, next);
    values.insert(PoseKey(key), ToPose(alignment * Pose(next)));
    values.insert(ErrorKey(key),
                   gtsam::Vector::Zero(JointErrorCovariance(next).rows()).eval());
    timestamps[PoseKey(key)] = timestamps[ErrorKey(key)] = next.stamp.time;
  }
  if (anchor.conditional_local_error_qualified) {
    const auto error_node = candidate->nodes.find(
        anchor.local_error_time.value_or(anchor.stamp.time));
    if (error_node == candidate->nodes.end())
      return {Reason::HISTORY_UNAVAILABLE, "Bootstrap has no exact conditioning error node"};
    Eigen::MatrixXd loading, conditional_noise;
    valid = ConditionalObservationModel(anchor, error_node->second.local,
                                         &loading, &conditional_noise);
    if (!valid.ok()) return valid;
    ConditionalNoise noise;
    valid = MakeConditionalNoise(conditional_noise, ObservationNoise(anchor),
                                 config_.robust_threshold, &noise);
    if (!valid.ok()) return valid;
    factors.emplace_shared<ConditionalObservationFactor>(
        1, error_node->second.key, anchor, loading, noise, config_.robust_threshold);
  } else {
    factors.emplace_shared<ObservationFactor>(PoseKey(1), anchor,
                                             config_.robust_threshold);
  }
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector>>(
      ErrorKey(1), gtsam::Vector::Zero(covariance.rows()),
      gtsam::noiseModel::Gaussian::Covariance(covariance));
  try {
    if (anchor.conditional_local_error_qualified && !following.empty())
      candidate->smoother.UpdateConditionalFactors(factors, values, timestamps);
    else
      candidate->smoother.update(factors, values, timestamps);
    valid = candidate->Finalize(config_, started);
  } catch (const gtsam::IndeterminantLinearSystemException& error) {
    return {Reason::DEGENERATE, error.what()};
  }
  if (!valid.ok()) {
    return valid;
  }
  if (retain_evaluation_history_) candidate->Archive(factors, values);
  impl_.swap(candidate);
  return {};
}

Result FixedLagGraph::AddLocal(const LocalState& local,
                               const MotionIncrement& motion) {
  if (!initialized()) {
    return {Reason::GLOBAL_UNAVAILABLE, "Graph has no verified anchor"};
  }
  const auto& previous = impl_->nodes.rbegin()->second;
  JointMotionModel model;
  auto valid = ModelMotion(previous.local, local, motion, config_, &model);
  if (!valid.ok()) return valid;
  const auto& verified = model.verified;
  const auto& transition = model.transition;
  const auto& conditional = model.noise;
  if (impl_->nodes.size() >= config_.max_states &&
      impl_->nodes.begin()->first >= local.stamp.time - config_.lag) {
    return {Reason::INVALID_INPUT, "Fixed-lag state capacity exceeded"};
  }
  const auto started = Clock::now();
  std::unique_ptr<Impl> candidate(new Impl(*impl_));
  const uint64_t key = candidate->next_key++;
  candidate->nodes.emplace(local.stamp.time, Impl::Node{key, local});
  gtsam::NonlinearFactorGraph factors;
  factors.emplace_shared<ErrorChainFactor>(ErrorKey(previous.key),
                                           ErrorKey(key), transition,
                                           conditional.projection,
                                           conditional.model, conditional.sigmas);
  factors.emplace_shared<MotionFactor>(previous.key, key, verified,
                                      previous.local, local);
  gtsam::Values values;
  const auto previous_pose =
      candidate->smoother.calculateEstimate<gtsam::Pose3>(
          PoseKey(previous.key));
  const gtsam::Vector previous_error =
      candidate->smoother.calculateEstimate<gtsam::Vector>(
          ErrorKey(previous.key));
  const gtsam::Vector next_error = transition * previous_error;
  Matrix6x15d first;
  Matrix6x15d second;
  RelativeMotionJacobians(verified.start_pose, verified.delta, &first, &second);
  const Eigen::Matrix<double, 6, 1> relative_error =
      first * JointOutputJacobian(previous.local) * previous_error +
      second * JointOutputJacobian(local) * next_error;
  Eigen::Isometry3d initial_delta = verified.delta;
  initial_delta.translation() += relative_error.head<3>();
  initial_delta.linear() =
      (initial_delta.linear() *
       ExpRotation(relative_error.tail<3>()).toRotationMatrix())
          .eval();
  values.insert(PoseKey(key), previous_pose.compose(ToPose(initial_delta)));
  values.insert(ErrorKey(key), next_error);
  const char* stage = "motion update: ";
  try {
    candidate->smoother.update(
        factors, values,
        {{PoseKey(key), local.stamp.time}, {ErrorKey(key), local.stamp.time}});
    candidate->Prune(local.stamp.time - config_.lag);
    stage = "motion snapshot: ";
    valid = candidate->Finalize(config_, started);
  } catch (const gtsam::IndeterminantLinearSystemException& error) {
    return {Reason::DEGENERATE, std::string(stage) + error.what()};
  }
  if (!valid.ok()) {
    return valid;
  }
  if (retain_evaluation_history_) candidate->Archive(factors, values);
  impl_.swap(candidate);
  return {};
}

Result FixedLagGraph::Observe(const GlobalObservation& observation,
                              ObservationInnovation* diagnostics) {
  if (diagnostics != nullptr) {
    *diagnostics = ObservationInnovation();
    diagnostics->unavailable_reason = "innovation_not_computed";
  }
  if (!initialized()) {
    return {Reason::GLOBAL_UNAVAILABLE, "Graph has no verified anchor"};
  }
  auto valid = CheckObservation(observation);
  if (!valid.ok()) {
    return valid;
  }
  if (observation.epoch != impl_->output.epoch) {
    return {Reason::EPOCH_MISMATCH,
            "Observation ODOM epoch differs from graph"};
  }
  const auto& anchor = impl_->anchor;
  if (observation.map_id != anchor.map_id ||
      observation.map_version != anchor.map_version ||
      observation.map_content_id != anchor.map_content_id ||
      observation.map_frame != anchor.map_frame ||
      observation.base_frame != anchor.base_frame ||
      observation.calibration_id != anchor.calibration_id ||
      observation.stamp.clock_id != anchor.stamp.clock_id) {
    return {Reason::MAP_MISMATCH,
            "Observation graph/map/clock identity mismatch"};
  }
  const auto found = impl_->nodes.find(observation.stamp.time);
  if (found == impl_->nodes.end()) {
    return {Reason::HISTORY_UNAVAILABLE,
            "Observation needs an exact retained graph state"};
  }
  Eigen::MatrixXd loading, conditional_noise;
  auto error_found = found;
  if (observation.conditional_local_error_qualified) {
    error_found = impl_->nodes.find(
        observation.local_error_time.value_or(observation.stamp.time));
    if (error_found == impl_->nodes.end())
      return {Reason::HISTORY_UNAVAILABLE,
              "Conditional observation needs an exact retained error node"};
    valid = ConditionalObservationModel(
        observation, error_found->second.local, &loading, &conditional_noise);
    if (!valid.ok()) return valid;
  }
  const auto identity =
      std::make_pair(observation.source, observation.stamp.sequence);
  const auto retired = impl_->retired_sequences.find(observation.source);
  if (impl_->observations.count(identity) != 0 ||
      (retired != impl_->retired_sequences.end() &&
       observation.stamp.sequence <= retired->second)) {
    return {Reason::DUPLICATE, "Observation identity already used or retired"};
  }
  for (const auto& accepted : impl_->observations) {
    if (accepted.first.first == observation.source &&
        accepted.second == observation.stamp.time) {
      return {Reason::DUPLICATE, "Source measurement time already consumed"};
    }
  }
  if (impl_->observations.size() >= config_.max_observations) {
    return {Reason::INVALID_INPUT, "Fixed-lag observation capacity exceeded"};
  }
  if (impl_->sources.count(observation.source) == 0 &&
      impl_->sources.size() >= 32) {
    return {Reason::INVALID_INPUT, "Fixed-lag source capacity exceeded"};
  }
  const auto started = Clock::now();
  std::unique_ptr<Impl> candidate(new Impl(*impl_));
  ObservationFactor factor(PoseKey(found->second.key), observation,
                           config_.robust_threshold);
  const auto predicted = candidate->smoother.calculateEstimate<gtsam::Pose3>(
      PoseKey(found->second.key));
  if (observation.kind == GlobalObservation::Kind::HEADING) {
    const Eigen::Vector3d direction =
        predicted.rotation().matrix() * observation.direction_in_base;
    if (direction.head<2>().squaredNorm() < 1e-8)
      return {Reason::DEGENERATE, "Graph baseline azimuth is undefined"};
  }
  gtsam::Matrix jacobian;
  gtsam::Vector residual = factor.evaluateError(predicted, &jacobian);
  gtsam::Matrix innovation;
  Eigen::MatrixXd joint;
  try {
    const auto estimate = candidate->smoother.calculateEstimate();
    const auto pose_key = PoseKey(found->second.key);
    std::vector<std::pair<gtsam::Key, Eigen::Index>> query{{pose_key, 6}};
    const auto error_key = ErrorKey(error_found->second.key);
    const Eigen::Index error_dimension = JointOutputJacobian(error_found->second.local).cols();
    if (observation.conditional_local_error_qualified)
      query.emplace_back(error_key, error_dimension);
    const auto marginal = PosteriorCovariance(candidate->smoother, query, &joint);
    if (!marginal.ok()) return marginal;
    innovation = jacobian * joint.topLeftCorner(6, 6) *
        jacobian.transpose() + ObservationNoise(observation);
    if (observation.conditional_local_error_qualified) {
      const Eigen::MatrixXd cross =
          jacobian * joint.topRightCorner(6, error_dimension) * loading.transpose();
      innovation = jacobian * joint.topLeftCorner(6, 6) * jacobian.transpose() +
          cross + cross.transpose() +
          loading * joint.bottomRightCorner(error_dimension, error_dimension) *
              loading.transpose() +
          conditional_noise;
      residual += loading * estimate.at<gtsam::Vector>(error_key);
    }
  } catch (const gtsam::IndeterminantLinearSystemException& error) {
    return {Reason::DEGENERATE, std::string("observation marginal: ") + error.what()};
  }
  Eigen::LLT<Eigen::MatrixXd> whiten(innovation);
  if (whiten.info() != Eigen::Success || !residual.allFinite()) {
    return {Reason::INVALID_INPUT, "Invalid graph innovation"};
  }
  const double distance = residual.dot(whiten.solve(residual));
  if (diagnostics != nullptr) {
    diagnostics->available = true;
    diagnostics->unavailable_reason.clear();
    diagnostics->prior_state_frame = "map_from_base";
    diagnostics->observed_dimension =
        static_cast<uint32_t>(residual.size());
    diagnostics->prior_state_dimension = 6;
    diagnostics->residual = residual;
    diagnostics->covariance = innovation;
    diagnostics->nis = distance;
    diagnostics->gate_threshold = config_.innovation_gate;
    diagnostics->covariance_model =
        observation.conditional_local_error_qualified
            ? "graph_conditional_joint_innovation"
            : "graph_independent_observation_innovation";
    const Eigen::Isometry3d prior_pose = FromPose(predicted);
    diagnostics->prior_pose.translation = prior_pose.translation();
    diagnostics->prior_pose.rotation =
        Eigen::Quaterniond(prior_pose.linear()).normalized();
    diagnostics->prior_covariance = joint.topLeftCorner<6, 6>();
  }
  if (!std::isfinite(distance) || distance > config_.innovation_gate) {
    return {Reason::INNOVATION_REJECTED,
            "Graph observed-subspace innovation rejected"};
  }
  gtsam::NonlinearFactorGraph factors;
  if (observation.conditional_local_error_qualified) {
    ConditionalNoise noise;
    valid = MakeConditionalNoise(conditional_noise, ObservationNoise(observation),
                                 config_.robust_threshold, &noise);
    if (!valid.ok()) return valid;
    factors.emplace_shared<ConditionalObservationFactor>(
        found->second.key, error_found->second.key, observation, loading, noise,
        config_.robust_threshold);
  } else {
    factors.emplace_shared<ObservationFactor>(
        PoseKey(found->second.key), observation, config_.robust_threshold);
  }
  const char* stage = "observation update: ";
  try {
    if (observation.conditional_local_error_qualified)
      candidate->smoother.UpdateConditionalFactors(factors);
    else
      candidate->smoother.update(factors);
    stage = "observation snapshot: ";
    valid = candidate->Finalize(config_, started);
  } catch (const gtsam::IndeterminantLinearSystemException& error) {
    return {Reason::DEGENERATE, std::string(stage) + error.what()};
  }
  if (!valid.ok()) {
    return valid;
  }
  candidate->observations[identity] = observation.stamp.time;
  candidate->sources.insert(observation.source);
  candidate->output.last_observation =
      std::max(candidate->output.last_observation, observation.stamp.time);
  if (observation.kind == GlobalObservation::Kind::POSE) {
    candidate->output.last_full_observation = std::max(
        candidate->output.last_full_observation, observation.stamp.time);
  }
  if (observation.georeferenced) {
    candidate->output.georeferenced = true;
    candidate->output.last_georeference =
        std::max(candidate->output.last_georeference, observation.stamp.time);
  }
  if (retain_evaluation_history_) candidate->Archive(factors, {});
  impl_.swap(candidate);
  return {};
}

}  // namespace unified
}  // namespace localization
}  // namespace openloc
