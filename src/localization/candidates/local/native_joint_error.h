// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "candidates/common/types.h"

namespace openloc::localization::unified {

// First-order stochastic memory of the existing owner. Each row is a physical
// error, each column a shared unit-variance driving variable. Compression
// changes coordinates, never assigns independent noise to a memory clone.
class NativeJointError {
 public:
  explicit NativeJointError(const LocalConfig& config) : config_(config) {}

  bool active() const { return active_; }
  bool structured() const { return config_.enable_native_structured_error; }
  const Result& status() const { return status_; }
  void Reset();
  void Fail(const Result& result);
  void Initialize(const LocalState& state);
  void InitializeStationary(const LocalState& state,
                            const Eigen::Vector3d& mean_acceleration,
                            uint32_t count, bool endpoint_in_mean);
  void InitializeRelative(const LocalState& state,
                          const Eigen::Matrix<double, 21, 21>& covariance);
  void RejectInitialization(const Result& result);
  Eigen::MatrixXd Get(const std::string& key) const;
  JointErrorExpression Error(const std::string& key) const;
  struct LinearTerm {
    Eigen::MatrixXd coefficient;
    std::string key;
  };
  void Linear(const std::string& key, const std::vector<LinearTerm>& terms);
  void Copy(const std::string& key, const std::string& source);
  void CopyInternalToOutput();
  Result StructuredLookup(double time, StructuredLocalState* state) const;
  Result StructuredMotionBetween(double start, double end,
                                MotionIncrement* motion) const;
  NativeStructuredResourceUsage StructuredResourceUsage() const;
  Result CheckAdditionalCapacity(std::size_t rows, std::size_t drivers) const;
  void Set(const std::string& key, const Eigen::MatrixXd& loading);
  void Drop(const std::string& key);
  void Retain(const std::vector<std::string>& keys);
  void Independent(const std::string& key, const Eigen::MatrixXd& covariance,
                   const std::string& source = "");
  void BeginImu(const ImuSample& first, const ImuSample& second);
  Eigen::MatrixXd ImuAt(double time) const;
  void AdvanceImu(const Matrix15d& transition,
                  const Eigen::Matrix<double, 15, 6>& first,
                  const Eigen::Matrix<double, 15, 6>& second,
                  double start, double end);
  Result SpliceCandidate(const NativeJointError& candidate,
                         const std::shared_ptr<const int>& fork,
                         uint64_t scan_sequence);
  std::shared_ptr<const int> PinFork();
  void AdvanceInternal(const Matrix15d& transition, const Matrix15d& noise);
  void AdvanceOutput(const ContinuousOutputErrorTransition& transition);
  void AddSource(const SourceSampleId& source);
  void Publish(const LocalState& state,
               const std::vector<SourceSampleId>& sources);
  Result FinalizeFrontier(const LocalState& state);
  Result Lookup(double time, LocalState* state) const;
  Result MotionBetween(double start, double end, MotionIncrement* motion) const;

  static std::string History(uint64_t sequence, bool propagated = false);
  static std::string Point(uint64_t sequence, std::size_t index);

 private:
  void Compact();
  struct Snapshot {
    LocalState state;
    Eigen::MatrixXd previous_cross;
    std::vector<SourceSampleId> sources;
    JointErrorExpression output_error;
  };
  LocalConfig config_;
  bool active_ = false;
  Result status_{Reason::CORRELATION_UNQUALIFIED,
                 "Native joint stochastic propagation is not enabled"};
  Eigen::Index dimension_ = 0;
  bool frontier_finalized_ = false;
  Eigen::MatrixXd previous_cross_;
  std::map<std::string, Eigen::MatrixXd> memory_;
  std::deque<Snapshot> history_;
  std::vector<SourceSampleId> pending_sources_;
  std::map<std::string, Eigen::MatrixXd> driving_cross_;
  uint64_t anonymous_source_ = 0;
  Stamp first_imu_;
  Stamp second_imu_;
  uint64_t next_fork_ = 0;
  std::map<std::string, std::weak_ptr<const int>> forks_;
  std::map<std::string, JointErrorExpression> structured_memory_;
  struct Primitive {
    std::shared_ptr<const int> identity;
    Eigen::MatrixXd root;
  };
  // Shared by task copies so independently replayed sampled drivers retain
  // their actual common identity.
  struct PrimitiveRegistry {
    std::mutex mutex;
    std::map<std::string, Primitive> entries;
    uint64_t rows = 0;
    uint64_t retired_sources = 0;
    uint64_t retired_rows = 0;
  };
  std::shared_ptr<PrimitiveRegistry> primitives_;
  std::set<std::string> inserted_primitives_;
  uint64_t publish_queries_ = 0;
  double publish_query_ms_ = 0;
};

}  // namespace openloc::localization::unified
