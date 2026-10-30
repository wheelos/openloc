// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include "candidates/common/types.h"

namespace openloc::localization::unified {

// Closed, caller-owned query frontier. Every future shared dependency, including
// delayed-task pins, must remain a named row until its last use. No unmanaged
// expression handles or replay of retired primitive identities are supported.
class BoundedJointError {
 public:
  struct Term {
    Eigen::MatrixXd coefficient;
    std::string key;
  };
  explicit BoundedJointError(Eigen::Index maximum_rows)
      : maximum_rows_(maximum_rows) {}
  Result Independent(const std::string& key, const Eigen::MatrixXd& root);
  Result Linear(const std::string& key, const std::vector<Term>& terms);
  Result Retain(const std::vector<std::string>& keys);
  Result Compact();
  Result CrossCovariance(const std::string& first, const std::string& second,
                         Eigen::MatrixXd* covariance) const;
  Eigen::Index rows() const;
  Eigen::Index dimension() const { return dimension_; }

 private:
  Eigen::Index maximum_rows_;
  Eigen::Index dimension_ = 0;
  std::map<std::string, Eigen::MatrixXd> memory_;
};

}  // namespace openloc::localization::unified
