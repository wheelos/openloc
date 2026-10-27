// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <filesystem>
#include <limits>

#include "gnss_quality_gate.h"
#include "openloc/localization/global/diagnostic_alignment.h"
#include "test/integration/map/replay_csv.h"

namespace openloc::replay {

// Receiver admission only; the caller submits to its existing global owner.
class NativeGnssDiagnostic {
 public:
  NativeGnssDiagnostic(const std::filesystem::path& bundle,
                       const std::filesystem::path& output)
      : events_(ReadRows((bundle / "events.csv").string(),
                         "timestamp_ns,receive_ns,sequence,x,y,quality,"
                         "satellites,hdop,age,adapter_reason",
                         10)),
        journal_(output / "gnss.csv") {
    const auto rows = ReadRows(
        (bundle / "config.csv").string(),
        "sigma_m,lever_x,lever_y,lever_z,min_satellites,max_hdop,max_age_s,"
        "dwell_ns,max_gap_ns,gnss_map_construction",
        10);
    if (rows.size() != 1) throw std::runtime_error("one GNSS policy required");
    const auto& row = rows[0];
    sigma_ = Number(row[0]);
    if (sigma_ <= 0) throw std::runtime_error("invalid diagnostic GNSS sigma");
    for (int i = 0; i < 3; ++i) lever_[i] = Number(row[i + 1]);
    if (row[9] != "unknown" && row[9] != "present")
      throw std::runtime_error(
          "GNSS map dependency must be unknown or present");
    dependency_ = row[9] == "present"
                      ? localization::ObservationDependency::kPresent
                      : localization::ObservationDependency::kUnknown;
    localization::internal::GnssQualityGatePolicy policy{
        Count(row[4]), Number(row[5]),     Number(row[6]), 1.0, 1.0,
        0.0,           std::stoll(row[7]), Time(row[8])};
    // GGA supplies no receiver accuracy; no diagnostic sigma is substituted.
    policy.allow_missing_horizontal_accuracy_diagnostic = true;
    const auto configured = gate_.Configure(policy);
    if (!configured.ok()) throw std::runtime_error(configured.message());
    int64_t receive = 0;
    for (const auto& event : events_) {
      if (Time(event[1]) < receive)
        throw std::runtime_error("GNSS arrival order regressed");
      receive = Time(event[1]);
    }
    if (!journal_) throw std::runtime_error("cannot open GNSS journal");
    journal_ << std::setprecision(17)
             << "sequence,timestamp_ns,receive_ns,processing_receive_ns,"
                "quality,gate_outcome,gate_reason,adapter_reason,owner_status,"
                "owner_reason,x,y,sigma_m,receiver_accuracy_available,"
                "heading_available,lever_x,lever_y,lever_z,"
                "gnss_map_construction,target_graph_available\n";
  }

  template <class Submit>
  void Drain(int64_t receive, Submit submit, bool odometry_available = true) {
    namespace loc = localization;
    while (next_ < events_.size() && Time(events_[next_][1]) <= receive) {
      const auto& row = events_[next_++];
      loc::GlobalHorizontalObservation observation;
      auto& context = observation.context;
      context.measurement_time = {std::stoll(row[0]),
                                  loc::ClockDomain::kUnixEpoch};
      context.receive_time = {Time(row[1]), loc::ClockDomain::kUnixEpoch};
      context.sequence = Time(row[2]);
      context.source = "raw-gga-horizontal-diagnostic";
      context.provenance.preparation_id = "native-gnss-diagnostic-v1";
      context.provenance.odometry_initialization =
          loc::ObservationDependency::kAbsent;
      context.provenance.odometry_deskew = loc::ObservationDependency::kAbsent;
      context.provenance.shared_local_scans =
          loc::ObservationDependency::kAbsent;
      context.provenance.gnss_map_construction = dependency_;
      observation.point_in_base = lever_;
      observation.covariance = Eigen::Matrix2d::Identity() * sigma_ * sigma_;
      observation.covariance_source = "explicit-uncalibrated-diagnostic-sigma";
      loc::GnssReceiverQuality receiver;
      receiver.carrier_solution =
          row[5] == "4"   ? loc::GnssCarrierSolution::kFixed
          : row[5] == "5" ? loc::GnssCarrierSolution::kFloat
                          : loc::GnssCarrierSolution::kInvalid;
      receiver.satellite_count = Count(row[6]);
      receiver.hdop = OptionalNumber(row[7]);
      receiver.differential_age_seconds = OptionalNumber(row[8]);
      receiver.horizontal_accuracy_m = std::numeric_limits<double>::quiet_NaN();
      const auto decision =
          gate_.CheckPosition(context.measurement_time, receiver);
      std::string owner_status = "not_submitted",
                  owner_reason = "gate_not_accepted";
      if (row[9] != "ok") {
        gate_.Reset();
        owner_reason = "adapter_rejected";
      } else if (decision.outcome ==
                 loc::internal::GnssQualityOutcome::kAccepted) {
        if (!odometry_available) {
          owner_reason = "native_odometry_unavailable";
        } else {
          observation.position = {Number(row[3]), Number(row[4])};
          const auto status = submit(observation);
          owner_status = std::to_string(static_cast<int>(status.code()));
          owner_reason = status.message();
        }
      }
      journal_ << row[2] << ',' << row[0] << ',' << row[1] << ',' << receive
               << ',' << row[5] << ',' << Outcome(decision.outcome) << ',';
      Text(journal_, decision.status.message());
      journal_ << ',';
      Text(journal_, row[9]);
      journal_ << ',' << owner_status << ',';
      Text(journal_, owner_reason);
      journal_ << ',' << row[3] << ',' << row[4] << ',' << sigma_ << ",0,0,"
               << lever_.x() << ',' << lever_.y() << ',' << lever_.z() << ','
               << (dependency_ == loc::ObservationDependency::kPresent
                       ? "present"
                       : "unknown")
               << ",0\n";
    }
  }

  void Finish() {
    journal_.flush();
    if (next_ != events_.size() || !journal_)
      throw std::runtime_error("GNSS event journal incomplete");
  }

  localization::ObservationDependency dependency() const { return dependency_; }

 private:
  static uint32_t Count(const std::string& text) {
    std::size_t used = 0;
    const auto value = std::stoull(text, &used);
    if (used != text.size() || value > UINT32_MAX || text.front() == '-')
      throw std::runtime_error("GNSS count outside uint32 range");
    return static_cast<uint32_t>(value);
  }

  static const char* Outcome(localization::internal::GnssQualityOutcome value) {
    using O = localization::internal::GnssQualityOutcome;
    switch (value) {
      case O::kAccepted:
        return "accepted";
      case O::kWaitingForStability:
        return "waiting_for_stability";
      case O::kRejected:
        return "rejected";
      case O::kInvalid:
        return "invalid";
    }
    throw std::runtime_error("unknown GNSS gate outcome");
  }

  static double OptionalNumber(const std::string& value) {
    if (value.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::size_t used = 0;
    const double result = std::stod(value, &used);
    if (used != value.size()) throw std::runtime_error("invalid GNSS scalar");
    return result;
  }
  std::vector<std::vector<std::string>> events_;
  std::size_t next_ = 0;
  double sigma_ = 0;
  Eigen::Vector3d lever_ = Eigen::Vector3d::Zero();
  localization::ObservationDependency dependency_;
  localization::internal::GnssQualityGate gate_;
  std::ofstream journal_;
};
}  // namespace openloc::replay
