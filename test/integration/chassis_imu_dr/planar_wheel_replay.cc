// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0.
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "candidates/local/planar_wheel_dr.h"

namespace {

double Number(const std::string& text) {
  std::size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::runtime_error("invalid finite numeric field: " + text);
  }
  return value;
}

void Run(const std::string& input_path, const std::string& output_path,
         double bias) {
  using openloc::localization::unified::PlanarWheelDr;
  using openloc::localization::unified::PlanarWheelInput;
  using openloc::localization::unified::ReasonName;
  std::ifstream input(input_path);
  std::ofstream output(output_path);
  if (!input || !output) {
    throw std::runtime_error("cannot open replay input or output");
  }
  output.exceptions(std::ios::failbit | std::ios::badbit);
  std::string line;
  if (!std::getline(input, line) ||
      line != "segment,time,vx,vy,gz,seed_yaw,score") {
    throw std::runtime_error("unexpected planar input header");
  }
  PlanarWheelDr candidate("unix", 0.05, bias);
  output << "segment,time,local_x,local_y,yaw,wheel_distance_m\n"
         << std::setprecision(17);
  int segment = -1;
  uint64_t sequence = 0;
  while (std::getline(input, line)) {
    std::stringstream stream(line);
    std::vector<std::string> fields;
    std::string field;
    while (std::getline(stream, field, ',')) {
      fields.push_back(field);
    }
    if (fields.size() != 7) {
      throw std::runtime_error("expected seven planar input fields");
    }
    const double id = Number(fields[0]);
    if (id != segment && id != segment + 1) {
      throw std::runtime_error(
          "segment IDs must start at zero and advance by one");
    }
    if (fields[6] != "0" && fields[6] != "1") {
      throw std::runtime_error("score flag must be zero or one");
    }
    PlanarWheelInput sample;
    sample.stamp.clock_id = "unix";
    sample.stamp.time = Number(fields[1]);
    sample.stamp.receive_time = sample.stamp.time;
    sample.stamp.sequence = ++sequence;
    sample.velocity = {Number(fields[2]), Number(fields[3])};
    sample.gyro_z = Number(fields[4]);
    const bool seed = id != segment;
    if (!seed && !fields[5].empty()) {
      throw std::runtime_error("seed yaw is allowed only at segment start");
    }
    const auto result = seed ? candidate.Seed(sample, Eigen::Vector2d::Zero(),
                                              Number(fields[5]))
                             : candidate.Add(sample);
    if (!result.ok()) {
      throw std::runtime_error(std::string(ReasonName(result.reason)) + ": " +
                               result.message);
    }
    segment = static_cast<int>(id);
    if (fields[6] == "1") {
      const auto& state = candidate.state();
      output << segment << ',' << state.stamp.time << ',' << state.position.x()
             << ',' << state.position.y() << ',' << state.yaw << ','
             << state.distance << '\n';
    }
  }
  if (segment < 0 || !input.eof()) {
    throw std::runtime_error("empty or unreadable planar input");
  }
  output.flush();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 7 || std::string(argv[1]) != "--input" ||
      std::string(argv[3]) != "--output" ||
      std::string(argv[5]) != "--gyro-bias") {
    std::cerr << "usage: planar_wheel_replay --input input.csv "
                 "--output trajectory.csv --gyro-bias rad_per_s\n";
    return 2;
  }
  try {
    Run(argv[2], argv[4], Number(argv[6]));
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "planar_wheel_replay: " << error.what() << '\n';
    return 1;
  }
}
