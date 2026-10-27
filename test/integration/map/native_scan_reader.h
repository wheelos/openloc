// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "candidates/common/types.h"
#include "test/integration/map/replay_csv.h"

namespace openloc::replay {

// Decode at most one future scan. Take is nonblocking; the host alone gates
// consumption by source arrival time. No future point reaches an estimator.
class NativeScanReader {
 public:
  using Point = localization::unified::TimedLidarPoint;
  using Clock = std::chrono::steady_clock;
  struct Decoded {
    std::vector<Point> points;
    double decoding_ms = 0.0;
  };

  NativeScanReader(std::filesystem::path root, std::vector<std::string> names,
                   std::size_t max_points, const std::filesystem::path& output)
      : root_(std::move(root)),
        names_(std::move(names)),
        max_points_(max_points),
        journal_(output / "lidar_decode.csv") {
    if (names_.empty() || max_points == 0 || !journal_)
      throw std::runtime_error("invalid bounded scan-reader input or journal");
    journal_ << "sequence,frontier_ns,status,points,decoding_ms\n";
    Start(0);
  }

  std::optional<Decoded> Take(std::size_t index, int64_t frontier_ns) {
    if (index >= names_.size() || index < next_)
      throw std::runtime_error("scan-reader index regressed or exceeded input");
    if (future_.wait_for(std::chrono::seconds(0)) !=
        std::future_status::ready) {
      Record(index, frontier_ns, "not_ready", 0, 0);
      return std::nullopt;
    }
    Decoded decoded = future_.get();
    if (index != next_) {
      Record(next_, frontier_ns, "skipped_before_local_ready",
             decoded.points.size(), decoded.decoding_ms);
      Start(index);
      return std::nullopt;
    }
    Record(index, frontier_ns, "consumed", decoded.points.size(),
           decoded.decoding_ms);
    ++next_;
    if (next_ < names_.size()) Start(next_);
    return decoded;
  }

  void Finish() {
    if (future_.valid()) {
      const auto unused = future_.get();
      Record(next_, 0, "decoded_not_consumed", unused.points.size(),
             unused.decoding_ms);
    }
    journal_.flush();
    if (!journal_)
      throw std::runtime_error("incomplete point decoding journal");
  }

 private:
  friend class NativeScanReaderTestAccess;

  void Start(std::size_t index) {
    next_ = index;
    const auto path = root_ / names_[index];
    future_ = std::async(std::launch::async, [path, limit = max_points_] {
      const auto started = Clock::now();
      std::ifstream input(path);
      std::string line;
      if (!input || !ReadLine(input, &line) || line != "timestamp_ns,x,y,z")
        throw std::runtime_error("unreadable point input/header: " +
                                 path.string());
      Decoded decoded;
      while (ReadLine(input, &line)) {
        const auto cells = Fields(line);
        if (cells.size() != 4 || line.back() == ',')
          throw std::runtime_error("invalid point input fields: " +
                                   path.string());
        if (decoded.points.size() >= limit)
          throw std::runtime_error(
              "point input exceeds configured point bound");
        decoded.points.push_back(
            {{Number(cells[1]), Number(cells[2]), Number(cells[3])},
             static_cast<double>(Time(cells[0])) / 1e9});
      }
      if (!input.eof() || decoded.points.empty())
        throw std::runtime_error("incomplete or empty raw point input");
      decoded.decoding_ms =
          std::chrono::duration<double, std::milli>(Clock::now() - started)
              .count();
      return decoded;
    });
  }

  void Record(std::size_t index, int64_t frontier, const char* status,
              std::size_t points, double elapsed) {
    journal_ << index + 1 << ',' << frontier << ',' << status << ',' << points
             << ',' << elapsed << '\n';
    journal_.flush();
    if (!journal_)
      throw std::runtime_error("incomplete point decoding journal");
  }

  std::filesystem::path root_;
  std::vector<std::string> names_;
  const std::size_t max_points_;
  std::ofstream journal_;
  std::size_t next_ = 0;
  std::future<Decoded> future_;
};

}  // namespace openloc::replay
