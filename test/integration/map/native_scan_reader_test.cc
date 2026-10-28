// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "test/integration/map/native_scan_reader.h"

#include "gtest/gtest.h"

namespace openloc::replay {

class NativeScanReaderTestAccess {
 public:
  static void Wait(NativeScanReader* reader) { reader->future_.wait(); }
};

namespace {

TEST(ReplayCsvTest, SplitPreservesStreamSemanticsForEmptyAndMalformedFields) {
  const std::string alphabet = ",;|\r\n\"x ";
  const auto compare = [](const std::string& line) {
    for (char delimiter : {',', ';', '|', '\r'}) {
      std::istringstream input(line);
      std::vector<std::string> expected;
      std::string field;
      while (std::getline(input, field, delimiter)) {
        if (!field.empty() && field.back() == '\r') field.pop_back();
        expected.push_back(field);
      }
      EXPECT_EQ(expected, Fields(line, delimiter));
    }
  };
  compare("");
  std::vector<std::string> previous{""};
  for (int length = 1; length <= 4; ++length) {
    std::vector<std::string> next;
    for (const auto& prefix : previous) {
      for (char c : alphabet) {
        next.push_back(prefix + c);
        compare(next.back());
      }
    }
    previous = std::move(next);
  }
  compare("1791882562723170042,1.2345678901234567,-2.5e-10,0\r");
  compare(std::string(4096, 'x') + ",");
}

TEST(NativeScanReaderTest, DecodesBoundedPointsAndAdvancesOnePrefetchSlot) {
  const std::filesystem::path root = ::testing::TempDir();
  for (const auto* name : {"scan_a.csv", "scan_b.csv"}) {
    std::ofstream stream(root / name);
    stream << "timestamp_ns,x,y,z\n1000000000,1.5,-2,3\n";
  }
  NativeScanReader reader(root, {"scan_a.csv", "scan_b.csv"}, 1, root);
  NativeScanReaderTestAccess::Wait(&reader);
  const auto first = reader.Take(0, 1000000000);
  ASSERT_TRUE(first.has_value());
  ASSERT_EQ(first->points.size(), 1U);
  EXPECT_DOUBLE_EQ(first->points[0].time, 1.0);
  EXPECT_TRUE(first->points[0].position.isApprox(Eigen::Vector3d(1.5, -2, 3)));
  NativeScanReaderTestAccess::Wait(&reader);
  ASSERT_TRUE(reader.Take(1, 1100000000).has_value());
  reader.Finish();
  EXPECT_THROW(reader.Take(0, 1200000000), std::runtime_error);
}

TEST(NativeScanReaderTest, PointOverflowAndInvalidInputPropagateFromWorker) {
  const std::filesystem::path root = ::testing::TempDir();
  const std::vector<std::string> inputs{
      "timestamp_ns,x,y,z\n1000000000,1,2,3\n1000000001,1,2,3\n",
      "wrong_header\n", "timestamp_ns,x,y,z\n1000000000,nan,2,3\n",
      "timestamp_ns,x,y,z\n1000000000,1,2,3,\n", "timestamp_ns,x,y,z\n"};
  for (const auto& content : inputs) {
    {
      std::ofstream stream(root / "invalid.csv");
      stream << content;
    }
    NativeScanReader reader(root, {"invalid.csv"}, 1, root);
    NativeScanReaderTestAccess::Wait(&reader);
    EXPECT_THROW(reader.Finish(), std::exception);
  }
}

}  // namespace
}  // namespace openloc::replay
