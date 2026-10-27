// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"

namespace {

using Matrix4d = Eigen::Matrix4d;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

struct Reference {
  std::string timestamp;
  Matrix4d world_from_lidar;
};

Matrix4d ReadTransform(std::istream& input) {
  Matrix4d transform = Matrix4d::Identity();
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 4; ++column) {
      if (!(input >> transform(row, column)) ||
          !std::isfinite(transform(row, column))) {
        throw std::runtime_error("invalid 3x4 transform");
      }
    }
  }
  return transform;
}

Matrix4d ReadCalibration(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open calib.txt");
  std::string line;
  while (std::getline(input, line)) {
    const auto separator = line.find(':');
    if (separator == std::string::npos) continue;
    if (line.substr(0, separator) != "Tr") continue;
    std::istringstream values(line.substr(separator + 1));
    Matrix4d result = Matrix4d::Identity();
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 4; ++column) {
        if (!(values >> result(row, column)) ||
            !std::isfinite(result(row, column))) {
          throw std::runtime_error("invalid Velodyne-to-camera calibration");
        }
      }
    }
    return result;
  }
  throw std::runtime_error("calib.txt has no Tr transform");
}

std::vector<Reference> ReadReferences(const std::filesystem::path& sequence,
                                     const Matrix4d& camera_from_lidar) {
  std::ifstream poses(sequence / "poses.txt");
  std::ifstream times(sequence / "times.txt");
  if (!poses || !times) throw std::runtime_error("missing poses.txt or times.txt");
  std::vector<Reference> references;
  std::string pose_line;
  std::string time_line;
  while (std::getline(poses, pose_line)) {
    if (!std::getline(times, time_line)) {
      throw std::runtime_error("poses.txt has more rows than times.txt");
    }
    std::istringstream pose_values(pose_line);
    references.push_back(
        {time_line, ReadTransform(pose_values) * camera_from_lidar});
    std::string extra;
    if (pose_values >> extra) throw std::runtime_error("extra pose values");
    try {
      std::size_t consumed = 0;
      const double timestamp = std::stod(time_line, &consumed);
      if (consumed != time_line.size() || !std::isfinite(timestamp)) {
        throw std::runtime_error("invalid timestamp");
      }
    } catch (const std::exception&) {
      throw std::runtime_error("invalid timestamp in times.txt");
    }
  }
  if (std::getline(times, time_line)) {
    throw std::runtime_error("times.txt has more rows than poses.txt");
  }
  if (references.empty()) throw std::runtime_error("empty reference trajectory");
  return references;
}

Cloud::Ptr ReadScan(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input) throw std::runtime_error("cannot open scan: " + path.string());
  const auto bytes = input.tellg();
  constexpr std::streamoff kPointBytes = 4 * sizeof(float);
  if (bytes <= 0 || bytes % kPointBytes != 0) {
    throw std::runtime_error("invalid KITTI scan byte count: " + path.string());
  }
  input.seekg(0);
  Cloud::Ptr cloud(new Cloud);
  const auto count = static_cast<std::size_t>(bytes / kPointBytes);
  cloud->reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    float point[4];
    input.read(reinterpret_cast<char*>(point), sizeof(point));
    if (!input) throw std::runtime_error("truncated KITTI scan");
    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
        !std::isfinite(point[2]) ||
        (point[0] == 0.0F && point[1] == 0.0F && point[2] == 0.0F)) {
      continue;
    }
    cloud->push_back(pcl::PointXYZ(point[0], point[1], point[2]));
  }
  if (cloud->empty()) throw std::runtime_error("scan has no valid points");
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

Cloud::Ptr Downsample(const Cloud::ConstPtr& input) {
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  filter.setLeafSize(0.5F, 0.5F, 0.5F);
  Cloud::Ptr output(new Cloud);
  filter.filter(*output);
  if (output->empty()) throw std::runtime_error("voxel filter removed all points");
  return output;
}

void WritePose(std::ostream& output, const Matrix4d& pose) {
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 4; ++column) {
      output << ',' << pose(row, column);
    }
  }
}

std::filesystem::path ScanPath(const std::filesystem::path& sequence,
                               int frame) {
  std::ostringstream name;
  name << std::setw(6) << std::setfill('0') << frame << ".bin";
  return sequence / "velodyne" / name.str();
}

void Run(const std::filesystem::path& sequence, int first_frame,
         int frame_count, const std::filesystem::path& output_path) {
  if (first_frame < 0 || frame_count < 2) {
    throw std::runtime_error("require first_frame >= 0 and frame_count >= 2");
  }
  const Matrix4d camera_from_lidar = ReadCalibration(sequence / "calib.txt");
  const std::vector<Reference> references =
      ReadReferences(sequence, camera_from_lidar);
  if (static_cast<std::size_t>(first_frame + frame_count) > references.size()) {
    throw std::runtime_error("requested range exceeds reference trajectory");
  }
  for (int frame = first_frame; frame < first_frame + frame_count; ++frame) {
    if (!std::filesystem::is_regular_file(ScanPath(sequence, frame))) {
      throw std::runtime_error("missing Velodyne frame " +
                               std::to_string(frame));
    }
  }

  std::ofstream output(output_path);
  if (!output) throw std::runtime_error("cannot create output CSV");
  output << std::setprecision(17);
  output << "frame,timestamp_s,status,fitness,valid_points,filtered_points";
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 4; ++column) {
      output << ",estimate_" << row << column;
    }
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 4; ++column) {
      output << ",reference_" << row << column;
    }
  }
  output << '\n';

  const Matrix4d reference_origin =
      references[static_cast<std::size_t>(first_frame)].world_from_lidar;
  Matrix4d estimate = Matrix4d::Identity();
  Cloud::Ptr previous;
  for (int frame = first_frame; frame < first_frame + frame_count; ++frame) {
    const Cloud::Ptr raw = ReadScan(ScanPath(sequence, frame));
    const Cloud::Ptr current = Downsample(raw);
    const Matrix4d reference =
        reference_origin.inverse() *
        references[static_cast<std::size_t>(frame)].world_from_lidar;
    output << frame << ',' << references[static_cast<std::size_t>(frame)].timestamp
           << ',';
    if (!previous) {
      output << "ANCHOR,0," << raw->size() << ',' << current->size();
      WritePose(output, estimate);
      WritePose(output, reference);
      output << '\n';
      previous = current;
      continue;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(current);
    icp.setInputTarget(previous);
    icp.setMaximumIterations(40);
    icp.setMaxCorrespondenceDistance(2.0);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-5);
    Cloud aligned;
    icp.align(aligned);
    const Eigen::Matrix4f previous_from_current = icp.getFinalTransformation();
    if (!icp.hasConverged() || !previous_from_current.allFinite()) {
      output << "ICP_REJECTED," << icp.getFitnessScore() << ',' << raw->size()
             << ',' << current->size();
      for (int index = 0; index < 24; ++index) output << ',';
      output << '\n';
      output.flush();
      return;
    }
    estimate = estimate * previous_from_current.cast<double>();
    output << "ICP_CONVERGED," << icp.getFitnessScore() << ',' << raw->size()
           << ',' << current->size();
    WritePose(output, estimate);
    WritePose(output, reference);
    output << '\n';
    previous = current;
  }
  if (!output) throw std::runtime_error("failed writing odometry CSV");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "usage: kitti_lidar_odometry <sequence_dir> <first_frame> "
                 "<frame_count> <output.csv>\n";
    return 2;
  }
  try {
    Run(argv[1], std::stoi(argv[2]), std::stoi(argv[3]), argv[4]);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  return 0;
}
