// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

#include "openloc/localization/local/native_types.h"

namespace openloc::replay {

// Export the actual first pre-LIO query support, not an accepted native model.
inline void WriteNativeJointQueryProbe(
    const std::filesystem::path& output,
    const localization::unified::JointLidarInputErrorMap& input) {
  std::ofstream scope(output / "native_query_scope.csv");
  std::ofstream matrices(output / "native_query_matrices.csv");
  std::ofstream points(output / "native_query_points.csv");
  std::ofstream dependencies(output / "native_query_dependencies.csv");
  for (auto* stream : {&scope, &matrices, &points, &dependencies})
    *stream << std::setprecision(17);
  scope << "sequence,reference_time,frontier_time,epoch_generation,"
           "lidar_revision,dimension,points,history,raw_point_variance,"
           "graph_motion_exported,trusted\n"
        << input.deskew.scan.sequence << ',' << input.deskew.scan.time << ','
        << input.deskew.local_frontier.time << ','
        << input.deskew.epoch.generation << ',' << input.deskew.lidar_revision
        << ',' << input.internal_loading.cols() << ','
        << input.deskew.points.size() << ',' << input.history_loadings.size()
        << ',' << input.raw_point_variance << ",0,0\n";
  const auto write_values = [](std::ostream& out, const auto& matrix) {
    for (Eigen::Index row = 0; row < matrix.rows(); ++row)
      for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
        if (row || col) out << ';';
        out << matrix(row, col);
      }
    out << '\n';
  };
  matrices << "kind,index,rows,cols,values\n";
  const auto write_matrix = [&](const char* kind, std::size_t index,
                                const Eigen::MatrixXd& matrix) {
    matrices << kind << ',' << index << ',' << matrix.rows() << ','
             << matrix.cols() << ',';
    write_values(matrices, matrix);
  };
  write_matrix("internal", 0, input.internal_loading);
  write_matrix("output", 0, input.output_loading);
  for (std::size_t i = 0; i < input.history_loadings.size(); ++i)
    write_matrix("history", i, input.history_loadings[i]);
  points << "index,time,x,y,z,raw_point\n";
  dependencies << "point,history,jacobian\n";
  for (std::size_t i = 0; i < input.deskew.points.size(); ++i) {
    const auto& point = input.deskew.points[i];
    points << i << ',' << point.time;
    for (int axis = 0; axis < 3; ++axis)
      points << ',' << point.point_in_end_base[axis];
    points << ',';
    write_values(points, point.raw_point);
    for (const auto& dependency : point.history) {
      dependencies << i << ',' << dependency.history_index << ',';
      write_values(dependencies, dependency.jacobian);
    }
  }
  for (auto* stream : {&scope, &matrices, &points, &dependencies}) {
    stream->flush();
    if (!*stream) throw std::runtime_error("incomplete native query probe");
  }
}

}  // namespace openloc::replay
