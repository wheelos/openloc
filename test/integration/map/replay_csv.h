// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"

namespace openloc {
namespace replay {

inline bool ReadLine(std::istream& input, std::string* line) {
  if (!std::getline(input, *line)) return false;
  if (!line->empty() && line->back() == '\r') line->pop_back();
  return true;
}

inline std::vector<std::string> Fields(const std::string& line,
                                       char delimiter = ',') {
  std::vector<std::string> result;
  std::size_t begin = 0;
  while (begin < line.size()) {
    const auto end = line.find(delimiter, begin);
    auto field = line.substr(begin, end == std::string::npos
                                        ? std::string::npos : end - begin);
    if (!field.empty() && field.back() == '\r') field.pop_back();
    result.push_back(std::move(field));
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  return result;
}

inline double Number(const std::string& field) {
  std::size_t used = 0;
  const double value = std::stod(field, &used);
  if (used != field.size() || !std::isfinite(value))
    throw std::runtime_error("invalid finite scalar");
  return value;
}

inline int64_t Time(const std::string& field) {
  std::size_t used = 0;
  const int64_t value = std::stoll(field, &used);
  if (used != field.size() || value <= 0)
    throw std::runtime_error("invalid positive integer");
  return value;
}

inline void Text(std::ostream& output, const std::string& value) {
  output << '"';
  for (char c : value) {
    if (c == '"') output << '"';
    output << c;
  }
  output << '"';
}

inline void Matrix(std::ostream& output, const Eigen::MatrixXd& matrix) {
  std::ostringstream values;
  values << std::setprecision(17);
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    if (row) values << '|';
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      if (col) values << ';';
      values << matrix(row, col);
    }
  }
  Text(output, values.str());
}

inline Eigen::MatrixXd ReadMatrix(const std::string& text, int columns) {
  const auto rows = Fields(text, '|');
  if (rows.empty()) throw std::runtime_error("empty matrix");
  Eigen::MatrixXd matrix(rows.size(), columns);
  for (std::size_t row = 0; row < rows.size(); ++row) {
    const auto cells = Fields(rows[row], ';');
    if (cells.size() != static_cast<std::size_t>(columns))
      throw std::runtime_error("invalid matrix columns");
    for (int col = 0; col < columns; ++col)
      matrix(row, col) = Number(cells[col]);
  }
  return matrix;
}

inline Eigen::MatrixXd SquareMatrix(const std::string& text, int size) {
  const auto matrix = ReadMatrix(text, size);
  if (matrix.rows() != size) throw std::runtime_error("invalid matrix rows");
  return matrix;
}

inline std::vector<std::vector<std::string>> ReadRows(
    const std::string& path, const std::string& header, std::size_t count,
    const std::string& optional_suffix = "") {
  std::ifstream input(path);
  std::string line;
  if (!input || !ReadLine(input, &line))
    throw std::runtime_error("unreadable input/header: " + path);
  if (!optional_suffix.empty() && line == header + "," + optional_suffix) {
    count += Fields(optional_suffix).size();
  } else if (line != header) {
    throw std::runtime_error("invalid input/header: " + path);
  }
  std::vector<std::vector<std::string>> rows;
  while (ReadLine(input, &line)) {
    auto fields = Fields(line);
    if (fields.size() != count || (!line.empty() && line.back() == ','))
      throw std::runtime_error("invalid field count: " + path);
    for (const auto& field : fields) {
      if (field.find('"') != std::string::npos)
        throw std::runtime_error("quoted input fields are unsupported: " +
                                 path);
    }
    rows.push_back(std::move(fields));
  }
  if (!input.eof()) throw std::runtime_error("incomplete input: " + path);
  return rows;
}

}  // namespace replay
}  // namespace openloc
