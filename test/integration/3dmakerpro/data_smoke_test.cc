#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace {

namespace fs = std::filesystem;

struct ManifestFrame {
  std::string file;
  uint64_t points = 0;
  int64_t timebase_ns = 0;
};

struct PcdHeader {
  uint64_t points = 0;
  uint64_t record_size = 0;
  uint64_t data_offset = 0;
  std::vector<std::string> fields;
  std::vector<uint64_t> sizes;
  std::vector<char> types;
  std::vector<uint64_t> counts;
};

struct SmokeSummary {
  std::size_t manifest_frames = 0;
  std::size_t decoded_frames = 0;
  uint64_t map_points = 0;
  std::string frame_id;
  std::string lidar_id;
};

bool ParseCsvRow(const std::string& line, std::vector<std::string>* cells,
                 std::string* error) {
  cells->clear();
  std::string cell;
  bool quoted = false;
  for (std::size_t index = 0; index < line.size(); ++index) {
    const char value = line[index];
    if (quoted) {
      if (value == '"' && index + 1 < line.size() && line[index + 1] == '"') {
        cell.push_back('"');
        ++index;
      } else if (value == '"') {
        quoted = false;
      } else {
        cell.push_back(value);
      }
    } else if (value == ',') {
      cells->push_back(cell);
      cell.clear();
    } else if (value == '"' && cell.empty()) {
      quoted = true;
    } else {
      cell.push_back(value);
    }
  }
  if (quoted) {
    *error = "unterminated quoted CSV field";
    return false;
  }
  cells->push_back(cell);
  return true;
}

bool ParseUnsigned(const std::string& text, uint64_t* value) {
  if (text.empty() ||
      !std::all_of(text.begin(), text.end(),
                   [](unsigned char c) { return std::isdigit(c) != 0; })) {
    return false;
  }
  try {
    std::size_t consumed = 0;
    const unsigned long long parsed = std::stoull(text, &consumed);
    if (consumed != text.size()) {
      return false;
    }
    *value = static_cast<uint64_t>(parsed);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseSigned(const std::string& text, int64_t* value) {
  if (text.empty()) {
    return false;
  }
  try {
    std::size_t consumed = 0;
    const long long parsed = std::stoll(text, &consumed);
    if (consumed != text.size()) {
      return false;
    }
    *value = static_cast<int64_t>(parsed);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseFiniteDouble(const std::string& text) {
  if (text.empty()) {
    return false;
  }
  try {
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    return consumed == text.size() && std::isfinite(value);
  } catch (const std::exception&) {
    return false;
  }
}

bool IsPathWithin(const fs::path& root, const fs::path& path) {
  auto root_it = root.begin();
  auto path_it = path.begin();
  for (; root_it != root.end() && path_it != path.end(); ++root_it, ++path_it) {
    if (*root_it != *path_it) {
      return false;
    }
  }
  return root_it == root.end();
}

bool ParseManifest(const fs::path& root, std::vector<ManifestFrame>* frames,
                   std::string* frame_id, std::string* lidar_id,
                   std::string* error) {
  const fs::path manifest_path = root / "pcd" / "frames.csv";
  std::ifstream input(manifest_path);
  if (!input) {
    *error = "cannot open manifest: " + manifest_path.string();
    return false;
  }
  std::string line;
  if (!std::getline(input, line)) {
    *error = "manifest is empty: " + manifest_path.string();
    return false;
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  std::vector<std::string> columns;
  if (!ParseCsvRow(line, &columns, error)) {
    *error = manifest_path.string() + ": " + *error;
    return false;
  }
  if (!columns.empty() && columns.front().rfind("\xef\xbb\xbf", 0) == 0) {
    columns.front().erase(0, 3);
  }
  std::map<std::string, std::size_t> column_index;
  for (std::size_t index = 0; index < columns.size(); ++index) {
    if (!column_index.emplace(columns[index], index).second) {
      *error = manifest_path.string() + ": duplicate column " + columns[index];
      return false;
    }
  }
  const std::array<const char*, 8> required = {
      "index",       "file",   "seq",      "stamp_sec",
      "timebase_ns", "points", "frame_id", "lidar_id"};
  for (const char* name : required) {
    if (column_index.find(name) == column_index.end()) {
      *error = manifest_path.string() + ": missing column " + name;
      return false;
    }
  }

  frames->clear();
  std::string line_error;
  int64_t previous_timebase_ns = std::numeric_limits<int64_t>::min();
  std::size_t row_number = 1;
  std::string current_frame_id;
  std::string current_lidar_id;
  std::error_code filesystem_error;
  const fs::path canonical_root = fs::canonical(root, filesystem_error);
  if (filesystem_error) {
    *error = "cannot resolve recording root: " + root.string();
    return false;
  }
  const fs::path canonical_pcd_root =
      fs::canonical(root / "pcd", filesystem_error);
  if (filesystem_error || canonical_pcd_root == canonical_root ||
      !IsPathWithin(canonical_root, canonical_pcd_root)) {
    *error = "PCD directory is not beneath recording root: " +
             (root / "pcd").string();
    return false;
  }
  while (std::getline(input, line)) {
    ++row_number;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    std::vector<std::string> cells;
    if (!ParseCsvRow(line, &cells, &line_error) ||
        cells.size() != columns.size()) {
      *error = manifest_path.string() + ":" + std::to_string(row_number) +
               ": malformed CSV row";
      if (!line_error.empty()) {
        *error += " (" + line_error + ")";
      }
      return false;
    }
    const auto& value =
        [&cells, &column_index](const char* name) -> const std::string& {
      return cells[column_index.at(name)];
    };
    uint64_t ignored = 0;
    uint64_t point_count = 0;
    int64_t timebase_ns = 0;
    if (!ParseUnsigned(value("index"), &ignored) ||
        !ParseUnsigned(value("seq"), &ignored) ||
        !ParseFiniteDouble(value("stamp_sec")) ||
        !ParseSigned(value("timebase_ns"), &timebase_ns) ||
        !ParseUnsigned(value("points"), &point_count) || point_count == 0 ||
        value("file").empty() || value("frame_id").empty() ||
        value("lidar_id").empty()) {
      *error = manifest_path.string() + ":" + std::to_string(row_number) +
               ": invalid required field";
      return false;
    }
    if (!frames->empty() && timebase_ns <= previous_timebase_ns) {
      *error = manifest_path.string() + ":" + std::to_string(row_number) +
               ": timebase_ns is not strictly increasing";
      return false;
    }
    const fs::path relative_file(value("file"));
    if (relative_file.is_absolute()) {
      *error = manifest_path.string() + ":" + std::to_string(row_number) +
               ": file path must be relative";
      return false;
    }
    for (const auto& part : relative_file) {
      if (part == ".." || part == ".") {
        *error = manifest_path.string() + ":" + std::to_string(row_number) +
                 ": file path contains traversal";
        return false;
      }
    }
    const fs::path pcd_path = root / "pcd" / relative_file;
    if (!fs::is_regular_file(pcd_path, filesystem_error)) {
      *error = manifest_path.string() + ":" + std::to_string(row_number) +
               ": PCD file does not exist: " + pcd_path.string();
      return false;
    }
    const fs::path canonical_pcd = fs::canonical(pcd_path, filesystem_error);
    if (filesystem_error) {
      *error = manifest_path.string() + ":" + std::to_string(row_number) +
               ": cannot resolve PCD file: " + pcd_path.string();
      return false;
    }
    if (!IsPathWithin(canonical_pcd_root, canonical_pcd)) {
      *error = manifest_path.string() + ":" + std::to_string(row_number) +
               ": PCD symlink escapes the recording directory";
      return false;
    }
    if (frames->empty()) {
      current_frame_id = value("frame_id");
      current_lidar_id = value("lidar_id");
    } else if (value("frame_id") != current_frame_id ||
               value("lidar_id") != current_lidar_id) {
      *error = manifest_path.string() + ":" + std::to_string(row_number) +
               ": frame_id or lidar_id changed within the session";
      return false;
    }
    frames->push_back(
        {relative_file.generic_string(), point_count, timebase_ns});
    previous_timebase_ns = timebase_ns;
  }
  if (frames->empty()) {
    *error = manifest_path.string() + ": no frame records";
    return false;
  }
  *frame_id = std::move(current_frame_id);
  *lidar_id = std::move(current_lidar_id);
  return true;
}

bool ReadPcdHeader(const fs::path& path, PcdHeader* header,
                   std::string* error) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    *error = "cannot open PCD: " + path.string();
    return false;
  }
  std::string line;
  bool have_points = false;
  bool have_data = false;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    std::istringstream parser(line);
    std::string key;
    parser >> key;
    if (key.empty() || key.front() == '#') {
      continue;
    }
    if (key == "FIELDS" || key == "FIELD") {
      std::string field;
      while (parser >> field) {
        header->fields.push_back(field);
      }
    } else if (key == "SIZE") {
      std::string token;
      while (parser >> token) {
        uint64_t value = 0;
        if (!ParseUnsigned(token, &value) || value == 0) {
          *error = path.string() + ": invalid SIZE entry";
          return false;
        }
        header->sizes.push_back(value);
      }
    } else if (key == "TYPE") {
      std::string token;
      while (parser >> token) {
        if (token.size() != 1) {
          *error = path.string() + ": invalid TYPE entry";
          return false;
        }
        header->types.push_back(token[0]);
      }
    } else if (key == "COUNT") {
      std::string token;
      while (parser >> token) {
        uint64_t value = 0;
        if (!ParseUnsigned(token, &value) || value == 0) {
          *error = path.string() + ": invalid COUNT entry";
          return false;
        }
        header->counts.push_back(value);
      }
    } else if (key == "POINTS") {
      std::string token;
      parser >> token;
      if (!ParseUnsigned(token, &header->points) || header->points == 0) {
        *error = path.string() + ": invalid POINTS entry";
        return false;
      }
      have_points = true;
    } else if (key == "DATA") {
      std::string encoding;
      parser >> encoding;
      if (encoding != "binary") {
        *error = path.string() + ": only binary PCD encoding is supported";
        return false;
      }
      const std::streampos offset = input.tellg();
      if (offset < 0) {
        *error = path.string() + ": cannot locate binary payload";
        return false;
      }
      header->data_offset = static_cast<uint64_t>(offset);
      have_data = true;
      break;
    }
  }
  if (!have_data || !have_points || header->fields.empty() ||
      header->sizes.size() != header->fields.size() ||
      header->types.size() != header->fields.size()) {
    *error = path.string() + ": incomplete PCD header";
    return false;
  }
  if (header->counts.empty()) {
    header->counts.assign(header->fields.size(), 1);
  }
  if (header->counts.size() != header->fields.size()) {
    *error = path.string() + ": COUNT does not match FIELDS";
    return false;
  }
  header->record_size = 0;
  for (std::size_t index = 0; index < header->fields.size(); ++index) {
    if (header->sizes[index] >
        std::numeric_limits<uint64_t>::max() / header->counts[index]) {
      *error = path.string() + ": PCD point record size overflow";
      return false;
    }
    const uint64_t field_size = header->sizes[index] * header->counts[index];
    if (header->record_size >
        std::numeric_limits<uint64_t>::max() - field_size) {
      *error = path.string() + ": PCD point record size overflow";
      return false;
    }
    header->record_size += field_size;
  }
  if (header->points >
      (std::numeric_limits<uint64_t>::max() - header->data_offset) /
          header->record_size) {
    *error = path.string() + ": PCD payload size overflow";
    return false;
  }
  std::error_code filesystem_error;
  const uint64_t file_size = fs::file_size(path, filesystem_error);
  const uint64_t required_size =
      header->data_offset + header->points * header->record_size;
  if (filesystem_error || file_size < required_size) {
    *error = path.string() + ": binary payload is shorter than POINTS";
    return false;
  }
  const uint16_t endian_probe = 1;
  if (*reinterpret_cast<const uint8_t*>(&endian_probe) != 1) {
    *error = path.string() + ": big-endian host is unsupported for binary PCD";
    return false;
  }
  return true;
}

bool DecodeAndCheckPoints(const fs::path& path, uint64_t expected_points,
                          uint64_t sample_limit, PcdHeader* header,
                          std::string* error) {
  if (!ReadPcdHeader(path, header, error)) {
    return false;
  }
  if (expected_points != 0 && header->points != expected_points) {
    *error = path.string() + ": manifest/header point count mismatch";
    return false;
  }
  std::array<std::size_t, 3> coordinate_indices{};
  const std::array<const char*, 3> coordinate_names = {"x", "y", "z"};
  for (std::size_t coordinate = 0; coordinate < coordinate_names.size();
       ++coordinate) {
    const auto field = std::find(header->fields.begin(), header->fields.end(),
                                 coordinate_names[coordinate]);
    if (field == header->fields.end()) {
      *error = path.string() + ": missing coordinate field " +
               coordinate_names[coordinate];
      return false;
    }
    const std::size_t index =
        static_cast<std::size_t>(field - header->fields.begin());
    if (header->types[index] != 'F' || header->counts[index] != 1 ||
        (header->sizes[index] != 4 && header->sizes[index] != 8)) {
      *error = path.string() + ": unsupported coordinate field encoding";
      return false;
    }
    coordinate_indices[coordinate] = index;
  }

  std::vector<uint64_t> coordinate_offsets(header->fields.size(), 0);
  for (std::size_t field = 1; field < header->fields.size(); ++field) {
    coordinate_offsets[field] =
        coordinate_offsets[field - 1] +
        header->sizes[field - 1] * header->counts[field - 1];
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    *error = "cannot open PCD: " + path.string();
    return false;
  }
  const uint64_t sample_count = std::min(header->points, sample_limit);
  std::vector<char> record(static_cast<std::size_t>(header->record_size));
  for (uint64_t point_index = 0; point_index < sample_count; ++point_index) {
    const uint64_t offset =
        header->data_offset + point_index * header->record_size;
    input.seekg(static_cast<std::streamoff>(offset));
    input.read(record.data(), static_cast<std::streamsize>(record.size()));
    if (!input) {
      *error = path.string() + ": cannot read binary point record";
      return false;
    }
    for (std::size_t coordinate = 0; coordinate < 3; ++coordinate) {
      const std::size_t field = coordinate_indices[coordinate];
      const std::size_t field_offset =
          static_cast<std::size_t>(coordinate_offsets[field]);
      double value = 0.0;
      if (header->sizes[field] == 4) {
        float decoded = 0.0f;
        std::memcpy(&decoded, record.data() + field_offset, sizeof(decoded));
        value = decoded;
      } else {
        std::memcpy(&value, record.data() + field_offset, sizeof(value));
      }
      if (!std::isfinite(value)) {
        *error = path.string() + ": non-finite XYZ in sampled point";
        return false;
      }
    }
  }
  return true;
}

std::vector<std::size_t> SampleIndices(std::size_t count) {
  std::vector<std::size_t> indices = {0, count / 2, count - 1};
  std::sort(indices.begin(), indices.end());
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
  return indices;
}

bool RunSmoke(const fs::path& requested_root, SmokeSummary* summary,
              std::string* error) {
  std::error_code filesystem_error;
  const fs::path root = fs::canonical(requested_root, filesystem_error);
  if (filesystem_error || !fs::is_directory(root)) {
    *error = "recording root is not an accessible directory: " +
             requested_root.string();
    return false;
  }
  std::vector<ManifestFrame> frames;
  std::string frame_id;
  std::string lidar_id;
  if (!ParseManifest(root, &frames, &frame_id, &lidar_id, error)) {
    return false;
  }
  const auto indices = SampleIndices(frames.size());
  for (const std::size_t index : indices) {
    const auto& frame = frames[index];
    PcdHeader header;
    const fs::path pcd_path = root / "pcd" / frame.file;
    if (!DecodeAndCheckPoints(pcd_path, frame.points, 64, &header, error)) {
      *error = "manifest frame " + std::to_string(index) + ": " + *error;
      return false;
    }
  }
  PcdHeader map_header;
  const fs::path map_path = root / "amap" / "amap_5cm.pcd";
  if (!DecodeAndCheckPoints(map_path, 0, 32, &map_header, error)) {
    if (map_header.points != 0 &&
        error->find("point count mismatch") != std::string::npos) {
      *error = map_path.string() + ": map has zero or malformed point count";
    }
    return false;
  }
  summary->manifest_frames = frames.size();
  summary->decoded_frames = indices.size();
  summary->map_points = map_header.points;
  summary->frame_id = std::move(frame_id);
  summary->lidar_id = std::move(lidar_id);
  return true;
}

class TemporaryDataset {
 public:
  TemporaryDataset() {
    const auto nonce =
        std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = fs::temp_directory_path() /
            ("openloc_3dmakerpro_" + std::to_string(nonce));
    fs::create_directories(root_ / "pcd");
    fs::create_directories(root_ / "amap");
  }

  ~TemporaryDataset() {
    std::error_code ignored;
    fs::remove_all(root_, ignored);
  }

  const fs::path& root() const { return root_; }

  void WritePcd(const fs::path& path, bool include_payload = true) const {
    std::ofstream output(path, std::ios::binary);
    output << "# .PCD v0.7\n"
           << "VERSION 0.7\n"
           << "FIELDS x y z intensity\n"
           << "SIZE 4 4 4 4\n"
           << "TYPE F F F F\n"
           << "COUNT 1 1 1 1\n"
           << "WIDTH 3\n"
           << "HEIGHT 1\n"
           << "POINTS 3\n"
           << "DATA binary\n";
    if (include_payload) {
      const std::array<float, 12> points = {1.0f, 2.0f,  3.0f,  4.0f,
                                            5.0f, 6.0f,  7.0f,  8.0f,
                                            9.0f, 10.0f, 11.0f, 12.0f};
      output.write(reinterpret_cast<const char*>(points.data()),
                   static_cast<std::streamsize>(sizeof(points)));
    }
  }

  void WriteManifest(const std::string& first_file = "frame_0.pcd") const {
    std::ofstream output(root_ / "pcd" / "frames.csv");
    output << "index,file,seq,stamp_sec,timebase_ns,points,frame_id,lidar_id\n"
           << "0," << first_file << ",1,1.0,1000000000,3,livox_frame,192\n"
           << "1,frame_1.pcd,2,1.1,1100000000,3,livox_frame,192\n"
           << "2,frame_2.pcd,3,1.2,1200000000,3,livox_frame,192\n";
  }

  void BuildValid() const {
    WritePcd(root_ / "pcd" / "frame_0.pcd");
    WritePcd(root_ / "pcd" / "frame_1.pcd");
    WritePcd(root_ / "pcd" / "frame_2.pcd");
    WritePcd(root_ / "amap" / "amap_5cm.pcd");
    WriteManifest();
  }

 private:
  fs::path root_;
};

TEST(ThreeDMakerProSmokeTest, ParsesManifestAndDecodesBoundedPcdSamples) {
  TemporaryDataset dataset;
  dataset.BuildValid();
  SmokeSummary summary;
  std::string error;
  ASSERT_TRUE(RunSmoke(dataset.root(), &summary, &error)) << error;
  EXPECT_EQ(3U, summary.manifest_frames);
  EXPECT_EQ(3U, summary.decoded_frames);
  EXPECT_EQ(3U, summary.map_points);
}

TEST(ThreeDMakerProSmokeTest, RejectsManifestPathTraversal) {
  TemporaryDataset dataset;
  dataset.BuildValid();
  dataset.WriteManifest("../outside.pcd");
  SmokeSummary summary;
  std::string error;
  EXPECT_FALSE(RunSmoke(dataset.root(), &summary, &error));
  EXPECT_NE(std::string::npos, error.find("traversal"));
}

TEST(ThreeDMakerProSmokeTest, RejectsNonMonotonicTimebase) {
  TemporaryDataset dataset;
  dataset.BuildValid();
  {
    std::ofstream output(dataset.root() / "pcd" / "frames.csv");
    output << "index,file,seq,stamp_sec,timebase_ns,points,frame_id,lidar_id\n"
           << "0,frame_0.pcd,1,1.0,1000000000,3,livox_frame,192\n"
           << "1,frame_1.pcd,2,1.1,1000000000,3,livox_frame,192\n";
  }
  SmokeSummary summary;
  std::string error;
  EXPECT_FALSE(RunSmoke(dataset.root(), &summary, &error));
  EXPECT_NE(std::string::npos, error.find("strictly increasing"));
}

TEST(ThreeDMakerProSmokeTest, RejectsTruncatedBinaryPcd) {
  TemporaryDataset dataset;
  dataset.BuildValid();
  dataset.WritePcd(dataset.root() / "pcd" / "frame_0.pcd", false);
  SmokeSummary summary;
  std::string error;
  EXPECT_FALSE(RunSmoke(dataset.root(), &summary, &error));
  EXPECT_NE(std::string::npos, error.find("payload is shorter"));
}

TEST(ThreeDMakerProSmokeTest, RejectsManifestPointCountMismatch) {
  TemporaryDataset dataset;
  dataset.BuildValid();
  {
    std::ofstream output(dataset.root() / "pcd" / "frames.csv");
    output << "index,file,seq,stamp_sec,timebase_ns,points,frame_id,lidar_id\n"
           << "0,frame_0.pcd,1,1.0,1000000000,4,livox_frame,192\n"
           << "1,frame_1.pcd,2,1.1,1100000000,3,livox_frame,192\n"
           << "2,frame_2.pcd,3,1.2,1200000000,3,livox_frame,192\n";
  }
  SmokeSummary summary;
  std::string error;
  EXPECT_FALSE(RunSmoke(dataset.root(), &summary, &error));
  EXPECT_NE(std::string::npos, error.find("point count mismatch"));
}

TEST(ThreeDMakerProSmokeTest, RejectsInvalidRoot) {
  TemporaryDataset dataset;
  SmokeSummary summary;
  std::string error;
  EXPECT_FALSE(RunSmoke(dataset.root() / "missing", &summary, &error));
  EXPECT_NE(std::string::npos, error.find("not an accessible directory"));
}

TEST(ThreeDMakerProSmokeTest, ExternalSessionSmokeWhenConfigured) {
  const char* root = std::getenv("OPENLOC_3DMAKERPRO_ROOT");
  if (root == nullptr || *root == '\0') {
    GTEST_SKIP() << "Set OPENLOC_3DMAKERPRO_ROOT to a mounted session, e.g. "
                 << "/mnt/synology/3dmakerpro/20260727101936";
  }
  SmokeSummary summary;
  std::string error;
  ASSERT_TRUE(RunSmoke(root, &summary, &error)) << error;
  EXPECT_EQ(3U, summary.decoded_frames);
  EXPECT_GT(summary.map_points, 0U);
  RecordProperty("manifest_frames", summary.manifest_frames);
  RecordProperty("decoded_frames", summary.decoded_frames);
  RecordProperty("map_points", summary.map_points);
  RecordProperty("frame_id", summary.frame_id);
  RecordProperty("lidar_id", summary.lidar_id);
}

}  // namespace
