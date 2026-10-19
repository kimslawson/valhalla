#include "sif/camera_coverage.h"
#include "midgard/logging.h"

#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <bit>
#include <cstring>
#include <fstream>
#include <iterator>
#include <mutex>
#include <stdexcept>

namespace valhalla {
namespace sif {

static_assert(std::endian::native == std::endian::little, "coverage.bin is little-endian");

namespace {

constexpr char kMagic[8] = {'O', 'U', 'T', 'I', 'S', 'C', 'O', 'V'};
constexpr uint16_t kVersion = 1;

std::mutex global_mutex;
std::shared_ptr<const CameraCoverage> global_coverage;

// Reads fixed-size values off a byte buffer, failing loudly past the end.
struct Reader {
  const std::vector<char>& data;
  size_t at = 0;

  template <typename T> T get() {
    if (at + sizeof(T) > data.size())
      throw std::runtime_error("coverage file is truncated");
    T value;
    std::memcpy(&value, data.data() + at, sizeof(T));
    at += sizeof(T);
    return value;
  }

  template <typename T> void get_many(std::vector<T>& out, size_t n) {
    if (n > (data.size() - at) / sizeof(T))
      throw std::runtime_error("coverage file is truncated");
    out.resize(n);
    if (n)
      std::memcpy(out.data(), data.data() + at, n * sizeof(T));
    at += n * sizeof(T);
  }
};

} // namespace

std::shared_ptr<const CameraCoverage> CameraCoverage::Load(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file)
    throw std::runtime_error("cannot open coverage file " + path);
  std::vector<char> data((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  auto coverage = std::make_shared<CameraCoverage>();
  Reader r{data};
  char magic[8];
  for (char& c : magic)
    c = r.get<char>();
  if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0)
    throw std::runtime_error("not a coverage file: " + path);
  if (r.get<uint16_t>() != kVersion)
    throw std::runtime_error("unsupported coverage file version: " + path);
  r.get<uint16_t>(); // reserved
  const auto build_id_len = r.get<uint16_t>();
  std::vector<char> build_id;
  r.get_many(build_id, build_id_len);
  coverage->build_id_.assign(build_id.begin(), build_id.end());

  const auto camera_count = r.get<uint32_t>();
  const auto edge_count = r.get<uint32_t>();
  r.get_many(coverage->objects_, camera_count);
  coverage->graph_ids_.reserve(edge_count);
  coverage->offsets_.reserve(edge_count);
  for (uint32_t i = 0; i < edge_count; ++i) {
    coverage->graph_ids_.push_back(r.get<uint64_t>());
    coverage->offsets_.push_back(r.get<uint32_t>());
  }
  const size_t rest = data.size() - r.at;
  if (rest % sizeof(uint32_t))
    throw std::runtime_error("coverage file list array is truncated");
  r.get_many(coverage->lists_, rest / sizeof(uint32_t));

  // Validate everything a lookup will rely on, once, here.
  if (!std::is_sorted(coverage->graph_ids_.begin(), coverage->graph_ids_.end()))
    throw std::runtime_error("coverage file edges are not sorted");
  const auto& lists = coverage->lists_;
  for (const auto offset : coverage->offsets_) {
    if (offset >= lists.size() || lists[offset] > lists.size() - offset - 1)
      throw std::runtime_error("coverage file offset out of range");
    for (uint32_t k = 1; k <= lists[offset]; ++k)
      if (lists[offset + k] >= camera_count)
        throw std::runtime_error("coverage file names an unknown camera");
  }
  return coverage;
}

std::shared_ptr<const CameraCoverage> CameraCoverage::Global() {
  std::lock_guard<std::mutex> lock(global_mutex);
  return global_coverage;
}

void CameraCoverage::SetGlobal(std::shared_ptr<const CameraCoverage> coverage) {
  std::lock_guard<std::mutex> lock(global_mutex);
  global_coverage = std::move(coverage);
}

void CameraCoverage::LoadFromConfig(const boost::property_tree::ptree& config) {
  const auto path = config.get<std::string>("thor.camera_coverage", "");
  if (path.empty() || Global())
    return;
  try {
    auto coverage = Load(path);
    const auto expected = config.get<std::string>("thor.camera_coverage_build_id", "");
    if (!expected.empty() && coverage->build_id() != expected) {
      LOG_ERROR("Camera coverage " + path + " was built for tiles '" + coverage->build_id() +
                "', not '" + expected + "': camera options disabled");
      return;
    }
    SetGlobal(coverage);
    LOG_INFO("Loaded camera coverage " + path + ": " + std::to_string(coverage->camera_count()) +
             " cameras, build " + coverage->build_id());
  } catch (const std::exception& e) {
    LOG_ERROR(std::string("Camera coverage not loaded, camera options disabled: ") + e.what());
  }
}

CameraCoverage::Span CameraCoverage::cameras_on(const baldr::GraphId& edge) const {
  const uint64_t id = edge.value;
  auto it = std::lower_bound(graph_ids_.begin(), graph_ids_.end(), id);
  if (it == graph_ids_.end() || *it != id)
    return {};
  const uint32_t offset = offsets_[it - graph_ids_.begin()];
  return {lists_.data() + offset + 1, lists_[offset]};
}

int64_t CameraCoverage::index_of(uint64_t packed_id) const {
  auto it = std::find(objects_.begin(), objects_.end(), packed_id);
  return it == objects_.end() ? -1 : static_cast<int64_t>(it - objects_.begin());
}

} // namespace sif
} // namespace valhalla
