#pragma once

#include <valhalla/baldr/graphid.h>

#include <boost/property_tree/ptree_fwd.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace valhalla {
namespace sif {

/**
 * Which cameras cover each directed edge, read from an Outis coverage.bin
 * (format documented in the Outis pipeline, outis_pipeline/coverage_bin.py).
 * Loaded once at startup and shared read-only by every request.
 *
 * Cameras are identified by counting index: cameras that count as one share
 * an index, so a route entering their coverage is charged once.
 */
class CameraCoverage {
public:
  /// Cameras covering an edge, as ascending counting indices.
  struct Span {
    const uint32_t* first = nullptr;
    uint32_t size = 0;
    const uint32_t* begin() const {
      return first;
    }
    const uint32_t* end() const {
      return first + size;
    }
    bool empty() const {
      return size == 0;
    }
  };

  /// Reads and validates a coverage file. Throws std::runtime_error if it is
  /// missing or damaged: a half-read file must never be used.
  static std::shared_ptr<const CameraCoverage> Load(const std::string& path);

  /// The coverage loaded at startup, or nullptr if none.
  static std::shared_ptr<const CameraCoverage> Global();
  static void SetGlobal(std::shared_ptr<const CameraCoverage> coverage);

  /// Loads thor.camera_coverage once, if configured. If the file is damaged,
  /// or thor.camera_coverage_build_id is set and doesn't match, nothing is
  /// loaded and camera options fail (robustness.md R-7).
  static void LoadFromConfig(const boost::property_tree::ptree& config);

  Span cameras_on(const baldr::GraphId& edge) const;

  const std::string& build_id() const {
    return build_id_;
  }
  uint32_t camera_count() const {
    return static_cast<uint32_t>(objects_.size());
  }
  /// Packed OSM id of each counting index: type in the top two bits
  /// (0 node, 1 way, 2 relation), id below.
  const std::vector<uint64_t>& objects() const {
    return objects_;
  }
  /// Counting index of a packed OSM id, or -1 if it isn't in the file.
  int64_t index_of(uint64_t packed_id) const;

private:
  std::string build_id_;
  std::vector<uint64_t> objects_;
  std::vector<uint64_t> graph_ids_;
  std::vector<uint32_t> offsets_;
  std::vector<uint32_t> lists_;
};

} // namespace sif
} // namespace valhalla
