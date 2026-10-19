// Exports every directed edge as one JSON line: its GraphId (as a 64-bit
// integer) and its shape, oriented along the edge:
//
//   {"id":123456789,"coords":[[lon,lat],[lon,lat],...]}
//
// The first line identifies the tile set, so data built from these edges can
// be checked against the tiles it's used with:
//
//   {"tileset_id":"osm<dataset_id>-b<build id>"}
//
// Useful for attaching external per-edge data keyed by GraphId, such as an
// edge penalty sidecar built from points of interest near the road.

#include "argparse_utils.h"
#include "baldr/graphconstants.h"
#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "midgard/logging.h"
#include "sif/camera_coverage.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace valhalla::midgard;
using namespace valhalla::baldr;

int main(int argc, char* argv[]) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  boost::property_tree::ptree config;
  bool all_modes = false;
  bool shortcuts = false;
  std::string output;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_PRINT_VERSION + "\n\n"
      "exports each directed edge's GraphId and shape as JSON lines.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("i,inline-config", "Inline json config.", cxxopts::value<std::string>())
      ("a,all-modes", "Also export edges no car may use [default=false]", cxxopts::value<bool>(all_modes)->default_value("false"))
      ("s,shortcuts", "Also export shortcut edges [default=false]", cxxopts::value<bool>(shortcuts)->default_value("false"))
      ("o,output", "Write to this file instead of stdout (keeps log lines out of the data).", cxxopts::value<std::string>(output));
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config))
      return EXIT_SUCCESS;
  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  std::ofstream file;
  if (!output.empty()) {
    file.open(output);
    if (!file) {
      std::cerr << "Cannot write " << output << std::endl;
      return EXIT_FAILURE;
    }
  }
  std::ostream& out = output.empty() ? std::cout : file;
  GraphReader reader(config.get_child("mjolnir"));
  const auto tileset_id = valhalla::sif::CameraCoverage::TilesetId(reader);
  if (tileset_id.empty()) {
    std::cerr << "No tiles found" << std::endl;
    return EXIT_FAILURE;
  }
  out << "{\"tileset_id\":\"" << tileset_id << "\"}\n";

  uint64_t exported = 0;
  char buf[64];
  for (const auto& level : TileHierarchy::levels()) {
    for (uint32_t t = 0; t < level.tiles.TileCount(); ++t) {
      GraphId tile_id{t, level.level, 0};
      if (!reader.DoesTileExist(tile_id))
        continue;
      auto tile = reader.GetGraphTile(tile_id);
      if (!tile)
        continue;
      for (uint32_t i = 0; i < tile->header()->directededgecount(); ++i) {
        const DirectedEdge* edge = tile->directededge(i);
        if (edge->is_shortcut() && !shortcuts)
          continue;
        // Keep both directions of any road a car can use in either direction:
        // reverse searches walk the opposing edges of one-way roads.
        if (!all_modes && !((edge->forwardaccess() | edge->reverseaccess()) & kAutoAccess))
          continue;
        GraphId edge_id = tile_id;
        edge_id.set_id(i);

        auto shape = tile->edgeinfo(edge).shape();
        if (!edge->forward())
          std::reverse(shape.begin(), shape.end());

        out << "{\"id\":" << edge_id.value << ",\"coords\":[";
        for (size_t p = 0; p < shape.size(); ++p) {
          std::snprintf(buf, sizeof(buf), "%s[%.7f,%.7f]", p ? "," : "", shape[p].lng(),
                        shape[p].lat());
          out << buf;
        }
        out << "]}\n";
        ++exported;
      }
      reader.Clear();
    }
  }
  out.flush();
  LOG_INFO("Exported " + std::to_string(exported) + " directed edges");
  return EXIT_SUCCESS;
}
