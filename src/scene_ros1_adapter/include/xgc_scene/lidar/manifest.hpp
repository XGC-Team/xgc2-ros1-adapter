#pragma once

#include <xgc2_math/scene/sensor_model.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace xgc_scene::lidar {

struct SensorSpec {
  std::string backend = "cpu";
  std::string name_space;
  std::string pose_topic;
  int body_id = 0;
  xgc2_math::scene::SensorConfig config;
  double rate_hz = 10.0;
  bool publish_beams = false;
};

struct BodySpec {
  int id = 0;
  std::string name_space;
  std::string pose_topic;
  double radius = 0.0;
};

struct ReferenceSpec {
  double surface_spacing = 0.1;
};

// Frozen for one engine lifetime. Indices in robots/bodies are the plain C++
// handles used by the edge; namespaces and pose topics are edge metadata only.
struct FleetManifest {
  std::vector<SensorSpec> robots;
  std::vector<BodySpec> bodies;
  bool body_roster = false;
  std::optional<ReferenceSpec> reference;
};

constexpr std::size_t kMaxManifestBytes = 1024 * 1024;
constexpr std::size_t kMaxFleetMembers = 512;

// The original fleet parser's defaults, conversions and refusal messages.
// defaults carries the node-level sensor configuration (including noise/seed).
FleetManifest
parseManifest(std::string_view text,
              const xgc2_math::scene::SensorConfig &defaults = {});

} // namespace xgc_scene::lidar
