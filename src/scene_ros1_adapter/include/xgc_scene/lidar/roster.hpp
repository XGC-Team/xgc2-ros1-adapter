#pragma once

#include "xgc_scene/common.hpp"
#include "xgc_scene/lidar/manifest.hpp"

#include <string>
#include <string_view>

namespace xgc_scene::lidar {

// The process receives the full frozen $session unchanged on stdin, at most
// 8388608 bytes. The fleet reads runMode, robots and visualizationTopics; Scene
// authoring resolves the same native working copy as World from scene/runtime.
// Core computes neither a fleet manifest nor the native working-copy path.
// RobotFact.namespace and binding.{hybridSource,simulationSensors.simpleLidar}
// supply the sensor selection. An absent simpleLidar is off; a boolean selects
// the simulator's native default model. Asset/profile/identity fields are not
// interpreted, and the authored numeric domains are validated by Core.
// The caller selects the Gazebo profile: the legacy absent-scene/xsim gate is
// outside this API. Physical runs produce an empty fleet; hybrid runs select
// only binding.hybridSource == "simulation". The 512-robot limit is checked
// before filtering. Sampled sensors require explicit "cpu" acceleration.
// A visible3d or visibleAr semantic sensor_msgs/PointCloud2 declaration for
// /xgc/scene/reference_cloud adds referenceCloud.surfaceSpacing = 0.1.
struct SessionInput {
  std::string run_mode;
  Json robots;
  Json visualization_topics;
  Json facts = Json::object();
};

// Requires a JSON object with all three members, a recognized runMode, an
// array of robot objects and a visualizationTopics array. Refusals are
// SceneError; structural errors name the offending member. Roster and topic
// values are retained unchanged, including fields this adapter does not read.
SessionInput parseSessionInput(std::string_view text);

// Accepts the shape returned by parseSessionInput. Produces the legacy
// schemaVersion-1 manifest, with only sampled sensors in robots and optional
// referenceCloud. Empty output is {"schemaVersion":1,"robots":[]}.
// Presets, defaults, positive numeric overrides, namespace normalization and
// refusal texts follow prepare_fleet.py. The dumped manifest is <= 1048576
// bytes; no body roster or asset-derived sensor fields are added.
Json buildFleetJson(const SessionInput &input);

// Non-empty output passes through parseManifest with the supplied node
// defaults. Empty output returns an empty FleetManifest because parseManifest
// requires at least one sensor or a reference cloud.
FleetManifest buildFleet(const SessionInput &input,
                         const xgc2_math::scene::SensorConfig &defaults = {});

} // namespace xgc_scene::lidar
