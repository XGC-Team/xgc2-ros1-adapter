#include "xgc_scene/lidar/roster.hpp"

#include <gtest/gtest.h>

#include <string>

using xgc_scene::Json;
using xgc_scene::SceneError;
using namespace xgc_scene::lidar;
using xgc2_math::scene::SensorConfig;

namespace {
struct GoldenCase {
  const char *name;
  const char *session;
  const char *expected;
  bool refused = false;
  std::size_t repeat_robots = 0;
};

// Frozen Session-to-sensor contracts; compute selection preserves the observation model.
// The generator and legacy envelopes stay outside the repository. Boundary
// cases expand their inline RobotFact to repeat_robots identical entries.
const GoldenCase
    golden_cases[] =
        {
            {"empty roster",
             R"json({"runMode":"simulation","robots":[],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"absent sensor selection",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"absent simulation sensors",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation"},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"boolean off",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":false}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"boolean native model",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":true}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"empty object",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"enabled native object",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"numeric parameters keep native model",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"rateHz":25,"rangeMeters":12}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"CPU alone keeps native model",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"disabled sampled object",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":false,"mode":"penetrating","acceleration":"gpu","publishBeams":true}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"missing enabled is off",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"mode":"penetrating","acceleration":"gpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"penetrating defaults",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"penetrating","rateHz":10,"rangeMeters":20,"hFovDeg":360,"vFovDeg":30,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}]})json"},
            {"depth defaults",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"depth_frustum","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"depth_frustum","rateHz":10,"rangeMeters":20,"hFovDeg":90,"vFovDeg":60,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}]})json"},
            {"raycast native without beams",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"raycast","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"default mode with beams",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"","acceleration":"cpu","publishBeams":true}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"raycast","rateHz":10,"rangeMeters":20,"hFovDeg":360,"vFovDeg":30,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":true,"backend":"cpu"}]})json"},
            {"raycast beams",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"raycast","acceleration":"cpu","publishBeams":true}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"raycast","rateHz":10,"rangeMeters":20,"hFovDeg":360,"vFovDeg":30,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":true,"backend":"cpu"}]})json"},
            {"depth beams",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"depth_frustum","acceleration":"cpu","publishBeams":true}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"depth_frustum","rateHz":10,"rangeMeters":20,"hFovDeg":90,"vFovDeg":60,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":true,"backend":"cpu"}]})json"},
            {"bridge preset",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"","acceleration":"cpu","preset":"bridge_equivalent"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"penetrating","rateHz":10,"rangeMeters":8,"hFovDeg":360,"vFovDeg":180,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":true,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}]})json"},
            {"bridge preset with explicit depth",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"depth_frustum","acceleration":"cpu","preset":"bridge_equivalent"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"depth_frustum","rateHz":10,"rangeMeters":8,"hFovDeg":90,"vFovDeg":60,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":true,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}]})json"},
            {"bridge preset with raycast beams",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"raycast","acceleration":"cpu","preset":"bridge_equivalent","publishBeams":true}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"raycast","rateHz":10,"rangeMeters":8,"hFovDeg":360,"vFovDeg":180,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":true,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":true,"backend":"cpu"}]})json"},
            {"bridge preset with native raycast",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"raycast","acceleration":"cpu","preset":"bridge_equivalent"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"bridge preset refuses penetrating beams",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"","acceleration":"cpu","preset":"bridge_equivalent","publishBeams":true}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(penetrating sampled clouds have no beams)json", true},
            {"CPU crop preset",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"","acceleration":"cpu","preset":"zju_cpu_crop"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"penetrating","rateHz":10,"rangeMeters":5,"hFovDeg":360,"vFovDeg":180,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":true,"headingCrop":true,"headingCosMin":0.5,"verticalSlabTan":0.5773502691896258,"publishBeams":false,"backend":"cpu"}]})json"},
            {"CPU crop preset with explicit depth",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"depth_frustum","acceleration":"cpu","preset":"zju_cpu_crop"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"depth_frustum","rateHz":10,"rangeMeters":5,"hFovDeg":90,"vFovDeg":60,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":true,"headingCrop":true,"headingCosMin":0.5,"verticalSlabTan":0.5773502691896258,"publishBeams":false,"backend":"cpu"}]})json"},
            {"CPU crop preset with raycast beams",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"raycast","acceleration":"cpu","preset":"zju_cpu_crop","publishBeams":true}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"raycast","rateHz":10,"rangeMeters":5,"hFovDeg":360,"vFovDeg":180,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":true,"headingCrop":true,"headingCosMin":0.5,"verticalSlabTan":0.5773502691896258,"publishBeams":true,"backend":"cpu"}]})json"},
            {"CPU crop preset with native raycast",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"raycast","acceleration":"cpu","preset":"zju_cpu_crop"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"CPU crop preset refuses penetrating beams",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"","acceleration":"cpu","preset":"zju_cpu_crop","publishBeams":true}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(penetrating sampled clouds have no beams)json", true},
            {"ego crop preset",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"","acceleration":"cpu","preset":"zju_cpu_crop_ego_v2"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"penetrating","rateHz":10,"rangeMeters":5,"hFovDeg":360,"vFovDeg":180,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":true,"headingCrop":true,"headingCosMin":0,"verticalSlabTan":0.5773502691896258,"publishBeams":false,"backend":"cpu"}]})json"},
            {"ego crop preset with explicit depth",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"depth_frustum","acceleration":"cpu","preset":"zju_cpu_crop_ego_v2"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"depth_frustum","rateHz":10,"rangeMeters":5,"hFovDeg":90,"vFovDeg":60,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":true,"headingCrop":true,"headingCosMin":0,"verticalSlabTan":0.5773502691896258,"publishBeams":false,"backend":"cpu"}]})json"},
            {"ego crop preset with raycast beams",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"raycast","acceleration":"cpu","preset":"zju_cpu_crop_ego_v2","publishBeams":true}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"raycast","rateHz":10,"rangeMeters":5,"hFovDeg":360,"vFovDeg":180,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":true,"headingCrop":true,"headingCosMin":0,"verticalSlabTan":0.5773502691896258,"publishBeams":true,"backend":"cpu"}]})json"},
            {"ego crop preset with native raycast",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"raycast","acceleration":"cpu","preset":"zju_cpu_crop_ego_v2"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"ego crop preset refuses penetrating beams",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"","acceleration":"cpu","preset":"zju_cpu_crop_ego_v2","publishBeams":true}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(penetrating sampled clouds have no beams)json", true},
            {"penetrating refuses beams",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu","publishBeams":true}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(penetrating sampled clouds have no beams)json", true},


            {"all numeric overrides",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu","rateHz":25.0,"rangeMeters":13.75,"hFovDeg":240.5,"vFovDeg":45.25,"hRes":72,"vRes":8}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"penetrating","rateHz":25.0,"rangeMeters":13.75,"hFovDeg":240.5,"vFovDeg":45.25,"hRes":72,"vRes":8,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}]})json"},
            {"depth overrides follow model defaults",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"depth_frustum","acceleration":"cpu","rateHz":12.5,"rangeMeters":7,"hFovDeg":80,"vFovDeg":40,"hRes":16,"vRes":12}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"depth_frustum","rateHz":12.5,"rangeMeters":7,"hFovDeg":80,"vFovDeg":40,"hRes":16,"vRes":12,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}]})json"},
            {"zero overrides keep defaults",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu","rateHz":0,"rangeMeters":0,"hFovDeg":0,"vFovDeg":0,"hRes":0,"vRes":0}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"penetrating","rateHz":10,"rangeMeters":20,"hFovDeg":360,"vFovDeg":30,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}]})json"},
            {"negative overrides are rejected",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu","rateHz":-1,"rangeMeters":-1,"hFovDeg":-1,"vFovDeg":-1,"hRes":-1,"vRes":-1}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(invalid authored simpleLidar.rateHz)json", true},
            {"fractional overrides round trip",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu","rateHz":1e-05,"rangeMeters":0.30000000000000004,"hFovDeg":123.45678901234567}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"penetrating","rateHz":1e-05,"rangeMeters":0.30000000000000004,"hFovDeg":123.45678901234567,"vFovDeg":30,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}]})json"},
            {"relative nested namespace",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"Robot_1/scan","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/Robot_1/scan","mode":"penetrating","rateHz":10,"rangeMeters":20,"hFovDeg":360,"vFovDeg":30,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}]})json"},
            {"leading slashes normalize",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"///Robot_1","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/Robot_1","mode":"penetrating","rateHz":10,"rangeMeters":20,"hFovDeg":360,"vFovDeg":30,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}]})json"},
            {"invalid namespace: root",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json", R"json(invalid lidar namespace)json",
             true},
            {"invalid namespace: empty",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(invalid lidar namespace)json", true},
            {"invalid namespace: leading digit",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/1robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(invalid lidar namespace)json", true},
            {"invalid namespace: hyphen",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot-1","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(invalid lidar namespace)json", true},
            {"invalid namespace: internal empty segment",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot//scan","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(invalid lidar namespace)json", true},
            {"invalid namespace: trailing slash",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot/","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(invalid lidar namespace)json", true},
            {"invalid namespace: space",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot scan","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(invalid lidar namespace)json", true},
            {"160 byte namespace accepted",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr","mode":"penetrating","rateHz":10,"rangeMeters":20,"hFovDeg":360,"vFovDeg":30,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}]})json"},
            {"161 byte namespace refused",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(invalid lidar namespace)json", true},
            {"invalid namespace ignored for native model",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/bad-","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":true}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"simulation ignores physical source",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"physical","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"penetrating","rateHz":10,"rangeMeters":20,"hFovDeg":360,"vFovDeg":30,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}]})json"},
            {"hybrid selects simulated slots",
             R"json({"runMode":"hybrid","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/sim","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"depth_frustum","acceleration":"cpu"}}},"asset":{}},{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/bad-","profile":{},"binding":{"hybridSource":"physical","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"gpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/sim","mode":"depth_frustum","rateHz":10,"rangeMeters":20,"hFovDeg":90,"vFovDeg":60,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}]})json"},
            {"hybrid skips physical slots",
             R"json({"runMode":"hybrid","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"physical","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"gpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"hybrid skips empty source",
             R"json({"runMode":"hybrid","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"physical ignores sampled refusals",
             R"json({"runMode":"physical","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/bad-","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"gpu","publishBeams":true}}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"512 robots accepted before selection",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":false}},"asset":{}}],"visualizationTopics":[]})json",
             R"json({"schemaVersion":1,"robots":[]})json", false, 512},
            {"513 robots refused before selection",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":false}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(world lidar supports at most 512 robots)json", true, 513},
            {"physical still refuses 513 robots",
             R"json({"runMode":"physical","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":false}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(world lidar supports at most 512 robots)json", true, 513},
            {"hybrid still refuses 513 physical robots",
             R"json({"runMode":"hybrid","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"physical","simulationSensors":{"simpleLidar":false}},"asset":{}}],"visualizationTopics":[]})json",
             R"json(world lidar supports at most 512 robots)json", true, 513},
            {"reference missing visibility",
             R"json({"runMode":"simulation","robots":[],"visualizationTopics":[{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/PointCloud2","role":"semantic"}]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"reference invisible",
             R"json({"runMode":"simulation","robots":[],"visualizationTopics":[{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/PointCloud2","role":"semantic","visible3d":false,"visibleAr":false}]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"reference visible 3D",
             R"json({"runMode":"simulation","robots":[],"visualizationTopics":[{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/PointCloud2","role":"semantic","visible3d":true}]})json",
             R"json({"schemaVersion":1,"robots":[],"referenceCloud":{"surfaceSpacing":0.1}})json"},
            {"reference visible AR",
             R"json({"runMode":"simulation","robots":[],"visualizationTopics":[{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/PointCloud2","role":"semantic","visibleAr":true}]})json",
             R"json({"schemaVersion":1,"robots":[],"referenceCloud":{"surfaceSpacing":0.1}})json"},
            {"reference visible both",
             R"json({"runMode":"simulation","robots":[],"visualizationTopics":[{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/PointCloud2","role":"semantic","visible3d":true,"visibleAr":true}]})json",
             R"json({"schemaVersion":1,"robots":[],"referenceCloud":{"surfaceSpacing":0.1}})json"},
            {"reference wrong role",
             R"json({"runMode":"simulation","robots":[],"visualizationTopics":[{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/PointCloud2","role":"telemetry","visible3d":true}]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"reference wrong message type",
             R"json({"runMode":"simulation","robots":[],"visualizationTopics":[{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/LaserScan","role":"semantic","visible3d":true}]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"reference wrong topic",
             R"json({"runMode":"simulation","robots":[],"visualizationTopics":[{"topic":"/other","messageType":"sensor_msgs/PointCloud2","role":"semantic","visible3d":true}]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
            {"reference after other layers",
             R"json({"runMode":"simulation","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[{"topic":"/other","messageType":"sensor_msgs/PointCloud2","role":"semantic","visible3d":true},{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/PointCloud2","role":"semantic","visibleAr":true}]})json",
             R"json({"schemaVersion":1,"robots":[{"namespace":"/robot","mode":"penetrating","rateHz":10,"rangeMeters":20,"hFovDeg":360,"vFovDeg":30,"hRes":360,"vRes":32,"surfaceSpacing":0.1,"keepBuried":false,"headingCrop":false,"headingCosMin":0,"verticalSlabTan":0,"publishBeams":false,"backend":"cpu"}],"referenceCloud":{"surfaceSpacing":0.1}})json"},
            {"reference duplicate declarations",
             R"json({"runMode":"simulation","robots":[],"visualizationTopics":[{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/PointCloud2","role":"semantic","visible3d":true},{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/PointCloud2","role":"semantic","visibleAr":true}]})json",
             R"json({"schemaVersion":1,"robots":[],"referenceCloud":{"surfaceSpacing":0.1}})json"},
            {"hybrid physical slots can have reference",
             R"json({"runMode":"hybrid","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"physical","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"gpu"}}},"asset":{}}],"visualizationTopics":[{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/PointCloud2","role":"semantic","visible3d":true}]})json",
             R"json({"schemaVersion":1,"robots":[],"referenceCloud":{"surfaceSpacing":0.1}})json"},
            {"physical ignores visible reference",
             R"json({"runMode":"physical","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{},"binding":{"hybridSource":"simulation","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating","acceleration":"cpu"}}},"asset":{}}],"visualizationTopics":[{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/PointCloud2","role":"semantic","visible3d":true}]})json",
             R"json({"schemaVersion":1,"robots":[]})json"},
};

void refusedInput(const std::string &text, const std::string &member) {
  try {
    parseSessionInput(text);
    FAIL() << "accepted session input";
  } catch (const SceneError &error) {
    EXPECT_NE(std::string::npos, std::string(error.what()).find(member));
  }
}
} // namespace

TEST(Roster, GoldenLegacyPrepare) {
  for (const auto &golden : golden_cases) {
    SCOPED_TRACE(golden.name);
    auto document = Json::parse(golden.session);
    if (golden.repeat_robots) {
      const auto robot = document["robots"][0];
      document["robots"] = Json::array();
      for (std::size_t i = 0; i < golden.repeat_robots; ++i)
        document["robots"].push_back(robot);
    }
    const auto input = parseSessionInput(document.dump());
    if (golden.refused) {
      try {
        buildFleetJson(input);
        FAIL() << "accepted legacy refusal";
      } catch (const SceneError &error) {
        EXPECT_EQ(std::string(golden.expected), error.what());
      }
      continue;
    }
    const auto expected = Json::parse(golden.expected);
    const auto actual = buildFleetJson(input);
    EXPECT_EQ(expected, actual);
    // JSON object member order does not change the native contract.
    EXPECT_EQ(expected, Json::parse(actual.dump()));
    if (!actual["robots"].empty() || actual.contains("referenceCloud")) {
      const auto fleet = parseManifest(actual.dump());
      EXPECT_EQ(expected["robots"].size(), fleet.robots.size());
      EXPECT_EQ(expected.contains("referenceCloud"),
                fleet.reference.has_value());
      if (fleet.reference) {
        EXPECT_EQ(0.1, fleet.reference->surface_spacing);
      }
    }
  }
}

TEST(Roster, ParsePreservesFrozenFacts) {
  const auto document = Json::parse(R"json({"runMode":"hybrid","robots":[{"slotId":"slot","assetId":"asset","assetCommitId":"frozen","name":"Robot","kind":"scout_mini","namespace":"/robot","profile":{"id":"profile","channels":[{"id":"sensor"}]},"binding":{"hybridSource":"simulation","namespace":"/binding","simulationSensors":{"simpleLidar":{"enabled":true,"mode":"depth_frustum","acceleration":"cpu"}},"initialPose":{"x":1.25}},"asset":{"name":"Frozen asset","scout":{"lidarSimulationEnabled":true}}}],"visualizationTopics":[{"topic":"/other","messageType":"std_msgs/Bool","role":"semantic","visibleAr":true}]})json");
  const auto input = parseSessionInput(document.dump());
  EXPECT_EQ("hybrid", input.run_mode);
  EXPECT_EQ(document["robots"], input.robots);
  EXPECT_EQ(document["visualizationTopics"], input.visualization_topics);
  EXPECT_EQ(document, input.facts);
  const auto before = input.robots;
  const auto fleet = buildFleetJson(input);
  ASSERT_EQ(1u, fleet["robots"].size());
  EXPECT_EQ("/robot", fleet["robots"][0]["namespace"]);
  EXPECT_EQ(before, input.robots);
}

TEST(Roster, ParseRefusesMalformedJsonAndWrongMembers) {
  for (const auto *text : {"", "{", "null", "[]", "42", "\"session\""})
    refusedInput(text, "runMode");
  const Json valid = {{"runMode", "simulation"},
                      {"robots", Json::array()},
                      {"visualizationTopics", Json::array()}};
  for (const auto *member : {"runMode", "robots", "visualizationTopics"}) {
    auto document = valid;
    document.erase(member);
    refusedInput(document.dump(), member);
  }
  for (const auto &mode :
       {Json(nullptr), Json(true), Json(1), Json::object(), Json::array(),
        Json("Simulation"), Json("xsim"), Json("")}) {
    auto document = valid;
    document["runMode"] = mode;
    refusedInput(document.dump(), "runMode");
  }
  for (const auto *member : {"robots", "visualizationTopics"}) {
    for (const auto &value :
         {Json(nullptr), Json(true), Json(1), Json("array"), Json::object()}) {
      auto document = valid;
      document[member] = value;
      refusedInput(document.dump(), member);
    }
  }
  for (const auto &robot :
       {Json(nullptr), Json(true), Json(1), Json("robot"), Json::array()}) {
    auto document = valid;
    document["robots"] = Json::array({Json::object(), robot});
    refusedInput(document.dump(), "robots");
  }
  for (const auto *mode : {"simulation", "hybrid", "physical"}) {
    auto document = valid;
    document["runMode"] = mode;
    EXPECT_EQ(mode, parseSessionInput(document.dump()).run_mode);
  }
}

TEST(Roster, SessionInputByteLimit) {
  std::string text =
      R"({"runMode":"simulation","robots":[],"visualizationTopics":[]})";
  text.resize(xgc_scene::kMaxDocumentBytes, ' ');
  EXPECT_TRUE(parseSessionInput(text).robots.empty());
  text += ' ';
  try {
    parseSessionInput(text);
    FAIL() << "accepted oversized session input";
  } catch (const SceneError &error) {
    EXPECT_STREQ("frozen Experiment input exceeds 8 MiB", error.what());
  }
}

TEST(Roster, ManifestSensorModelsRatesAndNodeDefaults) {
  const auto input = parseSessionInput(R"json({"runMode":"simulation","robots":[{"namespace":"/native","binding":{"simulationSensors":{"simpleLidar":true}}},{"namespace":"/rays","binding":{"simulationSensors":{"simpleLidar":{"enabled":true,"mode":"raycast","acceleration":"cpu","publishBeams":true,"rateHz":25,"rangeMeters":12,"hRes":24,"vRes":8}}}},{"namespace":"/samples","binding":{"simulationSensors":{"simpleLidar":{"enabled":true,"preset":"bridge_equivalent","acceleration":"cpu"}}}},{"namespace":"/camera","binding":{"simulationSensors":{"simpleLidar":{"enabled":true,"mode":"depth_frustum","acceleration":"cpu","rateHz":17.5,"hRes":20,"vRes":8}}}}],"visualizationTopics":[{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/PointCloud2","role":"semantic","visibleAr":true}]})json");
  SensorConfig defaults;
  defaults.noise_std = 0.02;
  defaults.seed = 37;
  defaults.min_range = 0.05;
  const auto fleet = buildFleet(input, defaults);
  ASSERT_EQ(3u, fleet.robots.size());
  EXPECT_FALSE(fleet.body_roster);
  EXPECT_TRUE(fleet.bodies.empty());
  ASSERT_TRUE(fleet.reference);
  EXPECT_EQ(0.1, fleet.reference->surface_spacing);
  const auto &rays = fleet.robots[0];
  EXPECT_EQ("/rays", rays.name_space);
  EXPECT_EQ("/rays/pose", rays.pose_topic);
  EXPECT_EQ(1, rays.body_id);
  EXPECT_EQ(SensorConfig::kRaycast, rays.config.mode);
  EXPECT_EQ(25.0, rays.rate_hz);
  EXPECT_EQ(12.0, rays.config.range);
  EXPECT_EQ(24, rays.config.h_res);
  EXPECT_EQ(8, rays.config.v_res);
  EXPECT_TRUE(rays.publish_beams);
  const auto &samples = fleet.robots[1];
  EXPECT_EQ(SensorConfig::kPenetrating, samples.config.mode);
  EXPECT_EQ(10.0, samples.rate_hz);
  EXPECT_EQ(8.0, samples.config.range);
  EXPECT_EQ(180.0, samples.config.v_fov_deg);
  EXPECT_TRUE(samples.config.penetrating_keep_buried);
  EXPECT_FALSE(samples.config.penetrating_heading_crop);
  EXPECT_FALSE(samples.publish_beams);
  const auto &camera = fleet.robots[2];
  EXPECT_EQ(SensorConfig::kDepthFrustum, camera.config.mode);
  EXPECT_EQ(17.5, camera.rate_hz);
  EXPECT_EQ(90.0, camera.config.h_fov_deg);
  EXPECT_EQ(60.0, camera.config.v_fov_deg);
  EXPECT_EQ(20, camera.config.width);
  EXPECT_EQ(8, camera.config.height);
  for (const auto &sensor : fleet.robots) {
    EXPECT_EQ(0.02, sensor.config.noise_std);
    EXPECT_EQ(37u, sensor.config.seed);
    EXPECT_EQ(0.05, sensor.config.min_range);
  }
}

TEST(Roster, SavedXsimCloudTargetsOptInEvenWithoutAnEnabledLidar) {
  const std::string text = R"json({"runMode":"simulation","scene":{"simulator":"xsim"},"robots":[
    {"namespace":"/scanner","binding":{"simulationSensors":{"simpleLidar":{"enabled":true,"acceleration":"gpu"}}}},
    {"namespace":"/body","binding":{"simulationSensors":{"simpleLidar":{"enabled":false,"bodyReturns":true}}}},
    {"namespace":"/off","binding":{"simulationSensors":{"simpleLidar":{"enabled":false,"bodyReturns":false}}}}
  ],"visualizationTopics":[]})json";
  auto input = parseSessionInput(text);
  const auto before = input.facts;
  const auto json = buildFleetJson(input, true);
  ASSERT_EQ(1u, json["robots"].size());
  ASSERT_EQ(1u, json["bodies"].size());
  EXPECT_EQ("/body", json["bodies"][0]["namespace"]);
  EXPECT_EQ("/body/pose", json["bodies"][0]["poseTopic"]);
  EXPECT_EQ(2, json["bodies"][0]["id"]);
  EXPECT_TRUE(json["bodies"][0]["bodyReturns"]);
  EXPECT_EQ(1, json["robots"][0]["bodyId"]);
  const auto fleet = buildFleet(input, {}, true);
  EXPECT_TRUE(fleet.bodies[0].body_returns);
  EXPECT_EQ("gpu", fleet.robots[0].backend);
  EXPECT_EQ(before, input.facts);
  input.facts["scene"]["simulator"] = "gazebo";
  EXPECT_FALSE(buildFleetJson(input, true).contains("bodies"));
  input.facts["scene"]["simulator"] = "xsim";
  EXPECT_FALSE(buildFleetJson(input, false).contains("bodies"));
}

TEST(Roster, SavedCloudDefaultFalseHasNoBodyRoster) {
  auto input = parseSessionInput(R"json({"runMode":"simulation","scene":{"simulator":"xsim"},
    "robots":[{"namespace":"/A","binding":{"simulationSensors":{"simpleLidar":true}}},
      {"namespace":"/B","binding":{"simulationSensors":{"simpleLidar":{"enabled":false,"bodyReturns":false}}}}],
    "visualizationTopics":[]})json");
  const auto json = buildFleetJson(input, true);
  EXPECT_FALSE(json.contains("bodies"));
  const auto fleet = buildFleet(input, {}, true);
  EXPECT_TRUE(fleet.bodies.empty());
  EXPECT_EQ("auto", fleet.robots[0].backend);
}

TEST(Roster, SavedXsimCloudInheritsSceneBackendWithoutChangingObservations) {
  const auto original = parseSessionInput(R"json({"runMode":"simulation","scene":{"simulator":"xsim"},
    "robots":[
      {"namespace":"/rays","binding":{"simulationSensors":{"simpleLidar":{"enabled":true,"mode":"raycast","rateHz":25,"rangeMeters":12,"hRes":24,"vRes":8}}}},
      {"namespace":"/samples","binding":{"simulationSensors":{"simpleLidar":{"enabled":true,"acceleration":"","preset":"zju_cpu_crop"}}}},
      {"namespace":"/camera","binding":{"simulationSensors":{"simpleLidar":{"enabled":true,"mode":"depth_frustum"}}}}
    ],"visualizationTopics":[]})json");
  const auto baseline = buildFleetJson(original, true);
  ASSERT_EQ(3u, baseline["robots"].size());
  EXPECT_EQ("raycast", baseline["robots"][0]["mode"]);
  EXPECT_EQ("penetrating", baseline["robots"][1]["mode"]);
  EXPECT_EQ("depth_frustum", baseline["robots"][2]["mode"]);
  for (const char* backend : {"cpu", "gpu", "auto"}) {
    SCOPED_TRACE(backend);
    auto input = original;
    input.facts["scene"]["parameters"]["pointCloudBackend"] = backend;
    const auto before = input.facts;
    auto actual = buildFleetJson(input, true);
    const auto fleet = buildFleet(input, {}, true);
    ASSERT_EQ(3u, fleet.robots.size());
    for (std::size_t i = 0; i < fleet.robots.size(); ++i) {
      EXPECT_EQ(backend, fleet.robots[i].backend);
      EXPECT_EQ(backend, actual["robots"][i]["backend"]);
      actual["robots"][i]["backend"] = "auto";
    }
    EXPECT_EQ(baseline, actual);
    EXPECT_EQ(before, input.facts);
  }
}

TEST(Roster, SavedXsimCloudExplicitRobotBackendOverridesSceneDefault) {
  auto input = parseSessionInput(R"json({"runMode":"simulation","scene":{"simulator":"xsim"},
    "robots":[
      {"namespace":"/cpu","binding":{"simulationSensors":{"simpleLidar":{"enabled":true,"acceleration":"cpu"}}}},
      {"namespace":"/gpu","binding":{"simulationSensors":{"simpleLidar":{"enabled":true,"acceleration":"gpu"}}}},
      {"namespace":"/auto","binding":{"simulationSensors":{"simpleLidar":{"enabled":true,"acceleration":"auto"}}}}
    ],"visualizationTopics":[]})json");
  for (const char* backend : {"cpu", "gpu", "auto"}) {
    SCOPED_TRACE(backend);
    input.facts["scene"]["parameters"]["pointCloudBackend"] = backend;
    const auto fleet = buildFleet(input, {}, true);
    ASSERT_EQ(3u, fleet.robots.size());
    EXPECT_EQ("cpu", fleet.robots[0].backend);
    EXPECT_EQ("gpu", fleet.robots[1].backend);
    EXPECT_EQ("auto", fleet.robots[2].backend);
  }
}

TEST(Roster, SavedXsimCloudRejectsMalformedSceneBackendBeforeSensorFiltering) {
  const auto original = parseSessionInput(R"json({"runMode":"simulation","scene":{"simulator":"xsim"},
    "robots":[],"visualizationTopics":[]})json");
  const auto rejects = [](const SessionInput& input, const char* field) {
    try {
      buildFleetJson(input, true);
      FAIL() << "accepted invalid " << field;
    } catch (const SceneError& error) {
      EXPECT_NE(std::string::npos, std::string(error.what()).find(field));
    }
  };
  for (const auto& value : {Json(nullptr), Json(true), Json(1), Json("xsim"), Json::array()}) {
    auto input = original;
    input.facts["scene"] = value;
    rejects(input, "scene");
  }
  for (const auto& value : {Json(nullptr), Json(true), Json(1), Json::object(), Json::array()}) {
    auto input = original;
    input.facts["scene"]["simulator"] = value;
    rejects(input, "scene.simulator");
  }
  for (const auto& value : {Json(nullptr), Json(true), Json(1), Json("parameters"), Json::array()}) {
    auto input = original;
    input.facts["scene"]["parameters"] = value;
    rejects(input, "scene.parameters");
  }
  for (const auto& value : {Json(nullptr), Json(true), Json(1), Json::object(), Json::array(), Json(""), Json("cuda")}) {
    auto input = original;
    input.facts["scene"]["parameters"]["pointCloudBackend"] = value;
    rejects(input, "scene.parameters.pointCloudBackend");
  }
}

TEST(Roster, SceneBackendDoesNotChangeGeometryOrGazeboFleet) {
  auto input = parseSessionInput(R"json({"runMode":"simulation","scene":{"simulator":"xsim"},
    "robots":[{"namespace":"/robot","binding":{"simulationSensors":{"simpleLidar":{"enabled":true,"mode":"penetrating"}}}}],
    "visualizationTopics":[]})json");
  const auto geometry = buildFleetJson(input, false);
  input.facts["scene"]["simulator"] = "gazebo";
  const auto gazebo = buildFleetJson(input, true);
  for (const auto& backend : {Json("cpu"), Json("gpu"), Json("auto"), Json("invalid"), Json(nullptr)}) {
    input.facts["scene"]["parameters"]["pointCloudBackend"] = backend;
    EXPECT_EQ(gazebo, buildFleetJson(input, true));
    input.facts["scene"]["simulator"] = "xsim";
    EXPECT_EQ(geometry, buildFleetJson(input, false));
    input.facts["scene"]["simulator"] = "gazebo";
  }
}

TEST(Roster, CropFloatRoundTripsExactlyLikePython) {
  const auto input = parseSessionInput(R"json({"runMode":"simulation","robots":[
    {"namespace":"/crop","binding":{"simulationSensors":{"simpleLidar":{
      "enabled":true,"preset":"zju_cpu_crop","acceleration":"cpu"}}}}
    ],"visualizationTopics":[]})json");
  const auto manifest = buildFleetJson(input);
  // Python's 1 / math.sqrt(3), not the one-ULP-lower Math default tan(pi/6).
  constexpr double python_slab = 0.5773502691896258;
  EXPECT_EQ(python_slab,
            manifest["robots"][0]["verticalSlabTan"].get<double>());
  const auto decoded = Json::parse(manifest.dump());
  EXPECT_EQ(python_slab, decoded["robots"][0]["verticalSlabTan"].get<double>());
  const auto fleet = buildFleet(input);
  ASSERT_EQ(1u, fleet.robots.size());
  EXPECT_EQ(python_slab, fleet.robots[0].config.vertical_slab_tan);
  EXPECT_EQ(0.5, fleet.robots[0].config.heading_cos_min);
  EXPECT_EQ(5.0, fleet.robots[0].config.range);
}

TEST(Roster, EmptyFleetAndReferenceOnly) {
  const auto empty = parseSessionInput(
      R"({"runMode":"simulation","robots":[],"visualizationTopics":[]})");
  EXPECT_THROW(parseManifest(buildFleetJson(empty).dump()),
               std::invalid_argument);
  const auto fleet = buildFleet(empty);
  EXPECT_TRUE(fleet.robots.empty());
  EXPECT_TRUE(fleet.bodies.empty());
  EXPECT_FALSE(fleet.body_roster);
  EXPECT_FALSE(fleet.reference);
  const auto reference = buildFleet(parseSessionInput(R"json({"runMode":"hybrid","robots":[],"visualizationTopics":[{"topic":"/xgc/scene/reference_cloud","messageType":"sensor_msgs/PointCloud2","role":"semantic","visible3d":true}]})json"));
  EXPECT_TRUE(reference.robots.empty());
  ASSERT_TRUE(reference.reference);
  EXPECT_EQ(0.1, reference.reference->surface_spacing);
}

TEST(Roster, FullSampledFleetAtRobotAndNamespaceLimits) {
  auto input = parseSessionInput(
      R"({"runMode":"simulation","robots":[],"visualizationTopics":[]})");
  for (std::size_t i = 0; i < kMaxFleetMembers; ++i) {
    const auto suffix = std::to_string(i);
    input.robots.push_back(
        {{"namespace",
          "/" + std::string(158 - suffix.size(), 'r') + "_" + suffix},
         {"binding",
          {{"simulationSensors",
            {{"simpleLidar",
              {{"enabled", true},
               {"mode", "penetrating"},
               {"acceleration", "cpu"},
               {"rateHz", 100},
               {"rangeMeters", 200},
               {"hFovDeg", 360},
               {"vFovDeg", 180},
               {"hRes", 4096},
               {"vRes", 4096}}}}}}}});
  }
  const auto manifest = buildFleetJson(input);
  EXPECT_GT(manifest.dump().size(), 131072u);
  EXPECT_LE(manifest.dump().size(), kMaxManifestBytes);
  const auto fleet = buildFleet(input);
  ASSERT_EQ(kMaxFleetMembers, fleet.robots.size());
  EXPECT_EQ(160u, fleet.robots.back().name_space.size());
  EXPECT_EQ(100.0, fleet.robots.back().rate_hz);
  EXPECT_EQ(200.0, fleet.robots.back().config.range);
  const auto extra = input.robots.back();
  input.robots.push_back(extra);
  try {
    buildFleet(input);
    FAIL() << "accepted 513 sampled robots";
  } catch (const SceneError &error) {
    EXPECT_STREQ("world lidar supports at most 512 robots", error.what());
  }
}
