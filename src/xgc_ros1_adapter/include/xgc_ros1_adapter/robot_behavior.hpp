#pragma once

#include <string>
#include <vector>

#include "xgc_ros1_adapter/robot_runtime.hpp"

namespace xgc_ros1_adapter {

enum class RobotFeature {
  TopicSignal,
  MavrosServices,
  MavrosTelemetry,
  PlainTwist,
  FixedWingTelemetry,
};

struct RobotBehaviorSpec {
  const char* name = "ugv";
  std::vector<RobotFeature> features;
  void (*set_default_endpoints)(RobotRuntime& robot) = nullptr;
  void (*apply_mapping)(const xgc::adapter::v1::RobotMapping& mapping, RobotRuntime& robot) = nullptr;
  void (*append_registration)(const RobotRuntime& robot, xgc::adapter::v1::RobotMapping* out) = nullptr;
  bool (*should_publish_state)(const RobotRuntime& robot) = nullptr;
  void (*add_state)(xgc::adapter::v1::PushRobotStateRequest& request, const RobotRuntime& robot) = nullptr;
};

bool hasFeature(const RobotBehaviorSpec& behavior, RobotFeature feature);

void commonSetDefaultEndpoints(RobotRuntime& robot);
void commonApplyMapping(const xgc::adapter::v1::RobotMapping& mapping, RobotRuntime& robot);
void commonAppendRegistration(const RobotRuntime& robot, xgc::adapter::v1::RobotMapping* out);
bool commonShouldPublishState(const RobotRuntime& robot);
void commonAddState(xgc::adapter::v1::PushRobotStateRequest& request, const RobotRuntime& robot);

const RobotBehaviorSpec& behaviorForType(const std::string& robot_type);
const RobotBehaviorSpec& ugvBehavior();
const RobotBehaviorSpec& telloBehavior();
const RobotBehaviorSpec& px4RotorBehavior();
const RobotBehaviorSpec& fixedWingBehavior();

}  // namespace xgc_ros1_adapter
