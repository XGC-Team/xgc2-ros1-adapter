#include "xgc_ros1_adapter/robot_behavior.hpp"

namespace xgc_ros1_adapter {
namespace {

void appendTelloRegistration(const RobotRuntime& robot, xgc::adapter::v1::RobotMapping* out) {
  commonAppendRegistration(robot, out);
  (*out->mutable_topics())["takeoff"] = robot.takeoff_topic;
  (*out->mutable_topics())["land"] = robot.land_topic;
}

}  // namespace

const RobotBehaviorSpec& telloBehavior() {
  static const RobotBehaviorSpec behavior{
      "tello",
      {RobotFeature::TopicSignal},
      commonSetDefaultEndpoints,
      commonApplyMapping,
      appendTelloRegistration,
      commonShouldPublishState,
      commonAddState,
  };
  return behavior;
}

}  // namespace xgc_ros1_adapter
