#include "xgc_ros1_adapter/robot_behavior.hpp"

namespace xgc_ros1_adapter {
namespace {

void appendFixedWingRegistration(const RobotRuntime& robot, xgc::adapter::v1::RobotMapping* out) {
  commonAppendRegistration(robot, out);
  (*out->mutable_topics())["global_position"] = robot.global_position_topic;
  (*out->mutable_topics())["stage"] = robot.stage_topic;
  (*out->mutable_topics())["angle"] = robot.angle_topic;
  (*out->mutable_topics())["ias"] = robot.ias_topic;
  (*out->mutable_topics())["pos"] = robot.pos_topic;
  (*out->mutable_topics())["baro"] = robot.baro_topic;
}

}  // namespace

const RobotBehaviorSpec& fixedWingBehavior() {
  static const RobotBehaviorSpec behavior{
      "fw_plane",
      {RobotFeature::PlainTwist, RobotFeature::FixedWingTelemetry},
      commonSetDefaultEndpoints,
      commonApplyMapping,
      appendFixedWingRegistration,
      commonShouldPublishState,
      commonAddState,
  };
  return behavior;
}

}  // namespace xgc_ros1_adapter
