#include "xgc_ros1_adapter/robot_behavior.hpp"

namespace xgc_ros1_adapter {
namespace {

void appendPx4RotorRegistration(const RobotRuntime& robot, xgc::adapter::v1::RobotMapping* out) {
  commonAppendRegistration(robot, out);
  auto& services = *out->mutable_services();
  services["arm"].set_service(robot.arm_service);
  services["arm"].set_service_type("mavros_msgs/CommandBool");
  services["mode"].set_service(robot.mode_service);
  services["mode"].set_service_type("mavros_msgs/SetMode");
  services["command"].set_service(robot.command_service);
  services["command"].set_service_type("mavros_msgs/CommandLong");
  services["takeoff"].set_service(robot.takeoff_service);
  services["takeoff"].set_service_type("mavros_msgs/CommandTOL");
  services["land"].set_service(robot.land_service);
  services["land"].set_service_type("mavros_msgs/CommandTOL");
}

}  // namespace

const RobotBehaviorSpec& px4RotorBehavior() {
  static const RobotBehaviorSpec behavior{
      "px4_rotor",
      {RobotFeature::MavrosServices, RobotFeature::MavrosTelemetry},
      commonSetDefaultEndpoints,
      commonApplyMapping,
      appendPx4RotorRegistration,
      commonShouldPublishState,
      commonAddState,
  };
  return behavior;
}

}  // namespace xgc_ros1_adapter
