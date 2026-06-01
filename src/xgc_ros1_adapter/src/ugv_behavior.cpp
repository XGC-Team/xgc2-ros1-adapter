#include "xgc_ros1_adapter/robot_behavior.hpp"

namespace xgc_ros1_adapter {

const RobotBehaviorSpec& ugvBehavior() {
  static const RobotBehaviorSpec behavior{
      "ugv",
      {},
      commonSetDefaultEndpoints,
      commonApplyMapping,
      commonAppendRegistration,
      commonShouldPublishState,
      commonAddState,
  };
  return behavior;
}

}  // namespace xgc_ros1_adapter
