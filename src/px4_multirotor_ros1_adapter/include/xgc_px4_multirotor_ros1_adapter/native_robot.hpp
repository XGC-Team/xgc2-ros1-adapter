#pragma once

#include "xgc2_ros1_robot_adapter/ros1_binding.hpp"
#include <memory>
#include <string>

namespace xgc_px4_multirotor_ros1_adapter {

std::shared_ptr<xgc2_ros1_robot_adapter::NativeRobot> CreatePx4NativeRobot(
    xgc2_ros1_robot_adapter::RosNode &ros_node,
    const xgc2_ros1_robot_adapter::RobotConfig &config,
    xgc2_ros1_robot_adapter::TelemetryEmitter emit,
    std::string *error);

} // namespace xgc_px4_multirotor_ros1_adapter
