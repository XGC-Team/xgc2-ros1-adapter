#pragma once
#include "xgc/robot/v1/server.pb.h"
#include "xgc2/xrpc/grpc.hpp"
#include "xgc2_ros1_robot_adapter/robot_server_entry.hpp"

namespace xgc2_ros1_robot_adapter {
xgc2::xrpc::GrpcLimits ProductGrpcLimits();
xgc::robot::v1::RobotServerBootstrap
ReadRobotServerBootstrap(const RobotServerArguments &,
                         const std::string &provider);
} // namespace xgc2_ros1_robot_adapter
