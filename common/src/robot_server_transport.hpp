#pragma once
#include "xgc/robot/v1/server.pb.h"
#include "xgc2/xrpc/grpc.hpp"
#include "xgc2/xrpc/runtime_policy.hpp"
#include "xgc2_ros1_robot_adapter/robot_server_entry.hpp"

namespace xgc2_ros1_robot_adapter {
xgc2::xrpc::RuntimePolicy StartupPolicy();
xgc2::xrpc::GrpcLimits ProductGrpcLimits(const xgc2::xrpc::RuntimePolicy &);
xgc::adapter::v1::RuntimePolicySnapshot
PolicyObservation(const xgc2::xrpc::RuntimePolicy &);
xgc::robot::v1::RobotServerBootstrap
ReadRobotServerBootstrap(const RobotServerArguments &,
                         const std::string &provider);
int CheckRobotServer(const RobotServerArguments &, const std::string &provider,
                     const xgc2::xrpc::RuntimePolicy &);
} // namespace xgc2_ros1_robot_adapter
