#include "robot_server_transport.hpp"
#include "xgc/robot/v1/server.grpc.pb.h"
namespace xgc2_ros1_robot_adapter {
xgc2::xrpc::GrpcLimits ProductGrpcLimits() {
  xgc2::xrpc::GrpcLimits limits;
  limits.connections = 32;
  limits.inflight = 32;
  limits.streams_per_connection = 32;
  limits.request_bytes = 1048576;
  limits.response_bytes = 1048576;
  limits.call_timeout = std::chrono::milliseconds(30000);
  // A fixed native pool serves the whole type host. The bounded domain waits
  // support a complete sixteen-robot fleet while leaving native control room.
  limits.native_threads = 32;
  limits.native_memory_bytes = 32 * 1048576;
  return limits;
}

xgc::robot::v1::RobotServerBootstrap
ReadRobotServerBootstrap(const RobotServerArguments &arguments,
                         const std::string &provider) {
  xgc::robot::v1::RobotServerBootstrap bootstrap;
  bootstrap.set_socket_path(arguments.socket_path);
  bootstrap.set_provider_definition_id(arguments.provider);
  bootstrap.set_target_id(arguments.target_id);
  if (!arguments.ros_master_uri.empty())
    (*bootstrap.mutable_ros_environment())["ros_master_uri"] =
        arguments.ros_master_uri;
  if (!arguments.ros_ip.empty())
    (*bootstrap.mutable_ros_environment())["ros_ip"] = arguments.ros_ip;
  ValidateRobotSocketPath(bootstrap.socket_path());
  if (bootstrap.provider_definition_id() != provider ||
      !ValidRuntimeReferenceId(bootstrap.target_id()))
    throw std::runtime_error(
        "robot bootstrap requires the exact provider and target identity");
  for (const auto &environment : bootstrap.ros_environment())
    if (environment.second.size() > 2048 ||
        environment.second.find('\0') != std::string::npos)
      throw std::runtime_error("invalid robot bootstrap ROS environment");
  return bootstrap;
}
} // namespace xgc2_ros1_robot_adapter
