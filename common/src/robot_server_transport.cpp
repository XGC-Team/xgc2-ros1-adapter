#include "robot_server_transport.hpp"
#include "xgc/robot/v1/server.grpc.pb.h"
#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
namespace xgc2_ros1_robot_adapter {
namespace operation = xgc::adapter::v1;
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
  if (!arguments.bootstrap_file.empty()) {
    const int file = open(arguments.bootstrap_file.c_str(),
                          O_RDONLY | O_NOFOLLOW | O_CLOEXEC | O_NONBLOCK);
    if (file < 0)
      throw std::runtime_error("cannot open private robot bootstrap");
    try {
      struct stat info {};
      if (fstat(file, &info) || !S_ISREG(info.st_mode) ||
          info.st_uid != geteuid() || (info.st_mode & 0777) != 0600 ||
          info.st_size <= 0 || info.st_size > 65536)
        throw std::runtime_error(
            "robot bootstrap must be an owned bounded mode-0600 regular file");
      std::string bytes(static_cast<std::size_t>(info.st_size), '\0');
      std::size_t position = 0;
      while (position < bytes.size()) {
        const auto amount =
            read(file, &bytes[position], bytes.size() - position);
        if (amount < 0 && errno == EINTR)
          continue;
        if (amount <= 0)
          throw std::runtime_error("robot bootstrap changed during read");
        position += static_cast<std::size_t>(amount);
      }
      char extra;
      if (read(file, &extra, 1) != 0 || !bootstrap.ParseFromString(bytes))
        throw std::runtime_error("invalid or changing robot bootstrap");
      close(file);
    } catch (...) {
      close(file);
      throw;
    }
  } else {
    bootstrap.set_socket_path(arguments.socket_path);
    bootstrap.set_provider_definition_id(arguments.provider);
    bootstrap.set_target_id(arguments.target_id);
    if (!arguments.ros_master_uri.empty())
      (*bootstrap.mutable_ros_environment())["ros_master_uri"] =
          arguments.ros_master_uri;
    if (!arguments.ros_ip.empty())
      (*bootstrap.mutable_ros_environment())["ros_ip"] = arguments.ros_ip;
  }
  ValidateRobotSocketPath(bootstrap.socket_path());
  if (bootstrap.provider_definition_id() != provider ||
      !ValidRuntimeReferenceId(bootstrap.target_id()))
    throw std::runtime_error(
        "robot bootstrap requires the exact provider and target identity");
  for (const auto &environment : bootstrap.ros_environment())
    if ((environment.first != "ros_master_uri" &&
         environment.first != "ros_ip") ||
        environment.second.size() > 2048 ||
        environment.second.find('\0') != std::string::npos)
      throw std::runtime_error("invalid robot bootstrap ROS environment");
  return bootstrap;
}
int CheckRobotServer(const RobotServerArguments &arguments,
                     const std::string &provider) {
  namespace wire = xgc::robot::v1;
  const auto limits = ProductGrpcLimits();
  const auto budget = std::min(std::chrono::milliseconds(arguments.timeout_ms),
                               limits.call_timeout);
  const auto deadline = xgc2::xrpc::grpc_stream_deadline(
      limits, xgc2::xrpc::GrpcClock::now() + budget);
  auto channel =
      xgc2::xrpc::make_grpc_unix_channel(arguments.socket_path, limits);
  auto stub = wire::RobotAdapterServerService::NewStub(std::move(channel));
  grpc::ClientContext describe_context;
  describe_context.set_wait_for_ready(true);
  xgc2::xrpc::GrpcClientCall discover(describe_context, {}, deadline, {}, {},
                                      true);
  wire::DescribeRequest describe_request;
  wire::DescribeResponse describe_response;
  auto status = discover.invoke([&] {
    return stub->Describe(&describe_context, describe_request,
                          &describe_response);
  });
  if (!status.ok()) {
    std::cerr << "robot server discovery: " << status.error_message() << '\n';
    return 1;
  }
  const auto &reference = describe_response.service_ref();
  if (reference.target_id() != arguments.target_id ||
      reference.service() != "xgc2.robot-adapter" ||
      reference.api_version() != "v1" || reference.profile() != "grpc.v1" ||
      reference.endpoint().kind() != "unix" ||
      reference.endpoint().address() != arguments.socket_path ||
      !ValidRuntimeReferenceId(reference.instance_id()) ||
      reference.instance_id() != discover.response_instance_id() ||
      describe_response.provider_definition_id() != provider) {
    std::cerr << "robot server discovery: complete service reference or "
                 "provider mismatch\n";
    return 1;
  }
  grpc::ClientContext health_context;
  health_context.set_wait_for_ready(true);
  xgc2::xrpc::GrpcClientCall health(health_context, reference.instance_id(),
                                    deadline);
  wire::HealthRequest health_request;
  wire::HealthResponse health_response;
  status = health.invoke([&] {
    return stub->Health(&health_context, health_request, &health_response);
  });
  if (!status.ok() || !health_response.serving() ||
      health_response.instance_id() != reference.instance_id() ||
      health.response_instance_id() != reference.instance_id()) {
    std::cerr << "robot server health: "
              << (status.ok() ? "not serving or instance mismatch"
                              : status.error_message())
              << '\n';
    return 1;
  }
  return 0;
}
} // namespace xgc2_ros1_robot_adapter
