#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "xgc/adapter/v1/adapter.pb.h"
#include "xgc/v1/message.pb.h"
#include "xgc2/adapter_runtime/client.hpp"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"

namespace xgc2_ros1_robot_adapter {

constexpr const char *kTelemetryCapability = "xgc.robot.telemetry";
constexpr const char *kCommandCapability = "xgc.robot.command";
constexpr std::uint32_t kRobotCapabilityVersion = 1u;

// Called once by main before constructing the reverse-link client.
std::vector<std::pair<std::string, std::string>> XrpcEnvironmentAtStartup();

// The Process Supervisor owns this argument. Robot applications deliberately
// have no socket/token/identity flags or ROS-parameter fallbacks.
bool BootstrapFileFromArguments(int argc, char **argv, std::string *path,
                                std::string *error);

// Read the experiment ROS endpoint before ros::init creates any ROS clients.
std::map<std::string, std::string>
RosEnvironmentFromSpec(const xgc::adapter::v1::AdapterInstanceSpec &spec);

// Attaches callbacks only to the exact contract delivered in the trusted
// bootstrap. Product code cannot manufacture or widen a contract digest.
bool BindBootstrapCapability(
    xgc2::adapter_runtime::ClientConfig *config,
    const std::string &capability_id,
    xgc2::adapter_runtime::CapabilityCallbacks callbacks, std::string *error);

// Validates the robot-domain subject and fences it to the applied group scope.
// The Host has already checked canonical key construction; this function owns
// the domain meaning of target-id/run-id/robot-id.
bool ResolveRobotSubject(const xgc::adapter::v1::WorkContext &context,
                         const RobotAdapterConfig &configuration,
                         std::string *robot_id, std::string *error);

// Successful command operations carry the registry-owned xgc.v1.Empty
// response required by their capability contract.
xgc2::adapter_runtime::OperationResult
EmptyOperationSuccess(std::uint32_t schema_version,
                      std::uint64_t schema_fingerprint);

} // namespace xgc2_ros1_robot_adapter
