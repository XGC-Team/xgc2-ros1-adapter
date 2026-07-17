#pragma once

#include <cstdint>
#include <string>

#include "xgc/adapter/v1/adapter.pb.h"
#include "xgc/v1/message.pb.h"
#include "xgc2/adapter_runtime/client.hpp"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"

namespace xgc2_ros1_robot_adapter {

constexpr const char *kTelemetryCapability = "xgc.robot.telemetry";
constexpr const char *kCommandCapability = "xgc.robot.command";
constexpr std::uint32_t kRobotCapabilityVersion = 1u;

// The Process Supervisor owns this argument. Robot applications deliberately
// have no socket/token/identity flags or ROS-parameter fallbacks.
bool BootstrapFileFromArguments(int argc, char **argv, std::string *path,
                                std::string *error);

// Attaches callbacks only to the exact contract delivered in the trusted
// bootstrap. Product code cannot manufacture or widen a contract digest.
bool BindBootstrapCapability(
    xgc2::adapter_runtime::ClientConfig *config,
    const std::string &capability_id,
    xgc2::adapter_runtime::CapabilityCallbacks callbacks,
    std::string *error);

// Validates the robot-domain subject and fences it to the applied group scope.
// The Host has already checked canonical key construction; this function owns
// the domain meaning of target-id/run-id/robot-id.
bool ResolveRobotSubject(
    const xgc::adapter::v1::WorkContext &context,
    const RobotAdapterConfig &configuration, std::string *robot_id,
    std::string *error);

// Successful command operations carry the registry-owned xgc.v1.Empty
// response required by their capability contract.
xgc2::adapter_runtime::OperationResult
EmptyOperationSuccess(std::uint32_t schema_version,
                      std::uint64_t schema_fingerprint);

} // namespace xgc2_ros1_robot_adapter
