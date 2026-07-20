#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ros/master.h>
#include <ros/ros.h>

#include "xgc/semantic/aerial/v1/control.pb.h"
#include "xgc/v1/message.pb.h"
#include "xgc2/adapter_runtime/client.hpp"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"
#include "xgc2_ros1_robot_adapter/runtime_support.hpp"
#include "xgc_px4_multirotor_ros1_adapter/generated_contract.hpp"
#include "xgc_px4_multirotor_ros1_adapter/px4_operations.hpp"
#include "xgc_px4_multirotor_ros1_adapter/robot_runtime.hpp"
#include "xgc_px4_multirotor_ros1_adapter/shutdown_signal.hpp"
#include "xgc_px4_multirotor_ros1_adapter/telemetry_batch.hpp"

namespace xgc_px4_multirotor_ros1_adapter {
namespace {

constexpr const char *kTelemetryEndpoint = "telemetry";
constexpr std::size_t kMaximumQueuedTelemetryPerRobot = 2048u;
constexpr std::size_t kMaximumQueuedTelemetryBytesPerRobot =
    16u * 1024u * 1024u;
constexpr std::size_t kMaximumQueuedTelemetry = 8192u;
constexpr std::size_t kMaximumQueuedTelemetryBytes = 64u * 1024u * 1024u;
constexpr std::size_t kMaximumPublishAttemptsPerTick = 256u;

template <typename Function> class ScopeExit {
public:
  explicit ScopeExit(Function function) : function_(std::move(function)) {}
  ~ScopeExit() noexcept {
    if (!active_)
      return;
    try {
      function_();
    } catch (...) {
    }
  }

  void release() noexcept { active_ = false; }
  ScopeExit(const ScopeExit &) = delete;
  ScopeExit &operator=(const ScopeExit &) = delete;
  ScopeExit(ScopeExit &&other) noexcept
      : function_(std::move(other.function_)), active_(other.active_) {
    other.active_ = false;
  }
  ScopeExit &operator=(ScopeExit &&) = delete;

private:
  Function function_;
  bool active_ = true;
};

template <typename Function>
ScopeExit<Function> makeScopeExit(Function function) {
  return ScopeExit<Function>(std::move(function));
}

void logFromClient(xgc2::adapter_runtime::LogLevel level,
                   const std::string &message) {
  switch (level) {
  case xgc2::adapter_runtime::LogLevel::kDebug:
    ROS_DEBUG_STREAM(message);
    break;
  case xgc2::adapter_runtime::LogLevel::kInfo:
    ROS_INFO_STREAM(message);
    break;
  case xgc2::adapter_runtime::LogLevel::kWarning:
    ROS_WARN_STREAM(message);
    break;
  case xgc2::adapter_runtime::LogLevel::kError:
    ROS_ERROR_STREAM(message);
    break;
  }
}

xgc2::adapter_runtime::OperationResult rejected(const std::string &code,
                                                const std::string &message) {
  return xgc2::adapter_runtime::OperationResult::Rejected(code, message);
}

xgc2::adapter_runtime::SourceOpenDecision
rejectedSource(xgc::adapter::v1::ErrorClass error_class,
               const std::string &code, const std::string &message) {
  return xgc2::adapter_runtime::SourceOpenDecision::Reject(error_class, code,
                                                           message);
}

struct ResolvedOperation {
  contract::ChannelMetadata channel{};
  contract::MessageMetadata input_schema{};
  contract::MessageMetadata output_schema{};
};

xgc2::adapter_runtime::OperationResult
succeeded(const ResolvedOperation &operation) {
  xgc::v1::Empty empty;
  if (operation.channel.output_message_id == 0u ||
      operation.output_schema.version == 0u ||
      operation.output_schema.fingerprint == 0u ||
      operation.output_schema.type_name != empty.GetDescriptor()->full_name()) {
    return xgc2::adapter_runtime::OperationResult::Failure(
        xgc::adapter::v1::ERROR_CLASS_PERMANENT, "empty-schema-unavailable",
        "installed operation output schema is not xgc.v1.Empty");
  }
  xgc::v1::Payload output;
  output.mutable_schema()->set_message_id(operation.channel.output_message_id);
  output.mutable_schema()->set_type_name(operation.output_schema.type_name);
  output.mutable_schema()->set_schema_version(operation.output_schema.version);
  output.mutable_schema()->set_schema_fingerprint(
      operation.output_schema.fingerprint);
  output.set_encoding(xgc::v1::PAYLOAD_ENCODING_PROTOBUF);
  output.set_value(empty.SerializeAsString());
  return xgc2::adapter_runtime::OperationResult::Success(std::move(output),
                                                         true);
}

xgc2::adapter_runtime::OperationResult
mapNativeResult(const OperationResult &result,
                const ResolvedOperation &operation) {
  if (result.succeeded())
    return succeeded(operation);

  xgc::adapter::v1::ErrorClass error_class =
      xgc::adapter::v1::ERROR_CLASS_PERMANENT;
  const char *code = "px4-operation-failed";
  switch (result.outcome) {
  case OperationOutcome::kSucceeded:
    break;
  case OperationOutcome::kInvalidArgument:
    error_class = xgc::adapter::v1::ERROR_CLASS_REJECTED;
    code = "invalid-argument";
    break;
  case OperationOutcome::kNotReady:
    error_class = xgc::adapter::v1::ERROR_CLASS_TRANSIENT;
    code = "px4-not-ready";
    break;
  case OperationOutcome::kRejected:
    error_class = xgc::adapter::v1::ERROR_CLASS_REJECTED;
    code = "px4-rejected";
    break;
  case OperationOutcome::kUnsupported:
    error_class = xgc::adapter::v1::ERROR_CLASS_REJECTED;
    code = "px4-unsupported";
    break;
  case OperationOutcome::kUncertain:
    error_class = xgc::adapter::v1::ERROR_CLASS_UNCERTAIN;
    code = "px4-result-uncertain";
    break;
  case OperationOutcome::kTransportError:
    error_class = xgc::adapter::v1::ERROR_CLASS_TRANSIENT;
    code = "ros-transport-error";
    break;
  case OperationOutcome::kTimedOut:
    error_class = xgc::adapter::v1::ERROR_CLASS_DEADLINE;
    code = "px4-deadline-exceeded";
    break;
  }
  return xgc2::adapter_runtime::OperationResult::Failure(
      error_class, code, result.detail,
      result.has_native_result ? static_cast<std::int32_t>(result.native_result)
                               : 0);
}

bool resolveOperation(const std::string &profile_id,
                      const std::string &operation_id,
                      ResolvedOperation *output, bool *known,
                      std::string *error) {
  if (output == nullptr || known == nullptr || error == nullptr)
    return false;
  *known = false;
  *output = {};

  std::size_t channel_count = 0u;
  const contract::ChannelMetadata *channels =
      contract::profileChannels(profile_id, &channel_count);
  if (channels == nullptr || channel_count == 0u) {
    *known = true;
    *error = "installed operation profile is unavailable";
    return false;
  }
  for (std::size_t index = 0u; index < channel_count; ++index) {
    const contract::ChannelMetadata &candidate = channels[index];
    if (candidate.kind != contract::ChannelKind::kOperation)
      continue;
    if (candidate.operation_id == nullptr ||
        candidate.operation_id[0] == '\0') {
      *known = true;
      *error = "installed profile contains an operation without operation_id";
      return false;
    }
    if (operation_id != candidate.operation_id)
      continue;
    if (*known) {
      *error = "installed profile contains a duplicate operation_id";
      return false;
    }
    *known = true;
    output->channel = candidate;
  }
  if (!*known) {
    *error = "unsupported PX4 command endpoint";
    return false;
  }

  const auto &channel = output->channel;
  if (channel.operation_id == nullptr || channel.operation_id[0] == '\0' ||
      channel.channel_id == nullptr || channel.channel_id[0] == '\0' ||
      channel.processor == nullptr || channel.processor[0] == '\0' ||
      channel.input_message_id == 0u || channel.output_message_id == 0u ||
      channel.operation_timeout_millis == 0u ||
      channel.operation_contract.side_effect == nullptr ||
      channel.operation_contract.side_effect[0] == '\0' ||
      channel.operation_contract.idempotency == nullptr ||
      std::string(channel.operation_contract.idempotency) != "required" ||
      channel.operation_contract.cancellation_supported ||
      !channel.operation_contract.deadline_required ||
      !contract::messageMetadata(channel.input_message_id,
                                 &output->input_schema) ||
      !contract::messageMetadata(channel.output_message_id,
                                 &output->output_schema) ||
      output->input_schema.type_name == nullptr ||
      output->input_schema.type_name[0] == '\0' ||
      output->input_schema.version == 0u ||
      output->input_schema.fingerprint == 0u ||
      output->output_schema.type_name == nullptr ||
      output->output_schema.type_name[0] == '\0' ||
      output->output_schema.version == 0u ||
      output->output_schema.fingerprint == 0u) {
    *error = "installed operation descriptor is incomplete or unsafe";
    return false;
  }
  xgc::v1::Empty empty;
  if (output->output_schema.type_name != empty.GetDescriptor()->full_name()) {
    *error = "installed operation output schema is not xgc.v1.Empty";
    return false;
  }
  return true;
}

bool inputSchemaMatches(const xgc::v1::Payload &input,
                        const ResolvedOperation &operation) {
  if (!input.has_schema())
    return false;
  const auto &schema = input.schema();
  return schema.message_id() == operation.channel.input_message_id &&
         schema.type_name() == operation.input_schema.type_name &&
         schema.schema_version() == operation.input_schema.version &&
         schema.schema_fingerprint() == operation.input_schema.fingerprint;
}

bool operationTiming(const xgc::adapter::v1::WorkContext &context,
                     const ResolvedOperation &operation,
                     OperationTiming *output, std::string *error) {
  if (output == nullptr || error == nullptr)
    return false;
  const auto &metadata = operation.channel;
  if (metadata.kind != contract::ChannelKind::kOperation ||
      metadata.operation_timeout_millis == 0u ||
      (metadata.operation_contract.deadline_required &&
       context.deadline().deadline_unix_nanos() <= 0)) {
    *error = "installed operation timing contract is invalid";
    return false;
  }
  *output = OperationTiming(
      static_cast<double>(metadata.operation_timeout_millis) / 1000.0,
      operationDeadlineFromUnixNanos(context.deadline().deadline_unix_nanos()));
  return true;
}

bool channelEnabled(const xgc2_ros1_robot_adapter::RobotConfig &robot,
                    const std::string &channel_id) {
  for (const auto &channel : robot.channels) {
    if (channel.channel_id == channel_id)
      return channel.enabled;
  }
  return false;
}

} // namespace

class Px4MultirotorRos1AdapterNode {
public:
  Px4MultirotorRos1AdapterNode(ros::NodeHandle node_handle,
                               const std::string &bootstrap_file)
      : node_handle_(std::move(node_handle)) {
    auto config =
        xgc2::adapter_runtime::ClientConfig::FromBootstrapFile(bootstrap_file);
    definition_id_ = config.registration().definition_id();
    config.dispatch_workers = 4;

    xgc2::adapter_runtime::CapabilityCallbacks telemetry;
    telemetry.source_open =
        [this](const xgc::adapter::v1::SourceOpenRequest &request,
               const xgc2::adapter_runtime::CancellationToken &cancellation) {
          return handleSourceOpen(request, cancellation);
        };
    telemetry.source_closed =
        [this](const xgc::adapter::v1::SourceOpenRequest &request,
               const xgc::adapter::v1::AdapterError &error) {
          handleSourceClosed(request, error);
        };

    xgc2::adapter_runtime::CapabilityCallbacks command;
    command.operation =
        [this](const xgc::adapter::v1::OperationRequest &request,
               const xgc2::adapter_runtime::CancellationToken &cancellation) {
          return handleCommand(request, cancellation);
        };

    std::string error;
    if (!xgc2_ros1_robot_adapter::BindBootstrapCapability(
            &config, xgc2_ros1_robot_adapter::kTelemetryCapability,
            std::move(telemetry), &error) ||
        !xgc2_ros1_robot_adapter::BindBootstrapCapability(
            &config, xgc2_ros1_robot_adapter::kCommandCapability,
            std::move(command), &error)) {
      throw std::runtime_error("cannot bind trusted robot capability: " +
                               error);
    }

    xgc2::adapter_runtime::ClientCallbacks callbacks;
    callbacks.apply_instance_spec =
        [this](const xgc::adapter::v1::AdapterInstanceSpec &spec,
               std::string *apply_error) {
          return applyInstanceSpec(spec, apply_error);
        };
    callbacks.clear_instance_spec = [this] { clearInstanceSpec(); };
    callbacks.stop_requested =
        [this](const xgc::adapter::v1::StopRequest &request) {
          ROS_INFO_STREAM("PX4 Adapter Runtime requested process stop: "
                          << request.reason());
          exit_requested_.store(true, std::memory_order_release);
        };
    callbacks.session_lost = [this](const std::string &reason) {
      ROS_ERROR_STREAM("PX4 Adapter Runtime session was lost: " << reason);
      exit_requested_.store(true, std::memory_order_release);
    };
    callbacks.log = logFromClient;

    client_.reset(new xgc2::adapter_runtime::Client(std::move(config),
                                                    std::move(callbacks)));
    if (!client_->Start(&error))
      throw std::runtime_error("Adapter Runtime startup failed: " + error);

    periodic_timer_ = node_handle_.createWallTimer(
        ros::WallDuration(0.1), &Px4MultirotorRos1AdapterNode::periodicTimer,
        this);
  }

  ~Px4MultirotorRos1AdapterNode() {
    periodic_timer_.stop();
    if (client_)
      client_->Stop();
    clearInstanceSpec();
  }

  Px4MultirotorRos1AdapterNode(const Px4MultirotorRos1AdapterNode &) = delete;
  Px4MultirotorRos1AdapterNode &
  operator=(const Px4MultirotorRos1AdapterNode &) = delete;

  bool exitRequested() const noexcept {
    return exit_requested_.load(std::memory_order_acquire);
  }

private:
  struct ConnectedRobot {
    std::shared_ptr<RobotRuntime> runtime;
    std::shared_ptr<Px4OperationExecutor> operations;
    xgc2_ros1_robot_adapter::RobotConfig robot;
    xgc2_ros1_robot_adapter::InstanceSpecFence fence;
    xgc::adapter::v1::SourceOpenRequest source_request;
    std::uint64_t source_generation = 0;
    bool active = false;
    std::size_t in_flight_commands = 0;
    std::deque<TelemetryQueueItem> telemetry;
    std::size_t telemetry_bytes = 0;
    std::uint64_t dropped = 0;
  };

  struct SourceLocator {
    std::string robot_id;
    std::uint64_t source_generation = 0;
  };

  struct TelemetrySnapshot {
    std::string robot_id;
    std::string stream_id;
    std::uint64_t source_generation = 0;
    TelemetryBatch batch;
  };

  class CommandLease {
  public:
    CommandLease(Px4MultirotorRos1AdapterNode *owner, std::string robot_id,
                 std::uint64_t source_generation,
                 std::shared_ptr<Px4OperationExecutor> operations)
        : owner_(owner), robot_id_(std::move(robot_id)),
          source_generation_(source_generation),
          operations_(std::move(operations)) {}

    ~CommandLease() {
      if (owner_ != nullptr)
        owner_->releaseCommand(robot_id_, source_generation_);
    }

    CommandLease(const CommandLease &) = delete;
    CommandLease &operator=(const CommandLease &) = delete;

    Px4OperationExecutor *operator->() const { return operations_.get(); }

  private:
    Px4MultirotorRos1AdapterNode *owner_;
    std::string robot_id_;
    std::uint64_t source_generation_;
    std::shared_ptr<Px4OperationExecutor> operations_;
  };

  bool validateRobotConfig(const xgc2_ros1_robot_adapter::RobotConfig &robot,
                           std::string *error) const {
    if (robot.profile_id != contract::kProfileId) {
      *error = "PX4 Adapter received unsupported profile " + robot.profile_id;
      return false;
    }
    const char *profile_digest = contract::profileDigest(robot.profile_id);
    if (profile_digest == nullptr || robot.profile_digest != profile_digest) {
      *error = "profile digest mismatch for robot " + robot.robot_id;
      return false;
    }
    if (robot.parameters.size() != 2 ||
        robot.parameters.find("namespace") == robot.parameters.end() ||
        robot.parameters.find("mocap_rigid_body") == robot.parameters.end()) {
      *error = "robot " + robot.robot_id +
               " must contain exactly namespace and "
               "mocap_rigid_body";
      return false;
    }
    std::string native_error;
    if (!validRobotNamespace(robot.parameters.at("namespace"), &native_error) ||
        !validMocapRigidBodyName(robot.parameters.at("mocap_rigid_body"),
                                 &native_error)) {
      *error = "invalid native configuration for robot " + robot.robot_id +
               ": " + native_error;
      return false;
    }
    for (const auto &channel : robot.channels) {
      contract::ChannelMetadata metadata{};
      if (!contract::channelMetadata(robot.profile_id, channel.channel_id,
                                     &metadata)) {
        *error = "robot " + robot.robot_id +
                 " contains an unknown channel " + channel.channel_id;
        return false;
      }
      if ((metadata.kind == contract::ChannelKind::kOperation) !=
          (metadata.operation_timeout_millis > 0)) {
        *error = "PX4 operation timing metadata is inconsistent";
        return false;
      }
    }
    NativeProfileConfig native_profile;
    if (!BuildNativeProfileConfig(robot, &native_profile, error))
      return false;
    return true;
  }

  bool applyInstanceSpec(const xgc::adapter::v1::AdapterInstanceSpec &spec,
                         std::string *error) {
    contract::MessageMetadata metadata{};
    if (!contract::messageMetadata(4001u, &metadata)) {
      *error = "installed contract does not contain "
               "xgc.robot.v1.RobotAdapterSpec";
      return false;
    }
    xgc2_ros1_robot_adapter::MessageSchema expected_schema;
    expected_schema.message_id = 4001u;
    expected_schema.type_name = "xgc.robot.v1.RobotAdapterSpec";
    expected_schema.version = metadata.version;
    expected_schema.fingerprint = metadata.fingerprint;

    xgc2_ros1_robot_adapter::RobotAdapterConfig candidate;
    if (!xgc2_ros1_robot_adapter::DecodeRobotAdapterConfig(
            spec, expected_schema, &candidate, error))
      return false;
    const auto provider = candidate.scope_attributes.find("provider");
    if (provider == candidate.scope_attributes.end() ||
        provider->second != definition_id_) {
      *error = "robot-group provider does not match this Adapter definition";
      return false;
    }
    std::set<std::string> namespaces;
    std::set<std::string> mocap_rigid_bodies;
    for (const auto &robot : candidate.robots) {
      if (!validateRobotConfig(robot, error))
        return false;
      if (!namespaces.insert(robot.parameters.at("namespace")).second) {
        *error = "multiple PX4 robots share ROS namespace " +
                 robot.parameters.at("namespace");
        return false;
      }
      if (!mocap_rigid_bodies.insert(robot.parameters.at("mocap_rigid_body"))
               .second) {
        *error = "multiple PX4 robots share mocap rigid body " +
                 robot.parameters.at("mocap_rigid_body");
        return false;
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!connected_.empty()) {
      *error =
          "instance spec cannot change while robot resources are connected";
      return false;
    }
    configuration_ = std::move(candidate);
    has_configuration_ = true;
    return true;
  }

  static bool sameSubject(const xgc::adapter::v1::ScopeReference &left,
                          const xgc::adapter::v1::ScopeReference &right) {
    if (left.kind() != right.kind() || left.key() != right.key() ||
        left.attributes_size() != right.attributes_size()) {
      return false;
    }
    for (const auto &attribute : left.attributes()) {
      const auto found = right.attributes().find(attribute.first);
      if (found == right.attributes().end() ||
          found->second != attribute.second) {
        return false;
      }
    }
    return true;
  }

  static bool
  sameSourceRequest(const xgc::adapter::v1::SourceOpenRequest &left,
                    const xgc::adapter::v1::SourceOpenRequest &right) {
    const auto &left_context = left.context();
    const auto &right_context = right.context();
    if (left_context.work_id() != right_context.work_id() ||
        left_context.capability_id() != right_context.capability_id() ||
        left_context.endpoint_id() != right_context.endpoint_id() ||
        left_context.spec_revision() != right_context.spec_revision() ||
        left_context.idempotency_key() != right_context.idempotency_key() ||
        left_context.request_digest() != right_context.request_digest() ||
        left_context.contract_version() != right_context.contract_version() ||
        left_context.contract_digest() != right_context.contract_digest() ||
        left_context.deadline().deadline_unix_nanos() !=
            right_context.deadline().deadline_unix_nanos() ||
        left_context.deadline().ttl_ms() != right_context.deadline().ttl_ms() ||
        left.initial_credit().messages() != right.initial_credit().messages() ||
        left.initial_credit().bytes() != right.initial_credit().bytes()) {
      return false;
    }
    return sameSubject(left_context.subject(), right_context.subject());
  }

  static void stopNativeResources(ConnectedRobot *connected) {
    if (connected == nullptr)
      return;
    if (connected->runtime)
      connected->runtime->Stop();
    connected->operations.reset();
    connected->runtime.reset();
  }

  void releaseCommand(const std::string &robot_id,
                      std::uint64_t source_generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = connected_.find(robot_id);
    if (found == connected_.end() ||
        found->second.source_generation != source_generation ||
        found->second.in_flight_commands == 0) {
      return;
    }
    --found->second.in_flight_commands;
    command_condition_.notify_all();
  }

  bool takeConnectedRobotLocked(
      const SourceLocator &locator,
      const xgc::adapter::v1::SourceOpenRequest *expected_request,
      ConnectedRobot *removed) {
    const auto found = connected_.find(locator.robot_id);
    if (found == connected_.end() ||
        found->second.source_generation != locator.source_generation ||
        (expected_request != nullptr &&
         !sameSourceRequest(found->second.source_request, *expected_request))) {
      return false;
    }

    const auto source =
        sources_by_work_.find(found->second.source_request.context().work_id());
    if (source == sources_by_work_.end() ||
        source->second.robot_id != locator.robot_id ||
        source->second.source_generation != locator.source_generation) {
      return false;
    }

    queued_telemetry_ -= found->second.telemetry.size();
    queued_telemetry_bytes_ -= found->second.telemetry_bytes;
    *removed = std::move(found->second);
    connected_.erase(found);
    sources_by_work_.erase(source);
    return true;
  }

  void clearInstanceSpec() {
    std::map<std::string, ConnectedRobot> removed;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      for (auto &entry : connected_) {
        entry.second.active = false;
        if (entry.second.runtime)
          entry.second.runtime->Deactivate();
      }
      command_condition_.wait(lock, [this] {
        for (const auto &entry : connected_) {
          if (entry.second.in_flight_commands != 0)
            return false;
        }
        return true;
      });
      removed.swap(connected_);
      sources_by_work_.clear();
      queued_telemetry_ = 0;
      queued_telemetry_bytes_ = 0;
      configuration_ = {};
      has_configuration_ = false;
    }
    for (auto &entry : removed)
      stopNativeResources(&entry.second);
  }

  xgc2::adapter_runtime::SourceOpenDecision handleSourceOpen(
      const xgc::adapter::v1::SourceOpenRequest &request,
      const xgc2::adapter_runtime::CancellationToken &cancellation) {
    if (cancellation.IsCancellationRequested()) {
      return rejectedSource(xgc::adapter::v1::ERROR_CLASS_CANCELLED,
                            "source-open-cancelled",
                            "telemetry source open was cancelled");
    }

    const auto &context = request.context();
    if (context.capability_id() !=
            xgc2_ros1_robot_adapter::kTelemetryCapability ||
        context.endpoint_id() != kTelemetryEndpoint) {
      return rejectedSource(
          xgc::adapter::v1::ERROR_CLASS_REJECTED, "invalid-source-endpoint",
          "PX4 telemetry source used an unsupported endpoint");
    }
    if (context.work_id().empty()) {
      return rejectedSource(xgc::adapter::v1::ERROR_CLASS_REJECTED,
                            "invalid-source-identity",
                            "PX4 telemetry source work_id is empty");
    }

    std::string error;
    std::string robot_id;
    xgc2_ros1_robot_adapter::RobotConfig robot;
    xgc2_ros1_robot_adapter::InstanceSpecFence fence;
    SourceLocator locator;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_configuration_) {
        return rejectedSource(xgc::adapter::v1::ERROR_CLASS_REJECTED,
                              "instance-spec-unavailable",
                              "robot instance configuration is not applied");
      }
      if (context.spec_revision() != configuration_.fence.revision) {
        return rejectedSource(
            xgc::adapter::v1::ERROR_CLASS_REJECTED, "stale-source-spec",
            "telemetry source does not target the applied spec");
      }
      if (!xgc2_ros1_robot_adapter::ResolveRobotSubject(context, configuration_,
                                                        &robot_id, &error)) {
        return rejectedSource(xgc::adapter::v1::ERROR_CLASS_REJECTED,
                              "invalid-robot-subject", error);
      }
      if (connected_.find(robot_id) != connected_.end()) {
        return rejectedSource(
            xgc::adapter::v1::ERROR_CLASS_REJECTED, "robot-source-already-open",
            "robot already has an opening or active telemetry source");
      }
      if (sources_by_work_.find(context.work_id()) != sources_by_work_.end()) {
        return rejectedSource(xgc::adapter::v1::ERROR_CLASS_REJECTED,
                              "source-identity-conflict",
                              "telemetry work_id is already reserved");
      }
      if (next_source_generation_ ==
          std::numeric_limits<std::uint64_t>::max()) {
        return rejectedSource(xgc::adapter::v1::ERROR_CLASS_RESOURCE_EXHAUSTED,
                              "source-generation-exhausted",
                              "telemetry source generation is exhausted");
      }
      for (const auto &candidate : configuration_.robots) {
        if (candidate.robot_id == robot_id) {
          robot = candidate;
          break;
        }
      }
      fence = configuration_.fence;
      locator.robot_id = robot_id;
      locator.source_generation = ++next_source_generation_;

      ConnectedRobot opening;
      opening.robot = robot;
      opening.fence = fence;
      opening.source_request = request;
      opening.source_generation = locator.source_generation;
      const auto connected_inserted =
          connected_.emplace(robot_id, std::move(opening));
      if (!connected_inserted.second) {
        return rejectedSource(
            xgc::adapter::v1::ERROR_CLASS_REJECTED, "robot-source-already-open",
            "robot source reservation raced with another open");
      }
      try {
        const auto source_inserted =
            sources_by_work_.emplace(context.work_id(), locator);
        if (!source_inserted.second) {
          connected_.erase(connected_inserted.first);
          return rejectedSource(
              xgc::adapter::v1::ERROR_CLASS_REJECTED,
              "source-identity-conflict",
              "telemetry work_id reservation raced with another open");
        }
      } catch (...) {
        connected_.erase(connected_inserted.first);
        throw;
      }
    }

    auto reservation = makeScopeExit([this, &locator, &request] {
      ConnectedRobot removed;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        takeConnectedRobotLocked(locator, nullptr, &removed);
      }
      stopNativeResources(&removed);
    });

    NativeProfileConfig native_profile;
    if (!BuildNativeProfileConfig(robot, &native_profile, &error)) {
      ConnectedRobot removed;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        takeConnectedRobotLocked(locator, nullptr, &removed);
      }
      return rejectedSource(xgc::adapter::v1::ERROR_CLASS_PERMANENT,
                            "native-profile-invalid", error);
    }

    std::shared_ptr<RobotRuntime> runtime;
    std::shared_ptr<Px4OperationExecutor> operations;
    auto native_resources = makeScopeExit([&runtime, &operations] {
      if (runtime)
        runtime->Stop();
      operations.reset();
      runtime.reset();
    });
    runtime = RobotRuntime::Create(
        node_handle_, robot, fence.revision,
        [this, robot_id, locator](std::string item) {
          enqueueTelemetry(robot_id, locator.source_generation,
                           std::move(item));
        },
        &error);
    if (!runtime) {
      ConnectedRobot removed;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        takeConnectedRobotLocked(locator, nullptr, &removed);
      }
      return rejectedSource(xgc::adapter::v1::ERROR_CLASS_TRANSIENT,
                            "robot-runtime-start-failed", error);
    }
    if (cancellation.IsCancellationRequested()) {
      ConnectedRobot removed;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        takeConnectedRobotLocked(locator, nullptr, &removed);
      }
      runtime->Stop();
      return rejectedSource(xgc::adapter::v1::ERROR_CLASS_CANCELLED,
                            "source-open-cancelled",
                            "telemetry source open was cancelled");
    }

    bool operation_enabled = false;
    for (const auto &channel : robot.channels) {
      contract::ChannelMetadata metadata{};
      if (channel.enabled &&
          contract::channelMetadata(robot.profile_id, channel.channel_id,
                                    &metadata) &&
          metadata.kind == contract::ChannelKind::kOperation) {
        operation_enabled = true;
        break;
      }
    }
    if (operation_enabled) {
      Px4OperationExecutor::Config operation_config;
      operation_config.state_endpoint =
          std::move(native_profile.state_endpoint);
      operation_config.arm_service_endpoint =
          std::move(native_profile.arm_service_endpoint);
      operation_config.mode_service_endpoint =
          std::move(native_profile.mode_service_endpoint);
      operation_config.reboot_service_endpoint =
          std::move(native_profile.reboot_service_endpoint);
      operation_config.require_state =
          channelEnabled(robot, "operation.autopilot-reboot");
      operation_config.state_timeout_seconds =
          native_profile.reboot_state_timeout_seconds;
      operation_config.maximum_operation_timeout_seconds =
          native_profile.maximum_operation_timeout_seconds;
      operation_config.allowed_modes = std::move(native_profile.allowed_modes);
      auto unique_operations = Px4OperationExecutor::Create(
          node_handle_, std::move(operation_config), &error);
      if (!unique_operations) {
        ConnectedRobot removed;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          takeConnectedRobotLocked(locator, nullptr, &removed);
        }
        runtime->Stop();
        return rejectedSource(xgc::adapter::v1::ERROR_CLASS_TRANSIENT,
                              "command-runtime-start-failed", error);
      }
      operations =
          std::shared_ptr<Px4OperationExecutor>(std::move(unique_operations));
    }

    auto decision = xgc2::adapter_runtime::SourceOpenDecision::Accept();
    bool committed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto found = connected_.find(robot_id);
      if (!cancellation.IsCancellationRequested() &&
          found != connected_.end() &&
          found->second.source_generation == locator.source_generation &&
          found->second.fence.matches(fence) &&
          sameSourceRequest(found->second.source_request, request)) {
        found->second.runtime = runtime;
        found->second.operations = operations;
        runtime->Activate();
        found->second.active = true;
        committed = true;
      } else {
        ConnectedRobot removed;
        takeConnectedRobotLocked(locator, nullptr, &removed);
      }
    }
    if (!committed) {
      runtime->Stop();
      operations.reset();
      return rejectedSource(
          xgc::adapter::v1::ERROR_CLASS_CANCELLED, "stale-source-open",
          "telemetry source became stale while native resources started");
    }
    native_resources.release();
    reservation.release();
    return decision;
  }

  void handleSourceClosed(const xgc::adapter::v1::SourceOpenRequest &request,
                          const xgc::adapter::v1::AdapterError &error) {
    ConnectedRobot removed;
    std::shared_ptr<RobotRuntime> runtime_to_stop;
    std::uint64_t source_generation = 0u;
    bool identified = false;
    bool matched = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto source = sources_by_work_.find(request.context().work_id());
      if (source != sources_by_work_.end()) {
        const auto connected = connected_.find(source->second.robot_id);
        if (connected != connected_.end() &&
            connected->second.source_generation ==
                source->second.source_generation &&
            sameSourceRequest(connected->second.source_request, request)) {
          identified = true;
          source_generation = connected->second.source_generation;
          connected->second.active = false;
          runtime_to_stop = connected->second.runtime;
          if (runtime_to_stop)
            runtime_to_stop->Deactivate();
          if (connected->second.in_flight_commands == 0u) {
            queued_telemetry_ -= connected->second.telemetry.size();
            queued_telemetry_bytes_ -= connected->second.telemetry_bytes;
            removed = std::move(connected->second);
            connected_.erase(connected);
            sources_by_work_.erase(source);
            matched = true;
          }
        }
      }
    }
    // Native publishers are fenced immediately when the Host closes a source;
    // command draining must never extend the side-effect lifetime.
    if (runtime_to_stop)
      runtime_to_stop->Stop();
    if (!identified)
      return;

    if (!matched) {
      std::unique_lock<std::mutex> lock(mutex_);
      command_condition_.wait(lock, [this, &request, source_generation] {
        const auto source = sources_by_work_.find(request.context().work_id());
        if (source == sources_by_work_.end() ||
            source->second.source_generation != source_generation) {
          return true;
        }
        const auto connected = connected_.find(source->second.robot_id);
        return connected == connected_.end() ||
               connected->second.source_generation != source_generation ||
               connected->second.in_flight_commands == 0u;
      });
      const auto source = sources_by_work_.find(request.context().work_id());
      if (source != sources_by_work_.end() &&
          source->second.source_generation == source_generation) {
        const auto connected = connected_.find(source->second.robot_id);
        if (connected != connected_.end() &&
            connected->second.source_generation == source_generation &&
            connected->second.in_flight_commands == 0u) {
          queued_telemetry_ -= connected->second.telemetry.size();
          queued_telemetry_bytes_ -= connected->second.telemetry_bytes;
          removed = std::move(connected->second);
          connected_.erase(connected);
          sources_by_work_.erase(source);
          matched = true;
        }
      }
    }
    if (!matched)
      return;
    stopNativeResources(&removed);
    ROS_WARN_STREAM("PX4 telemetry source " << request.context().work_id()
                                            << " closed: " << error.message());
  }

  xgc2::adapter_runtime::OperationResult
  handleCommand(const xgc::adapter::v1::OperationRequest &request,
                const xgc2::adapter_runtime::CancellationToken &cancellation) {
    if (request.input().encoding() != xgc::v1::PAYLOAD_ENCODING_PROTOBUF)
      return rejected("invalid-command-encoding",
                      "robot command input must use protobuf encoding");
    if (cancellation.IsCancellationRequested())
      return xgc2::adapter_runtime::OperationResult::Cancelled();

    std::string error;
    std::string robot_id;
    std::uint64_t source_generation = 0;
    std::shared_ptr<Px4OperationExecutor> operation_executor;
    xgc2_ros1_robot_adapter::RobotConfig robot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_configuration_)
        return rejected("instance-spec-unavailable",
                        "robot instance configuration is not applied");
      if (!xgc2_ros1_robot_adapter::ResolveRobotSubject(
              request.context(), configuration_, &robot_id, &error)) {
        return rejected("invalid-robot-subject", error);
      }
      const auto found = connected_.find(robot_id);
      if (found == connected_.end() || !found->second.active ||
          !found->second.fence.matches(configuration_.fence) ||
          found->second.fence.revision != request.context().spec_revision() ||
          !sameSubject(found->second.source_request.context().subject(),
                       request.context().subject())) {
        return rejected(
            "telemetry-source-not-open",
            "robot command requires the current telemetry source to be open");
      }
      if (!found->second.operations) {
        return rejected("command-disabled",
                        "robot spec enables no native command channels");
      }
      robot = found->second.robot;
      source_generation = found->second.source_generation;
      operation_executor = found->second.operations;
      ++found->second.in_flight_commands;
    }
    CommandLease operations(this, std::move(robot_id), source_generation,
                            std::move(operation_executor));
    if (cancellation.IsCancellationRequested())
      return xgc2::adapter_runtime::OperationResult::Cancelled();

    const std::string &endpoint = request.context().endpoint_id();
    ResolvedOperation operation;
    bool known_operation = false;
    if (!resolveOperation(robot.profile_id, endpoint, &operation,
                          &known_operation, &error)) {
      if (!known_operation)
        return rejected("unknown-command-endpoint", error);
      return xgc2::adapter_runtime::OperationResult::Failure(
          xgc::adapter::v1::ERROR_CLASS_PERMANENT, "operation-contract-invalid",
          error);
    }
    if (!channelEnabled(robot, operation.channel.channel_id)) {
      return rejected("command-disabled",
                      endpoint + " is disabled by the robot spec");
    }
    if (!inputSchemaMatches(request.input(), operation)) {
      return rejected("invalid-command-schema",
                      "command input schema does not match the installed "
                      "operation descriptor");
    }
    OperationTiming timing;
    if (!operationTiming(request.context(), operation, &timing, &error)) {
      return xgc2::adapter_runtime::OperationResult::Failure(
          xgc::adapter::v1::ERROR_CLASS_PERMANENT, "operation-contract-invalid",
          error);
    }

    OperationResult native;
    const std::string processor(operation.channel.processor);
    if (processor == "px4.arm") {
      xgc::semantic::aerial::v1::ArmRequest input;
      if (operation.input_schema.type_name !=
          input.GetDescriptor()->full_name()) {
        return xgc2::adapter_runtime::OperationResult::Failure(
            xgc::adapter::v1::ERROR_CLASS_PERMANENT,
            "operation-contract-invalid",
            "PX4 arm processor input schema drifted");
      }
      if (!input.ParseFromString(request.input().value()))
        return rejected("invalid-command-input", "ArmRequest is malformed");
      native = operations->setArmed(input.armed(), timing);
    } else if (processor == "px4.mode") {
      xgc::semantic::aerial::v1::ModeRequest input;
      if (operation.input_schema.type_name !=
          input.GetDescriptor()->full_name()) {
        return xgc2::adapter_runtime::OperationResult::Failure(
            xgc::adapter::v1::ERROR_CLASS_PERMANENT,
            "operation-contract-invalid",
            "PX4 mode processor input schema drifted");
      }
      if (!input.ParseFromString(request.input().value()))
        return rejected("invalid-command-input", "ModeRequest is malformed");
      native = operations->setMode(input.mode(), timing);
    } else if (processor == "px4.autopilot-reboot") {
      xgc::semantic::aerial::v1::AutopilotRebootRequest input;
      if (operation.input_schema.type_name !=
          input.GetDescriptor()->full_name()) {
        return xgc2::adapter_runtime::OperationResult::Failure(
            xgc::adapter::v1::ERROR_CLASS_PERMANENT,
            "operation-contract-invalid",
            "PX4 reboot processor input schema drifted");
      }
      if (!input.ParseFromString(request.input().value())) {
        return rejected("invalid-command-input",
                        "AutopilotRebootRequest is malformed");
      }
      native = operations->rebootAutopilot(timing);
    } else {
      return xgc2::adapter_runtime::OperationResult::Failure(
          xgc::adapter::v1::ERROR_CLASS_PERMANENT, "operation-contract-invalid",
          "installed operation processor has no PX4 native binding");
    }
    return mapNativeResult(native, operation);
  }

  void discardFrontTelemetryLocked(ConnectedRobot *connected) {
    if (connected == nullptr || connected->telemetry.empty())
      return;
    const std::size_t item_bytes = connected->telemetry.front().value.size();
    connected->telemetry.pop_front();
    connected->telemetry_bytes -= item_bytes;
    --queued_telemetry_;
    queued_telemetry_bytes_ -= item_bytes;
  }

  void enqueueTelemetry(const std::string &robot_id,
                        std::uint64_t source_generation, std::string item) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = connected_.find(robot_id);
    if (found == connected_.end() ||
        found->second.source_generation != source_generation)
      return;

    auto &connected = found->second;
    const std::size_t item_bytes = item.size();
    if (item_bytes > kMaximumTelemetryBatchBytes) {
      ++connected.dropped;
      return;
    }
    while (!connected.telemetry.empty() &&
           (connected.telemetry.size() >= kMaximumQueuedTelemetryPerRobot ||
            item_bytes > kMaximumQueuedTelemetryBytesPerRobot -
                             connected.telemetry_bytes)) {
      discardFrontTelemetryLocked(&connected);
      ++connected.dropped;
    }
    if (queued_telemetry_ >= kMaximumQueuedTelemetry ||
        item_bytes > kMaximumQueuedTelemetryBytes - queued_telemetry_bytes_ ||
        next_telemetry_token_ == std::numeric_limits<std::uint64_t>::max()) {
      ++connected.dropped;
      return;
    }

    TelemetryQueueItem queued;
    const std::uint64_t token = next_telemetry_token_ + 1u;
    queued.token = token;
    queued.value = std::move(item);
    connected.telemetry.push_back(std::move(queued));
    next_telemetry_token_ = token;
    connected.telemetry_bytes += item_bytes;
    ++queued_telemetry_;
    queued_telemetry_bytes_ += item_bytes;
  }

  bool nextTelemetrySnapshot(
      const std::set<std::pair<std::string, std::uint64_t>> &blocked,
      TelemetrySnapshot *snapshot) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (connected_.empty())
      return false;

    auto iterator = connected_.upper_bound(flush_cursor_);
    if (iterator == connected_.end())
      iterator = connected_.begin();
    for (std::size_t examined = 0; examined < connected_.size(); ++examined) {
      const auto current = iterator;
      ++iterator;
      if (iterator == connected_.end())
        iterator = connected_.begin();

      const auto source_key =
          std::make_pair(current->first, current->second.source_generation);
      if (!current->second.active || !current->second.runtime ||
          current->second.telemetry.empty() ||
          blocked.find(source_key) != blocked.end()) {
        continue;
      }

      flush_cursor_ = current->first;
      snapshot->robot_id = current->first;
      snapshot->stream_id = current->second.source_request.context().work_id();
      snapshot->source_generation = current->second.source_generation;
      snapshot->batch = buildTelemetryBatch(current->second.telemetry);
      return !snapshot->batch.items.empty();
    }
    return false;
  }

  void finishTelemetryAttempt(const TelemetrySnapshot &snapshot,
                              xgc2::adapter_runtime::SourceWriteResult result) {
    if (result != xgc2::adapter_runtime::SourceWriteResult::kAccepted &&
        result != xgc2::adapter_runtime::SourceWriteResult::kUnknownStream &&
        result != xgc2::adapter_runtime::SourceWriteResult::kTooLarge) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = connected_.find(snapshot.robot_id);
    if (found == connected_.end() ||
        found->second.source_generation != snapshot.source_generation ||
        found->second.source_request.context().work_id() !=
            snapshot.stream_id ||
        !telemetryBatchMatchesPrefix(found->second.telemetry,
                                     snapshot.batch.tokens)) {
      return;
    }
    const std::size_t batch_size = snapshot.batch.tokens.size();
    for (std::size_t index = 0; index < batch_size; ++index)
      discardFrontTelemetryLocked(&found->second);
    if (result != xgc2::adapter_runtime::SourceWriteResult::kAccepted)
      found->second.dropped += batch_size;
  }

  void flushTelemetry() {
    if (!client_)
      return;
    std::set<std::pair<std::string, std::uint64_t>> blocked;
    for (std::size_t attempted = 0; attempted < kMaximumPublishAttemptsPerTick;
         ++attempted) {
      TelemetrySnapshot snapshot;
      if (!nextTelemetrySnapshot(blocked, &snapshot))
        break;
      const auto result = client_->PublishSource(
          snapshot.stream_id, std::move(snapshot.batch.items));
      if (result == xgc2::adapter_runtime::SourceWriteResult::kNoCredit ||
          result == xgc2::adapter_runtime::SourceWriteResult::kNotReady ||
          result == xgc2::adapter_runtime::SourceWriteResult::kQueueFull) {
        blocked.insert(
            std::make_pair(snapshot.robot_id, snapshot.source_generation));
        continue;
      }
      finishTelemetryAttempt(snapshot, result);
    }
  }

  void periodicTimer(const ros::WallTimerEvent &) {
    std::unique_lock<std::mutex> periodic_lock(periodic_mutex_,
                                               std::try_to_lock);
    if (!periodic_lock.owns_lock())
      return;
    std::vector<std::shared_ptr<RobotRuntime>> runtimes;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto &entry : connected_) {
        if (entry.second.active && entry.second.runtime)
          runtimes.push_back(entry.second.runtime);
      }
    }
    const ros::WallTime now = ros::WallTime::now();
    for (const auto &runtime : runtimes)
      runtime->emitPeriodic(now);
    flushTelemetry();
  }

  ros::NodeHandle node_handle_;
  ros::WallTimer periodic_timer_;
  std::string definition_id_;
  std::atomic<bool> exit_requested_{false};

  mutable std::mutex mutex_;
  std::mutex periodic_mutex_;
  std::condition_variable command_condition_;
  bool has_configuration_ = false;
  xgc2_ros1_robot_adapter::RobotAdapterConfig configuration_;
  std::map<std::string, ConnectedRobot> connected_;
  std::map<std::string, SourceLocator> sources_by_work_;
  std::uint64_t next_source_generation_ = 0;
  std::uint64_t next_telemetry_token_ = 0;
  std::size_t queued_telemetry_ = 0;
  std::size_t queued_telemetry_bytes_ = 0;
  std::string flush_cursor_;
  // Keep the callback-owning client last so construction failures destroy and
  // stop it before any state captured by its callbacks.
  std::unique_ptr<xgc2::adapter_runtime::Client> client_;
};

} // namespace xgc_px4_multirotor_ros1_adapter

int main(int argc, char **argv) {
  std::string bootstrap_file;
  std::string error;
  if (!xgc2_ros1_robot_adapter::BootstrapFileFromArguments(
          argc, argv, &bootstrap_file, &error)) {
    std::cerr << "xgc_px4_multirotor_ros1_adapter: " << error << '\n';
    return 2;
  }
  ros::init(argc, argv, "xgc_px4_multirotor_ros1_adapter",
            ros::init_options::AnonymousName |
                ros::init_options::NoSigintHandler);
  ros::master::setRetryTimeout(ros::WallDuration(3.0));
  try {
    xgc_px4_multirotor_ros1_adapter::ShutdownSignalHandler shutdown_signals;
    xgc_px4_multirotor_ros1_adapter::Px4MultirotorRos1AdapterNode node(
        ros::NodeHandle(), bootstrap_file);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    while (ros::ok() && !shutdown_signals.requested() && !node.exitRequested())
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if ((shutdown_signals.requested() || node.exitRequested()) && ros::ok())
      ros::shutdown();
    ros::waitForShutdown();
    spinner.stop();
  } catch (const std::exception &exception) {
    ROS_FATAL_STREAM("PX4 robot Adapter startup failed: " << exception.what());
    return 1;
  }
  return 0;
}
