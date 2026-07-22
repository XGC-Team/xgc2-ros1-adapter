#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
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

#include "xgc/semantic/ground/v1/control.pb.h"
#include "xgc/v1/message.pb.h"
#include "xgc2/adapter_runtime/client.hpp"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"
#include "xgc2_ros1_robot_adapter/runtime_support.hpp"
#include "xgc_scout_mini_ros1_adapter/generated_contract.hpp"
#include "xgc_scout_mini_ros1_adapter/motion_command.hpp"
#include "xgc_scout_mini_ros1_adapter/robot_runtime.hpp"
#include "xgc_scout_mini_ros1_adapter/shutdown_signal.hpp"
#include "xgc_scout_mini_ros1_adapter/telemetry_batch.hpp"

namespace xgc_scout_mini_ros1_adapter {
namespace {

constexpr const char *kTelemetryEndpoint = "telemetry";
constexpr std::uint32_t kRobotAdapterSpecMessageId = 4001u;
constexpr std::size_t kMaximumQueuedTelemetryItemsPerRobot = 2048u;
constexpr std::size_t kMaximumQueuedTelemetryBytesPerRobot =
    16u * 1024u * 1024u;
constexpr std::size_t kMaximumQueuedTelemetryItems = 8192u;
constexpr std::size_t kMaximumQueuedTelemetryBytes = 64u * 1024u * 1024u;
constexpr std::size_t kMaximumFlushItemsPerTick = 256u;

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

xgc2::adapter_runtime::OperationResult rejectOperation(
    const std::string &code, const std::string &message) {
  return xgc2::adapter_runtime::OperationResult::Rejected(code, message);
}

struct ResolvedOperation {
  contract::ChannelMetadata channel{};
  contract::MessageMetadata input_schema{};
  contract::MessageMetadata output_schema{};
  bool lease_pulse = false;
};

bool resolveOperation(const std::string &profile_id,
                      const std::string &operation_id,
                      ResolvedOperation *resolved, bool *known,
                      std::string *error) {
  if (resolved == nullptr || known == nullptr || error == nullptr)
    return false;
  *resolved = {};
  *known = false;

  std::size_t channel_count = 0u;
  const auto *channels = contract::profileChannels(profile_id, &channel_count);
  if (channels == nullptr || channel_count == 0u) {
    *known = true;
    *error = "installed Scout operation profile is unavailable";
    return false;
  }
  for (std::size_t index = 0u; index < channel_count; ++index) {
    const auto &candidate = channels[index];
    if (candidate.kind != contract::ChannelKind::kOperation)
      continue;
    if (candidate.operation_id == nullptr ||
        candidate.operation_id[0] == '\0') {
      *known = true;
      *error = "installed Scout profile contains an unnamed operation";
      return false;
    }
    const bool public_operation = operation_id == candidate.operation_id;
    const bool lease_pulse =
        candidate.operation_lease.pulse_endpoint_id != nullptr &&
        candidate.operation_lease.pulse_endpoint_id[0] != '\0' &&
        operation_id == candidate.operation_lease.pulse_endpoint_id;
    if (!public_operation && !lease_pulse)
      continue;
    if (*known) {
      *error = "installed Scout profile repeats operation " + operation_id;
      return false;
    }
    *known = true;
    resolved->channel = candidate;
    resolved->lease_pulse = lease_pulse;
  }
  if (!*known) {
    *error = "unsupported Scout command endpoint";
    return false;
  }

  const auto &channel = resolved->channel;
  if (channel.channel_id == nullptr || channel.channel_id[0] == '\0' ||
      channel.processor == nullptr || channel.processor[0] == '\0' ||
      channel.input_message_id == 0u || channel.output_message_id != 1u ||
      channel.operation_timeout_millis == 0u ||
      channel.operation_contract.side_effect == nullptr ||
      std::string(channel.operation_contract.side_effect) != "idempotent" ||
      channel.operation_contract.idempotency == nullptr ||
      std::string(channel.operation_contract.idempotency) != "required" ||
      !channel.operation_contract.cancellation_supported ||
      !channel.operation_contract.deadline_required ||
      !contract::messageMetadata(channel.input_message_id,
                                 &resolved->input_schema) ||
      !contract::messageMetadata(channel.output_message_id,
                                 &resolved->output_schema) ||
      resolved->input_schema.type_name == nullptr ||
      resolved->input_schema.version == 0u ||
      resolved->input_schema.fingerprint == 0u ||
      resolved->output_schema.type_name == nullptr ||
      resolved->output_schema.version == 0u ||
      resolved->output_schema.fingerprint == 0u ||
      std::string(resolved->output_schema.type_name) != "xgc.v1.Empty") {
    *error = "installed Scout operation descriptor is incomplete or unsafe";
    return false;
  }
  if (channel.operation_lease.pulse_endpoint_id == nullptr ||
      channel.operation_lease.pulse_endpoint_id[0] == '\0' ||
      channel.operation_lease.heartbeat_interval_millis == 0u ||
      channel.operation_lease.timeout_millis == 0u ||
      !channel.operation_lease.volatile_supported) {
    *error = "installed Scout lease pulse descriptor is incomplete or unsafe";
    return false;
  }
  return true;
}

bool operationDeadline(const xgc::adapter::v1::WorkContext &context,
                       std::uint32_t maximum_ttl_millis,
                       ros::WallTime *expires_at, std::string *error) {
  if (expires_at == nullptr || error == nullptr || maximum_ttl_millis == 0u)
    return false;
  const std::int64_t deadline_nanos =
      context.deadline().deadline_unix_nanos();
  if (deadline_nanos <= 0) {
    *error = "Scout motion operation requires an absolute deadline";
    return false;
  }
  const auto now_system = std::chrono::system_clock::now();
  const auto now_nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             now_system.time_since_epoch())
                             .count();
  if (deadline_nanos <= now_nanos) {
    *error = "Scout motion operation deadline elapsed before publication";
    return false;
  }
  const std::int64_t maximum_ttl_nanos =
      static_cast<std::int64_t>(maximum_ttl_millis) * 1000000LL;
  const std::int64_t bounded_deadline =
      std::min(deadline_nanos, now_nanos + maximum_ttl_nanos);
  const std::int64_t seconds = bounded_deadline / 1000000000LL;
  const std::int64_t nanoseconds = bounded_deadline % 1000000000LL;
  if (seconds <= 0 ||
      seconds > static_cast<std::int64_t>(
                    std::numeric_limits<std::uint32_t>::max())) {
    *error = "Scout motion operation deadline is outside ROS wall-time range";
    return false;
  }
  *expires_at = ros::WallTime(static_cast<std::uint32_t>(seconds),
                             static_cast<std::uint32_t>(nanoseconds));
  error->clear();
  return true;
}

std::string operationLeaseOwner(
    const xgc::adapter::v1::ScopeReference &subject,
    const std::string &robot_id, const std::string &operation_id) {
  const auto run = subject.attributes().find("run-id");
  if (run == subject.attributes().end() || run->second.empty() ||
      robot_id.empty() || operation_id.empty()) {
    return {};
  }
  // Base commands, renewal pulses, and the declared inactive pulse derive the
  // same stable lease identity. RenewIntent separately requires the exact
  // command payload, so a stale pulse cannot change the current intent.
  return run->second + "\n" + robot_id + "\n" + operation_id;
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

bool channelEnabled(const xgc2_ros1_robot_adapter::RobotConfig &robot,
                    const std::string &channel_id) {
  for (const auto &channel : robot.channels) {
    if (channel.channel_id == channel_id)
      return channel.enabled;
  }
  return false;
}

xgc2::adapter_runtime::SourceOpenDecision
rejectSource(xgc::adapter::v1::ErrorClass error_class, const std::string &code,
             const std::string &message) {
  return xgc2::adapter_runtime::SourceOpenDecision::Reject(error_class, code,
                                                           message);
}

bool sameSubject(const xgc::adapter::v1::ScopeReference &left,
                 const xgc::adapter::v1::ScopeReference &right) {
  if (left.kind() != right.kind() || left.key() != right.key() ||
      left.attributes_size() != right.attributes_size()) {
    return false;
  }
  for (const auto &attribute : left.attributes()) {
    const auto found = right.attributes().find(attribute.first);
    if (found == right.attributes().end() || found->second != attribute.second)
      return false;
  }
  return true;
}

} // namespace

class ScoutMiniRos1AdapterNode {
public:
  ScoutMiniRos1AdapterNode(ros::NodeHandle node_handle,
                           const std::string &bootstrap_file)
      : node_handle_(std::move(node_handle)) {
    auto config =
        xgc2::adapter_runtime::ClientConfig::FromBootstrapFile(bootstrap_file);
    definition_id_ = config.registration().definition_id();

    xgc2::adapter_runtime::CapabilityCallbacks telemetry;
    telemetry.start = [](const xgc::adapter::v1::AdapterInstanceSpec &,
                         const xgc::adapter::v1::EnabledCapability &,
                         std::string *) { return true; };
    telemetry.stop = [this] { stopSources(); };
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
    command.unary =
        [this](const xgc::adapter::v1::UnaryRequest &request,
               const xgc2::adapter_runtime::CancellationToken &cancellation) {
          return handleLeasePulse(request, cancellation);
        };
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
    callbacks.stop_requested = [this](const xgc::adapter::v1::StopRequest &) {
      stopMotionControllers();
      exit_requested_.store(true, std::memory_order_release);
    };
    callbacks.session_lost = [this](const std::string &) {
      stopMotionControllers();
      exit_requested_.store(true, std::memory_order_release);
    };
    callbacks.log = logFromClient;

    client_.reset(new xgc2::adapter_runtime::Client(std::move(config),
                                                    std::move(callbacks)));
    if (!client_->Start(&error))
      throw std::runtime_error("Adapter Runtime startup failed: " + error);

    periodic_timer_ = node_handle_.createWallTimer(
        ros::WallDuration(0.1), &ScoutMiniRos1AdapterNode::periodicTimer, this);
  }

  ~ScoutMiniRos1AdapterNode() {
    periodic_timer_.stop();
    if (client_)
      client_->Stop();
    clearInstanceSpec();
  }

  ScoutMiniRos1AdapterNode(const ScoutMiniRos1AdapterNode &) = delete;
  ScoutMiniRos1AdapterNode &
  operator=(const ScoutMiniRos1AdapterNode &) = delete;

  bool exitRequested() const noexcept {
    return exit_requested_.load(std::memory_order_acquire);
  }

private:
  struct ConnectedRobot {
    std::shared_ptr<RobotRuntime> runtime;
    std::shared_ptr<MotionCommandPublisher> motion;
    xgc2_ros1_robot_adapter::RobotConfig robot;
    std::string work_id;
    xgc::adapter::v1::ScopeReference subject;
    xgc2_ros1_robot_adapter::InstanceSpecFence fence;
    std::uint64_t source_generation = 0;
    std::deque<TelemetryQueueItem> telemetry;
    std::size_t queued_bytes = 0;
    std::uint64_t dropped = 0;
  };

  struct TelemetrySnapshot {
    std::string robot_id;
    std::string work_id;
    std::uint64_t source_generation = 0;
    TelemetryBatch batch;
  };

  bool validateRobotConfig(const xgc2_ros1_robot_adapter::RobotConfig &robot,
                           std::string *error) const {
    if (!validateNativeProfileContract(error))
      return false;
    if (robot.profile_id != contract::kProfileId) {
      *error = "Scout Adapter received unsupported profile " + robot.profile_id;
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
               " must contain exactly namespace and mocap_rigid_body";
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
        *error = "Scout robot " + robot.robot_id +
                 " contains an unknown channel " + channel.channel_id;
        return false;
      }
      if ((metadata.kind == contract::ChannelKind::kOperation) !=
          (metadata.operation_timeout_millis > 0u)) {
        *error = "Scout operation timing metadata is inconsistent";
        return false;
      }
    }
    return true;
  }

  bool applyInstanceSpec(const xgc::adapter::v1::AdapterInstanceSpec &spec,
                         std::string *error) {
    contract::MessageMetadata metadata;
    if (!contract::messageMetadata(kRobotAdapterSpecMessageId, &metadata)) {
      *error = "installed contract does not contain RobotAdapterSpec schema";
      return false;
    }
    xgc2_ros1_robot_adapter::MessageSchema expected_schema;
    expected_schema.message_id = kRobotAdapterSpecMessageId;
    expected_schema.type_name = "xgc.robot.v1.RobotAdapterSpec";
    expected_schema.version = metadata.version;
    expected_schema.fingerprint = metadata.fingerprint;

    xgc2_ros1_robot_adapter::RobotAdapterConfig candidate;
    if (!xgc2_ros1_robot_adapter::DecodeRobotAdapterConfig(
            spec, expected_schema, &candidate, error)) {
      return false;
    }
    const auto provider = candidate.scope_attributes.find("provider");
    if (provider == candidate.scope_attributes.end() ||
        provider->second != definition_id_) {
      *error = "robot-group provider does not match this Adapter definition";
      return false;
    }
    std::set<std::string> namespaces;
    for (const auto &robot : candidate.robots) {
      if (!validateRobotConfig(robot, error))
        return false;
      if (!namespaces.insert(robot.parameters.at("namespace")).second) {
        *error = "multiple Scout robots share ROS namespace " +
                 robot.parameters.at("namespace");
        return false;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!connected_.empty()) {
      *error = "instance spec cannot change while robot sources are active";
      return false;
    }
    configuration_ = std::move(candidate);
    has_configuration_ = true;
    return true;
  }

  void stopSources() {
    std::map<std::string, ConnectedRobot> removed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      removed.swap(connected_);
      queued_telemetry_items_ = 0;
      queued_telemetry_bytes_ = 0;
      flush_cursor_ = 0;
    }
    for (auto &entry : removed) {
      if (entry.second.motion)
        entry.second.motion->Stop();
      if (entry.second.runtime)
        entry.second.runtime->Stop();
    }
  }

  void clearInstanceSpec() {
    std::map<std::string, ConnectedRobot> removed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      removed.swap(connected_);
      queued_telemetry_items_ = 0;
      queued_telemetry_bytes_ = 0;
      configuration_ = {};
      has_configuration_ = false;
      flush_cursor_ = 0;
    }
    for (auto &entry : removed) {
      if (entry.second.motion)
        entry.second.motion->Stop();
      if (entry.second.runtime)
        entry.second.runtime->Stop();
    }
  }

  void stopMotionControllers() {
    std::vector<std::shared_ptr<MotionCommandPublisher>> controllers;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      controllers.reserve(connected_.size());
      for (const auto &entry : connected_) {
        if (entry.second.motion)
          controllers.push_back(entry.second.motion);
      }
    }
    for (const auto &controller : controllers)
      controller->Stop();
  }

  xgc2::adapter_runtime::SourceOpenDecision handleSourceOpen(
      const xgc::adapter::v1::SourceOpenRequest &request,
      const xgc2::adapter_runtime::CancellationToken &cancellation) {
    if (cancellation.IsCancellationRequested()) {
      return rejectSource(xgc::adapter::v1::ERROR_CLASS_CANCELLED,
                          "source-open-cancelled",
                          "telemetry source open was cancelled");
    }
    const auto &context = request.context();
    if (context.capability_id() !=
            xgc2_ros1_robot_adapter::kTelemetryCapability ||
        context.endpoint_id() != kTelemetryEndpoint) {
      return rejectSource(xgc::adapter::v1::ERROR_CLASS_REJECTED,
                          "invalid-telemetry-endpoint",
                          "unsupported Scout telemetry source endpoint");
    }

    xgc2_ros1_robot_adapter::RobotConfig robot;
    xgc2_ros1_robot_adapter::InstanceSpecFence fence;
    std::string robot_id;
    std::uint64_t source_generation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_configuration_) {
        return rejectSource(xgc::adapter::v1::ERROR_CLASS_REJECTED,
                            "instance-spec-unavailable",
                            "robot instance configuration is not applied");
      }
      if (context.spec_revision() != configuration_.fence.revision) {
        return rejectSource(xgc::adapter::v1::ERROR_CLASS_REJECTED,
                            "stale-source-spec",
                            "telemetry source does not match the current spec");
      }
      std::string subject_error;
      if (!xgc2_ros1_robot_adapter::ResolveRobotSubject(
              context, configuration_, &robot_id, &subject_error)) {
        return rejectSource(xgc::adapter::v1::ERROR_CLASS_REJECTED,
                            "invalid-robot-subject", subject_error);
      }
      if (connected_.find(robot_id) != connected_.end()) {
        return rejectSource(xgc::adapter::v1::ERROR_CLASS_REJECTED,
                            "robot-source-already-open",
                            "a telemetry source already owns this robot");
      }
      if (next_source_generation_ ==
          std::numeric_limits<std::uint64_t>::max()) {
        return rejectSource(xgc::adapter::v1::ERROR_CLASS_RESOURCE_EXHAUSTED,
                            "source-generation-exhausted",
                            "telemetry source generation is exhausted");
      }
      const auto found = std::find_if(
          configuration_.robots.begin(), configuration_.robots.end(),
          [&robot_id](const xgc2_ros1_robot_adapter::RobotConfig &candidate) {
            return candidate.robot_id == robot_id;
          });
      if (found == configuration_.robots.end()) {
        return rejectSource(xgc::adapter::v1::ERROR_CLASS_REJECTED,
                            "robot-config-unavailable",
                            "robot is absent from the current instance spec");
      }
      robot = *found;
      fence = configuration_.fence;
      source_generation = ++next_source_generation_;

      ConnectedRobot connected;
      connected.robot = robot;
      connected.work_id = context.work_id();
      connected.subject = context.subject();
      connected.fence = fence;
      connected.source_generation = source_generation;
      const auto inserted = connected_.emplace(robot_id, std::move(connected));
      if (!inserted.second) {
        return rejectSource(xgc::adapter::v1::ERROR_CLASS_REJECTED,
                            "robot-source-already-open",
                            "robot source reservation raced with another open");
      }
    }

    auto reservation =
        makeScopeExit([this, &robot_id, &context, &source_generation] {
          auto runtime =
              detachSource(robot_id, context.work_id(), source_generation);
          if (runtime)
            runtime->Stop();
        });

    if (cancellation.IsCancellationRequested()) {
      detachSource(robot_id, context.work_id(), source_generation);
      return rejectSource(xgc::adapter::v1::ERROR_CLASS_CANCELLED,
                          "source-open-cancelled",
                          "telemetry source open was cancelled");
    }

    std::string native_error;
    std::shared_ptr<RobotRuntime> runtime;
    std::shared_ptr<MotionCommandPublisher> motion;
    auto native_resource = makeScopeExit([&runtime, &motion] {
      if (motion)
        motion->Stop();
      if (runtime)
        runtime->Stop();
      motion.reset();
      runtime.reset();
    });
    runtime = RobotRuntime::Create(
        node_handle_, robot, fence.revision,
        [this, robot_id, source_generation](std::string item) {
          enqueueTelemetry(robot_id, source_generation, std::move(item));
        },
        &native_error);
    if (!runtime) {
      detachSource(robot_id, context.work_id(), source_generation);
      return rejectSource(xgc::adapter::v1::ERROR_CLASS_TRANSIENT,
                          "robot-native-source-open-failed", native_error);
    }
    if (channelEnabled(robot, "operation.motion-intent")) {
      std::string command_topic;
      if (!resolveMotionCommandTopic(robot, &command_topic, &native_error)) {
        detachSource(robot_id, context.work_id(), source_generation);
        return rejectSource(xgc::adapter::v1::ERROR_CLASS_PERMANENT,
                            "motion-command-topic-invalid", native_error);
      }
      motion = MotionCommandPublisher::Create(node_handle_, command_topic,
                                               &native_error);
      if (!motion) {
        detachSource(robot_id, context.work_id(), source_generation);
        return rejectSource(xgc::adapter::v1::ERROR_CLASS_TRANSIENT,
                            "motion-command-publisher-failed", native_error);
      }
    }

    bool installed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto found = connected_.find(robot_id);
      if (found != connected_.end() &&
          found->second.work_id == context.work_id() &&
          found->second.source_generation == source_generation &&
          found->second.fence.matches(fence) &&
          sameSubject(found->second.subject, context.subject())) {
        found->second.runtime = runtime;
        found->second.motion = motion;
        installed = true;
      }
    }
    if (!installed) {
      runtime->Stop();
      return rejectSource(xgc::adapter::v1::ERROR_CLASS_CANCELLED,
                          "source-open-invalidated",
                          "telemetry source was invalidated while opening");
    }
    if (cancellation.IsCancellationRequested()) {
      auto detached =
          detachSource(robot_id, context.work_id(), source_generation);
      if (detached)
        detached->Stop();
      return rejectSource(xgc::adapter::v1::ERROR_CLASS_CANCELLED,
                          "source-open-cancelled",
                          "telemetry source open was cancelled");
    }
    auto decision = xgc2::adapter_runtime::SourceOpenDecision::Accept();
    native_resource.release();
    reservation.release();
    return decision;
  }

  std::shared_ptr<RobotRuntime> detachSource(const std::string &robot_id,
                                             const std::string &work_id,
                                             std::uint64_t source_generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = connected_.find(robot_id);
    if (found == connected_.end() || found->second.work_id != work_id ||
        found->second.source_generation != source_generation) {
      return {};
    }
    auto runtime = std::move(found->second.runtime);
    if (found->second.motion)
      found->second.motion->Stop();
    queued_telemetry_items_ -= found->second.telemetry.size();
    queued_telemetry_bytes_ -= found->second.queued_bytes;
    connected_.erase(found);
    return runtime;
  }

  void enqueueTelemetry(const std::string &robot_id,
                        std::uint64_t source_generation, std::string item) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = connected_.find(robot_id);
    if (found == connected_.end() ||
        found->second.source_generation != source_generation) {
      return;
    }
    auto &source = found->second;
    const std::size_t item_bytes = item.size();
    if (item.empty() || item_bytes > kMaximumTelemetryBatchBytes ||
        item_bytes > kMaximumQueuedTelemetryBytesPerRobot) {
      ++source.dropped;
      return;
    }
    while (!source.telemetry.empty() &&
           (source.telemetry.size() >= kMaximumQueuedTelemetryItemsPerRobot ||
            source.queued_bytes >
                kMaximumQueuedTelemetryBytesPerRobot - item_bytes)) {
      --queued_telemetry_items_;
      queued_telemetry_bytes_ -= source.telemetry.front().value.size();
      source.queued_bytes -= source.telemetry.front().value.size();
      source.telemetry.pop_front();
      ++source.dropped;
    }
    if (source.telemetry.size() >= kMaximumQueuedTelemetryItemsPerRobot ||
        source.queued_bytes >
            kMaximumQueuedTelemetryBytesPerRobot - item_bytes ||
        queued_telemetry_items_ >= kMaximumQueuedTelemetryItems ||
        queued_telemetry_bytes_ > kMaximumQueuedTelemetryBytes - item_bytes ||
        next_telemetry_token_ == std::numeric_limits<std::uint64_t>::max()) {
      ++source.dropped;
      return;
    }
    TelemetryQueueItem queued;
    const std::uint64_t token = next_telemetry_token_ + 1u;
    queued.token = token;
    queued.value = std::move(item);
    source.telemetry.push_back(std::move(queued));
    next_telemetry_token_ = token;
    source.queued_bytes += item_bytes;
    ++queued_telemetry_items_;
    queued_telemetry_bytes_ += item_bytes;
  }

  void handleSourceClosed(const xgc::adapter::v1::SourceOpenRequest &request,
                          const xgc::adapter::v1::AdapterError &error) {
    std::shared_ptr<RobotRuntime> runtime;
    std::shared_ptr<MotionCommandPublisher> motion;
    bool matched = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto found = connected_.begin(); found != connected_.end();
           ++found) {
        if (found->second.work_id != request.context().work_id() ||
            found->second.fence.revision != request.context().spec_revision() ||
            !sameSubject(found->second.subject, request.context().subject())) {
          continue;
        }
        runtime = std::move(found->second.runtime);
        motion = std::move(found->second.motion);
        queued_telemetry_items_ -= found->second.telemetry.size();
        queued_telemetry_bytes_ -= found->second.queued_bytes;
        connected_.erase(found);
        matched = true;
        break;
      }
    }
    if (motion)
      motion->Stop();
    if (runtime)
      runtime->Stop();
    if (matched) {
      ROS_WARN_STREAM("Scout telemetry source "
                      << request.context().work_id()
                      << " closed: " << error.message());
    }
  }

  xgc2::adapter_runtime::OperationResult handleCommand(
      const xgc::adapter::v1::OperationRequest &request,
      const xgc2::adapter_runtime::CancellationToken &cancellation) {
    return executeMotionCommand(request.context(), request.input(),
                                cancellation, false);
  }

  xgc2::adapter_runtime::UnaryResult handleLeasePulse(
      const xgc::adapter::v1::UnaryRequest &request,
      const xgc2::adapter_runtime::CancellationToken &cancellation) {
    auto result = executeMotionCommand(request.context(), request.input(),
                                       cancellation, true);
    if (result.phase == xgc::adapter::v1::OPERATION_PHASE_SUCCEEDED &&
        result.has_output) {
      return xgc2::adapter_runtime::UnaryResult::Success(
          std::move(result.output));
    }
    return xgc2::adapter_runtime::UnaryResult::Failure(
        result.error.error_class(), result.error.code(), result.error.message(),
        result.error.retry_after_ms());
  }

  xgc2::adapter_runtime::OperationResult executeMotionCommand(
      const xgc::adapter::v1::WorkContext &context,
      const xgc::v1::Payload &wire_input,
      const xgc2::adapter_runtime::CancellationToken &cancellation,
      bool expected_lease_pulse) {
    if (wire_input.encoding() != xgc::v1::PAYLOAD_ENCODING_PROTOBUF) {
      return rejectOperation("invalid-command-encoding",
                             "Scout command input must use protobuf encoding");
    }
    if (cancellation.IsCancellationRequested())
      return xgc2::adapter_runtime::OperationResult::Cancelled();

    std::string error;
    std::string robot_id;
    xgc2_ros1_robot_adapter::RobotConfig robot;
    std::shared_ptr<MotionCommandPublisher> motion;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_configuration_) {
        return rejectOperation("instance-spec-unavailable",
                               "robot instance configuration is not applied");
      }
      if (!xgc2_ros1_robot_adapter::ResolveRobotSubject(
              context, configuration_, &robot_id, &error)) {
        return rejectOperation("invalid-robot-subject", error);
      }
      const auto found = connected_.find(robot_id);
      if (found == connected_.end() || !found->second.runtime ||
          !found->second.fence.matches(configuration_.fence) ||
          found->second.fence.revision != context.spec_revision() ||
          !sameSubject(found->second.subject, context.subject())) {
        return rejectOperation(
            "telemetry-source-not-open",
            "Scout command requires the current robot source to be open");
      }
      robot = found->second.robot;
      motion = found->second.motion;
    }
    if (!motion) {
      return rejectOperation("command-disabled",
                             "Scout motion control is disabled by robot spec");
    }

    ResolvedOperation operation;
    bool known_operation = false;
    const std::string &endpoint = context.endpoint_id();
    if (!resolveOperation(robot.profile_id, endpoint, &operation,
                          &known_operation, &error)) {
      if (!known_operation)
        return rejectOperation("unknown-command-endpoint", error);
      return xgc2::adapter_runtime::OperationResult::Failure(
          xgc::adapter::v1::ERROR_CLASS_PERMANENT,
          "operation-contract-invalid", error);
    }
    if (operation.lease_pulse != expected_lease_pulse) {
      return xgc2::adapter_runtime::OperationResult::Failure(
          xgc::adapter::v1::ERROR_CLASS_PERMANENT,
          "operation-contract-invalid",
          "Scout command interaction does not match its endpoint contract");
    }
    if (!channelEnabled(robot, operation.channel.channel_id)) {
      return rejectOperation("command-disabled",
                             endpoint + " is disabled by the robot spec");
    }
    if (!inputSchemaMatches(wire_input, operation)) {
      return rejectOperation(
          "invalid-command-schema",
          "Scout command schema does not match the installed operation");
    }
    if (context.volatile_() != operation.lease_pulse) {
      return xgc2::adapter_runtime::OperationResult::Failure(
          xgc::adapter::v1::ERROR_CLASS_PERMANENT,
          "operation-contract-invalid",
          "Scout lease pulse volatility does not match its endpoint contract");
    }

    const std::string processor(operation.channel.processor);
    if (processor != "scout-mini.set-motion-intent") {
      return xgc2::adapter_runtime::OperationResult::Failure(
          xgc::adapter::v1::ERROR_CLASS_PERMANENT,
          "operation-contract-invalid",
          "installed Scout operation processor has no native binding");
    }
    xgc::semantic::ground::v1::MotionIntentRequest input;
    if (operation.input_schema.type_name !=
        input.GetDescriptor()->full_name()) {
      return xgc2::adapter_runtime::OperationResult::Failure(
          xgc::adapter::v1::ERROR_CLASS_PERMANENT,
          "operation-contract-invalid",
          "Scout motion-intent input schema drifted");
    }
    if (!input.ParseFromString(wire_input.value())) {
      return rejectOperation("invalid-command-input",
                             "MotionIntentRequest is malformed");
    }
    if (input.gear() < 1u || input.gear() > 3u ||
        input.longitudinal() < -1 || input.longitudinal() > 1 ||
        input.yaw() < -1 || input.yaw() > 1) {
      return rejectOperation(
          "invalid-command-input",
          "gear must be 1..3 and longitudinal/yaw must be -1..1");
    }
    if (cancellation.IsCancellationRequested())
      return xgc2::adapter_runtime::OperationResult::Cancelled();
    ros::WallTime expires_at;
    const std::uint32_t lease_timeout_millis =
        operation.channel.operation_lease.timeout_millis;
    if (!operationDeadline(context, lease_timeout_millis,
                           &expires_at, &error)) {
      return xgc2::adapter_runtime::OperationResult::Failure(
          xgc::adapter::v1::ERROR_CLASS_DEADLINE,
          "command-deadline-exceeded", error);
    }
    const std::string command_owner = operationLeaseOwner(
        context.subject(), robot_id, operation.channel.operation_id);
    if (command_owner.empty())
      return rejectOperation("invalid-command-owner",
                             "Scout motion lease has no run owner");
    std::uint64_t command_generation = 0u;
    const bool inactive_pulse = operation.lease_pulse &&
                                input.longitudinal() == 0 && input.yaw() == 0;
    const bool applied =
        inactive_pulse
            ? motion->RevokeIntent(command_owner, &error)
            : operation.lease_pulse
                  ? motion->RenewIntent(command_owner, input.gear(),
                                        input.longitudinal(), input.yaw(),
                                        expires_at, &command_generation, &error)
                  : motion->SetIntent(command_owner, input.gear(),
                                      input.longitudinal(), input.yaw(),
                                      expires_at, &command_generation, &error);
    if (!applied) {
      if (operation.lease_pulse)
        return rejectOperation("motion-lease-pulse-rejected", error);
      return xgc2::adapter_runtime::OperationResult::Failure(
          xgc::adapter::v1::ERROR_CLASS_TRANSIENT,
          "motion-command-publication-failed", error);
    }
    if (cancellation.IsCancellationRequested()) {
      if (!inactive_pulse)
        motion->Release(command_owner, command_generation);
      return xgc2::adapter_runtime::OperationResult::Cancelled();
    }
    return xgc2_ros1_robot_adapter::EmptyOperationSuccess(
        operation.output_schema.version, operation.output_schema.fingerprint);
  }

  bool readTelemetryBatch(const std::string &robot_id,
                          std::size_t maximum_items,
                          TelemetrySnapshot *snapshot) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = connected_.find(robot_id);
    if (found == connected_.end() || !found->second.runtime ||
        found->second.telemetry.empty()) {
      return false;
    }
    snapshot->robot_id = robot_id;
    snapshot->work_id = found->second.work_id;
    snapshot->source_generation = found->second.source_generation;
    snapshot->batch =
        buildTelemetryBatch(found->second.telemetry, maximum_items);
    return !snapshot->batch.items.empty();
  }

  bool consumeTelemetryBatch(const TelemetrySnapshot &snapshot,
                             xgc2::adapter_runtime::SourceWriteResult result) {
    if (result != xgc2::adapter_runtime::SourceWriteResult::kAccepted &&
        result != xgc2::adapter_runtime::SourceWriteResult::kUnknownStream &&
        result != xgc2::adapter_runtime::SourceWriteResult::kTooLarge) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = connected_.find(snapshot.robot_id);
    if (found == connected_.end() ||
        found->second.work_id != snapshot.work_id ||
        found->second.source_generation != snapshot.source_generation ||
        !telemetryBatchMatchesPrefix(found->second.telemetry,
                                     snapshot.batch.tokens)) {
      return false;
    }
    auto &source = found->second;
    const std::size_t batch_size = snapshot.batch.tokens.size();
    for (std::size_t index = 0; index < batch_size; ++index) {
      --queued_telemetry_items_;
      queued_telemetry_bytes_ -= source.telemetry.front().value.size();
      source.queued_bytes -= source.telemetry.front().value.size();
      source.telemetry.pop_front();
    }
    if (result != xgc2::adapter_runtime::SourceWriteResult::kAccepted)
      source.dropped += batch_size;
    return true;
  }

  void flushTelemetry() {
    if (!client_)
      return;
    std::vector<std::string> robot_ids;
    std::size_t start = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      robot_ids.reserve(connected_.size());
      for (const auto &entry : connected_)
        robot_ids.push_back(entry.first);
      if (!robot_ids.empty()) {
        start = flush_cursor_ % robot_ids.size();
        flush_cursor_ = (start + 1) % robot_ids.size();
      }
    }
    if (robot_ids.empty())
      return;

    std::set<std::pair<std::string, std::uint64_t>> blocked;
    std::size_t handled = 0;
    while (handled < kMaximumFlushItemsPerTick) {
      bool made_progress = false;
      for (std::size_t offset = 0;
           offset < robot_ids.size() && handled < kMaximumFlushItemsPerTick;
           ++offset) {
        const std::size_t index = (start + offset) % robot_ids.size();
        TelemetrySnapshot snapshot;
        if (!readTelemetryBatch(robot_ids[index],
                                kMaximumFlushItemsPerTick - handled, &snapshot))
          continue;
        const auto source_key =
            std::make_pair(snapshot.robot_id, snapshot.source_generation);
        if (blocked.find(source_key) != blocked.end())
          continue;
        const auto result = client_->PublishSource(
            snapshot.work_id, std::move(snapshot.batch.items));
        if (result == xgc2::adapter_runtime::SourceWriteResult::kNoCredit ||
            result == xgc2::adapter_runtime::SourceWriteResult::kNotReady ||
            result == xgc2::adapter_runtime::SourceWriteResult::kQueueFull) {
          blocked.insert(source_key);
          continue;
        }
        if (consumeTelemetryBatch(snapshot, result)) {
          handled += snapshot.batch.tokens.size();
          made_progress = true;
        }
      }
      if (!made_progress)
        break;
      start = (start + 1) % robot_ids.size();
    }
  }

  void periodicTimer(const ros::WallTimerEvent &) {
    if (exitRequested())
      return;
    std::unique_lock<std::mutex> periodic_lock(periodic_mutex_,
                                               std::try_to_lock);
    if (!periodic_lock.owns_lock())
      return;
    std::vector<std::shared_ptr<RobotRuntime>> runtimes;
    std::vector<std::shared_ptr<MotionCommandPublisher>> controllers;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      runtimes.reserve(connected_.size());
      controllers.reserve(connected_.size());
      for (const auto &entry : connected_) {
        if (entry.second.runtime)
          runtimes.push_back(entry.second.runtime);
        if (entry.second.motion)
          controllers.push_back(entry.second.motion);
      }
    }
    for (const auto &controller : controllers)
      controller->PublishPeriodic();
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
  bool has_configuration_ = false;
  xgc2_ros1_robot_adapter::RobotAdapterConfig configuration_;
  std::map<std::string, ConnectedRobot> connected_;
  std::uint64_t next_source_generation_ = 0;
  std::uint64_t next_telemetry_token_ = 0;
  std::size_t queued_telemetry_items_ = 0;
  std::size_t queued_telemetry_bytes_ = 0;
  std::size_t flush_cursor_ = 0;
  // Keep the callback-owning client last so construction failures destroy and
  // stop it before any state captured by its callbacks.
  std::unique_ptr<xgc2::adapter_runtime::Client> client_;
};

} // namespace xgc_scout_mini_ros1_adapter

int main(int argc, char **argv) {
  std::string bootstrap_file;
  std::string error;
  if (!xgc2_ros1_robot_adapter::BootstrapFileFromArguments(
          argc, argv, &bootstrap_file, &error)) {
    std::cerr << "xgc_scout_mini_ros1_adapter: " << error << '\n';
    return 2;
  }
  ros::init(argc, argv, "xgc_scout_mini_ros1_adapter",
            ros::init_options::NoSigintHandler |
                ros::init_options::AnonymousName);
  ros::master::setRetryTimeout(ros::WallDuration(3.0));
  try {
    xgc_scout_mini_ros1_adapter::ShutdownSignalHandler shutdown_signals;
    xgc_scout_mini_ros1_adapter::ScoutMiniRos1AdapterNode node(
        ros::NodeHandle(), bootstrap_file);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    while (ros::ok() && !shutdown_signals.requested() &&
           !node.exitRequested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if ((shutdown_signals.requested() || node.exitRequested()) && ros::ok())
      ros::shutdown();
    ros::waitForShutdown();
    spinner.stop();
  } catch (const std::exception &exception) {
    ROS_FATAL_STREAM(
        "Scout robot Adapter startup failed: " << exception.what());
    return 1;
  }
  return 0;
}
