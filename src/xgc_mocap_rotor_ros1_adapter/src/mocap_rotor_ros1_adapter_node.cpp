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

#include "xgc2/adapter_runtime/client.hpp"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"
#include "xgc2_ros1_robot_adapter/runtime_support.hpp"
#include "xgc_mocap_rotor_ros1_adapter/generated_contract.hpp"
#include "xgc_mocap_rotor_ros1_adapter/robot_runtime.hpp"
#include "xgc_mocap_rotor_ros1_adapter/shutdown_signal.hpp"
#include "xgc_mocap_rotor_ros1_adapter/telemetry_batch.hpp"
#include "xgc_mocap_rotor_ros1_adapter/wire_contract.hpp"
#include "xgc_mocap_rotor_ros1_adapter/zenoh_subscriber.hpp"

namespace xgc_mocap_rotor_ros1_adapter {
namespace {

constexpr const char *kTelemetryEndpoint = "telemetry";
constexpr std::uint32_t kRobotAdapterSpecMessageId = 4001u;
constexpr std::size_t kMaximumQueuedTelemetryItemsPerRobot = 2048u;
constexpr std::size_t kMaximumQueuedTelemetryBytesPerRobot =
    16u * 1024u * 1024u;
constexpr std::size_t kMaximumQueuedTelemetryItems = 8192u;
constexpr std::size_t kMaximumQueuedTelemetryBytes = 64u * 1024u * 1024u;
constexpr std::size_t kMaximumFlushItemsPerTick = 256u;

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

xgc2::adapter_runtime::SourceOpenDecision
rejectSource(xgc::adapter::v1::ErrorClass error_class, const std::string &code,
             const std::string &message) {
  return xgc2::adapter_runtime::SourceOpenDecision::Reject(error_class, code,
                                                           message);
}

bool sameSubject(const xgc::adapter::v1::ScopeReference &left,
                 const xgc::adapter::v1::ScopeReference &right) {
  if (left.kind() != right.kind() || left.key() != right.key() ||
      left.attributes_size() != right.attributes_size())
    return false;
  for (const auto &attribute : left.attributes()) {
    const auto found = right.attributes().find(attribute.first);
    if (found == right.attributes().end() || found->second != attribute.second)
      return false;
  }
  return true;
}

} // namespace

class MocapRotorRos1AdapterNode {
public:
  MocapRotorRos1AdapterNode(ros::NodeHandle node_handle,
                            const std::string &bootstrap_file)
      : node_handle_(std::move(node_handle)),
        wire_(new ZenohSubscriber(
            [this](std::string key, std::string payload) {
              dispatchWire(std::move(key), std::move(payload));
            })) {
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

    std::string error;
    if (!xgc2_ros1_robot_adapter::BindBootstrapCapability(
            &config, xgc2_ros1_robot_adapter::kTelemetryCapability,
            std::move(telemetry), &error)) {
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
      exit_requested_.store(true, std::memory_order_release);
    };
    callbacks.session_lost = [this](const std::string &) {
      exit_requested_.store(true, std::memory_order_release);
    };
    callbacks.log = logFromClient;

    client_.reset(new xgc2::adapter_runtime::Client(std::move(config),
                                                    std::move(callbacks)));
    if (!client_->Start(&error))
      throw std::runtime_error("Adapter Runtime startup failed: " + error);

    periodic_timer_ = node_handle_.createWallTimer(
        ros::WallDuration(0.1),
        &MocapRotorRos1AdapterNode::periodicTimer, this);
  }

  ~MocapRotorRos1AdapterNode() { Shutdown(); }

  MocapRotorRos1AdapterNode(const MocapRotorRos1AdapterNode &) = delete;
  MocapRotorRos1AdapterNode &
  operator=(const MocapRotorRos1AdapterNode &) = delete;

  void Shutdown() {
    periodic_timer_.stop();
    if (client_) {
      client_->Stop();
      client_.reset();
    }
    clearInstanceSpec();
  }

  bool exitRequested() const noexcept {
    return exit_requested_.load(std::memory_order_acquire);
  }

private:
  struct ConnectedRobot {
    std::shared_ptr<RobotRuntime> runtime;
    xgc2_ros1_robot_adapter::RobotConfig robot;
    std::string work_id;
    xgc::adapter::v1::ScopeReference subject;
    xgc2_ros1_robot_adapter::InstanceSpecFence fence;
    std::uint64_t source_generation = 0u;
    std::deque<TelemetryQueueItem> telemetry;
    std::size_t queued_bytes = 0u;
    std::uint64_t dropped = 0u;
  };

  struct TelemetrySnapshot {
    std::string robot_id;
    std::string work_id;
    std::uint64_t source_generation = 0u;
    TelemetryBatch batch;
  };

  bool validateRobotConfig(const xgc2_ros1_robot_adapter::RobotConfig &robot,
                           std::string *error) const {
    if (!ValidateNativeProfileContract(error))
      return false;
    if (robot.profile_id != contract::kProfileId) {
      *error =
          "Mocap Rotor Adapter received unsupported profile " + robot.profile_id;
      return false;
    }
    const char *profile_digest = contract::profileDigest(robot.profile_id);
    if (profile_digest == nullptr || robot.profile_digest != profile_digest) {
      *error = "profile digest mismatch for robot " + robot.robot_id;
      return false;
    }
    if (robot.parameters.size() != 4u ||
        robot.parameters.find("namespace") == robot.parameters.end() ||
        robot.parameters.find("robot_id") == robot.parameters.end() ||
        robot.parameters.find("wire_transport") == robot.parameters.end() ||
        robot.parameters.find("zenoh_listen") == robot.parameters.end()) {
      *error = "robot " + robot.robot_id +
               " must contain exactly namespace, robot_id, wire_transport, zenoh_listen";
      return false;
    }
    if (robot.parameters.at("robot_id") != robot.robot_id) {
      *error = "wire robot_id must equal Adapter Runtime robot id";
      return false;
    }
    if (robot.parameters.at("wire_transport") != "zenoh") {
      *error = "Mocap Rotor requires Zenoh and has no fallback transport";
      return false;
    }
    std::string native_error;
    if (!ValidateRobotNamespace(robot.parameters.at("namespace"),
                                &native_error) ||
        !ValidateZenohListenEndpoint(robot.parameters.at("zenoh_listen"),
                                     &native_error)) {
      *error = "invalid native configuration for robot " + robot.robot_id +
               ": " + native_error;
      return false;
    }
    static const std::set<std::string> baseline{
        "state.pose",          "state.velocity", "state.speed",
        "state.imu",           "state.power",    "state.health",
        "state.flight",        "diagnostic.link",
        "diagnostic.stream-health"};
    std::set<std::string> enabled;
    for (const auto &channel : robot.channels) {
      contract::ChannelMetadata metadata{};
      if (!contract::channelMetadata(robot.profile_id, channel.channel_id,
                                     &metadata)) {
        *error = "Mocap Rotor robot " + robot.robot_id +
                 " contains unknown channel " + channel.channel_id;
        return false;
      }
      if (metadata.kind != contract::ChannelKind::kStreamOut ||
          metadata.operation_timeout_millis != 0u) {
        *error = "Mocap Rotor profile must remain telemetry-only";
        return false;
      }
      if (channel.enabled)
        enabled.insert(channel.channel_id);
    }
    for (const auto &channel : baseline) {
      if (enabled.count(channel) == 0u) {
        *error = "Mocap Rotor baseline channel must be enabled: " + channel;
        return false;
      }
    }
    return true;
  }

  bool applyInstanceSpec(const xgc::adapter::v1::AdapterInstanceSpec &spec,
                         std::string *error) {
    contract::MessageMetadata metadata{};
    if (!contract::messageMetadata(kRobotAdapterSpecMessageId, &metadata)) {
      *error = "installed contract does not contain RobotAdapterSpec schema";
      return false;
    }
    xgc2_ros1_robot_adapter::MessageSchema expected_schema{
        kRobotAdapterSpecMessageId, "xgc.robot.v1.RobotAdapterSpec",
        metadata.version, metadata.fingerprint};
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
    std::string shared_listen;
    for (const auto &robot : candidate.robots) {
      if (!validateRobotConfig(robot, error))
        return false;
      const std::string &listen = robot.parameters.at("zenoh_listen");
      if (shared_listen.empty())
        shared_listen = listen;
      else if (listen != shared_listen) {
        *error = "one Mocap Rotor Adapter instance must use one shared Zenoh listener";
        return false;
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!connected_.empty()) {
      *error = "instance spec cannot change while robot sources are active";
      return false;
    }
    configuration_ = std::move(candidate);
    zenoh_listen_ = std::move(shared_listen);
    has_configuration_ = true;
    return true;
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
                          "unsupported Mocap Rotor telemetry endpoint");
    }

    xgc2_ros1_robot_adapter::RobotConfig robot;
    xgc2_ros1_robot_adapter::InstanceSpecFence fence;
    std::string robot_id;
    std::string listen;
    std::uint64_t source_generation = 0u;
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
      listen = zenoh_listen_;
      source_generation = ++next_source_generation_;
      ConnectedRobot connected;
      connected.robot = robot;
      connected.work_id = context.work_id();
      connected.subject = context.subject();
      connected.fence = fence;
      connected.source_generation = source_generation;
      connected_.emplace(robot_id, std::move(connected));
    }

    if (cancellation.IsCancellationRequested()) {
      detachSource(robot_id, context.work_id(), source_generation);
      return rejectSource(xgc::adapter::v1::ERROR_CLASS_CANCELLED,
                          "source-open-cancelled",
                          "telemetry source open was cancelled");
    }

    std::string native_error;
    auto runtime = RobotRuntime::Create(
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
        installed = true;
      }
    }
    if (!installed) {
      runtime->Stop();
      return rejectSource(xgc::adapter::v1::ERROR_CLASS_CANCELLED,
                          "source-open-invalidated",
                          "telemetry source was invalidated while opening");
    }

    {
      std::lock_guard<std::mutex> wire_lock(wire_mutex_);
      if (!wire_->running() && !wire_->Start(listen, &native_error)) {
        auto detached =
            detachSource(robot_id, context.work_id(), source_generation);
        if (detached)
          detached->Stop();
        return rejectSource(xgc::adapter::v1::ERROR_CLASS_TRANSIENT,
                            "zenoh-listener-open-failed", native_error);
      }
    }
    if (cancellation.IsCancellationRequested()) {
      auto detached = detachSource(robot_id, context.work_id(), source_generation);
      if (detached)
        detached->Stop();
      stopWireIfUnused();
      return rejectSource(xgc::adapter::v1::ERROR_CLASS_CANCELLED,
                          "source-open-cancelled",
                          "telemetry source open was cancelled");
    }
    return xgc2::adapter_runtime::SourceOpenDecision::Accept();
  }

  std::shared_ptr<RobotRuntime>
  detachSource(const std::string &robot_id, const std::string &work_id,
               std::uint64_t source_generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = connected_.find(robot_id);
    if (found == connected_.end() || found->second.work_id != work_id ||
        found->second.source_generation != source_generation)
      return {};
    auto runtime = std::move(found->second.runtime);
    queued_telemetry_items_ -= found->second.telemetry.size();
    queued_telemetry_bytes_ -= found->second.queued_bytes;
    connected_.erase(found);
    return runtime;
  }

  void handleSourceClosed(const xgc::adapter::v1::SourceOpenRequest &request,
                          const xgc::adapter::v1::AdapterError &error) {
    std::shared_ptr<RobotRuntime> runtime;
    bool matched = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto found = connected_.begin(); found != connected_.end(); ++found) {
        if (found->second.work_id != request.context().work_id() ||
            found->second.fence.revision !=
                request.context().spec_revision() ||
            !sameSubject(found->second.subject, request.context().subject()))
          continue;
        runtime = std::move(found->second.runtime);
        queued_telemetry_items_ -= found->second.telemetry.size();
        queued_telemetry_bytes_ -= found->second.queued_bytes;
        connected_.erase(found);
        matched = true;
        break;
      }
    }
    if (runtime)
      runtime->Stop();
    stopWireIfUnused();
    if (matched) {
      ROS_WARN_STREAM("Mocap Rotor telemetry source "
                      << request.context().work_id()
                      << " closed: " << error.message());
    }
  }

  void stopSources() {
    std::map<std::string, ConnectedRobot> removed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      removed.swap(connected_);
      queued_telemetry_items_ = 0u;
      queued_telemetry_bytes_ = 0u;
      flush_cursor_ = 0u;
    }
    stopWire();
    for (auto &entry : removed) {
      if (entry.second.runtime)
        entry.second.runtime->Stop();
    }
  }

  void clearInstanceSpec() {
    stopSources();
    std::lock_guard<std::mutex> lock(mutex_);
    configuration_ = {};
    zenoh_listen_.clear();
    has_configuration_ = false;
  }

  void stopWireIfUnused() {
    bool unused = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      unused = connected_.empty();
    }
    if (unused)
      stopWire();
  }

  void stopWire() {
    std::lock_guard<std::mutex> wire_lock(wire_mutex_);
    if (wire_)
      wire_->Stop();
  }

  void dispatchWire(std::string key, std::string payload) {
    ParsedWireKey parsed;
    std::string error;
    if (!ParseWireKey(key, &parsed, &error)) {
      ROS_WARN_STREAM_THROTTLE(2.0, "Mocap Rotor rejected Zenoh key: " << error);
      return;
    }
    std::shared_ptr<RobotRuntime> runtime;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto found = connected_.find(parsed.robot_id);
      if (found != connected_.end())
        runtime = found->second.runtime;
    }
    if (!runtime)
      return;
    if (!runtime->HandleWireFrame(parsed.channel, payload, &error)) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "Mocap Rotor wire sample rejected robot="
                   << parsed.robot_id << " channel="
                   << WireChannelLeaf(parsed.channel) << ": " << error);
    }
  }

  void enqueueTelemetry(const std::string &robot_id,
                        std::uint64_t source_generation, std::string item) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = connected_.find(robot_id);
    if (found == connected_.end() ||
        found->second.source_generation != source_generation)
      return;
    auto &source = found->second;
    const std::size_t item_bytes = item.size();
    if (item.empty() || item_bytes > kMaximumTelemetryBatchBytes ||
        item_bytes > kMaximumQueuedTelemetryBytesPerRobot) {
      ++source.dropped;
      return;
    }
    while (!source.telemetry.empty() &&
           (source.telemetry.size() >=
                kMaximumQueuedTelemetryItemsPerRobot ||
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
        next_telemetry_token_ ==
            std::numeric_limits<std::uint64_t>::max()) {
      ++source.dropped;
      return;
    }
    TelemetryQueueItem queued;
    queued.token = ++next_telemetry_token_;
    queued.value = std::move(item);
    source.queued_bytes += queued.value.size();
    ++queued_telemetry_items_;
    queued_telemetry_bytes_ += queued.value.size();
    source.telemetry.push_back(std::move(queued));
  }

  bool readTelemetryBatch(const std::string &robot_id,
                          std::size_t maximum_items,
                          TelemetrySnapshot *snapshot) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = connected_.find(robot_id);
    if (found == connected_.end() || !found->second.runtime ||
        found->second.telemetry.empty())
      return false;
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
        result != xgc2::adapter_runtime::SourceWriteResult::kTooLarge)
      return false;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = connected_.find(snapshot.robot_id);
    if (found == connected_.end() ||
        found->second.work_id != snapshot.work_id ||
        found->second.source_generation != snapshot.source_generation ||
        !telemetryBatchMatchesPrefix(found->second.telemetry,
                                     snapshot.batch.tokens))
      return false;
    auto &source = found->second;
    for (std::size_t index = 0u; index < snapshot.batch.tokens.size(); ++index) {
      --queued_telemetry_items_;
      queued_telemetry_bytes_ -= source.telemetry.front().value.size();
      source.queued_bytes -= source.telemetry.front().value.size();
      source.telemetry.pop_front();
    }
    if (result != xgc2::adapter_runtime::SourceWriteResult::kAccepted)
      source.dropped += snapshot.batch.tokens.size();
    return true;
  }

  void flushTelemetry() {
    if (!client_)
      return;
    std::vector<std::string> robot_ids;
    std::size_t start = 0u;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto &entry : connected_)
        robot_ids.push_back(entry.first);
      if (!robot_ids.empty()) {
        start = flush_cursor_ % robot_ids.size();
        flush_cursor_ = (start + 1u) % robot_ids.size();
      }
    }
    if (robot_ids.empty())
      return;
    std::set<std::pair<std::string, std::uint64_t>> blocked;
    std::size_t handled = 0u;
    while (handled < kMaximumFlushItemsPerTick) {
      bool progress = false;
      for (std::size_t offset = 0u;
           offset < robot_ids.size() && handled < kMaximumFlushItemsPerTick;
           ++offset) {
        const std::size_t index = (start + offset) % robot_ids.size();
        TelemetrySnapshot snapshot;
        if (!readTelemetryBatch(robot_ids[index],
                                kMaximumFlushItemsPerTick - handled,
                                &snapshot))
          continue;
        const auto source_key =
            std::make_pair(snapshot.robot_id, snapshot.source_generation);
        if (blocked.count(source_key) != 0u)
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
          progress = true;
        }
      }
      if (!progress)
        break;
      start = (start + 1u) % robot_ids.size();
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
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto &entry : connected_) {
        if (entry.second.runtime)
          runtimes.push_back(entry.second.runtime);
      }
    }
    const ros::WallTime now = ros::WallTime::now();
    for (const auto &runtime : runtimes)
      runtime->EmitPeriodic(now);
    flushTelemetry();
  }

  ros::NodeHandle node_handle_;
  ros::WallTimer periodic_timer_;
  std::string definition_id_;
  std::atomic<bool> exit_requested_{false};

  mutable std::mutex mutex_;
  std::mutex periodic_mutex_;
  std::mutex wire_mutex_;
  bool has_configuration_ = false;
  xgc2_ros1_robot_adapter::RobotAdapterConfig configuration_;
  std::string zenoh_listen_;
  std::map<std::string, ConnectedRobot> connected_;
  std::uint64_t next_source_generation_ = 0u;
  std::uint64_t next_telemetry_token_ = 0u;
  std::size_t queued_telemetry_items_ = 0u;
  std::size_t queued_telemetry_bytes_ = 0u;
  std::size_t flush_cursor_ = 0u;
  std::unique_ptr<ZenohSubscriber> wire_;
  // Keep the callback-owning client last so construction failures destroy it
  // before state captured by its callbacks.
  std::unique_ptr<xgc2::adapter_runtime::Client> client_;
};

} // namespace xgc_mocap_rotor_ros1_adapter

int main(int argc, char **argv) {
  std::string bootstrap_file;
  std::string error;
  if (!xgc2_ros1_robot_adapter::BootstrapFileFromArguments(
          argc, argv, &bootstrap_file, &error)) {
    std::cerr << "xgc_mocap_rotor_ros1_adapter: " << error << '\n';
    return 2;
  }
  ros::init(argc, argv, "xgc_mocap_rotor_ros1_adapter",
            ros::init_options::NoSigintHandler |
                ros::init_options::AnonymousName);
  ros::master::setRetryTimeout(ros::WallDuration(3.0));
  try {
    xgc_mocap_rotor_ros1_adapter::ShutdownSignalHandler shutdown_signals;
    xgc_mocap_rotor_ros1_adapter::MocapRotorRos1AdapterNode node(
        ros::NodeHandle(), bootstrap_file);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    while (ros::ok() && !shutdown_signals.requested() &&
           !node.exitRequested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    node.Shutdown();
    if ((shutdown_signals.requested() || node.exitRequested()) && ros::ok()) {
      ros::WallDuration(0.05).sleep();
      ros::shutdown();
    }
    ros::waitForShutdown();
    spinner.stop();
  } catch (const std::exception &exception) {
    ROS_FATAL_STREAM("Mocap Rotor Adapter startup failed: "
                     << exception.what());
    return 1;
  }
  return 0;
}
