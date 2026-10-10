#include "xgc_ros1_tools_adapter/publisher_registry.hpp"

#include <ros/master.h>
#include <ros/ros.h>

#include <chrono>
#include <thread>
#include <utility>

#include "xgc_ros1_tools_adapter/error.hpp"

namespace xgc_ros1_tools_adapter {
namespace {

std::int64_t currentMasterGeneration() {
  XmlRpc::XmlRpcValue request;
  XmlRpc::XmlRpcValue response;
  XmlRpc::XmlRpcValue payload;
  request.setSize(1);
  request[0] = "/xgc_ros1_tools_adapter_master_generation";
  if (!ros::master::execute("getPid", request, response, payload, false) ||
      payload.getType() != XmlRpc::XmlRpcValue::TypeInt ||
      static_cast<int>(payload) <= 0) {
    transientError("ros_master_generation_unavailable",
                   "ROS1 master process generation is unavailable");
  }
  return static_cast<int>(payload);
}

bool deadlineExpired(std::int64_t deadline_unix_nanos) {
  if (deadline_unix_nanos <= 0) {
    return false;
  }
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto now_nanos =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  return now_nanos >= deadline_unix_nanos;
}

bool cancellationRequested(const PublishRequest &request) {
  return request.cancellation_requested && request.cancellation_requested();
}

void requireDispatchAllowed(const PublishRequest &request) {
  if (cancellationRequested(request)) {
    cancelledError("publish_cancelled",
                   "ROS1 topic publish was cancelled before dispatch");
  }
  if (deadlineExpired(request.deadline_unix_nanos)) {
    deadlineError("publish_deadline_elapsed",
                  "ROS1 topic publish deadline elapsed before dispatch");
  }
}

void requireContinuationAllowed(const PublishRequest &request) {
  if (cancellationRequested(request)) {
    uncertainError("publish_cancelled_after_dispatch",
                   "ROS1 topic publish was cancelled after native dispatch");
  }
  if (deadlineExpired(request.deadline_unix_nanos)) {
    uncertainError(
        "publish_deadline_after_dispatch",
        "ROS1 topic publish crossed its deadline after native dispatch");
  }
}

void waitUntil(const PublishRequest &request,
               std::chrono::steady_clock::time_point target) {
  while (std::chrono::steady_clock::now() < target) {
    requireContinuationAllowed(request);
    const auto remaining = target - std::chrono::steady_clock::now();
    const auto quantum = std::chrono::milliseconds(5);
    std::this_thread::sleep_for(remaining < quantum ? remaining : quantum);
  }
}

}  // namespace

PublisherRegistry::PublisherRegistry(
    ros::NodeHandle node_handle, TypeRegistry &types, const JsonCodec &codec,
    std::size_t maximum_publishers,
    MasterGenerationProvider master_generation_provider)
    : node_handle_(std::move(node_handle)),
      types_(types),
      codec_(codec),
      maximum_publishers_(maximum_publishers),
      master_generation_provider_(master_generation_provider
                                      ? std::move(master_generation_provider)
                                      : currentMasterGeneration) {
  if (maximum_publishers_ == 0) {
    permanentError("invalid_configuration",
                   "maximum cached ROS1 publishers must be greater than zero");
  }
}

PublishResult PublisherRegistry::publish(const PublishRequest &request) {
  requireDispatchAllowed(request);
  refreshMasterGeneration();
  auto message = types_.createMessage(request.message_type);
  codec_.decode(request.message, *message);
  auto serialized = types_.serialize(message);
  if (!serialized) {
    permanentError("message_serialization_failed",
                   "dynamic ROS1 serializer returned no message");
  }

  ros::Publisher publisher;
  std::shared_ptr<Entry> selected_entry;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto iterator = entries_.find(request.topic);
    if (iterator == entries_.end()) {
      if (entries_.size() >= maximum_publishers_) {
        resourceExhaustedError(
            "publisher_capacity_exhausted",
            "maximum cached ROS1 publishers reached; replace or stop the "
            "Adapter instance before publishing another topic");
      }
      auto entry = std::make_shared<Entry>();
      entry->message_type = request.message_type;
      entry->latch = request.latch;
      entry->queue_size = request.queue_size;
      entry->publisher =
          types_.advertise(node_handle_, request.message_type, request.topic,
                           request.queue_size, request.latch);
      entry->active_calls = 1u;
      publisher = entry->publisher;
      selected_entry = entry;
      entries_.emplace(request.topic, std::move(entry));
    } else {
      const auto &entry = iterator->second;
      if (entry->message_type != request.message_type ||
          entry->latch != request.latch ||
          entry->queue_size != request.queue_size) {
        permanentError("publisher_configuration_conflict",
                       "topic '" + request.topic +
                           "' is already cached with a different type, latch, "
                           "or queueSize");
      }
      ++entry->active_calls;
      publisher = entry->publisher;
      selected_entry = entry;
    }
  }

  bool released = false;
  try {
    if (!publisher) {
      transientError(
          "publisher_unavailable",
          "ROS1 publisher is not available for topic '" + request.topic + "'");
    }

    if (request.wait_for_subscribers_ms > 0) {
      const auto deadline =
          std::chrono::steady_clock::now() +
          std::chrono::milliseconds(request.wait_for_subscribers_ms);
      while (publisher.getNumSubscribers() == 0 && ros::ok() &&
             std::chrono::steady_clock::now() < deadline) {
        requireDispatchAllowed(request);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      requireDispatchAllowed(request);
      if (publisher.getNumSubscribers() == 0) {
        transientError("subscriber_wait_timeout",
                       "no subscriber connected to topic '" + request.topic +
                           "' before waitForSubscribersMs elapsed");
      }
    }

    if (!ros::ok()) {
      transientError(
          "ros_shutdown",
          "ROS1 is shutting down before the message can be published");
    }
    PublishResult result;
    result.topic = request.topic;
    result.message_type = request.message_type;
    result.serialized_bytes = serialized->size();
    result.subscriber_count = publisher.getNumSubscribers();

    requireDispatchAllowed(request);
    // A throw from publish cannot prove that roscpp did not enqueue the
    // message, so entering this call is the native side-effect boundary. Retain
    // the cached publisher before crossing that boundary; failed pre-dispatch
    // leases are removed when their final active caller releases them.
    release(request.topic, selected_entry, true);
    released = true;
    const auto interval =
        std::chrono::duration<double>(1.0 / request.publish_rate_hz);
    auto next_publish = std::chrono::steady_clock::now();
    for (std::uint32_t index = 0; index < request.publish_count; ++index) {
      if (index > 0) {
        next_publish +=
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                interval);
        waitUntil(request, next_publish);
      }
      if (!ros::ok()) {
        uncertainError("ros_shutdown_after_dispatch",
                       "ROS1 shut down after one or more messages may have "
                       "been published");
      }
      requireContinuationAllowed(request);
      try {
        publisher.publish(*serialized);
      } catch (const std::exception &exception) {
        uncertainError("publish_dispatch_failed",
                       "ROS1 topic publish may have been dispatched: " +
                           std::string(exception.what()));
      } catch (...) {
        uncertainError("publish_dispatch_failed",
                       "ROS1 topic publish may have been dispatched");
      }
      ++result.published_count;
      requireContinuationAllowed(request);
    }

    result.subscriber_count = publisher.getNumSubscribers();
    return result;
  } catch (const Ros1ToolsError &) {
    if (!released) {
      release(request.topic, selected_entry, false);
    }
    throw;
  } catch (const std::exception &exception) {
    if (!released) {
      release(request.topic, selected_entry, false);
      throw;
    }
    uncertainError("publish_completion_failed",
                   "ROS1 topic publish committed but completion failed: " +
                       std::string(exception.what()));
  } catch (...) {
    if (!released) {
      release(request.topic, selected_entry, false);
      throw;
    }
    uncertainError("publish_completion_failed",
                   "ROS1 topic publish committed but completion failed");
  }
}

MasterBindingState PublisherRegistry::probeMasterBinding(
    std::int64_t *current_out) {
  std::int64_t current = 0;
  try {
    current = master_generation_provider_();
  } catch (...) {
    if (current_out != nullptr) {
      *current_out = 0;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    return master_generation_ == 0 ? MasterBindingState::Unbound
                                   : MasterBindingState::Unavailable;
  }
  if (current_out != nullptr) {
    *current_out = current;
  }
  if (current <= 0) {
    std::lock_guard<std::mutex> lock(mutex_);
    return master_generation_ == 0 ? MasterBindingState::Unbound
                                   : MasterBindingState::Unavailable;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (master_generation_ == 0) {
    master_generation_ = current;
    return MasterBindingState::Bound;
  }
  if (master_generation_ != current) {
    return MasterBindingState::Changed;
  }
  return MasterBindingState::Bound;
}

void PublisherRegistry::refreshMasterGeneration() {
  const std::int64_t current = master_generation_provider_();
  if (current <= 0) {
    transientError("ros_master_generation_unavailable",
                   "ROS1 master process generation is invalid");
  }
  std::map<std::string, std::shared_ptr<Entry>> stale;
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (master_generation_ == 0) {
      master_generation_ = current;
      return;
    }
    if (master_generation_ != current) {
      // roscpp binds ROS_MASTER_URI during ros::init. Recreating publishers on
      // this NodeHandle cannot re-register the node with a new master at the
      // same URI. Drop the stale cache and refuse dispatch so the process can
      // exit and Runtime can start a fresh generation.
      stale.swap(entries_);
      changed = true;
    }
  }
  if (changed) {
    transientError(
        "ros_master_generation_changed",
        "ROS1 master process generation changed; this Adapter process is "
        "bound to the previous master and must be recycled");
  }
}

void PublisherRegistry::release(const std::string &topic,
                                const std::shared_ptr<Entry> &entry,
                                bool native_dispatch_committed) {
  std::shared_ptr<Entry> removed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (entry->active_calls > 0u) {
      --entry->active_calls;
    }
    entry->native_dispatch_committed =
        entry->native_dispatch_committed || native_dispatch_committed;
    const auto iterator = entries_.find(topic);
    if (iterator != entries_.end() && iterator->second == entry &&
        entry->active_calls == 0u && !entry->native_dispatch_committed) {
      removed = std::move(iterator->second);
      entries_.erase(iterator);
    }
  }
}

void PublisherRegistry::clear() {
  std::map<std::string, std::shared_ptr<Entry>> removed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    removed.swap(entries_);
  }
  // Destroying the last ros::Publisher handle unregisters it from the ROS
  // graph. Do that outside the registry lock because roscpp may perform I/O.
}

std::size_t PublisherRegistry::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return entries_.size();
}

}  // namespace xgc_ros1_tools_adapter
