#pragma once
#include <atomic>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "xgc_ros1_tools_adapter/json_codec.hpp"
#include "xgc_ros1_tools_adapter/type_registry.hpp"

namespace xgc_ros1_tools_adapter {

struct PublishRequest {
  std::string topic;
  std::string message_type;
  Json::Value message;
  std::uint32_t publish_count{1};
  double publish_rate_hz{1.0};
  bool latch{false};
  std::uint32_t queue_size{10};
  std::uint32_t wait_for_subscribers_ms{0};
  std::int64_t deadline_unix_nanos{0};
  std::function<bool()> cancellation_requested;
};

struct PublishResult {
  std::string topic;
  std::string message_type;
  std::uint32_t serialized_bytes{0};
  std::uint32_t published_count{0};
  std::uint32_t subscriber_count{0};
};

class PublisherRegistry {
 public:
  PublisherRegistry(ros::NodeHandle node_handle, TypeRegistry &types,
                    const JsonCodec &codec, std::int64_t bound_master_pid,
                    const std::atomic<long> &observed_master_pid,
                    std::size_t maximum_publishers = 128);

  PublishResult publish(const PublishRequest &request);
  void clear();
  std::size_t size() const;

 private:
  struct Entry {
    std::string message_type;
    bool latch{false};
    std::uint32_t queue_size{0};
    ros::Publisher publisher;
    std::size_t active_calls{0};
    bool native_dispatch_committed{false};
    std::mutex connection_mutex;
    std::condition_variable connection_changed;
    std::uint64_t connection_version{0};
  };

  void release(const std::string &topic, const std::shared_ptr<Entry> &entry,
               bool native_dispatch_committed);
  void refreshMasterGeneration();

  ros::NodeHandle node_handle_;
  TypeRegistry &types_;
  const JsonCodec &codec_;
  const std::size_t maximum_publishers_;
  const std::atomic<long> &observed_master_pid_;
  const std::int64_t master_generation_;
  mutable std::mutex mutex_;
  std::map<std::string, std::shared_ptr<Entry>> entries_;
};

}  // namespace xgc_ros1_tools_adapter
