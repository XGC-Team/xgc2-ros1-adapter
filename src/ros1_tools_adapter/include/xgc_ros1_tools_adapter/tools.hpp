#pragma once
#include <json/json.h>
#include <ros/node_handle.h>
#include <cstdint>
#include <functional>
#include <string>
#include "xgc_ros1_tools_adapter/json_codec.hpp"
#include "xgc_ros1_tools_adapter/publisher_registry.hpp"
#include "xgc_ros1_tools_adapter/service_invoker.hpp"
#include "xgc_ros1_tools_adapter/type_registry.hpp"

namespace xgc_ros1_tools_adapter {
struct NativeContext {
  std::string ros_master_uri, ros_ip, ros_hostname;
  static NativeContext FromJson(const std::string& json);
  void ApplyEnvironment() const;
  bool operator==(const NativeContext& other) const noexcept;
};
struct CallContext {
  std::int64_t deadline_unix_nanos;
  std::function<bool()> cancellation_requested;
};
class Tools {
 public:
  Tools(ros::NodeHandle node_handle, NativeContext context);
  Json::Value Publish(const std::string& input, const CallContext& call);
  Json::Value CallService(const std::string& input, const CallContext& call);
  MasterBindingState ProbeMasterBinding(std::int64_t* current = nullptr);
 private:
  Json::Value success(const Json::Value& value) const;
  NativeContext native_context_;
  JsonCodec codec_;
  TypeRegistry types_;
  PublisherRegistry publishers_;
  ServiceInvoker services_;
};
} // namespace xgc_ros1_tools_adapter
