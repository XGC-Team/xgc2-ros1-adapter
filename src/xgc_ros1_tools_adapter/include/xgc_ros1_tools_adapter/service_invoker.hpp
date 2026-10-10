#pragma once

#include <json/json.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "xgc_ros1_tools_adapter/json_codec.hpp"
#include "xgc_ros1_tools_adapter/type_registry.hpp"

namespace xgc_ros1_tools_adapter {

struct ServiceCallRequest {
  std::string service;
  std::string service_type;
  Json::Value request;
  std::uint32_t wait_for_service_ms{5000};
  std::uint32_t call_timeout_ms{5000};
  std::int64_t deadline_unix_nanos{0};
  std::function<bool()> cancellation_requested;
};

struct ServiceCallResult {
  std::string service;
  std::string service_type;
  Json::Value response;
  std::uint64_t duration_ms{0};
};

class ServiceInvoker {
 public:
  ServiceInvoker(TypeRegistry& types, const JsonCodec& codec,
                 std::uint32_t maximum_in_flight = 16);

  ServiceCallResult call(const ServiceCallRequest& request);

 private:
  TypeRegistry& types_;
  const JsonCodec& codec_;
  std::uint32_t maximum_in_flight_;
  std::shared_ptr<std::atomic<std::uint32_t>> in_flight_;
};

}  // namespace xgc_ros1_tools_adapter
