#include "xgc_ros1_adapter/robot_runtime.hpp"

#include <cstdlib>

namespace xgc_ros1_adapter {

std::string trimSlash(const std::string& value) {
  std::size_t begin = 0;
  std::size_t end = value.size();
  while (begin < end && value[begin] == '/') {
    ++begin;
  }
  while (end > begin && value[end - 1] == '/') {
    --end;
  }
  return value.substr(begin, end - begin);
}

std::string topic(const std::string& ns, const std::string& name) {
  const std::string clean_ns = trimSlash(ns);
  if (clean_ns.empty()) {
    return "/" + trimSlash(name);
  }
  return "/" + clean_ns + "/" + trimSlash(name);
}

void setTopicIfPresent(const google::protobuf::Map<std::string, std::string>& topics,
                       const std::string& key,
                       std::string& target) {
  const auto it = topics.find(key);
  if (it != topics.end() && !it->second.empty()) {
    target = it->second;
  }
}

void setServiceIfPresent(
    const google::protobuf::Map<std::string, xgc::adapter::v1::ServiceBinding>& services,
    const std::string& key,
    std::string& target) {
  const auto it = services.find(key);
  if (it != services.end() && !it->second.service().empty()) {
    target = it->second.service();
  }
}

double fieldDouble(const google::protobuf::Map<std::string, std::string>& fields,
                   const std::string& key,
                   double fallback) {
  const auto it = fields.find(key);
  if (it == fields.end()) {
    return fallback;
  }
  char* end = nullptr;
  const double parsed = std::strtod(it->second.c_str(), &end);
  return end != it->second.c_str() ? parsed : fallback;
}

bool fieldBool(const google::protobuf::Map<std::string, std::string>& fields,
               const std::string& key,
               bool fallback) {
  const auto it = fields.find(key);
  if (it == fields.end()) {
    return fallback;
  }
  const std::string value = it->second;
  return value == "true" || value == "1" || value == "yes" || value == "on";
}

std::string fieldString(const google::protobuf::Map<std::string, std::string>& fields,
                        const std::string& key,
                        const std::string& fallback) {
  const auto it = fields.find(key);
  if (it == fields.end() || it->second.empty()) {
    return fallback;
  }
  return it->second;
}

}  // namespace xgc_ros1_adapter
