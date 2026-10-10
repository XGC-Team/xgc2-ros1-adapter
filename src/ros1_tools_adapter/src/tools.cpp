#include "xgc_ros1_tools_adapter/tools.hpp"

#include <arpa/inet.h>
#include <ros/names.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "xgc_ros1_tools_adapter/error.hpp"

namespace xgc_ros1_tools_adapter {
namespace {

constexpr std::uint32_t kMaximumQueueSize = 10000;
constexpr std::uint32_t kMaximumPublishCount = 10000;
constexpr double kMinimumPublishRateHz = 0.1;
constexpr double kMaximumPublishRateHz = 1000.0;
constexpr double kMaximumPublishDurationSeconds = 240.0;
constexpr std::uint32_t kMaximumWaitMilliseconds = 300000;
constexpr std::size_t kMaximumConfigurationBytes = 16u * 1024u;
constexpr std::size_t kMaximumMasterUriBytes = 2048;
constexpr std::size_t kMaximumNetworkIdentityBytes = 255;
constexpr std::size_t kMaximumRosNameBytes = 4096;
constexpr std::size_t kMaximumRosTypeBytes = 512;

const std::regex& rosTypePattern() {
  static const std::regex pattern(
      "^[A-Za-z][A-Za-z0-9_]*/[A-Za-z][A-Za-z0-9_]*$");
  return pattern;
}

const std::regex& absoluteRosNamePattern() {
  static const std::regex pattern(
      "^/[A-Za-z][A-Za-z0-9_]*(?:/[A-Za-z][A-Za-z0-9_]*)*$");
  return pattern;
}

const std::regex& masterUriPattern() {
  static const std::regex pattern(
      "^http://(?:[A-Za-z0-9._~-]+|\\[[0-9A-Fa-f:]+\\]):([0-9]{1,5})/?$");
  return pattern;
}


void rejectUnknownFields(const Json::Value& request,
                         const std::set<std::string>& allowed) {
  for (const auto& member : request.getMemberNames()) {
    if (allowed.count(member) == 0) {
      permanentError("invalid_request",
                     "unknown request field '" + member + "'");
    }
  }
}

void requireObject(const Json::Value& value, const std::string& field) {
  if (!value.isObject()) {
    permanentError("invalid_request", field + " must be a JSON object");
  }
}

std::string requiredString(const Json::Value& request,
                           const std::string& field) {
  if (!request.isMember(field) || !request[field].isString() ||
      request[field].asString().empty()) {
    permanentError("invalid_request", field + " must be a non-empty string");
  }
  return request[field].asString();
}

std::string requiredPossiblyEmptyString(const Json::Value& request,
                                        const std::string& field) {
  if (!request.isMember(field) || !request[field].isString()) {
    permanentError("invalid_configuration", field + " must be a string");
  }
  return request[field].asString();
}

bool requiredBool(const Json::Value& request, const std::string& field) {
  if (!request.isMember(field) || !request[field].isBool()) {
    permanentError("invalid_request", field + " must be a boolean");
  }
  return request[field].asBool();
}

std::uint32_t requiredUInt(const Json::Value& request, const std::string& field,
                           std::uint32_t minimum, std::uint32_t maximum) {
  if (!request.isMember(field)) {
    permanentError("invalid_request", field + " is required");
  }
  const Json::Value& value = request[field];
  if (!value.isIntegral() || (value.isInt64() && value.asInt64() < 0)) {
    permanentError("invalid_request", field + " must be an unsigned integer");
  }
  const Json::UInt64 parsed = value.asUInt64();
  if (parsed < minimum || parsed > maximum) {
    permanentError("invalid_request", field + " must be between " +
                                          std::to_string(minimum) + " and " +
                                          std::to_string(maximum));
  }
  return static_cast<std::uint32_t>(parsed);
}

double requiredNumber(const Json::Value& request, const std::string& field,
                      double minimum, double maximum) {
  if (!request.isMember(field) || !request[field].isNumeric()) {
    permanentError("invalid_request", field + " must be a number");
  }
  const double parsed = request[field].asDouble();
  if (!std::isfinite(parsed) || parsed < minimum || parsed > maximum) {
    permanentError("invalid_request", field + " must be between " +
                                          std::to_string(minimum) + " and " +
                                          std::to_string(maximum));
  }
  return parsed;
}

void validateAbsoluteRosName(const std::string& value,
                             const std::string& field) {
  if (value.empty() || value.size() > kMaximumRosNameBytes || value == "/" ||
      value.front() != '/' ||
      !std::regex_match(value, absoluteRosNamePattern())) {
    permanentError("invalid_ros_name",
                   field + " must be an absolute ROS1 graph name");
  }
  std::string validation_error;
  if (!ros::names::validate(value, validation_error)) {
    permanentError(
        "invalid_ros_name",
        field + " is not a valid ROS1 graph name: " + validation_error);
  }
}

void validateRosType(const std::string& value, const std::string& field) {
  if (value.size() > kMaximumRosTypeBytes ||
      !std::regex_match(value, rosTypePattern())) {
    permanentError("invalid_ros_type", field + " must use package/Type syntax");
  }
}

Json::Value parseStrictJson(const std::string& input,
                            const std::string& description) {
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  builder["allowComments"] = false;
  builder["allowTrailingCommas"] = false;
  builder["strictRoot"] = true;
  builder["rejectDupKeys"] = true;
  builder["stackLimit"] = 128;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  Json::Value value;
  std::string parse_errors;
  if (!reader->parse(input.data(), input.data() + input.size(), &value,
                     &parse_errors)) {
    permanentError("invalid_json",
                   description + " is not strict JSON: " + parse_errors);
  }
  return value;
}

std::string writeJson(const Json::Value& value) {
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  builder["commentStyle"] = "None";
  return Json::writeString(builder, value);
}

void validateNoControlCharacters(const std::string& value,
                                 const std::string& field) {
  if (std::any_of(value.begin(), value.end(), [](unsigned char character) {
        return character < 0x20 || character == 0x7f;
      })) {
    permanentError("invalid_configuration",
                   field + " must not contain control characters");
  }
}

void validateLength(const std::string& value, std::size_t maximum,
                    const std::string& field) {
  if (value.size() > maximum) {
    permanentError("invalid_configuration",
                   field + " exceeds its maximum encoded length");
  }
}

void validateIpAddress(const std::string& value) {
  if (value.empty()) {
    return;
  }
  in_addr ipv4{};
  in6_addr ipv6{};
  if (::inet_pton(AF_INET, value.c_str(), &ipv4) != 1 &&
      ::inet_pton(AF_INET6, value.c_str(), &ipv6) != 1) {
    permanentError("invalid_configuration",
                   "rosIp must be an IPv4 or IPv6 address");
  }
}

void validateHostname(const std::string& value) {
  if (value.empty()) {
    return;
  }
  std::size_t label_start = 0;
  const std::size_t effective_size =
      value.back() == '.' ? value.size() - 1 : value.size();
  if (effective_size == 0) {
    permanentError("invalid_configuration", "rosHostname is invalid");
  }
  while (label_start < effective_size) {
    const std::size_t separator = value.find('.', label_start);
    const std::size_t label_end =
        separator == std::string::npos ? effective_size : separator;
    const std::size_t label_size = label_end - label_start;
    const auto alpha_numeric = [](unsigned char character) {
      return std::isalnum(character) != 0;
    };
    if (label_size == 0 || label_size > 63 ||
        !alpha_numeric(static_cast<unsigned char>(value[label_start])) ||
        !alpha_numeric(static_cast<unsigned char>(value[label_end - 1])) ||
        !std::all_of(value.begin() + static_cast<std::ptrdiff_t>(label_start),
                     value.begin() + static_cast<std::ptrdiff_t>(label_end),
                     [](unsigned char character) {
                       return std::isalnum(character) != 0 ||
                              character == '-' || character == '_';
                     })) {
      permanentError("invalid_configuration",
                     "rosHostname must contain valid DNS-style labels");
    }
    if (separator == std::string::npos || separator >= effective_size) {
      break;
    }
    label_start = separator + 1;
  }
}

void setEnvironment(const char* name, const std::string& value) {
  const int result =
      value.empty() ? ::unsetenv(name) : ::setenv(name, value.c_str(), 1);
  if (result != 0) {
    throw std::runtime_error("unable to configure " + std::string(name) + ": " +
                             std::strerror(errno));
  }
}

}  // namespace

NativeContext NativeContext::FromJson(const std::string& json) {
  if (json.size() > kMaximumConfigurationBytes) {
    permanentError("invalid_configuration", "native context exceeds 16 KiB");
  }
  const Json::Value value = parseStrictJson(json, "native context");
  requireObject(value, "native context");
  rejectUnknownFields(value, {"rosMasterUri", "rosIp", "rosHostname"});

  NativeContext context;
  context.ros_master_uri = requiredString(value, "rosMasterUri");
  context.ros_ip = requiredPossiblyEmptyString(value, "rosIp");
  context.ros_hostname = requiredPossiblyEmptyString(value, "rosHostname");
  validateNoControlCharacters(context.ros_master_uri, "rosMasterUri");
  validateNoControlCharacters(context.ros_ip, "rosIp");
  validateNoControlCharacters(context.ros_hostname, "rosHostname");
  validateLength(context.ros_master_uri, kMaximumMasterUriBytes,
                 "rosMasterUri");
  validateLength(context.ros_ip, kMaximumNetworkIdentityBytes, "rosIp");
  validateLength(context.ros_hostname, kMaximumNetworkIdentityBytes,
                 "rosHostname");
  std::smatch master_uri_match;
  if (!std::regex_match(context.ros_master_uri, master_uri_match,
                        masterUriPattern())) {
    permanentError("invalid_configuration",
                   "rosMasterUri must be an absolute HTTP host and port");
  }
  const unsigned long master_port =
      std::stoul(master_uri_match[1].str(), nullptr, 10);
  if (master_port == 0 || master_port > 65535) {
    permanentError("invalid_configuration",
                   "rosMasterUri port must be between 1 and 65535");
  }
  if (!context.ros_ip.empty() && !context.ros_hostname.empty()) {
    permanentError("invalid_configuration",
                   "rosIp and rosHostname are mutually exclusive");
  }
  validateIpAddress(context.ros_ip);
  validateHostname(context.ros_hostname);

  return context;
}

void NativeContext::ApplyEnvironment() const {
  setEnvironment("ROS_MASTER_URI", ros_master_uri);
  setEnvironment("ROS_IP", ros_ip);
  setEnvironment("ROS_HOSTNAME", ros_hostname);
}

bool NativeContext::operator==(const NativeContext& other) const noexcept {
  return ros_master_uri == other.ros_master_uri && ros_ip == other.ros_ip &&
         ros_hostname == other.ros_hostname;
}

Tools::Tools(ros::NodeHandle node_handle,
                           NativeContext native_context)
    : native_context_(std::move(native_context)),
      codec_(),
      types_(),
      publishers_(node_handle, types_, codec_),
      services_(types_, codec_) {}

Json::Value Tools::Publish(const std::string& raw, const CallContext& call) {
  try {
    if (raw.size() > (1u << 20)) {
      resourceExhaustedError("request_too_large", "request exceeds 1 MiB");
    }
    const Json::Value input = parseStrictJson(raw, "request");
    requireObject(input, "request");
    rejectUnknownFields(
        input, {"topic", "messageType", "message", "publishCount",
                "publishRateHz", "latch", "queueSize", "waitForSubscribersMs"});
    PublishRequest publish_request;
    publish_request.topic = requiredString(input, "topic");
    publish_request.message_type = requiredString(input, "messageType");
    validateAbsoluteRosName(publish_request.topic, "topic");
    validateRosType(publish_request.message_type, "messageType");
    if (!input.isMember("message")) {
      permanentError("invalid_request", "message is required");
    }
    requireObject(input["message"], "message");
    publish_request.message = input["message"];
    publish_request.publish_count =
        requiredUInt(input, "publishCount", 1, kMaximumPublishCount);
    publish_request.publish_rate_hz = requiredNumber(
        input, "publishRateHz", kMinimumPublishRateHz, kMaximumPublishRateHz);
    if (static_cast<double>(publish_request.publish_count - 1u) /
            publish_request.publish_rate_hz >
        kMaximumPublishDurationSeconds) {
      permanentError(
          "invalid_request",
          "publishCount and publishRateHz must complete within 240 seconds");
    }
    publish_request.latch = requiredBool(input, "latch");
    publish_request.queue_size =
        requiredUInt(input, "queueSize", 1, kMaximumQueueSize);
    publish_request.wait_for_subscribers_ms = requiredUInt(
        input, "waitForSubscribersMs", 0, kMaximumWaitMilliseconds);
    publish_request.deadline_unix_nanos =
        call.deadline_unix_nanos;
    publish_request.cancellation_requested = call.cancellation_requested;

    const PublishResult published = publishers_.publish(publish_request);
    try {
      Json::Value result(Json::objectValue);
      result["event"] = "published";
      result["topic"] = published.topic;
      result["messageType"] = published.message_type;
      result["serializedBytes"] = Json::UInt(published.serialized_bytes);
      result["publishedCount"] = Json::UInt(published.published_count);
      result["subscriberCount"] = Json::UInt(published.subscriber_count);
      return success(result);
    } catch (const Ros1ToolsError&) {
      throw;
    } catch (const std::exception& exception) {
      uncertainError("publish_result_encoding_failed",
                     "ROS1 message was published but its result could not be "
                     "encoded: " +
                         std::string(exception.what()));
    } catch (...) {
      uncertainError("publish_result_encoding_failed",
                     "ROS1 message was published but its result could not be "
                     "encoded");
    }
  } catch (const std::exception& exception) {
    throw;
  }
}

MasterBindingState Tools::ProbeMasterBinding(std::int64_t* current) {
  return publishers_.probeMasterBinding(current);
}

Json::Value Tools::CallService(const std::string& raw, const CallContext& call) {
  try {
    if (raw.size() > (1u << 20)) {
      resourceExhaustedError("request_too_large", "request exceeds 1 MiB");
    }
    const Json::Value input = parseStrictJson(raw, "request");
    requireObject(input, "request");
    rejectUnknownFields(input, {"service", "serviceType", "request",
                                "waitForServiceMs", "callTimeoutMs"});
    ServiceCallRequest call_request;
    call_request.service = requiredString(input, "service");
    call_request.service_type = requiredString(input, "serviceType");
    validateAbsoluteRosName(call_request.service, "service");
    validateRosType(call_request.service_type, "serviceType");
    if (!input.isMember("request")) {
      permanentError("invalid_request", "request is required");
    }
    requireObject(input["request"], "request");
    call_request.request = input["request"];
    call_request.wait_for_service_ms =
        requiredUInt(input, "waitForServiceMs", 0, kMaximumWaitMilliseconds);
    call_request.call_timeout_ms =
        requiredUInt(input, "callTimeoutMs", 1, kMaximumWaitMilliseconds);
    call_request.deadline_unix_nanos =
        call.deadline_unix_nanos;
    call_request.cancellation_requested = call.cancellation_requested;

    const ServiceCallResult called = services_.call(call_request);
    try {
      Json::Value result(Json::objectValue);
      result["event"] = "response";
      result["service"] = called.service;
      result["serviceType"] = called.service_type;
      result["response"] = called.response;
      result["durationMs"] = Json::UInt64(called.duration_ms);
      return success(result);
    } catch (const Ros1ToolsError&) {
      throw;
    } catch (const std::exception& exception) {
      uncertainError("service_result_encoding_failed",
                     "ROS1 service executed but its result could not be "
                     "encoded: " +
                         std::string(exception.what()));
    } catch (...) {
      uncertainError("service_result_encoding_failed",
                     "ROS1 service executed but its result could not be "
                     "encoded");
    }
  } catch (const std::exception& exception) {
    throw;
  }
}

Json::Value Tools::success(const Json::Value& value) const {
  if (writeJson(value).size() > (1u << 20)) {
    uncertainError("operation_result_too_large", "native effect committed but result exceeds 1 MiB");
  }
  return value;
}

}  // namespace xgc_ros1_tools_adapter
