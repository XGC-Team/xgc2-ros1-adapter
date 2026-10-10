#include "xgc_ros1_tools_adapter/json_codec.hpp"

#include <ros/time.h>
#include <ros_babel_fish/messages/array_message.h>
#include <ros_babel_fish/messages/compound_message.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

#include "xgc_ros1_tools_adapter/error.hpp"

namespace xgc_ros1_tools_adapter {
namespace {

using ros_babel_fish::ArrayMessage;
using ros_babel_fish::ArrayMessageBase;
using ros_babel_fish::CompoundArrayMessage;
using ros_babel_fish::CompoundMessage;
using ros_babel_fish::Message;
using ros_babel_fish::MessageType;
using namespace ros_babel_fish::MessageTypes;

[[noreturn]] void codecError(const std::string& path,
                             const std::string& message) {
  permanentError("invalid_message_payload", path + ": " + message);
}

void checkDepth(std::size_t depth, std::size_t maximum,
                const std::string& path) {
  if (depth > maximum) {
    codecError(path, "maximum message nesting depth exceeded");
  }
}

std::string integerText(const Json::Value& value, const std::string& path) {
  if (value.isString()) {
    const std::string text = value.asString();
    if (text.empty()) {
      codecError(path, "integer string must not be empty");
    }
    const std::size_t first_digit = text.front() == '-' ? 1u : 0u;
    if (first_digit == text.size() || text[first_digit] < '0' ||
        text[first_digit] > '9' ||
        (text[first_digit] == '0' && text.size() != first_digit + 1u)) {
      codecError(path, "integer string must use canonical decimal syntax");
    }
    for (std::size_t index = first_digit + 1u; index < text.size(); ++index) {
      if (text[index] < '0' || text[index] > '9') {
        codecError(path, "integer string must use canonical decimal syntax");
      }
    }
    if (text == "-0") {
      codecError(path, "integer string must use canonical decimal syntax");
    }
    return text;
  }
  if (value.isIntegral()) {
    if (value.isUInt64()) {
      return std::to_string(value.asUInt64());
    }
    return std::to_string(value.asInt64());
  }
  codecError(path, "expected an integer or decimal integer string");
}

template <typename T>
T signedInteger(const Json::Value& value, const std::string& path) {
  static_assert(std::is_integral<T>::value && std::is_signed<T>::value,
                "signed integer type required");
  const std::string text = integerText(value, path);
  std::size_t consumed = 0;
  long long parsed = 0;
  try {
    parsed = std::stoll(text, &consumed, 10);
  } catch (const std::exception&) {
    codecError(path, "signed integer is outside the supported range");
  }
  if (consumed != text.size() ||
      parsed < static_cast<long long>(std::numeric_limits<T>::min()) ||
      parsed > static_cast<long long>(std::numeric_limits<T>::max())) {
    codecError(path, "signed integer is outside the ROS field range");
  }
  return static_cast<T>(parsed);
}

template <typename T>
T unsignedInteger(const Json::Value& value, const std::string& path) {
  static_assert(std::is_integral<T>::value && std::is_unsigned<T>::value,
                "unsigned integer type required");
  const std::string text = integerText(value, path);
  if (!text.empty() && text.front() == '-') {
    codecError(path, "unsigned integer must not be negative");
  }
  std::size_t consumed = 0;
  unsigned long long parsed = 0;
  try {
    parsed = std::stoull(text, &consumed, 10);
  } catch (const std::exception&) {
    codecError(path, "unsigned integer is outside the supported range");
  }
  if (consumed != text.size() ||
      parsed > static_cast<unsigned long long>(std::numeric_limits<T>::max())) {
    codecError(path, "unsigned integer is outside the ROS field range");
  }
  return static_cast<T>(parsed);
}

double floatingPoint(const Json::Value& value, const std::string& path) {
  if (!value.isNumeric()) {
    codecError(path, "expected a finite JSON number");
  }
  const double result = value.asDouble();
  if (!std::isfinite(result)) {
    codecError(path, "non-finite floating point values are not supported");
  }
  return result;
}

bool booleanValue(const Json::Value& value, const std::string& path) {
  if (!value.isBool()) {
    codecError(path, "expected a boolean");
  }
  return value.asBool();
}

std::string stringValue(const Json::Value& value, const std::string& path) {
  if (!value.isString()) {
    codecError(path, "expected a string");
  }
  return value.asString();
}

void rejectUnknownMembers(const Json::Value& value,
                          const std::set<std::string>& allowed,
                          const std::string& path) {
  for (const auto& member : value.getMemberNames()) {
    if (allowed.count(member) == 0) {
      codecError(path + "." + member, "unknown field");
    }
  }
}

ros::Time timeValue(const Json::Value& value, const std::string& path) {
  if (!value.isObject()) {
    codecError(path, "expected {secs, nsecs} for ROS time");
  }
  rejectUnknownMembers(value, {"secs", "nsecs"}, path);
  if (!value.isMember("secs") || !value.isMember("nsecs")) {
    codecError(path, "ROS time requires secs and nsecs");
  }
  const auto secs =
      unsignedInteger<std::uint32_t>(value["secs"], path + ".secs");
  const auto nsecs =
      unsignedInteger<std::uint32_t>(value["nsecs"], path + ".nsecs");
  if (nsecs >= 1000000000U) {
    codecError(path + ".nsecs", "ROS time nanoseconds must be below 1e9");
  }
  return ros::Time(secs, nsecs);
}

ros::Duration durationValue(const Json::Value& value, const std::string& path) {
  if (!value.isObject()) {
    codecError(path, "expected {secs, nsecs} for ROS duration");
  }
  rejectUnknownMembers(value, {"secs", "nsecs"}, path);
  if (!value.isMember("secs") || !value.isMember("nsecs")) {
    codecError(path, "ROS duration requires secs and nsecs");
  }
  const auto secs = signedInteger<std::int32_t>(value["secs"], path + ".secs");
  const auto nsecs =
      signedInteger<std::int32_t>(value["nsecs"], path + ".nsecs");
  if (nsecs <= -1000000000 || nsecs >= 1000000000) {
    codecError(path + ".nsecs",
               "ROS duration nanoseconds must be between -1e9 and 1e9");
  }
  return ros::Duration(secs, nsecs);
}

void validateArrayInput(const Json::Value& value, const ArrayMessageBase& array,
                        std::size_t maximum_elements, const std::string& path) {
  if (!value.isArray()) {
    codecError(path, "expected an array");
  }
  const std::size_t size = value.size();
  if (size > maximum_elements) {
    codecError(path, "maximum array element count exceeded");
  }
  if (array.isFixedSize() && size != array.length()) {
    codecError(path, "fixed ROS array requires exactly " +
                         std::to_string(array.length()) + " elements");
  }
  if (!array.isFixedSize() && array.length() != 0) {
    codecError(path, "dynamic ROS array target must be empty before decoding");
  }
}

template <typename T, typename Converter>
void decodePrimitiveArray(const Json::Value& value, Message& message,
                          std::size_t maximum_elements, const std::string& path,
                          Converter converter) {
  auto& array = message.as<ArrayMessage<T>>();
  validateArrayInput(value, array, maximum_elements, path);
  if (!array.isFixedSize()) {
    array.reserve(value.size());
  }
  for (Json::ArrayIndex index = 0; index < value.size(); ++index) {
    const std::string item_path = path + "[" + std::to_string(index) + "]";
    const T item = converter(value[index], item_path);
    if (array.isFixedSize()) {
      array.assign(index, item);
    } else {
      array.append(item);
    }
  }
}

template <typename T, typename Encoder>
Json::Value encodePrimitiveArray(const Message& message,
                                 const std::string& path, Encoder encoder) {
  const auto& array = message.as<ArrayMessage<T>>();
  Json::Value result(Json::arrayValue);
  for (std::size_t index = 0; index < array.length(); ++index) {
    result.append(
        encoder(array[index], path + "[" + std::to_string(index) + "]"));
  }
  return result;
}

Json::Value encodeTime(const ros::Time& value) {
  Json::Value result(Json::objectValue);
  result["secs"] = Json::UInt(value.sec);
  result["nsecs"] = Json::UInt(value.nsec);
  return result;
}

Json::Value encodeDuration(const ros::Duration& value) {
  Json::Value result(Json::objectValue);
  result["secs"] = Json::Int(value.sec);
  result["nsecs"] = Json::Int(value.nsec);
  return result;
}

}  // namespace

JsonCodec::JsonCodec() : JsonCodec(Limits{}) {}

JsonCodec::JsonCodec(Limits limits) : limits_(limits) {
  if (limits_.maximum_depth == 0 || limits_.maximum_array_elements == 0) {
    permanentError("invalid_configuration",
                   "JSON codec limits must be greater than zero");
  }
}

void JsonCodec::decode(const Json::Value& value, Message& message) const {
  decodeValue(value, message, "$", 0);
}

Json::Value JsonCodec::encode(const Message& message) const {
  return encodeValue(message, "$", 0);
}

void JsonCodec::decodeValue(const Json::Value& value, Message& message,
                            const std::string& path, std::size_t depth) const {
  checkDepth(depth, limits_.maximum_depth, path);
  switch (message.type()) {
    case Bool:
      message = booleanValue(value, path);
      return;
    case UInt8:
      message = unsignedInteger<std::uint8_t>(value, path);
      return;
    case UInt16:
      message = unsignedInteger<std::uint16_t>(value, path);
      return;
    case UInt32:
      message = unsignedInteger<std::uint32_t>(value, path);
      return;
    case UInt64:
      message = unsignedInteger<std::uint64_t>(value, path);
      return;
    case Int8:
      message = signedInteger<std::int8_t>(value, path);
      return;
    case Int16:
      message = signedInteger<std::int16_t>(value, path);
      return;
    case Int32:
      message = signedInteger<std::int32_t>(value, path);
      return;
    case Int64:
      message = signedInteger<std::int64_t>(value, path);
      return;
    case Float32: {
      const double parsed = floatingPoint(value, path);
      if (parsed < -std::numeric_limits<float>::max() ||
          parsed > std::numeric_limits<float>::max()) {
        codecError(path, "number is outside the ROS float32 range");
      }
      message = static_cast<float>(parsed);
      return;
    }
    case Float64:
      message = floatingPoint(value, path);
      return;
    case String:
      message = stringValue(value, path);
      return;
    case Time:
      message = timeValue(value, path);
      return;
    case Duration:
      message = durationValue(value, path);
      return;
    case Compound: {
      if (!value.isObject()) {
        codecError(path, "expected an object");
      }
      auto& compound = message.as<CompoundMessage>();
      const auto& keys = compound.keys();
      const auto& values = compound.values();
      rejectUnknownMembers(
          value, std::set<std::string>(keys.begin(), keys.end()), path);
      if (keys.size() != values.size()) {
        codecError(path, "invalid dynamic message field table");
      }
      for (std::size_t index = 0; index < keys.size(); ++index) {
        if (value.isMember(keys[index])) {
          decodeValue(value[keys[index]], *values[index],
                      path + "." + keys[index], depth + 1);
        }
      }
      return;
    }
    case Array:
      break;
    case None:
    default:
      codecError(path, "unsupported ROS message field type");
  }

  const auto& base = message.as<ArrayMessageBase>();
  switch (base.elementType()) {
    case Bool:
      decodePrimitiveArray<bool>(value, message, limits_.maximum_array_elements,
                                 path, booleanValue);
      return;
    case UInt8:
      decodePrimitiveArray<std::uint8_t>(value, message,
                                         limits_.maximum_array_elements, path,
                                         unsignedInteger<std::uint8_t>);
      return;
    case UInt16:
      decodePrimitiveArray<std::uint16_t>(value, message,
                                          limits_.maximum_array_elements, path,
                                          unsignedInteger<std::uint16_t>);
      return;
    case UInt32:
      decodePrimitiveArray<std::uint32_t>(value, message,
                                          limits_.maximum_array_elements, path,
                                          unsignedInteger<std::uint32_t>);
      return;
    case UInt64:
      decodePrimitiveArray<std::uint64_t>(value, message,
                                          limits_.maximum_array_elements, path,
                                          unsignedInteger<std::uint64_t>);
      return;
    case Int8:
      decodePrimitiveArray<std::int8_t>(value, message,
                                        limits_.maximum_array_elements, path,
                                        signedInteger<std::int8_t>);
      return;
    case Int16:
      decodePrimitiveArray<std::int16_t>(value, message,
                                         limits_.maximum_array_elements, path,
                                         signedInteger<std::int16_t>);
      return;
    case Int32:
      decodePrimitiveArray<std::int32_t>(value, message,
                                         limits_.maximum_array_elements, path,
                                         signedInteger<std::int32_t>);
      return;
    case Int64:
      decodePrimitiveArray<std::int64_t>(value, message,
                                         limits_.maximum_array_elements, path,
                                         signedInteger<std::int64_t>);
      return;
    case Float32:
      decodePrimitiveArray<float>(
          value, message, limits_.maximum_array_elements, path,
          [](const Json::Value& item, const std::string& item_path) {
            const double parsed = floatingPoint(item, item_path);
            if (parsed < -std::numeric_limits<float>::max() ||
                parsed > std::numeric_limits<float>::max()) {
              codecError(item_path, "number is outside the ROS float32 range");
            }
            return static_cast<float>(parsed);
          });
      return;
    case Float64:
      decodePrimitiveArray<double>(
          value, message, limits_.maximum_array_elements, path, floatingPoint);
      return;
    case String:
      decodePrimitiveArray<std::string>(
          value, message, limits_.maximum_array_elements, path, stringValue);
      return;
    case Time:
      decodePrimitiveArray<ros::Time>(
          value, message, limits_.maximum_array_elements, path, timeValue);
      return;
    case Duration:
      decodePrimitiveArray<ros::Duration>(
          value, message, limits_.maximum_array_elements, path, durationValue);
      return;
    case Compound: {
      auto& array = message.as<CompoundArrayMessage>();
      validateArrayInput(value, array, limits_.maximum_array_elements, path);
      for (Json::ArrayIndex index = 0; index < value.size(); ++index) {
        Message& child =
            array.isFixedSize() ? array[index] : array.appendEmpty();
        decodeValue(value[index], child,
                    path + "[" + std::to_string(index) + "]", depth + 1);
      }
      return;
    }
    case Array:
    case None:
    default:
      codecError(path, "unsupported ROS array element type");
  }
}

Json::Value JsonCodec::encodeValue(const Message& message,
                                   const std::string& path,
                                   std::size_t depth) const {
  checkDepth(depth, limits_.maximum_depth, path);
  switch (message.type()) {
    case Bool:
      return Json::Value(message.value<bool>());
    case UInt8:
      return Json::Value(Json::UInt(message.value<std::uint8_t>()));
    case UInt16:
      return Json::Value(Json::UInt(message.value<std::uint16_t>()));
    case UInt32:
      return Json::Value(Json::UInt(message.value<std::uint32_t>()));
    case UInt64:
      return Json::Value(std::to_string(message.value<std::uint64_t>()));
    case Int8:
      return Json::Value(Json::Int(message.value<std::int8_t>()));
    case Int16:
      return Json::Value(Json::Int(message.value<std::int16_t>()));
    case Int32:
      return Json::Value(Json::Int(message.value<std::int32_t>()));
    case Int64:
      return Json::Value(std::to_string(message.value<std::int64_t>()));
    case Float32: {
      const float value = message.value<float>();
      if (!std::isfinite(value)) {
        codecError(path, "non-finite float32 cannot be represented as JSON");
      }
      return Json::Value(value);
    }
    case Float64: {
      const double value = message.value<double>();
      if (!std::isfinite(value)) {
        codecError(path, "non-finite float64 cannot be represented as JSON");
      }
      return Json::Value(value);
    }
    case String:
      return Json::Value(message.value<std::string>());
    case Time:
      return encodeTime(message.value<ros::Time>());
    case Duration:
      return encodeDuration(message.value<ros::Duration>());
    case Compound: {
      const auto& compound = message.as<CompoundMessage>();
      const auto& keys = compound.keys();
      const auto& values = compound.values();
      if (keys.size() != values.size()) {
        codecError(path, "invalid dynamic message field table");
      }
      Json::Value result(Json::objectValue);
      for (std::size_t index = 0; index < keys.size(); ++index) {
        result[keys[index]] =
            encodeValue(*values[index], path + "." + keys[index], depth + 1);
      }
      return result;
    }
    case Array:
      break;
    case None:
    default:
      codecError(path, "unsupported ROS message field type");
  }

  const auto& base = message.as<ArrayMessageBase>();
  if (base.length() > limits_.maximum_array_elements) {
    codecError(path, "maximum array element count exceeded");
  }
  switch (base.elementType()) {
    case Bool:
      return encodePrimitiveArray<bool>(
          message, path,
          [](bool value, const std::string&) { return Json::Value(value); });
    case UInt8:
      return encodePrimitiveArray<std::uint8_t>(
          message, path, [](std::uint8_t value, const std::string&) {
            return Json::Value(Json::UInt(value));
          });
    case UInt16:
      return encodePrimitiveArray<std::uint16_t>(
          message, path, [](std::uint16_t value, const std::string&) {
            return Json::Value(Json::UInt(value));
          });
    case UInt32:
      return encodePrimitiveArray<std::uint32_t>(
          message, path, [](std::uint32_t value, const std::string&) {
            return Json::Value(Json::UInt(value));
          });
    case UInt64:
      return encodePrimitiveArray<std::uint64_t>(
          message, path, [](std::uint64_t value, const std::string&) {
            return Json::Value(std::to_string(value));
          });
    case Int8:
      return encodePrimitiveArray<std::int8_t>(
          message, path, [](std::int8_t value, const std::string&) {
            return Json::Value(Json::Int(value));
          });
    case Int16:
      return encodePrimitiveArray<std::int16_t>(
          message, path, [](std::int16_t value, const std::string&) {
            return Json::Value(Json::Int(value));
          });
    case Int32:
      return encodePrimitiveArray<std::int32_t>(
          message, path, [](std::int32_t value, const std::string&) {
            return Json::Value(Json::Int(value));
          });
    case Int64:
      return encodePrimitiveArray<std::int64_t>(
          message, path, [](std::int64_t value, const std::string&) {
            return Json::Value(std::to_string(value));
          });
    case Float32:
      return encodePrimitiveArray<float>(
          message, path, [](float value, const std::string& item_path) {
            if (!std::isfinite(value)) {
              codecError(item_path,
                         "non-finite float32 cannot be represented as JSON");
            }
            return Json::Value(value);
          });
    case Float64:
      return encodePrimitiveArray<double>(
          message, path, [](double value, const std::string& item_path) {
            if (!std::isfinite(value)) {
              codecError(item_path,
                         "non-finite float64 cannot be represented as JSON");
            }
            return Json::Value(value);
          });
    case String:
      return encodePrimitiveArray<std::string>(
          message, path, [](const std::string& value, const std::string&) {
            return Json::Value(value);
          });
    case Time:
      return encodePrimitiveArray<ros::Time>(
          message, path, [](const ros::Time& value, const std::string&) {
            return encodeTime(value);
          });
    case Duration:
      return encodePrimitiveArray<ros::Duration>(
          message, path, [](const ros::Duration& value, const std::string&) {
            return encodeDuration(value);
          });
    case Compound: {
      const auto& array = message.as<CompoundArrayMessage>();
      Json::Value result(Json::arrayValue);
      for (std::size_t index = 0; index < array.length(); ++index) {
        result.append(encodeValue(
            array[index], path + "[" + std::to_string(index) + "]", depth + 1));
      }
      return result;
    }
    case Array:
    case None:
    default:
      codecError(path, "unsupported ROS array element type");
  }
}

}  // namespace xgc_ros1_tools_adapter
