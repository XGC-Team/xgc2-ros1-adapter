#pragma once

#include <json/json.h>
#include <ros_babel_fish/message.h>

#include <cstddef>
#include <string>

namespace xgc_ros1_tools_adapter {

class JsonCodec {
 public:
  struct Limits {
    std::size_t maximum_depth{64};
    std::size_t maximum_array_elements{1000000};
  };

  JsonCodec();
  explicit JsonCodec(Limits limits);

  void decode(const Json::Value& value, ros_babel_fish::Message& message) const;
  Json::Value encode(const ros_babel_fish::Message& message) const;

 private:
  void decodeValue(const Json::Value& value, ros_babel_fish::Message& message,
                   const std::string& path, std::size_t depth) const;
  Json::Value encodeValue(const ros_babel_fish::Message& message,
                          const std::string& path, std::size_t depth) const;

  Limits limits_;
};

}  // namespace xgc_ros1_tools_adapter
