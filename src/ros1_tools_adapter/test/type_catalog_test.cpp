#include <dirent.h>
#include <gtest/gtest.h>
#include <json/json.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "xgc_ros1_tools_adapter/json_codec.hpp"
#include "xgc_ros1_tools_adapter/type_registry.hpp"

namespace xgc_ros1_tools_adapter {
namespace {

struct DirectoryCloser {
  void operator()(DIR* directory) const {
    if (directory != nullptr) {
      closedir(directory);
    }
  }
};

std::vector<std::string> catalogTypes(const std::string& directory,
                                      const std::string& extension) {
  std::vector<std::string> result;
  std::unique_ptr<DIR, DirectoryCloser> entries(opendir(directory.c_str()));
  if (!entries) {
    return result;
  }
  while (const dirent* entry = readdir(entries.get())) {
    const std::string name = entry->d_name;
    if (name.size() > extension.size() &&
        name.compare(name.size() - extension.size(), extension.size(),
                     extension) == 0) {
      result.push_back(name.substr(0, name.size() - extension.size()));
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

std::vector<std::uint8_t> serializeRaw(const ros_babel_fish::Message& message) {
  std::vector<std::uint8_t> result(message._sizeInBytes());
  if (!result.empty()) {
    EXPECT_EQ(result.size(), message.writeToStream(result.data()));
  }
  return result;
}

TEST(StandardCatalog, InstantiatesAndSerializesEveryInstalledMessage) {
  const std::string package_path = ros::package::getPath("std_msgs");
  ASSERT_FALSE(package_path.empty());
  const auto messages = catalogTypes(package_path + "/msg", ".msg");
  ASSERT_GE(messages.size(), 20U);

  TypeRegistry types;
  JsonCodec codec;
  for (const auto& name : messages) {
    SCOPED_TRACE(name);
    const std::string type = "std_msgs/" + name;
    auto original = types.createMessage(type);
    const Json::Value canonical = codec.encode(*original);
    auto decoded = types.createMessage(type);
    codec.decode(canonical, *decoded);
    const auto original_wire = types.serialize(original);
    const auto decoded_wire = types.serialize(decoded);
    ASSERT_TRUE(original_wire);
    ASSERT_TRUE(decoded_wire);
    EXPECT_EQ(type, original_wire->dataType());
    EXPECT_EQ(original_wire->size(), decoded_wire->size());
    ASSERT_EQ(original_wire->size(), decoded_wire->size());
    EXPECT_TRUE(std::equal(original_wire->buffer(),
                           original_wire->buffer() + original_wire->size(),
                           decoded_wire->buffer()));
  }
}

TEST(StandardCatalog, InstantiatesAndSerializesEveryInstalledServiceSide) {
  const std::string package_path = ros::package::getPath("std_srvs");
  ASSERT_FALSE(package_path.empty());
  const auto services = catalogTypes(package_path + "/srv", ".srv");
  ASSERT_GE(services.size(), 3U);

  TypeRegistry types;
  JsonCodec codec;
  for (const auto& name : services) {
    SCOPED_TRACE(name);
    const std::string type = "std_srvs/" + name;
    const auto description = types.resolveService(type);
    ASSERT_TRUE(description);
    ASSERT_FALSE(description->md5.empty());

    auto request = types.createServiceRequest(description);
    const Json::Value request_json = codec.encode(*request);
    auto decoded_request = types.createServiceRequest(description);
    codec.decode(request_json, *decoded_request);
    EXPECT_EQ(serializeRaw(*request), serializeRaw(*decoded_request));

    auto response = types.createServiceResponse(description);
    const Json::Value response_json = codec.encode(*response);
    auto decoded_response = types.createServiceResponse(description);
    codec.decode(response_json, *decoded_response);
    EXPECT_EQ(serializeRaw(*response), serializeRaw(*decoded_response));
  }
}

}  // namespace
}  // namespace xgc_ros1_tools_adapter

int main(int argc, char** argv) {
  ros::init(
      argc, argv, "xgc_ros1_tools_adapter_unit_tests",
      ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
