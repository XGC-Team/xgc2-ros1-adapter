#include "xgc_ros1_tools_adapter/json_codec.hpp"

#include <gtest/gtest.h>
#include <json/json.h>

#include <cstdint>
#include <limits>
#include <string>

#include "xgc_ros1_tools_adapter/error.hpp"
#include "xgc_ros1_tools_adapter/type_registry.hpp"

namespace xgc_ros1_tools_adapter {
namespace {

TEST(JsonCodec, RoundTripsEveryPrimitiveScalarType) {
  TypeRegistry types;
  JsonCodec codec;
  const std::pair<const char*, Json::Value> cases[] = {
      {"std_msgs/Bool", Json::Value(true)},
      {"std_msgs/UInt8", Json::Value(Json::UInt(255))},
      {"std_msgs/UInt16", Json::Value(Json::UInt(65535))},
      {"std_msgs/UInt32",
       Json::Value(Json::UInt64(std::numeric_limits<std::uint32_t>::max()))},
      {"std_msgs/UInt64", Json::Value("18446744073709551615")},
      {"std_msgs/Int8", Json::Value(Json::Int(-128))},
      {"std_msgs/Int16", Json::Value(Json::Int(-32768))},
      {"std_msgs/Int32",
       Json::Value(Json::Int64(std::numeric_limits<std::int32_t>::min()))},
      {"std_msgs/Int64", Json::Value("-9223372036854775808")},
      {"std_msgs/Float32", Json::Value(1.25)},
      {"std_msgs/Float64", Json::Value(-2.5)},
      {"std_msgs/String", Json::Value("XGC2 机器人")},
  };

  for (const auto& item : cases) {
    SCOPED_TRACE(item.first);
    auto message = types.createMessage(item.first);
    Json::Value input(Json::objectValue);
    input["data"] = item.second;
    codec.decode(input, *message);
    const Json::Value output = codec.encode(*message);
    EXPECT_EQ(item.second, output["data"]);
    EXPECT_GT(types.serialize(message)->size(), 0U);
  }
}

TEST(JsonCodec, HandlesNestedTimeAndFixedArrays) {
  TypeRegistry types;
  JsonCodec codec;

  auto header = types.createMessage("std_msgs/Header");
  Json::Value header_json(Json::objectValue);
  header_json["seq"] = Json::UInt(42);
  header_json["stamp"]["secs"] = Json::UInt(123);
  header_json["stamp"]["nsecs"] = Json::UInt(456);
  header_json["frame_id"] = "map";
  codec.decode(header_json, *header);
  EXPECT_EQ(header_json, codec.encode(*header));

  auto pose = types.createMessage("geometry_msgs/PoseWithCovariance");
  Json::Value pose_json = codec.encode(*pose);
  ASSERT_EQ(36U, pose_json["covariance"].size());
  pose_json["pose"]["position"]["x"] = 1.25;
  pose_json["pose"]["orientation"]["w"] = 1.0;
  pose_json["covariance"][0] = 0.1;
  codec.decode(pose_json, *pose);
  const Json::Value output = codec.encode(*pose);
  EXPECT_DOUBLE_EQ(1.25, output["pose"]["position"]["x"].asDouble());
  EXPECT_DOUBLE_EQ(1.0, output["pose"]["orientation"]["w"].asDouble());
  EXPECT_DOUBLE_EQ(0.1, output["covariance"][0].asDouble());
}

TEST(JsonCodec, HandlesDynamicPrimitiveAndCompoundArrays) {
  TypeRegistry types;
  JsonCodec codec;

  auto integers = types.createMessage("std_msgs/UInt64MultiArray");
  Json::Value integer_json = codec.encode(*integers);
  integer_json["data"].append("18446744073709551615");
  integer_json["data"].append("9007199254740993");
  codec.decode(integer_json, *integers);
  EXPECT_EQ(integer_json, codec.encode(*integers));

  auto poses = types.createMessage("geometry_msgs/PoseArray");
  Json::Value poses_json = codec.encode(*poses);
  Json::Value pose(Json::objectValue);
  pose["position"]["x"] = 1.0;
  pose["position"]["y"] = 2.0;
  pose["position"]["z"] = 3.0;
  pose["orientation"]["x"] = 0.0;
  pose["orientation"]["y"] = 0.0;
  pose["orientation"]["z"] = 0.0;
  pose["orientation"]["w"] = 1.0;
  poses_json["poses"].append(pose);
  codec.decode(poses_json, *poses);
  EXPECT_EQ(poses_json, codec.encode(*poses));
}

TEST(JsonCodec, RejectsUnknownFieldsOverflowAndWrongFixedLength) {
  TypeRegistry types;
  JsonCodec codec;

  auto text = types.createMessage("std_msgs/String");
  Json::Value unknown(Json::objectValue);
  unknown["data"] = "ok";
  unknown["noise"] = true;
  EXPECT_THROW(codec.decode(unknown, *text), Ros1ToolsError);

  auto value = types.createMessage("std_msgs/UInt8");
  Json::Value overflow(Json::objectValue);
  overflow["data"] = Json::UInt(256);
  EXPECT_THROW(codec.decode(overflow, *value), Ros1ToolsError);

  auto pose = types.createMessage("geometry_msgs/PoseWithCovariance");
  Json::Value pose_json = codec.encode(*pose);
  pose_json["covariance"].resize(35);
  EXPECT_THROW(codec.decode(pose_json, *pose), Ros1ToolsError);
}

TEST(JsonCodec, RepresentsInt64ExactlyAsDecimalStrings) {
  TypeRegistry types;
  JsonCodec codec;
  auto unsigned_value = types.createMessage("std_msgs/UInt64");
  Json::Value input(Json::objectValue);
  input["data"] = "9007199254740993";
  codec.decode(input, *unsigned_value);
  EXPECT_TRUE(codec.encode(*unsigned_value)["data"].isString());
  EXPECT_EQ("9007199254740993",
            codec.encode(*unsigned_value)["data"].asString());
}

TEST(JsonCodec, RejectsNonCanonicalIntegerStrings) {
  TypeRegistry types;
  JsonCodec codec;
  const char* invalid_values[] = {"+1", "01", "-0", " 1", "1 ", "1.0"};
  for (const char* invalid : invalid_values) {
    SCOPED_TRACE(invalid);
    auto value = types.createMessage("std_msgs/Int64");
    Json::Value input(Json::objectValue);
    input["data"] = invalid;
    EXPECT_THROW(codec.decode(input, *value), Ros1ToolsError);
  }
}

}  // namespace
}  // namespace xgc_ros1_tools_adapter
