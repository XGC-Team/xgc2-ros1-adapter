#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "xgc/semantic/aerial/v1/control.pb.h"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"
#include "xgc2_ros1_robot_adapter/runtime_support.hpp"

namespace xgc2_ros1_robot_adapter {
namespace {

constexpr const char *kProfileDigest =
    "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
constexpr const char *kSpecDigest =
    "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
constexpr const char *kAssetDigest =
    "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
constexpr std::uint32_t kRobotAdapterSpecMessageId = 4001u;
constexpr std::uint64_t kRobotAdapterSpecFingerprint =
    1932893837531035663ULL;

MessageSchema robotConfigSchema() {
  MessageSchema schema;
  schema.message_id = kRobotAdapterSpecMessageId;
  schema.type_name =
      xgc::robot::v1::RobotAdapterSpec::descriptor()->full_name();
  schema.version = kRobotAdapterSpecSchemaVersion;
  schema.fingerprint = kRobotAdapterSpecFingerprint;
  return schema;
}

xgc::adapter::v1::AdapterInstanceSpec makeValidInstanceSpec() {
  xgc::robot::v1::RobotAdapterSpec robot_spec;
  robot_spec.set_asset_digest(kAssetDigest);
  auto *robot = robot_spec.add_robots();
  robot->set_robot_id("px4-01");
  robot->set_profile_id("px4.multirotor.ros1.v5");
  robot->set_profile_digest(kProfileDigest);
  (*robot->mutable_parameters())["namespace"] = "/uav1";
  (*robot->mutable_parameters())["mocap_rigid_body"] = "px4_01";
  auto *pose = robot->add_channels();
  pose->set_channel_id("state.pose");
  pose->set_enabled(true);
  auto *arm = robot->add_channels();
  arm->set_channel_id("operation.arm");
  arm->set_enabled(false);

  std::string encoded;
  EXPECT_TRUE(robot_spec.SerializeToString(&encoded));

  xgc::adapter::v1::AdapterInstanceSpec instance;
  instance.set_instance_id("px4-run-123");
  instance.set_process_generation(7u);
  instance.set_revision(11u);
  instance.set_spec_digest(kSpecDigest);
  instance.mutable_scope()->set_kind("robot-group");
  instance.mutable_scope()->set_key(kSpecDigest);
  (*instance.mutable_scope()->mutable_attributes())["target-id"] = "local";
  (*instance.mutable_scope()->mutable_attributes())["run-id"] = "run-123";
  (*instance.mutable_scope()->mutable_attributes())["provider"] =
      "xgc2-px4-multirotor-ros1-adapter";
  instance.mutable_configuration()->mutable_schema()->set_type_name(
      xgc::robot::v1::RobotAdapterSpec::descriptor()->full_name());
  instance.mutable_configuration()->mutable_schema()->set_message_id(
      kRobotAdapterSpecMessageId);
  instance.mutable_configuration()->mutable_schema()->set_schema_version(
      kRobotAdapterSpecSchemaVersion);
  instance.mutable_configuration()->mutable_schema()->set_schema_fingerprint(
      kRobotAdapterSpecFingerprint);
  instance.mutable_configuration()->set_encoding(
      xgc::v1::PAYLOAD_ENCODING_PROTOBUF);
  instance.mutable_configuration()->set_value(encoded);
  return instance;
}

xgc::robot::v1::RobotAdapterSpec robotSpecFrom(
    const xgc::adapter::v1::AdapterInstanceSpec &instance) {
  xgc::robot::v1::RobotAdapterSpec robot_spec;
  EXPECT_TRUE(robot_spec.ParseFromString(instance.configuration().value()));
  return robot_spec;
}

bool replaceRobotSpec(
    const xgc::robot::v1::RobotAdapterSpec &robot_spec,
    xgc::adapter::v1::AdapterInstanceSpec *instance) {
  std::string encoded;
  if (instance == nullptr || !robot_spec.SerializeToString(&encoded))
    return false;
  instance->mutable_configuration()->set_value(std::move(encoded));
  return true;
}

std::string decodeFailure(
    const xgc::adapter::v1::AdapterInstanceSpec &instance) {
  RobotAdapterConfig decoded;
  std::string error;
  EXPECT_FALSE(DecodeRobotAdapterConfig(instance, robotConfigSchema(), &decoded,
                                        &error));
  return error;
}

MessageSchema modeSchema() {
  MessageSchema schema;
  schema.message_id = 3202u;
  schema.type_name =
      xgc::semantic::aerial::v1::ModeRequest::descriptor()->full_name();
  schema.version = 1u;
  schema.fingerprint = 5742784340350308261ULL;
  return schema;
}

RobotMessageContext modeContext() {
  RobotMessageContext context;
  context.robot_id = "px4-01";
  context.channel_id = "operation.mode";
  context.sequence = 42u;
  context.has_source_time = true;
  context.source_time_nanos = 123456789;
  context.source_clock_domain = xgc::v1::CLOCK_DOMAIN_SIMULATION;
  context.observed_unix_nanos = 987654321;
  return context;
}

TEST(RobotAdapterConfigDecoder, ProjectsTypedSpecAndFence) {
  const auto instance = makeValidInstanceSpec();
  RobotAdapterConfig decoded;
  std::string error;
  ASSERT_TRUE(DecodeRobotAdapterConfig(instance, robotConfigSchema(), &decoded,
                                       &error))
      << error;

  EXPECT_EQ("px4-run-123", decoded.fence.instance_id);
  EXPECT_EQ(7u, decoded.fence.process_generation);
  EXPECT_EQ(11u, decoded.fence.revision);
  EXPECT_EQ(kSpecDigest, decoded.fence.spec_digest);
  EXPECT_EQ("robot-group", decoded.scope_kind);
  EXPECT_EQ(kSpecDigest, decoded.scope_key);
  EXPECT_EQ("local", decoded.scope_attributes.at("target-id"));
  EXPECT_EQ("run-123", decoded.scope_attributes.at("run-id"));
  EXPECT_EQ(kAssetDigest, decoded.asset_digest);
  ASSERT_EQ(1u, decoded.robots.size());
  EXPECT_EQ("px4-01", decoded.robots[0].robot_id);
  EXPECT_EQ("/uav1", decoded.robots[0].parameters.at("namespace"));
  ASSERT_EQ(2u, decoded.robots[0].channels.size());
  EXPECT_TRUE(decoded.robots[0].channels[0].enabled);
  EXPECT_FALSE(decoded.robots[0].channels[1].enabled);

  InstanceSpecFence same = decoded.fence;
  EXPECT_TRUE(decoded.fence.matches(same));
  ++same.revision;
  EXPECT_FALSE(decoded.fence.matches(same));
}

TEST(RobotAdapterConfigDecoder, RejectsWrongSchemaEncodingAndMalformedPayload) {
  RobotAdapterConfig decoded;
  std::string error;

  auto instance = makeValidInstanceSpec();
  instance.mutable_configuration()->mutable_schema()->set_type_name(
      "xgc.robot.v1.OtherSpec");
  EXPECT_FALSE(DecodeRobotAdapterConfig(instance, robotConfigSchema(), &decoded,
                                        &error));
  EXPECT_NE(std::string::npos, error.find("registry entry"));

  instance = makeValidInstanceSpec();
  instance.mutable_configuration()->mutable_schema()->set_message_id(4002u);
  EXPECT_FALSE(DecodeRobotAdapterConfig(instance, robotConfigSchema(), &decoded,
                                        &error));
  EXPECT_NE(std::string::npos, error.find("registry entry"));

  instance = makeValidInstanceSpec();
  instance.mutable_configuration()->mutable_schema()->set_schema_fingerprint(
      kRobotAdapterSpecFingerprint + 1u);
  EXPECT_FALSE(DecodeRobotAdapterConfig(instance, robotConfigSchema(), &decoded,
                                        &error));
  EXPECT_NE(std::string::npos, error.find("registry entry"));

  instance = makeValidInstanceSpec();
  instance.mutable_configuration()->set_encoding(xgc::v1::PAYLOAD_ENCODING_JSON);
  EXPECT_FALSE(DecodeRobotAdapterConfig(instance, robotConfigSchema(), &decoded,
                                        &error));
  EXPECT_NE(std::string::npos, error.find("protobuf encoding"));

  instance = makeValidInstanceSpec();
  instance.mutable_configuration()->set_value("\xff\xff");
  EXPECT_FALSE(DecodeRobotAdapterConfig(instance, robotConfigSchema(), &decoded,
                                        &error));
  EXPECT_NE(std::string::npos, error.find("malformed"));
}

TEST(RobotAdapterConfigDecoder, RejectsDuplicateRobotsAndChannels) {
  RobotAdapterConfig decoded;
  std::string error;

  auto instance = makeValidInstanceSpec();
  xgc::robot::v1::RobotAdapterSpec robot_spec;
  ASSERT_TRUE(robot_spec.ParseFromString(instance.configuration().value()));
  *robot_spec.add_robots() = robot_spec.robots(0);
  ASSERT_TRUE(robot_spec.SerializeToString(
      instance.mutable_configuration()->mutable_value()));
  EXPECT_FALSE(DecodeRobotAdapterConfig(instance, robotConfigSchema(), &decoded,
                                        &error));
  EXPECT_NE(std::string::npos, error.find("repeats robot_id"));

  instance = makeValidInstanceSpec();
  ASSERT_TRUE(robot_spec.ParseFromString(instance.configuration().value()));
  *robot_spec.mutable_robots(0)->add_channels() =
      robot_spec.robots(0).channels(0);
  ASSERT_TRUE(robot_spec.SerializeToString(
      instance.mutable_configuration()->mutable_value()));
  EXPECT_FALSE(DecodeRobotAdapterConfig(instance, robotConfigSchema(), &decoded,
                                        &error));
  EXPECT_NE(std::string::npos, error.find("repeats channel"));
}

TEST(RobotAdapterConfigDecoder, EnforcesCanonicalBoundedProfileIds) {
  auto instance = makeValidInstanceSpec();
  auto robot_spec = robotSpecFrom(instance);
  robot_spec.mutable_robots(0)->set_profile_id("1robot.ros1.v4");
  ASSERT_TRUE(replaceRobotSpec(robot_spec, &instance));
  EXPECT_NE(std::string::npos,
            decodeFailure(instance).find("invalid profile_id"));

  instance = makeValidInstanceSpec();
  robot_spec = robotSpecFrom(instance);
  robot_spec.mutable_robots(0)->set_profile_id("Robot.ros1.v4");
  ASSERT_TRUE(replaceRobotSpec(robot_spec, &instance));
  EXPECT_NE(std::string::npos,
            decodeFailure(instance).find("invalid profile_id"));

  instance = makeValidInstanceSpec();
  robot_spec = robotSpecFrom(instance);
  robot_spec.mutable_robots(0)->set_profile_id(std::string(126u, 'a') +
                                               ".v4");
  ASSERT_TRUE(replaceRobotSpec(robot_spec, &instance));
  EXPECT_NE(std::string::npos,
            decodeFailure(instance).find("invalid profile_id"));

  instance = makeValidInstanceSpec();
  robot_spec = robotSpecFrom(instance);
  robot_spec.mutable_robots(0)->set_profile_id(std::string(125u, 'a') +
                                               ".v4");
  ASSERT_TRUE(replaceRobotSpec(robot_spec, &instance));
  RobotAdapterConfig decoded;
  std::string error;
  EXPECT_TRUE(DecodeRobotAdapterConfig(instance, robotConfigSchema(), &decoded,
                                       &error))
      << error;
}

TEST(RobotAdapterConfigDecoder, EnforcesRobotParameterMapBounds) {
  auto instance = makeValidInstanceSpec();
  auto robot_spec = robotSpecFrom(instance);
  auto *parameters = robot_spec.mutable_robots(0)->mutable_parameters();
  for (std::size_t index = 0u; index < 62u; ++index)
    (*parameters)["optional_" + std::to_string(index)] = "";
  ASSERT_EQ(64u, parameters->size());
  ASSERT_TRUE(replaceRobotSpec(robot_spec, &instance));
  RobotAdapterConfig decoded;
  std::string error;
  ASSERT_TRUE(DecodeRobotAdapterConfig(instance, robotConfigSchema(), &decoded,
                                       &error))
      << error;
  EXPECT_TRUE(decoded.robots[0].parameters.at("optional_0").empty());

  (*parameters)["overflow"] = "value";
  ASSERT_TRUE(replaceRobotSpec(robot_spec, &instance));
  EXPECT_NE(std::string::npos,
            decodeFailure(instance).find("exceeds 64 parameter entries"));

  instance = makeValidInstanceSpec();
  robot_spec = robotSpecFrom(instance);
  (*robot_spec.mutable_robots(0)->mutable_parameters())[std::string(64u, 'a')] =
      "value";
  ASSERT_TRUE(replaceRobotSpec(robot_spec, &instance));
  ASSERT_TRUE(DecodeRobotAdapterConfig(instance, robotConfigSchema(), &decoded,
                                       &error))
      << error;

  (*robot_spec.mutable_robots(0)->mutable_parameters())[std::string(65u, 'a')] =
      "value";
  ASSERT_TRUE(replaceRobotSpec(robot_spec, &instance));
  EXPECT_NE(std::string::npos,
            decodeFailure(instance).find("non-canonical parameter name"));

  instance = makeValidInstanceSpec();
  robot_spec = robotSpecFrom(instance);
  (*robot_spec.mutable_robots(0)->mutable_parameters())["Not-Canonical"] =
      "value";
  ASSERT_TRUE(replaceRobotSpec(robot_spec, &instance));
  EXPECT_NE(std::string::npos,
            decodeFailure(instance).find("non-canonical parameter name"));

  std::string utf8_value;
  utf8_value.reserve(4097u);
  for (std::size_t index = 0u; index < 2048u; ++index)
    utf8_value.append("\xc3\xa9", 2u);
  instance = makeValidInstanceSpec();
  robot_spec = robotSpecFrom(instance);
  (*robot_spec.mutable_robots(0)->mutable_parameters())["namespace"] =
      utf8_value;
  ASSERT_TRUE(replaceRobotSpec(robot_spec, &instance));
  ASSERT_TRUE(DecodeRobotAdapterConfig(instance, robotConfigSchema(), &decoded,
                                       &error))
      << error;

  (*robot_spec.mutable_robots(0)->mutable_parameters())["namespace"] += "x";
  ASSERT_TRUE(replaceRobotSpec(robot_spec, &instance));
  EXPECT_NE(std::string::npos,
            decodeFailure(instance).find("exceeds 4096 UTF-8 bytes"));
}

TEST(RobotAdapterConfigDecoder, FailureDoesNotReplacePreviousConfig) {
  RobotAdapterConfig decoded;
  decoded.asset_digest = "preserve-me";
  auto invalid = makeValidInstanceSpec();
  invalid.set_revision(0u);
  std::string error;

  EXPECT_FALSE(DecodeRobotAdapterConfig(invalid, robotConfigSchema(), &decoded,
                                        &error));
  EXPECT_EQ("preserve-me", decoded.asset_digest);
}

TEST(RobotMessageBuilder, BuildsSchemaAwareRoutedProtobufMessage) {
  xgc::semantic::aerial::v1::ModeRequest semantic;
  semantic.set_mode("OFFBOARD");
  xgc::robot::v1::RobotMessage message;
  std::string error;
  ASSERT_TRUE(BuildRobotMessage(modeContext(), modeSchema(), semantic, &message,
                                &error))
      << error;

  EXPECT_EQ("px4-01", message.robot_id());
  EXPECT_EQ("operation.mode", message.channel_id());
  EXPECT_EQ(42u, message.message().sequence());
  EXPECT_EQ(123456789, message.message().source_time().nanoseconds());
  EXPECT_EQ(xgc::v1::CLOCK_DOMAIN_SIMULATION,
            message.message().source_time().clock_domain());
  EXPECT_EQ(987654321, message.message().observed_unix_nanos());
  EXPECT_EQ(3202u, message.message().payload().schema().message_id());
  EXPECT_EQ(modeSchema().type_name,
            message.message().payload().schema().type_name());
  EXPECT_EQ(xgc::v1::PAYLOAD_ENCODING_PROTOBUF,
            message.message().payload().encoding());

  xgc::semantic::aerial::v1::ModeRequest decoded;
  ASSERT_TRUE(decoded.ParseFromString(message.message().payload().value()));
  EXPECT_EQ("OFFBOARD", decoded.mode());
}

TEST(RobotMessageBuilder, SerializesOneOpaqueStreamItem) {
  xgc::semantic::aerial::v1::ModeRequest semantic;
  semantic.set_mode("POSCTL");
  std::string item;
  std::string error;
  ASSERT_TRUE(SerializeRobotMessageItem(modeContext(), modeSchema(), semantic,
                                        &item, &error))
      << error;

  xgc::robot::v1::RobotMessage decoded;
  ASSERT_TRUE(decoded.ParseFromString(item));
  EXPECT_EQ("px4-01", decoded.robot_id());
  EXPECT_EQ("operation.mode", decoded.channel_id());
}

TEST(RobotMessageBuilder, RejectsIncompleteOrMismatchedMetadata) {
  xgc::semantic::aerial::v1::ModeRequest semantic;
  RobotMessageContext context = modeContext();
  MessageSchema schema = modeSchema();
  xgc::robot::v1::RobotMessage output;
  std::string error;

  schema.fingerprint = 0u;
  EXPECT_FALSE(BuildRobotMessage(context, schema, semantic, &output, &error));
  EXPECT_NE(std::string::npos, error.find("metadata is incomplete"));

  schema = modeSchema();
  schema.type_name =
      xgc::semantic::aerial::v1::ArmRequest::descriptor()->full_name();
  EXPECT_FALSE(BuildRobotMessage(context, schema, semantic, &output, &error));
  EXPECT_NE(std::string::npos, error.find("type does not match"));

  schema = modeSchema();
  context.sequence = 0u;
  EXPECT_FALSE(BuildRobotMessage(context, schema, semantic, &output, &error));
  EXPECT_NE(std::string::npos, error.find("sequence"));
}

TEST(RobotMessageBuilder, EnforcesRosClockDomainWhenSourceTimeExists) {
  xgc::semantic::aerial::v1::ModeRequest semantic;
  RobotMessageContext context = modeContext();
  context.source_clock_domain = xgc::v1::CLOCK_DOMAIN_UNIX;
  xgc::robot::v1::RobotMessage output;
  std::string error;

  EXPECT_FALSE(
      BuildRobotMessage(context, modeSchema(), semantic, &output, &error));
  EXPECT_NE(std::string::npos, error.find("native or simulation"));

  context.has_source_time = false;
  ASSERT_TRUE(BuildRobotMessage(context, modeSchema(), semantic, &output,
                                &error))
      << error;
  EXPECT_FALSE(output.message().has_source_time());
}

TEST(RuntimeSupport, SuccessfulOperationCarriesRegisteredEmptyPayload) {
  constexpr std::uint64_t kEmptyFingerprint = 11009224659857530918ULL;
  const auto result = EmptyOperationSuccess(1u, kEmptyFingerprint);

  EXPECT_EQ(xgc::adapter::v1::OPERATION_PHASE_SUCCEEDED, result.phase);
  ASSERT_TRUE(result.has_output);
  EXPECT_EQ(1u, result.output.schema().message_id());
  EXPECT_EQ("xgc.v1.Empty", result.output.schema().type_name());
  EXPECT_EQ(1u, result.output.schema().schema_version());
  EXPECT_EQ(kEmptyFingerprint,
            result.output.schema().schema_fingerprint());
  EXPECT_EQ(xgc::v1::PAYLOAD_ENCODING_PROTOBUF, result.output.encoding());
  xgc::v1::Empty empty;
  EXPECT_TRUE(empty.ParseFromString(result.output.value()));
}

} // namespace
} // namespace xgc2_ros1_robot_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
