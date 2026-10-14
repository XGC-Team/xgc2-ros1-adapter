#include <gtest/gtest.h>

#include <string>
#include <stdexcept>
#include <utility>
#include <limits>

#include "xgc/semantic/aerial/v1/control.pb.h"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"
#include "xgc2_ros1_robot_adapter/localization_projection.hpp"
#include "xgc2_ros1_robot_adapter/ground_health.hpp"
#include "xgc2_ros1_robot_adapter/native_command.hpp"

namespace xgc2_ros1_robot_adapter {
namespace {

TEST(LocalizationProjection, AppliesOnlyXYZAndPreservesTheSourceFact) {
  xgc2_ros1_robot_adapter::LocalizationProjectionConfig config;
  config.source_root = "/vrpn_client_node_physical";
  config.offset_x = 1.0;
  config.offset_y = -2.0;
  config.offset_z = 0.25;
  geometry_msgs::PoseStamped source;
  source.header.frame_id = "world";
  source.header.stamp = ros::Time(7, 8);
  source.pose.position.x = 3.0;
  source.pose.position.y = 4.0;
  source.pose.position.z = 5.0;
  source.pose.orientation.x = 0.1;
  source.pose.orientation.w = 0.9;
  geometry_msgs::PoseStamped projected;
  ASSERT_TRUE(xgc2_ros1_robot_adapter::projectLocalizationPose(source, config,
                                                               &projected));
  EXPECT_DOUBLE_EQ(projected.pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(projected.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(projected.pose.position.z, 5.25);
  EXPECT_EQ(projected.header, source.header);
  EXPECT_EQ(projected.pose.orientation, source.pose.orientation);
}

TEST(LocalizationProjection, MapsRawVrpnAccelerationWithoutChangingFields) {
  geometry_msgs::TwistStamped source;
  source.header.seq = 42;
  source.header.frame_id = "world";
  source.header.stamp = ros::Time(7, 8);
  source.twist.linear.x = 1.25;
  source.twist.linear.y = -2.5;
  source.twist.linear.z = 3.75;
  source.twist.angular.x = -4.25;
  source.twist.angular.y = 5.5;
  source.twist.angular.z = -6.75;

  const auto projected = projectVrpnAcceleration(source);
  EXPECT_EQ(source.header, projected.header);
  EXPECT_EQ(source.twist.linear, projected.accel.linear);
  EXPECT_EQ(source.twist.angular, projected.accel.angular);
  EXPECT_TRUE(validLocalizationAcceleration(projected));
}

TEST(LocalizationProjection, RawVrpnAccelerationKeepsTheExistingFiniteCheck) {
  for (std::size_t component = 0; component < 6u; ++component) {
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()}) {
      geometry_msgs::TwistStamped source;
      double *fields[] = {&source.twist.linear.x, &source.twist.linear.y,
                          &source.twist.linear.z, &source.twist.angular.x,
                          &source.twist.angular.y, &source.twist.angular.z};
      *fields[component] = invalid;
      EXPECT_FALSE(validLocalizationAcceleration(projectVrpnAcceleration(source)))
          << "component " << component;
    }
  }
}

TEST(LocalizationProjection, ParsesStrictRunScopedParameters) {
  std::map<std::string, std::string> parameters{
      {"mocap_source_root", "/vrpn_client_node_simulation"},
      {"localization_offset_x", "0"},
      {"localization_offset_y", "-1.25"},
      {"localization_offset_z", "2.5"},
  };
  xgc2_ros1_robot_adapter::LocalizationProjectionConfig config;
  std::string error;
  ASSERT_TRUE(xgc2_ros1_robot_adapter::parseLocalizationProjectionConfig(
      parameters, &config, &error))
      << error;
  EXPECT_EQ(config.source_root, "/vrpn_client_node_simulation");
  EXPECT_DOUBLE_EQ(config.offset_y, -1.25);
  parameters["mocap_source_root"] = "vrpn_client_node";
  EXPECT_FALSE(xgc2_ros1_robot_adapter::parseLocalizationProjectionConfig(
      parameters, &config, &error));
  parameters["mocap_source_root"] = "/vrpn_client_node";
  parameters["localization_offset_x"] = "nan";
  EXPECT_FALSE(xgc2_ros1_robot_adapter::parseLocalizationProjectionConfig(
      parameters, &config, &error));
}

TEST(LocalizationProjection, VisionCadenceUsesActualCallbackEmits) {
  xgc2_ros1_robot_adapter::VisionPublishCadence cadence(30.0, 5u);
  int emits = 0;
  for (int index = 0; index < 120; ++index) {
    if (cadence.take(static_cast<double>(index) / 120.0))
      ++emits;
  }
  EXPECT_EQ(emits, 30);
  EXPECT_FALSE(cadence.take(119.5 / 120.0));
}

TEST(LocalizationProjection, VisionCadenceSurvivesLongRunsAndClockRewind) {
  VisionPublishCadence cadence(30.0, 5u);
  for (int frame = 0; frame < 2400; ++frame) cadence.take(frame / 120.0);
  EXPECT_FALSE(cadence.take(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_TRUE(cadence.take(0.0));
  EXPECT_FALSE(cadence.take(0.005));
  EXPECT_TRUE(cadence.take(0.035));
}

TEST(PositioningLiveness, RepeatedFramesAgeOutAndClockRewindStartsANewWindow) {
  PositioningHealthWindow health({3u, 0.1, 0.1});
  for (int frame = 0; frame < 64; ++frame)
    health.recordPose(1.0 + frame * .001, 1.0, 2.0, 3.0);
  EXPECT_EQ(PositioningHealthState::kFrozen, health.evaluate(1.063).state);
  // One-axis movement proves freshness until the last different old frame
  // leaves the original three-frame comparison window.
  for (int frame = 0; frame < 3; ++frame) {
    health.recordPose(1.064 + frame * .001, 1.0, 3.0, 3.0);
    EXPECT_EQ(PositioningHealthState::kActive, health.evaluate(1.064 + frame * .001).state);
  }
  health.recordPose(1.067, 1.0, 3.0, 3.0);
  EXPECT_EQ(PositioningHealthState::kFrozen, health.evaluate(1.067).state);
  EXPECT_EQ(3u, health.evaluate(1.067).sample_count);
  health.recordPose(.01, 5.0, 6.0, 7.0);
  EXPECT_EQ(1u, health.evaluate(.01).sample_count);
  EXPECT_EQ(PositioningHealthState::kFrozen, health.evaluate(.01).state);
  EXPECT_DOUBLE_EQ(0.0, health.evaluate(.01).comparison_metric_m);
  EXPECT_EQ(PositioningHealthState::kTimedOut, health.evaluate(.111).state);
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

TEST(RobotMessageEncoder, EncodesSchemaAwareRoutedProtobufMessage) {
  xgc::semantic::aerial::v1::ModeRequest semantic;
  semantic.set_mode("OFFBOARD");
  xgc::robot::v1::RobotMessage message;
  std::string error;
  ASSERT_TRUE(EncodeRobotMessage(modeContext(), modeSchema(), semantic, &message,
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

TEST(RobotServer, SuccessfulOperationCarriesRegisteredEmptyPayload) {
  struct Channel { unsigned output_message_id = 1; } channel;
  struct Metadata { const char *type_name = "xgc.v1.Empty"; unsigned version = 1; std::uint64_t fingerprint = 11009224659857530918ULL; } metadata;
  xgc::adapter::v1::OperationRequest request; request.mutable_context()->set_work_id("native-result");
  const auto result = CommandSuccess(request, channel, metadata);
  EXPECT_EQ(xgc::adapter::v1::OPERATION_PHASE_SUCCEEDED, result.phase());
  EXPECT_EQ("native-result", result.work_id());
  ASSERT_TRUE(result.has_output());
  EXPECT_EQ(1u, result.output().schema().message_id());
  EXPECT_EQ("xgc.v1.Empty", result.output().schema().type_name());
  EXPECT_EQ(1u, result.output().schema().schema_version());
  EXPECT_EQ(metadata.fingerprint, result.output().schema().schema_fingerprint());
  EXPECT_EQ(xgc::v1::PAYLOAD_ENCODING_PROTOBUF, result.output().encoding());
  xgc::v1::Empty empty; EXPECT_TRUE(empty.ParseFromString(result.output().value()));
}

} // namespace
} // namespace xgc2_ros1_robot_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
