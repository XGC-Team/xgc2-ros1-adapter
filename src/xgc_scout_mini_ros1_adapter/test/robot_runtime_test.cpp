#include <gtest/gtest.h>

#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

#include "xgc_scout_mini_ros1_adapter/generated_contract.hpp"
#include "xgc_scout_mini_ros1_adapter/motion_command.hpp"
#include "xgc_scout_mini_ros1_adapter/robot_runtime.hpp"
#include "xgc_scout_mini_ros1_adapter/shutdown_signal.hpp"
#include "xgc_scout_mini_ros1_adapter/telemetry_batch.hpp"

namespace xgc_scout_mini_ros1_adapter {
namespace {

TEST(ShutdownSignal, CapturesSigintAndSigtermWithoutTerminating) {
  ShutdownSignalHandler handler;
  EXPECT_FALSE(handler.requested());

  ASSERT_EQ(0, raise(SIGINT));
  EXPECT_TRUE(handler.requested());
  EXPECT_EQ(SIGINT, handler.signalNumber());

  ASSERT_EQ(0, raise(SIGTERM));
  EXPECT_TRUE(handler.requested());
  EXPECT_EQ(SIGTERM, handler.signalNumber());
}

TEST(RosNames, BuildsNamespacedScoutTopics) {
  EXPECT_EQ("/scout1/cmd_vel", topicName("/scout1", "cmd_vel"));
  EXPECT_EQ("/vrpn_client_node/Scout1/pose",
            topicName("", "vrpn_client_node/Scout1/pose"));
  EXPECT_EQ("/fleet/scout2/scout_status",
            topicName("/fleet/scout2", "/scout_status"));
  EXPECT_EQ("/fleet/scout2/PowerVoltage",
            topicName("/fleet/scout2", "PowerVoltage"));
  EXPECT_EQ("/fleet/scout2/scout/chassis_state",
            topicName("/fleet/scout2", "scout/chassis_state"));
}

TEST(RosNames, AcceptsOnlyCanonicalAbsoluteRobotNamespaces) {
  std::string error;
  EXPECT_TRUE(validRobotNamespace("/scout1", &error));

  error.clear();
  EXPECT_FALSE(validRobotNamespace("scout1", &error));
  EXPECT_EQ("namespace must be absolute", error);

  error.clear();
  EXPECT_FALSE(validRobotNamespace("/scout1/", &error));
  EXPECT_EQ("namespace must not have a trailing slash", error);

  error.clear();
  EXPECT_FALSE(validRobotNamespace("/fleet//scout1", &error));
  EXPECT_EQ("namespace must not contain repeated slashes", error);
}

TEST(RosNames, AcceptsOnlyCanonicalMocapRigidBodyNames) {
  std::string error;
  EXPECT_TRUE(validMocapRigidBodyName("ugv1", &error));
  EXPECT_TRUE(validMocapRigidBodyName("Scout_01", &error));
  EXPECT_FALSE(validMocapRigidBodyName("scout-01", &error));
  EXPECT_FALSE(validMocapRigidBodyName("/vrpn/scout1", &error));
  EXPECT_FALSE(validMocapRigidBodyName("scout 1", &error));
}

TEST(RosNames, ResolvesTheMotionOutputFromTheProfileAndRobotNamespace) {
  xgc2_ros1_robot_adapter::RobotConfig config;
  config.profile_id = contract::kProfileId;
  config.parameters["namespace"] = "/fleet/scout1";
  config.parameters["mocap_rigid_body"] = "Scout1";

  std::string topic;
  std::string error;
  ASSERT_TRUE(resolveMotionCommandTopic(config, &topic, &error)) << error;
  EXPECT_EQ("/fleet/scout1/cmd_vel", topic);
}

TEST(MotionIntent, MapsThreeGearsAndClampsToScoutSdkLimits) {
  geometry_msgs::Twist command;
  std::string error;

  ASSERT_TRUE(motionIntentCommand(1, 1, 1, 1, &command, &error)) << error;
  EXPECT_DOUBLE_EQ(0.5, command.linear.x);
  EXPECT_DOUBLE_EQ(0.1745, command.angular.z);
  EXPECT_DOUBLE_EQ(0.0, command.linear.y);
  EXPECT_DOUBLE_EQ(0.0, command.angular.x);

  ASSERT_TRUE(motionIntentCommand(2, -1, 0, -1, &command, &error)) << error;
  EXPECT_DOUBLE_EQ(-1.0, command.linear.x);
  EXPECT_DOUBLE_EQ(-0.349, command.angular.z);

  ASSERT_TRUE(motionIntentCommand(3, 1, 0, 1, &command, &error)) << error;
  EXPECT_DOUBLE_EQ(kScoutMaximumLinearVelocityMetersPerSecond,
                   command.linear.x);
  EXPECT_DOUBLE_EQ(kScoutMaximumAngularVelocityRadiansPerSecond,
                   command.angular.z);

  ASSERT_TRUE(motionIntentCommand(3, 0, 0, 0, &command, &error)) << error;
  EXPECT_DOUBLE_EQ(0.0, command.linear.x);
  EXPECT_DOUBLE_EQ(0.0, command.angular.z);
}

TEST(MotionIntent, RejectsValuesOutsideTheClosedDiscreteContract) {
  geometry_msgs::Twist command;
  std::string error;
  EXPECT_FALSE(motionIntentCommand(0, 0, 0, 0, &command, &error));
  EXPECT_FALSE(motionIntentCommand(4, 0, 0, 0, &command, &error));
  EXPECT_FALSE(motionIntentCommand(1, -2, 0, 0, &command, &error));
  EXPECT_FALSE(motionIntentCommand(1, 0, 2, 0, &command, &error));
  EXPECT_FALSE(motionIntentCommand(1, 0, 0, 2, &command, &error));
  EXPECT_FALSE(motionIntentCommand(1, 0, 0, 0, nullptr, &error));
}

TEST(MotionPublisher, StartsPassiveThenRepublishesAndStopsWithZero) {
  std::vector<geometry_msgs::Twist> published;
  MotionCommandPublisher publisher(
      [&published](const geometry_msgs::Twist &command) {
        published.push_back(command);
      });

  publisher.PublishPeriodic();
  EXPECT_TRUE(published.empty());

  std::string error;
  ASSERT_TRUE(publisher.SetIntent(2, 1, 0, -1, &error)) << error;
  ASSERT_EQ(1u, published.size());
  EXPECT_DOUBLE_EQ(1.0, published.back().linear.x);
  EXPECT_DOUBLE_EQ(-0.349, published.back().angular.z);

  publisher.PublishPeriodic();
  ASSERT_EQ(2u, published.size());
  EXPECT_DOUBLE_EQ(1.0, published.back().linear.x);

  ASSERT_TRUE(publisher.SetIntent(2, 0, 0, 0, &error)) << error;
  ASSERT_EQ(3u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().linear.x);
  EXPECT_DOUBLE_EQ(0.0, published.back().angular.z);

  publisher.Stop();
  ASSERT_EQ(4u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().linear.x);
  EXPECT_DOUBLE_EQ(0.0, published.back().angular.z);
  publisher.PublishPeriodic();
  EXPECT_EQ(4u, published.size());
  EXPECT_FALSE(publisher.SetIntent(1, 1, 0, 0, &error));
}

TEST(MotionPublisher, FailedPublicationDoesNotCommitTheNewIntent) {
  std::vector<geometry_msgs::Twist> published;
  MotionCommandPublisher publisher(
      [&published](const geometry_msgs::Twist &command) {
        if (command.linear.x == 1.5)
          throw std::runtime_error("injected publication failure");
        published.push_back(command);
      });

  std::string error;
  ASSERT_TRUE(publisher.SetIntent(1, 1, 0, 0, &error)) << error;
  ASSERT_FALSE(publisher.SetIntent(3, 1, 0, 0, &error));
  EXPECT_NE(std::string::npos, error.find("injected publication failure"));

  publisher.PublishPeriodic();
  ASSERT_EQ(2u, published.size());
  EXPECT_DOUBLE_EQ(0.5, published.back().linear.x);
}

TEST(Freshness, AppliesTheScoutStatusBoundary) {
  const ros::WallTime now(10, 0);
  contract::ChannelMetadata health{};
  ASSERT_TRUE(
      contract::channelMetadata(contract::kProfileId, "state.health", &health));
  const double stale_after_seconds =
      static_cast<double>(health.stale_after_millis) / 1000.0;
  const ros::WallTime freshness_boundary =
      now - ros::WallDuration(stale_after_seconds);
  EXPECT_FALSE(sourceIsFresh(ros::WallTime(), now, stale_after_seconds));
  EXPECT_TRUE(sourceIsFresh(freshness_boundary, now, stale_after_seconds));
  EXPECT_FALSE(sourceIsFresh(freshness_boundary - ros::WallDuration(0, 1), now,
                             stale_after_seconds));
}

TEST(OnlineProjection, FollowsTheDeclaredChassisStatusInput) {
  EXPECT_TRUE(scoutIsOnline(true));
  EXPECT_FALSE(scoutIsOnline(false));
}

TEST(VrpnSpeedProjection, ProjectsWorldVelocityOntoTheSignedBodyXAxis) {
  const double half_sqrt_two = std::sqrt(0.5);
  EXPECT_DOUBLE_EQ(
      3.0, vrpnForwardSpeedMetersPerSecond(3.0, 4.0, 12.0, 0.0, 0.0, 0.0, 1.0));
  EXPECT_NEAR(4.0,
              vrpnForwardSpeedMetersPerSecond(0.0, 4.0, 2.0, 0.0, 0.0,
                                              half_sqrt_two, half_sqrt_two),
              1e-12);
  EXPECT_NEAR(0.0,
              vrpnForwardSpeedMetersPerSecond(-5.0, 0.0, 0.0, 0.0, 0.0,
                                              half_sqrt_two, half_sqrt_two),
              1e-12);
  EXPECT_DOUBLE_EQ(-3.0, vrpnForwardSpeedMetersPerSecond(-3.0, 4.0, 0.0, 0.0,
                                                         0.0, 0.0, 2.0));
  EXPECT_TRUE(std::isnan(
      vrpnForwardSpeedMetersPerSecond(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)));
}

TEST(BatteryProjection, UsesTheManualLinearVoltageModel) {
  EXPECT_DOUBLE_EQ(0.0, scoutBatteryPercentage(20.5));
  EXPECT_DOUBLE_EQ(1.0, scoutBatteryPercentage(29.2));
  EXPECT_NEAR(0.95, scoutBatteryPercentage(28.765), 1e-12);
  EXPECT_DOUBLE_EQ(0.0, scoutBatteryPercentage(18.0));
  EXPECT_DOUBLE_EQ(1.0, scoutBatteryPercentage(32.0));
}

TEST(ChassisState, PacksAndUnpacksModeBaseAndFault) {
  const std::uint32_t word = packScoutChassisState(1u, 2u, 0xABCDu);
  EXPECT_EQ(0xABCDu << 16 | 2u << 8 | 1u, word);
  ScoutChassisState parsed;
  ASSERT_TRUE(unpackScoutChassisState(word, &parsed));
  EXPECT_EQ(1u, parsed.control_mode);
  EXPECT_EQ(2u, parsed.base_state);
  EXPECT_EQ(0xABCDu, parsed.fault_code);
  EXPECT_FALSE(unpackScoutChassisState(0u, nullptr));
}

TEST(ChassisProjection, MapsNativeScoutControlModes) {
  using Status = xgc::semantic::ground::v1::ChassisStatus;
  EXPECT_EQ(Status::CONTROL_MODE_REMOTE, scoutControlMode(0));
  EXPECT_EQ(Status::CONTROL_MODE_COMMAND_CAN, scoutControlMode(1));
  EXPECT_EQ(Status::CONTROL_MODE_COMMAND_UART, scoutControlMode(2));
  EXPECT_EQ(Status::CONTROL_MODE_UNSPECIFIED, scoutControlMode(255));
}

TEST(InstalledProfile, KeepsRobotMetadataOutOfTheRuntimeProtocol) {
  std::string error;
  EXPECT_TRUE(validateNativeProfileContract(&error)) << error;
  const char *digest = contract::profileDigest("scout-mini.ros1.v6");
  ASSERT_NE(nullptr, digest);
  EXPECT_EQ(64u, std::string(digest).size());

  contract::ChannelMetadata position;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v6", "vrpn.position",
                                        &position));
  EXPECT_EQ(contract::ChannelKind::kStreamOut, position.kind);
  EXPECT_EQ(2001u, position.output_message_id);
  EXPECT_FALSE(
      contract::channelMetadata("scout-mini.ros1.v6", "state.pose", &position));

  contract::ChannelMetadata vrpn_velocity;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v6", "vrpn.velocity",
                                        &vrpn_velocity));
  EXPECT_EQ(2002u, vrpn_velocity.output_message_id);

  contract::ChannelMetadata vrpn_speed;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v6", "vrpn.speed",
                                        &vrpn_speed));
  EXPECT_EQ(2006u, vrpn_speed.output_message_id);

  contract::ChannelMetadata command_velocity;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v6",
                                        "command.velocity", &command_velocity));
  EXPECT_EQ(contract::ChannelKind::kStreamOut, command_velocity.kind);
  EXPECT_EQ(2002u, command_velocity.output_message_id);

  contract::ChannelMetadata diagnostics;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v6",
                                        "diagnostic.stream-health",
                                        &diagnostics));
  EXPECT_EQ(2011u, diagnostics.output_message_id);
  EXPECT_EQ("common.stream-health-report", std::string(diagnostics.processor));
  EXPECT_FALSE(contract::channelMetadata("scout-mini.ros1.v6",
                                         "diagnostic.channel-health",
                                         &diagnostics));

  contract::ChannelMetadata unknown;
  EXPECT_FALSE(contract::channelMetadata("scout-mini.ros1.v6", "operation.arm",
                                         &unknown));

  contract::ChannelMetadata motion;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v6",
                                        "operation.motion-intent", &motion));
  EXPECT_EQ(contract::ChannelKind::kOperation, motion.kind);
  EXPECT_EQ(3205u, motion.input_message_id);
  EXPECT_EQ(1u, motion.output_message_id);
  const auto *output = contract::channelEndpoint(
      motion, contract::EndpointKind::kOutput, "output");
  ASSERT_NE(nullptr, output);
  EXPECT_EQ("cmd_vel", std::string(output->name_template));
  EXPECT_EQ("geometry_msgs/Twist", std::string(output->ros_type));

  std::size_t operation_count = 0u;
  const auto *operations =
      contract::profileOperations("scout-mini.ros1.v6", &operation_count);
  ASSERT_NE(nullptr, operations);
  ASSERT_EQ(1u, operation_count);
  EXPECT_EQ("set-motion-intent", std::string(operations[0].operation_id));
}

TEST(InstalledContract, ContainsScoutMotionButNotPx4Operations) {
  contract::MessageMetadata metadata;
  EXPECT_TRUE(contract::messageMetadata(2001, &metadata));
  EXPECT_TRUE(contract::messageMetadata(3102, &metadata));
  EXPECT_TRUE(contract::messageMetadata(3205, &metadata));
  EXPECT_EQ("xgc.semantic.common.v1.RemoteControlIntentRequest",
            std::string(metadata.type_name));
  EXPECT_FALSE(contract::messageMetadata(3201, &metadata));
}

TEST(TelemetryBatch, BindsItemsToAnExactQueuePrefix) {
  const std::deque<TelemetryQueueItem> queue{
      {1u, "pose"}, {2u, "velocity"}, {3u, "power"}};
  const auto batch = buildTelemetryBatch(queue, 2u, 1024u);
  ASSERT_EQ(2u, batch.items.size());
  EXPECT_EQ((std::vector<std::uint64_t>{1u, 2u}), batch.tokens);
  EXPECT_EQ((std::vector<std::string>{"pose", "velocity"}), batch.items);
  EXPECT_EQ(12u, batch.bytes);
  EXPECT_TRUE(telemetryBatchMatchesPrefix(queue, batch.tokens));

  auto advanced = queue;
  advanced.pop_front();
  EXPECT_FALSE(telemetryBatchMatchesPrefix(advanced, batch.tokens));
}

TEST(TelemetryBatch, EnforcesBothItemAndByteLimitsWithoutSplittingItems) {
  const std::deque<TelemetryQueueItem> queue{
      {1u, "1234"}, {2u, "5678"}, {3u, "9"}};
  const auto byte_limited = buildTelemetryBatch(queue, 3u, 8u);
  EXPECT_EQ(2u, byte_limited.items.size());
  EXPECT_EQ(8u, byte_limited.bytes);

  const auto item_limited = buildTelemetryBatch(queue, 1u, 1024u);
  ASSERT_EQ(1u, item_limited.items.size());
  EXPECT_EQ("1234", item_limited.items.front());

  EXPECT_TRUE(buildTelemetryBatch(queue, 0u, 1024u).items.empty());
  EXPECT_TRUE(buildTelemetryBatch(queue, 3u, 3u).items.empty());
}

TEST(TelemetryBatch, DrainsQueuesLargerThanTheProtocolWindowInBoundedBatches) {
  std::deque<TelemetryQueueItem> queue;
  for (std::uint64_t token = 1u; token <= 37u; ++token)
    queue.push_back({token, "telemetry"});

  std::vector<std::size_t> batch_sizes;
  std::uint64_t expected_token = 1u;
  while (!queue.empty()) {
    const auto batch =
        buildTelemetryBatch(queue, 256u, kMaximumTelemetryBatchBytes);
    ASSERT_FALSE(batch.items.empty());
    ASSERT_LE(batch.items.size(), kMaximumTelemetryBatchItems);
    ASSERT_EQ(batch.items.size(), batch.tokens.size());
    for (const auto token : batch.tokens)
      EXPECT_EQ(expected_token++, token);
    batch_sizes.push_back(batch.items.size());
    for (std::size_t index = 0u; index < batch.items.size(); ++index)
      queue.pop_front();
  }

  EXPECT_EQ((std::vector<std::size_t>{16u, 16u, 5u}), batch_sizes);
  EXPECT_EQ(38u, expected_token);
}

} // namespace
} // namespace xgc_scout_mini_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
