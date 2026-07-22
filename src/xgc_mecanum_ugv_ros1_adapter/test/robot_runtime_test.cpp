#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

#include "xgc_mecanum_ugv_ros1_adapter/generated_contract.hpp"
#include "xgc_mecanum_ugv_ros1_adapter/motion_command.hpp"
#include "xgc_mecanum_ugv_ros1_adapter/robot_runtime.hpp"
#include "xgc_mecanum_ugv_ros1_adapter/shutdown_signal.hpp"
#include "xgc_mecanum_ugv_ros1_adapter/telemetry_batch.hpp"

namespace xgc_mecanum_ugv_ros1_adapter {
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

TEST(RosNames, BuildsNamespacedMecanumTopics) {
  EXPECT_EQ("/ugv1/cmd_vel", topicName("/ugv1", "cmd_vel"));
  EXPECT_EQ("/vrpn_client_node/Ugv1/pose",
            topicName("", "vrpn_client_node/Ugv1/pose"));
  EXPECT_EQ("/fleet/ugv2/twist", topicName("/fleet/ugv2", "/twist"));
}

TEST(RosNames, AcceptsOnlyCanonicalAbsoluteRobotNamespaces) {
  std::string error;
  EXPECT_TRUE(validRobotNamespace("/ugv1", &error));

  error.clear();
  EXPECT_FALSE(validRobotNamespace("ugv1", &error));
  EXPECT_EQ("namespace must be absolute", error);

  error.clear();
  EXPECT_FALSE(validRobotNamespace("/ugv1/", &error));
  EXPECT_EQ("namespace must not have a trailing slash", error);

  error.clear();
  EXPECT_FALSE(validRobotNamespace("/fleet//ugv1", &error));
  EXPECT_EQ("namespace must not contain repeated slashes", error);
}

TEST(RosNames, AcceptsOnlyCanonicalMocapRigidBodyNames) {
  std::string error;
  EXPECT_TRUE(validMocapRigidBodyName("ugv1", &error));
  EXPECT_TRUE(validMocapRigidBodyName("Mecanum_01", &error));
  EXPECT_FALSE(validMocapRigidBodyName("ugv-01", &error));
  EXPECT_FALSE(validMocapRigidBodyName("/vrpn/ugv1", &error));
  EXPECT_FALSE(validMocapRigidBodyName("ugv 1", &error));
}

TEST(RosNames, ResolvesMotionOutputFromProfileAndRobotNamespace) {
  xgc2_ros1_robot_adapter::RobotConfig config;
  config.profile_id = contract::kProfileId;
  config.parameters["namespace"] = "/fleet/ugv1";
  config.parameters["mocap_rigid_body"] = "Ugv1";

  std::string topic;
  std::string error;
  ASSERT_TRUE(resolveMotionCommandTopic(config, &topic, &error)) << error;
  EXPECT_EQ("/fleet/ugv1/cmd_vel", topic);
}

TEST(MotionIntent, MapsThreeGearsToTheDeployedSssLimits) {
  geometry_msgs::Twist command;
  std::string error;

  ASSERT_TRUE(motionIntentCommand(1, 1, 1, &command, &error)) << error;
  EXPECT_DOUBLE_EQ(0.5, command.linear.x);
  EXPECT_NEAR(0.5235987755982988, command.angular.z, 1e-15);
  EXPECT_DOUBLE_EQ(0.0, command.linear.y);
  EXPECT_DOUBLE_EQ(0.0, command.angular.x);

  ASSERT_TRUE(motionIntentCommand(2, -1, -1, &command, &error)) << error;
  EXPECT_DOUBLE_EQ(-1.0, command.linear.x);
  EXPECT_NEAR(-1.0471975511965976, command.angular.z, 1e-15);

  ASSERT_TRUE(motionIntentCommand(3, 1, 1, &command, &error)) << error;
  EXPECT_DOUBLE_EQ(kMecanumMaximumLinearVelocityMetersPerSecond,
                   command.linear.x);
  EXPECT_DOUBLE_EQ(kMecanumMaximumAngularVelocityRadiansPerSecond,
                   command.angular.z);

  ASSERT_TRUE(motionIntentCommand(3, 0, 0, &command, &error)) << error;
  EXPECT_DOUBLE_EQ(0.0, command.linear.x);
  EXPECT_DOUBLE_EQ(0.0, command.linear.y);
  EXPECT_DOUBLE_EQ(0.0, command.angular.z);
}

TEST(MotionIntent, RejectsValuesOutsideTheClosedExistingContract) {
  geometry_msgs::Twist command;
  std::string error;
  EXPECT_FALSE(motionIntentCommand(0, 0, 0, &command, &error));
  EXPECT_FALSE(motionIntentCommand(4, 0, 0, &command, &error));
  EXPECT_FALSE(motionIntentCommand(1, -2, 0, &command, &error));
  EXPECT_FALSE(motionIntentCommand(1, 0, 2, &command, &error));
  EXPECT_FALSE(motionIntentCommand(1, 0, 0, nullptr, &error));
}

TEST(MotionPublisher, StartsPassiveThenRepublishesAndStopsWithZero) {
  std::vector<geometry_msgs::Twist> published;
  MotionCommandPublisher publisher(
      [&published](const geometry_msgs::Twist &command) {
        published.push_back(command);
      });

  const ros::WallTime now = ros::WallTime::now();
  publisher.PublishPeriodic(now);
  EXPECT_TRUE(published.empty());

  std::string error;
  std::uint64_t generation = 0u;
  ASSERT_TRUE(publisher.SetIntent("session-a", 2, 1, -1,
                                  now + ros::WallDuration(2.0), &generation,
                                  &error))
      << error;
  ASSERT_EQ(1u, published.size());
  EXPECT_DOUBLE_EQ(1.0, published.back().linear.x);
  EXPECT_NEAR(-1.0471975511965976, published.back().angular.z, 1e-15);

  publisher.PublishPeriodic(now + ros::WallDuration(0.5));
  ASSERT_EQ(2u, published.size());
  EXPECT_DOUBLE_EQ(1.0, published.back().linear.x);

  ASSERT_TRUE(publisher.SetIntent("session-a", 2, 0, 0,
                                  now + ros::WallDuration(2.0), &generation,
                                  &error))
      << error;
  ASSERT_EQ(3u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().linear.x);
  EXPECT_DOUBLE_EQ(0.0, published.back().angular.z);

  publisher.Stop();
  ASSERT_EQ(4u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().linear.x);
  EXPECT_DOUBLE_EQ(0.0, published.back().angular.z);
  publisher.PublishPeriodic();
  EXPECT_EQ(4u, published.size());
  EXPECT_FALSE(publisher.SetIntent("session-a", 1, 1, 0,
                                   now + ros::WallDuration(2.0), &generation,
                                   &error));
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
  const ros::WallTime now = ros::WallTime::now();
  std::uint64_t generation = 0u;
  ASSERT_TRUE(publisher.SetIntent("session-a", 1, 1, 0,
                                  now + ros::WallDuration(2.0), &generation,
                                  &error))
      << error;
  ASSERT_FALSE(publisher.SetIntent("session-b", 3, 1, 0,
                                   now + ros::WallDuration(2.0), &generation,
                                   &error));
  EXPECT_NE(std::string::npos, error.find("injected publication failure"));

  publisher.PublishPeriodic(now + ros::WallDuration(0.5));
  ASSERT_EQ(2u, published.size());
  EXPECT_DOUBLE_EQ(0.5, published.back().linear.x);
}

TEST(MotionPublisher, LeasePulseExtendsWithoutAZeroGapThenExpiresToZero) {
  std::vector<geometry_msgs::Twist> published;
  MotionCommandPublisher publisher(
      [&published](const geometry_msgs::Twist &command) {
        published.push_back(command);
      });
  const ros::WallTime now = ros::WallTime::now();
  std::string error;
  std::uint64_t initial_generation = 0u;
  ASSERT_TRUE(publisher.SetIntent("session-a", 2, 1, 0,
                                  now + ros::WallDuration(0.5),
                                  &initial_generation, &error))
      << error;
  std::uint64_t pulse_generation = 0u;
  ASSERT_TRUE(publisher.RenewIntent("session-a", 2, 1, 0,
                                    now + ros::WallDuration(1.0),
                                    &pulse_generation, &error))
      << error;
  EXPECT_GT(pulse_generation, initial_generation);
  ASSERT_EQ(1u, published.size());

  publisher.PublishPeriodic(now + ros::WallDuration(0.75));
  ASSERT_EQ(2u, published.size());
  EXPECT_DOUBLE_EQ(1.0, published.back().linear.x);
  publisher.PublishPeriodic(now + ros::WallDuration(1.0));
  ASSERT_EQ(3u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().linear.x);
  EXPECT_DOUBLE_EQ(0.0, published.back().angular.z);
  publisher.PublishPeriodic(now + ros::WallDuration(1.1));
  EXPECT_EQ(3u, published.size());
}

TEST(MotionPublisher, StalePulseAndReleaseCannotChangeANewerIntent) {
  std::vector<geometry_msgs::Twist> published;
  MotionCommandPublisher publisher(
      [&published](const geometry_msgs::Twist &command) {
        published.push_back(command);
      });
  const ros::WallTime now = ros::WallTime::now();
  std::string error;
  std::uint64_t first_generation = 0u;
  ASSERT_TRUE(publisher.SetIntent("session-a", 1, 1, 0,
                                  now + ros::WallDuration(0.5),
                                  &first_generation, &error))
      << error;
  std::uint64_t pulse_generation = 0u;
  ASSERT_TRUE(publisher.RenewIntent("session-a", 1, 1, 0,
                                    now + ros::WallDuration(0.8),
                                    &pulse_generation, &error))
      << error;
  publisher.Release("session-a", first_generation);
  EXPECT_EQ(1u, published.size());

  std::uint64_t second_generation = 0u;
  ASSERT_TRUE(publisher.SetIntent("session-a", 2, -1, 1,
                                  now + ros::WallDuration(1.0),
                                  &second_generation, &error))
      << error;
  publisher.Release("session-a", pulse_generation);
  ASSERT_EQ(2u, published.size());
  EXPECT_DOUBLE_EQ(-1.0, published.back().linear.x);
  EXPECT_NEAR(1.0471975511965976, published.back().angular.z, 1e-15);

  EXPECT_FALSE(publisher.RenewIntent("session-a", 1, 1, 0,
                                     now + ros::WallDuration(1.2),
                                     &pulse_generation, &error));
  EXPECT_FALSE(publisher.RenewIntent("session-b", 2, -1, 1,
                                     now + ros::WallDuration(1.2),
                                     &pulse_generation, &error));
  publisher.PublishPeriodic(now + ros::WallDuration(0.9));
  ASSERT_EQ(3u, published.size());
  EXPECT_DOUBLE_EQ(-1.0, published.back().linear.x);
}

TEST(MotionPublisher, InactivePulseRevokesOnlyTheCurrentOwner) {
  std::vector<geometry_msgs::Twist> published;
  MotionCommandPublisher publisher(
      [&published](const geometry_msgs::Twist &command) {
        published.push_back(command);
      });
  const ros::WallTime now = ros::WallTime::now();
  std::string error;
  std::uint64_t generation = 0u;
  ASSERT_TRUE(publisher.SetIntent("session-a", 1, 1, 0,
                                  now + ros::WallDuration(1.0), &generation,
                                  &error))
      << error;
  EXPECT_FALSE(publisher.RevokeIntent("session-b", &error));
  EXPECT_EQ(1u, published.size());
  ASSERT_TRUE(publisher.RevokeIntent("session-a", &error)) << error;
  ASSERT_EQ(2u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().linear.x);
  ASSERT_TRUE(publisher.RevokeIntent("session-a", &error)) << error;
  EXPECT_EQ(2u, published.size());
}

TEST(Freshness, AppliesTheVrpnPositionBoundary) {
  const ros::WallTime now(10, 0);
  contract::ChannelMetadata position{};
  ASSERT_TRUE(contract::channelMetadata(contract::kProfileId, "vrpn.position",
                                        &position));
  const double stale_after_seconds =
      static_cast<double>(position.stale_after_millis) / 1000.0;
  EXPECT_FALSE(sourceIsFresh(ros::WallTime(), now, stale_after_seconds));
  EXPECT_TRUE(sourceIsFresh(ros::WallTime(9, 0), now, stale_after_seconds));
  EXPECT_FALSE(
      sourceIsFresh(ros::WallTime(8, 999999999), now, stale_after_seconds));
}

TEST(VrpnSpeedProjection, ProjectsWorldVelocityOntoSignedBodyXAxis) {
  const double half_sqrt_two = std::sqrt(0.5);
  EXPECT_DOUBLE_EQ(
      3.0, vrpnForwardSpeedMetersPerSecond(3.0, 4.0, 12.0,
                                           0.0, 0.0, 0.0, 1.0));
  EXPECT_NEAR(
      4.0, vrpnForwardSpeedMetersPerSecond(0.0, 4.0, 2.0, 0.0, 0.0,
                                           half_sqrt_two, half_sqrt_two),
      1e-12);
  EXPECT_NEAR(
      0.0, vrpnForwardSpeedMetersPerSecond(-5.0, 0.0, 0.0, 0.0, 0.0,
                                           half_sqrt_two, half_sqrt_two),
      1e-12);
  EXPECT_DOUBLE_EQ(
      -3.0, vrpnForwardSpeedMetersPerSecond(-3.0, 4.0, 0.0,
                                            0.0, 0.0, 0.0, 2.0));
  EXPECT_TRUE(std::isnan(vrpnForwardSpeedMetersPerSecond(
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)));
}

TEST(InstalledProfile, IsTheMinimalMecanumContractAtTenHertz) {
  std::string error;
  EXPECT_TRUE(validateNativeProfileContract(&error)) << error;
  EXPECT_EQ("mecanum-ugv.ros1.v1", std::string(contract::kProfileId));

  std::size_t channel_count = 0u;
  const auto *channels =
      contract::profileChannels(contract::kProfileId, &channel_count);
  ASSERT_NE(nullptr, channels);
  ASSERT_EQ(6u, channel_count);

  const std::vector<std::string> streams{
      "vrpn.position", "vrpn.velocity", "vrpn.speed", "command.velocity",
      "diagnostic.channel-health"};
  for (const auto &channel_id : streams) {
    contract::ChannelMetadata channel{};
    ASSERT_TRUE(
        contract::channelMetadata(contract::kProfileId, channel_id, &channel));
    EXPECT_EQ(contract::ChannelKind::kStreamOut, channel.kind);
    EXPECT_DOUBLE_EQ(10.0, channel.output_rate_hz);
  }

  contract::ChannelMetadata position{};
  contract::ChannelMetadata raw_velocity{};
  contract::ChannelMetadata speed{};
  contract::ChannelMetadata command{};
  ASSERT_TRUE(contract::channelMetadata(contract::kProfileId, "vrpn.position",
                                        &position));
  ASSERT_TRUE(contract::channelMetadata(contract::kProfileId, "vrpn.velocity",
                                        &raw_velocity));
  ASSERT_TRUE(contract::channelMetadata(contract::kProfileId, "vrpn.speed",
                                        &speed));
  ASSERT_TRUE(contract::channelMetadata(contract::kProfileId,
                                        "command.velocity", &command));
  EXPECT_EQ(2001u, position.output_message_id);
  EXPECT_EQ(2002u, raw_velocity.output_message_id);
  EXPECT_EQ(2006u, speed.output_message_id);
  EXPECT_EQ(2002u, command.output_message_id);

  contract::ChannelMetadata forbidden{};
  EXPECT_FALSE(contract::channelMetadata(contract::kProfileId, "state.odom",
                                         &forbidden));
  EXPECT_FALSE(contract::channelMetadata(contract::kProfileId, "state.health",
                                         &forbidden));
  EXPECT_FALSE(contract::channelMetadata(contract::kProfileId, "state.chassis",
                                         &forbidden));
}

TEST(InstalledProfile, KeepsTheExistingLongitudinalYawOperation) {
  contract::ChannelMetadata motion{};
  ASSERT_TRUE(contract::channelMetadata(contract::kProfileId,
                                        "operation.motion-intent", &motion));
  EXPECT_EQ(contract::ChannelKind::kOperation, motion.kind);
  EXPECT_EQ(3204u, motion.input_message_id);
  EXPECT_EQ(1u, motion.output_message_id);
  EXPECT_EQ("mecanum-ugv.set-motion-intent", std::string(motion.processor));
  EXPECT_EQ("pulse-motion-intent-lease",
            std::string(motion.operation_lease.pulse_endpoint_id));
  EXPECT_EQ(250u, motion.operation_lease.heartbeat_interval_millis);
  EXPECT_EQ(750u, motion.operation_lease.timeout_millis);
  EXPECT_TRUE(motion.operation_lease.volatile_supported);
  EXPECT_EQ("{\"longitudinal\":0,\"yaw\":0}",
            std::string(
                motion.operation_lease.inactive_parameter_values_json));
  const auto *output = contract::channelEndpoint(
      motion, contract::EndpointKind::kOutput, "output");
  ASSERT_NE(nullptr, output);
  EXPECT_EQ("cmd_vel", std::string(output->name_template));
  EXPECT_EQ("geometry_msgs/Twist", std::string(output->ros_type));

  std::size_t operation_count = 0u;
  const auto *operations =
      contract::profileOperations(contract::kProfileId, &operation_count);
  ASSERT_NE(nullptr, operations);
  ASSERT_EQ(1u, operation_count);
  EXPECT_EQ("set-motion-intent", std::string(operations[0].operation_id));
}

TEST(TelemetryBatch, BindsItemsToAnExactQueuePrefix) {
  const std::deque<TelemetryQueueItem> queue{
      {1u, "pose"}, {2u, "velocity"}, {3u, "speed"}};
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

TEST(TelemetryBatch, EnforcesItemAndByteLimitsWithoutSplittingItems) {
  const std::deque<TelemetryQueueItem> queue{
      {1u, "1234"}, {2u, "5678"}, {3u, "9"}};
  const auto byte_limited = buildTelemetryBatch(queue, 3u, 8u);
  EXPECT_EQ(2u, byte_limited.items.size());
  EXPECT_EQ(8u, byte_limited.bytes);
  EXPECT_EQ(1u, buildTelemetryBatch(queue, 1u, 1024u).items.size());
  EXPECT_TRUE(buildTelemetryBatch(queue, 0u, 1024u).items.empty());
  EXPECT_TRUE(buildTelemetryBatch(queue, 3u, 3u).items.empty());
}

} // namespace
} // namespace xgc_mecanum_ugv_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
