#include <gtest/gtest.h>

#include <deque>
#include <limits>
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

TEST(StreamRateEstimate, RetainsCredibleRatesAcrossFreshEmptyWindows) {
  const auto observed = updateStreamRateEstimate({}, 1u, 1u, 1.1, true);
  EXPECT_NEAR(0.909, observed.source_rate_hz, 0.001);
  EXPECT_NEAR(0.909, observed.output_rate_hz, 0.001);

  const auto empty =
      updateStreamRateEstimate(observed, 0u, 0u, 1.0, true);
  EXPECT_DOUBLE_EQ(observed.source_rate_hz, empty.source_rate_hz);
  EXPECT_DOUBLE_EQ(observed.output_rate_hz, empty.output_rate_hz);
}

TEST(StreamRateEstimate, ClearsRetainedRatesWhenTheSourceIsStale) {
  const StreamRateEstimate previous{0.9, 1.0};
  const auto stale =
      updateStreamRateEstimate(previous, 0u, 0u, 1.0, false);
  EXPECT_DOUBLE_EQ(0.0, stale.source_rate_hz);
  EXPECT_DOUBLE_EQ(0.0, stale.output_rate_hz);
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

TEST(VrpnAccelerationProjection, PreservesFrameAndBothTwistVectors) {
  geometry_msgs::TwistStamped source;
  source.header.frame_id = "world";
  source.twist.linear.x = 1.25;
  source.twist.linear.y = -2.5;
  source.twist.linear.z = 3.75;
  source.twist.angular.x = -0.1;
  source.twist.angular.y = 0.2;
  source.twist.angular.z = -0.3;

  const auto estimate = vrpnAccelerationEstimate(source);
  EXPECT_EQ("world", estimate.frame_id());
  EXPECT_DOUBLE_EQ(1.25, estimate.linear().x());
  EXPECT_DOUBLE_EQ(-2.5, estimate.linear().y());
  EXPECT_DOUBLE_EQ(3.75, estimate.linear().z());
  EXPECT_DOUBLE_EQ(-0.1, estimate.angular().x());
  EXPECT_DOUBLE_EQ(0.2, estimate.angular().y());
  EXPECT_DOUBLE_EQ(-0.3, estimate.angular().z());
}

TEST(BatteryProjection, UsesOnlyConfiguredCurvesAndKeepsMissingCurvesUnknown) {
  contract::ChannelMetadata power{};
  ASSERT_TRUE(contract::channelMetadata(contract::kProfileId, "state.power",
                                        &power));
  EXPECT_EQ(nullptr, contract::channelPolicy(
                         power, "battery_voltage_percentage_curve"));

  std::vector<xgc2_ros1_robot_adapter::BatteryCurvePoint> curve;
  double percentage = 0.0;
  EXPECT_FALSE(
      xgc2_ros1_robot_adapter::batteryPercentage(curve, 24.0, &percentage));

  const char *entries[] = {"10.0=0.0", "11.0=0.4", "12.0=1.0"};
  std::string error;
  ASSERT_TRUE(xgc2_ros1_robot_adapter::parseBatteryCurve(
      entries, 3u, &curve, &error)) << error;
  EXPECT_TRUE(
      xgc2_ros1_robot_adapter::batteryPercentage(curve, 9.0, &percentage));
  EXPECT_DOUBLE_EQ(0.0, percentage);
  EXPECT_TRUE(
      xgc2_ros1_robot_adapter::batteryPercentage(curve, 11.5, &percentage));
  EXPECT_NEAR(0.7, percentage, 1e-12);
  EXPECT_TRUE(
      xgc2_ros1_robot_adapter::batteryPercentage(curve, 13.0, &percentage));
  EXPECT_DOUBLE_EQ(1.0, percentage);
  EXPECT_FALSE(xgc2_ros1_robot_adapter::batteryPercentage(
      curve, std::numeric_limits<double>::quiet_NaN(), &percentage));
  EXPECT_TRUE(
      xgc2_ros1_robot_adapter::batteryPercentage(curve, 11.0, &percentage));
  EXPECT_DOUBLE_EQ(0.4, percentage);
}

TEST(PositioningHealth, DistinguishesStationaryJitterMotionTimeoutAndRecovery) {
  using xgc2_ros1_robot_adapter::PositioningHealthConfig;
  using xgc2_ros1_robot_adapter::PositioningHealthReason;
  using xgc2_ros1_robot_adapter::PositioningHealthState;
  using xgc2_ros1_robot_adapter::PositioningHealthWindow;
  const PositioningHealthConfig config{1.0, 5u, 0.05, 0.03, 1.0};

  PositioningHealthWindow stable(config);
  for (int index = 0; index < 5; ++index) {
    stable.recordPose(index * 0.1, index % 2 == 0 ? 0.002 : -0.002,
                      0.001, 0.0);
  }
  stable.recordVelocity(0.4, 0.0, 0.0, 0.0);
  const auto stable_result = stable.evaluate(0.45);
  EXPECT_EQ(PositioningHealthState::kStable, stable_result.state);
  EXPECT_EQ(PositioningHealthReason::kStationaryWindowStable,
            stable_result.reason);
  EXPECT_EQ(50u, stable_result.observed_age_ms);

  PositioningHealthWindow jitter(config);
  const double jitter_x[] = {0.0, 0.05, -0.05, 0.04, -0.04};
  for (int index = 0; index < 5; ++index)
    jitter.recordPose(index * 0.1, jitter_x[index], 0.0, 0.0);
  jitter.recordVelocity(0.4, 0.0, 0.0, 0.0);
  const auto jitter_result = jitter.evaluate(0.4);
  EXPECT_EQ(PositioningHealthState::kJittering, jitter_result.state);
  EXPECT_EQ(PositioningHealthReason::kStationaryJitterExceeded,
            jitter_result.reason);

  PositioningHealthWindow moving(config);
  for (int index = 0; index < 5; ++index)
    moving.recordPose(index * 0.1, index * 0.1, 0.0, 0.0);
  moving.recordVelocity(0.4, 0.5, 0.0, 0.0);
  const auto moving_result = moving.evaluate(0.4);
  EXPECT_EQ(PositioningHealthState::kMoving, moving_result.state);
  EXPECT_EQ(PositioningHealthReason::kRobotMoving, moving_result.reason);
  moving.recordVelocity(0.5, 0.0, 0.0, 0.0);
  moving.recordPose(0.6, 0.4, 0.0, 0.0);
  const auto stopped_result = moving.evaluate(0.6);
  EXPECT_EQ(PositioningHealthState::kWarmingUp, stopped_result.state);
  EXPECT_EQ(PositioningHealthReason::kInsufficientSamples,
            stopped_result.reason);
  for (int index = 1; index < 5; ++index)
    moving.recordPose(0.6 + index * 0.1, 0.4 + index * 0.001, 0.0, 0.0);
  moving.recordVelocity(1.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(PositioningHealthState::kStable, moving.evaluate(1.0).state);

  const auto timeout_result = stable.evaluate(1.6);
  EXPECT_EQ(PositioningHealthState::kTimedOut, timeout_result.state);
  EXPECT_EQ(PositioningHealthReason::kVrpnTimeout, timeout_result.reason);
  EXPECT_EQ(1200u, timeout_result.observed_age_ms);

  for (int index = 0; index < 5; ++index) {
    stable.recordPose(1.7 + index * 0.1, 1.0 + index * 0.001, 2.0, 0.0);
  }
  stable.recordVelocity(2.1, 0.0, 0.0, 0.0);
  EXPECT_EQ(PositioningHealthState::kStable, stable.evaluate(2.1).state);

  for (int index = 0; index < 5; ++index)
    jitter.recordPose(1.5 + index * 0.1, 3.0 + index * 0.001, 0.0, 0.0);
  jitter.recordVelocity(1.9, 0.0, 0.0, 0.0);
  EXPECT_EQ(PositioningHealthState::kStable, jitter.evaluate(1.9).state);
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
  EXPECT_EQ(Status::CONTROL_MODE_COMMAND_CAN, scoutControlMode(1));
  EXPECT_EQ(Status::CONTROL_MODE_REMOTE, scoutControlMode(3));
  EXPECT_EQ(Status::CONTROL_MODE_UNSPECIFIED, scoutControlMode(0));
  EXPECT_EQ(Status::CONTROL_MODE_UNSPECIFIED, scoutControlMode(2));
  EXPECT_EQ(Status::CONTROL_MODE_UNSPECIFIED, scoutControlMode(255));
}

TEST(InstalledProfile, KeepsRobotMetadataOutOfTheRuntimeProtocol) {
  std::string error;
  EXPECT_TRUE(validateNativeProfileContract(&error)) << error;
  const char *digest = contract::profileDigest("scout-mini.ros1.v7");
  ASSERT_NE(nullptr, digest);
  EXPECT_EQ(64u, std::string(digest).size());

  contract::ChannelMetadata position;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v7", "vrpn.position",
                                        &position));
  EXPECT_EQ(contract::ChannelKind::kStreamOut, position.kind);
  EXPECT_EQ(2001u, position.output_message_id);
  EXPECT_FALSE(
      contract::channelMetadata("scout-mini.ros1.v7", "state.pose", &position));

  contract::ChannelMetadata vrpn_velocity;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v7", "vrpn.velocity",
                                        &vrpn_velocity));
  EXPECT_EQ(2002u, vrpn_velocity.output_message_id);

  contract::ChannelMetadata vrpn_acceleration;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v7",
                                        "vrpn.acceleration",
                                        &vrpn_acceleration));
  EXPECT_EQ(2008u, vrpn_acceleration.output_message_id);

  contract::ChannelMetadata vrpn_speed;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v7", "vrpn.speed",
                                        &vrpn_speed));
  EXPECT_EQ(2006u, vrpn_speed.output_message_id);

  contract::ChannelMetadata command_velocity;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v7",
                                        "command.velocity", &command_velocity));
  EXPECT_EQ(contract::ChannelKind::kStreamOut, command_velocity.kind);
  EXPECT_EQ(2002u, command_velocity.output_message_id);

  contract::ChannelMetadata diagnostics;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v7",
                                        "diagnostic.stream-health",
                                        &diagnostics));
  EXPECT_EQ(2011u, diagnostics.output_message_id);
  EXPECT_EQ("common.stream-health-report", std::string(diagnostics.processor));
  EXPECT_FALSE(contract::channelMetadata("scout-mini.ros1.v7",
                                         "diagnostic.channel-health",
                                         &diagnostics));

  contract::ChannelMetadata health;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v7", "state.health",
                                        &health));
  EXPECT_EQ(2005u, health.output_message_id);
  EXPECT_EQ(3u, health.observes_count);
  EXPECT_EQ(4u, health.policy_count);

  contract::ChannelMetadata unknown;
  EXPECT_FALSE(contract::channelMetadata("scout-mini.ros1.v7", "operation.arm",
                                         &unknown));

  contract::ChannelMetadata motion;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v7",
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
      contract::profileOperations("scout-mini.ros1.v7", &operation_count);
  ASSERT_NE(nullptr, operations);
  ASSERT_EQ(1u, operation_count);
  EXPECT_EQ("set-motion-intent", std::string(operations[0].operation_id));
}

TEST(InstalledContract, ContainsScoutMotionButNotPx4Operations) {
  contract::MessageMetadata metadata;
  EXPECT_TRUE(contract::messageMetadata(2001, &metadata));
  ASSERT_TRUE(contract::messageMetadata(2004, &metadata));
  EXPECT_EQ(2u, metadata.version);
  ASSERT_TRUE(contract::messageMetadata(2005, &metadata));
  EXPECT_EQ(2u, metadata.version);
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
