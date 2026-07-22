#include <gtest/gtest.h>

#include <deque>
#include <string>
#include <vector>

#include "xgc_scout_mini_ros1_adapter/generated_contract.hpp"
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
  EXPECT_EQ("/scout1/odom", topicName("/scout1", "odom"));
  EXPECT_EQ("/fleet/scout2/scout_status",
            topicName("/fleet/scout2", "/scout_status"));
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

TEST(Freshness, AppliesTheScoutStatusBoundary) {
  const ros::WallTime now(10, 0);
  contract::ChannelMetadata health{};
  ASSERT_TRUE(
      contract::channelMetadata(contract::kProfileId, "state.health", &health));
  const double stale_after_seconds =
      static_cast<double>(health.stale_after_millis) / 1000.0;
  EXPECT_FALSE(sourceIsFresh(ros::WallTime(), now, stale_after_seconds));
  EXPECT_TRUE(sourceIsFresh(ros::WallTime(9, 0), now, stale_after_seconds));
  EXPECT_FALSE(
      sourceIsFresh(ros::WallTime(8, 999999999), now, stale_after_seconds));
}

TEST(OnlineProjection, FollowsTheDeclaredChassisStatusInput) {
  EXPECT_TRUE(scoutIsOnline(true));
  EXPECT_FALSE(scoutIsOnline(false));
}

TEST(BatteryProjection, UsesTheManualLinearVoltageModel) {
  EXPECT_DOUBLE_EQ(0.0, scoutBatteryPercentage(20.5));
  EXPECT_DOUBLE_EQ(1.0, scoutBatteryPercentage(29.2));
  EXPECT_NEAR(0.95, scoutBatteryPercentage(28.765), 1e-12);
  EXPECT_DOUBLE_EQ(0.0, scoutBatteryPercentage(18.0));
  EXPECT_DOUBLE_EQ(1.0, scoutBatteryPercentage(32.0));
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
  const char *digest = contract::profileDigest("scout-mini.ros1.v4");
  ASSERT_NE(nullptr, digest);
  EXPECT_EQ(64u, std::string(digest).size());

  contract::ChannelMetadata pose;
  ASSERT_TRUE(
      contract::channelMetadata("scout-mini.ros1.v4", "state.pose", &pose));
  EXPECT_EQ(contract::ChannelKind::kStreamOut, pose.kind);
  EXPECT_EQ(2001u, pose.output_message_id);

  contract::ChannelMetadata unknown;
  EXPECT_FALSE(contract::channelMetadata("scout-mini.ros1.v4", "operation.arm",
                                         &unknown));

  std::size_t operation_count = 1u;
  EXPECT_EQ(nullptr, contract::profileOperations("scout-mini.ros1.v4",
                                                 &operation_count));
  EXPECT_EQ(0u, operation_count);
}

TEST(InstalledContract, DoesNotContainPx4OperationMetadata) {
  contract::MessageMetadata metadata;
  EXPECT_TRUE(contract::messageMetadata(2001, &metadata));
  EXPECT_TRUE(contract::messageMetadata(3102, &metadata));
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

} // namespace
} // namespace xgc_scout_mini_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
