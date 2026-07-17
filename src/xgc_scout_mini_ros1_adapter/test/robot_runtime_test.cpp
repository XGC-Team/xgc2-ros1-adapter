#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "xgc_scout_mini_ros1_adapter/generated_contract.hpp"
#include "xgc_scout_mini_ros1_adapter/robot_runtime.hpp"
#include "xgc_scout_mini_ros1_adapter/shutdown_signal.hpp"

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
  ASSERT_TRUE(contract::channelMetadata(contract::kProfileId, "state.health",
                                        &health));
  const double stale_after_seconds =
      static_cast<double>(health.stale_after_millis) / 1000.0;
  EXPECT_FALSE(sourceIsFresh(ros::WallTime(), now, stale_after_seconds));
  EXPECT_TRUE(sourceIsFresh(ros::WallTime(9, 0), now, stale_after_seconds));
  EXPECT_FALSE(sourceIsFresh(ros::WallTime(8, 999999999), now,
                             stale_after_seconds));
}

TEST(OnlineProjection, FollowsTheDeclaredChassisStatusInput) {
  EXPECT_TRUE(scoutIsOnline(true));
  EXPECT_FALSE(scoutIsOnline(false));
}

TEST(InstalledProfile, KeepsRobotMetadataOutOfTheRuntimeProtocol) {
  std::string error;
  EXPECT_TRUE(validateNativeProfileContract(&error)) << error;
  const char *digest = contract::profileDigest("scout-mini.ros1.v1");
  ASSERT_NE(nullptr, digest);
  EXPECT_EQ(64u, std::string(digest).size());

  contract::ChannelMetadata pose;
  ASSERT_TRUE(contract::channelMetadata("scout-mini.ros1.v1", "state.pose",
                                        &pose));
  EXPECT_EQ(contract::ChannelKind::kStreamOut, pose.kind);
  EXPECT_EQ(2001u, pose.output_message_id);

  contract::ChannelMetadata unknown;
  EXPECT_FALSE(contract::channelMetadata("scout-mini.ros1.v1",
                                         "operation.arm", &unknown));
}

TEST(InstalledContract, DoesNotContainPx4OperationMetadata) {
  contract::MessageMetadata metadata;
  EXPECT_TRUE(contract::messageMetadata(2001, &metadata));
  EXPECT_FALSE(contract::messageMetadata(3201, &metadata));
}

} // namespace
} // namespace xgc_scout_mini_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
