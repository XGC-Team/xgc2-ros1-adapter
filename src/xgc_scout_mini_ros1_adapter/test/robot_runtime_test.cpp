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
  EXPECT_FALSE(
      sourceIsFresh(ros::WallTime(), now, kScoutStatusStaleAfterSeconds));
  EXPECT_TRUE(
      sourceIsFresh(ros::WallTime(9, 0), now, kScoutStatusStaleAfterSeconds));
  EXPECT_FALSE(sourceIsFresh(ros::WallTime(8, 999999999), now,
                             kScoutStatusStaleAfterSeconds));
}

TEST(OnlineProjection, RequiresFreshOdometryAndChassisStatus) {
  EXPECT_TRUE(scoutIsOnline(true, true));
  EXPECT_FALSE(scoutIsOnline(false, true));
  EXPECT_FALSE(scoutIsOnline(true, false));
}

TEST(AdapterPlanSafety, AcceptsMultipleScoutRobotsButRejectsEmptyPlans) {
  xgc::adapter::v1::AdapterPlan plan;
  std::string error;
  EXPECT_FALSE(validateNonEmptyAdapterPlan(plan, &error));

  plan.add_robots()->set_robot_id("scout-01");
  plan.add_robots()->set_robot_id("scout-02");
  error.clear();
  EXPECT_TRUE(validateNonEmptyAdapterPlan(plan, &error));
  EXPECT_EQ(2, plan.robots_size());
}

TEST(InstalledContract, AdvertisesOnlyTheScoutMiniProfile) {
  std::vector<xgc::adapter::v1::ProfileAdvertisement> profiles;
  contract::addSupportedProfiles(&profiles);
  ASSERT_EQ(1u, profiles.size());
  EXPECT_EQ("scout-mini.ros1.v1", profiles.front().profile_id());
  EXPECT_EQ(64u, profiles.front().profile_digest().size());
  for (const auto &channel : profiles.front().channels()) {
    EXPECT_EQ(xgc::adapter::v1::CHANNEL_KIND_STREAM_OUT, channel.kind());
  }
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
