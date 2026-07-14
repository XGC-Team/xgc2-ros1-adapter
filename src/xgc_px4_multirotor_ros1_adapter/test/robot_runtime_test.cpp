#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "xgc_px4_multirotor_ros1_adapter/generated_contract.hpp"
#include "xgc_px4_multirotor_ros1_adapter/robot_runtime.hpp"
#include "xgc_px4_multirotor_ros1_adapter/shutdown_signal.hpp"

namespace xgc_px4_multirotor_ros1_adapter {
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

TEST(RosNames, BuildsNamespacedMavrosTopics) {
  EXPECT_EQ("/uav1/mavros/state", topicName("/uav1", "mavros/state"));
  EXPECT_EQ("/fleet/uav2/mavros/local_position/pose",
            topicName("/fleet/uav2", "/mavros/local_position/pose"));
}

TEST(RosNames, AcceptsOnlyCanonicalAbsoluteRobotNamespaces) {
  std::string error;
  EXPECT_TRUE(validRobotNamespace("/uav1", &error));

  error.clear();
  EXPECT_FALSE(validRobotNamespace("uav1", &error));
  EXPECT_EQ("namespace must be absolute", error);

  error.clear();
  EXPECT_FALSE(validRobotNamespace("/uav1/", &error));
  EXPECT_EQ("namespace must not have a trailing slash", error);

  error.clear();
  EXPECT_FALSE(validRobotNamespace("/fleet//uav1", &error));
  EXPECT_EQ("namespace must not contain repeated slashes", error);

  error.clear();
  EXPECT_FALSE(validRobotNamespace("/", &error));
  EXPECT_FALSE(error.empty());
}

TEST(Freshness, AppliesThePx4PoseBoundary) {
  const ros::WallTime now(10, 0);
  EXPECT_FALSE(sourceIsFresh(ros::WallTime(), now, kPx4PoseStaleAfterSeconds));
  EXPECT_FALSE(
      sourceIsFresh(ros::WallTime(11, 0), now, kPx4PoseStaleAfterSeconds));
  EXPECT_TRUE(
      sourceIsFresh(ros::WallTime(9, 0), now, kPx4PoseStaleAfterSeconds));
  EXPECT_FALSE(sourceIsFresh(ros::WallTime(8, 999999999), now,
                             kPx4PoseStaleAfterSeconds));
}

TEST(OnlineProjection, RequiresFreshConnectedMavrosState) {
  EXPECT_TRUE(px4IsOnline(true, true, true));
  EXPECT_FALSE(px4IsOnline(false, true, true));
  EXPECT_FALSE(px4IsOnline(true, false, true));
  EXPECT_FALSE(px4IsOnline(true, true, false));
}

TEST(Px4ModeSafety, AllowsOnlyThePublishedModes) {
  EXPECT_TRUE(isAllowedPx4Mode("OFFBOARD"));
  EXPECT_TRUE(isAllowedPx4Mode("POSCTL"));
  EXPECT_TRUE(isAllowedPx4Mode("ALTCTL"));
  EXPECT_TRUE(isAllowedPx4Mode("STABILIZED"));
  EXPECT_FALSE(isAllowedPx4Mode("AUTO.LAND"));
  EXPECT_FALSE(isAllowedPx4Mode("offboard"));
}

TEST(Px4RebootSafety, RequiresFreshConnectedKnownDisarmedState) {
  EXPECT_EQ(Px4RebootReadiness::kStateUnknown,
            evaluatePx4RebootReadiness(false, false, false, false));
  EXPECT_EQ(Px4RebootReadiness::kStateStale,
            evaluatePx4RebootReadiness(true, false, true, false));
  EXPECT_EQ(Px4RebootReadiness::kDisconnected,
            evaluatePx4RebootReadiness(true, true, false, false));
  EXPECT_EQ(Px4RebootReadiness::kArmed,
            evaluatePx4RebootReadiness(true, true, true, true));
  EXPECT_EQ(Px4RebootReadiness::kReady,
            evaluatePx4RebootReadiness(true, true, true, false));
}

TEST(AdapterPlanSafety, AcceptsMultiplePx4RobotsButRejectsEmptyPlans) {
  xgc::adapter::v1::AdapterPlan plan;
  std::string error;
  EXPECT_FALSE(validateNonEmptyAdapterPlan(plan, &error));

  plan.add_robots()->set_robot_id("px4-01");
  plan.add_robots()->set_robot_id("px4-02");
  error.clear();
  EXPECT_TRUE(validateNonEmptyAdapterPlan(plan, &error));
  EXPECT_EQ(2, plan.robots_size());
}

TEST(InstalledContract, AdvertisesOnlyThePx4Profile) {
  std::vector<xgc::adapter::v1::ProfileAdvertisement> profiles;
  contract::addSupportedProfiles(&profiles);
  ASSERT_EQ(1u, profiles.size());
  EXPECT_EQ("px4.multirotor.ros1.v1", profiles.front().profile_id());
  EXPECT_EQ(64u, profiles.front().profile_digest().size());
  for (const auto &channel : profiles.front().channels()) {
    EXPECT_NE(xgc::adapter::v1::CHANNEL_KIND_STREAM_IN, channel.kind());
    EXPECT_NE(xgc::adapter::v1::CHANNEL_KIND_REQUEST_RESPONSE, channel.kind());
  }
}

TEST(InstalledContract, ContainsPx4OperationMetadataOnly) {
  contract::MessageMetadata metadata;
  EXPECT_TRUE(contract::messageMetadata(3201, &metadata));
  EXPECT_EQ(1u, metadata.version);
  EXPECT_NE(0u, metadata.fingerprint);
  EXPECT_FALSE(contract::messageMetadata(5001, &metadata));
}

} // namespace
} // namespace xgc_px4_multirotor_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
