#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "xgc_px4_multirotor_ros1_adapter/generated_contract.hpp"
#include "xgc_px4_multirotor_ros1_adapter/robot_runtime.hpp"
#include "xgc_px4_multirotor_ros1_adapter/shutdown_signal.hpp"

namespace xgc_px4_multirotor_ros1_adapter {
namespace {

xgc2_ros1_robot_adapter::RobotConfig makeProfileConfig() {
  xgc2_ros1_robot_adapter::RobotConfig config;
  config.robot_id = "uav1";
  config.profile_id = contract::kProfileId;
  config.parameters[contract::kNamespaceParameter] = "/uav1";
  config.parameters["mocap_rigid_body"] = "FS150_01";
  return config;
}

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

TEST(RosNames, AcceptsOnlyAssetSafeMocapRigidBodyNames) {
  std::string error;
  EXPECT_TRUE(validMocapRigidBodyName("fs150_01", &error));
  EXPECT_TRUE(validMocapRigidBodyName("FS150_01", &error));
  EXPECT_FALSE(validMocapRigidBodyName("FS150-01", &error));
  EXPECT_FALSE(validMocapRigidBodyName("/vrpn/arbitrary/topic", &error));
  EXPECT_FALSE(validMocapRigidBodyName("body name", &error));
}

TEST(VisionRelay, RejectsNonFiniteOrDegeneratePoses) {
  geometry_msgs::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  EXPECT_TRUE(validVisionPose(pose));

  pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validVisionPose(pose));
  pose.pose.position.x = 0.0;
  pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(validVisionPose(pose));

  pose.pose.orientation.x = std::numeric_limits<double>::max();
  pose.pose.orientation.y = std::numeric_limits<double>::max();
  EXPECT_FALSE(validVisionPose(pose));

  pose.pose.orientation.x = 3.0;
  pose.pose.orientation.y = 0.0;
  ASSERT_TRUE(normalizeVisionPose(&pose));
  EXPECT_DOUBLE_EQ(1.0, pose.pose.orientation.x);
  EXPECT_DOUBLE_EQ(0.0, pose.pose.orientation.w);
}

TEST(InstalledProfile, BuildsEveryNativeEndpointAndPolicyFromTheDescriptor) {
  NativeProfileConfig native;
  std::string error;
  ASSERT_TRUE(BuildNativeProfileConfig(makeProfileConfig(), &native, &error))
      << error;

  EXPECT_EQ("/uav1/mavros/local_position/pose", native.pose_endpoint);
  EXPECT_EQ("/vrpn_client_node/FS150_01/pose", native.mocap_endpoint);
  EXPECT_EQ("/uav1/mavros/vision_pose/pose", native.vision_pose_endpoint);
  EXPECT_EQ("/uav1/mavros/cmd/arming", native.arm_service_endpoint);
  EXPECT_EQ("/uav1/mavros/set_mode", native.mode_service_endpoint);
  EXPECT_EQ("/uav1/mavros/cmd/command", native.reboot_service_endpoint);
  EXPECT_DOUBLE_EQ(0.02, native.vision_minimum_period_seconds);
  EXPECT_DOUBLE_EQ(0.2, native.mocap_timeout_seconds);
  EXPECT_DOUBLE_EQ(0.5, native.offboard_source_timeout_seconds);
  EXPECT_DOUBLE_EQ(2.5, native.offboard_minimum_rate_hz);
  EXPECT_DOUBLE_EQ(1.0, native.reboot_state_timeout_seconds);
  EXPECT_DOUBLE_EQ(5.0, native.maximum_operation_timeout_seconds);
  EXPECT_EQ(
      (std::vector<std::string>{"OFFBOARD", "POSCTL", "ALTCTL", "STABILIZED"}),
      native.allowed_modes);
}

TEST(SetpointDiagnostics, HonorsEveryMavrosTypeMaskBit) {
  EXPECT_EQ(0x7ffu, localSetpointValidFields(0));
  EXPECT_EQ(0x7feu,
            localSetpointValidFields(mavros_msgs::PositionTarget::IGNORE_PX));

  EXPECT_EQ(0x1fu, attitudeSetpointValidFields(0));
  EXPECT_EQ(0x0fu, attitudeSetpointValidFields(
                       mavros_msgs::AttitudeTarget::IGNORE_THRUST));
  const std::uint8_t ignore_all =
      mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
      mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
      mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE |
      mavros_msgs::AttitudeTarget::IGNORE_THRUST |
      mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
  EXPECT_EQ(0u, attitudeSetpointValidFields(ignore_all));
}

TEST(Freshness, AppliesThePx4PoseBoundary) {
  contract::ChannelMetadata pose{};
  ASSERT_TRUE(
      contract::channelMetadata(contract::kProfileId, "state.pose", &pose));
  const double stale_after_seconds =
      static_cast<double>(pose.stale_after_millis) / 1000.0;
  const ros::WallTime now(10, 0);
  EXPECT_FALSE(sourceIsFresh(ros::WallTime(), now, stale_after_seconds));
  EXPECT_FALSE(sourceIsFresh(ros::WallTime(11, 0), now, stale_after_seconds));
  EXPECT_TRUE(sourceIsFresh(ros::WallTime(9, 0), now, stale_after_seconds));
  EXPECT_FALSE(
      sourceIsFresh(ros::WallTime(8, 999999999), now, stale_after_seconds));
}

TEST(OnlineProjection, RequiresFreshConnectedMavrosState) {
  EXPECT_TRUE(px4IsOnline(true, true, true));
  EXPECT_FALSE(px4IsOnline(false, true, true));
  EXPECT_FALSE(px4IsOnline(true, false, true));
  EXPECT_FALSE(px4IsOnline(true, true, false));
}

TEST(InstalledProfile, KeepsRobotMetadataOutOfTheRuntimeProtocol) {
  const char *digest = contract::profileDigest("px4.multirotor.ros1.v3");
  ASSERT_NE(nullptr, digest);
  EXPECT_EQ(64u, std::string(digest).size());

  contract::ChannelMetadata pose;
  ASSERT_TRUE(
      contract::channelMetadata("px4.multirotor.ros1.v3", "state.pose", &pose));
  EXPECT_EQ(contract::ChannelKind::kStreamOut, pose.kind);
  EXPECT_EQ(2001u, pose.output_message_id);

  contract::ChannelMetadata arm;
  ASSERT_TRUE(contract::channelMetadata("px4.multirotor.ros1.v3",
                                        "operation.arm", &arm));
  EXPECT_EQ(contract::ChannelKind::kOperation, arm.kind);
  EXPECT_EQ(3201u, arm.input_message_id);
}

TEST(InstalledProfile, ContainsExactSemanticMessageMetadata) {
  contract::MessageMetadata metadata;
  EXPECT_TRUE(contract::messageMetadata(3201, &metadata));
  EXPECT_TRUE(contract::messageMetadata(3202, &metadata));
  EXPECT_TRUE(contract::messageMetadata(3203, &metadata));
  EXPECT_FALSE(contract::messageMetadata(5001, &metadata));
  EXPECT_TRUE(contract::messageMetadata(3002, &metadata));
  EXPECT_TRUE(contract::messageMetadata(3003, &metadata));
  EXPECT_TRUE(contract::messageMetadata(3004, &metadata));
  EXPECT_TRUE(contract::messageMetadata(3005, &metadata));
}

} // namespace
} // namespace xgc_px4_multirotor_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
