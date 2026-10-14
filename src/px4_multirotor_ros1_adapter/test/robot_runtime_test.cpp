#include <gtest/gtest.h>

#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <vector>

#include "xgc_px4_multirotor_ros1_adapter/generated_contract.hpp"
#include "xgc2_ros1_robot_adapter/ground_health.hpp"
#include "xgc_px4_multirotor_ros1_adapter/robot_runtime.hpp"

namespace xgc_px4_multirotor_ros1_adapter {
namespace {

xgc2_ros1_robot_adapter::RobotConfig makeProfileConfig() {
  xgc2_ros1_robot_adapter::RobotConfig config;
  config.robot_id = "uav1";
  config.profile_id = InstalledContract().primaryProfile().profile_id;
  config.parameters["namespace"] = "/uav1";
  config.parameters["mocap_rigid_body"] = "FS150_01";
  config.parameters["mocap_source_root"] = "/vrpn_client_node_physical";
  config.parameters["localization_offset_x"] = "1.25";
  config.parameters["localization_offset_y"] = "-2";
  config.parameters["localization_offset_z"] = "0.1";
  config.parameters["positioning_frame_number"] = "5";
  config.parameters["positioning_comparison_threshold_m"] = "1e-10";
  return config;
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

TEST(LocalizationError, ComputesEuclideanPositionDistance) {
  geometry_msgs::Point local;
  geometry_msgs::Point mocap;
  local.x = 4.0;
  local.y = 6.0;
  local.z = 15.0;
  mocap.x = 1.0;
  mocap.y = 2.0;
  mocap.z = 3.0;
  EXPECT_DOUBLE_EQ(13.0, positionDistanceMeters(local, mocap));
}

TEST(PositioningLiveness, PreservesXgc1RepeatFrameAndAnyAxisAlgorithm) {
  using xgc2_ros1_robot_adapter::PositioningHealthConfig;
  using xgc2_ros1_robot_adapter::PositioningHealthState;
  using xgc2_ros1_robot_adapter::PositioningHealthWindow;
  const PositioningHealthConfig config{5u, 1.0e-10, 0.1};
  PositioningHealthWindow window(config);

  for (std::size_t index = 0u; index < 6u; ++index)
    window.recordPose(static_cast<double>(index) * 0.01, 1.0, 2.0, 3.0);
  const auto frozen = window.evaluate(0.051);
  EXPECT_EQ(PositioningHealthState::kFrozen, frozen.state);
  EXPECT_EQ(5u, frozen.sample_count);
  EXPECT_DOUBLE_EQ(0.0, frozen.comparison_metric_m);

  // Only Y varies. FS150 passes when any axis exceeds the threshold, matching
  // XAutoDronePhyMocapRosModule's en_single_axis_check=true path in XGC1.
  window.recordPose(0.06, 1.0, 2.0 + 2.0e-10, 3.0);
  const auto active = window.evaluate(0.061);
  EXPECT_EQ(PositioningHealthState::kActive, active.state);
  EXPECT_GT(active.comparison_metric_m, config.comparison_threshold_m);

  EXPECT_EQ(PositioningHealthState::kActive, window.evaluate(0.159).state);
  EXPECT_EQ(PositioningHealthState::kTimedOut,
            window.evaluate(0.16).state);
}

TEST(PositioningLiveness, ValidatesTheXgc1AssetParameterBounds) {
  std::string error;
  EXPECT_TRUE(xgc2_ros1_robot_adapter::validPositioningHealthConfig(
      {5u, 1.0e-10, 0.1}, &error));
  EXPECT_FALSE(xgc2_ros1_robot_adapter::validPositioningHealthConfig(
      {0u, 1.0e-10, 0.1}, &error));
  EXPECT_FALSE(xgc2_ros1_robot_adapter::validPositioningHealthConfig(
      {5u, 0.0, 0.1}, &error));
  EXPECT_FALSE(xgc2_ros1_robot_adapter::validPositioningHealthConfig(
      {5u, 1.0e-10, 0.0}, &error));
}

TEST(InstalledProfile, RawVrpnAccelerationUsesTwistAndCanonicalUsesAccel) {
  for (const char *profile : {"px4-multirotor.physical.vrpn",
                              "px4-multirotor.gazebo.vrpn"}) {
    SCOPED_TRACE(profile);
    contract::ChannelMetadata acceleration{};
    ASSERT_TRUE(InstalledContract().channelMetadata(
        profile, "state.mocap.acceleration", &acceleration));
    const auto *input = contract::channelEndpoint(
        acceleration, contract::EndpointKind::kInput, "acceleration");
    const auto *output = contract::channelEndpoint(
        acceleration, contract::EndpointKind::kOutput, "output");
    ASSERT_NE(nullptr, input);
    ASSERT_NE(nullptr, output);
    EXPECT_STREQ("geometry_msgs/TwistStamped", input->ros_type);
    EXPECT_STREQ("geometry_msgs/AccelStamped", output->ros_type);

    auto config = makeProfileConfig();
    config.profile_id = profile;
    NativeProfileConfig native;
    std::string error;
    EXPECT_TRUE(BuildNativeProfileConfig(config, &native, &error)) << error;
  }
}

TEST(InstalledProfile, BuildsEveryNativeEndpointAndPolicyFromTheDescriptor) {
  NativeProfileConfig native;
  std::string error;
  auto config = makeProfileConfig();
  config.parameters["ros_master_uri"] = "http://10.68.3.250:11311";
  config.parameters["ros_ip"] = "10.68.3.251";
  ASSERT_TRUE(BuildNativeProfileConfig(config, &native, &error))
      << error;

  EXPECT_EQ("/uav1/mavros/local_position/pose", native.pose_endpoint);
  EXPECT_EQ("/vrpn_client_node_physical/FS150_01/pose", native.mocap_endpoint);
  EXPECT_EQ("/uav1/pose", native.canonical_pose_endpoint);
  EXPECT_EQ("/uav1/mavros/vision_pose/pose", native.vision_pose_endpoint);
  EXPECT_EQ("/vrpn_client_node_physical/FS150_01/twist", native.mocap_velocity_endpoint);
  EXPECT_EQ("/uav1/twist", native.canonical_velocity_endpoint);
  EXPECT_EQ("/vrpn_client_node_physical/FS150_01/accel", native.mocap_acceleration_endpoint);
  EXPECT_EQ("/uav1/accel", native.canonical_acceleration_endpoint);
  EXPECT_EQ("/uav1/mavros/cmd/arming", native.arm_service_endpoint);
  EXPECT_EQ("/uav1/mavros/set_mode", native.mode_service_endpoint);
  EXPECT_EQ("/uav1/mavros/cmd/command", native.reboot_service_endpoint);
  EXPECT_EQ("/uav1/custom/statustext", native.controller_status_endpoint);
  EXPECT_DOUBLE_EQ(0.5, native.offboard_source_timeout_seconds);
  EXPECT_DOUBLE_EQ(2.5, native.offboard_minimum_rate_hz);
  EXPECT_DOUBLE_EQ(1.0, native.reboot_state_timeout_seconds);
  EXPECT_DOUBLE_EQ(5.0, native.maximum_operation_timeout_seconds);
  EXPECT_TRUE(native.publish_vision);
  EXPECT_DOUBLE_EQ(30.0, native.vision_publish_rate_hz);
  EXPECT_EQ(
      (std::vector<std::string>{"OFFBOARD", "POSCTL", "ALTCTL", "STABILIZED"}),
      native.allowed_modes);

  contract::ChannelMetadata mocap_pose{};
  ASSERT_TRUE(InstalledContract().channelMetadata(InstalledContract().primaryProfile().profile_id, "state.mocap.pose",
                                        &mocap_pose));
  EXPECT_STREQ("px4.mocap-pose", mocap_pose.processor);
  EXPECT_EQ(2u, mocap_pose.endpoint_count);
  EXPECT_EQ(1u, mocap_pose.policy_count);

  contract::ChannelMetadata vision_pose{};
  ASSERT_TRUE(InstalledContract().channelMetadata(InstalledContract().primaryProfile().profile_id, "state.vision.pose",
                                        &vision_pose));
  EXPECT_STREQ("px4.vision-pose", vision_pose.processor);
  EXPECT_EQ(1u, vision_pose.endpoint_count);
  EXPECT_EQ(1u, vision_pose.policy_count);
  std::int64_t vision_publish_rate = 0;
  ASSERT_TRUE(contract::channelPolicyInteger(vision_pose, "publish_rate_hz",
                                            &vision_publish_rate));
  EXPECT_EQ(30, vision_publish_rate);
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
      InstalledContract().channelMetadata(InstalledContract().primaryProfile().profile_id, "state.pose", &pose));
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

TEST(InstalledProfile, VisionForwardingIsOptionalAndRequiresAnExplicitBoolean) {
  auto config = makeProfileConfig();
  NativeProfileConfig native;
  std::string error;
  ASSERT_TRUE(BuildNativeProfileConfig(config, &native, &error)) << error;
  EXPECT_TRUE(native.publish_vision);

  config.parameters["publish_vision"] = "false";
  ASSERT_TRUE(BuildNativeProfileConfig(config, &native, &error)) << error;
  EXPECT_FALSE(native.publish_vision);
  EXPECT_EQ("/uav1/mavros/local_position/pose", native.pose_endpoint);
  EXPECT_EQ("/uav1/pose", native.canonical_pose_endpoint);
  EXPECT_EQ("/vrpn_client_node_physical/FS150_01/pose", native.mocap_endpoint);

  config.parameters["publish_vision"] = "true";
  ASSERT_TRUE(BuildNativeProfileConfig(config, &native, &error)) << error;
  EXPECT_TRUE(native.publish_vision);

  config.parameters["publish_vision"] = "0";
  EXPECT_FALSE(BuildNativeProfileConfig(config, &native, &error));
}

TEST(InstalledProfile, KeepsRobotMetadataOutOfTheRuntimeProtocol) {
  const char *digest = InstalledContract().profileDigest("px4-multirotor.physical.vrpn");
  ASSERT_NE(nullptr, digest);
  EXPECT_EQ(64u, std::string(digest).size());

  contract::ChannelMetadata pose;
  ASSERT_TRUE(
      InstalledContract().channelMetadata("px4-multirotor.physical.vrpn", "state.pose", &pose));
  EXPECT_EQ(contract::ChannelKind::kStreamOut, pose.kind);
  EXPECT_EQ(2001u, pose.output_message_id);

  contract::ChannelMetadata controller;
  ASSERT_TRUE(InstalledContract().channelMetadata("px4-multirotor.physical.vrpn",
                                        "state.controller", &controller));
  EXPECT_EQ(2012u, controller.output_message_id);
  EXPECT_DOUBLE_EQ(5.0, controller.output_rate_hz);
  EXPECT_EQ("px4.controller-status", std::string(controller.processor));

  contract::ChannelMetadata arm;
  ASSERT_TRUE(InstalledContract().channelMetadata("px4-multirotor.physical.vrpn",
                                        "operation.arm", &arm));
  EXPECT_EQ(contract::ChannelKind::kOperation, arm.kind);
  EXPECT_EQ(3201u, arm.input_message_id);

  contract::OperationMetadata mode;
  ASSERT_TRUE(InstalledContract().operationMetadata(
      "px4-multirotor.physical.vrpn", "set-flight-mode", &mode));
  EXPECT_EQ(5000u, mode.timeout_millis);
  const std::string parameter_schema(mode.parameter_schema_json);
  EXPECT_NE(std::string::npos, parameter_schema.find("additionalProperties"));
  EXPECT_NE(std::string::npos, parameter_schema.find("OFFBOARD"));
}

TEST(InstalledProfile, ContainsExactSemanticMessageMetadata) {
  contract::MessageMetadata metadata;
  EXPECT_TRUE(InstalledContract().messageMetadata(3201, &metadata));
  EXPECT_TRUE(InstalledContract().messageMetadata(3202, &metadata));
  EXPECT_TRUE(InstalledContract().messageMetadata(3203, &metadata));
  EXPECT_TRUE(InstalledContract().messageMetadata(3206, &metadata));
  EXPECT_FALSE(InstalledContract().messageMetadata(5001, &metadata));
  EXPECT_TRUE(InstalledContract().messageMetadata(3002, &metadata));
  EXPECT_TRUE(InstalledContract().messageMetadata(3003, &metadata));
  EXPECT_TRUE(InstalledContract().messageMetadata(3004, &metadata));
  EXPECT_TRUE(InstalledContract().messageMetadata(3005, &metadata));
  EXPECT_TRUE(InstalledContract().messageMetadata(2006, &metadata));
  EXPECT_TRUE(InstalledContract().messageMetadata(2007, &metadata));
}

TEST(InstalledContract, PinsTheRegistryItWasGeneratedFrom) {
  EXPECT_EQ(6362961126724301134ULL, InstalledContract().registryFingerprint());
  EXPECT_EQ(std::string("xgc2-px4-multirotor-ros1-adapter"),
            InstalledContract().providerDefinitionId());
}

} // namespace
} // namespace xgc_px4_multirotor_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
