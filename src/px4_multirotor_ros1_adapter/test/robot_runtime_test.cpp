#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
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

TEST(LocalizationError, DelayComesFromProfileAndAcceptsExplicitOverride) {
  auto config = makeProfileConfig();
  NativeProfileConfig native;
  std::string error;
  ASSERT_TRUE(BuildNativeProfileConfig(config, &native, &error)) << error;
  EXPECT_DOUBLE_EQ(0.05, native.localization_delay_seconds);
  config.parameters["localization_delay_ms"] = "25";
  ASSERT_TRUE(BuildNativeProfileConfig(config, &native, &error)) << error;
  EXPECT_DOUBLE_EQ(0.025, native.localization_delay_seconds);
  for (const std::string invalid : {"-1", "nan", "inf", "1001", "50ms"}) {
    config.parameters["localization_delay_ms"] = invalid;
    EXPECT_FALSE(BuildNativeProfileConfig(config, &native, &error)) << invalid;
  }
}

TEST(LocalizationProjection, AssetExtrinsicIsOnlyAcceptedByPhysicalProfile) {
  auto config = makeProfileConfig();
  config.profile_id = "px4-multirotor.physical.vrpn";
  config.parameters["mocap_marker_from_body"] =
      "{\"translation\":[0.01,0.02,-0.03],\"rotation\":[0,0,0,1]}";
  NativeProfileConfig native;
  std::string error;
  ASSERT_TRUE(BuildNativeProfileConfig(config, &native, &error)) << error;
  EXPECT_DOUBLE_EQ(0.05, native.localization_delay_seconds);
  for (const std::string profile : {"px4-multirotor.xsim.ros", "px4-multirotor.gazebo.vrpn"}) {
    config.profile_id = profile;
    EXPECT_FALSE(BuildNativeProfileConfig(config, &native, &error)) << profile;
  }
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

  // Unnormalized quaternions beyond 5% (norm = 3.0, 0.94, 1.06, 1e-6) fail.
  pose.pose.orientation.x = 3.0;
  pose.pose.orientation.y = 0.0;
  EXPECT_FALSE(validVisionPose(pose));
  EXPECT_FALSE(normalizeVisionPose(&pose));

  xgc2_ros1_robot_adapter::LocalizationProjectionConfig no_extrinsic;
  geometry_msgs::PoseStamped near_unit_pose;
  near_unit_pose.pose.orientation.w = 1.04;
  geometry_msgs::PoseStamped projected_near_unit;
  EXPECT_TRUE(xgc2_ros1_robot_adapter::projectLocalizationPose(
      near_unit_pose, no_extrinsic, &projected_near_unit));
  EXPECT_TRUE(validVisionPose(projected_near_unit));
  ASSERT_TRUE(normalizeVisionPose(&projected_near_unit));
  EXPECT_DOUBLE_EQ(1.0, projected_near_unit.pose.orientation.w);

  geometry_msgs::PoseStamped small_norm_pose;
  small_norm_pose.pose.orientation.w = 1.0e-6;
  EXPECT_FALSE(xgc2_ros1_robot_adapter::projectLocalizationPose(
      small_norm_pose, no_extrinsic, &projected_near_unit));
  EXPECT_FALSE(validVisionPose(small_norm_pose));

  geometry_msgs::PoseStamped outside_tolerance;
  outside_tolerance.pose.orientation.w = 0.94;
  EXPECT_FALSE(xgc2_ros1_robot_adapter::projectLocalizationPose(
      outside_tolerance, no_extrinsic, &projected_near_unit));
  EXPECT_FALSE(validVisionPose(outside_tolerance));
  outside_tolerance.pose.orientation.w = 1.06;
  EXPECT_FALSE(xgc2_ros1_robot_adapter::projectLocalizationPose(
      outside_tolerance, no_extrinsic, &projected_near_unit));
  EXPECT_FALSE(validVisionPose(outside_tolerance));
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

TEST(PositioningLiveness, ReplaysHeldPoseAdvancingStampAndRecoveryJumpWithoutZeroFallback) {
  using xgc2_ros1_robot_adapter::LocalizationProjectionConfig;
  using xgc2_ros1_robot_adapter::PositioningHealthConfig;
  using xgc2_ros1_robot_adapter::PositioningHealthReason;
  using xgc2_ros1_robot_adapter::PositioningHealthState;
  using xgc2_ros1_robot_adapter::PositioningHealthWindow;
  using xgc2_ros1_robot_adapter::projectLocalizationPose;

  const PositioningHealthConfig health_config{5u, 1.0e-10, 0.1};
  LocalizationProjectionConfig projection;
  projection.has_extrinsic = true;
  projection.marker_from_body_translation = {{0.08, -0.03, 0.04}};
  projection.marker_from_body_rotation = {{0.0, 0.0, 0.0, 1.0}};
  projection.offset_x = 1.25;
  projection.offset_y = -2.0;
  projection.offset_z = 0.1;

  auto make_pose = [](double stamp_seconds, double x, double y, double z,
                      double qx, double qy, double qz, double qw) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp = ros::Time(stamp_seconds);
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;
    return pose;
  };

  const double base_qz = std::sin(0.2);
  const double base_qw = std::cos(0.2);
  const double expected_x =
      2.0 + 0.08 * std::cos(0.4) - (-0.03) * std::sin(0.4) + 1.25;
  const double expected_y =
      1.0 + 0.08 * std::sin(0.4) + (-0.03) * std::cos(0.4) - 2.0;
  const double expected_z = 1.5 + 0.04 + 0.1;

  // 1. Held P+Q (unchanged stamp & pose) freezes within 100 ms and times out
  //    beyond 100 ms without ever fabricating a zero/origin pose.
  PositioningHealthWindow held_window(health_config);
  for (std::size_t index = 0u; index < 6u; ++index) {
    const auto raw =
        make_pose(100.0, 2.0, 1.0, 1.5, 0.0, 0.0, base_qz, base_qw);
    geometry_msgs::PoseStamped projected;
    ASSERT_TRUE(projectLocalizationPose(raw, projection, &projected));
    EXPECT_NEAR(expected_x, projected.pose.position.x, 1e-12);
    EXPECT_NEAR(expected_y, projected.pose.position.y, 1e-12);
    EXPECT_DOUBLE_EQ(expected_z, projected.pose.position.z);
    held_window.recordPose(10.0 + static_cast<double>(index) * 0.033,
                           projected.pose.position.x,
                           projected.pose.position.y,
                           projected.pose.position.z);
  }
  const auto held_frozen = held_window.evaluate(10.166);
  EXPECT_EQ(PositioningHealthState::kFrozen, held_frozen.state);
  EXPECT_EQ(PositioningHealthReason::kRepeatFrameWindowFrozen, held_frozen.reason);
  EXPECT_DOUBLE_EQ(0.0, held_frozen.comparison_metric_m);
  EXPECT_EQ(5u, held_frozen.sample_count);

  const auto held_timed_out = held_window.evaluate(10.165 + 0.105);
  EXPECT_EQ(PositioningHealthState::kTimedOut, held_timed_out.state);
  EXPECT_EQ(PositioningHealthReason::kVrpnTimeout, held_timed_out.reason);

  // 2. Advancing stamp (30 Hz) with identical P+Q still detects repeat-frame
  //    freeze while PositionErrorWindow measures divergence against moving EKF2.
  PositioningHealthWindow window(health_config);
  PositionErrorWindow error_window(0.05);
  geometry_msgs::PoseStamped last_projected;
  for (std::size_t index = 0u; index < 6u; ++index) {
    const double dt = static_cast<double>(index) / 30.0;
    const auto local = make_pose(200.0 + dt - 0.05, expected_x + 0.5 * dt,
                                 expected_y, expected_z, 0.0, 0.0, base_qz,
                                 base_qw);
    error_window.local(local);

    const auto raw =
        make_pose(200.0 + dt, 2.0, 1.0, 1.5, 0.0, 0.0, base_qz, base_qw);
    ASSERT_TRUE(projectLocalizationPose(raw, projection, &last_projected));
    EXPECT_NEAR(200.0 + dt, last_projected.header.stamp.toSec(), 1e-9);
    EXPECT_NEAR(expected_x, last_projected.pose.position.x, 1e-12);
    EXPECT_TRUE(error_window.mocap(last_projected));
    window.recordPose(20.0 + dt, last_projected.pose.position.x,
                      last_projected.pose.position.y,
                      last_projected.pose.position.z);
  }
  const auto frozen_advancing = window.evaluate(20.0 + 5.0 / 30.0 + 0.001);
  EXPECT_EQ(PositioningHealthState::kFrozen, frozen_advancing.state);
  EXPECT_EQ(PositioningHealthReason::kRepeatFrameWindowFrozen,
            frozen_advancing.reason);
  EXPECT_DOUBLE_EQ(0.0, frozen_advancing.comparison_metric_m);
  ASSERT_TRUE(error_window.latest().has_value());
  EXPECT_NEAR(0.5 * (5.0 / 30.0), error_window.latest()->meters, 1e-8);

  // 3. Onsite 4.443 m recovery jump flips Frozen -> Active immediately with
  //    exact jump magnitude, updates PositionErrorWindow without clamping, then
  //    re-freezes once the 5-frame ring turns over with stationary samples.
  const double jump_dt = 6.0 / 30.0;
  error_window.local(make_pose(200.0 + jump_dt - 0.05, expected_x, expected_y,
                               expected_z, 0.0, 0.0, base_qz, base_qw));
  const auto raw_jump = make_pose(200.0 + jump_dt, 2.0 + 4.443, 1.0, 1.5, 0.0,
                                  0.0, base_qz, base_qw);
  geometry_msgs::PoseStamped projected_jump;
  ASSERT_TRUE(projectLocalizationPose(raw_jump, projection, &projected_jump));
  EXPECT_NEAR(4.443, projected_jump.pose.position.x - last_projected.pose.position.x,
              1e-12);
  EXPECT_NEAR(expected_y, projected_jump.pose.position.y, 1e-12);
  EXPECT_DOUBLE_EQ(expected_z, projected_jump.pose.position.z);
  EXPECT_NEAR(base_qz, projected_jump.pose.orientation.z, 1e-12);
  EXPECT_NEAR(base_qw, projected_jump.pose.orientation.w, 1e-12);

  ASSERT_TRUE(error_window.mocap(projected_jump));
  ASSERT_TRUE(error_window.latest().has_value());
  EXPECT_NEAR(4.443, error_window.latest()->meters, 1e-8);

  window.recordPose(20.0 + jump_dt, projected_jump.pose.position.x,
                    projected_jump.pose.position.y,
                    projected_jump.pose.position.z);
  const auto active_jump = window.evaluate(20.0 + jump_dt + 0.001);
  EXPECT_EQ(PositioningHealthState::kActive, active_jump.state);
  EXPECT_EQ(PositioningHealthReason::kWindowVariationObserved, active_jump.reason);
  EXPECT_NEAR(std::sqrt(5.0) * 4.443, active_jump.comparison_metric_m, 1e-12);

  for (std::size_t extra = 1u; extra <= 5u; ++extra) {
    window.recordPose(20.0 + (6.0 + static_cast<double>(extra)) / 30.0,
                      projected_jump.pose.position.x,
                      projected_jump.pose.position.y,
                      projected_jump.pose.position.z);
  }
  const auto settled_frozen =
      window.evaluate(20.0 + 11.0 / 30.0 + 0.001);
  EXPECT_EQ(PositioningHealthState::kFrozen, settled_frozen.state);
  EXPECT_DOUBLE_EQ(0.0, settled_frozen.comparison_metric_m);
}

TEST(PositioningLiveness, AttitudeFlipPreservesSmallPositionErrorWithoutValidatingAttitude) {
  using xgc2_ros1_robot_adapter::LocalizationProjectionConfig;
  using xgc2_ros1_robot_adapter::PositioningHealthConfig;
  using xgc2_ros1_robot_adapter::PositioningHealthState;
  using xgc2_ros1_robot_adapter::PositioningHealthWindow;
  using xgc2_ros1_robot_adapter::projectLocalizationPose;

  constexpr double kPi = 3.14159265358979323846;
  LocalizationProjectionConfig projection;
  projection.offset_x = 1.0;
  projection.offset_y = -0.5;
  projection.offset_z = 0.25;

  PositioningHealthWindow xyz_window(PositioningHealthConfig{5u, 1.0e-10, 0.1});
  PositionErrorWindow error_window(0.05);

  auto feed_pair = [&](std::size_t idx, double raw_x, double angle_deg,
                       geometry_msgs::PoseStamped *projected_out) {
    const double t = static_cast<double>(idx) * 0.033;
    geometry_msgs::PoseStamped local;
    local.header.frame_id = "world";
    local.header.stamp = ros::Time(300.0 + t - 0.05);
    local.pose.position.x = 1.5;
    local.pose.position.y = -0.3;
    local.pose.position.z = 1.35;
    local.pose.orientation.w = 1.0;
    error_window.local(local);

    const double half = (angle_deg * kPi / 180.0) * 0.5;
    geometry_msgs::PoseStamped raw;
    raw.header.frame_id = "world";
    raw.header.stamp = ros::Time(300.0 + t);
    raw.pose.position.x = raw_x;
    raw.pose.position.y = 0.2;
    raw.pose.position.z = 1.1;
    raw.pose.orientation.z = std::sin(half);
    raw.pose.orientation.w = std::cos(half);
    ASSERT_TRUE(projectLocalizationPose(raw, projection, projected_out));
    ASSERT_TRUE(error_window.mocap(*projected_out));
    xyz_window.recordPose(t, projected_out->pose.position.x,
                          projected_out->pose.position.y,
                          projected_out->pose.position.z);
  };

  // Baseline orientation (0 deg) followed by the ~180 deg rebuild flip
  // magnitudes observed onsite (179, 172, 178 deg) with zero XYZ shift.
  geometry_msgs::PoseStamped baseline;
  feed_pair(0u, 0.5, 0.0, &baseline);
  for (std::size_t idx = 1u; idx <= 6u; ++idx) {
    const double flip_deg =
        (idx % 3u == 1u) ? 179.0 : ((idx % 3u == 2u) ? 172.0 : 178.0);
    geometry_msgs::PoseStamped flipped;
    feed_pair(idx, 0.5, flip_deg, &flipped);

    const double dot = std::min(
        1.0,
        std::fabs(baseline.pose.orientation.x * flipped.pose.orientation.x +
                  baseline.pose.orientation.y * flipped.pose.orientation.y +
                  baseline.pose.orientation.z * flipped.pose.orientation.z +
                  baseline.pose.orientation.w * flipped.pose.orientation.w));
    const double separation_deg = 2.0 * std::acos(dot) * 180.0 / kPi;
    EXPECT_GT(separation_deg, 171.5);
    ASSERT_TRUE(error_window.latest().has_value());
    EXPECT_DOUBLE_EQ(0.0, error_window.latest()->meters);
  }
  const auto xyz_only = xyz_window.evaluate(6u * 0.033 + 0.001);
  EXPECT_EQ(PositioningHealthState::kFrozen, xyz_only.state);
  EXPECT_DOUBLE_EQ(0.0, xyz_only.comparison_metric_m);
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

TEST(InstalledProfile, XsimAdmitsCanonicalInputsWithoutLocalizationError) {
  xgc2_ros1_robot_adapter::RobotConfig config;
  config.robot_id = "uav1";
  config.profile_id = "px4-multirotor.xsim.ros";
  config.parameters["namespace"] = "/uav1";
  config.parameters["localization_pose_topic"] = "/uav1/pose";
  config.parameters["localization_twist_topic"] = "/uav1/twist";
  config.parameters["positioning_frame_number"] = "5";
  config.parameters["positioning_comparison_threshold_m"] = "1e-10";
  NativeProfileConfig native;
  std::string error;
  ASSERT_TRUE(BuildNativeProfileConfig(config, &native, &error)) << error;
  EXPECT_EQ("/uav1/pose", native.mocap_endpoint);
  EXPECT_EQ("/uav1/twist", native.mocap_velocity_endpoint);
  EXPECT_FALSE(native.publish_vision);
  contract::ChannelMetadata channel{};
  EXPECT_FALSE(InstalledContract().channelMetadata(
      config.profile_id, "state.localization.error", &channel));
  for (const char *profile : {"px4-multirotor.physical.vrpn",
                              "px4-multirotor.gazebo.vrpn"}) {
    SCOPED_TRACE(profile);
    EXPECT_TRUE(InstalledContract().channelMetadata(
        profile, "state.localization.error", &channel));
    auto other = makeProfileConfig();
    other.profile_id = profile;
    EXPECT_TRUE(BuildNativeProfileConfig(other, &native, &error)) << error;
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
  EXPECT_EQ(4u, vision_pose.policy_count);
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

TEST(InstalledProfile, ZenohBindingAcceptsNoVrpnRosRoot) {
  auto config = makeProfileConfig();
  config.parameters.erase("mocap_source_root");
  config.parameters["zenoh_endpoint"] = "tcp/127.0.0.1:7447";
  config.parameters["localization_source"] = "gazebo";
  config.parameters["mocap_sensor"] = "0";
  NativeProfileConfig native;
  std::string error;
  ASSERT_TRUE(BuildNativeProfileConfig(config, &native, &error)) << error;
  EXPECT_TRUE(native.zenoh_localization.enabled);
  EXPECT_EQ("tcp/127.0.0.1:7447", native.zenoh_localization.endpoint);
  EXPECT_EQ("gazebo", native.zenoh_localization.source);
  EXPECT_EQ("FS150_01", native.zenoh_localization.tracker);
  EXPECT_EQ(0, native.zenoh_localization.sensor);
  EXPECT_DOUBLE_EQ(1.25, native.localization.offset_x);
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
  EXPECT_EQ(13142820175755956498ULL, InstalledContract().registryFingerprint());
  EXPECT_EQ(std::string("xgc2-px4-multirotor-ros1-adapter"),
            InstalledContract().providerDefinitionId());
}

TEST(InstalledProfile, LoadsVisionPolicyAndEstimatorStatusForVrpnProfiles) {
  auto make_vrpn_config = [](const std::string &profile_id) {
    xgc2_ros1_robot_adapter::RobotConfig config;
    config.robot_id = "drone-test";
    config.profile_id = profile_id;
    config.parameters["namespace"] = "/uav1";
    config.parameters["mocap_rigid_body"] = "uav1";
    config.parameters["mocap_source_root"] = "/vrpn_client_node";
    config.parameters["localization_offset_x"] = "0.0";
    config.parameters["localization_offset_y"] = "0.0";
    config.parameters["localization_offset_z"] = "0.0";
    config.parameters["positioning_frame_number"] = "5";
    config.parameters["positioning_comparison_threshold_m"] = "0.05";
    return config;
  };
  for (const std::string &profile_id :
       {std::string("px4-multirotor.physical.vrpn"),
        std::string("px4-multirotor.gazebo.vrpn")}) {
    NativeProfileConfig native;
    std::string error;
    ASSERT_TRUE(BuildNativeProfileConfig(make_vrpn_config(profile_id), &native,
                                         &error))
        << error;
    EXPECT_TRUE(native.publish_vision);
    EXPECT_NEAR(1.0471975511965976, native.max_vision_attitude_jump_rad, 1e-12);
    if (profile_id == "px4-multirotor.physical.vrpn") {
      EXPECT_EQ("/uav1/mavros/estimator_status",
                native.estimator_status_endpoint);
    } else {
      EXPECT_TRUE(native.estimator_status_endpoint.empty());
    }
  }

  auto zenoh_config = make_vrpn_config("px4-multirotor.physical.vrpn");
  zenoh_config.parameters.erase("mocap_source_root");
  zenoh_config.parameters["zenoh_endpoint"] = "tcp/127.0.0.1:7447";
  zenoh_config.parameters["localization_source"] = "vrpn";
  NativeProfileConfig zenoh_native;
  std::string zenoh_error;
  ASSERT_TRUE(BuildNativeProfileConfig(zenoh_config, &zenoh_native, &zenoh_error))
      << zenoh_error;
  EXPECT_TRUE(zenoh_native.zenoh_localization.enabled);
  EXPECT_TRUE(zenoh_native.publish_vision);
  EXPECT_EQ("/uav1/mavros/estimator_status",
            zenoh_native.estimator_status_endpoint);

  xgc2_ros1_robot_adapter::RobotConfig xsim_config;
  xsim_config.robot_id = "drone-xsim";
  xsim_config.profile_id = "px4-multirotor.xsim.ros";
  xsim_config.parameters["namespace"] = "/uav1";
  xsim_config.parameters["localization_pose_topic"] = "/uav1/mocap/pose";
  xsim_config.parameters["localization_twist_topic"] = "/uav1/mocap/twist";
  xsim_config.parameters["positioning_frame_number"] = "5";
  xsim_config.parameters["positioning_comparison_threshold_m"] = "0.05";
  NativeProfileConfig xsim_native;
  std::string xsim_error;
  ASSERT_TRUE(BuildNativeProfileConfig(xsim_config, &xsim_native, &xsim_error))
      << xsim_error;
  EXPECT_FALSE(xsim_native.publish_vision);
  EXPECT_TRUE(xsim_native.estimator_status_endpoint.empty());
}

TEST(QuaternionDistance, HandlesSignFlipAndDegenerateQuaternions) {
  auto make_quat = [](double axis_x, double axis_y, double axis_z,
                      double angle_deg) {
    constexpr double kPi = 3.14159265358979323846;
    const double half = (angle_deg * kPi / 180.0) * 0.5;
    const double s = std::sin(half);
    geometry_msgs::Quaternion q;
    q.x = axis_x * s;
    q.y = axis_y * s;
    q.z = axis_z * s;
    q.w = std::cos(half);
    return q;
  };
  const auto identity = make_quat(0.0, 0.0, 1.0, 0.0);
  geometry_msgs::Quaternion negated = identity;
  negated.x = -identity.x;
  negated.y = -identity.y;
  negated.z = -identity.z;
  negated.w = -identity.w;
  EXPECT_NEAR(0.0, quaternionAngularDistanceRad(identity, negated), 1e-12);
  EXPECT_NEAR(179.0 * 3.14159265358979323846 / 180.0,
              quaternionAngularDistanceRad(identity,
                                           make_quat(1.0, 0.0, 0.0, 179.0)),
              1e-9);
  EXPECT_NEAR(172.0 * 3.14159265358979323846 / 180.0,
              quaternionAngularDistanceRad(identity,
                                           make_quat(0.0, 1.0, 0.0, 172.0)),
              1e-9);
  EXPECT_NEAR(178.0 * 3.14159265358979323846 / 180.0,
              quaternionAngularDistanceRad(identity,
                                           make_quat(1.0, 0.0, 0.0, 178.0)),
              1e-9);

  geometry_msgs::Quaternion zero_q;
  zero_q.w = 0.0;
  EXPECT_TRUE(std::isinf(quaternionAngularDistanceRad(identity, zero_q)));
}

} // namespace
} // namespace xgc_px4_multirotor_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
