#include <cmath>
#include <gtest/gtest.h>

#include <string>
#include <stdexcept>
#include <utility>
#include <limits>

#include "xgc/semantic/aerial/v1/control.pb.h"
#include "xgc2_ros1_robot_adapter/robot_domain.hpp"
#include "xgc2_ros1_robot_adapter/localization_projection.hpp"
#include "xgc2_ros1_robot_adapter/ground_health.hpp"
#include "xgc2_ros1_robot_adapter/native_command.hpp"

namespace xgc2_ros1_robot_adapter {
namespace {

TEST(LocalizationProjection, ComposesMarkerBodyBeforeWorldOffsetAndKeepsTime) {
  LocalizationProjectionConfig config;
  config.has_extrinsic = true;
  config.marker_from_body_translation = {{1, 0, 0}};
  const double h = std::sqrt(0.5);
  config.marker_from_body_rotation = {{h, 0, 0, h}};
  config.offset_x = 10;
  geometry_msgs::PoseStamped source;
  source.header.stamp = ros::Time(123, 456);
  source.header.frame_id = "world";
  source.pose.position.x = 2;
  source.pose.orientation.z = h;
  source.pose.orientation.w = h;
  geometry_msgs::PoseStamped result;
  ASSERT_TRUE(projectLocalizationPose(source, config, &result));
  EXPECT_NEAR(result.pose.position.x, 12, 1e-12);
  EXPECT_NEAR(result.pose.position.y, 1, 1e-12);
  EXPECT_NEAR(result.pose.orientation.x, .5, 1e-12);
  EXPECT_NEAR(result.pose.orientation.y, .5, 1e-12);
  EXPECT_NEAR(result.pose.orientation.z, .5, 1e-12);
  EXPECT_NEAR(result.pose.orientation.w, .5, 1e-12);
  EXPECT_EQ(result.header, source.header);
}

TEST(LocalizationProjection, RejectsMalformedCalibrationBeforeAnyPose) {
  LocalizationProjectionConfig config;
  std::string error;
  for (const std::string json : {"{}", "{\"translation\":[0,0,0],\"rotation\":[0,0,0,0]}",
                                 "{\"translation\":[0,0],\"rotation\":[0,0,0,1]}"}) {
    EXPECT_FALSE(parseLocalizationExtrinsic({{"mocap_marker_from_body", json}}, &config, &error));
    EXPECT_FALSE(config.has_extrinsic);
  }
  EXPECT_TRUE(parseLocalizationExtrinsic({{"mocap_marker_from_body",
    "{\"translation\":[0.1,0.2,0.3],\"rotation\":[0,0,0,1]}"}}, &config, &error));
  EXPECT_TRUE(config.has_extrinsic);
}

TEST(LocalizationProjection, AppliesOnlyXYZAndPreservesTheSourceFact) {
  xgc2_ros1_robot_adapter::LocalizationProjectionConfig config;
  config.source_root = "/vrpn_client_node_physical";
  config.offset_x = 1.0;
  config.offset_y = -2.0;
  config.offset_z = 0.25;
  geometry_msgs::PoseStamped source;
  source.header.frame_id = "world";
  source.header.stamp = ros::Time(7, 8);
  source.pose.position.x = 3.0;
  source.pose.position.y = 4.0;
  source.pose.position.z = 5.0;
  source.pose.orientation.x = 0.6;
  source.pose.orientation.w = 0.8;
  geometry_msgs::PoseStamped projected;
  ASSERT_TRUE(xgc2_ros1_robot_adapter::projectLocalizationPose(source, config,
                                                               &projected));
  EXPECT_DOUBLE_EQ(projected.pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(projected.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(projected.pose.position.z, 5.25);
  EXPECT_EQ(projected.header, source.header);
  EXPECT_EQ(projected.pose.orientation, source.pose.orientation);
}

TEST(LocalizationProjection,
     EnforcesUnitQuaternionNormWithinFivePercentAndFiniteComponents) {
  LocalizationProjectionConfig no_extrinsic;
  LocalizationProjectionConfig with_extrinsic;
  with_extrinsic.has_extrinsic = true;
  with_extrinsic.marker_from_body_translation = {{0.1, 0.2, 0.3}};
  with_extrinsic.marker_from_body_rotation = {{0.0, 0.0, 0.0, 1.0}};

  geometry_msgs::PoseStamped projected;

  // Zero quaternion must be rejected cleanly with and without extrinsic.
  geometry_msgs::PoseStamped zero_quat;
  EXPECT_FALSE(validLocalizationPose(zero_quat));
  EXPECT_FALSE(projectLocalizationPose(zero_quat, no_extrinsic, &projected));
  EXPECT_FALSE(projectLocalizationPose(zero_quat, with_extrinsic, &projected));

  // Norm deviations > 0.05 (5%) must be rejected; <= 0.05 must pass.
  for (const double bad_norm : {1.0e-6, 0.94, 1.06, 3.0}) {
    geometry_msgs::PoseStamped bad_pose;
    bad_pose.pose.orientation.w = bad_norm;
    EXPECT_FALSE(validLocalizationPose(bad_pose)) << "norm=" << bad_norm;
    EXPECT_FALSE(projectLocalizationPose(bad_pose, no_extrinsic, &projected))
        << "norm=" << bad_norm;
    EXPECT_FALSE(projectLocalizationPose(bad_pose, with_extrinsic, &projected))
        << "norm=" << bad_norm;
  }
  for (const double ok_norm : {0.96, 1.0, 1.04}) {
    geometry_msgs::PoseStamped ok_pose;
    ok_pose.pose.orientation.w = ok_norm;
    EXPECT_TRUE(validLocalizationPose(ok_pose)) << "norm=" << ok_norm;
    EXPECT_TRUE(projectLocalizationPose(ok_pose, no_extrinsic, &projected))
        << "norm=" << ok_norm;
    EXPECT_TRUE(projectLocalizationPose(ok_pose, with_extrinsic, &projected))
        << "norm=" << ok_norm;
  }

  // Non-finite values (NaN, +Inf, -Inf) in any of the 7 pose components fail.
  for (std::size_t idx = 0u; idx < 7u; ++idx) {
    for (const double bad : {std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity()}) {
      geometry_msgs::PoseStamped pose;
      pose.pose.orientation.w = 1.0;
      double *fields[] = {&pose.pose.position.x,    &pose.pose.position.y,
                          &pose.pose.position.z,    &pose.pose.orientation.x,
                          &pose.pose.orientation.y, &pose.pose.orientation.z,
                          &pose.pose.orientation.w};
      *fields[idx] = bad;
      EXPECT_FALSE(validLocalizationPose(pose)) << "idx=" << idx;
      EXPECT_FALSE(projectLocalizationPose(pose, no_extrinsic, &projected))
          << "idx=" << idx;
      EXPECT_FALSE(projectLocalizationPose(pose, with_extrinsic, &projected))
          << "idx=" << idx;
    }
  }
}

TEST(LocalizationProjection, MapsRawVrpnAccelerationWithoutChangingFields) {
  geometry_msgs::TwistStamped source;
  source.header.seq = 42;
  source.header.frame_id = "world";
  source.header.stamp = ros::Time(7, 8);
  source.twist.linear.x = 1.25;
  source.twist.linear.y = -2.5;
  source.twist.linear.z = 3.75;
  source.twist.angular.x = -4.25;
  source.twist.angular.y = 5.5;
  source.twist.angular.z = -6.75;

  const auto projected = projectVrpnAcceleration(source);
  EXPECT_EQ(source.header, projected.header);
  EXPECT_EQ(source.twist.linear, projected.accel.linear);
  EXPECT_EQ(source.twist.angular, projected.accel.angular);
  EXPECT_TRUE(validLocalizationAcceleration(projected));
}

TEST(LocalizationProjection, RawVrpnAccelerationKeepsTheExistingFiniteCheck) {
  for (std::size_t component = 0; component < 6u; ++component) {
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()}) {
      geometry_msgs::TwistStamped source;
      double *fields[] = {&source.twist.linear.x, &source.twist.linear.y,
                          &source.twist.linear.z, &source.twist.angular.x,
                          &source.twist.angular.y, &source.twist.angular.z};
      *fields[component] = invalid;
      EXPECT_FALSE(validLocalizationAcceleration(projectVrpnAcceleration(source)))
          << "component " << component;
    }
  }
}

TEST(LocalizationProjection, ParsesStrictRunScopedParameters) {
  std::map<std::string, std::string> parameters{
      {"mocap_source_root", "/vrpn_client_node_simulation"},
      {"localization_offset_x", "0"},
      {"localization_offset_y", "-1.25"},
      {"localization_offset_z", "2.5"},
  };
  xgc2_ros1_robot_adapter::LocalizationProjectionConfig config;
  std::string error;
  ASSERT_TRUE(xgc2_ros1_robot_adapter::parseLocalizationProjectionConfig(
      parameters, &config, &error))
      << error;
  EXPECT_EQ(config.source_root, "/vrpn_client_node_simulation");
  EXPECT_DOUBLE_EQ(config.offset_y, -1.25);
  parameters["mocap_source_root"] = "vrpn_client_node";
  EXPECT_FALSE(xgc2_ros1_robot_adapter::parseLocalizationProjectionConfig(
      parameters, &config, &error));
  parameters["mocap_source_root"] = "/vrpn_client_node";
  parameters["localization_offset_x"] = "nan";
  EXPECT_FALSE(xgc2_ros1_robot_adapter::parseLocalizationProjectionConfig(
      parameters, &config, &error));
}

TEST(PositioningLiveness, RepeatedFramesAgeOutAndClockRewindStartsANewWindow) {
  PositioningHealthWindow health({3u, 0.1, 0.1});
  for (int frame = 0; frame < 64; ++frame)
    health.recordPose(1.0 + frame * .001, 1.0, 2.0, 3.0);
  EXPECT_EQ(PositioningHealthState::kFrozen, health.evaluate(1.063).state);
  // One-axis movement proves freshness until the last different old frame
  // leaves the original three-frame comparison window.
  for (int frame = 0; frame < 3; ++frame) {
    health.recordPose(1.064 + frame * .001, 1.0, 3.0, 3.0);
    EXPECT_EQ(PositioningHealthState::kActive, health.evaluate(1.064 + frame * .001).state);
  }
  health.recordPose(1.067, 1.0, 3.0, 3.0);
  EXPECT_EQ(PositioningHealthState::kFrozen, health.evaluate(1.067).state);
  EXPECT_EQ(3u, health.evaluate(1.067).sample_count);
  health.recordPose(.01, 5.0, 6.0, 7.0);
  EXPECT_EQ(1u, health.evaluate(.01).sample_count);
  EXPECT_EQ(PositioningHealthState::kFrozen, health.evaluate(.01).state);
  EXPECT_DOUBLE_EQ(0.0, health.evaluate(.01).comparison_metric_m);
  EXPECT_EQ(PositioningHealthState::kTimedOut, health.evaluate(.111).state);
}

TEST(PositioningLiveness, Uses250msDefaultTimeoutAndFourWayHealthTruth) {
  PositioningHealthConfig default_config;
  EXPECT_DOUBLE_EQ(0.25, default_config.timeout_seconds);

  PositioningHealthConfig parsed_config;
  std::string error;
  ASSERT_TRUE(parsePositioningHealthConfig(
      {{"positioning_frame_number", "5"},
       {"positioning_comparison_threshold_m", "0.05"}},
      &parsed_config, &error))
      << error;
  EXPECT_DOUBLE_EQ(0.25, parsed_config.timeout_seconds);

  PositioningHealthWindow window(parsed_config);
  for (int frame = 0; frame < 6; ++frame) {
    window.recordPose(1.0 + frame * 0.02, 1.0 + frame * 0.1, 2.0, 3.0);
  }
  EXPECT_EQ(PositioningHealthState::kActive, window.evaluate(1.349).state);
  EXPECT_EQ(PositioningHealthState::kTimedOut, window.evaluate(1.351).state);

  // Even when MAVLink #230 reports all healthy positive flags, ev_fusion_state
  // MUST remain "UNKNOWN" and never be inferred from #230 or proximity.
  Px4HealthTruthInput input;
  input.mocap_observed = true;
  input.positioning = window.evaluate(1.12);
  input.vision_output_enabled = true;
  input.vision_output_state = VisionOutputState::kPublishingValid;
  input.estimator_flags.observed = true;
  input.estimator_flags.attitude_ok = true;
  input.estimator_flags.velocity_horiz_ok = true;
  input.estimator_flags.velocity_vert_ok = true;
  input.estimator_flags.pos_horiz_rel_ok = true;
  input.estimator_flags.pos_horiz_abs_ok = true;
  input.estimator_flags.pos_vert_abs_ok = true;
  input.estimator_flags.pred_pos_horiz_rel_ok = true;
  input.estimator_flags.pred_pos_horiz_abs_ok = true;
  const auto healthy_truth = evaluatePx4HealthTruth(input);
  EXPECT_EQ(MocapInputState::kActive, healthy_truth.mocap_input.state);
  EXPECT_EQ(VisionOutputState::kPublishingValid, healthy_truth.vision_output);
  EXPECT_EQ(EstimatorHealthState::kHealthy,
            healthy_truth.estimator_health.state);
  EXPECT_TRUE(healthy_truth.estimator_health.active_degraded_flags.empty());
  EXPECT_EQ("UNKNOWN", healthy_truth.ev_fusion_state);

  // Degraded #230 flags expose generic EKF flags while ev_fusion_state stays UNKNOWN.
  input.estimator_flags.attitude_ok = false;
  input.estimator_flags.const_pos_mode = true;
  input.estimator_flags.gps_glitch = true;
  input.estimator_flags.accel_error = true;
  const auto degraded_truth = evaluatePx4HealthTruth(input);
  EXPECT_EQ(EstimatorHealthState::kDegraded,
            degraded_truth.estimator_health.state);
  EXPECT_EQ((std::vector<std::string>{"attitude_unhealthy", "const_pos_mode",
                                      "gps_glitch", "accel_error"}),
            degraded_truth.estimator_health.active_degraded_flags);
  EXPECT_EQ("UNKNOWN", degraded_truth.ev_fusion_state);
}

MessageSchema modeSchema() {
  MessageSchema schema;
  schema.message_id = 3202u;
  schema.type_name =
      xgc::semantic::aerial::v1::ModeRequest::descriptor()->full_name();
  schema.version = 1u;
  schema.fingerprint = 5742784340350308261ULL;
  return schema;
}

RobotMessageContext modeContext() {
  RobotMessageContext context;
  context.robot_id = "px4-01";
  context.channel_id = "operation.mode";
  context.sequence = 42u;
  context.has_source_time = true;
  context.source_time_nanos = 123456789;
  context.source_clock_domain = xgc::v1::CLOCK_DOMAIN_SIMULATION;
  context.observed_unix_nanos = 987654321;
  return context;
}

TEST(RobotMessageEncoder, EncodesSchemaAwareRoutedProtobufMessage) {
  xgc::semantic::aerial::v1::ModeRequest semantic;
  semantic.set_mode("OFFBOARD");
  xgc::robot::v1::RobotMessage message;
  std::string error;
  ASSERT_TRUE(EncodeRobotMessage(modeContext(), modeSchema(), semantic, &message,
                                &error))
      << error;

  EXPECT_EQ("px4-01", message.robot_id());
  EXPECT_EQ("operation.mode", message.channel_id());
  EXPECT_EQ(42u, message.message().sequence());
  EXPECT_EQ(123456789, message.message().source_time().nanoseconds());
  EXPECT_EQ(xgc::v1::CLOCK_DOMAIN_SIMULATION,
            message.message().source_time().clock_domain());
  EXPECT_EQ(987654321, message.message().observed_unix_nanos());
  EXPECT_EQ(3202u, message.message().payload().schema().message_id());
  EXPECT_EQ(modeSchema().type_name,
            message.message().payload().schema().type_name());
  EXPECT_EQ(xgc::v1::PAYLOAD_ENCODING_PROTOBUF,
            message.message().payload().encoding());

  xgc::semantic::aerial::v1::ModeRequest decoded;
  ASSERT_TRUE(decoded.ParseFromString(message.message().payload().value()));
  EXPECT_EQ("OFFBOARD", decoded.mode());
}

TEST(RobotServer, SuccessfulOperationCarriesRegisteredEmptyPayload) {
  struct Channel { unsigned output_message_id = 1; } channel;
  struct Metadata { const char *type_name = "xgc.v1.Empty"; unsigned version = 1; std::uint64_t fingerprint = 11009224659857530918ULL; } metadata;
  xgc::adapter::v1::OperationRequest request; request.mutable_context()->set_work_id("native-result");
  const auto result = CommandSuccess(request, channel, metadata);
  EXPECT_EQ(xgc::adapter::v1::OPERATION_PHASE_SUCCEEDED, result.phase());
  EXPECT_EQ("native-result", result.work_id());
  ASSERT_TRUE(result.has_output());
  EXPECT_EQ(1u, result.output().schema().message_id());
  EXPECT_EQ("xgc.v1.Empty", result.output().schema().type_name());
  EXPECT_EQ(1u, result.output().schema().schema_version());
  EXPECT_EQ(metadata.fingerprint, result.output().schema().schema_fingerprint());
  EXPECT_EQ(xgc::v1::PAYLOAD_ENCODING_PROTOBUF, result.output().encoding());
  xgc::v1::Empty empty; EXPECT_TRUE(empty.ParseFromString(result.output().value()));
}

} // namespace
} // namespace xgc2_ros1_robot_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
