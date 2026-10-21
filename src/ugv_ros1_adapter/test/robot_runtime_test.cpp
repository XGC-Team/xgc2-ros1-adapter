#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include "xgc_ugv_ros1_adapter/motion_command.hpp"
#include "xgc_ugv_ros1_adapter/robot_runtime.hpp"
#include "xgc_ugv_ros1_adapter/variant.hpp"

namespace xgc_ugv_ros1_adapter {
namespace {

std::string physicalProfile(const UgvVariant &variant) {
  return variant.contract.primaryProfile().profile_id;
}

// Everything every ground vehicle type of this runtime must satisfy.
class EveryUgv : public ::testing::TestWithParam<const UgvVariant *> {
protected:
  const UgvVariant &variant() const { return *GetParam(); }
};
INSTANTIATE_TEST_SUITE_P(
    RobotTypes, EveryUgv, ::testing::Values(&scoutMini(), &mecanumUgv()),
    [](const ::testing::TestParamInfo<const UgvVariant *> &info) {
      return std::string(info.param->short_name);
    });

TEST(Variants, AreFoundByTheirProviderAndNothingElse) {
  EXPECT_EQ(&scoutMini(), variantFor("xgc2-scout-mini-ros1-adapter"));
  EXPECT_EQ(&mecanumUgv(), variantFor("xgc2-mecanum-ugv-ros1-adapter"));
  EXPECT_EQ(nullptr, variantFor("xgc2-px4-multirotor-ros1-adapter"));
  EXPECT_EQ(std::string(scoutMini().provider),
            scoutMini().contract.providerDefinitionId());
  EXPECT_EQ(std::string(mecanumUgv().provider),
            mecanumUgv().contract.providerDefinitionId());
}

TEST(Variants, DifferOnlyInTheDeclaredConfiguration) {
  EXPECT_FALSE(scoutMini().motion.lateral);
  EXPECT_TRUE(mecanumUgv().motion.lateral);
  EXPECT_EQ(HealthSource::kChassisState, scoutMini().health);
  EXPECT_EQ(HealthSource::kImu, mecanumUgv().health);
  EXPECT_TRUE(scoutMini().filter_battery);
  EXPECT_FALSE(mecanumUgv().filter_battery);
  EXPECT_DOUBLE_EQ(0.0, scoutMini().stream_rate_window_seconds);
  EXPECT_DOUBLE_EQ(1.0, mecanumUgv().stream_rate_window_seconds);
}

TEST(RosNames, BuildsNamespacedTopics) {
  EXPECT_EQ("/scout1/cmd_vel", topicName("/scout1", "cmd_vel"));
  EXPECT_EQ("/ugv1/pose", topicName("/ugv1", "pose"));
  EXPECT_EQ("/ugv1/twist", topicName("/ugv1", "twist"));
  EXPECT_EQ("/ugv1/accel", topicName("/ugv1", "accel"));
  EXPECT_EQ("/fleet/scout2/scout_status",
            topicName("/fleet/scout2", "/scout_status"));
  EXPECT_EQ("/fleet/scout2/PowerVoltage",
            topicName("/fleet/scout2", "PowerVoltage"));
  EXPECT_EQ("/fleet/scout2/scout/chassis_state",
            topicName("/fleet/scout2", "scout/chassis_state"));
  EXPECT_EQ("/fleet/ugv2/twist", topicName("/fleet/ugv2", "/twist"));
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

TEST_P(EveryUgv, ResolvesTheMotionOutputFromTheProfileAndRobotNamespace) {
  xgc2_ros1_robot_adapter::RobotConfig config;
  config.profile_id = physicalProfile(variant());
  config.parameters["namespace"] = "/fleet/ugv1";
  config.parameters["mocap_rigid_body"] = "Ugv1";

  std::string topic;
  std::string error;
  ASSERT_TRUE(resolveMotionCommandTopic(variant(), config, &topic, &error))
      << error;
  EXPECT_EQ("/fleet/ugv1/cmd_vel", topic);
}

TEST_P(EveryUgv, MotionIntentScalesByGearAndClampsToTheChassisLimits) {
  const auto &limits = variant().motion;
  geometry_msgs::Twist command;
  std::string error;

  ASSERT_TRUE(motionIntentCommand(limits, 1, 1, 1, 1, &command, &error))
      << error;
  EXPECT_NEAR(limits.maximum_linear_mps / 3.0, command.linear.x, 1e-15);
  EXPECT_NEAR(limits.maximum_angular_rps / 3.0, command.angular.z, 1e-15);
  EXPECT_NEAR(limits.lateral ? limits.maximum_linear_mps / 3.0 : 0.0,
              command.linear.y, 1e-15);
  EXPECT_DOUBLE_EQ(0.0, command.angular.x);

  ASSERT_TRUE(motionIntentCommand(limits, 2, -1, 0, -1, &command, &error))
      << error;
  EXPECT_NEAR(-2.0 * limits.maximum_linear_mps / 3.0, command.linear.x, 1e-15);
  EXPECT_NEAR(-2.0 * limits.maximum_angular_rps / 3.0, command.angular.z,
              1e-15);

  ASSERT_TRUE(motionIntentCommand(limits, 3, 1, 0, 1, &command, &error))
      << error;
  EXPECT_DOUBLE_EQ(limits.maximum_linear_mps, command.linear.x);
  EXPECT_DOUBLE_EQ(limits.maximum_angular_rps, command.angular.z);

  ASSERT_TRUE(motionIntentCommand(limits, 3, 0, 0, 0, &command, &error))
      << error;
  EXPECT_DOUBLE_EQ(0.0, command.linear.x);
  EXPECT_DOUBLE_EQ(0.0, command.linear.y);
  EXPECT_DOUBLE_EQ(0.0, command.angular.z);
}

TEST(MotionIntent, OnlyAHolonomicChassisDrivesLinearY) {
  geometry_msgs::Twist command;
  std::string error;
  ASSERT_TRUE(
      motionIntentCommand(scoutMini().motion, 3, 0, 1, 0, &command, &error));
  EXPECT_DOUBLE_EQ(0.0, command.linear.y);
  ASSERT_TRUE(
      motionIntentCommand(mecanumUgv().motion, 3, 0, -1, 0, &command, &error));
  EXPECT_DOUBLE_EQ(-mecanumUgv().motion.maximum_linear_mps, command.linear.y);
}

TEST(MotionIntent, ScoutKeepsItsSdkLimitsAndMecanumItsDeployedOnes) {
  EXPECT_DOUBLE_EQ(1.5, scoutMini().motion.maximum_linear_mps);
  EXPECT_DOUBLE_EQ(0.5235, scoutMini().motion.maximum_angular_rps);
  EXPECT_DOUBLE_EQ(1.0, mecanumUgv().motion.maximum_linear_mps);
  EXPECT_DOUBLE_EQ(1.5707963267948966, mecanumUgv().motion.maximum_angular_rps);
}

TEST_P(EveryUgv, MotionIntentRejectsValuesOutsideTheClosedDiscreteContract) {
  const auto &limits = variant().motion;
  geometry_msgs::Twist command;
  std::string error;
  EXPECT_FALSE(motionIntentCommand(limits, 0, 0, 0, 0, &command, &error));
  EXPECT_FALSE(motionIntentCommand(limits, 4, 0, 0, 0, &command, &error));
  EXPECT_FALSE(motionIntentCommand(limits, 1, -2, 0, 0, &command, &error));
  EXPECT_FALSE(motionIntentCommand(limits, 1, 0, 2, 0, &command, &error));
  EXPECT_FALSE(motionIntentCommand(limits, 1, 0, 0, 2, &command, &error));
  EXPECT_FALSE(motionIntentCommand(limits, 1, 0, 0, 0, nullptr, &error));
}

TEST_P(EveryUgv, MotionPublisherHoldsZeroUntilReleaseAndStops) {
  const auto &limits = variant().motion;
  std::vector<geometry_msgs::Twist> published;
  MotionCommandPublisher publisher(
      [&published](const geometry_msgs::Twist &command) {
        published.push_back(command);
      },
      limits);

  publisher.PublishPeriodic();
  EXPECT_TRUE(published.empty());

  std::string error;
  ASSERT_TRUE(publisher.SetIntent(2, 1, 0, -1, &error)) << error;
  ASSERT_EQ(1u, published.size());
  EXPECT_NEAR(2.0 * limits.maximum_linear_mps / 3.0, published.back().linear.x,
              1e-15);
  EXPECT_NEAR(-2.0 * limits.maximum_angular_rps / 3.0,
              published.back().angular.z, 1e-15);

  publisher.PublishPeriodic();
  ASSERT_EQ(2u, published.size());
  EXPECT_NEAR(2.0 * limits.maximum_linear_mps / 3.0, published.back().linear.x,
              1e-15);

  ASSERT_TRUE(publisher.SetIntent(2, 0, 0, 0, &error)) << error;
  ASSERT_EQ(3u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().linear.x);
  EXPECT_DOUBLE_EQ(0.0, published.back().linear.y);
  EXPECT_DOUBLE_EQ(0.0, published.back().angular.z);
  publisher.PublishPeriodic();
  ASSERT_EQ(4u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().linear.x);

  ASSERT_TRUE(publisher.SetIntent(2, 1, 0, 0, &error)) << error;
  ASSERT_EQ(5u, published.size());
  publisher.PublishPeriodic();
  ASSERT_EQ(6u, published.size());
  EXPECT_NEAR(2.0 * limits.maximum_linear_mps / 3.0, published.back().linear.x,
              1e-15);

  ASSERT_TRUE(publisher.Release(&error)) << error;
  ASSERT_EQ(7u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().linear.x);
  publisher.PublishPeriodic();
  EXPECT_EQ(7u, published.size());

  ASSERT_TRUE(publisher.SetIntent(1, 0, 0, 0, &error)) << error;
  ASSERT_EQ(8u, published.size());
  publisher.PublishPeriodic();
  ASSERT_EQ(9u, published.size());

  ASSERT_TRUE(publisher.Release(&error)) << error;
  ASSERT_EQ(10u, published.size());
  publisher.Stop();
  ASSERT_EQ(10u, published.size());
  publisher.PublishPeriodic();
  EXPECT_EQ(10u, published.size());
  EXPECT_FALSE(publisher.SetIntent(1, 1, 0, 0, &error));
  EXPECT_FALSE(publisher.Release(&error));
}

TEST_P(EveryUgv, StopWhileAMotionIntentIsActiveSendsTheFinalZero) {
  std::vector<geometry_msgs::Twist> published;
  MotionCommandPublisher publisher(
      [&published](const geometry_msgs::Twist &command) {
        published.push_back(command);
      },
      variant().motion);
  std::string error;
  ASSERT_TRUE(publisher.SetIntent(3, 1, 0, 1, &error)) << error;
  ASSERT_EQ(1u, published.size());
  EXPECT_DOUBLE_EQ(variant().motion.maximum_linear_mps,
                   published.back().linear.x);
  publisher.Stop();
  ASSERT_EQ(2u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().linear.x);
  EXPECT_DOUBLE_EQ(0.0, published.back().angular.z);
  publisher.Stop();
  EXPECT_EQ(2u, published.size());
}

TEST_P(EveryUgv, FailedPublicationDoesNotCommitTheNewIntent) {
  const double maximum = variant().motion.maximum_linear_mps;
  std::vector<geometry_msgs::Twist> published;
  MotionCommandPublisher publisher(
      [&published, maximum](const geometry_msgs::Twist &command) {
        if (command.linear.x == maximum)
          throw std::runtime_error("injected publication failure");
        published.push_back(command);
      },
      variant().motion);

  std::string error;
  ASSERT_TRUE(publisher.SetIntent(1, 1, 0, 0, &error)) << error;
  ASSERT_FALSE(publisher.SetIntent(3, 1, 0, 0, &error));
  EXPECT_NE(std::string::npos, error.find("injected publication failure"));

  publisher.PublishPeriodic();
  ASSERT_EQ(2u, published.size());
  EXPECT_NEAR(maximum / 3.0, published.back().linear.x, 1e-15);
}

TEST_P(EveryUgv, FreshnessFollowsTheDeclaredBoundaryOfTheProfile) {
  const ros::WallTime now(10, 0);
  contract::ChannelMetadata position{};
  ASSERT_TRUE(variant().contract.channelMetadata(physicalProfile(variant()),
                                                 "vrpn.position", &position));
  const double stale_after_seconds =
      static_cast<double>(position.stale_after_millis) / 1000.0;
  const ros::WallTime freshness_boundary =
      now - ros::WallDuration(stale_after_seconds);
  EXPECT_FALSE(sourceIsFresh(ros::WallTime(), now, stale_after_seconds));
  EXPECT_TRUE(sourceIsFresh(freshness_boundary, now, stale_after_seconds));
  EXPECT_FALSE(sourceIsFresh(freshness_boundary - ros::WallDuration(0, 1), now,
                             stale_after_seconds));
}

TEST(StreamRateEstimate, RetainsCredibleRatesAcrossFreshEmptyWindows) {
  const auto observed = updateStreamRateEstimate({}, 1u, 1u, 1.1, true);
  EXPECT_NEAR(0.909, observed.source_rate_hz, 0.001);
  EXPECT_NEAR(0.909, observed.output_rate_hz, 0.001);

  const auto empty = updateStreamRateEstimate(observed, 0u, 0u, 1.0, true);
  EXPECT_DOUBLE_EQ(observed.source_rate_hz, empty.source_rate_hz);
  EXPECT_DOUBLE_EQ(observed.output_rate_hz, empty.output_rate_hz);
}

TEST(StreamRateEstimate, ClearsRetainedRatesWhenTheSourceIsStale) {
  const StreamRateEstimate previous{0.9, 1.0};
  const auto stale = updateStreamRateEstimate(previous, 0u, 0u, 1.0, false);
  EXPECT_DOUBLE_EQ(0.0, stale.source_rate_hz);
  EXPECT_DOUBLE_EQ(0.0, stale.output_rate_hz);
}

TEST(StreamRateWindow, DoesNotQuantizeAOneHertzTopicOnATenHertzHealthEmit) {
  const double window = mecanumUgv().stream_rate_window_seconds;
  const auto spike = updateStreamRateWindow({}, 1u, 1u, 0.1, true, window);
  EXPECT_DOUBLE_EQ(0.0, spike.rates.source_rate_hz);
  EXPECT_FALSE(spike.close);

  const auto closed = updateStreamRateWindow({}, 1u, 1u, 1.0, true, window);
  EXPECT_NEAR(1.0, closed.rates.source_rate_hz, 1e-12);
  EXPECT_TRUE(closed.close);

  const auto empty =
      updateStreamRateWindow(closed.rates, 0u, 0u, 1.0, true, window);
  EXPECT_DOUBLE_EQ(closed.rates.source_rate_hz, empty.rates.source_rate_hz);
  EXPECT_TRUE(empty.close);

  const auto stale =
      updateStreamRateWindow(closed.rates, 0u, 0u, 0.1, false, window);
  EXPECT_DOUBLE_EQ(0.0, stale.rates.source_rate_hz);
  EXPECT_TRUE(stale.close);
}

TEST(StreamRateWindow, AZeroWindowEstimatesAtEveryEmit) {
  const double window = scoutMini().stream_rate_window_seconds;
  const auto first = updateStreamRateWindow({}, 3u, 2u, 0.1, true, window);
  EXPECT_TRUE(first.close);
  EXPECT_NEAR(30.0, first.rates.source_rate_hz, 1e-9);
  EXPECT_NEAR(20.0, first.rates.output_rate_hz, 1e-9);
  // Same arguments as the estimator itself: the window adds nothing.
  const auto estimate = updateStreamRateEstimate({}, 3u, 2u, 0.1, true);
  EXPECT_DOUBLE_EQ(estimate.source_rate_hz, first.rates.source_rate_hz);
  EXPECT_DOUBLE_EQ(estimate.output_rate_hz, first.rates.output_rate_hz);
  const auto idle = updateStreamRateWindow(first.rates, 0u, 0u, 0.0, true, window);
  EXPECT_TRUE(idle.close);
  EXPECT_DOUBLE_EQ(first.rates.source_rate_hz, idle.rates.source_rate_hz);
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

TEST(VrpnAccelerationProjection, MapsCanonicalAccelStampedLinearAndAngular) {
  geometry_msgs::AccelStamped source;
  source.header.frame_id = "world";
  source.accel.linear.x = 1.25;
  source.accel.linear.y = -2.5;
  source.accel.linear.z = 3.75;
  source.accel.angular.x = -0.1;
  source.accel.angular.y = 0.2;
  source.accel.angular.z = -0.3;

  const auto estimate = vrpnAccelerationEstimate(source);
  EXPECT_EQ("world", estimate.frame_id());
  EXPECT_DOUBLE_EQ(1.25, estimate.linear().x());
  EXPECT_DOUBLE_EQ(-2.5, estimate.linear().y());
  EXPECT_DOUBLE_EQ(3.75, estimate.linear().z());
  EXPECT_DOUBLE_EQ(-0.1, estimate.angular().x());
  EXPECT_DOUBLE_EQ(0.2, estimate.angular().y());
  EXPECT_DOUBLE_EQ(-0.3, estimate.angular().z());
}

TEST(PositioningHealth, DetectsXgc1RepeatFramesTimeoutAndRecovery) {
  using xgc2_ros1_robot_adapter::PositioningHealthConfig;
  using xgc2_ros1_robot_adapter::PositioningHealthReason;
  using xgc2_ros1_robot_adapter::PositioningHealthState;
  using xgc2_ros1_robot_adapter::PositioningHealthWindow;
  const PositioningHealthConfig config{5u, 1.0e-10, 0.1};
  PositioningHealthWindow health(config);
  for (int index = 0; index < 6; ++index)
    health.recordPose(index * 0.01, 1.0, 2.0, 3.0);
  const auto frozen = health.evaluate(0.051);
  EXPECT_EQ(PositioningHealthState::kFrozen, frozen.state);
  EXPECT_EQ(PositioningHealthReason::kRepeatFrameWindowFrozen, frozen.reason);
  EXPECT_EQ(1u, frozen.observed_age_ms);

  // XGC1 UGV physical mocap also enables its single-axis check. Only Z needs
  // to vary for the source to become active.
  health.recordPose(0.06, 1.0, 2.0, 3.0 + 2.0e-10);
  const auto active = health.evaluate(0.061);
  EXPECT_EQ(PositioningHealthState::kActive, active.state);
  EXPECT_EQ(PositioningHealthReason::kWindowVariationObserved, active.reason);
  EXPECT_GT(active.comparison_metric_m, config.comparison_threshold_m);

  const auto timeout_result = health.evaluate(0.16);
  EXPECT_EQ(PositioningHealthState::kTimedOut, timeout_result.state);
  EXPECT_EQ(PositioningHealthReason::kVrpnTimeout, timeout_result.reason);
  EXPECT_EQ(100u, timeout_result.observed_age_ms);

  health.recordPose(0.17, 1.0 + 4.0e-10, 2.0, 3.0);
  EXPECT_EQ(PositioningHealthState::kActive, health.evaluate(0.171).state);
}

TEST(ChassisState, PacksAndUnpacksModeBaseAndFault) {
  const std::uint32_t word = packChassisState(1u, 2u, 0xABCDu);
  EXPECT_EQ(0xABCDu << 16 | 2u << 8 | 1u, word);
  ChassisState parsed;
  ASSERT_TRUE(unpackChassisState(word, &parsed));
  EXPECT_EQ(1u, parsed.control_mode);
  EXPECT_EQ(2u, parsed.base_state);
  EXPECT_EQ(0xABCDu, parsed.fault_code);
  EXPECT_FALSE(unpackChassisState(0u, nullptr));
}

TEST(ChassisProjection, MapsNativeScoutControlModes) {
  using Status = xgc::semantic::ground::v1::ChassisStatus;
  EXPECT_EQ(Status::CONTROL_MODE_COMMAND_CAN, chassisControlMode(0));
  EXPECT_EQ(Status::CONTROL_MODE_COMMAND_CAN, chassisControlMode(1));
  EXPECT_EQ(Status::CONTROL_MODE_COMMAND_UART, chassisControlMode(2));
  EXPECT_EQ(Status::CONTROL_MODE_REMOTE, chassisControlMode(3));
  EXPECT_EQ(Status::CONTROL_MODE_UNSPECIFIED, chassisControlMode(255));
}

std::vector<xgc2_ros1_robot_adapter::BatteryCurvePoint>
batteryCurve(const UgvVariant &variant) {
  contract::ChannelMetadata power{};
  EXPECT_TRUE(variant.contract.channelMetadata(physicalProfile(variant),
                                               "state.power", &power));
  const char *const *entries = nullptr;
  std::size_t count = 0u;
  EXPECT_TRUE(contract::channelPolicyStringArray(
      power, "battery_voltage_percentage_curve", &entries, &count));
  std::vector<xgc2_ros1_robot_adapter::BatteryCurvePoint> curve;
  std::string error;
  EXPECT_TRUE(xgc2_ros1_robot_adapter::parseBatteryCurve(entries, count, &curve,
                                                        &error))
      << error;
  return curve;
}

TEST(BatteryProjection, UsesNonlinearNmcEstimateForOriginalScoutPack) {
  const auto curve = batteryCurve(scoutMini());
  double percentage = 0.0;
  ASSERT_TRUE(
      xgc2_ros1_robot_adapter::batteryPercentage(curve, 24.4f, &percentage));
  EXPECT_GT(percentage, 0.09);
  EXPECT_LT(percentage, 0.10); // Field voltage, not 0% or the linear model's 45%.
  ASSERT_TRUE(xgc2_ros1_robot_adapter::batteryPercentage(curve, 25.772414,
                                                        &percentage));
  EXPECT_NEAR(0.5, percentage, 1e-6);
  ASSERT_TRUE(
      xgc2_ros1_robot_adapter::batteryPercentage(curve, 29.2, &percentage));
  EXPECT_DOUBLE_EQ(1.0, percentage);
  ASSERT_TRUE(
      xgc2_ros1_robot_adapter::batteryPercentage(curve, 20.5, &percentage));
  EXPECT_DOUBLE_EQ(0.0, percentage);
  EXPECT_FALSE(xgc2_ros1_robot_adapter::batteryPercentage(
      curve, std::numeric_limits<double>::quiet_NaN(), &percentage));
}

TEST(BatteryProjection, UsesTheFrozenMecanumProfileCurve) {
  contract::ChannelMetadata power{};
  ASSERT_TRUE(mecanumUgv().contract.channelMetadata(
      physicalProfile(mecanumUgv()), "state.power", &power));
  const char *const *entries = nullptr;
  std::size_t count = 0u;
  ASSERT_TRUE(contract::channelPolicyStringArray(
      power, "battery_voltage_percentage_curve", &entries, &count));
  ASSERT_EQ(2u, count);
  const auto curve = batteryCurve(mecanumUgv());
  double percentage = 0.0;
  EXPECT_TRUE(
      xgc2_ros1_robot_adapter::batteryPercentage(curve, 12.348, &percentage));
  EXPECT_NEAR(0.88, percentage, 1e-12);
}

TEST(BatteryProjection, FiltersTransientSagButTracksSustainedDischargeAndReconnect) {
  xgc2_ros1_robot_adapter::BatteryVoltageFilter filter;
  double voltage = 0.0;
  ASSERT_TRUE(filter.update(26.0, 10.0, &voltage));
  EXPECT_DOUBLE_EQ(26.0, voltage);
  ASSERT_TRUE(filter.update(26.0, 11.0, &voltage));
  ASSERT_TRUE(filter.update(23.0, 12.0, &voltage));
  EXPECT_DOUBLE_EQ(26.0, voltage);
  ASSERT_TRUE(filter.update(26.0, 13.0, &voltage));
  EXPECT_DOUBLE_EQ(26.0, voltage);
  for (int time = 14; time < 75; ++time)
    ASSERT_TRUE(filter.update(24.4, time, &voltage));
  EXPECT_NEAR(24.4, voltage, 0.01);
  ASSERT_TRUE(filter.update(28.0, 100.0, &voltage));
  EXPECT_DOUBLE_EQ(28.0, voltage);
  EXPECT_FALSE(filter.update(0.0, 101.0, &voltage));
  ASSERT_TRUE(filter.update(25.0, 102.0, &voltage));
  EXPECT_DOUBLE_EQ(25.0, voltage);
}

TEST_P(EveryUgv, InstalledProfilesMatchTheRuntimeBindings) {
  std::string error;
  EXPECT_TRUE(validateNativeProfileContract(variant(), &error)) << error;
  for (const char *suffix : {"physical.vrpn", "xsim.ros", "gazebo.vrpn"}) {
    const std::string profile =
        std::string(variant().processor_prefix) + "." + suffix;
    const char *digest = variant().contract.profileDigest(profile);
    ASSERT_NE(nullptr, digest) << profile;
    EXPECT_EQ(64u, std::string(digest).size());
  }
  EXPECT_EQ(std::string(variant().processor_prefix) + ".physical.vrpn",
            physicalProfile(variant()));
  EXPECT_EQ(nullptr, variant().contract.profileDigest("px4-multirotor.physical.vrpn"));
}

TEST_P(EveryUgv, OffersExactlyTheMotionOperationsAndNoPx4Ones) {
  const std::string profile = physicalProfile(variant());
  contract::ChannelMetadata motion;
  ASSERT_TRUE(variant().contract.channelMetadata(profile, "operation.motion-intent",
                                                 &motion));
  EXPECT_EQ(contract::ChannelKind::kOperation, motion.kind);
  EXPECT_EQ(3205u, motion.input_message_id);
  EXPECT_EQ(1u, motion.output_message_id);
  EXPECT_EQ(std::string(variant().processor_prefix) + ".set-motion-intent",
            std::string(motion.processor));
  const auto *output = contract::channelEndpoint(
      motion, contract::EndpointKind::kOutput, "output");
  ASSERT_NE(nullptr, output);
  EXPECT_EQ("cmd_vel", std::string(output->name_template));
  EXPECT_EQ("geometry_msgs/Twist", std::string(output->ros_type));

  std::size_t operation_count = 0u;
  const auto *operations =
      variant().contract.profileOperations(profile, &operation_count);
  ASSERT_NE(nullptr, operations);
  ASSERT_EQ(2u, operation_count);
  EXPECT_EQ("release-motion-intent", std::string(operations[0].operation_id));
  EXPECT_EQ("set-motion-intent", std::string(operations[1].operation_id));

  contract::MessageMetadata metadata;
  EXPECT_TRUE(variant().contract.messageMetadata(2001, &metadata));
  ASSERT_TRUE(variant().contract.messageMetadata(2005, &metadata));
  EXPECT_EQ(2u, metadata.version);
  ASSERT_TRUE(variant().contract.messageMetadata(3205, &metadata));
  EXPECT_EQ("xgc.semantic.common.v1.RemoteControlIntentRequest",
            std::string(metadata.type_name));
  EXPECT_FALSE(variant().contract.messageMetadata(3201, &metadata));
  contract::ChannelMetadata unknown;
  EXPECT_FALSE(variant().contract.channelMetadata(profile, "operation.arm", &unknown));
}

TEST_P(EveryUgv, RawVrpnAccelerationUsesTwistAndCanonicalUsesAccel) {
  for (const char *suffix : {"physical.vrpn", "gazebo.vrpn"}) {
    const std::string profile =
        std::string(variant().processor_prefix) + "." + suffix;
    SCOPED_TRACE(profile);
    contract::ChannelMetadata acceleration{};
    ASSERT_TRUE(variant().contract.channelMetadata(
        profile, "vrpn.acceleration", &acceleration));
    const auto *input = contract::channelEndpoint(
        acceleration, contract::EndpointKind::kInput, "acceleration");
    const auto *output = contract::channelEndpoint(
        acceleration, contract::EndpointKind::kOutput, "output");
    ASSERT_NE(nullptr, input);
    ASSERT_NE(nullptr, output);
    EXPECT_STREQ("geometry_msgs/TwistStamped", input->ros_type);
    EXPECT_STREQ("geometry_msgs/AccelStamped", output->ros_type);
  }
}

TEST_P(EveryUgv, StreamsPositionVelocityAccelerationAndSpeedFromTheMocapSource) {
  const std::string profile = physicalProfile(variant());
  contract::ChannelMetadata position, velocity, acceleration, speed, command;
  ASSERT_TRUE(variant().contract.channelMetadata(profile, "vrpn.position", &position));
  ASSERT_TRUE(variant().contract.channelMetadata(profile, "vrpn.velocity", &velocity));
  ASSERT_TRUE(variant().contract.channelMetadata(profile, "vrpn.acceleration", &acceleration));
  ASSERT_TRUE(variant().contract.channelMetadata(profile, "vrpn.speed", &speed));
  ASSERT_TRUE(variant().contract.channelMetadata(profile, "command.velocity", &command));
  EXPECT_EQ(2001u, position.output_message_id);
  EXPECT_EQ(2002u, velocity.output_message_id);
  EXPECT_EQ(2008u, acceleration.output_message_id);
  EXPECT_EQ(2006u, speed.output_message_id);
  EXPECT_EQ(2002u, command.output_message_id);
  const auto *accel_endpoint = contract::channelEndpoint(
      acceleration, contract::EndpointKind::kInput, "acceleration");
  ASSERT_NE(nullptr, accel_endpoint);
  EXPECT_EQ("{mocap_rigid_body}/accel", std::string(accel_endpoint->name_template));
  EXPECT_EQ("geometry_msgs/TwistStamped", std::string(accel_endpoint->ros_type));
  EXPECT_EQ(contract::EndpointScope::kGlobal, accel_endpoint->scope);
  const auto *canonical_accel = contract::channelEndpoint(
      acceleration, contract::EndpointKind::kOutput, "output");
  ASSERT_NE(nullptr, canonical_accel);
  EXPECT_EQ("accel", std::string(canonical_accel->name_template));
  EXPECT_EQ("geometry_msgs/AccelStamped", std::string(canonical_accel->ros_type));
  EXPECT_EQ(contract::EndpointScope::kRobotNamespace, canonical_accel->scope);

  contract::ChannelMetadata diagnostics, controller;
  ASSERT_TRUE(variant().contract.channelMetadata(profile, "diagnostic.stream-health", &diagnostics));
  EXPECT_EQ(2011u, diagnostics.output_message_id);
  EXPECT_EQ("common.stream-health-report", std::string(diagnostics.processor));
  EXPECT_FALSE(variant().contract.channelMetadata(profile, "diagnostic.channel-health", &diagnostics));
  ASSERT_TRUE(variant().contract.channelMetadata(profile, "state.controller", &controller));
  EXPECT_EQ(2012u, controller.output_message_id);
  EXPECT_DOUBLE_EQ(5.0, controller.output_rate_hz);
  EXPECT_EQ(std::string(variant().processor_prefix) + ".controller-status",
            std::string(controller.processor));

  contract::ChannelMetadata reset;
  ASSERT_TRUE(variant().contract.channelMetadata(profile, "state.reset", &reset));
  EXPECT_EQ(3105u, reset.output_message_id);
  EXPECT_DOUBLE_EQ(5.0, reset.output_rate_hz);
  EXPECT_EQ(2000u, reset.stale_after_millis);
  EXPECT_EQ(std::string(variant().processor_prefix) + ".reset-status",
            std::string(reset.processor));
  EXPECT_EQ(2u, reset.endpoint_count);
  const auto *session = contract::channelEndpoint(
      reset, contract::EndpointKind::kInput, "session");
  const auto *result = contract::channelEndpoint(
      reset, contract::EndpointKind::kInput, "result");
  ASSERT_NE(nullptr, session);
  ASSERT_NE(nullptr, result);
  EXPECT_EQ("reset/session", std::string(session->name_template));
  EXPECT_EQ("ugv_reset_msgs/ResetSession", std::string(session->ros_type));
  EXPECT_EQ(contract::EndpointScope::kRobotNamespace, session->scope);
  EXPECT_EQ("reset/result", std::string(result->name_template));
  EXPECT_EQ("ugv_reset_msgs/ResetResult", std::string(result->ros_type));
  EXPECT_EQ(contract::EndpointScope::kRobotNamespace, result->scope);
  contract::MessageMetadata reset_message{};
  ASSERT_TRUE(variant().contract.messageMetadata(3105u, &reset_message));
  EXPECT_EQ("xgc.semantic.ground.v1.ResetStatus",
            std::string(reset_message.type_name));
}

TEST(InstalledProfile, ScoutReportsHealthAndChassisFromTheChassisStateWord) {
  const std::string profile = physicalProfile(scoutMini());
  contract::ChannelMetadata health, chassis;
  ASSERT_TRUE(scoutMini().contract.channelMetadata(profile, "state.health", &health));
  EXPECT_EQ(2005u, health.output_message_id);
  EXPECT_EQ(3u, health.observes_count);
  EXPECT_EQ(0u, health.policy_count);
  const auto *state = contract::channelEndpoint(
      health, contract::EndpointKind::kInput, "chassis_state");
  ASSERT_NE(nullptr, state);
  EXPECT_EQ("scout/chassis_state", std::string(state->name_template));
  EXPECT_EQ("std_msgs/UInt32", std::string(state->ros_type));
  ASSERT_TRUE(scoutMini().contract.channelMetadata(profile, "state.chassis", &chassis));
  EXPECT_EQ(3102u, chassis.output_message_id);
  std::size_t channel_count = 0u;
  scoutMini().contract.profileChannels(profile, &channel_count);
  EXPECT_EQ(14u, channel_count);
}

TEST(InstalledProfile, MecanumIsTheMinimalContractAtTenHertzWithoutChassisState) {
  const std::string profile = physicalProfile(mecanumUgv());
  std::size_t channel_count = 0u;
  ASSERT_NE(nullptr, mecanumUgv().contract.profileChannels(profile, &channel_count));
  ASSERT_EQ(13u, channel_count);
  for (const char *channel_id :
       {"vrpn.position", "vrpn.velocity", "vrpn.acceleration", "vrpn.speed",
        "command.velocity", "diagnostic.stream-health"}) {
    contract::ChannelMetadata channel{};
    ASSERT_TRUE(mecanumUgv().contract.channelMetadata(profile, channel_id, &channel));
    EXPECT_EQ(contract::ChannelKind::kStreamOut, channel.kind);
    EXPECT_DOUBLE_EQ(10.0, channel.output_rate_hz) << channel_id;
  }
  contract::ChannelMetadata health{};
  ASSERT_TRUE(mecanumUgv().contract.channelMetadata(profile, "state.health", &health));
  EXPECT_EQ(2005u, health.output_message_id);
  EXPECT_DOUBLE_EQ(1.0, health.output_rate_hz);
  EXPECT_EQ(3u, health.observes_count);
  EXPECT_EQ(0u, health.endpoint_count);
  contract::ChannelMetadata forbidden{};
  EXPECT_FALSE(mecanumUgv().contract.channelMetadata(profile, "state.odom", &forbidden));
  EXPECT_FALSE(mecanumUgv().contract.channelMetadata(profile, "state.chassis", &forbidden));
}

} // namespace
} // namespace xgc_ugv_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
