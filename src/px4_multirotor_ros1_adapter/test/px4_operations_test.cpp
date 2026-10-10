#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "xgc_px4_multirotor_ros1_adapter/px4_operations.hpp"

namespace xgc_px4_multirotor_ros1_adapter {
namespace {

TEST(Px4OperationMapping, BuildsTypedMavrosRequests) {
  const mavros_msgs::CommandBool arm = makeArmCommand(true);
  EXPECT_TRUE(arm.request.value);

  const mavros_msgs::CommandBool disarm = makeArmCommand(false);
  EXPECT_FALSE(disarm.request.value);

  const mavros_msgs::SetMode mode = makeModeCommand("OFFBOARD");
  EXPECT_EQ(0u, mode.request.base_mode);
  EXPECT_EQ("OFFBOARD", mode.request.custom_mode);

  const mavros_msgs::CommandLong reboot = makeAutopilotRebootCommand();
  EXPECT_FALSE(reboot.request.broadcast);
  EXPECT_EQ(kPx4RebootMavCommand, reboot.request.command);
  EXPECT_EQ(0u, reboot.request.confirmation);
  EXPECT_FLOAT_EQ(kPx4NormalRebootParam1, reboot.request.param1);
  EXPECT_FLOAT_EQ(0.0F, reboot.request.param2);
  EXPECT_FLOAT_EQ(0.0F, reboot.request.param7);

  const mavros_msgs::CommandLong kill = makeForceDisarmCommand();
  EXPECT_FALSE(kill.request.broadcast);
  EXPECT_EQ(kPx4ForceDisarmMavCommand, kill.request.command);
  EXPECT_EQ(0u, kill.request.confirmation);
  EXPECT_FLOAT_EQ(0.0F, kill.request.param1);
  EXPECT_FLOAT_EQ(kPx4ForceDisarmParam2, kill.request.param2);
}

TEST(Px4OperationPolicy, AllowsOnlyTheProfileModeAllowlist) {
  const std::vector<std::string> allowed_modes{"OFFBOARD", "POSCTL", "ALTCTL",
                                               "STABILIZED"};
  EXPECT_TRUE(isAllowedPx4Mode("OFFBOARD", allowed_modes));
  EXPECT_TRUE(isAllowedPx4Mode("POSCTL", allowed_modes));
  EXPECT_TRUE(isAllowedPx4Mode("ALTCTL", allowed_modes));
  EXPECT_TRUE(isAllowedPx4Mode("STABILIZED", allowed_modes));

  EXPECT_FALSE(isAllowedPx4Mode("", allowed_modes));
  EXPECT_FALSE(isAllowedPx4Mode("offboard", allowed_modes));
  EXPECT_FALSE(isAllowedPx4Mode("AUTO.MISSION", allowed_modes));
  EXPECT_FALSE(isAllowedPx4Mode("MANUAL", allowed_modes));
  EXPECT_FALSE(isAllowedPx4Mode("OFFBOARD", {}));
}

TEST(Px4RebootSafety, RequiresKnownFreshConnectedDisarmedState) {
  const ros::WallTime now(100u, 0u);
  Px4StateSnapshot state;

  EXPECT_EQ(Px4RebootReadiness::kStateUnknown,
            evaluatePx4RebootReadiness(state, now));

  state.known = true;
  state.observed_at = ros::WallTime(99u, 0u);
  state.connected = true;
  EXPECT_EQ(Px4RebootReadiness::kReady, evaluatePx4RebootReadiness(state, now));

  state.observed_at = ros::WallTime(98u, 999999999u);
  EXPECT_EQ(Px4RebootReadiness::kStateStale,
            evaluatePx4RebootReadiness(state, now));

  state.observed_at = ros::WallTime(99u, 500000000u);
  state.connected = false;
  EXPECT_EQ(Px4RebootReadiness::kDisconnected,
            evaluatePx4RebootReadiness(state, now));

  state.connected = true;
  state.armed = true;
  EXPECT_EQ(Px4RebootReadiness::kArmed, evaluatePx4RebootReadiness(state, now));
  EXPECT_STREQ("armed autopilot cannot be rebooted",
               px4RebootReadinessDetail(Px4RebootReadiness::kArmed));
}

TEST(Px4NativeResponse, InterpretsArmResultAndPreservesNativeCode) {
  mavros_msgs::CommandBool::Response response;
  response.success = true;
  response.result = 0u;
  OperationResult result = interpretArmResponse(true, response);
  EXPECT_TRUE(result.succeeded());
  EXPECT_TRUE(result.dispatched);
  EXPECT_TRUE(result.has_native_result);
  EXPECT_EQ(0u, result.native_result);
  EXPECT_NE(std::string::npos, result.detail.find("MAV_RESULT_ACCEPTED"));

  response.success = false;
  response.result = 1u;
  result = interpretArmResponse(true, response);
  EXPECT_EQ(OperationOutcome::kNotReady, result.outcome);
  EXPECT_NE(std::string::npos,
            result.detail.find("MAV_RESULT_TEMPORARILY_REJECTED"));

  response.result = 2u;
  result = interpretArmResponse(false, response);
  EXPECT_EQ(OperationOutcome::kRejected, result.outcome);
  EXPECT_NE(std::string::npos, result.detail.find("disarm"));

  response.result = 3u;
  result = interpretArmResponse(true, response);
  EXPECT_EQ(OperationOutcome::kUnsupported, result.outcome);

  response.result = 5u;
  response.success = true;
  result = interpretArmResponse(true, response);
  EXPECT_EQ(OperationOutcome::kUncertain, result.outcome);
  EXPECT_TRUE(result.dispatched);
  EXPECT_NE(std::string::npos, result.detail.find("MAV_RESULT_IN_PROGRESS"));

  response.result = 0u;
  response.success = false;
  result = interpretArmResponse(true, response);
  EXPECT_EQ(OperationOutcome::kUncertain, result.outcome);
}

TEST(Px4NativeResponse, InterpretsModeAndRebootResponses) {
  mavros_msgs::SetMode::Response mode_response;
  mode_response.mode_sent = true;
  OperationResult result = interpretModeResponse("OFFBOARD", mode_response);
  EXPECT_TRUE(result.succeeded());
  EXPECT_TRUE(result.dispatched);
  EXPECT_FALSE(result.has_native_result);

  mode_response.mode_sent = false;
  result = interpretModeResponse("POSCTL", mode_response);
  EXPECT_EQ(OperationOutcome::kRejected, result.outcome);

  mavros_msgs::CommandLong::Response reboot_response;
  reboot_response.success = false;
  reboot_response.result = 4u;
  result = interpretAutopilotRebootResponse(reboot_response);
  EXPECT_EQ(OperationOutcome::kRejected, result.outcome);
  EXPECT_TRUE(result.dispatched);
  EXPECT_EQ(4u, result.native_result);
  EXPECT_NE(std::string::npos, result.detail.find("MAV_RESULT_FAILED"));

  reboot_response.success = true;
  reboot_response.result = 0u;
  result = interpretAutopilotRebootResponse(reboot_response);
  EXPECT_TRUE(result.succeeded());
}

TEST(Px4NativeResponse, NamesUnknownMavResultWithoutDiscardingIt) {
  mavros_msgs::CommandLong::Response response;
  response.success = false;
  response.result = 99u;
  const OperationResult result = interpretAutopilotRebootResponse(response);
  EXPECT_EQ(OperationOutcome::kUncertain, result.outcome);
  EXPECT_EQ(99u, result.native_result);
  EXPECT_NE(std::string::npos, result.detail.find("MAV_RESULT_UNKNOWN=99"));
}

} // namespace
} // namespace xgc_px4_multirotor_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
