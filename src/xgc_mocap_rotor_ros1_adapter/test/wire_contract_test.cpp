#include <gtest/gtest.h>

#include <array>
#include <string>

#include "xgc_mocap_rotor_ros1_adapter/wire_contract.hpp"

namespace xgc_mocap_rotor_ros1_adapter {
namespace {

TEST(MocapRotorWireContract, AcceptsOnlyTheSixReadOnlyUplinkLeaves) {
  const std::array<std::string, 6u> leaves{{
      "local_pose", "local_velocity", "imu", "power", "flight_state",
      "forwarder_hb"}};
  for (const auto &leaf : leaves) {
    ParsedWireKey parsed;
    std::string error;
    ASSERT_TRUE(ParseWireKey("xgc2/mocap-rotor-01/up/" + leaf, &parsed,
                             &error))
        << error;
    EXPECT_EQ(parsed.robot_id, "mocap-rotor-01");
    EXPECT_EQ(WireChannelLeaf(parsed.channel), leaf);
    EXPECT_GT(MaximumPayloadBytes(parsed.channel), 0u);
  }

  for (const std::string &key : {
           "xgc2/mocap-rotor-01/up/gps",
           "xgc2/mocap-rotor-01/up/navsatfix",
           "xgc2/mocap-rotor-01/down/cmd",
           "xgc2/mocap-rotor-01/up/setpoint",
           "xgc2/mocap-rotor-01/up/local_pose/extra",
           "xgc2/MocapRotor01/up/local_pose",
       }) {
    ParsedWireKey parsed;
    std::string error;
    EXPECT_FALSE(ParseWireKey(key, &parsed, &error)) << key;
    EXPECT_FALSE(error.empty());
  }
}

TEST(MocapRotorWireContract, ListenEndpointCannotInjectAnotherZenohSetting) {
  std::string error;
  EXPECT_TRUE(ValidateZenohListenEndpoint("tcp/0.0.0.0:7457", &error));
  EXPECT_TRUE(ValidateZenohListenEndpoint("tcp/ground-core:7457", &error));
  for (const std::string &endpoint : {
           "udp/0.0.0.0:7457",
           "tcp/0.0.0.0:0",
           "tcp/0.0.0.0:65536",
           "tcp/0.0.0.0:7457','tcp/evil:7447",
           "tcp/127.0.0.1:7457;mode=client",
           "tcp/[::]:7457",
       }) {
    error.clear();
    EXPECT_FALSE(ValidateZenohListenEndpoint(endpoint, &error)) << endpoint;
    EXPECT_FALSE(error.empty());
  }
}

} // namespace
} // namespace xgc_mocap_rotor_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
