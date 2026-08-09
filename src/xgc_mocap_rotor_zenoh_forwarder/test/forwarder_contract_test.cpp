#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "xgc_mocap_rotor_zenoh_forwarder/forwarder_contract.hpp"

namespace xgc_mocap_rotor_zenoh_forwarder {
namespace {

ForwarderConfig validConfig() {
  return ForwarderConfig{
      "mocap-rotor-01", "tcp/192.0.2.10:7457",
      "/mavros/local_position/pose",
      "/mavros/local_position/velocity_local", "/mavros/imu/data",
      "/mavros/battery", "/mavros/state", "/mavros/extended_state",
      "base_link"};
}

TEST(MocapRotorForwarderContract, AllowsOnlySixFixedReadOnlyUplinkKeys) {
  const UplinkChannel channels[] = {
      UplinkChannel::kLocalPose, UplinkChannel::kLocalVelocity,
      UplinkChannel::kImu, UplinkChannel::kPower,
      UplinkChannel::kFlightState, UplinkChannel::kForwarderHeartbeat};
  const char *leaves[] = {"local_pose", "local_velocity", "imu",
                          "power",      "flight_state",   "forwarder_hb"};
  for (std::size_t index = 0u; index < 6u; ++index) {
    EXPECT_EQ(UplinkLeaf(channels[index]), leaves[index]);
    EXPECT_EQ(UplinkKey("mocap-rotor-01", channels[index]),
              std::string("xgc2/mocap-rotor-01/up/") + leaves[index]);
    EXPECT_GT(MaximumPayloadBytes(channels[index]), 0u);
    EXPECT_GT(MaximumRateHz(channels[index]), 0.0);
    EXPECT_LE(MaximumRateHz(channels[index]), 15.0);
  }
}

TEST(MocapRotorForwarderContract, RequiresExplicitUniqueOnboardMappings) {
  std::string error;
  auto config = validConfig();
  EXPECT_TRUE(ValidateForwarderConfig(config, &error)) << error;
  config.local_pose_topic.clear();
  EXPECT_FALSE(ValidateForwarderConfig(config, &error));
  config = validConfig();
  config.extended_state_topic = config.flight_state_topic;
  EXPECT_FALSE(ValidateForwarderConfig(config, &error));
  config = validConfig();
  config.local_pose_topic = "mavros/local_position/pose";
  EXPECT_FALSE(ValidateForwarderConfig(config, &error));
}

TEST(MocapRotorForwarderContract, RejectsDiscoveryAndInjectedEndpoints) {
  std::string error;
  EXPECT_TRUE(
      ValidateZenohConnectEndpoint("tcp/ground.example:7457", &error));
  for (const auto &endpoint : {"udp/ground:7457", "tcp/ground:0",
                               "tcp/ground:65536", "tcp/ground:7457,foo",
                               "tcp/[::1]:7457"}) {
    EXPECT_FALSE(ValidateZenohConnectEndpoint(endpoint, &error)) << endpoint;
  }
}

} // namespace
} // namespace xgc_mocap_rotor_zenoh_forwarder

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
